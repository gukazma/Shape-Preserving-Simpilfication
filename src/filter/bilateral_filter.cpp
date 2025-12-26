#include "sps/filter/bilateral_filter.h"
#include <cmath>
#include <iostream>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace sps {

void BilateralMeshFilter::filter(Mesh& mesh, const Params& params) {
    if (params.verbose)
        std::cout << "Bilateral filtering (iterations=" << params.maxIterations << ")...\n";

    double avgEdgeLen = computeAverageEdgeLength(mesh);
    double sigma = params.sigmaSpace * avgEdgeLen;

    // Build adjacency for fast vertex-face lookup
    if (mesh.vertexFaces.empty()) {
        mesh.buildAdjacency();
    }

    for (int iter = 0; iter < params.maxIterations; ++iter) {
        computeFaceNormals(mesh);
        filterFaceNormals(mesh, params);
        updateVertexPositions(mesh);

        if (params.verbose)
            std::cout << "  Iteration " << (iter + 1) << " done\n";
    }
}

void BilateralMeshFilter::computeFaceNormals(Mesh& mesh) {
    mesh.computeFaceNormals();
}

void BilateralMeshFilter::filterFaceNormals(Mesh& mesh, const Params& params) {
    filteredNormals_.resize(mesh.faces.size());
    double sigma = params.sigmaSpace * computeAverageEdgeLength(mesh);

    #pragma omp parallel for
    for (int i = 0; i < static_cast<int>(mesh.faces.size()); ++i) {
        if (mesh.faces[i].removed) continue;

        auto neighbors = getAdaptiveNeighborhood(mesh, i, params.angleThreshold);
        Vector3d weightedNormal = Vector3d::Zero();
        double totalWeight = 0.0;

        for (Index j : neighbors) {
            const Face& fi = mesh.faces[i];
            const Face& fj = mesh.faces[j];

            double alpha = computeSpatialWeight(fi.centroid, fj.centroid, sigma);
            double beta = computeNormalWeight(fi.normal, fj.normal, params.angleThreshold);
            double w = fj.area * alpha * beta;

            weightedNormal += w * fj.normal;
            totalWeight += w;
        }

        if (totalWeight > 1e-10)
            filteredNormals_[i] = weightedNormal.normalized();
        else
            filteredNormals_[i] = mesh.faces[i].normal;
    }

    // Apply filtered normals
    for (size_t i = 0; i < mesh.faces.size(); ++i) {
        if (!mesh.faces[i].removed)
            mesh.faces[i].normal = filteredNormals_[i];
    }
}

void BilateralMeshFilter::updateVertexPositions(Mesh& mesh) {
    // For each vertex, move it to better align with filtered face normals
    // Use adjacency lists for O(1) face lookup per vertex

    #pragma omp parallel for
    for (int vi = 0; vi < static_cast<int>(mesh.vertices.size()); ++vi) {
        if (mesh.vertices[vi].removed) continue;

        Vector3d delta = Vector3d::Zero();
        int count = 0;

        // Use adjacency list for fast lookup
        if (vi < static_cast<int>(mesh.vertexFaces.size())) {
            for (Index fi : mesh.vertexFaces[vi]) {
                if (mesh.faces[fi].removed) continue;
                const Face& f = mesh.faces[fi];

                Vector3d diff = f.centroid - mesh.vertices[vi].position;
                double proj = diff.dot(f.normal);
                delta += proj * f.normal;
                count++;
            }
        }

        if (count > 0) {
            mesh.vertices[vi].position += delta / count;
        }
    }
}

double BilateralMeshFilter::computeSpatialWeight(const Vector3d& ci, const Vector3d& cj, double sigma) const {
    double dist2 = (ci - cj).squaredNorm();
    return std::exp(-dist2 / (2.0 * sigma * sigma));
}

double BilateralMeshFilter::computeNormalWeight(const Vector3d& ni, const Vector3d& nj, double angleThreshold) const {
    double cosTheta = std::cos(angleThreshold * M_PI / 180.0);
    double dot = ni.dot(nj);
    double diff = 1.0 - dot;
    double denom = 1.0 - cosTheta;
    if (denom < 1e-10) return 1.0;
    return std::exp(-(diff * diff) / (denom * denom));
}

std::vector<Index> BilateralMeshFilter::getAdaptiveNeighborhood(const Mesh& mesh, Index faceIdx, double angleThreshold) const {
    std::vector<Index> result;
    result.push_back(faceIdx);  // Include self

    double cosTheta = std::cos(angleThreshold * M_PI / 180.0);
    const Vector3d& ni = mesh.faces[faceIdx].normal;

    // Get face neighbors
    auto neighbors = mesh.getFaceNeighbors(faceIdx);
    for (Index nIdx : neighbors) {
        const Vector3d& nj = mesh.faces[nIdx].normal;
        if (ni.dot(nj) >= cosTheta) {
            result.push_back(nIdx);
        }
    }

    return result;
}

double BilateralMeshFilter::computeAverageEdgeLength(const Mesh& mesh) const {
    double totalLen = 0.0;
    int count = 0;

    for (const Face& f : mesh.faces) {
        if (f.removed) continue;
        for (int i = 0; i < 3; ++i) {
            const Vector3d& p0 = mesh.vertices[f.vertices[i]].position;
            const Vector3d& p1 = mesh.vertices[f.vertices[(i + 1) % 3]].position;
            totalLen += (p1 - p0).norm();
            count++;
        }
    }

    return (count > 0) ? (totalLen / count) : 1.0;
}

} // namespace sps
