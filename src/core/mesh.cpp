#include "sps/core/mesh.h"
#include <iostream>
#include <algorithm>
#include <map>
#include <unordered_set>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace sps {

Index Mesh::addVertex(const Vector3d& position) {
    Index idx = static_cast<Index>(vertices.size());
    Vertex v;
    v.position = position;
    vertices.push_back(v);
    boundingBox.expand(position);
    return idx;
}

Index Mesh::addFace(Index v0, Index v1, Index v2) {
    Index idx = static_cast<Index>(faces.size());
    Face f;
    f.vertices = {v0, v1, v2};

    const Vector3d& p0 = vertices[v0].position;
    const Vector3d& p1 = vertices[v1].position;
    const Vector3d& p2 = vertices[v2].position;

    f.normal = computeFaceNormal(p0, p1, p2);
    f.area = f.normal.norm() * 0.5;
    if (f.area > 1e-10) f.normal.normalize();
    f.centroid = computeCentroid(p0, p1, p2);

    faces.push_back(f);
    return idx;
}

size_t Mesh::vertexCount() const {
    return std::count_if(vertices.begin(), vertices.end(),
                         [](const Vertex& v) { return !v.removed; });
}

size_t Mesh::faceCount() const {
    if (faceCountDirty_) {
        cachedFaceCount_ = std::count_if(faces.begin(), faces.end(),
                             [](const Face& f) { return !f.removed; });
        faceCountDirty_ = false;
    }
    return cachedFaceCount_;
}

size_t Mesh::edgeCount() const {
    return std::count_if(edges.begin(), edges.end(),
                         [](const Edge& e) { return !e.removed; });
}

void Mesh::computeFaceNormals() {
    for (Face& face : faces) {
        if (face.removed) continue;
        const Vector3d& p0 = vertices[face.vertices[0]].position;
        const Vector3d& p1 = vertices[face.vertices[1]].position;
        const Vector3d& p2 = vertices[face.vertices[2]].position;
        Vector3d normal = computeFaceNormal(p0, p1, p2);
        face.area = normal.norm() * 0.5;
        face.normal = (face.area > 1e-10) ? normal.normalized() : Vector3d(0, 0, 1);
        face.centroid = computeCentroid(p0, p1, p2);
    }
}

void Mesh::computeVertexNormals() {
    for (Vertex& v : vertices) v.normal = Vector3d::Zero();
    for (const Face& face : faces) {
        if (face.removed) continue;
        Vector3d wn = face.normal * face.area;
        for (int i = 0; i < 3; ++i)
            vertices[face.vertices[i]].normal += wn;
    }
    for (Vertex& v : vertices) {
        if (!v.removed && v.normal.squaredNorm() > 1e-10)
            v.normal.normalize();
    }
}

void Mesh::computeBoundingBox() {
    boundingBox = BoundingBox();
    for (const Vertex& v : vertices)
        if (!v.removed) boundingBox.expand(v.position);
}

Vector3d Mesh::computeCentroid(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2) {
    return (v0 + v1 + v2) / 3.0;
}

Vector3d Mesh::computeFaceNormal(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2) {
    return (v1 - v0).cross(v2 - v0);
}

double Mesh::computeFaceArea(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2) {
    return computeFaceNormal(v0, v1, v2).norm() * 0.5;
}

void Mesh::buildHalfEdgeStructure() {
    halfEdges.clear();
    // Simple implementation - build half-edges from faces
    for (Index fi = 0; fi < static_cast<Index>(faces.size()); ++fi) {
        if (faces[fi].removed) continue;
        const Face& f = faces[fi];
        for (int i = 0; i < 3; ++i) {
            HalfEdge he;
            he.vertex = f.vertices[i];
            he.face = fi;
            halfEdges.push_back(he);
        }
    }
}

void Mesh::buildEdgeList() {
    edges.clear();

    // Use map to track unique edges (smaller vertex index first)
    std::map<std::pair<Index, Index>, Index> edgeMap;

    for (Index fi = 0; fi < static_cast<Index>(faces.size()); ++fi) {
        if (faces[fi].removed) continue;
        const Face& f = faces[fi];

        // Three edges per triangle: (v0,v1), (v1,v2), (v2,v0)
        for (int i = 0; i < 3; ++i) {
            Index v1 = f.vertices[i];
            Index v2 = f.vertices[(i + 1) % 3];

            // Ensure smaller index first for consistent key
            if (v1 > v2) std::swap(v1, v2);
            auto key = std::make_pair(v1, v2);

            if (edgeMap.find(key) == edgeMap.end()) {
                Index edgeIdx = static_cast<Index>(edges.size());
                Edge e;
                e.v1 = v1;
                e.v2 = v2;
                e.removed = false;
                edges.push_back(e);
                edgeMap[key] = edgeIdx;
            }
        }
    }
}

void Mesh::buildAdjacency() {
    // Build vertex -> faces adjacency
    vertexFaces.clear();
    vertexFaces.resize(vertices.size());
    for (Index fi = 0; fi < static_cast<Index>(faces.size()); ++fi) {
        if (faces[fi].removed) continue;
        const Face& f = faces[fi];
        for (int i = 0; i < 3; ++i) {
            vertexFaces[f.vertices[i]].push_back(fi);
        }
    }

    // Build vertex -> edges adjacency
    vertexEdges.clear();
    vertexEdges.resize(vertices.size());
    for (Index ei = 0; ei < static_cast<Index>(edges.size()); ++ei) {
        if (edges[ei].removed) continue;
        const Edge& e = edges[ei];
        vertexEdges[e.v1].push_back(ei);
        vertexEdges[e.v2].push_back(ei);
    }
}

std::vector<Index> Mesh::getVertexFaces(Index vi) const {
    if (vi < static_cast<Index>(vertexFaces.size()))
        return vertexFaces[vi];
    return {};
}
std::vector<Index> Mesh::getVertexEdges(Index vi) const {
    if (vi < static_cast<Index>(vertexEdges.size()))
        return vertexEdges[vi];
    return {};
}
std::vector<Index> Mesh::getVertexNeighbors(Index) const { return {}; }
std::vector<Index> Mesh::getFaceNeighbors(Index faceIdx) const {
    std::vector<Index> result;
    if (faceIdx < 0 || faceIdx >= static_cast<Index>(faces.size())) return result;

    const Face& f = faces[faceIdx];

    // Use adjacency list if available - O(degree) instead of O(n)
    if (!vertexFaces.empty()) {
        std::unordered_set<Index> candidates;

        // Get all faces that share at least one vertex with this face
        for (int i = 0; i < 3; ++i) {
            Index vi = f.vertices[i];
            if (vi < static_cast<Index>(vertexFaces.size())) {
                for (Index fi : vertexFaces[vi]) {
                    if (fi != faceIdx && !faces[fi].removed) {
                        candidates.insert(fi);
                    }
                }
            }
        }

        // Check which candidates share at least 2 vertices (an edge)
        for (Index fi : candidates) {
            int shared = 0;
            const Face& other = faces[fi];
            for (int a = 0; a < 3; ++a) {
                for (int b = 0; b < 3; ++b) {
                    if (f.vertices[a] == other.vertices[b]) {
                        shared++;
                    }
                }
            }
            if (shared >= 2) {
                result.push_back(fi);
            }
        }
    } else {
        // Fallback: O(n) implementation if adjacency not built
        for (Index i = 0; i < static_cast<Index>(faces.size()); ++i) {
            if (i == faceIdx || faces[i].removed) continue;
            int shared = 0;
            for (int a = 0; a < 3; ++a)
                for (int b = 0; b < 3; ++b)
                    if (f.vertices[a] == faces[i].vertices[b]) shared++;
            if (shared >= 2) result.push_back(i);
        }
    }

    return result;
}

bool Mesh::isValid() const { return true; }
bool Mesh::isManifold() const { return true; }
bool Mesh::hasDegenrateFaces() const { return false; }

void Mesh::compact() {}
void Mesh::clear() {
    vertices.clear(); faces.clear(); halfEdges.clear();
    edges.clear(); regions.clear(); texCoords.clear(); normals.clear();
    vertexFaces.clear(); vertexEdges.clear();
}

void Mesh::printStats() const {
    std::cout << "Mesh: V=" << vertexCount() << " F=" << faceCount() << std::endl;
}

bool Vertex::isBoundary() const { return false; }

} // namespace sps
