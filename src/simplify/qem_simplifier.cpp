#include "sps/simplify/qem_simplifier.h"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <set>
#include <unordered_set>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace sps {

// ==================== QuadricCalculator ====================

void QuadricCalculator::initializeQuadrics(Mesh& mesh) {
    // Initialize all quadrics to zero
    #pragma omp parallel for
    for (int i = 0; i < static_cast<int>(mesh.vertices.size()); ++i) {
        mesh.vertices[i].quadric = Matrix4d::Zero();
    }

    // Compute face quadrics and accumulate to vertices
    // Note: Need to be careful with race conditions when updating vertices
    size_t numFaces = mesh.faces.size();

    #pragma omp parallel for
    for (int fi = 0; fi < static_cast<int>(numFaces); ++fi) {
        const Face& face = mesh.faces[fi];
        if (face.removed) continue;
        const Vector3d& p = mesh.vertices[face.vertices[0]].position;
        Matrix4d Q = computeFaceQuadric(face.normal, p);

        // Use atomic updates to avoid race conditions
        for (int i = 0; i < 3; ++i) {
            #pragma omp critical
            {
                mesh.vertices[face.vertices[i]].quadric += Q;
            }
        }
    }
}

Matrix4d QuadricCalculator::computeFaceQuadric(const Vector3d& normal, const Vector3d& point) {
    double a = normal.x(), b = normal.y(), c = normal.z();
    double d = -normal.dot(point);
    Matrix4d Q;
    Q << a*a, a*b, a*c, a*d,
         a*b, b*b, b*c, b*d,
         a*c, b*c, c*c, c*d,
         a*d, b*d, c*d, d*d;
    return Q;
}

Matrix4d QuadricCalculator::mergeQuadrics(const Matrix4d& q1, const Matrix4d& q2) {
    return q1 + q2;
}

double QuadricCalculator::computeError(const Matrix4d& Q, const Vector3d& point) {
    Vector4d v(point.x(), point.y(), point.z(), 1.0);
    return v.transpose() * Q * v;
}

Vector3d QuadricCalculator::computeOptimalPosition(const Matrix4d& Q, const Vector3d& v1, const Vector3d& v2) {
    Matrix4d Qbar = Q;
    Qbar.row(3) << 0, 0, 0, 1;
    double det = Qbar.determinant();
    if (std::abs(det) > 1e-10) {
        Vector4d b(0, 0, 0, 1);
        Vector4d opt = Qbar.inverse() * b;
        return Vector3d(opt.x(), opt.y(), opt.z());
    }
    Vector3d mid = (v1 + v2) * 0.5;
    double e1 = computeError(Q, v1);
    double e2 = computeError(Q, v2);
    double em = computeError(Q, mid);
    if (em <= e1 && em <= e2) return mid;
    return (e1 <= e2) ? v1 : v2;
}

// ==================== ConstraintManager ====================

void ConstraintManager::classifyVertices(Mesh& mesh, const std::vector<PlanarRegion>& regions) {
    for (Vertex& v : mesh.vertices) {
        v.type = VertexType::PLANAR;
        v.region = INVALID_INDEX;
        v.regionCount = 0;
    }

    std::vector<std::set<Index>> vertexRegions(mesh.vertices.size());
    for (const auto& region : regions) {
        if (region.removed) continue;
        for (Index fi : region.faces) {
            const Face& f = mesh.faces[fi];
            for (int i = 0; i < 3; ++i) {
                vertexRegions[f.vertices[i]].insert(region.id);
            }
        }
    }

    for (size_t vi = 0; vi < mesh.vertices.size(); ++vi) {
        int count = static_cast<int>(vertexRegions[vi].size());
        mesh.vertices[vi].regionCount = static_cast<uint8_t>(std::min(count, 255));
        if (count >= 3) mesh.vertices[vi].type = VertexType::CORNER;
        else if (count == 2) mesh.vertices[vi].type = VertexType::EDGE;
        else mesh.vertices[vi].type = VertexType::PLANAR;
        if (!vertexRegions[vi].empty())
            mesh.vertices[vi].region = *vertexRegions[vi].begin();
    }
}

bool ConstraintManager::isCollapseValid(const Mesh& mesh, const Edge& edge, const Vector3d& newPos) const {
    if (wouldCauseFlip(mesh, edge, newPos)) return false;
    return true;
}

double ConstraintManager::getEdgeWeight(const Mesh& mesh, const Edge& edge) const {
    return std::max(mesh.vertices[edge.v1].weight(), mesh.vertices[edge.v2].weight());
}

bool ConstraintManager::wouldCauseFlip(const Mesh& mesh, const Edge& edge, const Vector3d& newPos) const {
    // Only check faces adjacent to the edge vertices
    std::unordered_set<Index> facesToCheck;
    if (edge.v1 < mesh.vertexFaces.size()) {
        for (Index fi : mesh.vertexFaces[edge.v1]) {
            if (!mesh.faces[fi].removed) facesToCheck.insert(fi);
        }
    }
    if (edge.v2 < mesh.vertexFaces.size()) {
        for (Index fi : mesh.vertexFaces[edge.v2]) {
            if (!mesh.faces[fi].removed) facesToCheck.insert(fi);
        }
    }

    for (Index fi : facesToCheck) {
        const Face& f = mesh.faces[fi];
        bool hasV1 = false, hasV2 = false;
        for (int i = 0; i < 3; ++i) {
            if (f.vertices[i] == edge.v1) hasV1 = true;
            if (f.vertices[i] == edge.v2) hasV2 = true;
        }
        // Skip faces that will be removed (have both vertices)
        if (hasV1 && hasV2) continue;

        if (hasV1 || hasV2) {
            Vector3d p[3];
            for (int i = 0; i < 3; ++i) {
                if (f.vertices[i] == edge.v1 || f.vertices[i] == edge.v2)
                    p[i] = newPos;
                else
                    p[i] = mesh.vertices[f.vertices[i]].position;
            }
            Vector3d newNormal = (p[1] - p[0]).cross(p[2] - p[0]);
            if (newNormal.squaredNorm() < 1e-10) continue;
            newNormal.normalize();
            if (f.normal.dot(newNormal) < 0.0) return true;
        }
    }
    return false;
}

// ==================== QEMSimplifier ====================

void QEMSimplifier::simplify(Mesh& mesh, const std::vector<PlanarRegion>& regions, const Params& params) {
    useConstraints_ = params.useShapeConstraints;
    initialize(mesh, regions, params);

    int targetCount = (params.targetFaceCount > 0)
        ? params.targetFaceCount
        : static_cast<int>(mesh.faceCount() * params.targetRatio);

    int initialCount = static_cast<int>(mesh.faceCount());
    int totalToRemove = initialCount - targetCount;

    if (params.verbose) {
        std::cout << "Simplifying: " << initialCount << " -> " << targetCount << " faces\n";
        std::cout << "Need to remove: " << totalToRemove << " faces\n";
    }

    int collapsed = 0;
    int skippedVersion = 0, skippedRemoved = 0, skippedFlip = 0, skippedError = 0;

    auto startTime = std::chrono::steady_clock::now();
    auto lastReportTime = startTime;
    int lastReportFaces = initialCount;

    while (static_cast<int>(mesh.faceCount()) > targetCount && !priorityQueue_.empty()) {
        EdgeCost ec = priorityQueue_.top();
        priorityQueue_.pop();

        if (ec.version != edgeVersions_[ec.edgeIdx]) { skippedVersion++; continue; }
        Edge& edge = mesh.edges[ec.edgeIdx];
        if (edge.removed) { skippedRemoved++; continue; }
        if (!constraintMgr_.isCollapseValid(mesh, edge, edge.optimalPos)) { skippedFlip++; continue; }
        if (params.maxError > 0 && ec.cost > params.maxError) { skippedError++; continue; }

        collapseEdge(mesh, ec.edgeIdx);
        collapsed++;

        // Progress report every 1 second
        if (params.verbose) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastReportTime).count();

            if (elapsed >= 1000) {
                int currentFaces = static_cast<int>(mesh.faceCount());
                int removed = initialCount - currentFaces;
                double progress = (totalToRemove > 0) ? (100.0 * removed / totalToRemove) : 100.0;

                // Calculate speed (faces removed per second)
                int facesRemovedSinceLastReport = lastReportFaces - currentFaces;
                double speed = facesRemovedSinceLastReport * 1000.0 / elapsed;

                // Estimate remaining time
                int remaining = currentFaces - targetCount;
                double etaSeconds = (speed > 0) ? (remaining / speed) : 0;

                int etaMin = static_cast<int>(etaSeconds) / 60;
                int etaSec = static_cast<int>(etaSeconds) % 60;

                std::cout << "\r  Progress: " << std::fixed << std::setprecision(1) << progress << "% "
                          << "| Faces: " << currentFaces << "/" << targetCount << " "
                          << "| Speed: " << static_cast<int>(speed) << " f/s "
                          << "| ETA: " << etaMin << "m " << etaSec << "s "
                          << "| Queue: " << priorityQueue_.size()
                          << "        " << std::flush;

                lastReportTime = now;
                lastReportFaces = currentFaces;
            }
        }
    }

    if (params.verbose) {
        auto endTime = std::chrono::steady_clock::now();
        auto totalMs = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();

        std::cout << "\n  Completed in " << (totalMs / 1000.0) << "s\n";
        std::cout << "  Collapsed: " << collapsed << " edges\n";
        std::cout << "  Skipped - version: " << skippedVersion
                  << ", removed: " << skippedRemoved
                  << ", flip: " << skippedFlip
                  << ", error: " << skippedError << "\n";
        std::cout << "  Final: " << mesh.faceCount() << " faces\n";
    }
}

void QEMSimplifier::simplify(Mesh& mesh, const Params& params) {
    std::vector<PlanarRegion> empty;
    Params p = params;
    p.useShapeConstraints = false;
    simplify(mesh, empty, p);
}

void QEMSimplifier::initialize(Mesh& mesh, const std::vector<PlanarRegion>& regions, const Params& params) {
    auto t0 = std::chrono::steady_clock::now();

    if (params.verbose) std::cout << "  [Init] Computing quadrics..." << std::flush;
    QuadricCalculator::initializeQuadrics(mesh);
    auto t1 = std::chrono::steady_clock::now();
    if (params.verbose) std::cout << " done (" << std::chrono::duration<double>(t1-t0).count() << "s)\n";

    if (params.useShapeConstraints && !regions.empty()) {
        if (params.verbose) std::cout << "  [Init] Classifying vertices..." << std::flush;
        constraintMgr_.classifyVertices(mesh, regions);
        auto t2 = std::chrono::steady_clock::now();
        if (params.verbose) std::cout << " done (" << std::chrono::duration<double>(t2-t1).count() << "s)\n";
        t1 = t2;
    }

    if (mesh.edges.empty()) {
        if (params.verbose) std::cout << "  [Init] Building half-edge structure..." << std::flush;
        mesh.buildHalfEdgeStructure();
        auto t2 = std::chrono::steady_clock::now();
        if (params.verbose) std::cout << " done (" << std::chrono::duration<double>(t2-t1).count() << "s)\n";

        if (params.verbose) std::cout << "  [Init] Building edge list..." << std::flush;
        mesh.buildEdgeList();
        auto t3 = std::chrono::steady_clock::now();
        if (params.verbose) std::cout << " done (" << std::chrono::duration<double>(t3-t2).count() << "s)\n";
        t1 = t3;
    }

    // Build adjacency lists for fast lookup
    if (params.verbose) std::cout << "  [Init] Building adjacency..." << std::flush;
    mesh.buildAdjacency();
    auto t4 = std::chrono::steady_clock::now();
    if (params.verbose) std::cout << " done (" << std::chrono::duration<double>(t4-t1).count() << "s)\n";

    edgeVersions_.assign(mesh.edges.size(), 0);
    while (!priorityQueue_.empty()) priorityQueue_.pop();

    // Compute edge costs in parallel
    if (params.verbose) std::cout << "  [Init] Computing edge costs (" << mesh.edges.size() << " edges)..." << std::flush;
    size_t numEdges = mesh.edges.size();
    #pragma omp parallel for
    for (int i = 0; i < static_cast<int>(numEdges); ++i) {
        Edge& edge = mesh.edges[i];
        if (edge.removed) continue;
        edge.cost = computeEdgeCost(mesh, edge);
    }
    auto t5 = std::chrono::steady_clock::now();
    if (params.verbose) std::cout << " done (" << std::chrono::duration<double>(t5-t4).count() << "s)\n";

    // Insert into priority queue (sequential - not thread-safe)
    if (params.verbose) std::cout << "  [Init] Building priority queue..." << std::flush;
    for (Index i = 0; i < static_cast<Index>(numEdges); ++i) {
        Edge& edge = mesh.edges[i];
        if (edge.removed) continue;
        priorityQueue_.push(EdgeCost{i, edge.cost, edgeVersions_[i]});
    }
    auto t6 = std::chrono::steady_clock::now();
    if (params.verbose) std::cout << " done (" << std::chrono::duration<double>(t6-t5).count() << "s)\n";

    auto totalInit = std::chrono::duration<double>(t6-t0).count();
    if (params.verbose) std::cout << "  [Init] Total init time: " << totalInit << "s\n";
}

double QEMSimplifier::computeEdgeCost(const Mesh& mesh, const Edge& edge) const {
    const Vertex& v1 = mesh.vertices[edge.v1];
    const Vertex& v2 = mesh.vertices[edge.v2];
    Matrix4d Q = QuadricCalculator::mergeQuadrics(v1.quadric, v2.quadric);
    Vector3d optPos = QuadricCalculator::computeOptimalPosition(Q, v1.position, v2.position);
    double baseCost = QuadricCalculator::computeError(Q, optPos);
    if (useConstraints_)
        baseCost *= constraintMgr_.getEdgeWeight(mesh, edge);
    const_cast<Edge&>(edge).optimalPos = optPos;
    return baseCost;
}

void QEMSimplifier::collapseEdge(Mesh& mesh, Index edgeIdx) {
    Edge& edge = mesh.edges[edgeIdx];
    Index v1 = edge.v1, v2 = edge.v2;
    Vector3d newPos = edge.optimalPos;

    // Update v1 position and quadric
    mesh.vertices[v1].position = newPos;
    mesh.vertices[v1].quadric = QuadricCalculator::mergeQuadrics(
        mesh.vertices[v1].quadric, mesh.vertices[v2].quadric);
    if (mesh.vertices[v2].type > mesh.vertices[v1].type)
        mesh.vertices[v1].type = mesh.vertices[v2].type;
    mesh.vertices[v2].removed = true;

    // Collect faces to update (only those adjacent to v1 or v2)
    std::unordered_set<Index> affectedFaces;
    if (v1 < mesh.vertexFaces.size()) {
        for (Index fi : mesh.vertexFaces[v1]) affectedFaces.insert(fi);
    }
    if (v2 < mesh.vertexFaces.size()) {
        for (Index fi : mesh.vertexFaces[v2]) affectedFaces.insert(fi);
    }

    // Update faces: replace v2 with v1, remove degenerate faces
    size_t removedFaces = 0;
    for (Index fi : affectedFaces) {
        Face& f = mesh.faces[fi];
        if (f.removed) continue;

        bool hasV1 = false, hasV2 = false;
        for (int i = 0; i < 3; ++i) {
            if (f.vertices[i] == v1) hasV1 = true;
            if (f.vertices[i] == v2) { hasV2 = true; f.vertices[i] = v1; }
        }

        // Remove faces that had both v1 and v2 (they become degenerate)
        if (hasV1 && hasV2) {
            f.removed = true;
            removedFaces++;
            continue;
        }

        // Also check for any degenerate face (duplicate vertices)
        if (f.vertices[0] == f.vertices[1] ||
            f.vertices[1] == f.vertices[2] ||
            f.vertices[2] == f.vertices[0]) {
            f.removed = true;
            removedFaces++;
        }
    }
    mesh.decrementFaceCount(removedFaces);

    // Collect edges to update (only those adjacent to v1 or v2)
    std::unordered_set<Index> affectedEdges;
    if (v1 < mesh.vertexEdges.size()) {
        for (Index ei : mesh.vertexEdges[v1]) affectedEdges.insert(ei);
    }
    if (v2 < mesh.vertexEdges.size()) {
        for (Index ei : mesh.vertexEdges[v2]) affectedEdges.insert(ei);
    }

    // Update edges: replace v2 with v1, remove duplicates
    std::set<std::pair<Index, Index>> existingEdgeKeys;
    for (Index ei : affectedEdges) {
        Edge& e = mesh.edges[ei];
        if (e.removed) continue;

        // Replace v2 with v1
        if (e.v1 == v2) e.v1 = v1;
        if (e.v2 == v2) e.v2 = v1;

        // Remove self-loops
        if (e.v1 == e.v2) {
            e.removed = true;
            continue;
        }

        // Normalize edge direction
        Index minV = std::min(e.v1, e.v2);
        Index maxV = std::max(e.v1, e.v2);
        auto key = std::make_pair(minV, maxV);

        // Check for duplicates among affected edges
        if (existingEdgeKeys.count(key)) {
            e.removed = true;
        } else {
            existingEdgeKeys.insert(key);
        }
    }

    edge.removed = true;

    // Merge v2's adjacency into v1's
    if (v2 < mesh.vertexFaces.size()) {
        for (Index fi : mesh.vertexFaces[v2]) {
            if (!mesh.faces[fi].removed) {
                mesh.vertexFaces[v1].push_back(fi);
            }
        }
        mesh.vertexFaces[v2].clear();
    }
    if (v2 < mesh.vertexEdges.size()) {
        for (Index ei : mesh.vertexEdges[v2]) {
            if (!mesh.edges[ei].removed) {
                mesh.vertexEdges[v1].push_back(ei);
            }
        }
        mesh.vertexEdges[v2].clear();
    }

    // Update affected edges in priority queue
    updateAffectedEdges(mesh, affectedEdges);
}

void QEMSimplifier::updateAffectedEdges(Mesh& mesh, const std::unordered_set<Index>& affectedEdges) {
    for (Index i : affectedEdges) {
        Edge& edge = mesh.edges[i];
        if (edge.removed) continue;
        edge.cost = computeEdgeCost(mesh, edge);
        edgeVersions_[i]++;
        priorityQueue_.push(EdgeCost{i, edge.cost, edgeVersions_[i]});
    }
}

} // namespace sps
