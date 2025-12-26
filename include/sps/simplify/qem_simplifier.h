#pragma once

#include "sps/core/mesh.h"
#include <queue>
#include <set>
#include <unordered_set>

namespace sps {

// QEM Quadric Calculator
class QuadricCalculator {
public:
    static void initializeQuadrics(Mesh& mesh);
    static Matrix4d computeFaceQuadric(const Vector3d& normal, const Vector3d& point);
    static Matrix4d mergeQuadrics(const Matrix4d& q1, const Matrix4d& q2);
    static double computeError(const Matrix4d& Q, const Vector3d& point);
    static Vector3d computeOptimalPosition(const Matrix4d& Q, const Vector3d& v1, const Vector3d& v2);
};

// Constraint Manager - Makes lines straighter by classifying vertices
class ConstraintManager {
public:
    void classifyVertices(Mesh& mesh, const std::vector<PlanarRegion>& regions);
    bool isCollapseValid(const Mesh& mesh, const Edge& edge, const Vector3d& newPos) const;
    double getEdgeWeight(const Mesh& mesh, const Edge& edge) const;

private:
    int countVertexRegions(const Mesh& mesh, Index vertexIdx,
                           const std::vector<PlanarRegion>& regions) const;
    bool wouldCauseFlip(const Mesh& mesh, const Edge& edge, const Vector3d& newPos) const;
    bool wouldPreserveManifold(const Mesh& mesh, const Edge& edge) const;
};

// Shape-Preserving QEM Simplifier
class QEMSimplifier {
public:
    struct Params {
        double targetRatio = 0.1;
        int targetFaceCount = -1;
        bool preserveBoundary = true;
        bool preserveTexture = true;
        bool useShapeConstraints = true;
        double maxError = -1.0;
        bool verbose = false;
    };

    void simplify(Mesh& mesh, const std::vector<PlanarRegion>& regions, const Params& params = Params());
    void simplify(Mesh& mesh, const Params& params = Params());

private:
    struct EdgeCost {
        Index edgeIdx;
        double cost;
        size_t version;
        bool operator>(const EdgeCost& other) const { return cost > other.cost; }
    };
    using PriorityQueue = std::priority_queue<EdgeCost, std::vector<EdgeCost>, std::greater<EdgeCost>>;

    void initialize(Mesh& mesh, const std::vector<PlanarRegion>& regions, const Params& params);
    double computeEdgeCost(const Mesh& mesh, const Edge& edge) const;
    void collapseEdge(Mesh& mesh, Index edgeIdx);
    void updateAffectedEdges(Mesh& mesh, const std::unordered_set<Index>& affectedEdges);

    ConstraintManager constraintMgr_;
    PriorityQueue priorityQueue_;
    std::vector<size_t> edgeVersions_;
    bool useConstraints_ = true;
};

} // namespace sps
