#pragma once

#include "sps/core/mesh.h"

namespace sps {

// Bilateral Mesh Filter - Makes planes flatter while preserving sharp edges
class BilateralMeshFilter {
public:
    struct Params {
        double sigmaSpace = 1.0;
        double angleThreshold = 25.0;
        int maxIterations = 10;
        double convergenceThreshold = 1e-6;
        bool verbose = false;
    };

    void filter(Mesh& mesh, const Params& params = Params());

private:
    void computeFaceNormals(Mesh& mesh);
    void filterFaceNormals(Mesh& mesh, const Params& params);
    void updateVertexPositions(Mesh& mesh);
    double computeSpatialWeight(const Vector3d& ci, const Vector3d& cj, double sigma) const;
    double computeNormalWeight(const Vector3d& ni, const Vector3d& nj, double angleThreshold) const;
    std::vector<Index> getAdaptiveNeighborhood(const Mesh& mesh, Index faceIdx, double angleThreshold) const;
    double computeAverageEdgeLength(const Mesh& mesh) const;

    std::vector<Vector3d> filteredNormals_;
};

} // namespace sps
