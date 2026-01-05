#pragma once

#include "sps/core/mesh.h"

namespace sps {

/**
 * Bilateral Mesh Filter - Makes planes flatter while preserving sharp edges
 *
 * TODO: This module needs to be updated to work with CGAL Surface_mesh.
 * Currently provides placeholder implementation.
 */
class BilateralMeshFilter {
public:
    struct Params {
        double sigmaSpace = 1.0;
        double angleThreshold = 25.0;
        int maxIterations = 10;
        double convergenceThreshold = 1e-6;
        bool verbose = false;
    };

    /**
     * Apply bilateral filtering to smooth the mesh while preserving edges
     * @param mesh The mesh to filter
     * @param params Filter parameters
     */
    void filter(Mesh& mesh, const Params& params = Params()) {
        // TODO: Implement bilateral filtering for CGAL Surface_mesh
        // For now, just compute face normals
        if (params.verbose) {
            std::cout << "BilateralMeshFilter: Computing face normals...\n";
        }
        mesh.computeFaceNormals();
        if (params.verbose) {
            std::cout << "BilateralMeshFilter: Done.\n";
        }
    }
};

} // namespace sps
