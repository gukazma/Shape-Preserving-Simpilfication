#pragma once

#include "sps/core/types.h"
#include <set>

namespace sps {

/**
 * Planar region detected by shape detection algorithm
 */
struct PlanarRegion {
    Index id = INVALID_INDEX;

    // Region properties
    Vector3d avgNormal{0.0, 0.0, 0.0};  // Average normal of faces in this region
    Vector3d centroid{0.0, 0.0, 0.0};   // Centroid of the region
    double totalArea = 0.0;              // Total area of faces

    // Membership
    std::vector<Index> faces;            // Face indices in this region
    std::set<Index> vertices;            // Vertex indices in this region
    std::set<Index> boundaryVertices;    // Vertices on the boundary
    std::set<Index> neighborRegions;     // Adjacent region IDs

    // State
    bool removed = false;

    // Get face count
    size_t faceCount() const { return faces.size(); }

    // Check if this is a small region
    bool isSmall(size_t threshold = 10) const {
        return faces.size() < threshold;
    }
};

} // namespace sps
