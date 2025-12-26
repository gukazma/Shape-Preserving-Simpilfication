#pragma once

#include "sps/core/types.h"

namespace sps {

/**
 * Triangle face structure
 */
struct Face {
    // Vertex indices (counter-clockwise winding)
    std::array<Index, 3> vertices{INVALID_INDEX, INVALID_INDEX, INVALID_INDEX};

    // Texture coordinate indices (if different from vertex indices)
    std::array<Index, 3> texCoords{INVALID_INDEX, INVALID_INDEX, INVALID_INDEX};

    // Normal indices (if different from vertex indices)
    std::array<Index, 3> normals{INVALID_INDEX, INVALID_INDEX, INVALID_INDEX};

    // Computed properties
    Vector3d normal{0.0, 0.0, 0.0};   // Face normal
    Vector3d centroid{0.0, 0.0, 0.0}; // Face centroid
    double area = 0.0;                 // Face area

    // Shape detection
    Index region = INVALID_INDEX;      // Planar region this face belongs to

    // Topology
    Index halfEdge = INVALID_INDEX;    // One half-edge of this face

    // State
    bool removed = false;

    // Get vertex index at local position (0, 1, or 2)
    Index& operator[](size_t i) { return vertices[i]; }
    const Index& operator[](size_t i) const { return vertices[i]; }
};

} // namespace sps
