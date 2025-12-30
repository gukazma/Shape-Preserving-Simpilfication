#pragma once

#include "sps/core/types.h"

namespace sps {

/**
 * Vertex structure with QEM support
 */
struct Vertex {
    // Geometry
    Vector3d position{0.0, 0.0, 0.0};
    Vector3d normal{0.0, 0.0, 0.0};
    Vector2d texCoord{0.0, 0.0};

    // QEM quadric matrix (4x4 symmetric matrix)
    Matrix4d quadric = Matrix4d::Zero();

    // Shape preservation
    Index region = INVALID_INDEX;     // Region this vertex belongs to
    VertexType type = VertexType::PLANAR;
    uint8_t regionCount = 0;          // Number of adjacent regions (for adaptive weight)

    // Topology
    Index halfEdge = INVALID_INDEX;   // One outgoing half-edge

    // State
    bool removed = false;

    // Get weight based on adjacent region count (adaptive weight)
    double weight() const {
        return getAdaptiveWeight(regionCount);
    }

    // Check if vertex is on boundary
    bool isBoundary() const;
};

} // namespace sps
