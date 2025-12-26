#pragma once

#include "sps/core/types.h"

namespace sps {

/**
 * Half-edge data structure for efficient mesh traversal
 *
 * Each edge is represented by two half-edges pointing in opposite directions.
 * This allows efficient traversal of faces and vertices.
 */
struct HalfEdge {
    Index vertex = INVALID_INDEX;    // Vertex this half-edge points TO
    Index face = INVALID_INDEX;      // Face this half-edge belongs to
    Index next = INVALID_INDEX;      // Next half-edge in the face (CCW)
    Index prev = INVALID_INDEX;      // Previous half-edge in the face
    Index opposite = INVALID_INDEX;  // Opposite half-edge (twin)

    bool removed = false;

    // Check if this is a boundary half-edge
    bool isBoundary() const { return opposite == INVALID_INDEX; }
};

/**
 * Edge structure for simplification
 * An edge connects two vertices and has an associated collapse cost
 */
struct Edge {
    Index v1 = INVALID_INDEX;
    Index v2 = INVALID_INDEX;
    Index halfEdge = INVALID_INDEX;  // One of the half-edges

    double cost = 0.0;               // Collapse cost
    Vector3d optimalPos{0, 0, 0};    // Optimal position after collapse

    bool removed = false;
    bool heap_valid = true;          // For lazy deletion in priority queue

    // Create ordered pair for comparison
    std::pair<Index, Index> orderedPair() const {
        return (v1 < v2) ? std::make_pair(v1, v2) : std::make_pair(v2, v1);
    }
};

} // namespace sps
