#pragma once

#include "sps/core/types.h"
#include "sps/core/vertex.h"
#include "sps/core/face.h"
#include "sps/core/half_edge.h"
#include "sps/core/region.h"

#include <vector>
#include <unordered_map>
#include <functional>

namespace sps {

/**
 * Triangle mesh with half-edge data structure
 * Supports shape-preserving simplification for urban buildings
 */
class Mesh {
public:
    // Data
    std::vector<Vertex> vertices;
    std::vector<Face> faces;
    std::vector<HalfEdge> halfEdges;
    std::vector<Edge> edges;
    std::vector<PlanarRegion> regions;

    // Adjacency lists for fast lookup
    std::vector<std::vector<Index>> vertexFaces;   // vertex -> adjacent faces
    std::vector<std::vector<Index>> vertexEdges;   // vertex -> incident edges

    // Cached counts for O(1) lookup during simplification
    mutable size_t cachedFaceCount_ = 0;
    mutable bool faceCountDirty_ = true;

    // Separate texture coordinates and normals (for OBJ format)
    std::vector<Vector2d> texCoords;
    std::vector<Vector3d> normals;

    // Properties
    BoundingBox boundingBox;
    std::string name;

public:
    Mesh() = default;

    // ==================== Basic Operations ====================

    // Add geometry
    Index addVertex(const Vector3d& position);
    Index addFace(Index v0, Index v1, Index v2);

    // Count (excluding removed)
    size_t vertexCount() const;
    size_t faceCount() const;
    size_t edgeCount() const;

    // Invalidate cached counts (call after modifying faces)
    void invalidateFaceCount() { faceCountDirty_ = true; }

    // Decrement face count directly (for fast simplification)
    void decrementFaceCount(size_t n = 1) {
        if (!faceCountDirty_) cachedFaceCount_ -= n;
    }

    // ==================== Topology Building ====================

    // Build half-edge structure from faces
    void buildHalfEdgeStructure();

    // Build edge list from half-edges
    void buildEdgeList();

    // Build adjacency lists (call after buildEdgeList)
    void buildAdjacency();

    // ==================== Geometry Computation ====================

    // Compute all face normals and areas
    void computeFaceNormals();

    // Compute vertex normals (area-weighted average of adjacent faces)
    void computeVertexNormals();

    // Compute bounding box
    void computeBoundingBox();

    // Compute face centroid
    static Vector3d computeCentroid(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2);

    // Compute face normal (unnormalized, magnitude = 2*area)
    static Vector3d computeFaceNormal(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2);

    // Compute face area
    static double computeFaceArea(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2);

    // ==================== Adjacency Queries ====================

    // Get faces adjacent to a vertex
    std::vector<Index> getVertexFaces(Index vertexIdx) const;

    // Get vertices adjacent to a vertex (1-ring neighborhood)
    std::vector<Index> getVertexNeighbors(Index vertexIdx) const;

    // Get faces adjacent to a face (sharing an edge)
    std::vector<Index> getFaceNeighbors(Index faceIdx) const;

    // Get edges incident to a vertex
    std::vector<Index> getVertexEdges(Index vertexIdx) const;

    // ==================== Validation ====================

    // Check mesh integrity
    bool isValid() const;

    // Check if mesh is manifold
    bool isManifold() const;

    // Check for degenerate faces
    bool hasDegenrateFaces() const;

    // ==================== Utility ====================

    // Compact mesh (remove deleted elements)
    void compact();

    // Clear all data
    void clear();

    // Get mesh statistics
    void printStats() const;

private:
    // Edge hash for building half-edge structure
    struct EdgeHash {
        size_t operator()(const std::pair<Index, Index>& e) const {
            return std::hash<Index>()(e.first) ^ (std::hash<Index>()(e.second) << 16);
        }
    };

    using EdgeMap = std::unordered_map<std::pair<Index, Index>, Index, EdgeHash>;
};

} // namespace sps
