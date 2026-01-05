#pragma once

// CGAL headers
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/property_map.h>

// Eigen headers
#include <Eigen/Core>
#include <Eigen/Dense>

// STL headers
#include <vector>
#include <array>
#include <cstdint>
#include <cmath>
#include <limits>
#include <string>
#include <optional>

namespace sps {

// =============================================================================
// CGAL type definitions
// =============================================================================
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3 = Kernel::Point_3;
using Point_2 = Kernel::Point_2;  // For texture coordinates (UV)
using Vector_3 = Kernel::Vector_3;
using FT = Kernel::FT;

// Surface mesh type
using CGALMesh = CGAL::Surface_mesh<Point_3>;

// Mesh element handles
using vertex_descriptor = CGALMesh::Vertex_index;
using halfedge_descriptor = CGALMesh::Halfedge_index;
using edge_descriptor = CGALMesh::Edge_index;
using face_descriptor = CGALMesh::Face_index;

// =============================================================================
// Eigen type aliases (for backward compatibility and general math)
// =============================================================================
using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;

// =============================================================================
// Index types
// =============================================================================
using Index = int32_t;
using Size = size_t;

constexpr Index INVALID_INDEX = -1;

// =============================================================================
// Conversion utilities between CGAL and Eigen types
// =============================================================================
inline Vector3d to_eigen(const Point_3& p) {
    return Vector3d(CGAL::to_double(p.x()),
                    CGAL::to_double(p.y()),
                    CGAL::to_double(p.z()));
}

inline Vector3d to_eigen(const Vector_3& v) {
    return Vector3d(CGAL::to_double(v.x()),
                    CGAL::to_double(v.y()),
                    CGAL::to_double(v.z()));
}

inline Point_3 to_cgal_point(const Vector3d& v) {
    return Point_3(v.x(), v.y(), v.z());
}

inline Vector_3 to_cgal_vector(const Vector3d& v) {
    return Vector_3(v.x(), v.y(), v.z());
}

// =============================================================================
// Vertex type classification for constrained simplification
// =============================================================================
enum class VertexType : uint8_t {
    PLANAR = 0,   // V1: belongs to one planar region (weight: 1)
    EDGE = 1,     // V2: belongs to two regions (weight: 10)
    CORNER = 2    // V3: belongs to three or more regions (weight: 100)
};

// Get weight for vertex type (fixed weights)
inline double getVertexWeight(VertexType type) {
    switch (type) {
        case VertexType::PLANAR: return 1.0;
        case VertexType::EDGE:   return 10.0;
        case VertexType::CORNER: return 100.0;
        default: return 1.0;
    }
}

// Get adaptive weight based on adjacent region count
inline double getAdaptiveWeight(int regionCount) {
    if (regionCount <= 1) return 1.0;
    return 1.0 + std::log2(static_cast<double>(regionCount));
}

// =============================================================================
// Bounding box
// =============================================================================
struct BoundingBox {
    Vector3d min{std::numeric_limits<double>::max(),
                 std::numeric_limits<double>::max(),
                 std::numeric_limits<double>::max()};
    Vector3d max{std::numeric_limits<double>::lowest(),
                 std::numeric_limits<double>::lowest(),
                 std::numeric_limits<double>::lowest()};

    void expand(const Vector3d& point) {
        min = min.cwiseMin(point);
        max = max.cwiseMax(point);
    }

    void expand(const Point_3& point) {
        expand(to_eigen(point));
    }

    Vector3d center() const { return (min + max) * 0.5; }
    Vector3d size() const { return max - min; }
    double diagonal() const { return size().norm(); }

    bool isValid() const {
        return min.x() <= max.x() && min.y() <= max.y() && min.z() <= max.z();
    }
};

// =============================================================================
// Planar region for shape-preserving simplification
// =============================================================================
struct PlanarRegion {
    Index id = INVALID_INDEX;
    Vector3d avgNormal{0.0, 0.0, 0.0};
    double avgDistance = 0.0;  // plane equation: n·x = d
    std::vector<face_descriptor> faces;
    std::set<vertex_descriptor> boundaryVertices;
    std::set<Index> neighborRegions;
    bool removed = false;
};

// =============================================================================
// Simplification parameters
// =============================================================================
struct SimplifyParams {
    double targetRatio = 0.1;           // Target ratio (keep 10% of faces)
    int targetFaceCount = -1;           // Or specify exact target face count
    double maxError = 0.0;              // Maximum error threshold (0 = no limit)
    bool useShapeConstraints = true;    // Use vertex weight constraints
    bool preserveBoundary = true;       // Preserve mesh boundary
    bool verbose = true;                // Print progress info

    // Garland-Heckbert specific options
    bool useGarlandHeckbert = true;     // Use Garland-Heckbert (QEM) policies
    bool useBoundedNormalChange = true; // Filter out collapses that change normals too much
};

} // namespace sps
