#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <array>
#include <cstdint>
#include <limits>

namespace sps {

// Type aliases
using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;

// Index types
using Index = int32_t;
using Size = size_t;

constexpr Index INVALID_INDEX = -1;

// Vertex type classification for constrained simplification
enum class VertexType : uint8_t {
    PLANAR = 0,   // V1: belongs to one planar region (weight: 1)
    EDGE = 1,     // V2: belongs to two regions (weight: 10)
    CORNER = 2    // V3: belongs to three or more regions (weight: 100)
};

// Get weight for vertex type
inline double getVertexWeight(VertexType type) {
    switch (type) {
        case VertexType::PLANAR: return 1.0;
        case VertexType::EDGE:   return 10.0;
        case VertexType::CORNER: return 100.0;
        default: return 1.0;
    }
}

// Bounding box
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

    Vector3d center() const { return (min + max) * 0.5; }
    Vector3d size() const { return max - min; }
    double diagonal() const { return size().norm(); }
};

} // namespace sps
