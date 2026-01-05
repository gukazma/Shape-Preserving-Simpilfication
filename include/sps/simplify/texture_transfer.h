#pragma once

#include "sps/core/mesh.h"
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>

namespace sps {

/**
 * TextureTransfer - Transfers UV coordinates from original mesh to simplified mesh
 * using AABB tree for closest point queries and barycentric interpolation
 */
class TextureTransfer {
public:
    // CGAL types for AABB tree using face graph primitive
    using Primitive = CGAL::AABB_face_graph_triangle_primitive<CGALMesh>;
    using AABB_traits = CGAL::AABB_traits_3<Kernel, Primitive>;
    using AABB_tree = CGAL::AABB_tree<AABB_traits>;
    using Point_and_primitive_id = AABB_tree::Point_and_primitive_id;

    // Original mesh data for UV lookup
    struct HalfedgeUV {
        Point_2 uv[3];  // UV for each halfedge of the face (h, next(h), next(next(h)))
    };

    TextureTransfer() = default;
    ~TextureTransfer() = default;

    /**
     * Initialize with original mesh data before simplification
     * @param mesh The original mesh with texture coordinates
     */
    void initialize(const Mesh& mesh);

    /**
     * Transfer UV coordinates to simplified mesh
     * @param simplifiedMesh The simplified mesh (modified in place)
     */
    void transfer(Mesh& simplifiedMesh);

    /**
     * Check if texture transfer is initialized
     */
    bool isInitialized() const { return initialized_; }

private:
    /**
     * Compute barycentric coordinates of point p in triangle (a, b, c)
     */
    static void getBarycentricCoordinates(
        const Point_3& a, const Point_3& b, const Point_3& c,
        const Point_3& p,
        double& u, double& v, double& w);

    /**
     * Find closest point on original mesh and interpolate UV
     */
    Point_2 interpolateUV(const Point_3& point) const;

    // Copy of original mesh for AABB tree
    CGALMesh originalMesh_;

    // AABB tree for fast closest point queries
    AABB_tree tree_;

    // UV coordinates per face (indexed by face_descriptor)
    std::map<face_descriptor, HalfedgeUV> faceUVs_;

    bool initialized_ = false;
};

} // namespace sps
