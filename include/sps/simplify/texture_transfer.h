#pragma once

#include "sps/core/mesh.h"
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <memory>

namespace sps {

/**
 * TextureTransfer - Transfers UV coordinates from original mesh to simplified mesh
 * using AABB tree for closest point queries and barycentric interpolation
 * Supports multi-material meshes by using separate AABB trees per material
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
        Point_3 pos[3]; // Vertex positions (for consistent barycentric computation)
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

    /**
     * Interpolate UV from closest point query result
     */
    Point_2 interpolateUVFromClosest(const Point_3& point,
        const Point_and_primitive_id& closest) const;

    /**
     * Interpolate UV from a specific original face
     * Projects point onto triangle plane and computes barycentric interpolation
     */
    Point_2 interpolateUVFromFace(
        const Point_3& point,
        const Point_3& op0, const Point_3& op1, const Point_3& op2,
        const HalfedgeUV& uvData) const;

    /**
     * Get triangle vertices and UV data for a face
     */
    void getFaceData(face_descriptor f,
        Point_3& op0, Point_3& op1, Point_3& op2,
        const HalfedgeUV*& uvData) const;

    /**
     * Calculate extrapolation error - how far outside [0,1] barycentric coords are
     */
    double calcExtrapolationError(
        const Point_3& point,
        const Point_3& op0, const Point_3& op1, const Point_3& op2) const;

    /**
     * Find closest point on a specific material's faces
     */
    Point_and_primitive_id findClosestPointForMaterial(const Point_3& point, int materialIdx) const;

    // Copy of original mesh for AABB tree
    CGALMesh originalMesh_;

    // Main AABB tree for fast closest point queries (all faces)
    AABB_tree tree_;

    // Per-material meshes and AABB trees (for fast material-specific queries)
    std::vector<std::unique_ptr<CGALMesh>> materialMeshes_;
    std::vector<std::unique_ptr<AABB_tree>> materialTrees_;
    std::vector<std::map<face_descriptor, face_descriptor>> materialFaceMap_;  // Maps material mesh face -> original face

    // UV coordinates per face (indexed by face_descriptor)
    std::map<face_descriptor, HalfedgeUV> faceUVs_;

    // Material index per face (indexed by face_descriptor)
    std::map<face_descriptor, int> faceMaterials_;

    // Material names
    std::vector<std::string> materialNames_;

    bool initialized_ = false;
    bool hasMultipleMaterials_ = false;
};

} // namespace sps
