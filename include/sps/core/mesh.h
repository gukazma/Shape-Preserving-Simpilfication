#pragma once

#include "sps/core/types.h"
#include <iostream>
#include <set>
#include <map>

namespace sps {

/**
 * Mesh class wrapping CGAL::Surface_mesh with additional functionality
 * for shape-preserving simplification
 */
class Mesh {
public:
    // The underlying CGAL mesh
    CGALMesh cgal_mesh;

    // Planar regions for shape-preserving simplification
    std::vector<PlanarRegion> regions;

    // Properties
    BoundingBox boundingBox;
    std::string name;
    std::string mtlFile;  // Material file path (for OBJ textures)

    // Property maps for vertex attributes (stored as mesh properties)
    using VertexTypeMap = CGALMesh::Property_map<vertex_descriptor, VertexType>;
    using VertexRegionMap = CGALMesh::Property_map<vertex_descriptor, Index>;
    using VertexRegionCountMap = CGALMesh::Property_map<vertex_descriptor, int>;
    using FaceRegionMap = CGALMesh::Property_map<face_descriptor, Index>;
    using FaceNormalMap = CGALMesh::Property_map<face_descriptor, Vector_3>;
    using EdgeConstraintMap = CGALMesh::Property_map<edge_descriptor, bool>;
    using HalfedgeTexcoordMap = CGALMesh::Property_map<halfedge_descriptor, Point_2>;
    using FaceMaterialMap = CGALMesh::Property_map<face_descriptor, int>;  // Material index per face

public:
    Mesh() = default;

    // Initialize property maps (call after loading mesh)
    void initPropertyMaps() {
        // Vertex properties
        auto [vtype, vtype_created] = cgal_mesh.add_property_map<vertex_descriptor, VertexType>(
            "v:type", VertexType::PLANAR);
        vertexTypeMap_ = vtype;

        auto [vregion, vregion_created] = cgal_mesh.add_property_map<vertex_descriptor, Index>(
            "v:region", INVALID_INDEX);
        vertexRegionMap_ = vregion;

        auto [vcount, vcount_created] = cgal_mesh.add_property_map<vertex_descriptor, int>(
            "v:region_count", 0);
        vertexRegionCountMap_ = vcount;

        // Face properties
        auto [fregion, fregion_created] = cgal_mesh.add_property_map<face_descriptor, Index>(
            "f:region", INVALID_INDEX);
        faceRegionMap_ = fregion;

        auto [fnormal, fnormal_created] = cgal_mesh.add_property_map<face_descriptor, Vector_3>(
            "f:normal", Vector_3(0, 0, 1));
        faceNormalMap_ = fnormal;

        // Edge properties
        auto [econstraint, econstraint_created] = cgal_mesh.add_property_map<edge_descriptor, bool>(
            "e:constrained", false);
        edgeConstraintMap_ = econstraint;
    }

    // Initialize texture coordinate property map
    void initTexcoordMap() {
        auto [htexcoord, htexcoord_created] = cgal_mesh.add_property_map<halfedge_descriptor, Point_2>(
            "h:texcoord", Point_2(0, 0));
        halfedgeTexcoordMap_ = htexcoord;
        hasTexcoords_ = true;
    }

    // ==================== Basic Operations ====================

    // Add a vertex
    vertex_descriptor addVertex(const Point_3& point) {
        return cgal_mesh.add_vertex(point);
    }

    vertex_descriptor addVertex(const Vector3d& position) {
        return cgal_mesh.add_vertex(to_cgal_point(position));
    }

    // Add a face
    face_descriptor addFace(vertex_descriptor v0, vertex_descriptor v1, vertex_descriptor v2) {
        return cgal_mesh.add_face(v0, v1, v2);
    }

    // Count elements
    size_t vertexCount() const { return cgal_mesh.number_of_vertices(); }
    size_t faceCount() const { return cgal_mesh.number_of_faces(); }
    size_t edgeCount() const { return cgal_mesh.number_of_edges(); }
    size_t halfedgeCount() const { return cgal_mesh.number_of_halfedges(); }

    // ==================== Property Access ====================

    // Vertex type (PLANAR, EDGE, CORNER)
    VertexType& vertexType(vertex_descriptor v) { return vertexTypeMap_[v]; }
    VertexType vertexType(vertex_descriptor v) const { return vertexTypeMap_[v]; }

    // Vertex region
    Index& vertexRegion(vertex_descriptor v) { return vertexRegionMap_[v]; }
    Index vertexRegion(vertex_descriptor v) const { return vertexRegionMap_[v]; }

    // Vertex region count
    int& vertexRegionCount(vertex_descriptor v) { return vertexRegionCountMap_[v]; }
    int vertexRegionCount(vertex_descriptor v) const { return vertexRegionCountMap_[v]; }

    // Face region
    Index& faceRegion(face_descriptor f) { return faceRegionMap_[f]; }
    Index faceRegion(face_descriptor f) const { return faceRegionMap_[f]; }

    // Face normal
    Vector_3& faceNormal(face_descriptor f) { return faceNormalMap_[f]; }
    Vector_3 faceNormal(face_descriptor f) const { return faceNormalMap_[f]; }

    // Edge constraint
    void setEdgeConstrained(edge_descriptor e, bool constrained) { edgeConstraintMap_[e] = constrained; }
    bool edgeConstrained(edge_descriptor e) const { return edgeConstraintMap_[e]; }

    // Texture coordinates (per halfedge)
    bool hasTexcoords() const { return hasTexcoords_; }
    Point_2& texcoord(halfedge_descriptor h) { return halfedgeTexcoordMap_[h]; }
    Point_2 texcoord(halfedge_descriptor h) const { return halfedgeTexcoordMap_[h]; }
    void setTexcoord(halfedge_descriptor h, const Point_2& uv) { halfedgeTexcoordMap_[h] = uv; }
    HalfedgeTexcoordMap& getTexcoordMap() { return halfedgeTexcoordMap_; }

    // Material index (per face) for multi-texture support
    bool hasMaterials() const { return hasMaterials_; }
    int faceMaterial(face_descriptor f) const { return hasMaterials_ ? faceMaterialMap_[f] : 0; }
    void setFaceMaterial(face_descriptor f, int matIdx) { faceMaterialMap_[f] = matIdx; }
    FaceMaterialMap& getFaceMaterialMap() { return faceMaterialMap_; }
    const std::vector<std::string>& getMaterialNames() const { return materialNames_; }
    void setMaterialNames(const std::vector<std::string>& names) { materialNames_ = names; }
    void initMaterialMap() {
        auto [fmat, fmat_created] = cgal_mesh.add_property_map<face_descriptor, int>("f:material", 0);
        faceMaterialMap_ = fmat;
        hasMaterials_ = true;
    }
    void clearMaterials() {
        materialNames_.clear();
        hasMaterials_ = false;
    }

    // Get vertex weight based on region count
    double vertexWeight(vertex_descriptor v) const {
        return getAdaptiveWeight(vertexRegionCountMap_[v]);
    }

    // Get point position
    Point_3& point(vertex_descriptor v) { return cgal_mesh.point(v); }
    const Point_3& point(vertex_descriptor v) const { return cgal_mesh.point(v); }

    // ==================== Geometry Computation ====================

    // Compute all face normals
    void computeFaceNormals() {
        for (face_descriptor f : cgal_mesh.faces()) {
            halfedge_descriptor h = cgal_mesh.halfedge(f);
            Point_3 p0 = cgal_mesh.point(cgal_mesh.target(h));
            Point_3 p1 = cgal_mesh.point(cgal_mesh.target(cgal_mesh.next(h)));
            Point_3 p2 = cgal_mesh.point(cgal_mesh.target(cgal_mesh.next(cgal_mesh.next(h))));

            Vector_3 v1 = p1 - p0;
            Vector_3 v2 = p2 - p0;
            Vector_3 normal = CGAL::cross_product(v1, v2);

            double len = std::sqrt(CGAL::to_double(normal.squared_length()));
            if (len > 1e-10) {
                normal = normal / len;
            }
            faceNormalMap_[f] = normal;
        }
    }

    // Compute bounding box
    void computeBoundingBox() {
        boundingBox = BoundingBox();
        for (vertex_descriptor v : cgal_mesh.vertices()) {
            boundingBox.expand(cgal_mesh.point(v));
        }
    }

    // Compute face area
    static double computeFaceArea(const Point_3& p0, const Point_3& p1, const Point_3& p2) {
        Vector_3 v1 = p1 - p0;
        Vector_3 v2 = p2 - p0;
        Vector_3 cross = CGAL::cross_product(v1, v2);
        return 0.5 * std::sqrt(CGAL::to_double(cross.squared_length()));
    }

    // ==================== Validation ====================

    bool isValid() const {
        return cgal_mesh.is_valid();
    }

    bool isTriangleMesh() const {
        return CGAL::is_triangle_mesh(cgal_mesh);
    }

    // ==================== Utility ====================

    void clear() {
        cgal_mesh.clear();
        regions.clear();
        boundingBox = BoundingBox();
        name.clear();
        mtlFile.clear();
        materialNames_.clear();
        hasTexcoords_ = false;
        hasMaterials_ = false;
    }

    void printStats() const {
        std::cout << "Mesh Statistics:\n"
                  << "  Name: " << (name.empty() ? "(unnamed)" : name) << "\n"
                  << "  Vertices: " << vertexCount() << "\n"
                  << "  Faces: " << faceCount() << "\n"
                  << "  Edges: " << edgeCount() << "\n"
                  << "  Halfedges: " << halfedgeCount() << "\n"
                  << "  Regions: " << regions.size() << "\n"
                  << "  Valid: " << (isValid() ? "yes" : "no") << "\n"
                  << "  Triangle mesh: " << (isTriangleMesh() ? "yes" : "no") << "\n";

        if (boundingBox.isValid()) {
            std::cout << "  Bounding box: ["
                      << boundingBox.min.transpose() << "] - ["
                      << boundingBox.max.transpose() << "]\n"
                      << "  Diagonal: " << boundingBox.diagonal() << "\n";
        }
    }

    // ==================== Edge Constraint Map Access ====================

    EdgeConstraintMap& getEdgeConstraintMap() { return edgeConstraintMap_; }
    const EdgeConstraintMap& getEdgeConstraintMap() const { return edgeConstraintMap_; }

private:
    // Property maps
    VertexTypeMap vertexTypeMap_;
    VertexRegionMap vertexRegionMap_;
    VertexRegionCountMap vertexRegionCountMap_;
    FaceRegionMap faceRegionMap_;
    FaceNormalMap faceNormalMap_;
    EdgeConstraintMap edgeConstraintMap_;
    HalfedgeTexcoordMap halfedgeTexcoordMap_;
    FaceMaterialMap faceMaterialMap_;
    std::vector<std::string> materialNames_;
    bool hasTexcoords_ = false;
    bool hasMaterials_ = false;
};

} // namespace sps
