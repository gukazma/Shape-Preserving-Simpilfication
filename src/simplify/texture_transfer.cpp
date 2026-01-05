#include "sps/simplify/texture_transfer.h"
#include <iostream>

namespace sps {

void TextureTransfer::initialize(const Mesh& mesh) {
    if (!mesh.hasTexcoords()) {
        std::cout << "TextureTransfer: No texture coordinates to transfer\n";
        initialized_ = false;
        return;
    }

    const CGALMesh& sm = mesh.cgal_mesh;
    size_t numFaces = sm.number_of_faces();

    // Copy the mesh geometry for AABB tree
    originalMesh_ = sm;

    // Store UV coordinates for each face
    faceUVs_.clear();
    for (face_descriptor f : sm.faces()) {
        halfedge_descriptor h = sm.halfedge(f);

        HalfedgeUV uvData;
        for (int i = 0; i < 3; ++i) {
            uvData.uv[i] = mesh.texcoord(h);
            h = sm.next(h);
        }
        faceUVs_[f] = uvData;
    }

    // Build AABB tree from the copied mesh
    tree_.rebuild(faces(originalMesh_).first, faces(originalMesh_).second, originalMesh_);
    tree_.accelerate_distance_queries();

    initialized_ = true;
    std::cout << "TextureTransfer: Initialized with " << numFaces << " faces\n";
}

void TextureTransfer::transfer(Mesh& simplifiedMesh) {
    if (!initialized_) {
        std::cerr << "TextureTransfer: Not initialized\n";
        return;
    }

    CGALMesh& sm = simplifiedMesh.cgal_mesh;

    // Ensure texture coordinate map is initialized
    if (!simplifiedMesh.hasTexcoords()) {
        simplifiedMesh.initTexcoordMap();
    }

    size_t numFaces = sm.number_of_faces();
    size_t processed = 0;

    // For each face in simplified mesh, interpolate UV for each halfedge
    for (face_descriptor f : sm.faces()) {
        halfedge_descriptor h = sm.halfedge(f);

        for (int i = 0; i < 3; ++i) {
            vertex_descriptor v = sm.target(h);
            Point_3 pos = sm.point(v);

            // Interpolate UV from original mesh
            Point_2 uv = interpolateUV(pos);
            simplifiedMesh.setTexcoord(h, uv);

            h = sm.next(h);
        }

        processed++;
        if (processed % 10000 == 0) {
            std::cout << "\r  UV Transfer: " << processed << "/" << numFaces << " faces" << std::flush;
        }
    }

    std::cout << "\r  UV Transfer: " << numFaces << "/" << numFaces << " faces - Done\n";
}

void TextureTransfer::getBarycentricCoordinates(
    const Point_3& a, const Point_3& b, const Point_3& c,
    const Point_3& p,
    double& u, double& v, double& w)
{
    Vector_3 v0 = b - a;
    Vector_3 v1 = c - a;
    Vector_3 v2 = p - a;

    double d00 = CGAL::to_double(v0 * v0);
    double d01 = CGAL::to_double(v0 * v1);
    double d11 = CGAL::to_double(v1 * v1);
    double d20 = CGAL::to_double(v2 * v0);
    double d21 = CGAL::to_double(v2 * v1);

    double denom = d00 * d11 - d01 * d01;
    if (std::abs(denom) < 1e-12) {
        // Degenerate triangle, use equal weights
        u = v = w = 1.0 / 3.0;
        return;
    }

    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0 - v - w;

    // Clamp to valid range
    u = std::max(0.0, std::min(1.0, u));
    v = std::max(0.0, std::min(1.0, v));
    w = std::max(0.0, std::min(1.0, w));

    // Normalize
    double sum = u + v + w;
    if (sum > 0) {
        u /= sum;
        v /= sum;
        w /= sum;
    }
}

Point_2 TextureTransfer::interpolateUV(const Point_3& point) const {
    // Find closest point on original mesh
    Point_and_primitive_id closest = tree_.closest_point_and_primitive(point);

    // Get the face descriptor from the primitive
    face_descriptor originalFace = closest.second;

    // Look up UV data for this face
    auto it = faceUVs_.find(originalFace);
    if (it == faceUVs_.end()) {
        return Point_2(0, 0);
    }

    const HalfedgeUV& uvData = it->second;

    // Get triangle vertices from original mesh
    halfedge_descriptor h = originalMesh_.halfedge(originalFace);
    Point_3 p0 = originalMesh_.point(originalMesh_.target(h));
    h = originalMesh_.next(h);
    Point_3 p1 = originalMesh_.point(originalMesh_.target(h));
    h = originalMesh_.next(h);
    Point_3 p2 = originalMesh_.point(originalMesh_.target(h));

    // Compute barycentric coordinates using the closest point
    double u, v, w;
    getBarycentricCoordinates(p0, p1, p2, closest.first, u, v, w);

    // Interpolate UV using barycentric coordinates
    double uvX = u * CGAL::to_double(uvData.uv[0].x()) +
                 v * CGAL::to_double(uvData.uv[1].x()) +
                 w * CGAL::to_double(uvData.uv[2].x());

    double uvY = u * CGAL::to_double(uvData.uv[0].y()) +
                 v * CGAL::to_double(uvData.uv[1].y()) +
                 w * CGAL::to_double(uvData.uv[2].y());

    return Point_2(uvX, uvY);
}

} // namespace sps
