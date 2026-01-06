#include "sps/simplify/texture_transfer.h"
#include <iostream>
#include <cmath>

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

    // Check for multi-material support
    hasMultipleMaterials_ = mesh.hasMaterials();
    if (hasMultipleMaterials_) {
        materialNames_ = mesh.getMaterialNames();
        std::cout << "TextureTransfer: Multi-material mode (" << materialNames_.size() << " materials)\n";
    }

    // Store UV coordinates and material index for each face
    // Use originalMesh_'s face descriptors as keys since AABB tree returns them
    faceUVs_.clear();
    faceMaterials_.clear();

    for (face_descriptor f : originalMesh_.faces()) {
        halfedge_descriptor h = originalMesh_.halfedge(f);

        HalfedgeUV uvData;
        for (int i = 0; i < 3; ++i) {
            uvData.uv[i] = mesh.texcoord(h);
            uvData.pos[i] = originalMesh_.point(originalMesh_.target(h));
            h = originalMesh_.next(h);
        }
        faceUVs_[f] = uvData;

        // Store material index
        if (hasMultipleMaterials_) {
            faceMaterials_[f] = mesh.faceMaterial(f);
        }
    }

    // Build main AABB tree from the copied mesh (for single material or fallback)
    tree_.rebuild(faces(originalMesh_).first, faces(originalMesh_).second, originalMesh_);
    tree_.accelerate_distance_queries();

    // Build per-material AABB trees for fast material-specific queries
    if (hasMultipleMaterials_) {
        int numMaterials = static_cast<int>(materialNames_.size());
        materialMeshes_.resize(numMaterials);
        materialTrees_.resize(numMaterials);
        materialFaceMap_.resize(numMaterials);

        // Group faces by material and create per-material meshes
        for (int matIdx = 0; matIdx < numMaterials; ++matIdx) {
            materialMeshes_[matIdx] = std::make_unique<CGALMesh>();
            CGALMesh& matMesh = *materialMeshes_[matIdx];

            // Collect vertices and faces for this material
            std::map<vertex_descriptor, vertex_descriptor> vertexMap;

            for (face_descriptor f : originalMesh_.faces()) {
                if (mesh.faceMaterial(f) != matIdx) continue;

                halfedge_descriptor h = originalMesh_.halfedge(f);
                std::array<vertex_descriptor, 3> newVerts;

                for (int i = 0; i < 3; ++i) {
                    vertex_descriptor v = originalMesh_.target(h);
                    auto it = vertexMap.find(v);
                    if (it == vertexMap.end()) {
                        vertex_descriptor nv = matMesh.add_vertex(originalMesh_.point(v));
                        vertexMap[v] = nv;
                        newVerts[i] = nv;
                    } else {
                        newVerts[i] = it->second;
                    }
                    h = originalMesh_.next(h);
                }

                face_descriptor newF = matMesh.add_face(newVerts[0], newVerts[1], newVerts[2]);
                if (newF != CGALMesh::null_face()) {
                    materialFaceMap_[matIdx][newF] = f;  // Map new face to original face
                }
            }

            // Build AABB tree for this material
            if (matMesh.number_of_faces() > 0) {
                materialTrees_[matIdx] = std::make_unique<AABB_tree>(
                    faces(matMesh).first, faces(matMesh).second, matMesh);
                materialTrees_[matIdx]->accelerate_distance_queries();
                std::cout << "  Material " << matIdx << " (" << materialNames_[matIdx]
                          << "): " << matMesh.number_of_faces() << " faces\n";
            }
        }
    }

    initialized_ = true;
    std::cout << "TextureTransfer: Initialized with " << numFaces << " faces\n";
}

// Check if UV values indicate a seam crossing (large UV discontinuity)
static bool hasSeamCrossing(const Point_2& uv0, const Point_2& uv1, const Point_2& uv2, double threshold = 0.3) {
    double du01 = std::abs(CGAL::to_double(uv0.x()) - CGAL::to_double(uv1.x()));
    double dv01 = std::abs(CGAL::to_double(uv0.y()) - CGAL::to_double(uv1.y()));
    double du02 = std::abs(CGAL::to_double(uv0.x()) - CGAL::to_double(uv2.x()));
    double dv02 = std::abs(CGAL::to_double(uv0.y()) - CGAL::to_double(uv2.y()));
    double du12 = std::abs(CGAL::to_double(uv1.x()) - CGAL::to_double(uv2.x()));
    double dv12 = std::abs(CGAL::to_double(uv1.y()) - CGAL::to_double(uv2.y()));

    return (du01 > threshold || dv01 > threshold ||
            du02 > threshold || dv02 > threshold ||
            du12 > threshold || dv12 > threshold);
}

// Get triangle vertices and UV data for a face
void TextureTransfer::getFaceData(face_descriptor f,
    Point_3& op0, Point_3& op1, Point_3& op2,
    const HalfedgeUV*& uvData) const
{
    auto it = faceUVs_.find(f);
    if (it == faceUVs_.end()) {
        uvData = nullptr;
        return;
    }
    uvData = &(it->second);

    // Use stored positions for consistency
    op0 = uvData->pos[0];
    op1 = uvData->pos[1];
    op2 = uvData->pos[2];
}

// Calculate how well a point fits within a triangle (returns extrapolation error)
double TextureTransfer::calcExtrapolationError(
    const Point_3& point,
    const Point_3& op0, const Point_3& op1, const Point_3& op2) const
{
    // Project point onto triangle plane
    Vector_3 normal = CGAL::cross_product(op1 - op0, op2 - op0);
    double normalLen = std::sqrt(CGAL::to_double(normal.squared_length()));

    if (normalLen < 1e-12) return 1e10; // Degenerate triangle

    Vector_3 toPoint = point - op0;
    double planeDist = std::abs(CGAL::to_double(toPoint * normal)) / normalLen;

    // Compute barycentric coordinates
    Vector_3 v0 = op1 - op0;
    Vector_3 v1 = op2 - op0;
    Vector_3 v2 = point - op0;

    double d00 = CGAL::to_double(v0 * v0);
    double d01 = CGAL::to_double(v0 * v1);
    double d11 = CGAL::to_double(v1 * v1);
    double d20 = CGAL::to_double(v2 * v0);
    double d21 = CGAL::to_double(v2 * v1);

    double denom = d00 * d11 - d01 * d01;
    if (std::abs(denom) < 1e-12) return 1e10;

    double bv = (d11 * d20 - d01 * d21) / denom;
    double bw = (d00 * d21 - d01 * d20) / denom;
    double bu = 1.0 - bv - bw;

    // Error is how far outside [0,1] the barycentric coords are
    double error = 0;
    if (bu < 0) error += -bu;
    if (bu > 1) error += bu - 1;
    if (bv < 0) error += -bv;
    if (bv > 1) error += bv - 1;
    if (bw < 0) error += -bw;
    if (bw > 1) error += bw - 1;

    return error + planeDist * 0.1; // Include plane distance as secondary factor
}

// Find closest point on faces with a specific material (using per-material AABB tree)
TextureTransfer::Point_and_primitive_id TextureTransfer::findClosestPointForMaterial(
    const Point_3& point, int materialIdx) const
{
    // Use per-material AABB tree for O(log N) query instead of O(N) brute force
    if (materialIdx >= 0 && materialIdx < static_cast<int>(materialTrees_.size()) &&
        materialTrees_[materialIdx]) {

        // Query the material-specific AABB tree
        auto closest = materialTrees_[materialIdx]->closest_point_and_primitive(point);

        // Map the material mesh face back to original mesh face
        auto it = materialFaceMap_[materialIdx].find(closest.second);
        if (it != materialFaceMap_[materialIdx].end()) {
            return Point_and_primitive_id(closest.first, it->second);
        }
    }

    // Fallback to main tree if material tree not available
    return tree_.closest_point_and_primitive(point);
}

void TextureTransfer::transfer(Mesh& simplifiedMesh) {
    if (!initialized_) {
        std::cerr << "TextureTransfer: Not initialized\n";
        return;
    }

    CGALMesh& sm = simplifiedMesh.cgal_mesh;

    if (!simplifiedMesh.hasTexcoords()) {
        simplifiedMesh.initTexcoordMap();
    }

    // Initialize material map for simplified mesh if original had materials
    if (hasMultipleMaterials_ && !simplifiedMesh.hasMaterials()) {
        simplifiedMesh.initMaterialMap();
        simplifiedMesh.setMaterialNames(materialNames_);
    }

    size_t numFaces = sm.number_of_faces();
    size_t processed = 0;
    size_t seamFixes = 0;

    for (face_descriptor f : sm.faces()) {
        halfedge_descriptor h0 = sm.halfedge(f);
        halfedge_descriptor h1 = sm.next(h0);
        halfedge_descriptor h2 = sm.next(h1);

        Point_3 p0 = sm.point(sm.target(h0));
        Point_3 p1 = sm.point(sm.target(h1));
        Point_3 p2 = sm.point(sm.target(h2));

        // Calculate centroid for material determination
        Point_3 centroid(
            (CGAL::to_double(p0.x()) + CGAL::to_double(p1.x()) + CGAL::to_double(p2.x())) / 3.0,
            (CGAL::to_double(p0.y()) + CGAL::to_double(p1.y()) + CGAL::to_double(p2.y())) / 3.0,
            (CGAL::to_double(p0.z()) + CGAL::to_double(p1.z()) + CGAL::to_double(p2.z())) / 3.0
        );

        // Determine material for this face
        int faceMaterial = 0;
        if (hasMultipleMaterials_) {
            Point_and_primitive_id closestCentroid = tree_.closest_point_and_primitive(centroid);
            auto matIt = faceMaterials_.find(closestCentroid.second);
            if (matIt != faceMaterials_.end()) {
                faceMaterial = matIt->second;
            }
            simplifiedMesh.setFaceMaterial(f, faceMaterial);
        }

        Point_2 uv0, uv1, uv2;

        if (hasMultipleMaterials_) {
            // Multi-material: only query from faces with the same material
            Point_and_primitive_id closest0 = findClosestPointForMaterial(p0, faceMaterial);
            Point_and_primitive_id closest1 = findClosestPointForMaterial(p1, faceMaterial);
            Point_and_primitive_id closest2 = findClosestPointForMaterial(p2, faceMaterial);

            uv0 = interpolateUVFromClosest(p0, closest0);
            uv1 = interpolateUVFromClosest(p1, closest1);
            uv2 = interpolateUVFromClosest(p2, closest2);

            // Check for seam crossing within same material
            if (hasSeamCrossing(uv0, uv1, uv2)) {
                Point_and_primitive_id closestCentroidMat = findClosestPointForMaterial(centroid, faceMaterial);

                face_descriptor candidates[4] = {
                    closestCentroidMat.second,
                    closest0.second,
                    closest1.second,
                    closest2.second
                };

                double bestError = 1e10;
                int bestCandidate = -1;

                for (int i = 0; i < 4; ++i) {
                    Point_3 op0, op1, op2;
                    const HalfedgeUV* uvData;
                    getFaceData(candidates[i], op0, op1, op2, uvData);

                    if (!uvData) continue;

                    // Check material
                    auto matIt = faceMaterials_.find(candidates[i]);
                    if (matIt != faceMaterials_.end() && matIt->second != faceMaterial) continue;

                    double totalError =
                        calcExtrapolationError(p0, op0, op1, op2) +
                        calcExtrapolationError(p1, op0, op1, op2) +
                        calcExtrapolationError(p2, op0, op1, op2);

                    if (totalError < bestError) {
                        bestError = totalError;
                        bestCandidate = i;
                    }
                }

                if (bestCandidate >= 0 && bestError < 2.0) {
                    Point_3 op0, op1, op2;
                    const HalfedgeUV* uvData;
                    getFaceData(candidates[bestCandidate], op0, op1, op2, uvData);

                    if (uvData) {
                        uv0 = interpolateUVFromFace(p0, op0, op1, op2, *uvData);
                        uv1 = interpolateUVFromFace(p1, op0, op1, op2, *uvData);
                        uv2 = interpolateUVFromFace(p2, op0, op1, op2, *uvData);
                        seamFixes++;
                    }
                }
            }
        } else {
            // Single material: use fast AABB tree queries
            Point_and_primitive_id closest0 = tree_.closest_point_and_primitive(p0);
            Point_and_primitive_id closest1 = tree_.closest_point_and_primitive(p1);
            Point_and_primitive_id closest2 = tree_.closest_point_and_primitive(p2);

            uv0 = interpolateUVFromClosest(p0, closest0);
            uv1 = interpolateUVFromClosest(p1, closest1);
            uv2 = interpolateUVFromClosest(p2, closest2);

            // Check if this crosses a UV seam
            if (hasSeamCrossing(uv0, uv1, uv2)) {
                Point_and_primitive_id closestCentroid = tree_.closest_point_and_primitive(centroid);

                face_descriptor candidates[4] = {
                    closestCentroid.second,
                    closest0.second,
                    closest1.second,
                    closest2.second
                };

                double bestError = 1e10;
                int bestCandidate = -1;

                for (int i = 0; i < 4; ++i) {
                    Point_3 op0, op1, op2;
                    const HalfedgeUV* uvData;
                    getFaceData(candidates[i], op0, op1, op2, uvData);

                    if (!uvData) continue;

                    double totalError =
                        calcExtrapolationError(p0, op0, op1, op2) +
                        calcExtrapolationError(p1, op0, op1, op2) +
                        calcExtrapolationError(p2, op0, op1, op2);

                    if (totalError < bestError) {
                        bestError = totalError;
                        bestCandidate = i;
                    }
                }

                if (bestCandidate >= 0 && bestError < 2.0) {
                    Point_3 op0, op1, op2;
                    const HalfedgeUV* uvData;
                    getFaceData(candidates[bestCandidate], op0, op1, op2, uvData);

                    if (uvData) {
                        uv0 = interpolateUVFromFace(p0, op0, op1, op2, *uvData);
                        uv1 = interpolateUVFromFace(p1, op0, op1, op2, *uvData);
                        uv2 = interpolateUVFromFace(p2, op0, op1, op2, *uvData);
                        seamFixes++;
                    }
                }
            }
        }

        simplifiedMesh.setTexcoord(h0, uv0);
        simplifiedMesh.setTexcoord(h1, uv1);
        simplifiedMesh.setTexcoord(h2, uv2);

        processed++;
        if (processed % 10000 == 0) {
            std::cout << "\r  UV Transfer: " << processed << "/" << numFaces << " faces" << std::flush;
        }
    }

    std::cout << "\r  UV Transfer: " << numFaces << "/" << numFaces << " faces - Done\n";
    std::cout << "  Seam fixes applied: " << seamFixes << " triangles\n";
}

Point_2 TextureTransfer::interpolateUVFromClosest(const Point_3& point,
    const Point_and_primitive_id& closest) const
{
    face_descriptor originalFace = closest.second;

    auto it = faceUVs_.find(originalFace);
    if (it == faceUVs_.end()) {
        return Point_2(0, 0);
    }

    const HalfedgeUV& uvData = it->second;

    // Use stored positions for consistency
    const Point_3& p0 = uvData.pos[0];
    const Point_3& p1 = uvData.pos[1];
    const Point_3& p2 = uvData.pos[2];

    double u, v, w;
    getBarycentricCoordinates(p0, p1, p2, closest.first, u, v, w);

    double uvX = u * CGAL::to_double(uvData.uv[0].x()) +
                 v * CGAL::to_double(uvData.uv[1].x()) +
                 w * CGAL::to_double(uvData.uv[2].x());

    double uvY = u * CGAL::to_double(uvData.uv[0].y()) +
                 v * CGAL::to_double(uvData.uv[1].y()) +
                 w * CGAL::to_double(uvData.uv[2].y());

    return Point_2(uvX, uvY);
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
        u = v = w = 1.0 / 3.0;
        return;
    }

    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0 - v - w;

    u = std::max(0.0, std::min(1.0, u));
    v = std::max(0.0, std::min(1.0, v));
    w = std::max(0.0, std::min(1.0, w));

    double sum = u + v + w;
    if (sum > 0) {
        u /= sum;
        v /= sum;
        w /= sum;
    }
}

Point_2 TextureTransfer::interpolateUVFromFace(
    const Point_3& point,
    const Point_3& op0, const Point_3& op1, const Point_3& op2,
    const HalfedgeUV& uvData) const
{
    Vector_3 normal = CGAL::cross_product(op1 - op0, op2 - op0);
    double normalLen = std::sqrt(CGAL::to_double(normal.squared_length()));

    Point_3 projectedPoint = point;
    if (normalLen > 1e-12) {
        Vector_3 toPoint = point - op0;
        double dist = CGAL::to_double(toPoint * normal) / (normalLen * normalLen);
        projectedPoint = point - dist * normal;
    }

    double u, v, w;
    getBarycentricCoordinates(op0, op1, op2, projectedPoint, u, v, w);

    double uvX = u * CGAL::to_double(uvData.uv[0].x()) +
                 v * CGAL::to_double(uvData.uv[1].x()) +
                 w * CGAL::to_double(uvData.uv[2].x());

    double uvY = u * CGAL::to_double(uvData.uv[0].y()) +
                 v * CGAL::to_double(uvData.uv[1].y()) +
                 w * CGAL::to_double(uvData.uv[2].y());

    return Point_2(uvX, uvY);
}

Point_2 TextureTransfer::interpolateUV(const Point_3& point) const {
    Point_and_primitive_id closest = tree_.closest_point_and_primitive(point);
    return interpolateUVFromClosest(point, closest);
}

} // namespace sps
