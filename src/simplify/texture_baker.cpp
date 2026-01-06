#include "sps/simplify/texture_baker.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <filesystem>

#ifdef SPS_USE_OPENMP
#include <omp.h>
#endif

// stb_image for texture loading/saving
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

// xatlas for UV generation
#include "xatlas/xatlas.h"

namespace fs = std::filesystem;

namespace sps {

bool TextureBaker::parseMTLFile(const std::string& mtlPath) {
    std::ifstream input(mtlPath);
    if (!input.is_open()) {
        std::cerr << "TextureBaker: Cannot open MTL file: " << mtlPath << "\n";
        return false;
    }

    fs::path mtlDir = fs::path(mtlPath).parent_path();

    // First pass: collect all materials and their textures
    std::map<std::string, std::string> materialToTexture;  // material name -> texture path
    std::string currentMaterial;

    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;

        if (prefix == "newmtl") {
            iss >> currentMaterial;
        }
        else if (prefix == "map_Kd" && !currentMaterial.empty()) {
            std::string texFile;
            iss >> texFile;
            fs::path texPath = mtlDir / texFile;
            materialToTexture[currentMaterial] = texPath.string();
        }
    }
    input.close();

    // Now load textures in the order they appear in materialNames_ (from mesh)
    // This ensures material index consistency
    std::cout << "TextureBaker: Loading textures for " << materialNames_.size() << " materials...\n";

    materialTextures_.resize(materialNames_.size());

    for (size_t i = 0; i < materialNames_.size(); ++i) {
        const std::string& matName = materialNames_[i];
        MaterialTexture& mat = materialTextures_[i];
        mat.name = matName;

        auto it = materialToTexture.find(matName);
        if (it != materialToTexture.end()) {
            mat.texturePath = it->second;

            unsigned char* data = stbi_load(mat.texturePath.c_str(),
                                            &mat.width, &mat.height, &mat.channels, 0);
            if (!data) {
                std::cerr << "TextureBaker: Failed to load texture: " << mat.texturePath << "\n";
                // Use a default color instead of failing
                mat.width = 1;
                mat.height = 1;
                mat.channels = 3;
                mat.data = {128, 128, 128};  // Gray
                std::cerr << "  Using default gray for material " << i << " (" << matName << ")\n";
            } else {
                mat.data.assign(data, data + mat.width * mat.height * mat.channels);
                stbi_image_free(data);
                std::cout << "  Material " << i << " (" << matName << "): "
                          << mat.width << "x" << mat.height << " (" << mat.channels << " ch)\n";
            }
        } else {
            // No texture for this material, use default gray
            mat.width = 1;
            mat.height = 1;
            mat.channels = 3;
            mat.data = {128, 128, 128};
            std::cout << "  Material " << i << " (" << matName << "): no texture, using gray\n";
        }
    }

    return true;
}

bool TextureBaker::initializeWithMTL(const Mesh& mesh, const std::string& mtlPath) {
    if (!mesh.hasTexcoords()) {
        std::cerr << "TextureBaker: Original mesh has no texture coordinates\n";
        return false;
    }

    if (!mesh.hasMaterials()) {
        std::cerr << "TextureBaker: Original mesh has no material info, falling back to single texture\n";
        // Try to get first texture from MTL and use single texture mode
    }

    // Copy material names from mesh BEFORE parsing MTL
    // This ensures materialTextures_ indices match mesh's material indices
    materialNames_ = mesh.getMaterialNames();
    std::cout << "TextureBaker: Mesh has " << materialNames_.size() << " materials:\n";
    for (size_t i = 0; i < materialNames_.size(); ++i) {
        std::cout << "  [" << i << "] " << materialNames_[i] << "\n";
    }

    // Parse MTL and load textures (uses materialNames_ order)
    if (!parseMTLFile(mtlPath)) {
        std::cerr << "TextureBaker: Failed to parse MTL file\n";
        return false;
    }

    multiTextureMode_ = true;

    // Copy original mesh for AABB tree
    const CGALMesh& sm = mesh.cgal_mesh;
    originalMesh_ = sm;

    // Build AABB tree
    tree_.rebuild(faces(originalMesh_).first, faces(originalMesh_).second, originalMesh_);

    // Store UV coordinates, positions, and material per face
    faceUVs_.clear();
    faceMaterials_.clear();

    for (face_descriptor f : originalMesh_.faces()) {
        halfedge_descriptor h = originalMesh_.halfedge(f);
        FaceUV uvData;
        for (int i = 0; i < 3; ++i) {
            uvData.uv[i] = mesh.texcoord(h);
            uvData.pos[i] = originalMesh_.point(originalMesh_.target(h));
            h = originalMesh_.next(h);
        }
        faceUVs_[f] = uvData;

        // Store material index
        if (mesh.hasMaterials()) {
            faceMaterials_[f] = mesh.faceMaterial(f);
        } else {
            faceMaterials_[f] = 0;
        }
    }

    tree_.accelerate_distance_queries();

    initialized_ = true;
    std::cout << "TextureBaker: Initialized multi-texture mode with "
              << materialTextures_.size() << " materials, "
              << sm.number_of_faces() << " faces\n";
    return true;
}

bool TextureBaker::initialize(const Mesh& mesh, const std::string& texturePath) {
    if (!mesh.hasTexcoords()) {
        std::cerr << "TextureBaker: Original mesh has no texture coordinates\n";
        return false;
    }

    // Load texture
    texturePath_ = texturePath;
    unsigned char* data = stbi_load(texturePath.c_str(), &textureWidth_, &textureHeight_, &textureChannels_, 0);
    if (!data) {
        std::cerr << "TextureBaker: Failed to load texture: " << texturePath << "\n";
        return false;
    }

    textureData_.assign(data, data + textureWidth_ * textureHeight_ * textureChannels_);
    stbi_image_free(data);

    std::cout << "TextureBaker: Loaded texture " << textureWidth_ << "x" << textureHeight_
              << " (" << textureChannels_ << " channels)\n";

    // Copy original mesh for AABB tree
    const CGALMesh& sm = mesh.cgal_mesh;
    originalMesh_ = sm;

    // Build AABB tree FIRST from originalMesh_
    tree_.rebuild(faces(originalMesh_).first, faces(originalMesh_).second, originalMesh_);

    // Store UV coordinates AND vertex positions for each face
    // IMPORTANT: Use originalMesh_'s face descriptors as keys since AABB tree returns them
    // The halfedge indices are the same between sm and originalMesh_ since it's a copy
    faceUVs_.clear();
    for (face_descriptor f : originalMesh_.faces()) {
        halfedge_descriptor h = originalMesh_.halfedge(f);
        FaceUV uvData;
        for (int i = 0; i < 3; ++i) {
            // mesh.texcoord() uses halfedge index, which is the same in both meshes
            uvData.uv[i] = mesh.texcoord(h);
            uvData.pos[i] = originalMesh_.point(originalMesh_.target(h));
            h = originalMesh_.next(h);
        }
        faceUVs_[f] = uvData;
    }
    tree_.accelerate_distance_queries();

    initialized_ = true;
    std::cout << "TextureBaker: Initialized with " << sm.number_of_faces() << " faces\n";
    return true;
}

bool TextureBaker::generateUVsWithXatlas(Mesh& mesh, int targetSize, int& atlasWidth, int& atlasHeight) {
    CGALMesh& sm = mesh.cgal_mesh;

    // Prepare vertex data for xatlas
    std::vector<float> positions;
    std::vector<uint32_t> indices;
    std::map<vertex_descriptor, uint32_t> vertexIndexMap;

    uint32_t idx = 0;
    for (vertex_descriptor v : sm.vertices()) {
        const Point_3& p = sm.point(v);
        positions.push_back(static_cast<float>(CGAL::to_double(p.x())));
        positions.push_back(static_cast<float>(CGAL::to_double(p.y())));
        positions.push_back(static_cast<float>(CGAL::to_double(p.z())));
        vertexIndexMap[v] = idx++;
    }

    for (face_descriptor f : sm.faces()) {
        halfedge_descriptor h = sm.halfedge(f);
        for (int i = 0; i < 3; ++i) {
            vertex_descriptor v = sm.target(h);
            indices.push_back(vertexIndexMap[v]);
            h = sm.next(h);
        }
    }

    // Create xatlas
    xatlas::Atlas* atlas = xatlas::Create();

    xatlas::MeshDecl meshDecl;
    meshDecl.vertexCount = static_cast<uint32_t>(positions.size() / 3);
    meshDecl.vertexPositionData = positions.data();
    meshDecl.vertexPositionStride = sizeof(float) * 3;
    meshDecl.indexCount = static_cast<uint32_t>(indices.size());
    meshDecl.indexData = indices.data();
    meshDecl.indexFormat = xatlas::IndexFormat::UInt32;

    xatlas::AddMeshError error = xatlas::AddMesh(atlas, meshDecl);
    if (error != xatlas::AddMeshError::Success) {
        std::cerr << "TextureBaker: xatlas AddMesh failed\n";
        xatlas::Destroy(atlas);
        return false;
    }

    // Configure chart and pack options
    xatlas::ChartOptions chartOptions;
    chartOptions.maxIterations = 4;

    xatlas::PackOptions packOptions;
    packOptions.resolution = targetSize;
    packOptions.padding = 8;  // Increased padding to reduce seam artifacts
    packOptions.bilinear = true;
    packOptions.blockAlign = true;

    std::cout << "TextureBaker: Generating UV atlas with xatlas...\n";
    xatlas::Generate(atlas, chartOptions, packOptions);

    atlasWidth = atlas->width;
    atlasHeight = atlas->height;

    std::cout << "TextureBaker: Generated atlas " << atlasWidth << "x" << atlasHeight
              << " with " << atlas->chartCount << " charts\n";

    // Apply new UVs to mesh
    if (!mesh.hasTexcoords()) {
        mesh.initTexcoordMap();
    }

    const xatlas::Mesh& outMesh = atlas->meshes[0];

    // Apply UVs per-halfedge using the xatlas output indices
    uint32_t faceIdx = 0;
    for (face_descriptor f : sm.faces()) {
        halfedge_descriptor h = sm.halfedge(f);
        for (int i = 0; i < 3; ++i) {
            uint32_t xatlasIdx = outMesh.indexArray[faceIdx * 3 + i];
            const xatlas::Vertex& xv = outMesh.vertexArray[xatlasIdx];

            float u = xv.uv[0] / static_cast<float>(atlasWidth);
            float v = xv.uv[1] / static_cast<float>(atlasHeight);

            mesh.setTexcoord(h, Point_2(u, v));
            h = sm.next(h);
        }
        faceIdx++;
    }

    xatlas::Destroy(atlas);
    return true;
}

void TextureBaker::sampleTexture(double u, double v, unsigned char& r, unsigned char& g, unsigned char& b) const {
    // Single texture mode - use default texture
    sampleTexture(0, u, v, r, g, b);
}

void TextureBaker::sampleTexture(int materialIdx, double u, double v,
                                  unsigned char& r, unsigned char& g, unsigned char& b) const {
    // Determine which texture to sample from
    const unsigned char* texData;
    int texWidth, texHeight, texChannels;

    if (multiTextureMode_ && materialIdx >= 0 && materialIdx < static_cast<int>(materialTextures_.size())) {
        const MaterialTexture& mat = materialTextures_[materialIdx];
        texData = mat.data.data();
        texWidth = mat.width;
        texHeight = mat.height;
        texChannels = mat.channels;
    } else {
        // Fallback to single texture mode
        texData = textureData_.data();
        texWidth = textureWidth_;
        texHeight = textureHeight_;
        texChannels = textureChannels_;
    }

    if (!texData || texWidth == 0 || texHeight == 0) {
        r = g = b = 128;  // Default gray
        return;
    }

    // Clamp UV to [0, 1]
    u = std::max(0.0, std::min(1.0, u));
    v = std::max(0.0, std::min(1.0, v));

    // Convert to pixel coordinates (flip V for OpenGL convention)
    double px = u * (texWidth - 1);
    double py = (1.0 - v) * (texHeight - 1);

    // Bilinear interpolation
    int x0 = static_cast<int>(std::floor(px));
    int y0 = static_cast<int>(std::floor(py));
    int x1 = std::min(x0 + 1, texWidth - 1);
    int y1 = std::min(y0 + 1, texHeight - 1);

    double fx = px - x0;
    double fy = py - y0;

    auto getPixel = [texData, texWidth, texChannels](int x, int y, int c) -> unsigned char {
        return texData[(y * texWidth + x) * texChannels + c];
    };

    for (int c = 0; c < 3 && c < texChannels; ++c) {
        double v00 = getPixel(x0, y0, c);
        double v10 = getPixel(x1, y0, c);
        double v01 = getPixel(x0, y1, c);
        double v11 = getPixel(x1, y1, c);

        double val = v00 * (1 - fx) * (1 - fy) +
                     v10 * fx * (1 - fy) +
                     v01 * (1 - fx) * fy +
                     v11 * fx * fy;

        if (c == 0) r = static_cast<unsigned char>(std::clamp(val, 0.0, 255.0));
        else if (c == 1) g = static_cast<unsigned char>(std::clamp(val, 0.0, 255.0));
        else if (c == 2) b = static_cast<unsigned char>(std::clamp(val, 0.0, 255.0));
    }

    // Handle grayscale
    if (texChannels == 1) {
        g = b = r;
    }
}

void TextureBaker::getBarycentricCoords(
    const Point_3& p, const Point_3& a, const Point_3& b, const Point_3& c,
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

    // Clamp and normalize
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

void TextureBaker::sampleColorAt3DPoint(const Point_3& point, unsigned char& r, unsigned char& g, unsigned char& b) const {
    // Find closest point on original mesh
    Point_and_primitive_id closest = tree_.closest_point_and_primitive(point);
    face_descriptor originalFace = closest.second;

    auto it = faceUVs_.find(originalFace);
    if (it == faceUVs_.end()) {
        r = g = b = 128; // Default gray
        return;
    }

    const FaceUV& uvData = it->second;

    // Use stored vertex positions (consistent with UV order)
    const Point_3& p0 = uvData.pos[0];
    const Point_3& p1 = uvData.pos[1];
    const Point_3& p2 = uvData.pos[2];

    // Get barycentric coordinates of closest point
    double bu, bv, bw;
    getBarycentricCoords(closest.first, p0, p1, p2, bu, bv, bw);

    // Interpolate UV
    double uvX = bu * CGAL::to_double(uvData.uv[0].x()) +
                 bv * CGAL::to_double(uvData.uv[1].x()) +
                 bw * CGAL::to_double(uvData.uv[2].x());
    double uvY = bu * CGAL::to_double(uvData.uv[0].y()) +
                 bv * CGAL::to_double(uvData.uv[1].y()) +
                 bw * CGAL::to_double(uvData.uv[2].y());

    // Determine material index for multi-texture sampling
    int materialIdx = 0;
    if (multiTextureMode_) {
        auto matIt = faceMaterials_.find(originalFace);
        if (matIt != faceMaterials_.end()) {
            materialIdx = matIt->second;
        }
    }

    // Sample texture with material index
    sampleTexture(materialIdx, uvX, uvY, r, g, b);
}

void TextureBaker::rasterizeTriangle(
    int v0Idx, int v1Idx, int v2Idx,
    const std::vector<std::pair<float, float>>& uvs,
    const std::vector<Point_3>& positions,
    std::vector<unsigned char>& outputTexture,
    int textureWidth, int textureHeight)
{
    // Get UV coordinates (already normalized to [0,1])
    float u0 = uvs[v0Idx].first, uv_v0 = uvs[v0Idx].second;
    float u1 = uvs[v1Idx].first, uv_v1 = uvs[v1Idx].second;
    float u2 = uvs[v2Idx].first, uv_v2 = uvs[v2Idx].second;

    // Convert to pixel coordinates
    int x0 = static_cast<int>(u0 * textureWidth);
    int y0 = static_cast<int>(uv_v0 * textureHeight);
    int x1 = static_cast<int>(u1 * textureWidth);
    int y1 = static_cast<int>(uv_v1 * textureHeight);
    int x2 = static_cast<int>(u2 * textureWidth);
    int y2 = static_cast<int>(uv_v2 * textureHeight);

    // Bounding box
    int minX = std::max(0, std::min({x0, x1, x2}));
    int maxX = std::min(textureWidth - 1, std::max({x0, x1, x2}));
    int minY = std::max(0, std::min({y0, y1, y2}));
    int maxY = std::min(textureHeight - 1, std::max({y0, y1, y2}));

    // 3D positions
    const Point_3& p0 = positions[v0Idx];
    const Point_3& p1 = positions[v1Idx];
    const Point_3& p2 = positions[v2Idx];

    // Precompute 3D position components
    double p0x = CGAL::to_double(p0.x()), p0y = CGAL::to_double(p0.y()), p0z = CGAL::to_double(p0.z());
    double p1x = CGAL::to_double(p1.x()), p1y = CGAL::to_double(p1.y()), p1z = CGAL::to_double(p1.z());
    double p2x = CGAL::to_double(p2.x()), p2y = CGAL::to_double(p2.y()), p2z = CGAL::to_double(p2.z());

    // Edge function for barycentric coordinates
    auto edgeFunction = [](float ax, float ay, float bx, float by, float cx, float cy) -> float {
        return (cx - ax) * (by - ay) - (cy - ay) * (bx - ax);
    };

    float fx0 = static_cast<float>(x0), fy0 = static_cast<float>(y0);
    float fx1 = static_cast<float>(x1), fy1 = static_cast<float>(y1);
    float fx2 = static_cast<float>(x2), fy2 = static_cast<float>(y2);

    float area = edgeFunction(fx0, fy0, fx1, fy1, fx2, fy2);

    if (std::abs(area) < 1e-6f) return; // Degenerate triangle

    float invArea = 1.0f / area;

    // Rasterize
    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            float px = static_cast<float>(x) + 0.5f;
            float py = static_cast<float>(y) + 0.5f;

            float w0 = edgeFunction(fx1, fy1, fx2, fy2, px, py);
            float w1 = edgeFunction(fx2, fy2, fx0, fy0, px, py);
            float w2 = edgeFunction(fx0, fy0, fx1, fy1, px, py);

            // Check if inside triangle (with small tolerance for edges)
            bool inside = (area > 0) ? (w0 >= -0.5f && w1 >= -0.5f && w2 >= -0.5f)
                                     : (w0 <= 0.5f && w1 <= 0.5f && w2 <= 0.5f);

            if (inside) {
                // Normalize barycentric coordinates
                w0 *= invArea;
                w1 *= invArea;
                w2 *= invArea;

                // Clamp
                w0 = std::max(0.0f, std::min(1.0f, w0));
                w1 = std::max(0.0f, std::min(1.0f, w1));
                w2 = std::max(0.0f, std::min(1.0f, w2));
                float sum = w0 + w1 + w2;
                if (sum > 0) { w0 /= sum; w1 /= sum; w2 /= sum; }

                // Interpolate 3D position
                Point_3 pos3D(
                    w0 * p0x + w1 * p1x + w2 * p2x,
                    w0 * p0y + w1 * p1y + w2 * p2y,
                    w0 * p0z + w1 * p1z + w2 * p2z
                );

                // Sample color from original mesh
                unsigned char r, g, b;
                sampleColorAt3DPoint(pos3D, r, g, b);

                // Write to output texture (flip Y for image convention)
                int outY = textureHeight - 1 - y;
                int idx = (outY * textureWidth + x) * 3;
                outputTexture[idx + 0] = r;
                outputTexture[idx + 1] = g;
                outputTexture[idx + 2] = b;
            }
        }
    }
}

void TextureBaker::dilateTexture(std::vector<unsigned char>& texture, int width, int height, int iterations) {
    // Create a mask to track which pixels are filled
    std::vector<bool> filled(width * height, false);

    // Mark initially filled pixels (non-black)
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = (y * width + x) * 3;
            if (texture[idx] != 0 || texture[idx + 1] != 0 || texture[idx + 2] != 0) {
                filled[y * width + x] = true;
            }
        }
    }

    // 8-connected neighbor offsets
    const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};

    std::vector<unsigned char> tempTexture = texture;

    for (int iter = 0; iter < iterations; ++iter) {
        std::vector<bool> newFilled = filled;
        bool changed = false;

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int pixelIdx = y * width + x;

                // Skip already filled pixels
                if (filled[pixelIdx]) continue;

                // Collect colors from filled neighbors
                int sumR = 0, sumG = 0, sumB = 0;
                int count = 0;

                for (int n = 0; n < 8; ++n) {
                    int nx = x + dx[n];
                    int ny = y + dy[n];

                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        int neighborIdx = ny * width + nx;
                        if (filled[neighborIdx]) {
                            int texIdx = neighborIdx * 3;
                            sumR += tempTexture[texIdx + 0];
                            sumG += tempTexture[texIdx + 1];
                            sumB += tempTexture[texIdx + 2];
                            count++;
                        }
                    }
                }

                // If we have filled neighbors, fill this pixel with average
                if (count > 0) {
                    int texIdx = pixelIdx * 3;
                    texture[texIdx + 0] = static_cast<unsigned char>(sumR / count);
                    texture[texIdx + 1] = static_cast<unsigned char>(sumG / count);
                    texture[texIdx + 2] = static_cast<unsigned char>(sumB / count);
                    newFilled[pixelIdx] = true;
                    changed = true;
                }
            }
        }

        if (!changed) break;  // No more pixels to fill

        filled = newFilled;
        tempTexture = texture;
    }
}

// Structure to hold pixel work item for parallel processing
struct PixelWorkItem {
    int x, y;           // Output pixel coordinates
    Point_3 pos3D;      // 3D position to sample from
};

bool TextureBaker::bake(Mesh& simplifiedMesh, const std::string& outputTexturePath, int textureSize) {
    if (!initialized_) {
        std::cerr << "TextureBaker: Not initialized\n";
        return false;
    }

    // Determine output texture size based on mode
    int targetSize;
    int atlasWidth, atlasHeight;

    if (multiTextureMode_) {
        // Multi-texture mode: find max dimensions from all material textures
        int maxWidth = 0, maxHeight = 0;
        for (const auto& mat : materialTextures_) {
            maxWidth = std::max(maxWidth, mat.width);
            maxHeight = std::max(maxHeight, mat.height);
        }
        targetSize = std::max(maxWidth, maxHeight);
        atlasWidth = maxWidth;
        atlasHeight = maxHeight;
        std::cout << "TextureBaker: Multi-texture mode, using max resolution " << maxWidth << "x" << maxHeight << "\n";
    } else {
        // Single texture mode
        targetSize = std::max(textureWidth_, textureHeight_);
        atlasWidth = textureWidth_;
        atlasHeight = textureHeight_;
        std::cout << "TextureBaker: Using original texture resolution " << textureWidth_ << "x" << textureHeight_ << "\n";
    }

    // Step 1: Generate new UVs with xatlas
    int xatlasWidth, xatlasHeight;
    if (!generateUVsWithXatlas(simplifiedMesh, targetSize, xatlasWidth, xatlasHeight)) {
        std::cerr << "TextureBaker: Failed to generate UV atlas\n";
        return false;
    }

    // Use xatlas dimensions if larger, otherwise use our target
    atlasWidth = std::max(atlasWidth, xatlasWidth);
    atlasHeight = std::max(atlasHeight, xatlasHeight);

    // Step 2: Collect mesh data for rasterization
    CGALMesh& sm = simplifiedMesh.cgal_mesh;

    std::vector<Point_3> positions;
    std::vector<std::pair<float, float>> uvs;
    std::vector<std::array<int, 3>> triangles;

    std::map<vertex_descriptor, int> vertexIndexMap;
    int idx = 0;
    for (vertex_descriptor v : sm.vertices()) {
        positions.push_back(sm.point(v));
        vertexIndexMap[v] = idx++;
    }

    // Resize uvs to match positions
    uvs.resize(positions.size(), {0.0f, 0.0f});

    // Collect triangles and UVs (per-halfedge UVs for proper seam handling)
    struct TriangleData {
        std::array<int, 3> vertexIndices;
        std::array<std::pair<float, float>, 3> uvCoords;
        std::array<Point_3, 3> pos;
    };
    std::vector<TriangleData> triData;
    triData.reserve(sm.number_of_faces());

    for (face_descriptor f : sm.faces()) {
        halfedge_descriptor h = sm.halfedge(f);
        TriangleData td;
        for (int i = 0; i < 3; ++i) {
            vertex_descriptor v = sm.target(h);
            int vIdx = vertexIndexMap[v];
            td.vertexIndices[i] = vIdx;
            td.pos[i] = positions[vIdx];

            Point_2 uv = simplifiedMesh.texcoord(h);
            td.uvCoords[i] = {static_cast<float>(CGAL::to_double(uv.x())),
                             static_cast<float>(CGAL::to_double(uv.y()))};
            h = sm.next(h);
        }
        triData.push_back(td);
    }

    // Step 3: Create output texture
    std::vector<unsigned char> outputTexture(atlasWidth * atlasHeight * 3, 0);

    std::cout << "TextureBaker: Collecting pixels to bake...\n";

    // Step 4: Collect all pixels that need to be baked (parallel-friendly)
    std::vector<PixelWorkItem> workItems;
    workItems.reserve(atlasWidth * atlasHeight / 4); // Rough estimate

    auto edgeFunction = [](float ax, float ay, float bx, float by, float cx, float cy) -> float {
        return (cx - ax) * (by - ay) - (cy - ay) * (bx - ax);
    };

    for (const auto& td : triData) {
        // Get UV coordinates
        float u0 = td.uvCoords[0].first, uv_v0 = td.uvCoords[0].second;
        float u1 = td.uvCoords[1].first, uv_v1 = td.uvCoords[1].second;
        float u2 = td.uvCoords[2].first, uv_v2 = td.uvCoords[2].second;

        // Convert to pixel coordinates
        int x0 = static_cast<int>(u0 * atlasWidth);
        int y0 = static_cast<int>(uv_v0 * atlasHeight);
        int x1 = static_cast<int>(u1 * atlasWidth);
        int y1 = static_cast<int>(uv_v1 * atlasHeight);
        int x2 = static_cast<int>(u2 * atlasWidth);
        int y2 = static_cast<int>(uv_v2 * atlasHeight);

        // Bounding box
        int minX = std::max(0, std::min({x0, x1, x2}));
        int maxX = std::min(atlasWidth - 1, std::max({x0, x1, x2}));
        int minY = std::max(0, std::min({y0, y1, y2}));
        int maxY = std::min(atlasHeight - 1, std::max({y0, y1, y2}));

        float fx0 = static_cast<float>(x0), fy0 = static_cast<float>(y0);
        float fx1 = static_cast<float>(x1), fy1 = static_cast<float>(y1);
        float fx2 = static_cast<float>(x2), fy2 = static_cast<float>(y2);

        float area = edgeFunction(fx0, fy0, fx1, fy1, fx2, fy2);
        if (std::abs(area) < 1e-6f) continue;

        float invArea = 1.0f / area;

        // Precompute 3D position components
        double p0x = CGAL::to_double(td.pos[0].x()), p0y = CGAL::to_double(td.pos[0].y()), p0z = CGAL::to_double(td.pos[0].z());
        double p1x = CGAL::to_double(td.pos[1].x()), p1y = CGAL::to_double(td.pos[1].y()), p1z = CGAL::to_double(td.pos[1].z());
        double p2x = CGAL::to_double(td.pos[2].x()), p2y = CGAL::to_double(td.pos[2].y()), p2z = CGAL::to_double(td.pos[2].z());

        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                float px = static_cast<float>(x) + 0.5f;
                float py = static_cast<float>(y) + 0.5f;

                float w0 = edgeFunction(fx1, fy1, fx2, fy2, px, py);
                float w1 = edgeFunction(fx2, fy2, fx0, fy0, px, py);
                float w2 = edgeFunction(fx0, fy0, fx1, fy1, px, py);

                bool inside = (area > 0) ? (w0 >= -0.5f && w1 >= -0.5f && w2 >= -0.5f)
                                         : (w0 <= 0.5f && w1 <= 0.5f && w2 <= 0.5f);

                if (inside) {
                    w0 *= invArea;
                    w1 *= invArea;
                    w2 *= invArea;

                    w0 = std::max(0.0f, std::min(1.0f, w0));
                    w1 = std::max(0.0f, std::min(1.0f, w1));
                    w2 = std::max(0.0f, std::min(1.0f, w2));
                    float sum = w0 + w1 + w2;
                    if (sum > 0) { w0 /= sum; w1 /= sum; w2 /= sum; }

                    PixelWorkItem item;
                    item.x = x;
                    item.y = y;
                    item.pos3D = Point_3(
                        w0 * p0x + w1 * p1x + w2 * p2x,
                        w0 * p0y + w1 * p1y + w2 * p2y,
                        w0 * p0z + w1 * p1z + w2 * p2z
                    );
                    workItems.push_back(item);
                }
            }
        }
    }

    std::cout << "TextureBaker: Baking " << workItems.size() << " pixels using OpenMP...\n";

    // Step 5: Process pixels in parallel
    std::atomic<size_t> progress(0);
    size_t totalPixels = workItems.size();

#ifdef SPS_USE_OPENMP
    int numThreads = omp_get_max_threads();
    std::cout << "TextureBaker: Using " << numThreads << " threads\n";

    #pragma omp parallel for schedule(dynamic, 1000)
    for (int i = 0; i < static_cast<int>(workItems.size()); ++i) {
        const auto& item = workItems[i];

        unsigned char r, g, b;
        sampleColorAt3DPoint(item.pos3D, r, g, b);

        int outY = atlasHeight - 1 - item.y;
        int idx = (outY * atlasWidth + item.x) * 3;
        outputTexture[idx + 0] = r;
        outputTexture[idx + 1] = g;
        outputTexture[idx + 2] = b;

        size_t p = ++progress;
        if (p % 100000 == 0) {
            #pragma omp critical
            {
                std::cout << "\r  Progress: " << p << "/" << totalPixels << " pixels ("
                          << (100 * p / totalPixels) << "%)" << std::flush;
            }
        }
    }
#else
    // Fallback to sequential processing
    for (size_t i = 0; i < workItems.size(); ++i) {
        const auto& item = workItems[i];

        unsigned char r, g, b;
        sampleColorAt3DPoint(item.pos3D, r, g, b);

        int outY = atlasHeight - 1 - item.y;
        int idx = (outY * atlasWidth + item.x) * 3;
        outputTexture[idx + 0] = r;
        outputTexture[idx + 1] = g;
        outputTexture[idx + 2] = b;

        if (i % 100000 == 0) {
            std::cout << "\r  Progress: " << i << "/" << totalPixels << " pixels ("
                      << (100 * i / totalPixels) << "%)" << std::flush;
        }
    }
#endif

    std::cout << "\r  Progress: " << totalPixels << "/" << totalPixels << " pixels - Done    \n";

    // Step 6: Dilate texture to fill seam gaps
    std::cout << "TextureBaker: Dilating texture to fill seam gaps...\n";
    dilateTexture(outputTexture, atlasWidth, atlasHeight, 16);

    // Step 7: Save output texture
    int result = 0;
    std::string ext = outputTexturePath.substr(outputTexturePath.rfind('.') + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    if (ext == "png") {
        result = stbi_write_png(outputTexturePath.c_str(), atlasWidth, atlasHeight, 3, outputTexture.data(), atlasWidth * 3);
    } else if (ext == "jpg" || ext == "jpeg") {
        result = stbi_write_jpg(outputTexturePath.c_str(), atlasWidth, atlasHeight, 3, outputTexture.data(), 75);
    } else {
        result = stbi_write_png(outputTexturePath.c_str(), atlasWidth, atlasHeight, 3, outputTexture.data(), atlasWidth * 3);
    }

    if (result == 0) {
        std::cerr << "TextureBaker: Failed to save texture: " << outputTexturePath << "\n";
        return false;
    }

    std::cout << "TextureBaker: Saved baked texture to " << outputTexturePath << "\n";
    return true;
}

} // namespace sps
