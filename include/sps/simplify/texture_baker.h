#pragma once

#include "sps/core/mesh.h"
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <vector>
#include <string>

namespace sps {

/**
 * TextureBaker - Rebakes texture from original mesh to simplified mesh
 * Uses xatlas for UV generation and samples colors from original mesh/texture
 * Supports multi-material/multi-texture meshes
 */
class TextureBaker {
public:
    // CGAL types for AABB tree
    using Primitive = CGAL::AABB_face_graph_triangle_primitive<CGALMesh>;
    using AABB_traits = CGAL::AABB_traits_3<Kernel, Primitive>;
    using AABB_tree = CGAL::AABB_tree<AABB_traits>;
    using Point_and_primitive_id = AABB_tree::Point_and_primitive_id;

    // Per-face UV and position data from original mesh
    struct FaceUV {
        Point_2 uv[3];
        Point_3 pos[3];  // Store positions to ensure consistent vertex order
    };

    // Per-material texture data
    struct MaterialTexture {
        std::string name;           // Material name
        std::string texturePath;    // Texture file path
        std::vector<unsigned char> data;  // Texture pixel data
        int width = 0;
        int height = 0;
        int channels = 0;
    };

    TextureBaker() = default;
    ~TextureBaker() = default;

    /**
     * Initialize with original mesh and texture (single texture mode)
     * @param mesh Original mesh with texture coordinates
     * @param texturePath Path to original texture image
     */
    bool initialize(const Mesh& mesh, const std::string& texturePath);

    /**
     * Initialize with original mesh and MTL file (multi-texture mode)
     * Parses MTL file to load all referenced textures
     * @param mesh Original mesh with texture coordinates and materials
     * @param mtlPath Path to MTL file
     */
    bool initializeWithMTL(const Mesh& mesh, const std::string& mtlPath);

    /**
     * Bake texture for simplified mesh
     * Uses xatlas to generate new UV layout, then samples colors from original
     * @param simplifiedMesh Simplified mesh (UV coords will be regenerated)
     * @param outputTexturePath Output texture path
     * @param textureSize Output texture size (width = height)
     * @return true on success
     */
    bool bake(Mesh& simplifiedMesh, const std::string& outputTexturePath, int textureSize = 2048);

    /**
     * Check if initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * Check if multi-texture mode
     */
    bool isMultiTexture() const { return multiTextureMode_; }

    /**
     * Get original texture path (single texture mode)
     */
    const std::string& getTexturePath() const { return texturePath_; }

    /**
     * Get number of materials
     */
    size_t getMaterialCount() const { return materialTextures_.size(); }

private:
    /**
     * Parse MTL file and load all referenced textures
     */
    bool parseMTLFile(const std::string& mtlPath);

    /**
     * Generate UV layout using xatlas
     */
    bool generateUVsWithXatlas(Mesh& mesh, int targetSize, int& atlasWidth, int& atlasHeight);

    /**
     * Sample color from a specific texture at given UV
     */
    void sampleTexture(int materialIdx, double u, double v,
                       unsigned char& r, unsigned char& g, unsigned char& b) const;

    /**
     * Sample color from original texture at given UV (single texture mode)
     */
    void sampleTexture(double u, double v, unsigned char& r, unsigned char& g, unsigned char& b) const;

    /**
     * Find color for a 3D point by sampling from original mesh
     * Automatically handles multi-texture by determining material from closest face
     */
    void sampleColorAt3DPoint(const Point_3& point, unsigned char& r, unsigned char& g, unsigned char& b) const;

    /**
     * Get barycentric coordinates
     */
    static void getBarycentricCoords(
        const Point_3& p, const Point_3& a, const Point_3& b, const Point_3& c,
        double& u, double& v, double& w);

    /**
     * Rasterize a triangle in UV space and fill the texture
     */
    void rasterizeTriangle(
        int v0, int v1, int v2,
        const std::vector<std::pair<float, float>>& uvs,
        const std::vector<Point_3>& positions,
        std::vector<unsigned char>& outputTexture,
        int textureWidth, int textureHeight);

    /**
     * Dilate texture to fill empty pixels near edges (reduces black seam artifacts)
     * @param texture RGB texture data
     * @param width Texture width
     * @param height Texture height
     * @param iterations Number of dilation iterations
     */
    static void dilateTexture(
        std::vector<unsigned char>& texture,
        int width, int height, int iterations = 8);

    // Original mesh data
    CGALMesh originalMesh_;
    AABB_tree tree_;
    std::map<face_descriptor, FaceUV> faceUVs_;
    std::map<face_descriptor, int> faceMaterials_;  // Material index per face

    // Single texture mode data
    std::string texturePath_;
    std::vector<unsigned char> textureData_;
    int textureWidth_ = 0;
    int textureHeight_ = 0;
    int textureChannels_ = 0;

    // Multi-texture mode data
    std::vector<MaterialTexture> materialTextures_;
    std::vector<std::string> materialNames_;
    bool multiTextureMode_ = false;

    bool initialized_ = false;
};

} // namespace sps
