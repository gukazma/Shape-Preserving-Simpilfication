#include "sps/core/mesh.h"
#include "sps/io/mesh_io.h"
#include "sps/simplify/qem_simplifier.h"
#include "sps/simplify/texture_transfer.h"
#include "sps/simplify/texture_baker.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <filesystem>
#include <vector>

namespace fs = std::filesystem;
using namespace sps;

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " <input> [output] [options]\n"
              << "\nSupported formats: OBJ, PLY, OFF, STL\n"
              << "\nOptions:\n"
              << "  -r <ratio>    Simplification ratio (0.0-1.0, default: 0.1)\n"
              << "  -f <count>    Target face count (overrides ratio)\n"
              << "  -g            Use Garland-Heckbert (QEM) policies (default: on)\n"
              << "  -l            Use Lindstrom-Turk policies instead of G-H\n"
              << "  -b            Enable bounded normal change filter (default: on)\n"
              << "  -n            Disable bounded normal change filter\n"
              << "  -t [path]     Enable texture baking (auto-detect or specify texture path)\n"
              << "  -ts <size>    Texture baking output size (default: 2048)\n"
              << "  -v            Verbose output\n"
              << "  -h            Show this help\n"
              << "\nTexture Handling:\n"
              << "  By default, UV coordinates are transferred from original to simplified mesh.\n"
              << "  Use -t to enable texture rebaking which generates new UVs and bakes a new texture.\n"
              << "\nExample:\n"
              << "  " << prog << " model.obj simplified.obj -r 0.1\n"
              << "  " << prog << " model.obj simplified.obj -r 0.1 -t texture.jpg\n"
              << "  " << prog << " model.ply output.ply -f 1000\n";
}

std::string formatDuration(double seconds) {
    if (seconds < 60) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << seconds << "s";
        return oss.str();
    }
    int min = static_cast<int>(seconds) / 60;
    int sec = static_cast<int>(seconds) % 60;
    return std::to_string(min) + "m " + std::to_string(sec) + "s";
}

// Try to find texture file from MTL file or common patterns
std::string findTextureFile(const std::string& objFile, const std::string& mtlFile) {
    fs::path objPath(objFile);
    fs::path dir = objPath.parent_path();

    // First check MTL file
    if (!mtlFile.empty() && fs::exists(mtlFile)) {
        std::ifstream mtl(mtlFile);
        std::string line;
        while (std::getline(mtl, line)) {
            if (line.find("map_Kd") != std::string::npos) {
                std::istringstream iss(line);
                std::string prefix, texFile;
                iss >> prefix >> texFile;
                if (!texFile.empty()) {
                    fs::path texPath = fs::path(mtlFile).parent_path() / texFile;
                    if (fs::exists(texPath)) {
                        return texPath.string();
                    }
                }
            }
        }
    }

    // Try common texture naming patterns
    std::string baseName = objPath.stem().string();
    std::vector<std::string> extensions = {".jpg", ".jpeg", ".png", ".tga", ".bmp"};
    std::vector<std::string> suffixes = {"", "_diffuse", "_albedo", "_color", "_tex"};

    for (const auto& suffix : suffixes) {
        for (const auto& ext : extensions) {
            fs::path texPath = dir / (baseName + suffix + ext);
            if (fs::exists(texPath)) {
                return texPath.string();
            }
        }
    }

    return "";
}

int main(int argc, char* argv[]) {
    std::cout << "=== Shape-Preserving Simplification (CGAL) ===\n";
    std::cout << "High-quality mesh simplification using Garland-Heckbert algorithm\n\n";

    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string inputFile = argv[1];
    std::string outputFile;
    SimplifyParams params;
    params.targetRatio = 0.1;
    params.verbose = true;
    params.useGarlandHeckbert = true;
    params.useBoundedNormalChange = true;

    // Texture baking options
    bool enableTextureBaking = false;
    std::string texturePath;
    int textureSize = 2048;

    // Parse arguments
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-r" && i + 1 < argc) {
            params.targetRatio = std::stod(argv[++i]);
        } else if (arg == "-f" && i + 1 < argc) {
            params.targetFaceCount = std::stoi(argv[++i]);
        } else if (arg == "-g") {
            params.useGarlandHeckbert = true;
        } else if (arg == "-l") {
            params.useGarlandHeckbert = false;
        } else if (arg == "-b") {
            params.useBoundedNormalChange = true;
        } else if (arg == "-n") {
            params.useBoundedNormalChange = false;
        } else if (arg == "-t") {
            enableTextureBaking = true;
            // Check if next arg is a texture path (not another option)
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                texturePath = argv[++i];
            }
        } else if (arg == "-ts" && i + 1 < argc) {
            textureSize = std::stoi(argv[++i]);
        } else if (arg == "-v") {
            params.verbose = true;
        } else if (arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (arg[0] != '-') {
            outputFile = arg;
        }
    }

    // Generate output filename if not specified
    if (outputFile.empty()) {
        size_t dotPos = inputFile.rfind('.');
        if (dotPos != std::string::npos) {
            outputFile = inputFile.substr(0, dotPos) + "_simplified" + inputFile.substr(dotPos);
        } else {
            outputFile = inputFile + "_simplified.obj";
        }
    }

    // ========== Load mesh ==========
    std::cout << "[1/4] Loading mesh: " << inputFile << "\n";
    auto loadStart = std::chrono::steady_clock::now();

    Mesh mesh;
    if (!MeshIO::load(inputFile, mesh)) {
        std::cerr << "Error: Failed to load: " << inputFile << std::endl;
        return 1;
    }

    auto loadEnd = std::chrono::steady_clock::now();
    double loadTime = std::chrono::duration<double>(loadEnd - loadStart).count();

    std::cout << "  Loaded in " << formatDuration(loadTime) << "\n";
    mesh.printStats();

    if (!mesh.isTriangleMesh()) {
        std::cerr << "Error: Input mesh must be a triangle mesh\n";
        return 1;
    }

    bool hasTextures = mesh.hasTexcoords();
    bool hasMultipleMaterials = mesh.hasMaterials() && mesh.getMaterialNames().size() > 1;

    // ========== Initialize texture handling ==========
    TextureTransfer textureTransfer;
    TextureBaker textureBaker;
    bool useTextureBaking = false;

    if (hasTextures) {
        if (enableTextureBaking) {
            std::cout << "\n[2/5] Initializing texture baker...\n";

            // Check if we have multi-material mesh with MTL file
            if (hasMultipleMaterials && !mesh.mtlFile.empty() && fs::exists(mesh.mtlFile)) {
                std::cout << "  Multi-material mode: " << mesh.getMaterialNames().size() << " materials\n";
                std::cout << "  MTL file: " << mesh.mtlFile << "\n";

                if (textureBaker.initializeWithMTL(mesh, mesh.mtlFile)) {
                    useTextureBaking = true;
                } else {
                    std::cerr << "Warning: Failed to initialize multi-texture baker, falling back to UV transfer\n";
                    textureTransfer.initialize(mesh);
                }
            } else {
                // Single texture mode - find texture file if not specified
                if (texturePath.empty()) {
                    texturePath = findTextureFile(inputFile, mesh.mtlFile);
                }

                if (!texturePath.empty() && fs::exists(texturePath)) {
                    std::cout << "  Texture: " << texturePath << "\n";
                    if (textureBaker.initialize(mesh, texturePath)) {
                        useTextureBaking = true;
                    } else {
                        std::cerr << "Warning: Failed to initialize texture baker, falling back to UV transfer\n";
                        textureTransfer.initialize(mesh);
                    }
                } else {
                    std::cerr << "Warning: Texture file not found";
                    if (!texturePath.empty()) std::cerr << ": " << texturePath;
                    std::cerr << ", falling back to UV transfer\n";
                    textureTransfer.initialize(mesh);
                }
            }
        } else {
            std::cout << "\n[2/4] Initializing texture transfer...\n";
            textureTransfer.initialize(mesh);
        }
    }

    // ========== Simplify ==========
    int stepNum = hasTextures ? 3 : 2;
    int totalSteps = hasTextures ? (useTextureBaking ? 5 : 4) : 3;
    std::cout << "\n[" << stepNum << "/" << totalSteps << "] Simplifying mesh...\n";
    auto simplifyStart = std::chrono::steady_clock::now();

    int edgesRemoved = CGALSimplifier::simplify(mesh, params);

    auto simplifyEnd = std::chrono::steady_clock::now();
    double simplifyTime = std::chrono::duration<double>(simplifyEnd - simplifyStart).count();

    std::cout << "  Edges collapsed: " << edgesRemoved << "\n";
    std::cout << "  Simplification time: " << formatDuration(simplifyTime) << "\n";

    // ========== Transfer/Bake texture ==========
    double textureTime = 0;
    std::string outputTexturePath;

    if (hasTextures) {
        stepNum++;
        if (useTextureBaking) {
            std::cout << "\n[" << stepNum << "/" << totalSteps << "] Baking texture...\n";
            auto bakeStart = std::chrono::steady_clock::now();

            // Generate output texture path (JPEG for smaller file size)
            fs::path outPath(outputFile);
            outputTexturePath = (outPath.parent_path() / (outPath.stem().string() + "_baked.jpg")).string();

            if (!textureBaker.bake(mesh, outputTexturePath, textureSize)) {
                std::cerr << "Warning: Texture baking failed\n";
            } else {
                // Clear old material info and create new MTL for baked texture
                mesh.clearMaterials();
                mesh.mtlFile = "";  // Will be set below

                // Create MTL file for baked texture
                fs::path outputPath(outputFile);
                std::string mtlName = outputPath.stem().string() + ".mtl";
                fs::path mtlPath = outputPath.parent_path() / mtlName;

                std::ofstream mtlOut(mtlPath);
                if (mtlOut.is_open()) {
                    fs::path bakedTexPath(outputTexturePath);
                    mtlOut << "# Baked texture material\n";
                    mtlOut << "newmtl baked_material\n";
                    mtlOut << "Ka 1.0 1.0 1.0\n";
                    mtlOut << "Kd 1.0 1.0 1.0\n";
                    mtlOut << "map_Kd " << bakedTexPath.filename().string() << "\n";
                    mtlOut.close();
                    mesh.mtlFile = mtlPath.string();
                    std::cout << "  Created MTL: " << mtlPath << "\n";
                }
            }

            auto bakeEnd = std::chrono::steady_clock::now();
            textureTime = std::chrono::duration<double>(bakeEnd - bakeStart).count();
            std::cout << "  Bake time: " << formatDuration(textureTime) << "\n";
        } else if (textureTransfer.isInitialized()) {
            std::cout << "\n[" << stepNum << "/" << totalSteps << "] Transferring texture coordinates...\n";
            auto transferStart = std::chrono::steady_clock::now();

            textureTransfer.transfer(mesh);

            auto transferEnd = std::chrono::steady_clock::now();
            textureTime = std::chrono::duration<double>(transferEnd - transferStart).count();
            std::cout << "  Transfer time: " << formatDuration(textureTime) << "\n";
        }
    }

    // ========== Save result ==========
    stepNum++;
    std::cout << "\n[" << stepNum << "/" << totalSteps << "] Saving result: " << outputFile << "\n";
    auto saveStart = std::chrono::steady_clock::now();

    if (!MeshIO::save(outputFile, mesh)) {
        std::cerr << "Error: Failed to save: " << outputFile << std::endl;
        return 1;
    }

    auto saveEnd = std::chrono::steady_clock::now();
    double saveTime = std::chrono::duration<double>(saveEnd - saveStart).count();

    // ========== Summary ==========
    double totalTime = loadTime + simplifyTime + textureTime + saveTime;

    std::cout << "\n========== Summary ==========\n";
    std::cout << "  Input:  " << inputFile << "\n";
    std::cout << "  Output: " << outputFile << "\n";
    if (useTextureBaking && !outputTexturePath.empty()) {
        std::cout << "  Baked Texture: " << outputTexturePath << "\n";
    }
    std::cout << "  Final:  " << mesh.vertexCount() << " vertices, "
              << mesh.faceCount() << " faces\n";
    std::cout << "  Total time: " << formatDuration(totalTime) << "\n";
    std::cout << "    - Load:     " << formatDuration(loadTime) << "\n";
    std::cout << "    - Simplify: " << formatDuration(simplifyTime) << "\n";
    if (hasTextures) {
        if (useTextureBaking) {
            std::cout << "    - Texture Bake: " << formatDuration(textureTime) << "\n";
        } else {
            std::cout << "    - UV Transfer: " << formatDuration(textureTime) << "\n";
        }
    }
    std::cout << "    - Save:     " << formatDuration(saveTime) << "\n";
    std::cout << "Done!\n";

    return 0;
}
