#include "sps/core/mesh.h"
#include "sps/io/mesh_io.h"
#include "sps/simplify/qem_simplifier.h"
#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>

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
              << "  -v            Verbose output\n"
              << "  -h            Show this help\n"
              << "\nExample:\n"
              << "  " << prog << " model.obj simplified.obj -r 0.1\n"
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
    std::cout << "[1/3] Loading mesh: " << inputFile << "\n";
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

    // ========== Simplify ==========
    std::cout << "\n[2/3] Simplifying mesh...\n";
    auto simplifyStart = std::chrono::steady_clock::now();

    int edgesRemoved = CGALSimplifier::simplify(mesh, params);

    auto simplifyEnd = std::chrono::steady_clock::now();
    double simplifyTime = std::chrono::duration<double>(simplifyEnd - simplifyStart).count();

    std::cout << "  Edges collapsed: " << edgesRemoved << "\n";
    std::cout << "  Simplification time: " << formatDuration(simplifyTime) << "\n";

    // ========== Save result ==========
    std::cout << "\n[3/3] Saving result: " << outputFile << "\n";
    auto saveStart = std::chrono::steady_clock::now();

    if (!MeshIO::save(outputFile, mesh)) {
        std::cerr << "Error: Failed to save: " << outputFile << std::endl;
        return 1;
    }

    auto saveEnd = std::chrono::steady_clock::now();
    double saveTime = std::chrono::duration<double>(saveEnd - saveStart).count();

    // ========== Summary ==========
    double totalTime = loadTime + simplifyTime + saveTime;

    std::cout << "\n========== Summary ==========\n";
    std::cout << "  Input:  " << inputFile << "\n";
    std::cout << "  Output: " << outputFile << "\n";
    std::cout << "  Final:  " << mesh.vertexCount() << " vertices, "
              << mesh.faceCount() << " faces\n";
    std::cout << "  Total time: " << formatDuration(totalTime) << "\n";
    std::cout << "    - Load:     " << formatDuration(loadTime) << "\n";
    std::cout << "    - Simplify: " << formatDuration(simplifyTime) << "\n";
    std::cout << "    - Save:     " << formatDuration(saveTime) << "\n";
    std::cout << "Done!\n";

    return 0;
}
