#include "sps/core/mesh.h"
#include "sps/io/mesh_io.h"
#include "sps/filter/bilateral_filter.h"
#include "sps/detection/region_detection.h"
#include "sps/simplify/qem_simplifier.h"
#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>

using namespace sps;

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " <input.obj> [output.obj] [options]\n"
              << "Options:\n"
              << "  -r <ratio>    Simplification ratio (0.0-1.0, default: 0.1)\n"
              << "  -f <count>    Target face count\n"
              << "  -v            Verbose output\n"
              << "  -h            Show this help\n";
}

// Helper to format duration
std::string formatDuration(double seconds) {
    if (seconds < 60) {
        return std::to_string(static_cast<int>(seconds)) + "s";
    }
    int min = static_cast<int>(seconds) / 60;
    int sec = static_cast<int>(seconds) % 60;
    return std::to_string(min) + "m " + std::to_string(sec) + "s";
}

int main(int argc, char* argv[]) {
    std::cout << "=== Shape-Preserving Simplification ===\n";
    std::cout << "Making planes flatter, lines straighter!\n\n";

    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string inputFile = argv[1];
    std::string outputFile = "output.obj";
    double ratio = 0.1;
    int targetFaces = 0;
    bool verbose = false;

    // Parse arguments
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-r" && i + 1 < argc) {
            ratio = std::stod(argv[++i]);
        } else if (arg == "-f" && i + 1 < argc) {
            targetFaces = std::stoi(argv[++i]);
        } else if (arg == "-v") {
            verbose = true;
        } else if (arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (arg[0] != '-') {
            outputFile = arg;
        }
    }

    // Load mesh
    std::cout << "[Step 0] Loading mesh..." << std::endl;
    auto loadStart = std::chrono::steady_clock::now();
    Mesh mesh;
    if (!MeshIO::load(inputFile, mesh)) {
        std::cerr << "Failed to load: " << inputFile << std::endl;
        return 1;
    }
    auto loadEnd = std::chrono::steady_clock::now();
    double loadTime = std::chrono::duration<double>(loadEnd - loadStart).count();

    std::cout << "Original: V=" << mesh.vertexCount() << " F=" << mesh.faceCount()
              << " (loaded in " << formatDuration(loadTime) << ")" << std::endl;

    auto totalStart = std::chrono::steady_clock::now();

    // Step 1: Bilateral filtering (makes planes flatter)
    std::cout << "\n[Step 1] Bilateral Filtering..." << std::endl;
    auto step1Start = std::chrono::steady_clock::now();
    BilateralMeshFilter filter;
    BilateralMeshFilter::Params filterParams;
    filterParams.maxIterations = 3;
    filterParams.angleThreshold = 25.0;
    filterParams.verbose = verbose;
    filter.filter(mesh, filterParams);
    auto step1End = std::chrono::steady_clock::now();
    double step1Time = std::chrono::duration<double>(step1End - step1Start).count();
    std::cout << "[Step 1] Done in " << formatDuration(step1Time) << std::endl;

    // Step 2: Region detection
    std::cout << "\n[Step 2] Region Detection..." << std::endl;
    auto step2Start = std::chrono::steady_clock::now();
    RegionGrowing regionGrower;
    RegionGrowing::Params growParams;
    growParams.angleThreshold = 30.0;
    growParams.verbose = verbose;
    auto regions = regionGrower.detect(mesh, growParams);
    auto step2End = std::chrono::steady_clock::now();
    double step2Time = std::chrono::duration<double>(step2End - step2Start).count();
    std::cout << "[Step 2] Found " << regions.size() << " regions in " << formatDuration(step2Time) << std::endl;

    // Step 2.5: Region optimization
    std::cout << "\n[Step 2.5] Region Optimization..." << std::endl;
    auto step25Start = std::chrono::steady_clock::now();
    RegionOptimizer optimizer;
    RegionOptimizer::Params optParams;
    optParams.minRegionSize = 5;
    optParams.verbose = verbose;
    optimizer.optimize(mesh, regions, optParams);
    auto step25End = std::chrono::steady_clock::now();
    double step25Time = std::chrono::duration<double>(step25End - step25Start).count();
    std::cout << "[Step 2.5] Done in " << formatDuration(step25Time) << std::endl;

    // Step 3: Constrained QEM simplification (keeps edges straighter)
    std::cout << "\n[Step 3] Shape-Preserving Simplification..." << std::endl;
    auto step3Start = std::chrono::steady_clock::now();
    QEMSimplifier simplifier;
    QEMSimplifier::Params simplifyParams;
    simplifyParams.targetRatio = ratio;
    simplifyParams.targetFaceCount = targetFaces;
    simplifyParams.useShapeConstraints = true;
    simplifyParams.verbose = verbose;
    simplifier.simplify(mesh, regions, simplifyParams);
    auto step3End = std::chrono::steady_clock::now();
    double step3Time = std::chrono::duration<double>(step3End - step3Start).count();
    std::cout << "[Step 3] Done in " << formatDuration(step3Time) << std::endl;

    auto totalEnd = std::chrono::steady_clock::now();
    double totalTime = std::chrono::duration<double>(totalEnd - totalStart).count();

    std::cout << "\n========== Summary ==========\n";
    std::cout << "Simplified: V=" << mesh.vertexCount() << " F=" << mesh.faceCount() << std::endl;
    std::cout << "Total processing time: " << formatDuration(totalTime) << std::endl;
    std::cout << "  - Bilateral filter: " << formatDuration(step1Time) << std::endl;
    std::cout << "  - Region detection: " << formatDuration(step2Time) << std::endl;
    std::cout << "  - Region optimize:  " << formatDuration(step25Time) << std::endl;
    std::cout << "  - QEM simplify:     " << formatDuration(step3Time) << std::endl;

    // Save result
    std::cout << "\n[Step 4] Saving result..." << std::endl;
    if (!MeshIO::save(outputFile, mesh)) {
        std::cerr << "Failed to save: " << outputFile << std::endl;
        return 1;
    }

    std::cout << "Saved: " << outputFile << "\nDone!\n";
    return 0;
}
