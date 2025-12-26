#include "sps/core/mesh.h"
#include "sps/io/mesh_io.h"
#include "sps/filter/bilateral_filter.h"
#include "sps/detection/region_detection.h"
#include "sps/simplify/qem_simplifier.h"
#include <iostream>
#include <string>

using namespace sps;

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " <input.obj> [output.obj] [options]\n"
              << "Options:\n"
              << "  -r <ratio>    Simplification ratio (0.0-1.0, default: 0.1)\n"
              << "  -f <count>    Target face count\n"
              << "  -v            Verbose output\n"
              << "  -h            Show this help\n";
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
    Mesh mesh;
    if (!MeshIO::load(inputFile, mesh)) {
        std::cerr << "Failed to load: " << inputFile << std::endl;
        return 1;
    }

    std::cout << "Original: V=" << mesh.vertexCount() << " F=" << mesh.faceCount() << std::endl;

    // Step 1: Bilateral filtering (makes planes flatter)
    if (verbose) std::cout << "\n=== Step 1: Bilateral Filtering ===\n";
    BilateralMeshFilter filter;
    BilateralMeshFilter::Params filterParams;
    filterParams.maxIterations = 3;
    filterParams.angleThreshold = 25.0;
    filterParams.verbose = verbose;
    filter.filter(mesh, filterParams);

    // Step 2: Region detection
    if (verbose) std::cout << "\n=== Step 2: Region Detection ===\n";
    RegionGrowing regionGrower;
    RegionGrowing::Params growParams;
    growParams.angleThreshold = 30.0;
    growParams.verbose = verbose;
    auto regions = regionGrower.detect(mesh, growParams);

    RegionOptimizer optimizer;
    RegionOptimizer::Params optParams;
    optParams.minRegionSize = 5;
    optParams.verbose = verbose;
    optimizer.optimize(mesh, regions, optParams);

    // Step 3: Constrained QEM simplification (keeps edges straighter)
    if (verbose) std::cout << "\n=== Step 3: Shape-Preserving Simplification ===\n";
    QEMSimplifier simplifier;
    QEMSimplifier::Params simplifyParams;
    simplifyParams.targetRatio = ratio;
    simplifyParams.targetFaceCount = targetFaces;
    simplifyParams.useShapeConstraints = true;
    simplifyParams.verbose = verbose;
    simplifier.simplify(mesh, regions, simplifyParams);

    std::cout << "Simplified: V=" << mesh.vertexCount() << " F=" << mesh.faceCount() << std::endl;

    // Save result
    if (!MeshIO::save(outputFile, mesh)) {
        std::cerr << "Failed to save: " << outputFile << std::endl;
        return 1;
    }

    std::cout << "Saved: " << outputFile << "\nDone!\n";
    return 0;
}
