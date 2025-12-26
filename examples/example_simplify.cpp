/**
 * Example: Complete simplification pipeline
 *
 * Demonstrates the complete shape-preserving simplification workflow:
 * 1. Bilateral filtering (makes planes flatter)
 * 2. Region detection
 * 3. Constrained QEM simplification (keeps edges straighter)
 */

#include "sps/core/mesh.h"
#include "sps/io/mesh_io.h"
#include "sps/filter/bilateral_filter.h"
#include "sps/detection/region_detection.h"
#include "sps/simplify/qem_simplifier.h"
#include <iostream>

using namespace sps;

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <input.obj> [ratio]\n";
        std::cout << "  ratio: simplification ratio (0.0-1.0, default: 0.1)\n";
        return 1;
    }

    std::string inputFile = argv[1];
    double ratio = (argc > 2) ? std::stod(argv[2]) : 0.1;

    // Load mesh
    Mesh mesh;
    if (!MeshIO::load(inputFile, mesh)) {
        return 1;
    }
    std::cout << "\nOriginal mesh:\n";
    mesh.printStats();

    // Step 1: Bilateral filtering (makes planes flatter)
    std::cout << "\n=== Step 1: Bilateral Filtering ===\n";
    BilateralMeshFilter filter;
    BilateralMeshFilter::Params filterParams;
    filterParams.maxIterations = 5;
    filterParams.angleThreshold = 25.0;
    filterParams.verbose = true;
    filter.filter(mesh, filterParams);

    // Step 2: Region detection
    std::cout << "\n=== Step 2: Region Detection ===\n";
    RegionGrowing regionGrower;
    RegionGrowing::Params growParams;
    growParams.angleThreshold = 30.0;
    growParams.verbose = true;
    auto regions = regionGrower.detect(mesh, growParams);

    RegionOptimizer optimizer;
    RegionOptimizer::Params optParams;
    optParams.minRegionSize = 10;
    optParams.verbose = true;
    optimizer.optimize(mesh, regions, optParams);

    // Step 3: Constrained QEM simplification (keeps edges straighter)
    std::cout << "\n=== Step 3: Shape-Preserving Simplification ===\n";
    QEMSimplifier simplifier;
    QEMSimplifier::Params simplifyParams;
    simplifyParams.targetRatio = ratio;
    simplifyParams.useShapeConstraints = true;
    simplifyParams.verbose = true;
    simplifier.simplify(mesh, regions, simplifyParams);

    std::cout << "\nSimplified mesh:\n";
    mesh.printStats();

    // Save result
    std::string outputFile = "simplified_" + std::to_string(static_cast<int>(ratio * 100)) + ".obj";
    MeshIO::save(outputFile, mesh);

    return 0;
}
