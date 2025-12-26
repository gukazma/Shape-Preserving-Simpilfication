#pragma once

#include "sps/core/mesh.h"

namespace sps {

/**
 * Region Growing for Shape Detection
 *
 * Core function: Detect planar regions in building models
 *
 * Algorithm:
 * 1. Select seed face, start new region
 * 2. Expand to adjacent faces with similar normals
 * 3. Repeat until all faces are grouped
 */
class RegionGrowing {
public:
    struct Params {
        double angleThreshold = 30.0;   // Region growing angle threshold (degrees)
        int minRegionSize = 10;         // Minimum faces per region
        bool verbose = false;
    };

    /**
     * Execute region detection
     * @param mesh Input mesh
     * @param params Parameters
     * @return List of detected planar regions
     */
    std::vector<PlanarRegion> detect(const Mesh& mesh, const Params& params = Params());

private:
    // Select seed face (choose ungrouped face with most consistent normal)
    Index selectSeedFace(const Mesh& mesh, const std::vector<bool>& grouped) const;

    // Grow region starting from seed face
    void growRegion(const Mesh& mesh, PlanarRegion& region,
                    std::vector<bool>& grouped, double angleThreshold) const;

    // Check if two faces can be merged
    bool canMerge(const Vector3d& normal1, const Vector3d& normal2, double angleThreshold) const;

    // Compute average normal of region
    Vector3d computeRegionNormal(const Mesh& mesh, const PlanarRegion& region) const;
};

/**
 * Region Optimizer
 *
 * Optimize region detection results:
 * 1. Handle ungrouped isolated faces
 * 2. Merge too-small regions
 * 3. Smooth jagged boundaries
 */
class RegionOptimizer {
public:
    struct Params {
        int minRegionSize = 10;         // Minimum faces per region
        int boundarySmoothing = 2;      // Boundary smoothing iterations
        bool verbose = false;
    };

    /**
     * Optimize regions
     * @param mesh Input mesh (face region fields will be updated)
     * @param regions Region list (will be modified)
     * @param params Parameters
     */
    void optimize(Mesh& mesh, std::vector<PlanarRegion>& regions, const Params& params = Params());

private:
    // Handle ungrouped faces
    void handleUngroupedFaces(Mesh& mesh, std::vector<PlanarRegion>& regions);

    // Merge small regions
    void mergeSmallRegions(Mesh& mesh, std::vector<PlanarRegion>& regions, int minSize);

    // Smooth boundaries
    void smoothBoundaries(Mesh& mesh, std::vector<PlanarRegion>& regions, int iterations);

    // Find best region for a face
    Index findBestRegion(const Mesh& mesh, Index faceIdx,
                         const std::vector<PlanarRegion>& regions) const;
};

} // namespace sps
