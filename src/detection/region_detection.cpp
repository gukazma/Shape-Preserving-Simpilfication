#include "sps/detection/region_detection.h"
#include <cmath>
#include <queue>
#include <iostream>

namespace sps {

// ==================== RegionGrowing ====================

std::vector<PlanarRegion> RegionGrowing::detect(const Mesh& mesh, const Params& params) {
    if (params.verbose)
        std::cout << "Region growing (angle=" << params.angleThreshold << ")...\n";

    std::vector<PlanarRegion> regions;
    std::vector<bool> grouped(mesh.faces.size(), false);
    Index regionId = 0;

    while (true) {
        Index seedIdx = selectSeedFace(mesh, grouped);
        if (seedIdx == INVALID_INDEX) break;

        PlanarRegion region;
        region.id = regionId++;
        region.faces.push_back(seedIdx);
        grouped[seedIdx] = true;

        growRegion(mesh, region, grouped, params.angleThreshold);
        region.avgNormal = computeRegionNormal(mesh, region);

        // Compute region area
        region.totalArea = 0;
        for (Index fi : region.faces) {
            region.totalArea += mesh.faces[fi].area;
        }

        regions.push_back(std::move(region));
    }

    if (params.verbose)
        std::cout << "  Found " << regions.size() << " regions\n";

    return regions;
}

Index RegionGrowing::selectSeedFace(const Mesh& mesh, const std::vector<bool>& grouped) const {
    for (Index i = 0; i < static_cast<Index>(mesh.faces.size()); ++i) {
        if (!mesh.faces[i].removed && !grouped[i]) {
            return i;
        }
    }
    return INVALID_INDEX;
}

void RegionGrowing::growRegion(const Mesh& mesh, PlanarRegion& region,
                               std::vector<bool>& grouped, double angleThreshold) const {
    std::queue<Index> queue;
    for (Index fi : region.faces) queue.push(fi);

    Vector3d regionNormal = mesh.faces[region.faces[0]].normal;

    while (!queue.empty()) {
        Index current = queue.front();
        queue.pop();

        auto neighbors = mesh.getFaceNeighbors(current);
        for (Index neighbor : neighbors) {
            if (grouped[neighbor] || mesh.faces[neighbor].removed) continue;

            if (canMerge(regionNormal, mesh.faces[neighbor].normal, angleThreshold)) {
                grouped[neighbor] = true;
                region.faces.push_back(neighbor);
                queue.push(neighbor);

                // Update region normal (incremental average)
                regionNormal = computeRegionNormal(mesh, region);
            }
        }
    }
}

bool RegionGrowing::canMerge(const Vector3d& normal1, const Vector3d& normal2, double angleThreshold) const {
    double cosTheta = std::cos(angleThreshold * M_PI / 180.0);
    return normal1.dot(normal2) >= cosTheta;
}

Vector3d RegionGrowing::computeRegionNormal(const Mesh& mesh, const PlanarRegion& region) const {
    Vector3d avgNormal = Vector3d::Zero();
    for (Index fi : region.faces) {
        avgNormal += mesh.faces[fi].normal * mesh.faces[fi].area;
    }
    if (avgNormal.squaredNorm() > 1e-10)
        avgNormal.normalize();
    return avgNormal;
}

// ==================== RegionOptimizer ====================

void RegionOptimizer::optimize(Mesh& mesh, std::vector<PlanarRegion>& regions, const Params& params) {
    if (params.verbose)
        std::cout << "Optimizing regions...\n";

    handleUngroupedFaces(mesh, regions);
    mergeSmallRegions(mesh, regions, params.minRegionSize);

    for (int i = 0; i < params.boundarySmoothing; ++i) {
        smoothBoundaries(mesh, regions, 1);
    }

    // Update face region assignments
    for (auto& region : regions) {
        for (Index fi : region.faces) {
            mesh.faces[fi].region = region.id;
        }
    }

    if (params.verbose)
        std::cout << "  " << regions.size() << " regions after optimization\n";
}

void RegionOptimizer::handleUngroupedFaces(Mesh& mesh, std::vector<PlanarRegion>& regions) {
    for (Index fi = 0; fi < static_cast<Index>(mesh.faces.size()); ++fi) {
        if (mesh.faces[fi].removed) continue;

        bool found = false;
        for (const auto& region : regions) {
            for (Index rfi : region.faces) {
                if (rfi == fi) { found = true; break; }
            }
            if (found) break;
        }

        if (!found && !regions.empty()) {
            Index bestRegion = findBestRegion(mesh, fi, regions);
            if (bestRegion != INVALID_INDEX) {
                regions[bestRegion].faces.push_back(fi);
            }
        }
    }
}

void RegionOptimizer::mergeSmallRegions(Mesh& mesh, std::vector<PlanarRegion>& regions, int minSize) {
    for (size_t i = 0; i < regions.size(); ++i) {
        if (regions[i].removed) continue;
        if (static_cast<int>(regions[i].faces.size()) < minSize) {
            // Find best neighbor region to merge into
            Index bestNeighbor = INVALID_INDEX;
            double bestDot = -1.0;

            for (size_t j = 0; j < regions.size(); ++j) {
                if (i == j || regions[j].removed) continue;
                double dot = regions[i].avgNormal.dot(regions[j].avgNormal);
                if (dot > bestDot) {
                    bestDot = dot;
                    bestNeighbor = static_cast<Index>(j);
                }
            }

            if (bestNeighbor != INVALID_INDEX) {
                // Merge region i into bestNeighbor
                for (Index fi : regions[i].faces) {
                    regions[bestNeighbor].faces.push_back(fi);
                }
                regions[i].faces.clear();
                regions[i].removed = true;
            }
        }
    }
}

void RegionOptimizer::smoothBoundaries(Mesh&, std::vector<PlanarRegion>&, int) {
    // TODO: implement boundary smoothing
}

Index RegionOptimizer::findBestRegion(const Mesh& mesh, Index faceIdx,
                                      const std::vector<PlanarRegion>& regions) const {
    const Vector3d& fn = mesh.faces[faceIdx].normal;
    Index best = INVALID_INDEX;
    double bestDot = -1.0;

    for (size_t i = 0; i < regions.size(); ++i) {
        if (regions[i].removed) continue;
        double dot = fn.dot(regions[i].avgNormal);
        if (dot > bestDot) {
            bestDot = dot;
            best = static_cast<Index>(i);
        }
    }

    return best;
}

} // namespace sps
