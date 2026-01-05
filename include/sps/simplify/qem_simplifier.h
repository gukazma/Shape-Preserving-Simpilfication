#pragma once

#include "sps/core/mesh.h"

// CGAL Surface Mesh Simplification headers
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Face_count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Face_count_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk_cost.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/GarlandHeckbert_policies.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_filter.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Constrained_placement.h>
#include <CGAL/Surface_mesh_simplification/Edge_collapse_visitor_base.h>

#include <chrono>
#include <iostream>
#include <iomanip>
#include <functional>

namespace sps {

namespace SMS = CGAL::Surface_mesh_simplification;

/**
 * Progress visitor for edge collapse - reports simplification progress
 */
class SimplificationVisitor : public SMS::Edge_collapse_visitor_base<CGALMesh> {
public:
    using Base = SMS::Edge_collapse_visitor_base<CGALMesh>;
    using Profile = typename Base::Profile;

    SimplificationVisitor(size_t initialFaces, size_t targetFaces, bool verbose = true)
        : initialFaces_(initialFaces)
        , targetFaces_(targetFaces)
        , verbose_(verbose)
        , currentFaces_(initialFaces)
        , lastReportTime_(std::chrono::steady_clock::now())
        , lastReportFaces_(initialFaces)
        , startTime_(std::chrono::steady_clock::now())
    {}

    // Called after each edge collapse (CGAL 6.x API: receives Edge_profile)
    void OnCollapsed(const Profile& profile, vertex_descriptor) {
        currentFaces_ = profile.surface_mesh().number_of_faces();

        if (verbose_) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastReportTime_).count();

            if (elapsed >= 1000) {  // Report every second
                size_t totalToRemove = initialFaces_ - targetFaces_;
                size_t removed = initialFaces_ - currentFaces_;
                double progress = (totalToRemove > 0) ? (100.0 * removed / totalToRemove) : 100.0;

                // Calculate speed
                size_t facesRemovedSinceLastReport = lastReportFaces_ - currentFaces_;
                double speed = facesRemovedSinceLastReport * 1000.0 / elapsed;

                // Estimate remaining time
                size_t remaining = (currentFaces_ > targetFaces_) ? (currentFaces_ - targetFaces_) : 0;
                double etaSeconds = (speed > 0) ? (remaining / speed) : 0;

                int etaMin = static_cast<int>(etaSeconds) / 60;
                int etaSec = static_cast<int>(etaSeconds) % 60;

                std::cout << "\r  Progress: " << std::fixed << std::setprecision(1) << progress << "% "
                          << "| Faces: " << currentFaces_ << "/" << targetFaces_ << " "
                          << "| Speed: " << static_cast<int>(speed) << " f/s "
                          << "| ETA: " << etaMin << "m " << etaSec << "s "
                          << "        " << std::flush;

                lastReportTime_ = now;
                lastReportFaces_ = currentFaces_;
            }
        }
    }

    void printFinalStats() const {
        if (verbose_) {
            auto endTime = std::chrono::steady_clock::now();
            auto totalMs = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime_).count();
            std::cout << "\n  Completed in " << (totalMs / 1000.0) << "s\n";
            std::cout << "  Initial faces: " << initialFaces_ << "\n";
            std::cout << "  Final faces: " << currentFaces_ << "\n";
            std::cout << "  Reduction: " << std::fixed << std::setprecision(1)
                      << (100.0 * (initialFaces_ - currentFaces_) / initialFaces_) << "%\n";
        }
    }

private:
    size_t initialFaces_;
    size_t targetFaces_;
    bool verbose_;
    size_t currentFaces_;
    std::chrono::steady_clock::time_point lastReportTime_;
    size_t lastReportFaces_;
    std::chrono::steady_clock::time_point startTime_;
};

/**
 * CGAL-based QEM mesh simplifier
 * Uses Garland-Heckbert (QEM) policies for high-quality simplification
 */
class CGALSimplifier {
public:
    /**
     * Simplify mesh using CGAL's edge collapse with Garland-Heckbert policies
     * @param mesh The mesh to simplify (modified in place)
     * @param params Simplification parameters
     * @return Number of edges removed
     */
    static int simplify(Mesh& mesh, const SimplifyParams& params = SimplifyParams()) {
        if (!mesh.isTriangleMesh()) {
            std::cerr << "Error: Mesh must be a triangle mesh for simplification\n";
            return 0;
        }

        CGALMesh& sm = mesh.cgal_mesh;
        size_t initialFaces = sm.number_of_faces();
        size_t initialVertices = sm.number_of_vertices();

        // Calculate target face count
        size_t targetFaces = (params.targetFaceCount > 0)
            ? static_cast<size_t>(params.targetFaceCount)
            : static_cast<size_t>(initialFaces * params.targetRatio);

        if (params.verbose) {
            std::cout << "CGAL Simplification:\n"
                      << "  Initial: " << initialVertices << " vertices, " << initialFaces << " faces\n"
                      << "  Target: " << targetFaces << " faces ("
                      << std::fixed << std::setprecision(1)
                      << (100.0 * targetFaces / initialFaces) << "%)\n";
        }

        // Create visitor for progress reporting
        SimplificationVisitor visitor(initialFaces, targetFaces, params.verbose);

        // Create stop predicate
        SMS::Face_count_stop_predicate<CGALMesh> stop(targetFaces);

        int edgesRemoved = 0;

        if (params.useGarlandHeckbert) {
            // Use Garland-Heckbert (QEM) policies - requires Eigen
            using GH_policies = SMS::GarlandHeckbert_plane_policies<CGALMesh, Kernel>;
            using Get_cost = typename GH_policies::Get_cost;
            using Get_placement = typename GH_policies::Get_placement;

            GH_policies gh_policies(sm);
            Get_cost get_cost = gh_policies.get_cost();
            Get_placement get_placement = gh_policies.get_placement();

            if (params.useBoundedNormalChange) {
                // Add bounded normal change filter to prevent large normal changes
                using Filter = SMS::Bounded_normal_change_filter<SMS::Bounded_normal_change_filter<>>;
                Filter filter;

                edgesRemoved = SMS::edge_collapse(
                    sm,
                    stop,
                    CGAL::parameters::get_cost(get_cost)
                                     .get_placement(get_placement)
                                     .filter(filter)
                                     .visitor(visitor)
                );
            } else {
                edgesRemoved = SMS::edge_collapse(
                    sm,
                    stop,
                    CGAL::parameters::get_cost(get_cost)
                                     .get_placement(get_placement)
                                     .visitor(visitor)
                );
            }
        } else {
            // Use Lindstrom-Turk policies (simpler, no Eigen dependency)
            using LT_cost = SMS::LindstromTurk_cost<CGALMesh>;
            using LT_placement = SMS::LindstromTurk_placement<CGALMesh>;

            LT_cost get_cost;
            LT_placement get_placement;

            if (params.useBoundedNormalChange) {
                using Filter = SMS::Bounded_normal_change_filter<SMS::Bounded_normal_change_filter<>>;
                Filter filter;

                edgesRemoved = SMS::edge_collapse(
                    sm,
                    stop,
                    CGAL::parameters::get_cost(get_cost)
                                     .get_placement(get_placement)
                                     .filter(filter)
                                     .visitor(visitor)
                );
            } else {
                edgesRemoved = SMS::edge_collapse(
                    sm,
                    stop,
                    CGAL::parameters::get_cost(get_cost)
                                     .get_placement(get_placement)
                                     .visitor(visitor)
                );
            }
        }

        visitor.printFinalStats();

        // Collect garbage (remove deleted elements)
        sm.collect_garbage();

        if (params.verbose) {
            std::cout << "  After garbage collection: "
                      << sm.number_of_vertices() << " vertices, "
                      << sm.number_of_faces() << " faces\n";
        }

        return edgesRemoved;
    }

    /**
     * Simplify mesh with edge constraints (preserve marked edges)
     * @param mesh The mesh to simplify
     * @param params Simplification parameters
     * @return Number of edges removed
     */
    static int simplifyWithConstraints(Mesh& mesh, const SimplifyParams& params = SimplifyParams()) {
        if (!mesh.isTriangleMesh()) {
            std::cerr << "Error: Mesh must be a triangle mesh for simplification\n";
            return 0;
        }

        CGALMesh& sm = mesh.cgal_mesh;
        size_t initialFaces = sm.number_of_faces();
        size_t initialVertices = sm.number_of_vertices();

        // Calculate target face count
        size_t targetFaces = (params.targetFaceCount > 0)
            ? static_cast<size_t>(params.targetFaceCount)
            : static_cast<size_t>(initialFaces * params.targetRatio);

        if (params.verbose) {
            std::cout << "CGAL Constrained Simplification:\n"
                      << "  Initial: " << initialVertices << " vertices, " << initialFaces << " faces\n"
                      << "  Target: " << targetFaces << " faces\n";
        }

        // Get edge constraint map
        auto& edge_is_constrained_map = mesh.getEdgeConstraintMap();

        SimplificationVisitor visitor(initialFaces, targetFaces, params.verbose);
        SMS::Face_count_stop_predicate<CGALMesh> stop(targetFaces);

        // Use Garland-Heckbert with constrained placement
        using GH_policies = SMS::GarlandHeckbert_plane_policies<CGALMesh, Kernel>;
        using Base_placement = typename GH_policies::Get_placement;
        using Constrained_placement = SMS::Constrained_placement<Base_placement, Mesh::EdgeConstraintMap>;

        GH_policies gh_policies(sm);
        Constrained_placement get_placement(edge_is_constrained_map, gh_policies.get_placement());

        int edgesRemoved = SMS::edge_collapse(
            sm,
            stop,
            CGAL::parameters::get_cost(gh_policies.get_cost())
                             .get_placement(get_placement)
                             .edge_is_constrained_map(edge_is_constrained_map)
                             .visitor(visitor)
        );

        visitor.printFinalStats();
        sm.collect_garbage();

        return edgesRemoved;
    }
};

// Legacy alias for backward compatibility
using QEMSimplifier = CGALSimplifier;

} // namespace sps
