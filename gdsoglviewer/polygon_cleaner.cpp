/*
 * @Author: ka1shu1 cwh979946@163.com
 * @Date: 2026-04-18
 * @Description: Narrow edge cleaning implementation.
 *               Converts GDSPolygon coords to Clipper integer paths,
 *               performs union + morphological open (expand/shrink),
 *               then writes cleaned coordinates back.
 */

#include "polygon_cleaner.h"
#include "gds_globals.h"

// Must include gdselements.h BEFORE gdspolygon.h (defines Point2D)
#include "../libgdsto3d/gdselements.h"
#include "../libgdsto3d/gdspolygon.h"
#include "../libgdsto3d/clipper/clipper.hpp"

#include <algorithm>  // for std::max
#include <cmath>      // for std::abs

using namespace ClipperLib;

// --- Helper: GDSPolygon -> Clipper Path ---
static Path PolygonToClipperPath(GDSPolygon* poly)
{
    Path path;
    size_t n = poly->GetPoints();
    path.resize(n);
    double unitu = poly->GetLayer()->Units->Unitu;
    for (size_t i = 0; i < n; i++) {
        path[i].X = static_cast<cInt>(rounded(poly->GetXCoords(i) / unitu));
        path[i].Y = static_cast<cInt>(rounded(poly->GetYCoords(i) / unitu));
    }
    return path;
}

// --- Helper: Clipper Path -> Point2D vector ---
static vector<Point2D> ClipperPathToPoints(const Path& path, double unitu)
{
    vector<Point2D> coords;
    coords.reserve(path.size());
    for (size_t i = 0; i < path.size(); i++) {
        coords.push_back(Point2D(
            static_cast<double>(path[i].X) * unitu,
            static_cast<double>(path[i].Y) * unitu
        ));
    }
    return coords;
}

// --- Core algorithm: Union overlapping polygons ---
// For now only union is applied to merge overlapping polygons.
// Morphological open (expand+shrink) is disabled because it bridges
// nearby-but-separate polygons, creating false geometry.
static Paths UnionAndClean(const Paths& input, cInt delta_int)
{
    if (input.empty()) return Paths();

    // Union: merge overlapping polygons, keep disconnected regions separate
    Clipper c;
    c.AddPaths(input, ptSubject, true);
    Paths unioned;
    c.Execute(ctUnion, unioned, pftNonZero, pftNonZero);

    // Filter to outer contours only (skip holes)
    Paths contours;
    for (size_t i = 0; i < unioned.size(); i++) {
        if (Orientation(unioned[i])) {
            contours.push_back(unioned[i]);
        }
    }

    return contours;
}

// --- Remove sliver vertices from a single path ---
// A sliver vertex is one where at least one adjacent edge is shorter than threshold.
// Removing it "flattens" the narrow edge.
// Works in Clipper integer coordinates (threshold is in integer units).
static Path RemoveSliverVertices(const Path& path, cInt threshold)
{
    if (path.size() < 4) return path; // Need at least a triangle

    vector<bool> keep(path.size(), true);
    size_t n = path.size();

    // Find the minimum edge length for diagnostics
    cInt minEdgeLen = 0x7FFFFFFFFFFFFFFFLL; // INT64_MAX

    // Mark sliver vertices for removal
    for (size_t i = 0; i < n; i++) {
        size_t prev = (i + n - 1) % n;
        size_t next = (i + 1) % n;

        cInt dx_prev = path[i].X - path[prev].X;
        cInt dy_prev = path[i].Y - path[prev].Y;
        cInt len_prev = std::max(std::abs(dx_prev), std::abs(dy_prev)); // L-infinity for axis-aligned

        cInt dx_next = path[next].X - path[i].X;
        cInt dy_next = path[next].Y - path[i].Y;
        cInt len_next = std::max(std::abs(dx_next), std::abs(dy_next));

        // Track minimum edge length
        cInt minAdj = (len_prev < len_next) ? len_prev : len_next;
        if (minAdj < minEdgeLen) minEdgeLen = minAdj;

        // If one adjacent edge is a sliver (< threshold), mark this vertex for removal
        if (len_prev < threshold || len_next < threshold) {
            keep[i] = false;
        }
    }

    // Build result with only kept vertices
    Path result;
    result.reserve(n);
    int removed = 0;
    for (size_t i = 0; i < n; i++) {
        if (keep[i]) {
            result.push_back(path[i]);
        } else {
            removed++;
        }
    }

    v_printf(1, "[SliverRemoval] %zu vertices -> %zu (removed %d, threshold=%lld, minEdge=%lld)\n",
             n, result.size(), removed, threshold, minEdgeLen);

    if (result.size() < 3) return path; // Safety: don't degenerate
    return result;
}

size_t PolygonCleaner::CleanPolygonsInPlace(
    std::vector<GDSPolygon*>& polygons,
    double delta_db)
{
    if (polygons.empty()) return 0;

    // Get unit conversion factor from first polygon's layer
    double unitu = polygons[0]->GetLayer()->Units->Unitu;
    cInt delta_int = static_cast<cInt>(rounded(delta_db / unitu));

    v_printf(1, "[PolygonCleaner] InPlace: %zu polygons, delta_int=%lld, unitu=%.10f\n", polygons.size(), delta_int, unitu);

    // Convert all polygons to Clipper paths
    Paths input;
    input.reserve(polygons.size());
    for (size_t i = 0; i < polygons.size(); i++) {
        if (polygons[i]->GetPoints() >= 3) {
            input.push_back(PolygonToClipperPath(polygons[i]));
        }
    }

    if (input.empty()) return 0;

    v_printf(1, "[PolygonCleaner] Input paths: %zu\n", input.size());

    // Run union
    Paths cleaned = UnionAndClean(input, delta_int);
    if (cleaned.empty()) {
        v_printf(1, "[PolygonCleaner] Warning: cleaning produced empty result.\n");
        return 0;
    }

    v_printf(1, "[PolygonCleaner] Union result: %zu polygons (input was %zu)\n",
             cleaned.size(), input.size());

    // Remove sliver vertices from each unioned polygon
    for (size_t i = 0; i < cleaned.size(); i++) {
        cleaned[i] = RemoveSliverVertices(cleaned[i], delta_int);
    }

    // Write cleaned coordinates back into original polygon objects
    size_t outputCount = cleaned.size();
    size_t minCount = (polygons.size() < outputCount) ? polygons.size() : outputCount;

    // Update existing polygons with new coords
    for (size_t i = 0; i < minCount; i++) {
        vector<Point2D> newCoords = ClipperPathToPoints(cleaned[i], unitu);
        if (newCoords.size() >= 3) {
            polygons[i]->SetCoords(newCoords);
        }
    }

    // Clear excess original polygons (union reduced count)
    vector<Point2D> emptyCoords;
    for (size_t i = minCount; i < polygons.size(); i++) {
        polygons[i]->SetCoords(emptyCoords);
    }

    v_printf(2, "[PolygonCleaner] %zu input -> %zu output polygons (delta=%.1f)\n",
             polygons.size(), outputCount, delta_db);

    return outputCount;
}

size_t PolygonCleaner::CleanPolygonsToCoords(
    const std::vector<GDSPolygon*>& polygons,
    double delta_db,
    std::map<GDSPolygon*, std::pair<std::vector<double>, std::vector<double>>>& outCoords)
{
    outCoords.clear();
    if (polygons.empty()) return 0;

    // Get unit conversion factor from first polygon's layer
    double unitu = polygons[0]->GetLayer()->Units->Unitu;
    cInt delta_int = static_cast<cInt>(rounded(delta_db / unitu));

    v_printf(2, "[PolygonCleaner] ToCoords: %zu polygons, delta_int=%lld\n", polygons.size(), delta_int);

    // Convert all polygons to Clipper paths
    Paths input;
    input.reserve(polygons.size());
    for (size_t i = 0; i < polygons.size(); i++) {
        if (polygons[i]->GetPoints() >= 3) {
            input.push_back(PolygonToClipperPath(polygons[i]));
        }
    }

    if (input.empty()) return 0;

    v_printf(1, "[PolygonCleaner] Input paths: %zu\n", input.size());

    // Run union
    Paths cleaned = UnionAndClean(input, delta_int);
    if (cleaned.empty()) {
        v_printf(1, "[PolygonCleaner] Warning: cleaning produced empty result.\n");
        return 0;
    }

    v_printf(1, "[PolygonCleaner] Union result: %zu polygons (input was %zu)\n",
             cleaned.size(), input.size());

    // Remove sliver vertices from each unioned polygon
    for (size_t i = 0; i < cleaned.size(); i++) {
        cleaned[i] = RemoveSliverVertices(cleaned[i], delta_int);
    }

    // Map cleaned results back to original polygon pointers
    size_t outputCount = cleaned.size();
    size_t minCount = (polygons.size() < outputCount) ? polygons.size() : outputCount;

    for (size_t i = 0; i < minCount; i++) {
        vector<Point2D> pts = ClipperPathToPoints(cleaned[i], unitu);
        std::vector<double> xs, ys;
        xs.reserve(pts.size());
        ys.reserve(pts.size());
        for (auto& p : pts) {
            xs.push_back(p.X);
            ys.push_back(p.Y);
        }
        outCoords[polygons[i]] = std::make_pair(xs, ys);
    }

    // Mark excess polygons as empty (merged into others)
    for (size_t i = minCount; i < polygons.size(); i++) {
        outCoords[polygons[i]] = std::make_pair(std::vector<double>(), std::vector<double>());
    }

    v_printf(2, "[PolygonCleaner] %zu input -> %zu output polygons (delta=%.1f)\n",
             polygons.size(), outputCount, delta_db);

    return outputCount;
}
