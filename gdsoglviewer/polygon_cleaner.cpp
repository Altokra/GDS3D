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
// Algorithm:
//   1. Scan the polygon for "step" patterns: 3 consecutive vertices A-B-C where
//      one of the edges (A→B or B→C) is a short sliver (< threshold).
//   2. For axis-aligned polygons from Clipper, a step is an L-shaped corner.
//   3. Use the 2D cross product at B to determine the corner type:
//        cross(A,B,C) > 0  → convex (spike)   → remove B directly
//        cross(A,B,C) < 0  → concave (notch)   → extend/fill: keep A, skip B, insert new corner
//   4. Result path stays axis-aligned (no diagonal edges).
static Path RemoveSliverVertices(const Path& path, cInt threshold)
{
    if (path.size() < 4) return path;

    const size_t n = path.size();
    Path result;
    result.reserve(n * 2); // worst case: each step becomes 3 vertices

    for (size_t i = 0; i < n; ++i) {
        size_t prev = (i + n - 1) % n;
        size_t next = (i + 1) % n;

        const IntPoint& A = path[prev]; // previous vertex
        const IntPoint& B = path[i];    // current vertex (potential sliver)
        const IntPoint& C = path[next];  // next vertex

        // Length of adjacent edges (L-infinity for axis-aligned)
        cInt len_prev = std::max(std::abs(B.X - A.X), std::abs(B.Y - A.Y));
        cInt len_next = std::max(std::abs(C.X - B.X), std::abs(C.Y - B.Y));

        bool isShort = (len_prev < threshold) || (len_next < threshold);

        if (!isShort) {
            // Normal vertex, keep it
            result.push_back(B);
            continue;
        }

        // --- Sliver corner detected at B ---
        // Compute 2D cross product of (A→B) × (B→C)
        // For CCW polygons: cross > 0 means convex, cross < 0 means concave
        cInt cross = (B.X - A.X) * (C.Y - B.Y) - (B.Y - A.Y) * (C.X - B.X);

        if (cross > 0) {
            // === CONVEX SPIKE: B is a protruding corner ===
            // Example:    A---B
            //             |   |
            //             |   C
            // Just skip B; the edge A→C replaces the spike.
            // Don't add B, continue. (A is already in result from last iteration,
            // or will be added. But A might have been skipped too if it was also sliver.
            // Safe approach: add B only if it's not consecutive sliver.)
            // Actually, since we skip B, A→C connects directly. This is always safe.
            // Skip B. But don't duplicate A either.
            // We already added A in the previous iteration (if it wasn't short).
            // So just skip B.
            // (No action needed, B is not added.)
            continue;
        } else {
            // === CONCAVE NOTCH: B is a re-entrant corner ===
            // Example:    A   C
            //             | / |
            //             B   |
            //                |
            // Skip B, extend A in the direction of C (fill the notch).
            // Insert a new vertex at the intersection of A's axis and C's axis.
            // The L-shaped step: we want A→NEW→C where NEW keeps both axis directions.
            //
            // Extension strategy: project C's position onto the axis that extends from A.
            // Since axis-aligned, one of the two edges is horizontal, one is vertical.
            // We extend A in the direction of C, then drop down/up to C.
            // The notch "fill" always extends in the direction of the longer leg.
            //
            // Choose fill direction based on which edge is longer (more significant)
            if (len_next >= len_prev) {
                // Extend B's outgoing direction: B→C is the dominant axis
                // Skip B, insert new corner at (C.X, A.Y)
                cInt newX = C.X;
                cInt newY = A.Y;
                // Only add if it's different from last vertex in result
                if (result.empty() || (result.back().X != newX || result.back().Y != newY)) {
                    result.push_back(IntPoint(newX, newY));
                }
            } else {
                // Extend A's incoming direction: A→B is the dominant axis
                // Skip B, insert new corner at (A.X, C.Y)
                cInt newX = A.X;
                cInt newY = C.Y;
                if (result.empty() || (result.back().X != newX || result.back().Y != newY)) {
                    result.push_back(IntPoint(newX, newY));
                }
            }
            // Skip B (don't add it)
            continue;
        }
    }

    // Remove consecutive duplicates
    if (result.size() >= 2) {
        Path deduped;
        deduped.push_back(result[0]);
        for (size_t i = 1; i < result.size(); ++i) {
            if (result[i].X != deduped.back().X || result[i].Y != deduped.back().Y) {
                deduped.push_back(result[i]);
            }
        }
        result.swap(deduped);
    }

    // Also check: last vertex might equal first (closed polygon)
    while (result.size() >= 2 &&
           result.back().X == result[0].X && result.back().Y == result[0].Y) {
        result.pop_back();
    }

    v_printf(1, "[SliverRemoval] %zu vertices -> %zu (threshold=%lld)\n",
             n, result.size(), threshold);

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

    // Remove sliver vertices from each unioned polygon, track changes
    vector<size_t> origSizes(cleaned.size());
    for (size_t i = 0; i < cleaned.size(); i++) {
        origSizes[i] = cleaned[i].size();
        cleaned[i] = RemoveSliverVertices(cleaned[i], delta_int);
    }

    // Log center coordinates of modified polygons (for locating in FreeCAD)
    for (size_t i = 0; i < cleaned.size(); i++) {
        if (cleaned[i].size() < origSizes[i]) {
            double cx = 0, cy = 0;
            for (size_t j = 0; j < cleaned[i].size(); j++) {
                cx += static_cast<double>(cleaned[i][j].X) * unitu;
                cy += static_cast<double>(cleaned[i][j].Y) * unitu;
            }
            cx /= cleaned[i].size();
            cy /= cleaned[i].size();
            v_printf(1, "[SliverRemoval] Polygon #%zu center: (%.3f, %.3f), %zu->%zu vertices\n",
                     i, cx, cy, origSizes[i], cleaned[i].size());
        }
    }

    // Log which polygons were actually modified (center coords for locating in viewer)
    // Already logged per-polygon in RemoveSliverVertices; no extra action needed here.

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
