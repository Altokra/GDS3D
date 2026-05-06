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

// --- Check if a Clipper path is a valid polygon (not bow-tie, not degenerate) ---
static bool IsPathValid(const Path& p) {
    if (p.size() < 3) return false;
    // Check area: bow-tie / self-intersecting polygons have very small area
    double area = std::abs(ClipperLib::Area(p));
    if (area < 0.5) return false;  // degenerate
    return true;
}

// --- Fix diagonal edges in a path to make it Manhattan-compliant ---
// For each diagonal edge A→B, replace with L-shaped path A→(B.X,A.Y)→B or A→(A.X,B.Y)→B
// Choose the shorter L-path to minimize distortion.
static Path FixDiagonalEdges(const Path& input)
{
    if (input.size() < 3) return input;

    Path result;
    result.reserve(input.size() * 2);

    for (size_t i = 0; i < input.size(); i++) {
        const IntPoint& A = input[i];
        const IntPoint& B = input[(i + 1) % input.size()];

        cInt dx = B.X - A.X;
        cInt dy = B.Y - A.Y;

        if (dx == 0 || dy == 0) {
            // Already axis-aligned: keep the edge as-is
            if (result.empty() || (result.back().X != A.X || result.back().Y != A.Y)) {
                result.push_back(A);
            }
        } else {
            // Diagonal edge: replace with L-shaped path
            // Option 1: A → (B.X, A.Y) → B  (horizontal first)
            cInt len_h_first = std::max(std::abs(dx), std::abs(dy));
            // Option 2: A → (A.X, B.Y) → B  (vertical first)
            cInt len_v_first = std::max(std::abs(dx), std::abs(dy));
            // Both are equal in Chebyshev metric, pick horizontal-first
            IntPoint corner(B.X, A.Y);
            if (result.empty() || (result.back().X != A.X || result.back().Y != A.Y)) {
                result.push_back(A);
            }
            // Only add corner if it's different from both A and B (avoid degenerate edge)
            if ((corner.X != A.X || corner.Y != A.Y) && (corner.X != B.X || corner.Y != B.Y)) {
                result.push_back(corner);
            }
        }
    }

    // Remove consecutive duplicates
    if (result.size() >= 2) {
        Path deduped;
        deduped.push_back(result[0]);
        for (size_t i = 1; i < result.size(); i++) {
            if (result[i].X != deduped.back().X || result[i].Y != deduped.back().Y) {
                deduped.push_back(result[i]);
            }
        }
        result.swap(deduped);
    }

    // Remove trailing duplicate (closed polygon)
    while (result.size() >= 2 &&
           result.back().X == result[0].X && result.back().Y == result[0].Y) {
        result.pop_back();
    }

    return result;
}

// --- Remove sliver vertices from a single path ---
// Manhattan Intersection Projection method:
//   1. Scan 3 consecutive vertices A-B-C where B has a short adjacent edge.
//   2. Cross product determines convex (spike) vs concave (notch):
//      - Spike (cross > 0): directly skip B (chop off the protrusion)
//      - Notch (cross < 0): skip B, insert Manhattan intersection point B'
//        B' = (C.X, A.Y) or (A.X, C.Y), chosen by dominant axis
//   3. Result path stays strictly axis-aligned (no diagonal edges).
static Path RemoveSliverVertices(const Path& path, cInt threshold)
{
    if (path.size() < 4) return path;

    const size_t n = path.size();

    // Phase 1: identify all sliver vertices
    vector<bool> isSliver(n, false);
    for (size_t i = 0; i < n; i++) {
        size_t prev = (i + n - 1) % n;
        size_t next = (i + 1) % n;
        cInt len_prev = std::max(std::abs(path[i].X - path[prev].X),
                                 std::abs(path[i].Y - path[prev].Y));
        cInt len_next = std::max(std::abs(path[next].X - path[i].X),
                                 std::abs(path[next].Y - path[i].Y));
        if (len_prev < threshold || len_next < threshold) {
            isSliver[i] = true;
        }
    }

    // Phase 2: build result path with Manhattan projection for notches
    Path result;
    result.reserve(n * 2);
    int spikeCount = 0, notchCount = 0;

    for (size_t i = 0; i < n; i++) {
        if (!isSliver[i]) {
            result.push_back(path[i]);
            continue;
        }

        // Find the nearest non-sliver predecessor A in the original path
        size_t aIdx = (i + n - 1) % n;
        while (aIdx != i && isSliver[aIdx]) {
            aIdx = (aIdx + n - 1) % n;
        }
        // Find the nearest non-sliver successor C in the original path
        size_t cIdx = (i + 1) % n;
        while (cIdx != i && isSliver[cIdx]) {
            cIdx = (cIdx + 1) % n;
        }

        const IntPoint& A = path[aIdx];
        const IntPoint& B = path[i];
        const IntPoint& C = path[cIdx];

        // Cross product at B
        cInt cross = (B.X - A.X) * (C.Y - B.Y) - (B.Y - A.Y) * (C.X - B.X);

        if (cross > 0) {
            // CONVEX SPIKE: skip B, A→C connection replaces the spike
            spikeCount++;
            continue;
        } else if (cross < 0) {
            // CONCAVE NOTCH: skip B, insert Manhattan intersection point B'
            notchCount++;
            cInt len_prev = std::max(std::abs(B.X - A.X), std::abs(B.Y - A.Y));
            cInt len_next = std::max(std::abs(C.X - B.X), std::abs(C.Y - B.Y));

            IntPoint Bprime;
            if (len_next >= len_prev) {
                Bprime.X = C.X; Bprime.Y = A.Y;
            } else {
                Bprime.X = A.X; Bprime.Y = C.Y;
            }

            // Insert B' if not duplicate
            if (result.empty() ||
                (result.back().X != Bprime.X || result.back().Y != Bprime.Y)) {
                result.push_back(Bprime);
            }
            continue;
        } else {
            // cross == 0: collinear, just skip B
            continue;
        }
    }

    // Remove consecutive duplicates
    if (result.size() >= 2) {
        Path deduped;
        deduped.push_back(result[0]);
        for (size_t i = 1; i < result.size(); i++) {
            if (result[i].X != deduped.back().X || result[i].Y != deduped.back().Y) {
                deduped.push_back(result[i]);
            }
        }
        result.swap(deduped);
    }

    // Remove trailing vertex if it equals first (closed polygon)
    while (result.size() >= 2 &&
           result.back().X == result[0].X && result.back().Y == result[0].Y) {
        result.pop_back();
    }

    v_printf(1, "[SliverRemoval] %zu -> %zu verts (spikes=%d, notches=%d, thr=%lld)\n",
             n, result.size(), spikeCount, notchCount, threshold);

    if (result.size() < 3) return path; // Safety: don't degenerate
    return result;
}

// --- Core algorithm: same-layer union + optional narrow-edge cleanup ---
// This avoids OCCT solid booleans: Clipper does the 2D layer union first, then
// OCCT only receives clean closed wires to extrude into STEP solids.
static Paths UnionAndClean(const Paths& input, double delta_db, double unitu)
{
    if (input.empty()) return Paths();

    // Remove exact duplicate paths (same vertices in same order)
    Paths uniqueInput;
    uniqueInput.reserve(input.size());
    size_t dupCount = 0;
    for (size_t i = 0; i < input.size(); i++) {
        if (!IsPathValid(input[i])) continue;

        bool isDup = false;
        for (size_t j = 0; j < uniqueInput.size(); j++) {
            if (input[i].size() == uniqueInput[j].size()) {
                bool same = true;
                for (size_t k = 0; k < input[i].size(); k++) {
                    if (input[i][k].X != uniqueInput[j][k].X ||
                        input[i][k].Y != uniqueInput[j][k].Y) {
                        same = false; break;
                    }
                }
                if (same) { isDup = true; break; }
            }
        }
        if (isDup) {
            dupCount++;
        } else {
            uniqueInput.push_back(input[i]);
        }
    }
    if (dupCount > 0) {
        v_printf(1, "[UnionAndClean] Removed %zu duplicate polygons\n", dupCount);
    }
    if (uniqueInput.empty()) return Paths();

    Paths unionResult;
    Clipper c;
    c.AddPaths(uniqueInput, ptSubject, true);
    c.Execute(ctUnion, unionResult, pftNonZero, pftNonZero);

    v_printf(1, "[UnionAndClean] Direct union: %zu input -> %zu output\n",
             uniqueInput.size(), unionResult.size());

    if (delta_db <= 0.0 || unitu <= 0.0) return unionResult;

    cInt threshold = static_cast<cInt>(std::max(1.0, rounded(delta_db / unitu)));
    Paths sliverCleaned;
    sliverCleaned.reserve(unionResult.size());

    for (size_t i = 0; i < unionResult.size(); i++) {
        Path cleaned = RemoveSliverVertices(unionResult[i], threshold);
        cleaned = FixDiagonalEdges(cleaned);
        if (IsPathValid(cleaned)) {
            sliverCleaned.push_back(cleaned);
        }
    }

    if (sliverCleaned.empty()) return unionResult;

    // Re-union once after vertex cleanup to remove any tiny overlaps introduced
    // by Manhattan projection.
    Paths finalResult;
    Clipper c2;
    c2.AddPaths(sliverCleaned, ptSubject, true);
    c2.Execute(ctUnion, finalResult, pftNonZero, pftNonZero);

    v_printf(1, "[UnionAndClean] Sliver cleanup: %zu union paths -> %zu final paths (thr=%lld)\n",
             unionResult.size(), finalResult.size(), threshold);

    return finalResult.empty() ? unionResult : finalResult;
}

size_t PolygonCleaner::CleanPolygonsInPlace(
    std::vector<GDSPolygon*>& polygons,
    double delta_db)
{
    if (polygons.empty()) return 0;

    // Get unit conversion factor from first polygon's layer
    double unitu = polygons[0]->GetLayer()->Units->Unitu;

    v_printf(1, "[PolygonCleaner] InPlace: %zu polygons, delta_db=%.1f, unitu=%.6f\n", polygons.size(), delta_db, unitu);

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

    // --- Diagnostic: check for completely coincident (duplicate) polygons ---
    size_t duplicateCount = 0;
    for (size_t i = 0; i < input.size(); i++) {
        for (size_t j = i + 1; j < input.size(); j++) {
            if (input[i].size() != input[j].size()) continue;
            bool same = true;
            for (size_t k = 0; k < input[i].size(); k++) {
                if (input[i][k].X != input[j][k].X || input[i][k].Y != input[j][k].Y) {
                    same = false;
                    break;
                }
            }
            if (same) {
                duplicateCount++;
                if (duplicateCount <= 3) {
                    v_printf(1, "[PolygonCleaner] DUPLICATE: polygon #%zu and #%zu are identical\n", i, j);
                }
            }
        }
    }
    v_printf(1, "[PolygonCleaner] Found %zu duplicate polygon pairs\n", duplicateCount);

    // --- Diagnostic: print first polygon's coordinates ---
    if (input.size() > 0) {
        v_printf(1, "[PolygonCleaner] First polygon (%zu vertices):\n", input[0].size());
        for (size_t k = 0; k < std::min((size_t)8, input[0].size()); k++) {
            v_printf(1, "  [%zu] X=%lld Y=%lld\n", k, input[0][k].X, input[0][k].Y);
        }
    }

    // Run union (pass delta_db and unitu so ClipperOffset delta is in correct units)
    Paths cleaned = UnionAndClean(input, delta_db, unitu);
    if (cleaned.empty()) {
        v_printf(1, "[PolygonCleaner] Warning: cleaning produced empty result.\n");
        return 0;
    }

    v_printf(1, "[PolygonCleaner] Union result: %zu polygons (input was %zu)\n",
             cleaned.size(), input.size());

    // --- Diagnostic: print first output polygon's coordinates ---
    if (cleaned.size() > 0) {
        v_printf(1, "[PolygonCleaner] First OUTPUT polygon (%zu vertices):\n", cleaned[0].size());
        for (size_t k = 0; k < std::min((size_t)8, cleaned[0].size()); k++) {
            v_printf(1, "  [%zu] X=%lld Y=%lld\n", k, cleaned[0][k].X, cleaned[0][k].Y);
        }
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

    v_printf(2, "[PolygonCleaner] ToCoords: %zu polygons, delta_db=%.1f\n", polygons.size(), delta_db);

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

    // Run union (pass delta_db and unitu so ClipperOffset delta is in correct units)
    Paths cleaned = UnionAndClean(input, delta_db, unitu);
    if (cleaned.empty()) {
        v_printf(1, "[PolygonCleaner] Warning: cleaning produced empty result.\n");
        return 0;
    }

    v_printf(1, "[PolygonCleaner] Union result: %zu polygons (input was %zu)\n",
             cleaned.size(), input.size());

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
