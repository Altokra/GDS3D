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

size_t PolygonCleaner::CleanPolygonsInPlace(
    std::vector<GDSPolygon*>& polygons,
    double delta_db)
{
    if (polygons.empty()) return 0;

    // Get unit conversion factor from first polygon's layer
    double unitu = polygons[0]->GetLayer()->Units->Unitu;
    cInt delta_int = static_cast<cInt>(rounded(delta_db / unitu));

    v_printf(2, "[PolygonCleaner] InPlace: %zu polygons, delta_int=%lld\n", polygons.size(), delta_int);

    // Convert all polygons to Clipper paths
    Paths input;
    input.reserve(polygons.size());
    for (size_t i = 0; i < polygons.size(); i++) {
        if (polygons[i]->GetPoints() >= 3) {
            input.push_back(PolygonToClipperPath(polygons[i]));
        }
    }

    if (input.empty()) return 0;

    v_printf(2, "[PolygonCleaner] Input paths: %zu\n", input.size());

    // Run union
    Paths cleaned = UnionAndClean(input, delta_int);
    if (cleaned.empty()) {
        v_printf(1, "[PolygonCleaner] Warning: cleaning produced empty result.\n");
        return 0;
    }

    v_printf(2, "[PolygonCleaner] Union result: %zu polygons (input was %zu)\n",
             cleaned.size(), input.size());

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

    v_printf(2, "[PolygonCleaner] Input paths: %zu\n", input.size());

    // Run union
    Paths cleaned = UnionAndClean(input, delta_int);
    if (cleaned.empty()) {
        v_printf(1, "[PolygonCleaner] Warning: cleaning produced empty result.\n");
        return 0;
    }

    v_printf(2, "[PolygonCleaner] Union result: %zu polygons (input was %zu)\n",
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
