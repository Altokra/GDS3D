/*
 * @Author: ka1shu1 cwh979946@163.com
 * @Date: 2026-04-10 20:34:32
 * @LastEditors: ka1shu1 cwh979946@163.com
 * @LastEditTime: 2026-04-11 2026
 * @FilePath: \AICAD_Research\references\GDS3D\gdsoglviewer\step_export.cpp
 * @Description: STEP export with performance optimization - batch + parallel + progress bar
 */

#include "step_export.h"
#include "gdsobject_ogl.h"
#include "gds_globals.h"
#include "gdspolygon.h"
#include "outputStream.h"
#include "process_cfg.h"
#include "../libgdsto3d/clipper/clipper.hpp"

// --- OpenCASCADE core headers ---
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <STEPControl_Writer.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <BRep_Builder.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>

#include <map>
#include <vector>
#include <utility>
#include <atomic>
#include <cmath>
#include <queue>
#include <string>

using namespace ClipperLib;

// --- Helper: recursive hierarchy traversal (same as Output::Traverse) ---
static void CollectPolygonsRecursive(
    GDSObject* obj,
    const GDSMat& mat,
    std::vector<GDSPolygon*>& out_polygons,
    bool includeChildren)
{
    // Collect polygons from current cell
    for (size_t i = 0; i < obj->PolygonItems.size(); i++) {
        GDSPolygon* src = obj->PolygonItems[i];
        if (!src || src->GetLayer()->Show == 0) continue;

        GDSPolygon* copy = new GDSPolygon(src->GetLayer());
        src->CopyInto(copy);
        copy->transformPoints(mat);
        copy->Orientate();
        out_polygons.push_back(copy);
    }

    if (!includeChildren) return;

    // Recursively traverse child references
    for (size_t i = 0; i < obj->refs.size(); i++) {
        GDSRef* ref = obj->refs[i];
        if (!ref->object) continue;
        GDSMat childMat = mat * ref->mat;
        CollectPolygonsRecursive(ref->object, childMat, out_polygons, true);
    }
}

// Progress bar width (number of '#' characters)
#define PROGRESS_BAR_WIDTH 30

#ifdef WIN32
#include <windows.h>
#endif

// Z-layer grouping key: polygons with same (height, thickness) belong to same physical layer
struct ZLayerKey {
    double height;
    double thickness;
    long long heightKey;
    long long thicknessKey;

    bool operator<(const ZLayerKey& o) const {
        if (heightKey != o.heightKey) return heightKey < o.heightKey;
        return thicknessKey < o.thicknessKey;
    }
};

static ZLayerKey MakeZLayerKey(double height, double thickness)
{
    const double Z_KEY_SCALE = 1000000.0;
    ZLayerKey key;
    key.height = height;
    key.thickness = thickness;
    key.heightKey = (long long)rounded(height * Z_KEY_SCALE);
    key.thicknessKey = (long long)rounded(thickness * Z_KEY_SCALE);
    return key;
}

template <typename PolygonT>
static void LogZLayerGroupSummary(
    const char* prefix,
    const ZLayerKey& key,
    const std::vector<PolygonT*>& polys)
{
    std::map<std::string, size_t> layerCounts;
    size_t validPolygonCount = 0;

    for (size_t i = 0; i < polys.size(); i++) {
        if (!polys[i] || !polys[i]->GetLayer()) continue;
        if (polys[i]->GetPoints() >= 3) validPolygonCount++;

        ProcessLayer* layer = polys[i]->GetLayer();
        char label[256];
        sprintf(label, "%s L%d/DT%d H=%.6g T=%.6g Show=%d",
                layer->Name ? layer->Name : "(unnamed)",
                layer->Layer,
                layer->Datatype,
                layer->Height,
                layer->Thickness,
                layer->Show);
        layerCounts[std::string(label)]++;
    }

    v_printf(1, "[STEP Export] %s Z(H=%.6g,T=%.6g): %zu/%zu valid polygons, %zu layer labels\n",
             prefix, key.height, key.thickness, validPolygonCount, polys.size(), layerCounts.size());

    for (std::map<std::string, size_t>::iterator it = layerCounts.begin();
         it != layerCounts.end(); ++it) {
        v_printf(1, "  - %s: %zu\n", it->first.c_str(), it->second);
    }
}

// Helper: print a single-line progress bar using the same console output as v_printf
static void print_progress(const char* prefix, int current, int total) {
    int pct = (int)((double)current / (double)total * 100.0);
    int filled = (int)((double)current / (double)total * PROGRESS_BAR_WIDTH);

    // Build the progress string
    char buf[256];
    int pos = 0;

    // prefix
    pos += sprintf(buf + pos, "%s [", prefix);
    for (int k = 0; k < PROGRESS_BAR_WIDTH; k++) {
        buf[pos++] = (k < filled) ? '#' : ' ';
    }
    pos += sprintf(buf + pos, "] %d/%d  %3d%%", current, total, pct);

#ifdef WIN32
    // Use WriteConsole like v_printf does (printf doesn't work in Win32 GUI app)
    size_t wlen = strlen(buf) + 1;
    wchar_t* wText = new wchar_t[wlen];
    if (wText) {
        memset(wText, 0, wlen * sizeof(wchar_t));
        MultiByteToWideChar(CP_ACP, NULL, buf, -1, wText, (int)wlen);

        HANDLE std_out = GetStdHandle(STD_OUTPUT_HANDLE);
        if (std_out != INVALID_HANDLE_VALUE) {
            AttachConsole(ATTACH_PARENT_PROCESS);
            // \r to overwrite the same line
            const wchar_t* cr = L"\r";
            unsigned long cChars;
            WriteConsole(std_out, cr, 1, &cChars, NULL);
            WriteConsole(std_out, wText, (DWORD)wcslen(wText), &cChars, NULL);
        }
        delete[] wText;
    }
#else
    printf("\r%s", buf);
    fflush(stdout);
#endif
}

// Helper: print a newline to finish a progress bar line
static void print_progress_end() {
#ifdef WIN32
    HANDLE std_out = GetStdHandle(STD_OUTPUT_HANDLE);
    if (std_out != INVALID_HANDLE_VALUE) {
        unsigned long cChars;
        const wchar_t* nl = L"\r\n";
        WriteConsole(std_out, nl, 2, &cChars, NULL);
    }
#else
    printf("\n");
    fflush(stdout);
#endif
}

static TopoDS_Shape MakeCompoundFromShapes(const std::vector<TopoDS_Shape>& shapes)
{
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);

    for (size_t i = 0; i < shapes.size(); i++) {
        if (!shapes[i].IsNull()) {
            builder.Add(compound, shapes[i]);
        }
    }

    return compound;
}

static TopoDS_Shape FuseTwoShapes(const TopoDS_Shape& a, const TopoDS_Shape& b, size_t opIndex)
{
    if (a.IsNull()) return b;
    if (b.IsNull()) return a;

    BRepAlgoAPI_Fuse fuseOp(a, b);
    fuseOp.SetRunParallel(Standard_True);
    fuseOp.Build();

    if (fuseOp.IsDone() && !fuseOp.Shape().IsNull()) {
        return fuseOp.Shape();
    }

    v_printf(1, "[STEP Export] Warning: fuse op %zu failed, keeping compound fallback\n", opIndex);
    std::vector<TopoDS_Shape> fallback;
    fallback.push_back(a);
    fallback.push_back(b);
    return MakeCompoundFromShapes(fallback);
}

static TopoDS_Shape FuseShapesBalanced(const std::vector<TopoDS_Shape>& shapes)
{
    std::vector<TopoDS_Shape> current;
    current.reserve(shapes.size());
    for (size_t i = 0; i < shapes.size(); i++) {
        if (!shapes[i].IsNull()) current.push_back(shapes[i]);
    }

    if (current.empty()) return TopoDS_Shape();
    if (current.size() == 1) return current[0];

    size_t opIndex = 0;
    size_t level = 0;
    while (current.size() > 1) {
        std::vector<TopoDS_Shape> next;
        next.reserve((current.size() + 1) / 2);

        for (size_t i = 0; i < current.size(); i += 2) {
            if (i + 1 >= current.size()) {
                next.push_back(current[i]);
            } else {
                next.push_back(FuseTwoShapes(current[i], current[i + 1], ++opIndex));
            }
        }

        current.swap(next);
        level++;
        v_printf(1, "[STEP Export] Fuse level %zu complete: %zu shapes remain\n",
                 level, current.size());
    }

    return current[0];
}

static std::vector<std::vector<size_t>> BuildBBoxComponents(const std::vector<TopoDS_Shape>& shapes)
{
    const double FUSE_TOUCH_TOL = 1e-7;

    std::vector<Bnd_Box> boxes(shapes.size());
    for (size_t i = 0; i < shapes.size(); i++) {
        if (!shapes[i].IsNull()) {
            BRepBndLib::Add(shapes[i], boxes[i]);
            boxes[i].SetGap(FUSE_TOUCH_TOL);
        }
    }

    std::vector<int> visited(shapes.size(), 0);
    std::vector<std::vector<size_t>> components;

    for (size_t i = 0; i < shapes.size(); i++) {
        if (visited[i] || shapes[i].IsNull()) continue;

        std::vector<size_t> component;
        std::queue<size_t> q;
        visited[i] = 1;
        q.push(i);

        while (!q.empty()) {
            size_t cur = q.front();
            q.pop();
            component.push_back(cur);

            for (size_t j = 0; j < shapes.size(); j++) {
                if (visited[j] || shapes[j].IsNull()) continue;
                if (!boxes[cur].IsOut(boxes[j])) {
                    visited[j] = 1;
                    q.push(j);
                }
            }
        }

        components.push_back(component);
    }

    return components;
}

static TopoDS_Shape FuseShapesOptimized(const std::vector<TopoDS_Shape>& shapes)
{
    std::vector<TopoDS_Shape> validShapes;
    validShapes.reserve(shapes.size());
    for (size_t i = 0; i < shapes.size(); i++) {
        if (!shapes[i].IsNull()) validShapes.push_back(shapes[i]);
    }

    if (validShapes.empty()) return TopoDS_Shape();
    if (validShapes.size() == 1) return validShapes[0];

    std::vector<std::vector<size_t>> components = BuildBBoxComponents(validShapes);
    size_t singletonComponents = 0;
    size_t maxComponentSize = 0;
    for (size_t i = 0; i < components.size(); i++) {
        if (components[i].size() == 1) singletonComponents++;
        if (components[i].size() > maxComponentSize) maxComponentSize = components[i].size();
    }
    v_printf(1, "[STEP Export] Fuse clustering: %zu solids -> %zu bbox components\n",
             validShapes.size(), components.size());
    v_printf(1, "[STEP Export] Fuse clustering detail: %zu singletons, largest component=%zu solids\n",
             singletonComponents, maxComponentSize);

    std::vector<TopoDS_Shape> componentResults;
    componentResults.reserve(components.size());

    for (size_t c = 0; c < components.size(); c++) {
        std::vector<TopoDS_Shape> componentShapes;
        componentShapes.reserve(components[c].size());
        for (size_t k = 0; k < components[c].size(); k++) {
            componentShapes.push_back(validShapes[components[c][k]]);
        }

        if (componentShapes.size() == 1) {
            componentResults.push_back(componentShapes[0]);
        } else {
            v_printf(1, "[STEP Export] Fusing component %zu/%zu (%zu solids)\n",
                     c + 1, components.size(), componentShapes.size());
            componentResults.push_back(FuseShapesBalanced(componentShapes));
        }
    }

    TopoDS_Shape result = MakeCompoundFromShapes(componentResults);
    if (!result.IsNull()) {
        ShapeUpgrade_UnifySameDomain unify(result, Standard_True, Standard_True, Standard_True);
        unify.Build();
        if (!unify.Shape().IsNull()) {
            result = unify.Shape();
            v_printf(1, "[STEP Export] UnifySameDomain complete.\n");
        }
    }

    return result;
}

template <typename PolygonT>
static Path PolygonToClipperPathForStep(PolygonT* poly)
{
    Path path;
    if (!poly || !poly->GetLayer()) return path;

    double unitu = poly->GetLayer()->Units->Unitu;
    size_t n = poly->GetPoints();
    path.reserve(n);

    for (size_t i = 0; i < n; i++) {
        path.push_back(IntPoint(
            (cInt)rounded(poly->GetXCoords(i) / unitu),
            (cInt)rounded(poly->GetYCoords(i) / unitu)
        ));
    }

    return path;
}

static TopoDS_Wire MakeWireFromClipperPath(
    const Path& path,
    double unitu,
    double z,
    bool wantPositiveArea)
{
    BRepBuilderAPI_MakePolygon polygonMaker;
    bool reverseOrder = (Orientation(path) != wantPositiveArea);

    if (reverseOrder) {
        for (int i = (int)path.size() - 1; i >= 0; i--) {
            polygonMaker.Add(gp_Pnt(path[i].X * unitu, path[i].Y * unitu, z));
        }
    } else {
        for (size_t i = 0; i < path.size(); i++) {
            polygonMaker.Add(gp_Pnt(path[i].X * unitu, path[i].Y * unitu, z));
        }
    }

    polygonMaker.Close();
    if (!polygonMaker.IsDone()) return TopoDS_Wire();

    return polygonMaker.Wire();
}

static void BuildPrismsFromPolyNode(
    const PolyNode* node,
    double unitu,
    double height,
    double thickness,
    std::vector<TopoDS_Shape>& outShapes)
{
    if (!node || node->Contour.size() < 3) return;

    if (!node->IsHole()) {
        TopoDS_Wire outerWire = MakeWireFromClipperPath(node->Contour, unitu, height, true);
        if (!outerWire.IsNull()) {
            BRepBuilderAPI_MakeFace faceMaker(outerWire);

            int holeCount = 0;
            for (int i = 0; i < node->ChildCount(); i++) {
                PolyNode* child = node->Childs[i];
                if (!child || !child->IsHole() || child->Contour.size() < 3) continue;

                // Inner wires must have the opposite orientation from the outer wire.
                TopoDS_Wire holeWire = MakeWireFromClipperPath(child->Contour, unitu, height, false);
                if (!holeWire.IsNull()) {
                    faceMaker.Add(holeWire);
                    holeCount++;
                }
            }

            if (faceMaker.IsDone()) {
                gp_Vec extrusionVector(0.0, 0.0, thickness);
                BRepPrimAPI_MakePrism prismMaker(faceMaker.Face(), extrusionVector);
                if (prismMaker.IsDone()) {
                    outShapes.push_back(prismMaker.Shape());
                    if (holeCount > 0) {
                        v_printf(1, "[STEP Export] Built prism with %d holes\n", holeCount);
                    }
                }
            }
        }
    }

    // Recurse into children. Hole children may contain island children that are
    // metal again and must be exported as separate outer contours.
    for (int i = 0; i < node->ChildCount(); i++) {
        BuildPrismsFromPolyNode(node->Childs[i], unitu, height, thickness, outShapes);
    }
}

static void AddClipperPathForStep(Paths& input, Path path, bool wantPositiveArea)
{
    if (path.size() < 3 || std::abs(Area(path)) < 0.5) return;

    if (Orientation(path) != wantPositiveArea) {
        ReversePath(path);
    }
    input.push_back(path);
}

static void AddPolygonPathsForStep(GDSPolygon* poly, Paths& input)
{
    if (!poly || poly->GetPoints() < 3) return;
    AddClipperPathForStep(input, PolygonToClipperPathForStep(poly), true);
}

static void AddGeoPolygonPathsRecursive(GeoPolygon* poly, Paths& input)
{
    if (!poly || poly->GetPoints() < 3) return;

    AddClipperPathForStep(input, PolygonToClipperPathForStep(poly), !poly->IsHole());

    std::vector<GeoPolygon*> holes = poly->GetHoles();
    for (size_t i = 0; i < holes.size(); i++) {
        AddGeoPolygonPathsRecursive(holes[i], input);
    }
}

static void AddPolygonPathsForStep(GeoPolygon* poly, Paths& input)
{
    AddGeoPolygonPathsRecursive(poly, input);
}

template <typename PolygonT>
static std::vector<TopoDS_Shape> BuildLayerPrismsWithHoles(
    const std::vector<PolygonT*>& polys,
    const char* logPrefix)
{
    std::vector<TopoDS_Shape> shapes;
    if (polys.empty()) return shapes;

    PolygonT* firstValid = NULL;
    for (size_t i = 0; i < polys.size(); i++) {
        if (polys[i] && polys[i]->GetLayer() && polys[i]->GetPoints() >= 3) {
            firstValid = polys[i];
            break;
        }
    }
    if (!firstValid) return shapes;

    double unitu = firstValid->GetLayer()->Units->Unitu;
    double height = firstValid->GetHeight();
    double thickness = firstValid->GetThickness();

    Paths input;
    input.reserve(polys.size());
    for (size_t i = 0; i < polys.size(); i++) {
        AddPolygonPathsForStep(polys[i], input);
    }

    if (input.empty()) return shapes;

    PolyTree unionTree;
    Clipper c;
    c.AddPaths(input, ptSubject, true);
    c.Execute(ctUnion, unionTree, pftPositive, pftPositive);

    int holeNodes = 0;
    for (PolyNode* node = unionTree.GetFirst(); node; node = node->GetNext()) {
        if (node->IsHole()) holeNodes++;
    }

    v_printf(1, "[STEP Export] %s PolyTree union: %zu input paths, total nodes=%d, holes=%d\n",
             logPrefix ? logPrefix : "Layer", input.size(), unionTree.Total(), holeNodes);

    for (int i = 0; i < unionTree.ChildCount(); i++) {
        BuildPrismsFromPolyNode(unionTree.Childs[i], unitu, height, thickness, shapes);
    }

    v_printf(1, "[STEP Export] %s built %zu prism solids preserving holes\n",
             logPrefix ? logPrefix : "Layer", shapes.size());

    return shapes;
}

bool STEPExport::Export(GDSObject_ogl* obj, const char* filename) {
    return Export(obj, filename, true, false, 0.0);
}

bool STEPExport::Export(GDSObject_ogl* obj, const char* filename, bool includeChildren) {
    return Export(obj, filename, includeChildren, false, 0.0);
}

bool STEPExport::Export(GDSObject_ogl* obj, const char* filename,
                       bool includeChildren, bool enableNarrowEdgeClean,
                       double narrowEdgeDelta) {
    if (!obj) {
        v_printf(1, "[STEP Export] Error: NULL object passed to STEP export\n");
        return false;
    }

    v_printf(1, "\n[STEP Export] Starting optimized GDSII to STEP conversion: %s\n", filename);
    if (enableNarrowEdgeClean && narrowEdgeDelta > 0.0) {
        v_printf(1, "[STEP Export] Using hole-preserving PolyTree union; legacy in-place cleaner is skipped (delta=%.6g)\n",
                 narrowEdgeDelta);
    }

    // Declare outside try block so cleanup in catch can access it
    std::vector<GDSPolygon*> allPolygons;

    try {
        // ============================================================
        // Step 1: Collect ALL polygons from full hierarchy (recursive,
        //         same as original Output::Traverse)
        // ============================================================
        GDSMat identity;
        identity.loadIdentity();
        CollectPolygonsRecursive(obj, identity, allPolygons, includeChildren);

        v_printf(1, "[STEP Export] Collected %zu total polygons (full hierarchy)\n", allPolygons.size());

        if (allPolygons.empty()) {
            v_printf(1, "[STEP Export] Warning: No polygons to export\n");
            for (auto* p : allPolygons) delete p;
            return false;
        }

        // ============================================================
        // Step 2: Group polygons by (height, thickness) - Z layer
        // ============================================================
        std::map<ZLayerKey, std::vector<GDSPolygon*>> layerGroups;

        for (auto* poly : allPolygons) {
            double h = poly->GetHeight();
            double t = poly->GetThickness();
            if (t <= 1e-6) continue;  // Skip degenerate (2D) layers
            size_t np = poly->GetPoints();
            if (np < 3) continue;

            ZLayerKey key = MakeZLayerKey(h, t);
            layerGroups[key].push_back(poly);
        }

        size_t totalLayers = layerGroups.size();
        v_printf(1, "[STEP Export] Grouped into %zu Z-layers\n", totalLayers);
        for (auto& kv : layerGroups) {
            LogZLayerGroupSummary("Group before clean", kv.first, kv.second);
        }

        // ============================================================
        // Step 3: Build OCCT shapes per layer (parallel). Clipper PolyTree
        // keeps outer contours and holes, so empty metal regions stay empty.
        // ============================================================
        // Flatten the map to indexable arrays for OpenMP
        std::vector<ZLayerKey> layerKeys;
        std::vector<std::vector<GDSPolygon*>*> layerPolys;
        layerKeys.reserve(totalLayers);
        layerPolys.reserve(totalLayers);

        for (auto& kv : layerGroups) {
            layerKeys.push_back(kv.first);
            layerPolys.push_back(&kv.second);
        }

        // Result array - each thread writes its layer's individual prism solids
        std::vector<std::vector<TopoDS_Shape>> layerShapeGroups(totalLayers);

        // Atomic counter for progress tracking (parallel layers finish out of order)
        std::atomic<int> completedLayers(0);

        v_printf(1, "[STEP Export] Building geometry:");

        #pragma omp parallel for schedule(dynamic)
        for (int idx = 0; idx < (int)totalLayers; idx++) {
            auto& polys = *layerPolys[idx];

            layerShapeGroups[idx] = BuildLayerPrismsWithHoles(polys, "Direct layer");

            // Update progress bar
            int done = completedLayers.fetch_add(1) + 1;
            #pragma omp critical
            {
                print_progress("[STEP Export]", done, (int)totalLayers);
            }
        }

        print_progress_end();
        v_printf(1, "[STEP Export] All %zu layers built.\n", totalLayers);

        // ============================================================
        // Step 4: Fuse solids within each Z-layer only. Adjacent metal/via
        // layers must remain separate process/material bodies.
        // ============================================================
        std::vector<TopoDS_Shape> fusedLayerShapes;
        fusedLayerShapes.reserve(totalLayers);

        for (size_t i = 0; i < totalLayers; i++) {
            if (layerShapeGroups[i].empty()) continue;

            LogZLayerGroupSummary("Fuse Z-layer", layerKeys[i], *layerPolys[i]);
            v_printf(1, "[STEP Export] Fusing Z-layer %zu/%zu (%zu solids)\n",
                     i + 1, totalLayers, layerShapeGroups[i].size());
            TopoDS_Shape fusedLayer = FuseShapesOptimized(layerShapeGroups[i]);
            if (!fusedLayer.IsNull()) {
                fusedLayerShapes.push_back(fusedLayer);
            }
        }

        v_printf(1, "[STEP Export] Combining %zu fused Z-layers without cross-layer fuse.\n",
                 fusedLayerShapes.size());
        TopoDS_Shape mainAssembly = MakeCompoundFromShapes(fusedLayerShapes);
        if (mainAssembly.IsNull()) {
            v_printf(1, "[STEP Export] Error: no valid shape after layer fuse.\n");
            for (auto* p : allPolygons) delete p;
            return false;
        }

        v_printf(1, "[STEP Export] Writing STEP file...");
        STEPControl_Writer stepWriter;
        stepWriter.Transfer(mainAssembly, STEPControl_AsIs);

        if (stepWriter.Write(filename) == IFSelect_RetDone) {
            v_printf(1, "[STEP Export] Success! File saved as: %s\n", filename);
            // Cleanup allocated polygon copies
            for (auto* p : allPolygons) delete p;
            return true;
        } else {
            v_printf(1, "[STEP Export] Error: Failed to write file!\n");
            for (auto* p : allPolygons) delete p;
            return false;
        }
    }
    catch (...) {
        v_printf(1, "[STEP Export] Critical Exception: OCCT Kernel crashed during geometric processing!\n");
        for (auto* p : allPolygons) delete p;
        return false;
    }
}

// --- Export using merged polygons from Output::GetFullGDSItems ---
// This uses the already-merged GeoPolygon* from SimplifyPolyItems_wClipper
bool STEPExport::Export(Output* output, const char* filename,
                       bool enableNarrowEdgeClean, double narrowEdgeDelta) {
    if (!output) {
        v_printf(1, "[STEP Export] Error: NULL Output passed to STEP export\n");
        return false;
    }

    v_printf(1, "\n[STEP Export] Using merged polygons from Output::GetFullGDSItems\n");
    if (enableNarrowEdgeClean && narrowEdgeDelta > 0.0) {
        v_printf(1, "[STEP Export] Using hole-preserving PolyTree union; legacy in-place cleaner is skipped (delta=%.6g)\n",
                 narrowEdgeDelta);
    }

    std::vector<GDSGroup*>& fullGDSItems = output->GetFullGDSItems();
    v_printf(1, "[STEP Export] FullGDSItems has %zu GDS groups\n", fullGDSItems.size());

    // Collect all GeoPolygon* from all GDS groups
    std::vector<GeoPolygon*> allGeoPolygons;
    for (size_t g = 0; g < fullGDSItems.size(); g++) {
        GDSGroup* group = fullGDSItems[g];
        for (size_t p = 0; p < group->FullPolygonItems.size(); p++) {
            allGeoPolygons.push_back(group->FullPolygonItems[p]);
        }
    }
    v_printf(1, "[STEP Export] Total merged GeoPolygons: %zu\n", allGeoPolygons.size());

    if (allGeoPolygons.empty()) {
        v_printf(1, "[STEP Export] Warning: No polygons to export\n");
        return false;
    }

    // Group by Z-layer (height, thickness)
    std::map<ZLayerKey, std::vector<GeoPolygon*>> layerGroups;
    for (auto* poly : allGeoPolygons) {
        double h = poly->GetHeight();
        double t = poly->GetThickness();
        if (t <= 1e-6) continue;
        if (poly->GetPoints() < 3) continue;
        ZLayerKey key = MakeZLayerKey(h, t);
        layerGroups[key].push_back(poly);
    }

    size_t totalLayers = layerGroups.size();
    v_printf(1, "[STEP Export] Grouped into %zu Z-layers\n", totalLayers);
    for (auto& kv : layerGroups) {
        LogZLayerGroupSummary("Merged group", kv.first, kv.second);
    }

    // Flatten for OpenMP
    std::vector<ZLayerKey> layerKeys;
    std::vector<std::vector<GeoPolygon*>*> layerPolys;
    layerKeys.reserve(totalLayers);
    layerPolys.reserve(totalLayers);
    for (auto& kv : layerGroups) {
        layerKeys.push_back(kv.first);
        layerPolys.push_back(&kv.second);
    }

    std::vector<std::vector<TopoDS_Shape>> layerShapeGroups(totalLayers);
    std::atomic<int> completedLayers(0);

    v_printf(1, "[STEP Export] Building geometry from merged polygons:");

    #pragma omp parallel for schedule(dynamic)
    for (int idx = 0; idx < (int)totalLayers; idx++) {
        auto& polys = *layerPolys[idx];

        layerShapeGroups[idx] = BuildLayerPrismsWithHoles(polys, "Merged layer");

        int done = completedLayers.fetch_add(1) + 1;
        #pragma omp critical
        {
            print_progress("[STEP Export]", done, (int)totalLayers);
        }
    }

    print_progress_end();
    v_printf(1, "[STEP Export] All %zu layers built.\n", totalLayers);

    // Fuse solids within each Z-layer only, then combine layers as a compound.
    std::vector<TopoDS_Shape> fusedLayerShapes;
    fusedLayerShapes.reserve(totalLayers);

    for (size_t i = 0; i < totalLayers; i++) {
        if (layerShapeGroups[i].empty()) continue;

        LogZLayerGroupSummary("Fuse merged Z-layer", layerKeys[i], *layerPolys[i]);
        v_printf(1, "[STEP Export] Fusing Z-layer %zu/%zu (%zu solids)\n",
                 i + 1, totalLayers, layerShapeGroups[i].size());
        TopoDS_Shape fusedLayer = FuseShapesOptimized(layerShapeGroups[i]);
        if (!fusedLayer.IsNull()) {
            fusedLayerShapes.push_back(fusedLayer);
        }
    }

    v_printf(1, "[STEP Export] Combining %zu fused Z-layers without cross-layer fuse.\n",
             fusedLayerShapes.size());
    TopoDS_Shape mainAssembly = MakeCompoundFromShapes(fusedLayerShapes);
    if (mainAssembly.IsNull()) {
        v_printf(1, "[STEP Export] Error: no valid shape after layer fuse.\n");
        return false;
    }

    v_printf(1, "[STEP Export] Writing STEP file...");

    STEPControl_Writer stepWriter;
    stepWriter.Transfer(mainAssembly, STEPControl_AsIs);

    if (stepWriter.Write(filename) == IFSelect_RetDone) {
        v_printf(1, "[STEP Export] Success! File saved as: %s\n", filename);
        return true;
    } else {
        v_printf(1, "[STEP Export] Error: Failed to write file!\n");
        return false;
    }
}
