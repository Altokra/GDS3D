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
#include "polygon_cleaner.h"

// --- OpenCASCADE core headers ---
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <STEPControl_Writer.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>

#include <map>
#include <vector>
#include <utility>
#include <atomic>
#include <cmath>

// Progress bar width (number of '#' characters)
#define PROGRESS_BAR_WIDTH 30

#ifdef WIN32
#include <windows.h>
#endif

// Z-layer grouping key: polygons with same (height, thickness) belong to same physical layer
struct ZLayerKey {
    double height;
    double thickness;

    bool operator<(const ZLayerKey& o) const {
        // Use tolerance comparison to avoid floating-point precision issues
        // e.g. 0.139999999 vs 0.14 should be treated as the same layer
        const double EPS = 1e-9;
        if (std::abs(height - o.height) > EPS) return height < o.height;
        return thickness < o.thickness;
    }
};

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

    try {
        // ============================================================
        // Step 1: Collect ALL polygons into a flat list
        // ============================================================
        std::vector<GDSPolygon*> allPolygons;

        // Main object polygons
        for (size_t i = 0; i < obj->PolygonItems.size(); i++) {
            if (obj->PolygonItems[i]) {
                allPolygons.push_back(obj->PolygonItems[i]);
            }
        }

        // Child reference polygons
        if (includeChildren) {
            for (size_t i = 0; i < obj->refs.size(); i++) {
                GDSObject* ref = obj->refs[i]->object;
                if (ref) {
                    GDSObject_ogl* refOgl = (GDSObject_ogl*)ref;
                    for (size_t j = 0; j < refOgl->PolygonItems.size(); j++) {
                        if (refOgl->PolygonItems[j]) {
                            allPolygons.push_back(refOgl->PolygonItems[j]);
                        }
                    }
                }
            }
        }

        v_printf(1, "[STEP Export] Collected %zu total polygons\n", allPolygons.size());

        if (allPolygons.empty()) {
            v_printf(1, "[STEP Export] Warning: No polygons to export\n");
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

            ZLayerKey key = {h, t};
            layerGroups[key].push_back(poly);
        }

        size_t totalLayers = layerGroups.size();
        v_printf(1, "[STEP Export] Grouped into %zu Z-layers\n", totalLayers);

        // ============================================================
        // Step 2.5: Clean narrow edges per layer (optional)
        // ============================================================
        if (enableNarrowEdgeClean && narrowEdgeDelta > 0.0) {
            v_printf(1, "[STEP Export] Cleaning narrow edges (delta=%.1f)...\n", narrowEdgeDelta);
            for (auto& kv : layerGroups) {
                PolygonCleaner::CleanPolygonsInPlace(kv.second, narrowEdgeDelta);
            }
            v_printf(1, "[STEP Export] Narrow edge cleaning complete.\n");
        }

        // ============================================================
        // Step 3: Build OCCT shapes per layer (parallel)
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

        // Result array - each thread writes to its own index
        std::vector<TopoDS_Shape> layerShapes(totalLayers);

        // Atomic counter for progress tracking (parallel layers finish out of order)
        std::atomic<int> completedLayers(0);

        v_printf(1, "[STEP Export] Building geometry:");

        #pragma omp parallel for schedule(dynamic)
        for (int idx = 0; idx < (int)totalLayers; idx++) {
            auto& polys = *layerPolys[idx];
            size_t polysInLayer = polys.size();

            BRep_Builder b;
            TopoDS_Compound layerCompound;
            b.MakeCompound(layerCompound);

            int exported_in_layer = 0;

            for (size_t p = 0; p < polysInLayer; p++) {
                GDSPolygon* polygon = polys[p];
                size_t numPoints = polygon->GetPoints();
                if (numPoints < 3) continue;

                double height = polygon->GetHeight();
                double thickness = polygon->GetThickness();

                // Build 2D wire from polygon vertices
                BRepBuilderAPI_MakePolygon polygonMaker;
                for (size_t j = 0; j < numPoints; j++) {
                    polygonMaker.Add(gp_Pnt(
                        polygon->GetXCoords(j),
                        polygon->GetYCoords(j),
                        height
                    ));
                }
                polygonMaker.Close();

                if (!polygonMaker.IsDone()) continue;

                // Build face from wire
                BRepBuilderAPI_MakeFace faceMaker(polygonMaker.Wire());
                if (!faceMaker.IsDone()) continue;

                // Extrude to 3D solid
                gp_Vec extrusionVector(0.0, 0.0, thickness);
                BRepPrimAPI_MakePrism prismMaker(faceMaker.Face(), extrusionVector);
                if (!prismMaker.IsDone()) continue;

                // Add to layer compound
                b.Add(layerCompound, prismMaker.Shape());
                exported_in_layer++;
            }

            layerShapes[idx] = layerCompound;

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
        // Step 4: Merge all layers into ONE compound, then single Transfer
        //         (same file structure as original, avoids size change)
        // ============================================================
        v_printf(1, "[STEP Export] Writing STEP file...");

        TopoDS_Compound mainAssembly;
        BRep_Builder compoundBuilder;
        compoundBuilder.MakeCompound(mainAssembly);

        for (size_t i = 0; i < totalLayers; i++) {
            compoundBuilder.Add(mainAssembly, layerShapes[i]);
            print_progress("[STEP Export]", (int)(i + 1), (int)totalLayers);
        }

        print_progress_end();

        // Single Transfer call (preserves original file structure)
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
    catch (...) {
        v_printf(1, "[STEP Export] Critical Exception: OCCT Kernel crashed during geometric processing!\n");
        return false;
    }
}
