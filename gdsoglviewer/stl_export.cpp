//  GDS3D, a program for viewing GDSII files in 3D.
//  Created by Jasper Velner and Michiel Soer, IC-Design Group, University of Twente: http://icd.el.utwente.nl
//
//  Copyright (C) 2024 Contributors
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this program; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

#include "stl_export.h"
#include "gdsobject_ogl.h"
#include "gdsobject.h"
#include "gds_globals.h"
#include "polygon_cleaner.h"
#include <cmath>
#include <cstdio>
#include <map>

bool STLExport::Export(GDSObject_ogl* obj, const char* filename) {
    return Export(obj, filename, true, false, 0.0);
}

bool STLExport::Export(GDSObject_ogl* obj, const char* filename, bool includeChildren) {
    return Export(obj, filename, includeChildren, false, 0.0);
}

bool STLExport::Export(GDSObject_ogl* obj, const char* filename,
                       bool includeChildren, bool enableNarrowEdgeClean,
                       double narrowEdgeDelta) {
    if (!obj) {
        v_printf(1, "Error: NULL object passed to STL export\n");
        return false;
    }

    // ============================================================
    // Step 1: Collect all polygons into a flat list
    // ============================================================
    std::vector<GDSPolygon*> allPolygons;

    for (size_t i = 0; i < obj->PolygonItems.size(); i++) {
        if (obj->PolygonItems[i]) {
            allPolygons.push_back(obj->PolygonItems[i]);
        }
    }

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

    v_printf(1, "STL Export: collected %zu polygons\n", allPolygons.size());

    // ============================================================
    // Step 2: Clean narrow edges per Z-layer (optional, non-destructive)
    // ============================================================
    // Maps polygon pointer -> cleaned (xCoords, yCoords).
    // If empty, use original polygon data.
    std::map<GDSPolygon*, std::pair<std::vector<double>, std::vector<double>>> cleanedCoords;

    if (enableNarrowEdgeClean && narrowEdgeDelta > 0.0) {
        // Z-layer grouping key
        struct ZLayerKey {
            double height, thickness;
            bool operator<(const ZLayerKey& o) const {
                if (height != o.height) return height < o.height;
                return thickness < o.thickness;
            }
        };

        std::map<ZLayerKey, std::vector<GDSPolygon*>> layerGroups;
        for (auto* poly : allPolygons) {
            double h = poly->GetHeight();
            double t = poly->GetThickness();
            if (t <= 1e-6 || poly->GetPoints() < 3) continue;
            ZLayerKey key = {h, t};
            layerGroups[key].push_back(poly);
        }

        v_printf(1, "[STL Export] Cleaning narrow edges (delta=%.1f) across %zu layers...\n",
                 narrowEdgeDelta, layerGroups.size());
        for (auto& kv : layerGroups) {
            PolygonCleaner::CleanPolygonsToCoords(kv.second, narrowEdgeDelta, cleanedCoords);
        }
        v_printf(1, "[STL Export] Narrow edge cleaning complete.\n");
    }

    // ============================================================
    // Step 3: Generate triangles from (cleaned) polygons
    // ============================================================
    std::vector<Triangle> triangles;

    for (auto* polygon : allPolygons) {
        if (!polygon) continue;

        double height = polygon->GetHeight();
        double thickness = polygon->GetThickness();
        if (thickness <= 1e-6) continue;

        std::vector<double> xCoords, yCoords;

        // Use cleaned coordinates if available, otherwise use original
        auto it = cleanedCoords.find(polygon);
        if (it != cleanedCoords.end()) {
            xCoords = it->second.first;
            yCoords = it->second.second;
        } else {
            for (size_t j = 0; j < polygon->GetPoints(); j++) {
                xCoords.push_back(polygon->GetXCoords(j));
                yCoords.push_back(polygon->GetYCoords(j));
            }
        }

        size_t numPoints = xCoords.size();
        if (numPoints < 3) continue;

        // Tessellate cleaned polygon using a temporary GDSPolygon (ear-clipping)
        std::vector<size_t> triIndices;
        if (it != cleanedCoords.end()) {
            GDSPolygon tempPoly(height, thickness, polygon->GetLayer());
            for (size_t j = 0; j < numPoints; j++) {
                tempPoly.AddPoint(xCoords[j], yCoords[j]);
            }
            tempPoly.Tesselate();
            std::vector<size_t>* tempIndices = tempPoly.GetIndices();
            if (tempIndices) triIndices = *tempIndices;
        } else {
            std::vector<size_t>* indices = polygon->GetIndices();
            if (!indices || indices->empty()) {
                polygon->Tesselate();
                indices = polygon->GetIndices();
            }
            if (indices) {
                triIndices = *indices;
            }
        }

        if (!triIndices.empty()) {
            AddPolygonTriangles(height, thickness, xCoords, yCoords, triIndices, triangles);
        }
    }

    v_printf(1, "STL Export: generated %d triangles from object %s\n", (int)triangles.size(), obj->GetName());

    return WriteBinarySTL(filename, triangles);
}

void STLExport::AddPolygonTriangles(
    double height,
    double thickness,
    const std::vector<double>& xCoords,
    const std::vector<double>& yCoords,
    const std::vector<size_t>& indices,
    std::vector<Triangle>& triangles)
{
    // 如果厚度太小（例如2D标记层），跳过不导出，防止生成退化三角形
    if (thickness <= 1e-6) return;

    double z1 = height;               // 底部 Z
    double z2 = height + thickness;   // 顶部 Z
    size_t numPoints = xCoords.size();

    if (numPoints < 3 || indices.empty()) return;

    Triangle tri;
    tri.attribute = 0;

    // --- 1. 处理顶面和底面 ---
    for (size_t j = 0; j < indices.size() / 3; j++) {
        size_t v0 = indices[j * 3 + 0];
        size_t v1 = indices[j * 3 + 1];
        size_t v2 = indices[j * 3 + 2];

        // 顶面 (z2)
        tri.v1[0] = xCoords[v0]; tri.v1[1] = yCoords[v0]; tri.v1[2] = z2;
        tri.v2[0] = xCoords[v1]; tri.v2[1] = yCoords[v1]; tri.v2[2] = z2;
        tri.v3[0] = xCoords[v2]; tri.v3[1] = yCoords[v2]; tri.v3[2] = z2;
        ComputeNormal(tri.v1, tri.v2, tri.v3, tri.normal);
        
        // 强制法线朝上 (+Z)
        if (tri.normal[2] < 0) {
            std::swap(tri.v2[0], tri.v3[0]);
            std::swap(tri.v2[1], tri.v3[1]);
            // 重新计算正确的法线
            ComputeNormal(tri.v1, tri.v2, tri.v3, tri.normal);
        }
        triangles.push_back(tri);

        // 底面 (z1)
        tri.v1[0] = xCoords[v0]; tri.v1[1] = yCoords[v0]; tri.v1[2] = z1;
        tri.v2[0] = xCoords[v1]; tri.v2[1] = yCoords[v1]; tri.v2[2] = z1;
        tri.v3[0] = xCoords[v2]; tri.v3[1] = yCoords[v2]; tri.v3[2] = z1;
        ComputeNormal(tri.v1, tri.v2, tri.v3, tri.normal);
        
        // 强制法线朝下 (-Z)
        if (tri.normal[2] > 0) {
            std::swap(tri.v2[0], tri.v3[0]);
            std::swap(tri.v2[1], tri.v3[1]);
            ComputeNormal(tri.v1, tri.v2, tri.v3, tri.normal);
        }
        triangles.push_back(tri);
    }

    // --- 2. 判断 2D 多边形顶点的缠绕方向 (Shoelace formula) ---
    double area = 0.0;
    for (size_t j = 0; j < numPoints; j++) {
        size_t j1 = (j + 1) % numPoints;
        area += (xCoords[j] * yCoords[j1] - xCoords[j1] * yCoords[j]);
    }
    bool isCCW = (area > 0); // 判断是否为逆时针

    // --- 3. 处理侧面 ---
    for (size_t j = 0; j < numPoints; j++) {
        size_t j1 = (j + 1) % numPoints;

        float vp0[3] = {(float)xCoords[j],  (float)yCoords[j],  (float)z1}; // 当前点底
        float vp1[3] = {(float)xCoords[j1], (float)yCoords[j1], (float)z1}; // 下一点底
        float vp2[3] = {(float)xCoords[j1], (float)yCoords[j1], (float)z2}; // 下一点顶
        float vp3[3] = {(float)xCoords[j],  (float)yCoords[j],  (float)z2}; // 当前点顶

        // 如果多边形本身是顺时针，需要翻转四边形的左右点，以保证法线朝外
        if (!isCCW) {
            std::swap(vp0[0], vp1[0]); std::swap(vp0[1], vp1[1]); // 交换底部左右
            std::swap(vp3[0], vp2[0]); std::swap(vp3[1], vp2[1]); // 交换顶部左右
        }

        // Triangle 1: vp0, vp1, vp2
        tri.v1[0] = vp0[0]; tri.v1[1] = vp0[1]; tri.v1[2] = vp0[2];
        tri.v2[0] = vp1[0]; tri.v2[1] = vp1[1]; tri.v2[2] = vp1[2];
        tri.v3[0] = vp2[0]; tri.v3[1] = vp2[1]; tri.v3[2] = vp2[2];
        ComputeNormal(tri.v1, tri.v2, tri.v3, tri.normal);
        triangles.push_back(tri);

        // Triangle 2: vp0, vp2, vp3
        tri.v1[0] = vp0[0]; tri.v1[1] = vp0[1]; tri.v1[2] = vp0[2];
        tri.v2[0] = vp2[0]; tri.v2[1] = vp2[1]; tri.v2[2] = vp2[2];
        tri.v3[0] = vp3[0]; tri.v3[1] = vp3[1]; tri.v3[2] = vp3[2];
        ComputeNormal(tri.v1, tri.v2, tri.v3, tri.normal);
        triangles.push_back(tri);
    }
}


void STLExport::ComputeNormal(const float v1[3], const float v2[3], const float v3[3], float normal[3]) {
    float edge1[3] = {v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2]};
    float edge2[3] = {v3[0] - v1[0], v3[1] - v1[1], v3[2] - v1[2]};

    // Cross product
    normal[0] = edge1[1] * edge2[2] - edge1[2] * edge2[1];
    normal[1] = edge1[2] * edge2[0] - edge1[0] * edge2[2];
    normal[2] = edge1[0] * edge2[1] - edge1[1] * edge2[0];

    // Normalize
    float len = sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
    if (len > 0.0001f) {
        normal[0] /= len;
        normal[1] /= len;
        normal[2] /= len;
    } else {
        normal[0] = 0; normal[1] = 0; normal[2] = 1;
    }
}

bool STLExport::WriteBinarySTL(const char* filename, const std::vector<Triangle>& triangles) {
    FILE* fp = fopen(filename, "wb");
    if (!fp) {
        v_printf(1, "Error: Cannot open file %s for writing\n", filename);
        return false;
    }

    // Write 80-byte header
    char header[80] = "GDS3D STL Export - GDSObject";
    fwrite(header, 1, 80, fp);

    // Write triangle count
    unsigned int numTriangles = (unsigned int)triangles.size();
    fwrite(&numTriangles, 4, 1, fp);

    // Write each triangle
    for (size_t i = 0; i < triangles.size(); i++) {
        const Triangle& t = triangles[i];
        fwrite(t.normal, 4, 3, fp);      // 12 bytes
        fwrite(t.v1, 4, 3, fp);         // 12 bytes
        fwrite(t.v2, 4, 3, fp);         // 12 bytes
        fwrite(t.v3, 4, 3, fp);         // 12 bytes
        fwrite(&t.attribute, 2, 1, fp); // 2 bytes
    }

    fclose(fp);
    v_printf(1, "STL file written: %s (%d triangles)\n", filename, numTriangles);
    return true;
}