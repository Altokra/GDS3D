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
#include <cmath>
#include <cstdio>

bool STLExport::Export(GDSObject_ogl* obj, const char* filename) {
    return Export(obj, filename, true);
}

bool STLExport::Export(GDSObject_ogl* obj, const char* filename, bool includeChildren) {
    if (!obj) {
        v_printf(1, "Error: NULL object passed to STL export\n");
        return false;
    }

    std::vector<Triangle> triangles;

    // Helper lambda to collect triangles from one object
    auto collectTriangles = [&](GDSObject_ogl* object, std::vector<Triangle>& tris) {
        for (size_t i = 0; i < object->PolygonItems.size(); i++) {
            class GDSPolygon* polygon = object->PolygonItems[i];
            if (!polygon) continue;

            double height = polygon->GetHeight();
            double thickness = polygon->GetThickness();

            std::vector<double> xCoords;
            std::vector<double> yCoords;
            for (size_t j = 0; j < polygon->GetPoints(); j++) {
                xCoords.push_back(polygon->GetXCoords(j));
                yCoords.push_back(polygon->GetYCoords(j));
            }

            std::vector<size_t>* indices = polygon->GetIndices();
            if (!indices || indices->empty()) {
                polygon->Tesselate();
                indices = polygon->GetIndices();
            }

            if (indices) {
                AddPolygonTriangles(height, thickness, xCoords, yCoords, *indices, tris);
            }
        }
    };

    // Collect from main object
    collectTriangles(obj, triangles);

    // Recursively collect from children if requested
    if (includeChildren) {
        for (size_t i = 0; i < obj->refs.size(); i++) {
            GDSObject* ref = obj->refs[i]->object;
            if (ref) {
                collectTriangles((GDSObject_ogl*)ref, triangles);
            }
        }
    }

    v_printf(1, "STL Export: collected %d triangles from object %s\n", (int)triangles.size(), obj->Name);

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
    double z1 = height;
    double z2 = height + thickness;
    size_t numPoints = xCoords.size();

    if (numPoints < 3 || indices.empty()) return;

    Triangle tri;
    tri.attribute = 0;

    // Top and bottom faces (using even-odd rule for winding)
    int e = 1;  // even
    int o = 1;  // odd

    // Determine winding order
    for (size_t j = 0; j < indices.size() / 3; j++) {
        size_t v0 = indices[j * 3 + 0];
        size_t v1 = indices[j * 3 + 1];
        size_t v2 = indices[j * 3 + 2];

        if ((v0 % 2) == 0 && (v1 % 2) == 0 && (v2 % 2) == 0)
            e = 0;
        if ((v0 % 2) == 1 && (v1 % 2) == 1 && (v2 % 2) == 1)
            o = 0;
    }

    // Top face
    for (size_t j = 0; j < indices.size() / 3; j++) {
        size_t v0 = indices[j * 3 + 0];
        size_t v1 = indices[j * 3 + 1];
        size_t v2 = indices[j * 3 + 2];

        float vp0[3] = {(float)xCoords[v0], (float)yCoords[v0], (float)z2};
        float vp1[3] = {(float)xCoords[v1], (float)yCoords[v1], (float)z2};
        float vp2[3] = {(float)xCoords[v2], (float)yCoords[v2], (float)z2};

        if ((e && v1 % 2 == 0) || (o && v1 % 2 == 1)) {
            tri.v1[0] = vp0[0]; tri.v1[1] = vp0[1]; tri.v1[2] = vp0[2];
            tri.v2[0] = vp1[0]; tri.v2[1] = vp1[1]; tri.v2[2] = vp1[2];
            tri.v3[0] = vp2[0]; tri.v3[1] = vp2[1]; tri.v3[2] = vp2[2];
        } else if ((e && v2 % 2 == 0) || (o && v2 % 2 == 1)) {
            tri.v1[0] = vp1[0]; tri.v1[1] = vp1[1]; tri.v1[2] = vp1[2];
            tri.v2[0] = vp2[0]; tri.v2[1] = vp2[1]; tri.v2[2] = vp2[2];
            tri.v3[0] = vp0[0]; tri.v3[1] = vp0[1]; tri.v3[2] = vp0[2];
        } else {
            tri.v1[0] = vp2[0]; tri.v1[1] = vp2[1]; tri.v1[2] = vp2[2];
            tri.v2[0] = vp0[0]; tri.v2[1] = vp0[1]; tri.v2[2] = vp0[2];
            tri.v3[0] = vp1[0]; tri.v3[1] = vp1[1]; tri.v3[2] = vp1[2];
        }

        ComputeNormal(tri.v1, tri.v2, tri.v3, tri.normal);
        triangles.push_back(tri);
    }

    // Bottom face (reverse winding for correct normal)
    for (size_t j = 0; j < indices.size() / 3; j++) {
        size_t v0 = indices[j * 3 + 0];
        size_t v1 = indices[j * 3 + 1];
        size_t v2 = indices[j * 3 + 2];

        float vp0[3] = {(float)xCoords[v0], (float)yCoords[v0], (float)z1};
        float vp1[3] = {(float)xCoords[v1], (float)yCoords[v1], (float)z1};
        float vp2[3] = {(float)xCoords[v2], (float)yCoords[v2], (float)z1};

        // Reverse order for bottom face (normal should point down)
        if ((e && v1 % 2 == 0) || (o && v1 % 2 == 1)) {
            tri.v1[0] = vp0[0]; tri.v1[1] = vp0[1]; tri.v1[2] = vp0[2];
            tri.v2[0] = vp1[0]; tri.v2[1] = vp1[1]; tri.v2[2] = vp1[2];
            tri.v3[0] = vp2[0]; tri.v3[1] = vp2[1]; tri.v3[2] = vp2[2];
        } else if ((e && v2 % 2 == 0) || (o && v2 % 2 == 1)) {
            tri.v1[0] = vp2[0]; tri.v1[1] = vp2[1]; tri.v1[2] = vp2[2];
            tri.v2[0] = vp1[0]; tri.v2[1] = vp1[1]; tri.v2[2] = vp1[2];
            tri.v3[0] = vp0[0]; tri.v3[1] = vp0[1]; tri.v3[2] = vp0[2];
        } else {
            tri.v1[0] = vp1[0]; tri.v1[1] = vp1[1]; tri.v1[2] = vp1[2];
            tri.v2[0] = vp0[0]; tri.v2[1] = vp0[1]; tri.v2[2] = vp0[2];
            tri.v3[0] = vp2[0]; tri.v3[1] = vp2[1]; tri.v3[2] = vp2[2];
        }

        ComputeNormal(tri.v1, tri.v2, tri.v3, tri.normal);
        triangles.push_back(tri);
    }

    // Side faces
    for (size_t j = 0; j < numPoints; j++) {
        size_t j1 = (j + 1) % numPoints;

        float vp0[3] = {(float)xCoords[j], (float)yCoords[j], (float)z1};
        float vp1[3] = {(float)xCoords[j1], (float)yCoords[j1], (float)z1};
        float vp2[3] = {(float)xCoords[j1], (float)yCoords[j1], (float)z2};
        float vp3[3] = {(float)xCoords[j], (float)yCoords[j], (float)z2};

        // Two triangles per side quad
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