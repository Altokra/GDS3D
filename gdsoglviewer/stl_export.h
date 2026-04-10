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

#ifndef __STL_EXPORT_H__
#define __STL_EXPORT_H__

#include <vector>
#include <string>

// Forward declaration
class GDSObject_ogl;

/**
 * @brief STL Export Module
 *
 * Exports GDSObject_ogl geometry to binary STL format.
 * Each polygon is extruded to 3D by creating:
 *   - Top face (at Height + Thickness)
 *   - Bottom face (at Height)
 *   - Side faces (connecting top and bottom)
 */
class STLExport {
public:
    /**
     * @brief Export GDSObject_ogl to binary STL file
     * @param obj The GDS object to export
     * @param filename Output STL filename
     * @return true if export successful
     */
    static bool Export(GDSObject_ogl* obj, const char* filename);

    /**
     * @brief Export GDSObject_ogl to binary STL with all children
     * @param obj The GDS object to export
     * @param filename Output STL filename
     * @param includeChildren Whether to recursively export child refs
     * @return true if export successful
     */
    static bool Export(GDSObject_ogl* obj, const char* filename, bool includeChildren);

private:
    // Triangle structure for STL
    struct Triangle {
        float normal[3];
        float v1[3];
        float v2[3];
        float v3[3];
        unsigned short attribute; // usually 0
    };

    // Collect triangles from a single polygon
    static void AddPolygonTriangles(
        double height,
        double thickness,
        const std::vector<double>& xCoords,
        const std::vector<double>& yCoords,
        const std::vector<size_t>& indices,
        std::vector<Triangle>& triangles
    );

    // Compute face normal
    static void ComputeNormal(const float v1[3], const float v2[3], const float v3[3], float normal[3]);

    // Write binary STL file
    static bool WriteBinarySTL(const char* filename, const std::vector<Triangle>& triangles);
};

#endif // __STL_EXPORT_H__