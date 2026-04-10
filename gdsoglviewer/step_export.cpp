/*
 * @Author: ka1shu1 cwh979946@163.com
 * @Date: 2026-04-10 20:34:32
 * @LastEditors: ka1shu1 cwh979946@163.com
 * @LastEditTime: 2026-04-10 21:26:43
 * @FilePath: \AICAD_Research\references\GDS3D\gdsoglviewer\step_export.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "step_export.h"
#include "gdsobject_ogl.h"
#include "gds_globals.h" // 提供 v_printf 支持

// 引入 GDS3D 中多边形的头文件 (根据你的实际项目结构，如果不叫这个名字请微调)
#include "gdspolygon.h" 

// --- OpenCASCADE 核心几何与拓扑头文件 ---
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <STEPControl_Writer.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <ShapeFix_Face.hxx>

bool STEPExport::Export(GDSObject_ogl* obj, const char* filename) {
    return Export(obj, filename, true);
}

bool STEPExport::Export(GDSObject_ogl* obj, const char* filename, bool includeChildren) {
    if (!obj) {
        v_printf(1, "[STEP Export] Error: NULL object passed to STEP export\n");
        return false;
    }

    v_printf(1, "\n[STEP Export] Starting GDSII to STEP conversion: %s\n", filename);

    try {
        // 1. 创建主装配体 (Compound)，用于容纳所有的 3D 实体
        TopoDS_Compound mainAssembly;
        BRep_Builder compoundBuilder;
        compoundBuilder.MakeCompound(mainAssembly);

        int exported_polygons = 0;

        // 2. 定义处理单个 GDSObject 的 Lambda 函数
        auto processObjectPolygons = [&](GDSObject_ogl* object) {
            for (size_t i = 0; i < object->PolygonItems.size(); i++) {
                class GDSPolygon* polygon = object->PolygonItems[i];
                if (!polygon) continue;

                double height = polygon->GetHeight();
                double thickness = polygon->GetThickness();

                // 如果厚度太小，跳过拉伸以防止生成退化实体
                if (thickness <= 1e-6) continue;

                size_t numPoints = polygon->GetPoints();
                if (numPoints < 3) continue;

                // --- 开始 OCCT 建模 ---
                
                // A. 依次添加顶点，构建 2D 轮廓 (Wire)
                BRepBuilderAPI_MakePolygon polygonMaker;
                for (size_t j = 0; j < numPoints; j++) {
                    polygonMaker.Add(gp_Pnt(polygon->GetXCoords(j), polygon->GetYCoords(j), height));
                }
                polygonMaker.Close(); // 闭合轮廓

                // 如果多边形有自交叉等不合法情况，跳过
                if (!polygonMaker.IsDone()) continue; 

                // B. 将轮廓转换为 2D 表面 (Face)
                BRepBuilderAPI_MakeFace faceMaker(polygonMaker.Wire());
                if (!faceMaker.IsDone()) continue;

                TopoDS_Face bottomFace = faceMaker.Face();

                // C. 沿着 Z 轴进行拉伸 (Extrude)，生成 3D 实体 (Solid Prism)
                gp_Vec extrusionVector(0.0, 0.0, thickness);
                BRepPrimAPI_MakePrism prismMaker(bottomFace, extrusionVector);
                
                if (!prismMaker.IsDone()) continue;

                // D. 将生成的 3D 实体加入到主装配体中
                compoundBuilder.Add(mainAssembly, prismMaker.Shape());
                exported_polygons++;
            }
        };

        // 3. 收集主对象的实体
        processObjectPolygons(obj);

        // 4. 根据需求，递归收集子引用 (Refs) 的实体
        if (includeChildren) {
            for (size_t i = 0; i < obj->refs.size(); i++) {
                GDSObject* ref = obj->refs[i]->object;
                if (ref) {
                    processObjectPolygons((GDSObject_ogl*)ref);
                }
            }
        }

        v_printf(1, "[STEP Export] Successfully extruded %d polygons. Writing to file...\n", exported_polygons);

        // 5. 调用 OCCT 的写入器，生成最终的 STEP 文件
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
        // OCCT 内核报错拦截，防止整个 GDS3D 程序崩溃
        v_printf(1, "[STEP Export] Critical Exception: OCCT Kernel crashed during geometric processing!\n");
        return false;
    }
}