/*
 * @Author: ka1shu1 cwh979946@163.com
 * @Date: 2026-04-10 20:34:00
 * @LastEditors: ka1shu1 cwh979946@163.com
 * @LastEditTime: 2026-04-10 21:12:03
 * @FilePath: \AICAD_Research\references\GDS3D\gdsoglviewer\step_export.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */



 
#ifndef __STEP_EXPORT_H__
#define __STEP_EXPORT_H__

class GDSObject_ogl;
class Output;

class STEPExport {
public:
    // 导出 GDS 对象为 STEP 文件（默认包含所有子层级）
    static bool Export(GDSObject_ogl* obj, const char* filename);

    // 导出 GDS 对象为 STEP 文件（可选是否包含子层级）
    static bool Export(GDSObject_ogl* obj, const char* filename, bool includeChildren);

    // 导出 GDS 对象为 STEP 文件（可选窄边清理）
    static bool Export(GDSObject_ogl* obj, const char* filename,
                      bool includeChildren, bool enableNarrowEdgeClean,
                      double narrowEdgeDelta = 5.0);

    // 使用已合并的多边形（从 Output::GetFullGDSItems 获取）导出 STEP
    static bool Export(Output* output, const char* filename,
                      bool enableNarrowEdgeClean = false, double narrowEdgeDelta = 5.0);
};

#endif // __STEP_EXPORT_H__