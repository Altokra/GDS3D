/*
 * @Author: ka1shu1 cwh979946@163.com
 * @Date: 2026-04-18
 * @Description: Narrow edge cleaning module for export pipeline.
 *               Uses Clipper library to union + morphological open (expand/shrink)
 *               per Z-layer, removing narrow protrusions from via enclosure.
 */

#ifndef __POLYGON_CLEANER_H__
#define __POLYGON_CLEANER_H__

#include <vector>
#include <utility>
#include <map>

class GDSPolygon;

class PolygonCleaner {
public:
    /**
     * Clean narrow edges from same-layer polygons in-place.
     * Algorithm: Clipper union -> ClipperOffset(+delta) -> ClipperOffset(-delta)
     *
     * @param polygons   Same-layer GDSPolygon* vector (modified in-place).
     * @param delta_db   Narrow-edge threshold in database units.
     *                   Features narrower than 2*delta will be removed.
     *                   0.0 = union only, no morphological cleaning.
     * @return           Number of cleaned output polygons.
     */
    static size_t CleanPolygonsInPlace(
        std::vector<GDSPolygon*>& polygons,
        double delta_db
    );

    /**
     * Clean narrow edges, returning cleaned coordinates WITHOUT modifying originals.
     *
     * @param polygons   Same-layer GDSPolygon* vector (unchanged).
     * @param delta_db  Narrow-edge threshold in database units.
     * @param outCoords Output: maps each polygon pointer to its cleaned coordinates.
     *                 Each pair.first = vector of x coords, pair.second = vector of y coords.
     * @return          Number of cleaned output polygons.
     */
    static size_t CleanPolygonsToCoords(
        const std::vector<GDSPolygon*>& polygons,
        double delta_db,
        std::map<GDSPolygon*, std::pair<std::vector<double>, std::vector<double>>>& outCoords
    );
};

#endif // __POLYGON_CLEANER_H__
