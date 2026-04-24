#ifndef VOXEL_NAV_BUILD_CONTOURS_RAW_H
#define VOXEL_NAV_BUILD_CONTOURS_RAW_H

#ifdef VOXEL_ENABLE_NAVIGATION

#include <Recast.h>

namespace zylann::voxel {

// Diagnostic variant of rcBuildContours that skips simplification.
//
// Feeds the raw output of walkContour() directly into rcContour::verts,
// bypassing both simplifyContour() and removeDegenerateSegments().  Corner
// heights and the RC_BORDER_VERTEX / RC_AREA_BORDER flags are preserved
// from walkContour exactly as the stock pipeline would see them.
//
// Used to inspect where vertices are silently dropped by the simplify /
// remove-degenerate steps along chunk-seam contours.
//
// Hole merging is still performed (so the polymesh step can consume the
// result), matching the structure of rcBuildContours.
bool nav_build_contours_raw(rcContext *ctx, const rcCompactHeightfield &chf, rcContourSet &cset);

} // namespace zylann::voxel

#endif // VOXEL_ENABLE_NAVIGATION
#endif // VOXEL_NAV_BUILD_CONTOURS_RAW_H
