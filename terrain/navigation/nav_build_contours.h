#ifndef VOXEL_NAV_BUILD_CONTOURS_H
#define VOXEL_NAV_BUILD_CONTOURS_H

#ifdef VOXEL_ENABLE_NAVIGATION

#include <Recast.h>

namespace zylann::voxel {

// Cross-chunk-aware contour builder for tiled navmesh generation.
//
// Replaces stock rcBuildContours with a version that:
// - Walks both interior AND border region contours (stock skips border regions)
// - Uses RC_BORDER_VERTEX filtering to produce matching portal edges between chunks
// - Forces direction-independent simplification and tessellation on all edge types
// - Fuses walk, simplification, and tessellation into a consistent pipeline
//
// Border region contours are walked with modified flag clearing and direction
// reversal at mandatory non-portal vertices to correctly trace the
// border-interior boundary.
//
// Raw vertices are a working array only — cont->rverts is set to nullptr.
// Hole merging applies to interior regions only; border contours are skipped
// to avoid spurious "Bad outline" warnings.
bool nav_build_contours(rcContext *ctx, const rcCompactHeightfield &chf,
		float maxError, int maxEdgeLen, rcContourSet &cset);

} // namespace zylann::voxel

#endif // VOXEL_ENABLE_NAVIGATION
#endif // VOXEL_NAV_BUILD_CONTOURS_H
