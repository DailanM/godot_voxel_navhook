#ifndef VOXEL_BUILD_CHUNK_CONTOURS_H
#define VOXEL_BUILD_CHUNK_CONTOURS_H

#ifdef VOXEL_ENABLE_NAVIGATION

#include <Recast.h>

namespace zylann::voxel {
// Contour builder for chunked navmesh generation.
//
// Replaces stock rcBuildContours. This contour walking function creates contours 
// that are push-offs of the boundary to force cross-chunk compatibility
//
// Boundary regions that are connected to interior regions are walked to create
// contours at the boundary, which extend into the interior region when two interior
// region meet the boundary. two interior regions meeting at the boundary would
// break cross chunk continuity in the nav-mesh, and this function prevents that.  
//
bool build_chunk_contours(rcContext *ctx, const rcCompactHeightfield &chf,
		                  float maxError, int maxEdgeLen, rcContourSet &cset);

} // namespace zylann::voxel

#endif // VOXEL_ENABLE_NAVIGATION
#endif // VOXEL_BUILD_CHUNK_CONTOURS_H