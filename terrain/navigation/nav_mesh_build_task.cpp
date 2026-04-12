#include "nav_mesh_build_task.h"

#ifdef VOXEL_ENABLE_NAVIGATION

#include "../../util/io/log.h"
#include "../../util/profiling.h"
#include "../../util/string/format.h"

namespace zylann::voxel {

void NavMeshBuildTask::run(ThreadedTaskContext &ctx) {
	ZN_PROFILE_SCOPE();

	ZN_PRINT_VERBOSE(format("NavMeshBuildTask::run() chunk=({},{},{}) triangles={} neighbors={} obstacles={}",
			chunk_position.x, chunk_position.y, chunk_position.z,
			chunk_triangles.indices.size() / 3,
			neighbor_triangles.size(),
			obstacles.size()));

	rcContext recast_ctx;

	// --- Step 1: Setup heightfield bounds ---

	const int border = cfg.borderSize;
	const float cs = cfg.cs;

	// Chunk world bounds from stored AABB
	const AABB &chunk_aabb = chunk_triangles.world_aabb;

	// Y bounds: chunk's own vertical extent, padded on both sides so that
	// Y-neighbor geometry can be rasterized and participate in filters,
	// compact build, and erosion the same way XZ borderSize geometry does.
	// The Y analog of XZ borderSize — out-of-range spans get excluded later
	// via rcMarkBoxArea after erode, so polys only form in this chunk's
	// owned Y range.
	// - pad_below = walkableClimb so step-up climb detection works at the
	//   lower seam.
	// - pad_above = walkableHeight so clearance rejection works at the
	//   upper seam.
	const float pad_below = cfg.walkableClimb * cfg.ch;
	const float pad_above = cfg.walkableHeight * cfg.ch;

	cfg.bmin[0] = chunk_aabb.position.x - border * cs;
	cfg.bmin[1] = chunk_aabb.position.y - pad_below;
	cfg.bmin[2] = chunk_aabb.position.z - border * cs;
	cfg.bmax[0] = chunk_aabb.position.x + chunk_aabb.size.x + border * cs;
	cfg.bmax[1] = chunk_aabb.position.y + chunk_aabb.size.y + pad_above;
	cfg.bmax[2] = chunk_aabb.position.z + chunk_aabb.size.z + border * cs;

	rcCalcGridSize(cfg.bmin, cfg.bmax, cfg.cs, &cfg.width, &cfg.height);

	rcHeightfield *hf = rcAllocHeightfield();
	ERR_FAIL_NULL(hf);

	if (!rcCreateHeightfield(&recast_ctx, *hf, cfg.width, cfg.height,
				cfg.bmin, cfg.bmax, cfg.cs, cfg.ch)) {
		rcFreeHeightField(hf);
		return;
	}

	// --- Step 2: Rasterize terrain triangles ---

	// All chunk + neighbor data is rasterized normally (as walkable where
	// slope permits). Continuous terrain crossing chunk boundaries produces
	// one merged solid span per column thanks to addSpan's overlap merging
	// — so there's no fake "ceiling" that would trip up the low-height
	// filter. Out-of-chunk-range polys are excluded later via rcMarkBoxArea
	// on the compact heightfield (the Y analog of XZ borderSize).
	auto rasterize = [&](const NavChunkData &data) {
		if (data.indices.size() == 0) {
			return;
		}
		const int num_tris = data.indices.size() / 3;
		StdVector<unsigned char> tri_areas(num_tris, 0);
		rcMarkWalkableTriangles(&recast_ctx, cfg.walkableSlopeAngle,
				(const float *)data.positions.data(), data.positions.size(),
				(const int *)data.indices.data(), num_tris, tri_areas.data());
		rcRasterizeTriangles(&recast_ctx,
				(const float *)data.positions.data(), data.positions.size(),
				(const int *)data.indices.data(), tri_areas.data(), num_tris,
				*hf, cfg.walkableClimb);
	};

	rasterize(chunk_triangles);
	for (const auto &neighbor : neighbor_triangles) {
		rasterize(neighbor);
	}

	// --- Step 3: Rasterize obstacles ---
	// Obstacle mesh extraction will be implemented in Phase 5.
	// The obstacle snapshot is already captured in the `obstacles` vector.

	// --- Step 4: Filtering ---

	if (filter_low_hanging) {
		rcFilterLowHangingWalkableObstacles(&recast_ctx, cfg.walkableClimb, *hf);
	}
	if (filter_ledge_spans) {
		// Must run AFTER LowHangingWalkableObstacles (ordering matters per Recast.h)
		rcFilterLedgeSpans(&recast_ctx, cfg.walkableHeight, cfg.walkableClimb, *hf);
	}
	// TEMP: disabled to test Y-seam hypothesis — the low-height filter
	// creates false ceilings where two chunks' surface meshes overlap
	// vertically in the Y-padding zone.
	// if (filter_low_height_spans) {
	// 	rcFilterWalkableLowHeightSpans(&recast_ctx, cfg.walkableHeight, *hf);
	// }

	// --- Step 5: Compact, erode, build regions ---

	rcCompactHeightfield *chf = rcAllocCompactHeightfield();
	if (!chf) {
		rcFreeHeightField(hf);
		ERR_FAIL_MSG("NavMeshBuild: Failed to allocate compact heightfield");
	}

	if (!rcBuildCompactHeightfield(&recast_ctx, cfg.walkableHeight,
				cfg.walkableClimb, *hf, *chf)) {
		rcFreeHeightField(hf);
		rcFreeCompactHeightfield(chf);
		ERR_FAIL_MSG("NavMeshBuild: Failed to build compact heightfield");
	}

	rcFreeHeightField(hf);
	hf = nullptr;

	// TEMP: disabled to test Y-seam hypothesis — erosion may be eating
	// into walkable area at the Y boundary due to missing neighbor connectivity
	// if (!rcErodeWalkableArea(&recast_ctx, cfg.walkableRadius, *chf)) {
	// 	rcFreeCompactHeightfield(chf);
	// 	ERR_FAIL_MSG("NavMeshBuild: Failed to erode walkable area");
	// }

	// --- Y-border exclusion (analog of XZ borderSize paintRectRegion) ---
	//
	// Mark compact spans whose walkable floor lies outside this chunk's
	// owned Y range [chunk.y, chunk.y + size.y) as RC_NULL_AREA, so that
	// region/contour/polymesh build produces polys only for spans we own.
	// Recast's XZ tile mechanism does the same thing for the borderSize
	// ring via paintRectRegion inside rcBuildRegions*. We're applying it
	// here for Y because Recast has no built-in Y tiling.
	//
	// Critically this runs AFTER erode — the pad spans are walkable through
	// erosion so the chunk interior isn't shrunk at the Y seams, matching
	// how XZ borderSize spans stay walkable during erosion.
	{
		// Below box: everything with Y cell < chunk floor cell.
		// Using (chunk.y - 0.5*ch) as the upper bound gives miny=0,
		// maxy = chunk_floor_cell - 1 under rcMarkBoxArea's truncating
		// integer conversion.
		const float below_min[3] = { cfg.bmin[0], cfg.bmin[1], cfg.bmin[2] };
		const float below_max[3] = {
			cfg.bmax[0],
			float(chunk_aabb.position.y) - 0.5f * cfg.ch,
			cfg.bmax[2]
		};
		rcMarkBoxArea(&recast_ctx, below_min, below_max, RC_NULL_AREA, *chf);

		// Above box: everything with Y cell >= chunk ceil cell.
		// Using chunk.y + size.y exactly gives miny = chunk_ceil_cell,
		// maxy = top of heightfield.
		const float above_min[3] = {
			cfg.bmin[0],
			float(chunk_aabb.position.y + chunk_aabb.size.y),
			cfg.bmin[2]
		};
		const float above_max[3] = { cfg.bmax[0], cfg.bmax[1], cfg.bmax[2] };
		rcMarkBoxArea(&recast_ctx, above_min, above_max, RC_NULL_AREA, *chf);
	}

	if (!rcBuildDistanceField(&recast_ctx, *chf)) {
		rcFreeCompactHeightfield(chf);
		ERR_FAIL_MSG("NavMeshBuild: Failed to build distance field");
	}

	// Monotone partitioning - faster, good for runtime/streaming use
	if (!rcBuildRegionsMonotone(&recast_ctx, *chf, cfg.borderSize,
				cfg.minRegionArea, cfg.mergeRegionArea)) {
		rcFreeCompactHeightfield(chf);
		ERR_FAIL_MSG("NavMeshBuild: Failed to build regions");
	}

	// --- Step 6: Contours, polymesh, detail mesh ---

	rcContourSet *cset = rcAllocContourSet();
	if (!cset) {
		rcFreeCompactHeightfield(chf);
		ERR_FAIL_MSG("NavMeshBuild: Failed to allocate contour set");
	}

	// TEMP: disable contour simplification to test Y-seam hypothesis
	if (!rcBuildContours(&recast_ctx, *chf, 0.0f,
				cfg.maxEdgeLen, *cset)) {
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		ERR_FAIL_MSG("NavMeshBuild: Failed to build contours");
	}

	rcPolyMesh *pmesh = rcAllocPolyMesh();
	if (!pmesh) {
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		ERR_FAIL_MSG("NavMeshBuild: Failed to allocate poly mesh");
	}

	if (!rcBuildPolyMesh(&recast_ctx, *cset, cfg.maxVertsPerPoly, *pmesh)) {
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		rcFreePolyMesh(pmesh);
		ERR_FAIL_MSG("NavMeshBuild: Failed to build poly mesh");
	}

	rcPolyMeshDetail *dmesh = rcAllocPolyMeshDetail();
	if (!dmesh) {
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		rcFreePolyMesh(pmesh);
		ERR_FAIL_MSG("NavMeshBuild: Failed to allocate detail mesh");
	}

	if (!rcBuildPolyMeshDetail(&recast_ctx, *pmesh, *chf,
				cfg.detailSampleDist, cfg.detailSampleMaxError, *dmesh)) {
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		rcFreePolyMesh(pmesh);
		rcFreePolyMeshDetail(dmesh);
		ERR_FAIL_MSG("NavMeshBuild: Failed to build detail mesh");
	}

	// Free intermediates no longer needed
	rcFreeCompactHeightfield(chf);
	rcFreeContourSet(cset);

	// --- Step 7: Convert rcPolyMeshDetail to NavigationMesh ---

	if (dmesh->nmeshes == 0 || dmesh->nverts == 0) {
		rcFreePolyMesh(pmesh);
		rcFreePolyMeshDetail(dmesh);
		return;
	}

	result_nav_mesh.instantiate();

	// Chunk world origin for converting Recast world-space output to chunk-local space.
	// NavigationServer3D applies region_set_transform() to these vertices,
	// so they must be relative to the chunk origin.
	const Vector3 chunk_origin = chunk_aabb.position;

	// Deduplicate vertices (detail mesh may have shared and interpolated vertices)
	HashMap<Vector3, int> vertex_map;
	PackedVector3Array unique_verts;
	StdVector<int> deduped_index(dmesh->nverts);

	for (int i = 0; i < dmesh->nverts; i++) {
		const float *v = &dmesh->verts[i * 3];
		// Convert from world space to chunk-local space
		Vector3 vert(v[0] - chunk_origin.x, v[1] - chunk_origin.y, v[2] - chunk_origin.z);

		auto *it = vertex_map.getptr(vert);
		if (it != nullptr) {
			deduped_index[i] = *it;
		} else {
			int idx = unique_verts.size();
			unique_verts.push_back(vert);
			vertex_map[vert] = idx;
			deduped_index[i] = idx;
		}
	}

	// Extract triangles with reversed winding order (Godot convention)
	for (int i = 0; i < dmesh->nmeshes; i++) {
		const unsigned int *m = &dmesh->meshes[i * 4];
		const unsigned int bverts = m[0]; // vertex start index
		const unsigned int btris = m[2]; // triangle start index
		const unsigned int ntris = m[3]; // triangle count

		for (unsigned int j = 0; j < ntris; j++) {
			const unsigned char *t = &dmesh->tris[(btris + j) * 4];
			PackedInt32Array polygon;
			polygon.resize(3);
			// Input triangles were already wound for Recast (CCW), so Recast output
			// is in the correct orientation. No additional winding reversal needed.
			polygon.write[0] = deduped_index[bverts + t[0]];
			polygon.write[1] = deduped_index[bverts + t[1]];
			polygon.write[2] = deduped_index[bverts + t[2]];
			result_nav_mesh->add_polygon(polygon);
		}
	}

	result_nav_mesh->set_vertices(unique_verts);
	result_nav_mesh->set_cell_size(cfg.cs);
	result_nav_mesh->set_cell_height(cfg.ch);

	rcFreePolyMesh(pmesh);
	rcFreePolyMeshDetail(dmesh);

	ZN_PRINT_VERBOSE(format("NavMeshBuild: chunk ({},{},{}) produced {} verts, {} polygons",
			chunk_position.x, chunk_position.y, chunk_position.z,
			unique_verts.size(), result_nav_mesh->get_polygon_count()));
}

void NavMeshBuildTask::apply_result() {
	if (nav_mesh_manager && nav_mesh_manager->valid && result_nav_mesh.is_valid()) {
		nav_mesh_manager->apply_nav_result(chunk_position, result_nav_mesh, build_generation);
	}
}

TaskPriority NavMeshBuildTask::get_priority() {
	// Lowest priority — nav tasks run after all mesh tasks
	return TaskPriority::min();
}

bool NavMeshBuildTask::is_cancelled() {
	return nav_mesh_manager == nullptr || !nav_mesh_manager->valid;
}

} // namespace zylann::voxel

#endif // VOXEL_ENABLE_NAVIGATION
