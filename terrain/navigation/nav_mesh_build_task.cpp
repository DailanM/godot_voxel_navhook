#include "nav_mesh_build_task.h"

#ifdef VOXEL_ENABLE_NAVIGATION

#include "nav_build_contours_raw.h"
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

	if (debug_this_chunk) {
		print_line(format("[NAV DEBUG] === Building chunk ({},{},{}) ===",
				chunk_position.x, chunk_position.y, chunk_position.z));
		print_line(format("[NAV DEBUG] Input: {} triangles, {} neighbors, {} obstacles",
				chunk_triangles.indices.size() / 3,
				neighbor_triangles.size(),
				obstacles.size()));
		print_line(format("[NAV DEBUG] AABB: origin=({:.2f},{:.2f},{:.2f}) size=({:.2f},{:.2f},{:.2f})",
				chunk_triangles.world_aabb.position.x,
				chunk_triangles.world_aabb.position.y,
				chunk_triangles.world_aabb.position.z,
				chunk_triangles.world_aabb.size.x,
				chunk_triangles.world_aabb.size.y,
				chunk_triangles.world_aabb.size.z));
		print_line(format("[NAV DEBUG] Config: cs={:.3f} ch={:.3f} walkableRadius={} walkableHeight={} "
				"walkableClimb={} slopeAngle={:.1f} borderSize={}",
				cfg.cs, cfg.ch, cfg.walkableRadius, cfg.walkableHeight,
				cfg.walkableClimb, cfg.walkableSlopeAngle, cfg.borderSize));
	}

	rcContext recast_ctx;

	// --- Step 1: Setup heightfield bounds ---

	ERR_FAIL_COND_MSG(cfg.borderSize <= cfg.walkableRadius,
			"NavMeshBuild: borderSize must be greater than walkableRadius for proper tile stitching");

	const int border = cfg.borderSize;
	const float cs = cfg.cs;

	// Chunk world bounds from stored AABB
	const AABB &chunk_aabb = chunk_triangles.world_aabb;

	// Y bounds: chunk's own vertical extent, padded so that Y-neighbor
	// geometry participates in filtering, erosion, and the seam-strip
	// region build correctly.
	//
	// Worst-case slope: terrain can rise/fall walkableClimb voxels per XZ
	// cell.  Over walkableRadius cells (the erosion distance), that is
	// walkableRadius * walkableClimb voxels.  If the heightfield doesn't
	// capture that geometry, rasterization clips it, creating artificial
	// disconnected XZ edges that erosion treats as walls — shrinking the
	// walkable interior at Y seams.
	//
	// The seam-strip region build also needs y_band_strip_radius voxels of
	// context on each side of the chunk's Y boundary so that neighboring
	// chunks see the same pre-erosion span layout in the shared strip —
	// otherwise strip region IDs would diverge and vertex alignment at the
	// seam would fail.
	//
	// pad_below  = (walkableRadius * walkableClimb + y_band_strip_radius) * ch
	// pad_above  = (walkableRadius * walkableClimb + walkableHeight
	//              + y_band_strip_radius) * ch
	const float pad_below = (cfg.walkableRadius * cfg.walkableClimb + y_band_strip_radius) * cfg.ch;
	const float pad_above =
			(cfg.walkableRadius * cfg.walkableClimb + cfg.walkableHeight + y_band_strip_radius) * cfg.ch;

	cfg.bmin[0] = chunk_aabb.position.x - border * cs;
	cfg.bmin[1] = chunk_aabb.position.y - pad_below;
	cfg.bmin[2] = chunk_aabb.position.z - border * cs;
	cfg.bmax[0] = chunk_aabb.position.x + chunk_aabb.size.x + border * cs;
	cfg.bmax[1] = chunk_aabb.position.y + chunk_aabb.size.y + pad_above;
	cfg.bmax[2] = chunk_aabb.position.z + chunk_aabb.size.z + border * cs;

	// Validate that the heightfield bounds fit within the 3x3x3 neighbor
	// volume.  If they don't, rasterization will be missing geometry and
	// erosion/filtering will produce incorrect results at the boundary.
	{
		const Vector3 chunk_size = chunk_aabb.size;
		const float neighbor_min_x = chunk_aabb.position.x - chunk_size.x;
		const float neighbor_min_y = chunk_aabb.position.y - chunk_size.y;
		const float neighbor_min_z = chunk_aabb.position.z - chunk_size.z;
		const float neighbor_max_x = chunk_aabb.position.x + chunk_size.x * 2.0f;
		const float neighbor_max_y = chunk_aabb.position.y + chunk_size.y * 2.0f;
		const float neighbor_max_z = chunk_aabb.position.z + chunk_size.z * 2.0f;

		ERR_FAIL_COND_MSG(
				cfg.bmin[0] < neighbor_min_x || cfg.bmin[1] < neighbor_min_y || cfg.bmin[2] < neighbor_min_z ||
				cfg.bmax[0] > neighbor_max_x || cfg.bmax[1] > neighbor_max_y || cfg.bmax[2] > neighbor_max_z,
				format("NavMeshBuild: heightfield bounds exceed 3x3x3 neighbor volume for chunk ({},{},{})."
					   " Reduce borderSize, walkableRadius, walkableClimb, walkableHeight,"
					   " or y_band_strip_radius.",
						chunk_position.x, chunk_position.y, chunk_position.z).c_str());
	}

	// Calculate width and height (now in cs units) from bounding box
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

	if (debug_this_chunk) {
		print_line(format("[NAV DEBUG] Heightfield: {}x{} cells", cfg.width, cfg.height));
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
	if (filter_low_height_spans) {
		rcFilterWalkableLowHeightSpans(&recast_ctx, cfg.walkableHeight, *hf);
	}

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

	if (use_erosion) {
		if (!rcErodeWalkableArea(&recast_ctx, cfg.walkableRadius, *chf)) {
			rcFreeCompactHeightfield(chf);
			ERR_FAIL_MSG("NavMeshBuild: Failed to erode walkable area");
		}
	}

	if (debug_this_chunk) {
		print_line(format("[NAV DEBUG] Compact heightfield: {} spans (after erosion={})",
				chf->spanCount, use_erosion));
	}

	if (!rcBuildDistanceField(&recast_ctx, *chf)) {
		rcFreeCompactHeightfield(chf);
		ERR_FAIL_MSG("NavMeshBuild: Failed to build distance field");
	}

	// --- 5-band monotone region build for cross-chunk Y-seam determinism ---
	//
	// Stock rcBuildRegionsMonotone partitions the entire chf in one pass, so
	// its output is chunk-local — interior region IDs near a Y boundary
	// fragment differently in each chunk, and rcBuildContours's simplify
	// step (RecastContour.cpp:230) places mandatory vertices at every
	// neighbor-region transition, producing T-junctions along the seam.
	//
	// Fix: split the sweep into five disjoint Y bands.
	//
	//   B1: [chunk_y_min - R, chunk_y_min)       — EXTERIOR below bottom
	//   B2: [chunk_y_min,     chunk_y_min + R)   — interior near bottom
	//   B3: [chunk_y_min + R, chunk_y_max - R)   — interior middle
	//   B4: [chunk_y_max - R, chunk_y_max)       — interior near top
	//   B5: [chunk_y_max,     chunk_y_max + R)   — EXTERIOR above top
	//
	// Each band gets its own sub-monotone pass (we null-mask every span
	// outside the target band before each call — rcBuildRegionsMonotone
	// skips null-area spans at RecastRegion.cpp:1412).  Since bands are
	// disjoint Y slabs, a region cannot span multiple bands via climb-
	// bridging — eliminating the "one region ID assigned to multiple
	// disconnected XZ components" failure we had with a single-strip sweep.
	//
	// Band identity across chunks:
	//   Chunk A's B5 spans == Chunk B's B2 spans  (shared +Y seam)
	//   Chunk A's B4 spans == Chunk B's B1 spans  (shared +Y seam, below)
	// Sub-monotone is deterministic given identical inputs, so the XZ
	// partition inside each shared band matches bit-identically across the
	// two chunks.
	//
	// Exterior bands (B1, B5): region IDs get RC_BORDER_REG OR'd on so
	// rcBuildContours skips tracing their contours (RecastContour.cpp:878,
	// 919 — no duplicate polygons with the adjacent chunk's interior).
	// Crucially, each exterior region keeps its own distinct ID (NOT a
	// single uniform border ID).  When an interior band's contour walks
	// its seam-facing edge, walkContour records each raw point's neighbor
	// reg (RecastContour.cpp:141), so the contour sees the EXTERIOR band's
	// region partition — which matches the adjacent chunk's interior
	// partition, giving bit-matching mandatory vertex placements on both
	// sides of the seam.
	//
	// Band radius constraint: R must be >= walkableClimb.  Otherwise a
	// span's XZ neighbor (reachable via walkableClimb in the full-chf
	// connectivity graph) could lie outside this band and we'd lose
	// visibility of the adjacent chunk's region structure at the seam.
	ERR_FAIL_COND_MSG(y_band_strip_radius < cfg.walkableClimb,
			format("NavMeshBuild: y_band_strip_radius ({}) must be >= walkableClimb ({}) "
				   "to guarantee cross-seam region visibility",
					y_band_strip_radius, cfg.walkableClimb).c_str());

	const int chunk_y_min_voxel =
			(int)floorf((chunk_aabb.position.y - chf->bmin[1]) / cfg.ch);
	const int chunk_y_max_voxel =
			(int)floorf((chunk_aabb.position.y + chunk_aabb.size.y - chf->bmin[1]) / cfg.ch);

	const int band_ranges[5][2] = {
		// [bandYMin, bandYMax)
		{ chunk_y_min_voxel - y_band_strip_radius, chunk_y_min_voxel },                          // B1 exterior
		{ chunk_y_min_voxel,                        chunk_y_min_voxel + y_band_strip_radius },   // B2 interior strip
		{ chunk_y_min_voxel + y_band_strip_radius,  chunk_y_max_voxel - y_band_strip_radius },   // B3 middle
		{ chunk_y_max_voxel - y_band_strip_radius,  chunk_y_max_voxel },                         // B4 interior strip
		{ chunk_y_max_voxel,                        chunk_y_max_voxel + y_band_strip_radius },   // B5 exterior
	};
	const bool band_is_exterior[5] = { true, false, false, false, true };

	StdVector<unsigned char> areas_backup(chf->spanCount);
	memcpy(areas_backup.data(), chf->areas, chf->spanCount);

	StdVector<unsigned short> combined_regs(chf->spanCount, 0);
	unsigned short id_offset = 0;

	for (int band = 0; band < 5; ++band) {
		const int bandYMin = band_ranges[band][0];
		const int bandYMax = band_ranges[band][1];
		if (bandYMax <= bandYMin) {
			// B3 (middle) can be empty if the chunk is thinner than 2 * R.
			continue;
		}

		// Restore areas then null-mask everything outside this band.
		memcpy(chf->areas, areas_backup.data(), chf->spanCount);
		for (int i = 0; i < chf->spanCount; ++i) {
			const int sy = (int)chf->spans[i].y;
			if (sy < bandYMin || sy >= bandYMax) {
				chf->areas[i] = RC_NULL_AREA;
			}
		}

		// Only the middle band (B3) applies the user's minRegionArea filter.
		// Strip bands (B2/B4) and exterior bands (B1/B5) are too thin in Y
		// for cfg.minRegionArea to be meaningful — applying it per-band
		// would drop regions that would have passed a full-chf sweep, and
		// dropped regions become null spans (walkability holes at the
		// seam).  Exterior dropped regions are equally harmful: they'd
		// leave interior-side contours seeing "null wall" instead of a
		// border-flagged portal, breaking seam connectivity.
		//
		// mergeRegionArea stays at the user's setting everywhere — merging
		// consolidates IDs but does not drop spans.
		const int band_min_region_area = (band == 2) ? cfg.minRegionArea : 0;

		if (!rcBuildRegionsMonotone(&recast_ctx, *chf, cfg.borderSize,
					band_min_region_area, cfg.mergeRegionArea)) {
			memcpy(chf->areas, areas_backup.data(), chf->spanCount);
			rcFreeCompactHeightfield(chf);
			ERR_FAIL_MSG("NavMeshBuild: Failed to build regions for band");
		}

		// Copy in-band region IDs into the combined buffer with the current
		// ID offset so IDs from different bands never collide.  Stock
		// rcBuildRegionsMonotone assigns IDs starting from 1 and writes
		// them into chf.spans[i].reg; out-of-band spans (those we null-
		// masked) stay at 0.
		//
		// Exterior bands (B1, B5) additionally OR on RC_BORDER_REG so
		// rcBuildContours skips tracing their contours while still
		// reporting their distinct IDs as neighbor-reg annotations to
		// interior contours walking the seam face.
		const unsigned short exterior_flag = band_is_exterior[band] ? RC_BORDER_REG : 0;
		for (int i = 0; i < chf->spanCount; ++i) {
			const unsigned short r = chf->spans[i].reg;
			if (r != 0) {
				// Preserve any RC_BORDER_REG already set (from the sub-call's
				// own XZ borderSize paint) and offset only the numeric part.
				const unsigned short flags = r & RC_BORDER_REG;
				const unsigned short numeric = r & ~RC_BORDER_REG;
				combined_regs[i] = (unsigned short)(numeric + id_offset) | flags | exterior_flag;
			}
		}

		id_offset = (unsigned short)(id_offset + chf->maxRegions);
	}

	// Restore areas for the remaining pipeline steps.
	memcpy(chf->areas, areas_backup.data(), chf->spanCount);

	// Write combined region IDs into chf.  Out-of-all-bands spans (in the
	// pad region beyond the outermost band) retain reg = 0; rcBuildContours
	// skips them (RecastContour.cpp:878).
	for (int i = 0; i < chf->spanCount; ++i) {
		chf->spans[i].reg = combined_regs[i];
	}
	chf->maxRegions = id_offset;

	if (debug_this_chunk) {
		print_line(format("[NAV DEBUG] Region build: {} total regions across 5 Y-bands", id_offset));
	}

	// --- Step 6: Contours, polymesh, detail mesh ---

	rcContourSet *cset = rcAllocContourSet();
	if (!cset) {
		rcFreeCompactHeightfield(chf);
		ERR_FAIL_MSG("NavMeshBuild: Failed to allocate contour set");
	}

	// DIAGNOSTIC: use the raw (non-simplified) contour builder to inspect
	// per-edge vertex placement along chunk-seam contours.  See
	// nav_build_contours_raw.h.
	if (!nav_build_contours_raw(&recast_ctx, *chf, *cset)) {
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

	if (use_detail_mesh) {
		// --- Detail mesh path: adds height detail to the flat poly mesh ---

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

		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);

		if (dmesh->nmeshes == 0 || dmesh->nverts == 0) {
			rcFreePolyMesh(pmesh);
			rcFreePolyMeshDetail(dmesh);
			return;
		}

		// --- Convert rcPolyMeshDetail to NavigationMesh ---

		result_nav_mesh.instantiate();

		const Vector3 chunk_origin = chunk_aabb.position;

		HashMap<Vector3, int> vertex_map;
		PackedVector3Array unique_verts;
		StdVector<int> deduped_index(dmesh->nverts);

		for (int i = 0; i < dmesh->nverts; i++) {
			const float *v = &dmesh->verts[i * 3];
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

		for (int i = 0; i < dmesh->nmeshes; i++) {
			const unsigned int *m = &dmesh->meshes[i * 4];
			const unsigned int bverts = m[0];
			const unsigned int btris = m[2];
			const unsigned int ntris = m[3];
			// dmesh->meshes is indexed by source pmesh polygon, so all
			// detail triangles for mesh entry i share pmesh->regs[i].
			const int region_id = (int)pmesh->regs[i];

			for (unsigned int j = 0; j < ntris; j++) {
				const unsigned char *t = &dmesh->tris[(btris + j) * 4];
				PackedInt32Array polygon;
				polygon.resize(3);
				polygon.write[0] = deduped_index[bverts + t[0]];
				polygon.write[1] = deduped_index[bverts + t[1]];
				polygon.write[2] = deduped_index[bverts + t[2]];
				result_nav_mesh->add_polygon(polygon);
				result_poly_regions.push_back(region_id);
			}
		}

		result_nav_mesh->set_vertices(unique_verts);
		result_nav_mesh->set_cell_size(cfg.cs);
		result_nav_mesh->set_cell_height(cfg.ch);

		rcFreePolyMesh(pmesh);
		rcFreePolyMeshDetail(dmesh);
	} else {
		// --- No detail mesh: convert rcPolyMesh directly ---
		// Vertices are quantized to cell grid. This is useful for debugging
		// since it shows exactly what the region/contour pipeline produced
		// without any detail mesh interpolation.

		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);

		if (pmesh->npolys == 0 || pmesh->nverts == 0) {
			rcFreePolyMesh(pmesh);
			return;
		}

		result_nav_mesh.instantiate();

		const Vector3 chunk_origin = chunk_aabb.position;

		// rcPolyMesh vertices are stored as unsigned short [x, y, z] in cell units
		// relative to pmesh->bmin. Convert to world-space then to chunk-local.
		PackedVector3Array verts;
		verts.resize(pmesh->nverts);
		for (int i = 0; i < pmesh->nverts; i++) {
			const unsigned short *pv = &pmesh->verts[i * 3];
			float wx = pmesh->bmin[0] + pv[0] * pmesh->cs;
			float wy = pmesh->bmin[1] + pv[1] * pmesh->ch;
			float wz = pmesh->bmin[2] + pv[2] * pmesh->cs;
			verts.write[i] = Vector3(wx - chunk_origin.x, wy - chunk_origin.y, wz - chunk_origin.z);
		}

		// rcPolyMesh polygons: each polygon is maxVertsPerPoly indices,
		// with RC_MESH_NULL_IDX (0xffff) marking unused slots.
		// Triangulate each polygon as a fan from vertex 0.
		const int nvp = pmesh->nvp;
		for (int i = 0; i < pmesh->npolys; i++) {
			const unsigned short *p = &pmesh->polys[i * nvp * 2];
			const int region_id = (int)pmesh->regs[i];

			// Count valid vertices in this polygon
			int nv = 0;
			for (int j = 0; j < nvp; j++) {
				if (p[j] == RC_MESH_NULL_IDX) {
					break;
				}
				nv++;
			}
			if (nv < 3) {
				continue;
			}

			// Fan triangulation
			for (int j = 1; j < nv - 1; j++) {
				PackedInt32Array polygon;
				polygon.resize(3);
				polygon.write[0] = p[0];
				polygon.write[1] = p[j];
				polygon.write[2] = p[j + 1];
				result_nav_mesh->add_polygon(polygon);
				result_poly_regions.push_back(region_id);
			}
		}

		result_nav_mesh->set_vertices(verts);
		result_nav_mesh->set_cell_size(cfg.cs);
		result_nav_mesh->set_cell_height(cfg.ch);

		rcFreePolyMesh(pmesh);
	}

	if (debug_this_chunk) {
		print_line(format("[NAV DEBUG] Result: {} verts, {} polygons (detail_mesh={})",
				result_nav_mesh->get_vertices().size(),
				result_nav_mesh->get_polygon_count(),
				use_detail_mesh));
		print_line(format("[NAV DEBUG] === Done chunk ({},{},{}) ===",
				chunk_position.x, chunk_position.y, chunk_position.z));
	}

	ZN_PRINT_VERBOSE(format("NavMeshBuild: chunk ({},{},{}) produced {} verts, {} polygons",
			chunk_position.x, chunk_position.y, chunk_position.z,
			result_nav_mesh->get_vertices().size(), result_nav_mesh->get_polygon_count()));
}

void NavMeshBuildTask::apply_result() {
	if (nav_mesh_manager && nav_mesh_manager->valid && result_nav_mesh.is_valid()) {
		nav_mesh_manager->apply_nav_result(chunk_position, result_nav_mesh, result_poly_regions, build_generation);
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
