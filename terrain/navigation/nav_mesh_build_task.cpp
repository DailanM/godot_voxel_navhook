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

	if (!rcBuildDistanceField(&recast_ctx, *chf)) {
		rcFreeCompactHeightfield(chf);
		ERR_FAIL_MSG("NavMeshBuild: Failed to build distance field");
	}

	// --- 3-band monotone region build for cross-chunk Y-seam determinism ---
	//
	// Problem: stock rcBuildRegionsMonotone partitions the entire chf as a
	// single pass.  Its output is chunk-local — interior region IDs near a
	// Y boundary fragment differently in each chunk, so the contour builder
	// places mandatory vertices at different XZ positions along the shared
	// seam, producing T-junctions when the two chunks' navmeshes meet.
	//
	// Fix: split the region sweep into three disjoint Y bands — a bottom
	// strip around chunk_y_min, a middle band (chunk interior), and a top
	// strip around chunk_y_max.  Each strip extends y_band_strip_radius
	// voxels on either side of the chunk's Y boundary, so it fully overlaps
	// the corresponding strip in the vertically-adjacent chunk.  Running
	// monotone independently on each band, with identical input spans in
	// the shared strips, gives vertically-adjacent chunks bit-identical
	// strip region IDs — which makes contour vertices at the seam line up.
	//
	// Implementation: back up chf.areas, null-mask every span outside the
	// target band (monotone skips null spans — RecastRegion.cpp:1412), run
	// stock rcBuildRegionsMonotone (which gives us mergeAndFilterRegions
	// for free per band), copy resulting IDs with an offset into a combined
	// buffer, restore areas, and repeat for the next band.  Finally write
	// the combined IDs into chf.spans[i].reg.
	const int chunk_y_min_voxel =
			(int)floorf((chunk_aabb.position.y - chf->bmin[1]) / cfg.ch);
	const int chunk_y_max_voxel =
			(int)floorf((chunk_aabb.position.y + chunk_aabb.size.y - chf->bmin[1]) / cfg.ch);

	const int band_ranges[3][2] = {
		// [bandYMin, bandYMax)
		{ chunk_y_min_voxel - y_band_strip_radius, chunk_y_min_voxel + y_band_strip_radius },
		{ chunk_y_min_voxel + y_band_strip_radius, chunk_y_max_voxel - y_band_strip_radius },
		{ chunk_y_max_voxel - y_band_strip_radius, chunk_y_max_voxel + y_band_strip_radius },
	};

	StdVector<unsigned char> areas_backup(chf->spanCount);
	memcpy(areas_backup.data(), chf->areas, chf->spanCount);

	StdVector<unsigned short> combined_regs(chf->spanCount, 0);
	unsigned short id_offset = 0;

	for (int band = 0; band < 3; ++band) {
		const int bandYMin = band_ranges[band][0];
		const int bandYMax = band_ranges[band][1];
		if (bandYMax <= bandYMin) {
			continue; // middle band may be empty if the chunk is thinner than 2 * strip_radius
		}

		// Restore areas then null-mask everything outside this band.
		memcpy(chf->areas, areas_backup.data(), chf->spanCount);
		for (int i = 0; i < chf->spanCount; ++i) {
			const int sy = (int)chf->spans[i].y;
			if (sy < bandYMin || sy >= bandYMax) {
				chf->areas[i] = RC_NULL_AREA;
			}
		}

		if (!rcBuildRegionsMonotone(&recast_ctx, *chf, cfg.borderSize,
					cfg.minRegionArea, cfg.mergeRegionArea)) {
			memcpy(chf->areas, areas_backup.data(), chf->spanCount);
			rcFreeCompactHeightfield(chf);
			ERR_FAIL_MSG("NavMeshBuild: Failed to build regions for band");
		}

		// Copy in-band region IDs into the combined buffer with the current
		// ID offset so IDs from different bands never collide.  Stock
		// rcBuildRegionsMonotone assigns IDs starting from 1 and writes
		// them to chf.spans[i].reg, setting out-of-band spans (those we
		// null-masked) to 0 — we ignore those here.
		for (int i = 0; i < chf->spanCount; ++i) {
			const unsigned short r = chf->spans[i].reg;
			if (r != 0) {
				// Preserve the RC_BORDER_REG flag (painted by rcBuildRegionsMonotone
				// on XZ-border spans) and offset only the numeric region ID.
				const unsigned short flags = r & RC_BORDER_REG;
				const unsigned short numeric = r & ~RC_BORDER_REG;
				combined_regs[i] = (unsigned short)(numeric + id_offset) | flags;
			}
		}

		// Next band's IDs start after this band's max.
		id_offset = (unsigned short)(id_offset + chf->maxRegions);
	}

	// Restore areas for the remaining pipeline steps.
	memcpy(chf->areas, areas_backup.data(), chf->spanCount);

	// --- Y-border region assignment ---
	//
	// Write combined region IDs into chf.  Spans strictly outside the
	// chunk's own Y range are OVERWRITTEN with one of two single dedicated
	// border IDs (below / above).  Using uniform IDs is essential for
	// vertex alignment: rcBuildContours (RecastContour.cpp:878,919) skips
	// tracing border-flagged spans, and walkContour (line 141) records the
	// neighbor span's reg as each contour-edge point's annotation.
	// simplifyContour (line 230) then places a mandatory vertex wherever
	// that annotation changes.  If the out-of-chunk spans retained their
	// per-band region IDs (only OR'd with RC_BORDER_REG), the varying IDs
	// along the Y-face would produce the very T-junctions this three-band
	// scheme exists to prevent.  A single uniform border ID per side means
	// the contour along the Y-face sees no annotation changes — so no
	// mandatory vertices arise from the Y-border itself.
	//
	// Two distinct IDs (below vs above) so that where Y-border meets other
	// boundaries (XZ borders, interior region transitions), the region
	// change still forces a corner vertex.
	//
	// Interior-side spans keep their band-region IDs (shared strip IDs on
	// strip spans, chunk-local IDs on middle-band spans).  The contour of
	// a shared-strip region then bounds the interior-side half only, with
	// its Y-face neighbor being the uniform y_border_above_reg or
	// y_border_below_reg — producing a clean portal edge at the seam that
	// matches the symmetric edge in the vertically-adjacent chunk.
	const unsigned short y_border_below_reg =
			(unsigned short)(id_offset + 1) | RC_BORDER_REG;
	const unsigned short y_border_above_reg =
			(unsigned short)(id_offset + 2) | RC_BORDER_REG;
	id_offset = (unsigned short)(id_offset + 2);

	for (int i = 0; i < chf->spanCount; ++i) {
		const int sy = (int)chf->spans[i].y;
		if (sy < chunk_y_min_voxel) {
			chf->spans[i].reg = y_border_below_reg;
		} else if (sy >= chunk_y_max_voxel) {
			chf->spans[i].reg = y_border_above_reg;
		} else {
			chf->spans[i].reg = combined_regs[i];
		}
	}
	chf->maxRegions = id_offset;

	// --- Step 6: Contours, polymesh, detail mesh ---

	rcContourSet *cset = rcAllocContourSet();
	if (!cset) {
		rcFreeCompactHeightfield(chf);
		ERR_FAIL_MSG("NavMeshBuild: Failed to allocate contour set");
	}

	if (!rcBuildContours(&recast_ctx, *chf, cfg.maxSimplificationError,
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

			for (unsigned int j = 0; j < ntris; j++) {
				const unsigned char *t = &dmesh->tris[(btris + j) * 4];
				PackedInt32Array polygon;
				polygon.resize(3);
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
			}
		}

		result_nav_mesh->set_vertices(verts);
		result_nav_mesh->set_cell_size(cfg.cs);
		result_nav_mesh->set_cell_height(cfg.ch);

		rcFreePolyMesh(pmesh);
	}

	ZN_PRINT_VERBOSE(format("NavMeshBuild: chunk ({},{},{}) produced {} verts, {} polygons",
			chunk_position.x, chunk_position.y, chunk_position.z,
			result_nav_mesh->get_vertices().size(), result_nav_mesh->get_polygon_count()));
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
