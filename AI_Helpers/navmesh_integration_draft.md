# Godot-Voxel ↔ Recast Navigation Integration Plan

## Overview

This document describes the work required to generate navigation meshes with proper clearance handling for smooth (Transvoxel) terrain produced by Zylann's godot-voxel module. The approach feeds the triangle mesh already produced by the Transvoxel mesher into Recast's standard pipeline (rasterization → filtering → regions → contours → polymesh), then registers the resulting navmesh with Godot's NavigationServer3D so that standard NavigationAgent3D nodes can pathfind on it.

This avoids re-implementing clearance logic and gives us Recast's battle-tested handling of vertical clearance, ledge detection, and radius erosion — while using Godot's native navigation infrastructure for pathfinding and agent control.

---

## Architecture Diagram

```
┌──────────────────────────────────────────────────────────────────────┐
│                        GODOT-VOXEL SIDE                              │
│                                                                      │
│  VoxelLodTerrain / VoxelTerrain                                      │
│       │                                                              │
│       ├── VoxelData (chunked spatial container)                      │
│       │     └── VoxelBuffer per chunk (SDF channel, 8/16-bit)        │
│       │                                                              │
│       ├── Transvoxel mesher output (triangle mesh per chunk)         │
│       │     └── vertices + indices, already produced for             │
│       │         rendering and physics                                │
│       │                                                              │
│       ├── Mesh update callback (mesh_update_notification branch)     │
│       │     └── Fires when a chunk mesh is created/updated           │
│       │                                                              │
│       └── Chunk coordinates + world transform                        │
│                                                                      │
└──────────────────────┬───────────────────────────────────────────────┘
                       │
                       │  INTERFACE POINT 1: Triangle mesh → Recast
                       │
┌──────────────────────▼───────────────────────────────────────────────┐
│                     BRIDGE LAYER (new C++ code)                      │
│                                                                      │
│  VoxelNavBuilder                                                     │
│       │                                                              │
│       ├── 1. Receive triangle mesh from Transvoxel mesher            │
│       ├── 2. Classify walkable terrain triangles by slope            │
│       ├── 3. Rasterize terrain into rcHeightfield                    │
│       ├── 4. Query obstacle source registry for this chunk's AABB    │
│       ├── 5. Rasterize obstacle meshes into same rcHeightfield       │
│       │                                                              │
│       │  INTERFACE POINT 2: rcHeightfield → Recast pipeline          │
│       │                                                              │
│       ├── 6. Run Recast filters (clearance, ledge, low-height)       │
│       ├── 7. Compact → erode → distance field → regions              │
│       ├── 8. Contours → PolyMesh → PolyMeshDetail                   │
│       │                                                              │
│       │  INTERFACE POINT 3: PolyMesh → Godot NavigationServer3D      │
│       │                                                              │
│       └── 9. Convert rcPolyMesh to NavigationMesh resource           │
│              and register with NavigationServer3D                    │
│                                                                      │
└──────────────────────────────────────────────────────────────────────┘
                       │
                       │
┌──────────────────────▼───────────────────────────────────────────────┐
│                     GODOT NAVIGATION SYSTEM                          │
│                                                                      │
│  NavigationServer3D                                                  │
│       ├── Manages navigation map (auto-stitches regions)             │
│       ├── Handles path queries across all registered regions         │
│       └── Supports NavigationAgent3D for steering/avoidance          │
│                                                                      │
│  NavigationAgent3D (standard Godot node)                             │
│       └── Works out of the box on our registered regions             │
│                                                                      │
└──────────────────────────────────────────────────────────────────────┘
```

---

## Phase 1: Obtaining the Triangle Mesh (godot-voxel side)

### What we need

The Transvoxel mesher already produces a triangle mesh per chunk for rendering and physics. We need access to the same vertex and index data for nav generation. This means nav resolution is decoupled from terrain voxel resolution — we can set Recast's cell size (`cs`) to whatever we need for accurate clearance (typically `agent_radius / 2` or `agent_radius / 3`), regardless of how large the terrain voxels are. The triangle mesh rasterizes cleanly at any nav voxel resolution.

**Per chunk, we need:**

| Data | Source | Notes |
|------|--------|-------|
| Vertex positions | Transvoxel mesher output | `float[]` in (x, y, z) triples |
| Triangle indices | Transvoxel mesher output | `int[]` in triples |
| Chunk world position | Chunk coordinates × voxel size | For rcConfig bounds |
| Chunk dimensions | Terrain node properties | For tile sizing |

### When to trigger nav rebuild

The `mesh_update_notification` feature branch adds a virtual method to `VoxelTerrain` called when a chunk mesh is created or updated, with access to the mesh data. This is the natural hook. The nav builder should subscribe to these notifications and queue affected chunks for navmesh rebuilding.

### Files to look at

- `meshers/transvoxel/transvoxel.cpp` — mesher output structure, vertex/index arrays
- `terrain/variable_lod/voxel_lod_terrain.h` — LOD terrain node, chunk management
- `storage/voxel_data.h` — chunked spatial container, block access

---

## Phase 2: Recast Pipeline (rasterization through navmesh generation)

This is the standard Recast build process. The reference implementation is in `RecastDemo/Source/Sample_SoloMesh.cpp`. For tiled builds (which is what we want), see `Sample_TileMesh.cpp`.

### Step 2a: Configure and create the heightfield

```cpp
rcConfig cfg = {};
cfg.cs = agent_radius / 2.0f;          // nav cell size (horizontal)
cfg.ch = cfg.cs / 2.0f;                // nav cell height (vertical, finer)
cfg.walkableSlopeAngle = 45.0f;        // max walkable slope in degrees
cfg.walkableHeight = (int)ceilf(agent_height / cfg.ch);
cfg.walkableClimb  = (int)ceilf(agent_max_climb / cfg.ch);
cfg.walkableRadius = (int)ceilf(agent_radius / cfg.cs);
cfg.maxEdgeLen = cfg.walkableRadius * 8;
cfg.maxSimplificationError = 1.3f;
cfg.minRegionArea = 8;
cfg.mergeRegionArea = 20;
cfg.maxVertsPerPoly = 6;               // max 6 for Detour compatibility
cfg.detailSampleDist = 6.0f * cfg.cs;
cfg.detailSampleMaxError = cfg.ch;

// Tile border for proper edge stitching
cfg.borderSize = cfg.walkableRadius + 3;

// Set bounds from chunk world position (expanded by borderSize)
rcVcopy(cfg.bmin, chunk_bmin);
rcVcopy(cfg.bmax, chunk_bmax);
cfg.bmin[0] -= cfg.borderSize * cfg.cs;
cfg.bmin[2] -= cfg.borderSize * cfg.cs;
cfg.bmax[0] += cfg.borderSize * cfg.cs;
cfg.bmax[2] += cfg.borderSize * cfg.cs;
rcCalcGridSize(cfg.bmin, cfg.bmax, cfg.cs, &cfg.width, &cfg.height);

rcHeightfield* hf = rcAllocHeightfield();
rcCreateHeightfield(ctx, *hf, cfg.width, cfg.height,
                    cfg.bmin, cfg.bmax, cfg.cs, cfg.ch);
```

**Note on voxel size:** The nav cell size (`cs`) will typically be much smaller than the terrain voxel size. For example, with terrain voxels at 1m and an agent radius of 0.4m, `cs` should be around 0.15–0.2m. This is fine — Recast rasterizes the triangle mesh at whatever resolution we specify, independent of how the mesh was generated.

### Step 2b: Classify triangles and rasterize

```cpp
// Allocate triangle area IDs
unsigned char* tri_areas = new unsigned char[num_triangles];
memset(tri_areas, 0, num_triangles);

// Mark walkable triangles by slope (compares triangle normal Y
// against cos(walkableSlopeAngle))
rcMarkWalkableTriangles(ctx, cfg.walkableSlopeAngle,
                        verts, num_verts, tris, num_tris, tri_areas);

// Rasterize triangle mesh into heightfield
rcRasterizeTriangles(ctx, verts, num_verts, tris, tri_areas,
                     num_tris, *hf, cfg.walkableClimb);
```

This is where slope filtering happens. `rcMarkWalkableTriangles` checks each triangle's normal against the slope threshold and marks non-walkable triangles with `RC_NULL_AREA`. Because we're using the actual Transvoxel output mesh, the slope classification matches what physics sees — agents won't try to walk on surfaces they'd slide off of.

**Border geometry:** Because of the `borderSize` expansion, the heightfield extends beyond the chunk boundary. The triangle mesh rasterized here should also include geometry from neighboring chunks that falls within this expanded region. This ensures walkable spans near chunk edges aren't incorrectly eroded. The Transvoxel mesher already accesses neighboring voxel data for stitching — the nav builder needs similar overlap.

### Step 2c: Rasterize obstacle geometry

After terrain is rasterized, obstacle meshes registered with the nav builder are rasterized into the same heightfield. This is how obstructions (trees, rocks, buildings, etc.) carve into the navmesh — their geometry creates solid spans that block walkable space, and Recast's filters then handle the clearance consequences.

The nav builder maintains an **obstacle source registry**: a collection of references to collision meshes that should affect navigation. Developers register obstacle sources through the plugin API, and the nav builder queries the registry during each chunk build to find obstacles whose AABBs overlap the chunk's expanded bounds.

```cpp
// Obstacle source registry (maintained by the nav builder)
// Developers add/remove obstacle sources through the plugin API
struct ObstacleSource {
    Ref<Mesh> collision_mesh;   // or raw vertex/index data
    Transform3D transform;      // world-space transform
    AABB world_aabb;            // cached world-space AABB for spatial query
};

// During chunk navmesh build, after terrain rasterization:
// Query registry for obstacles overlapping this chunk's expanded AABB
AABB chunk_query_aabb = AABB(
    Vector3(cfg.bmin[0], cfg.bmin[1], cfg.bmin[2]),
    Vector3(cfg.bmax[0] - cfg.bmin[0],
            cfg.bmax[1] - cfg.bmin[1],
            cfg.bmax[2] - cfg.bmin[2]));

Vector<const ObstacleSource*> overlapping = registry.query(chunk_query_aabb);

for (const ObstacleSource* obs : overlapping) {
    // Extract triangles from the obstacle's collision mesh
    // Transform vertices to world space using obs->transform
    const float* obs_verts = ...;
    const int* obs_tris = ...;
    int obs_num_tris = ...;

    // All obstacle triangles are non-walkable
    unsigned char* obs_areas = new unsigned char[obs_num_tris];
    memset(obs_areas, RC_NULL_AREA, obs_num_tris);

    // Rasterize into the SAME heightfield as terrain
    rcRasterizeTriangles(ctx, obs_verts, obs_num_verts,
                         obs_tris, obs_areas, obs_num_tris,
                         *hf, cfg.walkableClimb);
    delete[] obs_areas;
}
```

**Exposed API for developers:**

```cpp
// Plugin API — called by game code to register obstacles
class VoxelNavBuilder : public Node {
public:
    // Add an obstacle source. Returns an ID for later removal.
    // The collision mesh will be rasterized into any chunk whose
    // navmesh overlaps the obstacle's world AABB.
    int add_obstacle(Ref<Mesh> collision_mesh, Transform3D transform);

    // Remove a previously added obstacle and mark affected chunks dirty.
    void remove_obstacle(int obstacle_id);

    // Update an obstacle's transform (e.g. if it moved).
    // Marks both old and new affected chunks dirty for rebuild.
    void update_obstacle_transform(int obstacle_id, Transform3D new_transform);
};
```

When an obstacle is added, removed, or moved, the nav builder determines which chunks are affected (by intersecting the obstacle's AABB with chunk AABBs) and queues those chunks for navmesh rebuild. The rebuild follows the same pipeline — terrain rasterization, obstacle rasterization, filters, polymesh — and the result is swapped into the NavigationServer via `region_set_navigation_mesh`.

Note that obstacles with walkable surfaces (bridges, ramps, building roofs that agents should walk on) would be rasterized with `rcMarkWalkableTriangles` instead of a blanket `RC_NULL_AREA`, so their slope is evaluated like terrain. The area ID assignment could be extended to support this through the registry API.

### Step 2d: Filtering (clearance handling)

```cpp
// Allow walking over small obstacles (curbs, small steps)
rcFilterLowHangingWalkableObstacles(ctx, cfg.walkableClimb, *hf);

// Remove spans near cliff edges
// Checks each span's neighbor height differences against walkableClimb
rcFilterLedgeSpans(ctx, cfg.walkableHeight, cfg.walkableClimb, *hf);

// Remove spans without enough vertical clearance (floor to ceiling)
// This is the key overhead/tunnel clearance check
rcFilterWalkableLowHeightSpans(ctx, cfg.walkableHeight, *hf);
```

**What each filter does:**

| Filter | What it checks | Effect |
|--------|---------------|--------|
| `LowHangingWalkableObstacles` | Obstacle span close above a walkable span | Marks obstacle as walkable (step-over) |
| `LedgeSpans` | Height difference to axis-neighbor > walkableClimb | Cliff edges become unwalkable |
| `WalkableLowHeightSpans` | Floor-to-ceiling gap < walkableHeight | Low overhangs, tunnels become unwalkable |

### Step 2e: Compact, erode, and build regions

```cpp
// Compact heightfield (more efficient representation)
rcCompactHeightfield* chf = rcAllocCompactHeightfield();
rcBuildCompactHeightfield(ctx, cfg.walkableHeight,
                          cfg.walkableClimb, *hf, *chf);

// Erode walkable area by agent radius (horizontal clearance)
// Shrinks navmesh away from walls and obstacles
rcErodeWalkableArea(ctx, cfg.walkableRadius, *chf);

// Build distance field (needed for watershed partitioning)
rcBuildDistanceField(ctx, *chf);

// Partition into regions
// Monotone is faster (good for runtime/streaming), Watershed is higher quality
rcBuildRegions(ctx, *chf, cfg.borderSize,
               cfg.minRegionArea, cfg.mergeRegionArea);
```

### Step 2f: Contours and polygon mesh

```cpp
// Trace contours from region boundaries
rcContourSet* cset = rcAllocContourSet();
rcBuildContours(ctx, *chf, cfg.maxSimplificationError,
                cfg.maxEdgeLen, *cset);

// Build polygon mesh from contours
rcPolyMesh* pmesh = rcAllocPolyMesh();
rcBuildPolyMesh(ctx, *cset, cfg.maxVertsPerPoly, *pmesh);

// Build detail mesh (adds vertical accuracy within polygons)
rcPolyMeshDetail* dmesh = rcAllocPolyMeshDetail();
rcBuildPolyMeshDetail(ctx, *pmesh, *chf,
                      cfg.detailSampleDist, cfg.detailSampleMaxError, *dmesh);

// Cleanup intermediate data
rcFreeHeightField(hf);
rcFreeCompactHeightfield(chf);
rcFreeContourSet(cset);
```

At this point, `pmesh` contains the navigation polygon mesh and `dmesh` contains the high-resolution detail mesh. These are the final Recast outputs.

---

## Phase 3: Integration with Godot NavigationServer3D

### Converting rcPolyMesh to Godot NavigationMesh

The `rcPolyMesh` contains vertices (in voxel-space integers) and polygon index arrays. These need to be converted to world-space floats and packaged into a Godot `NavigationMesh` resource.

```cpp
Ref<NavigationMesh> godot_nav_mesh;
godot_nav_mesh.instantiate();

// Convert vertices from Recast voxel coords to world space
PackedVector3Array vertices;
for (int i = 0; i < pmesh->nverts; i++) {
    const unsigned short* v = &pmesh->verts[i * 3];
    Vector3 world_pos;
    world_pos.x = pmesh->bmin[0] + v[0] * pmesh->cs;
    world_pos.y = pmesh->bmin[1] + v[1] * pmesh->ch;
    world_pos.z = pmesh->bmin[2] + v[2] * pmesh->cs;
    vertices.push_back(world_pos);
}
godot_nav_mesh->set_vertices(vertices);

// Add polygons (Recast polygons are convex, up to maxVertsPerPoly sides)
for (int i = 0; i < pmesh->npolys; i++) {
    // Skip null-area polygons
    if (pmesh->areas[i] == RC_NULL_AREA) continue;

    const unsigned short* p = &pmesh->polys[i * pmesh->nvp * 2];
    PackedInt32Array polygon;
    for (int j = 0; j < pmesh->nvp; j++) {
        if (p[j] == RC_MESH_NULL_IDX) break;
        polygon.push_back(p[j]);
    }
    godot_nav_mesh->add_polygon(polygon);
}
```

### Registering with NavigationServer3D

Each chunk gets its own navigation region. The NavigationServer automatically stitches adjacent regions when their edge vertices match exactly.

```cpp
// One-time setup (or on chunk enter nav range)
RID region_rid = NavigationServer3D::get_singleton()->region_create();
NavigationServer3D::get_singleton()->region_set_enabled(region_rid, true);
NavigationServer3D::get_singleton()->region_set_map(
    region_rid, world->get_navigation_map());
NavigationServer3D::get_singleton()->region_set_transform(
    region_rid, chunk_world_transform);

// Set the navmesh data
NavigationServer3D::get_singleton()->region_set_navigation_mesh(
    region_rid, godot_nav_mesh);
```

**On chunk unload:**
```cpp
NavigationServer3D::get_singleton()->free(region_rid);
```

**On terrain edit (chunk navmesh needs rebuild):**
```cpp
// Rebuild navmesh on worker thread, then on main thread:
NavigationServer3D::get_singleton()->region_set_navigation_mesh(
    region_rid, new_nav_mesh);
```

### Cell size synchronization

The navigation map's cell size must match the NavigationMesh cell size. Since Recast's `cs` will likely differ from Godot's default (0.25m), set this at initialization:

```cpp
RID map_rid = world->get_navigation_map();
NavigationServer3D::get_singleton()->map_set_cell_size(map_rid, cfg.cs);
NavigationServer3D::get_singleton()->map_set_cell_height(map_rid, cfg.ch);
```

### Region edge stitching

Godot's NavigationServer merges regions when edge vertices overlap exactly. For chunk-based tiling, this means the navmesh polygons along chunk boundaries must produce vertices at identical positions for adjacent chunks. Recast's tiled build system handles this through the `borderSize` parameter and portal edge detection — polygons along tile boundaries are clipped to the tile edge, and edges on the boundary are flagged as portals. When converting to Godot's format, ensure that boundary vertices are snapped to exact tile-edge positions so the NavigationServer can match them.

If vertex positions don't match exactly due to floating-point precision, Godot provides `NavigationServer3D.map_set_edge_connection_margin()` to control the tolerance for merging. Setting this to a small value (e.g., `cs * 0.5`) can help.

---

## Phase 4: Tile Management and Streaming

### Chunk ↔ Region mapping

Each godot-voxel chunk at LOD 0 maps to one NavigationServer3D region. The region's RID is stored alongside other chunk metadata.

```
chunk (cx, cz) at LOD 0  →  NavigationServer3D region RID
```

Only LOD 0 chunks within a configurable navigation range need regions. Distant chunks don't get navmesh data.

### Lifecycle

| Event | Action |
|-------|--------|
| Chunk mesh created (enters nav range) | Build navmesh → `region_create()` + `region_set_navigation_mesh()` |
| Chunk mesh updated (terrain edit) | Rebuild navmesh → `region_set_navigation_mesh()` with new data |
| Chunk unloaded (leaves nav range) | `free(region_rid)` |

### Border overlap

When building a chunk's navmesh, the Recast heightfield is expanded by `borderSize` voxels on each side. This means the rasterization must include triangle geometry from neighboring chunks within this expanded area. Without this, walkable spans near chunk edges get incorrectly eroded by `rcErodeWalkableArea`, creating gaps in the navmesh at chunk boundaries.

The Transvoxel mesher already accesses neighboring chunk data for LOD stitching. The nav builder needs similar access — either by requesting the triangle mesh from neighboring chunks, or by reading neighbor VoxelBuffer data and running a local Transvoxel pass for just the border region.

### Threading

Navmesh generation should follow godot-voxel's existing threading model:

1. **Worker thread:** Receive triangle mesh data → run full Recast pipeline → produce `NavigationMesh` resource.
2. **Main thread:** Register/update the region with `NavigationServer3D`.

The Recast pipeline is fully self-contained and thread-safe per tile — each tile build has its own `rcContext` and intermediate data structures. Only the final `NavigationServer3D` calls must happen on the main thread.

---

## Summary of Interface Points

### Interface 1: Transvoxel mesh output → Recast rasterization
**godot-voxel provides:** Vertex positions (`float[]`), triangle indices (`int[]`), chunk world bounds
**Bridge code does:** `rcMarkWalkableTriangles()` + `rcRasterizeTriangles()` into an `rcHeightfield`

### Interface 2: Obstacle source registry → Recast rasterization
**Developer provides:** Collision meshes + transforms via `add_obstacle()` / `remove_obstacle()` / `update_obstacle_transform()`
**Bridge code does:** Spatial query for obstacles overlapping chunk AABB, then `rcRasterizeTriangles()` with `RC_NULL_AREA` into the same `rcHeightfield`
**Dirty propagation:** Adding, removing, or moving an obstacle marks affected chunks for navmesh rebuild

### Interface 3: rcHeightfield → Recast pipeline
**Input:** Populated `rcHeightfield`
**Output:** `rcPolyMesh` + `rcPolyMeshDetail`
**Key functions:** `rcFilter*`, `rcBuildCompactHeightfield`, `rcErodeWalkableArea`, `rcBuildRegions`, `rcBuildContours`, `rcBuildPolyMesh`, `rcBuildPolyMeshDetail`

### Interface 4: rcPolyMesh → Godot NavigationMesh
**Input:** `rcPolyMesh` vertices (voxel-space) and polygon indices
**Output:** `NavigationMesh` resource with world-space vertices and polygon arrays
**Key conversion:** Voxel coordinates → world space via `bmin + v * cs/ch`

### Interface 5: NavigationMesh → NavigationServer3D
**Input:** `NavigationMesh` resource
**Output:** Registered navigation region on the world's navigation map
**Key functions:** `region_create()`, `region_set_map()`, `region_set_navigation_mesh()`
**Stitching:** NavigationServer auto-merges regions with overlapping edge vertices

### Interface 6: Chunk lifecycle → Region management
**Input:** Chunk load/unload/edit events from `VoxelLodTerrain` or `VoxelTerrain`
**Output:** Region create/update/free calls to NavigationServer3D
**Hook:** `mesh_update_notification` feature branch or polling for dirty chunks
