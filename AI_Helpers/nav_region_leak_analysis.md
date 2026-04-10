# Nav Region RID Leak Analysis

## Symptom

Performance degrades after some terrain regeneration. Edge count seems to grow, suggesting NavigationServer region RIDs are leaking.

## Hypothesis: Stale apply_result() Creates Orphan Regions

### The Race Condition

When terrain regenerates, `stop_updater()` is called:

```
stop_updater()
  → MeshingDependency::reset(...)        // invalidates old dependency
  → _nav_mesh_manager->clear_all()       // frees all region RIDs, clears _regions HashMap
```

**But `clear_all()` does NOT set `nav_mesh_manager->valid = false`.** Only `set_generate_navigation(false)` and `~VoxelTerrain()` do that.

This means in-flight `NavMeshBuildTask`s are NOT cancelled. Their `is_cancelled()` checks `nav_mesh_manager->valid`, which is still `true`.

### Timeline

```
T0: stop_updater()
    - clear_all() frees all region RIDs, clears _regions HashMap
    - MeshingDependency::reset() invalidates old dependency (prevents new mesh tasks)
    - But NavMeshBuildTasks already completed and queued for apply_result() are unaffected

T1: start_updater()
    - New mesh tasks begin

T2: Old NavMeshBuildTask::apply_result() fires on main thread
    - Calls nav_mesh_manager->apply_nav_result(chunk_pos, nav_mesh, generation)
    - _regions HashMap is empty (was cleared at T0)
    - !region.rid.is_valid() → creates a NEW region RID
    - This RID is stored in _regions[chunk_pos]

T3: New mesh tasks complete → new NavMeshBuildTasks dispatch and complete

T4: New NavMeshBuildTask::apply_result() fires
    - _regions[chunk_pos] already has the RID from T2
    - Calls region_set_navigation_mesh() on that RID — updates in place
    - No leak in this specific case

T5: BUT — if old tasks for DIFFERENT chunk positions arrive after new tasks
    have already populated those positions:
    - Old task creates RID at T2 for chunk (3,0,2)
    - New task arrives at T4, finds RID already exists, updates it
    - No leak here either
```

### Wait — Is There Actually a Leak?

On closer analysis, the race may not produce a leak in the simple case because:
- `clear_all()` clears the HashMap
- Old `apply_result()` creates a new entry
- New `apply_result()` overwrites it (reuses the RID)

**The leak happens when:**
1. Old task A creates a region RID for chunk_pos X at T2
2. `clear_all()` is called AGAIN (second regeneration) at T3
3. `clear_all()` at T3 frees the RID from step 1 — **this is correct**

So the HashMap-based lifecycle might actually be safe for the simple case. However:

### Alternative Leak Path: Generation Counter Mismatch

```
T0: Chunk (1,0,1) has generation=5, nav build dispatched with generation=5
T1: stop_updater() → clear_all() → _applied_generations cleared
T2: start_updater() → chunk re-meshes → on_mesh_built() → generation=1 (reset)
    → nav build dispatched with generation=1
T3: Old build (generation=5) apply_result() arrives
    - _applied_generations is empty → no staleness check fails
    - Creates region RID, stores generation=5 in _applied_generations
T4: New build (generation=1) apply_result() arrives
    - _applied_generations[chunk_pos] = 5
    - build_generation (1) <= applied (5) → SKIPPED as stale!
    - The old navmesh stays, new one is silently dropped
```

This is a correctness bug, not a leak. But it means the navmesh never updates after regeneration until generation catches up.

### Another Alternative: clear_all() Doesn't Free Deferred Commands

`NavigationServer3D::free_rid()` is a `COMMAND_1` — it's deferred, not immediate. Meanwhile `region_create()` is immediate. So:

```
T0: clear_all() queues free_rid(old_rid_A) for chunk (1,0,1)
T1: Old apply_result() runs: region_create() → new_rid_B, stored in _regions
T2: physics frame: deferred commands flush
    - free_rid(old_rid_A) executes — frees the OLD rid (correct)
    - region_set_navigation_mesh(new_rid_B, ...) executes — sets up the new one
```

This ordering is actually fine. The deferred free targets the old RID by value, not by HashMap lookup.

### Most Likely Actual Leak: _chunk_cache Not Cleared Before New Builds

`clear_all()` clears `_chunk_cache`. But `on_mesh_built()` runs on worker threads. The sequence:

```
T0: Worker thread completes mesh for chunk (1,0,1), about to call on_mesh_built()
T1: Main thread: stop_updater() → clear_all() → _chunk_cache cleared
T2: Worker thread: on_mesh_built() → caches data, bumps generation to 1
    → checks neighbors → neighbors not ready (were cleared) → no dispatch
T3: Main thread: start_updater() → new mesh tasks begin
T4: New worker: on_mesh_built() for chunk (1,0,1) → generation bumps to 2
T5: More chunks arrive → neighborhood completes → dispatch with generation 2
T6: apply_result() with generation 2 → creates region, stores gen=2
```

No leak here either. The generation counter handles this correctly because `_applied_generations` was also cleared.

## Summary: Confidence Level

The analysis suggests several potential issues but no definitive single leak path. The most suspicious scenarios are:

1. **Generation counter inversion** (T3 above): Old builds with high generation numbers arrive after `clear_all()` resets everything, causing new builds (with low generation numbers) to be rejected as stale. This would manifest as chunks with permanently outdated navmeshes, but not as a region count increase.

2. **Multiple in-flight tasks for the same chunk**: If `_dispatch_nav_build()` can fire multiple times for the same chunk before the first completes (no dedup guard), each `apply_result()` would check the generation counter — but if they all have the same generation, only the first creates the region. Others would see `region.rid.is_valid()` and update in place. No leak.

3. **Something outside NavMeshManager**: The performance degradation could be in the NavigationServer's internal bookkeeping even with correct RID lifecycle. Worth verifying by checking `NavigationServer3D.get_process_info(INFO_REGION_COUNT)` across regenerations.

## Suggested Debugging Steps

1. **Log region count per regeneration**: Add `NavigationServer3D.get_process_info(INFO_REGION_COUNT)` to the benchmark script. If it grows across regenerations, regions are leaking.

2. **Log _regions HashMap size**: Add a debug method on NavMeshManager to report `_regions.size()`. If this stays constant but server region count grows, the leak is from orphan RIDs created by stale `apply_result()` calls.

3. **Log apply_result() calls**: Print chunk_pos and build_generation on every `apply_result()`. Look for calls with unexpectedly high generation numbers after a regeneration (the generation inversion issue).

4. **Test with valid=false**: Temporarily add `_nav_mesh_manager->valid = false` at the start of `stop_updater()` and `_nav_mesh_manager->valid = true` after `clear_all()`. If this fixes the leak, the stale-task hypothesis is confirmed.

5. **Check if the issue is Phase 1 or Phase 2 specific**: Run with `register_with_server = false` and see if the benchmark script still shows increasing edge counts. If not, the issue is definitely in region lifecycle.
