#include "nav_mesh_build_task.h"

#ifdef VOXEL_ENABLE_NAVIGATION

#include "../../util/io/log.h"
#include "../../util/string/format.h"

namespace zylann::voxel {

void NavMeshBuildTask::run(ThreadedTaskContext &ctx) {
	// Stub — Phase 4 will implement the Recast pipeline here
	ZN_PRINT_VERBOSE(format("NavMeshBuildTask::run() chunk=({},{},{}) triangles={} neighbors={} obstacles={}",
			chunk_position.x, chunk_position.y, chunk_position.z,
			chunk_triangles.indices.size() / 3,
			neighbor_triangles.size(),
			obstacles.size()));
	// result_nav_mesh remains null — no actual Recast work yet
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
