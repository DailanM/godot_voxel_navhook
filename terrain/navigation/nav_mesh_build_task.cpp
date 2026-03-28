#include "nav_mesh_build_task.h"

#ifdef VOXEL_ENABLE_NAVIGATION

namespace zylann::voxel {

void NavMeshBuildTask::run(ThreadedTaskContext &ctx) {
	// Stub — Phase 4 (Recast pipeline)
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
