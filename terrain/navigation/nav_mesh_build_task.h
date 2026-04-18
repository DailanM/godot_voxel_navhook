#ifndef VOXEL_NAV_MESH_BUILD_TASK_H
#define VOXEL_NAV_MESH_BUILD_TASK_H

#ifdef VOXEL_ENABLE_NAVIGATION

#include "nav_mesh_manager.h"
#include "../../util/containers/std_vector.h"
#include "../../util/tasks/threaded_task.h"

#include "scene/resources/navigation_mesh.h"

#include <Recast.h>

#include <memory>

namespace zylann::voxel {

class NavMeshBuildTask : public IThreadedTask {
public:
	// Input (set before dispatch, immutable during run)
	Vector3i chunk_position;
	uint32_t build_generation = 0;
	NavChunkData chunk_triangles;
	StdVector<NavChunkData> neighbor_triangles;
	StdVector<NavMeshManager::ObstacleEntry> obstacles;
	rcConfig cfg = {};
	bool filter_low_hanging = true;
	bool filter_ledge_spans = true;
	bool filter_low_height_spans = true;
	bool use_erosion = true;
	bool use_detail_mesh = true;

	// Output
	Ref<NavigationMesh> result_nav_mesh;

	// Dependency
	std::shared_ptr<NavMeshManager> nav_mesh_manager;

	void run(ThreadedTaskContext &ctx) override;
	void apply_result() override;
	TaskPriority get_priority() override;
	bool is_cancelled() override;
	const char *get_debug_name() const override { return "NavMeshBuild"; }
};

} // namespace zylann::voxel

#endif // VOXEL_ENABLE_NAVIGATION

#endif // VOXEL_NAV_MESH_BUILD_TASK_H
