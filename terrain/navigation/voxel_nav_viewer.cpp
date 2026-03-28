#include "voxel_nav_viewer.h"

#ifdef VOXEL_ENABLE_NAVIGATION

#include "../../engine/voxel_engine.h"
#include "../../util/godot/classes/engine.h"
#include "../../util/godot/classes/node.h"

#ifdef ZN_GODOT
#include "../../util/godot/core/callable_mp.h"
#endif

namespace zylann::voxel {

VoxelNavViewer::VoxelNavViewer() {
	set_notify_transform(!Engine::get_singleton()->is_editor_hint());
}

void VoxelNavViewer::set_nav_distance(unsigned int distance) {
	_nav_distance = distance;
	if (is_active()) {
		VoxelEngine::get_singleton().set_nav_viewer_distance(_viewer_id, distance);
	}
}

unsigned int VoxelNavViewer::get_nav_distance() const {
	return _nav_distance;
}

void VoxelNavViewer::set_enabled_in_editor(bool enable) {
	if (_enabled_in_editor == enable) {
		return;
	}

	_enabled_in_editor = enable;

#ifdef TOOLS_ENABLED
	if (Engine::get_singleton()->is_editor_hint()) {
		set_notify_transform(_enabled_in_editor);

		if (is_inside_tree()) {
			if (_enabled_in_editor) {
				_viewer_id = VoxelEngine::get_singleton().add_nav_viewer();
				sync_all_parameters();
			} else {
				VoxelEngine::get_singleton().remove_nav_viewer(_viewer_id);
			}
		}
	}
#endif
}

bool VoxelNavViewer::is_enabled_in_editor() const {
	return _enabled_in_editor;
}

void VoxelNavViewer::sync_all_parameters() {
	VoxelEngine &ve = VoxelEngine::get_singleton();
	ve.set_nav_viewer_position(_viewer_id, get_global_transform().origin);
	ve.set_nav_viewer_distance(_viewer_id, _nav_distance);
}

void VoxelNavViewer::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			if (!Engine::get_singleton()->is_editor_hint() || _enabled_in_editor) {
				if (!_pending_deferred_unregistration) {
					_viewer_id = VoxelEngine::get_singleton().add_nav_viewer();
				} else {
					ZN_ASSERT_RETURN(VoxelEngine::get_singleton().nav_viewer_exists(_viewer_id));
				}
				sync_all_parameters();
			}
		} break;

		case NOTIFICATION_EXIT_TREE:
			if (!Engine::get_singleton()->is_editor_hint() || _enabled_in_editor) {
				if (!_pending_deferred_unregistration) {
					_pending_deferred_unregistration = true;
					callable_mp_static(&VoxelNavViewer::unregister_deferred_callback)
							.bind(get_instance_id(), Vector2i(_viewer_id.index, _viewer_id.version.value))
							.call_deferred();
				}
			}
			break;

		case NOTIFICATION_TRANSFORM_CHANGED:
			if (is_active()) {
				const Vector3 pos = get_global_transform().origin;
				VoxelEngine::get_singleton().set_nav_viewer_position(_viewer_id, pos);
			}
			break;

		default:
			break;
	}
}

void VoxelNavViewer::unregister_deferred_callback(const int64_t viewer_node_id, const Vector2i encoded_viewer_id) {
	Object *obj = ObjectDB::get_instance(ObjectID(viewer_node_id));
	VoxelNavViewer *viewer = Object::cast_to<VoxelNavViewer>(obj);
	if (viewer != nullptr) {
		viewer->_pending_deferred_unregistration = false;
		if (viewer->is_inside_tree()) {
			// Still in tree — was reparented, don't unregister
			return;
		}
	}
	// The node got removed and not added back, or was destroyed
	NavViewerID viewer_id;
	viewer_id.index = encoded_viewer_id.x;
	viewer_id.version.value = encoded_viewer_id.y;
	ZN_ASSERT_RETURN(VoxelEngine::get_singleton().nav_viewer_exists(viewer_id));
	VoxelEngine::get_singleton().remove_nav_viewer(viewer_id);
}

bool VoxelNavViewer::is_active() const {
	return is_inside_tree() && (!Engine::get_singleton()->is_editor_hint() || _enabled_in_editor);
}

void VoxelNavViewer::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_nav_distance", "distance"), &VoxelNavViewer::set_nav_distance);
	ClassDB::bind_method(D_METHOD("get_nav_distance"), &VoxelNavViewer::get_nav_distance);

	ClassDB::bind_method(D_METHOD("set_enabled_in_editor", "enabled"), &VoxelNavViewer::set_enabled_in_editor);
	ClassDB::bind_method(D_METHOD("is_enabled_in_editor"), &VoxelNavViewer::is_enabled_in_editor);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "nav_distance"), "set_nav_distance", "get_nav_distance");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "enabled_in_editor"), "set_enabled_in_editor", "is_enabled_in_editor");
}

} // namespace zylann::voxel

#endif // VOXEL_ENABLE_NAVIGATION
