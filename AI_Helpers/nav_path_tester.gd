@tool
extends Node3D
## Drop two Marker3D children (or any Node3D) into the scene, assign them,
## and this draws the NavigationServer3D path between them.

@export var start_marker: Node3D
@export var end_marker: Node3D
@export var navigation_layers: int = 1
@export var path_color: Color = Color.RED
@export var path_width: float = 0.15
@export var update_every_frame: bool = true

var _mesh_instance: MeshInstance3D
var _immediate_mesh: ImmediateMesh
var _material: StandardMaterial3D
var _last_start: Vector3
var _last_end: Vector3

func _ready() -> void:
	_mesh_instance = MeshInstance3D.new()
	_immediate_mesh = ImmediateMesh.new()
	_material = StandardMaterial3D.new()
	_material.albedo_color = path_color
	_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_material.no_depth_test = true
	_mesh_instance.mesh = _immediate_mesh
	_mesh_instance.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(_mesh_instance)

func _process(_delta: float) -> void:
	if not start_marker or not end_marker:
		return
	var s = start_marker.global_position
	var e = end_marker.global_position
	if not update_every_frame and s.is_equal_approx(_last_start) and e.is_equal_approx(_last_end):
		return
	_last_start = s
	_last_end = e
	_update_path(s, e)

func _update_path(from: Vector3, to: Vector3) -> void:
	var maps = NavigationServer3D.get_maps()
	if maps.size() == 0:
		print("NavPathTester: No navigation maps found")
		return
	var map_rid = maps[0]
	if not map_rid.is_valid():
		print("NavPathTester: Map RID invalid")
		return
	var regions = NavigationServer3D.map_get_regions(map_rid)
	print("NavPathTester: map has %d regions, querying from %s to %s" % [regions.size(), from, to])
	var closest_start = NavigationServer3D.map_get_closest_point(map_rid, from)
	var closest_end = NavigationServer3D.map_get_closest_point(map_rid, to)
	print("NavPathTester: closest nav point to start: %s (dist %.2f)" % [closest_start, from.distance_to(closest_start)])
	print("NavPathTester: closest nav point to end: %s (dist %.2f)" % [closest_end, to.distance_to(closest_end)])
	var path = NavigationServer3D.map_get_path(map_rid, from, to, true, navigation_layers)
	print("NavPathTester: path has %d points" % path.size())
	_draw_path(path)

func _draw_path(path: PackedVector3Array) -> void:
	_immediate_mesh.clear_surfaces()
	if path.size() < 2:
		return
	# Draw as triangle strip facing camera for visibility
	var cam = get_viewport().get_camera_3d()
	if not cam:
		return
	_immediate_mesh.surface_begin(Mesh.PRIMITIVE_TRIANGLE_STRIP, _material)
	var hw = path_width * 0.5
	for i in path.size():
		var p = path[i]
		# Offset slightly above surface
		p.y += 0.05
		var forward: Vector3
		if i < path.size() - 1:
			forward = (path[i + 1] - path[i]).normalized()
		else:
			forward = (path[i] - path[i - 1]).normalized()
		var to_cam = (cam.global_position - p).normalized()
		var right = forward.cross(to_cam).normalized() * hw
		_immediate_mesh.surface_add_vertex(p - right)
		_immediate_mesh.surface_add_vertex(p + right)
	_immediate_mesh.surface_end()
