# debug_nav_viewer.gd
# Attach to any Node3D in the scene. Set terrain_path to your VoxelTerrain node.
# Toggle "refresh" in the inspector to update the visualization.
# Call refresh_nav() from the console to update manually.
@tool
extends Node3D


@export var terrain_path: NodePath
@export var refresh: bool = false:
	set(v):
		refresh = false
		refresh_nav()

var _instances: Array[MeshInstance3D] = []

func refresh_nav() -> void:
	_clear()

	if terrain_path.is_empty():
		push_warning("debug_nav_viewer: terrain_path not set")
		return

	var terrain = get_node_or_null(terrain_path)
	if terrain == null:
		push_warning("debug_nav_viewer: terrain node not found")
		return

	if not terrain.has_method("debug_get_nav_meshes"):
		push_warning("debug_nav_viewer: terrain has no debug_get_nav_meshes method")
		return

	var nav_meshes: Array = terrain.debug_get_nav_meshes()
	print("debug_nav_viewer: drawing ", nav_meshes.size(), " nav regions")

	for entry in nav_meshes:
		var xform: Transform3D = entry[0]
		var nav_mesh: NavigationMesh = entry[1]

		if nav_mesh == null:
			continue

		var verts := nav_mesh.get_vertices()
		if verts.size() == 0:
			continue

		# Build index array from polygons
		var indices := PackedInt32Array()
		for i in nav_mesh.get_polygon_count():
			var poly = nav_mesh.get_polygon(i)
			# Fan triangulation for polygons with more than 3 verts
			for j in range(1, poly.size() - 1):
				indices.append(poly[0])
				indices.append(poly[j])
				indices.append(poly[j + 1])

		if indices.size() == 0:
			continue

		var arr := []
		arr.resize(Mesh.ARRAY_MAX)
		arr[Mesh.ARRAY_VERTEX] = verts
		arr[Mesh.ARRAY_INDEX] = indices

		var arr_mesh := ArrayMesh.new()
		arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)

		var mat := StandardMaterial3D.new()
		mat.albedo_color = Color(0, 0.8, 0.4, 0.4)
		mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		mat.cull_mode = BaseMaterial3D.CULL_DISABLED
		mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		arr_mesh.surface_set_material(0, mat)

		var mesh_inst := MeshInstance3D.new()
		mesh_inst.mesh = arr_mesh
		mesh_inst.transform = xform
		add_child(mesh_inst)
		_instances.append(mesh_inst)

	print("debug_nav_viewer: created ", _instances.size(), " mesh instances")

func _clear() -> void:
	for inst in _instances:
		inst.queue_free()
	_instances.clear()

func _ready() -> void:
	print("Howdy")
