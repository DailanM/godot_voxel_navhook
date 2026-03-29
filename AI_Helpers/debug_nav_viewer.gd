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

@export_group("Display")
@export var show_faces: bool = true
@export var show_edges: bool = true
@export var face_color: Color = Color(0, 0.8, 0.4, 0.4)
@export var edge_color: Color = Color(1, 1, 0, 1.0)
@export var boundary_edge_color: Color = Color(0.0, 0.5, 0.8, 1.0)
@export var edge_y_offset: float = 0.02

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

	# Collect all edges across all regions to detect boundary edges (edges that
	# appear in only one region, meaning they sit at the border of a navmesh chunk).
	# Key: sorted pair of world-space vertex positions, Value: count of regions containing it
	var global_edge_counts := {}

	# First pass: gather per-region data and count global edges
	var region_data := []
	for entry in nav_meshes:
		var xform: Transform3D = entry[0]
		var nav_mesh: NavigationMesh = entry[1]
		if nav_mesh == null:
			continue
		var verts := nav_mesh.get_vertices()
		if verts.size() == 0:
			continue

		# Collect edges for this region
		var edges := []
		for i in nav_mesh.get_polygon_count():
			var poly = nav_mesh.get_polygon(i)
			for j in poly.size():
				var a_idx = poly[j]
				var b_idx = poly[(j + 1) % poly.size()]
				edges.append(Vector2i(a_idx, b_idx))
				# Count in world space for cross-region matching
				var wa = xform * verts[a_idx]
				var wb = xform * verts[b_idx]
				var edge_key = _make_edge_key(wa, wb)
				global_edge_counts[edge_key] = global_edge_counts.get(edge_key, 0) + 1

		region_data.append({
			"xform": xform,
			"nav_mesh": nav_mesh,
			"verts": verts,
			"edges": edges,
		})

	# Second pass: build meshes
	for rd in region_data:
		var xform: Transform3D = rd["xform"]
		var nav_mesh: NavigationMesh = rd["nav_mesh"]
		var verts: PackedVector3Array = rd["verts"]
		var edges: Array = rd["edges"]

		# --- Face mesh ---
		if show_faces:
			var indices := PackedInt32Array()
			for i in nav_mesh.get_polygon_count():
				var poly = nav_mesh.get_polygon(i)
				for j in range(1, poly.size() - 1):
					indices.append(poly[0])
					indices.append(poly[j])
					indices.append(poly[j + 1])

			if indices.size() > 0:
				var arr := []
				arr.resize(Mesh.ARRAY_MAX)
				arr[Mesh.ARRAY_VERTEX] = verts
				arr[Mesh.ARRAY_INDEX] = indices

				var arr_mesh := ArrayMesh.new()
				arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)

				var mat := StandardMaterial3D.new()
				mat.albedo_color = face_color
				mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
				mat.cull_mode = BaseMaterial3D.CULL_DISABLED
				mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
				arr_mesh.surface_set_material(0, mat)

				var mesh_inst := MeshInstance3D.new()
				mesh_inst.mesh = arr_mesh
				mesh_inst.transform = xform
				add_child(mesh_inst)
				_instances.append(mesh_inst)

		# --- Edge wireframe ---
		if show_edges and edges.size() > 0:
			# Separate interior vs boundary edges
			var interior_verts := PackedVector3Array()
			var boundary_verts := PackedVector3Array()

			for e in edges:
				var a: Vector3 = verts[e.x]
				var b: Vector3 = verts[e.y]
				a.y += edge_y_offset
				b.y += edge_y_offset

				var wa = xform * verts[e.x]
				var wb = xform * verts[e.y]
				var edge_key = _make_edge_key(wa, wb)
				var count = global_edge_counts.get(edge_key, 0)

				# An edge shared by 2 polygons within the same region that also appears
				# in another region is interior. An edge appearing in only 1 region
				# globally is a boundary/free edge.
				if count <= 1:
					boundary_verts.append(a)
					boundary_verts.append(b)
				else:
					interior_verts.append(a)
					interior_verts.append(b)

			if interior_verts.size() > 0:
				_add_line_mesh(interior_verts, edge_color, xform)

			if boundary_verts.size() > 0:
				_add_line_mesh(boundary_verts, boundary_edge_color, xform)

	print("debug_nav_viewer: created ", _instances.size(), " mesh instances")


func _add_line_mesh(line_verts: PackedVector3Array, color: Color, xform: Transform3D) -> void:
	var arr := []
	arr.resize(Mesh.ARRAY_MAX)
	arr[Mesh.ARRAY_VERTEX] = line_verts

	var arr_mesh := ArrayMesh.new()
	arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_LINES, arr)

	var mat := StandardMaterial3D.new()
	mat.albedo_color = color
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.no_depth_test = true
	arr_mesh.surface_set_material(0, mat)

	var mesh_inst := MeshInstance3D.new()
	mesh_inst.mesh = arr_mesh
	mesh_inst.transform = xform
	add_child(mesh_inst)
	_instances.append(mesh_inst)


func _make_edge_key(a: Vector3, b: Vector3) -> String:
	# Snap to a grid to handle float imprecision, then sort for order-independence
	var sa = _snap_vec(a)
	var sb = _snap_vec(b)
	if sa < sb:
		return "%s|%s" % [sa, sb]
	return "%s|%s" % [sb, sa]


func _snap_vec(v: Vector3) -> String:
	# Snap to 0.01 precision (finer than typical cell_size * merge_scale)
	return "%d,%d,%d" % [roundi(v.x * 100), roundi(v.y * 100), roundi(v.z * 100)]


func _clear() -> void:
	for inst in _instances:
		inst.queue_free()
	_instances.clear()

func _ready() -> void:
	pass
