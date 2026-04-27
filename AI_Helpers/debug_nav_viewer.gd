# debug_nav_viewer.gd
# Attach to any Node3D in the scene. Set terrain_path to your VoxelTerrain node.
# At runtime, automatically visualizes nav meshes once generation stabilizes.
# In the editor, toggle "refresh" in the inspector or call refresh_nav() manually.
@tool
extends Node3D


@export var terrain_path: NodePath
@export var refresh: bool = false:
	set(v):
		refresh = false
		refresh_nav()

@export_group("Auto-Detect (Runtime)")
## Enable polling at runtime to auto-trigger once nav mesh count stabilizes.
@export var auto_detect: bool = true
## How long (seconds) the nav mesh count must stay unchanged before we draw.
@export var settle_time: float = 2.0
## Seconds between polls while waiting for generation to finish.
@export var poll_interval: float = 0.5

@export_group("Display")
@export var show_faces: bool = true
@export var show_edges: bool = true
@export var face_color: Color = Color(0, 0.8, 0.4, 0.4)
@export var edge_color: Color = Color(1, 1, 0, 1.0)
@export var boundary_edge_color: Color = Color(0.0, 0.5, 0.8, 1.0)
@export var edge_y_offset: float = 0.02

@export_group("Region Coloring")
# 0 = Cycle through region_cycle_palette; 1 = HSV hue from region ID
@export_enum("Cycle 4", "HSV Hue") var region_color_mode: int = 0
@export var region_cycle_palette: PackedColorArray = PackedColorArray([
	Color(0.90, 0.30, 0.30, 0.5),
	Color(0.30, 0.70, 0.90, 0.5),
	Color(0.95, 0.80, 0.25, 0.5),
	Color(0.55, 0.35, 0.85, 0.5),
])

var _instances: Array[MeshInstance3D] = []
var _poll_timer: float = 0.0
var _last_nav_count: int = -1
var _stable_elapsed: float = 0.0
var _auto_done: bool = false

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
		# Third element added by debug_get_nav_meshes: one Recast region ID
		# per NavigationMesh polygon.  Fall back to empty if the module hasn't
		# been rebuilt yet.
		var poly_regions: PackedInt32Array = PackedInt32Array()
		if entry.size() >= 3 and entry[2] != null:
			poly_regions = entry[2]
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
			"poly_regions": poly_regions,
		})

	# Second pass: build meshes
	for rd in region_data:
		var xform: Transform3D = rd["xform"]
		var nav_mesh: NavigationMesh = rd["nav_mesh"]
		var verts: PackedVector3Array = rd["verts"]
		var edges: Array = rd["edges"]
		var poly_regions: PackedInt32Array = rd["poly_regions"]

		# --- Face mesh (region-colored via vertex colors) ---
		if show_faces:
			# Build an un-indexed triangle soup so each triangle can carry
			# its own (region-derived) vertex color.
			var tri_verts := PackedVector3Array()
			var tri_colors := PackedColorArray()
			var poly_count := nav_mesh.get_polygon_count()
			var have_regions := poly_regions.size() == poly_count
			for i in poly_count:
				var poly = nav_mesh.get_polygon(i)
				var region_id: int = poly_regions[i] if have_regions else 0
				var tri_color := _region_color(region_id)
				for j in range(1, poly.size() - 1):
					tri_verts.append(verts[poly[0]])
					tri_verts.append(verts[poly[j]])
					tri_verts.append(verts[poly[j + 1]])
					tri_colors.append(tri_color)
					tri_colors.append(tri_color)
					tri_colors.append(tri_color)

			if tri_verts.size() > 0:
				var arr := []
				arr.resize(Mesh.ARRAY_MAX)
				arr[Mesh.ARRAY_VERTEX] = tri_verts
				arr[Mesh.ARRAY_COLOR] = tri_colors

				var arr_mesh := ArrayMesh.new()
				arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)

				var mat := StandardMaterial3D.new()
				mat.albedo_color = Color(1, 1, 1, 1)
				mat.vertex_color_use_as_albedo = true
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


func _region_color(region_id: int) -> Color:
	match region_color_mode:
		1: # HSV Hue
			var hue := fposmod(region_id * 0.17, 1.0)
			return Color.from_hsv(hue, 0.65, 0.9, face_color.a)
		_: # Cycle palette
			if region_cycle_palette.size() == 0:
				return face_color
			return region_cycle_palette[region_id % region_cycle_palette.size()]


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
	if Engine.is_editor_hint():
		return
	if auto_detect:
		set_process(true)
		print("debug_nav_viewer: waiting for nav mesh generation to finish...")


func _process(delta: float) -> void:
	if Engine.is_editor_hint() or _auto_done:
		set_process(false)
		return

	_poll_timer += delta
	if _poll_timer < poll_interval:
		return
	_poll_timer = 0.0

	var terrain = get_node_or_null(terrain_path)
	if terrain == null or not terrain.has_method("debug_get_nav_meshes"):
		return

	var count: int = terrain.debug_get_nav_meshes().size()

	if count == 0:
		_last_nav_count = 0
		_stable_elapsed = 0.0
		return

	if count != _last_nav_count:
		if _last_nav_count >= 0:
			print("debug_nav_viewer: nav regions %d -> %d" % [_last_nav_count, count])
		_last_nav_count = count
		_stable_elapsed = 0.0
		return

	_stable_elapsed += poll_interval
	if _stable_elapsed >= settle_time:
		print("debug_nav_viewer: generation settled at %d regions, drawing..." % count)
		_auto_done = true
		set_process(false)
		refresh_nav()
