# chunk_locator.gd
# Attach to any Node3D. Displays which voxel chunk (mesh block coords)
# the node currently occupies. Set terrain_path to your VoxelTerrain node.
@tool
extends Node3D

@export var terrain_path: NodePath
## Fallback block size if terrain node is unavailable.
@export var fallback_block_size: int = 16
@export var print_chunk: bool = false:
	set(v):
		print_chunk = false
		var chunk := get_chunk_position()
		print("chunk_locator [%s]: chunk %v  (block_size=%d)" % [name, chunk, _get_block_size()])

func get_chunk_position() -> Vector3i:
	var bs := _get_block_size()
	var pos := global_position
	return Vector3i(
		floori(pos.x / bs),
		floori(pos.y / bs),
		floori(pos.z / bs),
	)

func _get_block_size() -> int:
	var terrain = get_node_or_null(terrain_path)
	if terrain != null and terrain.has_method("get_mesh_block_size"):
		return terrain.get_mesh_block_size()
	return fallback_block_size

