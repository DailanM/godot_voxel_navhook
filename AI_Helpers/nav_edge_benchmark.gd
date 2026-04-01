# nav_edge_benchmark.gd
# Attach to any Node3D in the scene. Polls NavigationServer3D performance
# counters each sync cycle and prints a summary so you can see whether the
# bottleneck is Phase 1 (HashMap, linear in total edges) or Phase 2
# (brute-force, quadratic in free/unmatched edges).
#
# "Free edges" are boundary edges that did NOT find a quantization-space
# match and must go through the expensive O(F^2) margin-based comparison.
# If free_edge_count is high relative to edge_count, vertex welding (or
# full mesh merging) will help significantly.
@tool
extends Node3D

@export var auto_poll: bool = true
@export var poll_interval: float = 2.0
@export var log_to_console: bool = true

var _timer: float = 0.0
var _prev_sync_usec: int = 0

# History for summary stats
var _samples: Array[Dictionary] = []
const MAX_SAMPLES := 60

func _process(delta: float) -> void:
	if not auto_poll:
		return
	_timer += delta
	if _timer >= poll_interval:
		_timer = 0.0
		poll()

func poll() -> void:
	var ns := NavigationServer3D

	var info := {
		"region_count": ns.get_process_info(ns.INFO_REGION_COUNT),
		"edge_count": ns.get_process_info(ns.INFO_EDGE_COUNT),
		"edge_merge_count": ns.get_process_info(ns.INFO_EDGE_MERGE_COUNT),
		"edge_connection_count": ns.get_process_info(ns.INFO_EDGE_CONNECTION_COUNT),
		"edge_free_count": ns.get_process_info(ns.INFO_EDGE_FREE_COUNT),
	}

	_samples.append(info)
	if _samples.size() > MAX_SAMPLES:
		_samples.pop_front()

	if log_to_console:
		var free_pct := 0.0
		if info["edge_count"] > 0:
			free_pct = 100.0 * info["edge_free_count"] / info["edge_count"]

		print_rich("[b]NavEdgeBenchmark[/b] | ",
			"regions: %d | " % info["region_count"],
			"edges: %d | " % info["edge_count"],
			"merged: %d | " % info["edge_merge_count"],
			"connected: %d | " % info["edge_connection_count"],
			"[color=yellow]free: %d (%.1f%%)[/color]" % [info["edge_free_count"], free_pct])

		# If free edges are high, flag it
		if free_pct > 10.0:
			print_rich("[color=red]  >> %.1f%% of edges are free — Phase 2 (O(F²) brute force) is likely the bottleneck.[/color]" % free_pct)
			print_rich("[color=red]  >> F²=%d comparisons vs E=%d HashMap lookups.[/color]" % [
				info["edge_free_count"] * info["edge_free_count"],
				info["edge_count"]])
		elif info["edge_count"] > 0:
			print_rich("[color=green]  >> Free edges low — Phase 1 (O(E) HashMap) dominates. Reducing total region/edge count is the path to optimization.[/color]")

func print_summary() -> void:
	if _samples.is_empty():
		print("NavEdgeBenchmark: no samples yet")
		return

	var totals := {"region_count": 0, "edge_count": 0, "edge_merge_count": 0, "edge_connection_count": 0, "edge_free_count": 0}
	var peak_free := 0
	var peak_edges := 0
	for s in _samples:
		for k in totals:
			totals[k] += s[k]
		if s["edge_free_count"] > peak_free:
			peak_free = s["edge_free_count"]
			peak_edges = s["edge_count"]

	var n := _samples.size()
	print_rich("\n[b]===== NavEdgeBenchmark Summary (%d samples) =====[/b]" % n)
	print("  Avg regions:     %.1f" % (float(totals["region_count"]) / n))
	print("  Avg edges:       %.1f" % (float(totals["edge_count"]) / n))
	print("  Avg merged:      %.1f" % (float(totals["edge_merge_count"]) / n))
	print("  Avg connected:   %.1f" % (float(totals["edge_connection_count"]) / n))
	print("  Avg free:        %.1f" % (float(totals["edge_free_count"]) / n))
	print("  Peak free:       %d / %d edges (%.1f%%)" % [peak_free, peak_edges,
		100.0 * peak_free / max(peak_edges, 1)])
	print_rich("[b]================================================[/b]\n")
