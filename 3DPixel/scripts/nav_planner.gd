class_name NavPlanner
extends RefCounted
## Grid path planner for click-to-walk. Samples the ground height on a coarse grid with downward
## raycasts (terrain and obstacles are both on the ground layer), marks every cell that rises more
## than the robot can step up from a neighbour as an obstacle, inflates the obstacles by the robot's
## radius and runs A* over the free cells. The result is string-pulled into a few waypoints.

const CELL := 0.05  ## m
const MARGIN := 1.5  ## m of free space around the start / goal box the grid covers
const MAX_SPAN := 12.0  ## m, the grid never grows beyond this on either axis
const RAY_TOP := 3.0
const RAY_BOTTOM := -1.0
const SOFT_BAND := 0.15  ## m beyond the robot radius where cells cost extra, keeps paths off edges

var origin := Vector2.ZERO  ## world XZ of cell (0, 0)'s center
var width := 0
var depth := 0
var heights := PackedFloat32Array()
var clearance := PackedFloat32Array()  ## m to the nearest obstacle cell

var _radius := 0.4


## Plans from `start` to `goal` (world positions). `max_climb` is the tallest rise the robot can
## step over, `radius` its footprint radius. `extra_blocked` are world points (with a radius in .y
## of a Vector3(x, r, z)) treated as obstacles too, e.g. spots where the robot got stuck.
## Returns the waypoints after the start, the last one is the goal; empty if there is no path.
func plan(space: PhysicsDirectSpaceState3D, start: Vector3, goal: Vector3, max_climb: float, radius: float, extra_blocked: Array[Vector3] = []) -> PackedVector3Array:
	_radius = radius
	var lo := Vector2(minf(start.x, goal.x), minf(start.z, goal.z)) - Vector2(MARGIN, MARGIN)
	var hi := Vector2(maxf(start.x, goal.x), maxf(start.z, goal.z)) + Vector2(MARGIN, MARGIN)
	var center := (lo + hi) * 0.5
	var span := (hi - lo).clamp(Vector2.ZERO, Vector2(MAX_SPAN, MAX_SPAN))
	width = int(ceil(span.x / CELL)) + 1
	depth = int(ceil(span.y / CELL)) + 1
	origin = center - Vector2(width - 1, depth - 1) * CELL * 0.5
	_sample_heights(space)
	_compute_clearance(max_climb, extra_blocked)

	var s := _cell_of(start)
	var g := _cell_of(goal)
	if s < 0 or g < 0:
		return PackedVector3Array()
	# A goal inside an obstacle's inflated zone: plan to the nearest reachable free cell, then walk
	# the last bit straight (unless the goal is on the obstacle itself).
	var goal_cell := g
	if clearance[g] < _radius:
		goal_cell = _nearest_free(g)
		if goal_cell < 0:
			return PackedVector3Array()
	var cells := _astar(s, goal_cell)
	if cells.is_empty():
		return PackedVector3Array()
	cells = _string_pull(cells)
	var out := PackedVector3Array()
	for i in range(1, cells.size()):
		out.append(cell_world(cells[i]))
	if goal_cell != g and clearance[g] > 0.0:
		out.append(cell_world(g))
	if out.is_empty():
		out.append(cell_world(g))
	# The last waypoint is the exact clicked point, not its cell center.
	out[out.size() - 1] = Vector3(goal.x, height_at(goal), goal.z)
	return out


## Ground height under a world point, from the sampled grid.
func height_at(p: Vector3) -> float:
	var c := _cell_of(p)
	return heights[c] if c >= 0 else 0.0


func cell_world(c: int) -> Vector3:
	var x := c % width
	var z := c / width
	return Vector3(origin.x + x * CELL, heights[c], origin.y + z * CELL)


func _cell_of(p: Vector3) -> int:
	var x := roundi((p.x - origin.x) / CELL)
	var z := roundi((p.z - origin.y) / CELL)
	if x < 0 or z < 0 or x >= width or z >= depth:
		return -1
	return z * width + x


func _sample_heights(space: PhysicsDirectSpaceState3D) -> void:
	heights.resize(width * depth)
	var query := PhysicsRayQueryParameters3D.new()
	query.collision_mask = SpiderRobot.GROUND_LAYER
	for z in depth:
		for x in width:
			var wx := origin.x + x * CELL
			var wz := origin.y + z * CELL
			query.from = Vector3(wx, RAY_TOP, wz)
			query.to = Vector3(wx, RAY_BOTTOM, wz)
			var hit := space.intersect_ray(query)
			heights[z * width + x] = (hit["position"] as Vector3).y if not hit.is_empty() else RAY_BOTTOM


## A cell is an obstacle when it stands more than `max_climb` above one of its 8 neighbours
## (a wall, a crate, a terrain ledge). Then a two-pass chamfer distance transform gives every
## cell its distance to the nearest obstacle cell.
func _compute_clearance(max_climb: float, extra_blocked: Array[Vector3]) -> void:
	var n := width * depth
	clearance.resize(n)
	var far := 1e6
	for z in depth:
		for x in width:
			var c := z * width + x
			var h := heights[c]
			var blocked := h <= RAY_BOTTOM + 0.01  # no ground: a hole
			for dz in [-1, 0, 1]:
				for dx in [-1, 0, 1]:
					var nx: int = x + dx
					var nz: int = z + dz
					if nx < 0 or nz < 0 or nx >= width or nz >= depth:
						continue
					if h - heights[nz * width + nx] > max_climb:
						blocked = true
			clearance[c] = 0.0 if blocked else far
	for spot in extra_blocked:
		var r := spot.y
		var cs := int(ceil(r / CELL))
		var cx := roundi((spot.x - origin.x) / CELL)
		var cz := roundi((spot.z - origin.y) / CELL)
		for z in range(maxi(cz - cs, 0), mini(cz + cs + 1, depth)):
			for x in range(maxi(cx - cs, 0), mini(cx + cs + 1, width)):
				if Vector2(x - cx, z - cz).length() * CELL <= r:
					clearance[z * width + x] = 0.0
	var diag := CELL * sqrt(2.0)
	for z in depth:
		for x in width:
			var c := z * width + x
			var d := clearance[c]
			if x > 0:
				d = minf(d, clearance[c - 1] + CELL)
			if z > 0:
				d = minf(d, clearance[c - width] + CELL)
				if x > 0:
					d = minf(d, clearance[c - width - 1] + diag)
				if x < width - 1:
					d = minf(d, clearance[c - width + 1] + diag)
			clearance[c] = d
	for z in range(depth - 1, -1, -1):
		for x in range(width - 1, -1, -1):
			var c := z * width + x
			var d := clearance[c]
			if x < width - 1:
				d = minf(d, clearance[c + 1] + CELL)
			if z < depth - 1:
				d = minf(d, clearance[c + width] + CELL)
				if x < width - 1:
					d = minf(d, clearance[c + width + 1] + diag)
				if x > 0:
					d = minf(d, clearance[c + width - 1] + diag)
			clearance[c] = d


func _nearest_free(from: int) -> int:
	var seen := {from: true}
	var queue: Array[int] = [from]
	var head := 0
	while head < queue.size():
		var c := queue[head]
		head += 1
		if clearance[c] >= _radius:
			return c
		for nb in _neighbours(c):
			if not seen.has(nb) and clearance[nb] > 0.0:
				seen[nb] = true
				queue.append(nb)
	return -1


func _neighbours(c: int) -> Array[int]:
	var out: Array[int] = []
	var x := c % width
	var z := c / width
	for dz in [-1, 0, 1]:
		for dx in [-1, 0, 1]:
			if dx == 0 and dz == 0:
				continue
			var nx: int = x + dx
			var nz: int = z + dz
			if nx >= 0 and nz >= 0 and nx < width and nz < depth:
				out.append(nz * width + nx)
	return out


## Cells closer than the robot radius to an obstacle are blocked. A start inside that zone (the
## robot standing next to a wall) may still move to cells that are no closer to the obstacle,
## so it can back out.
func _can_enter(from: int, to: int) -> bool:
	if clearance[to] >= _radius:
		return true
	return clearance[to] > 0.0 and clearance[to] >= clearance[from] - 1e-4


func _cost(c: int) -> float:
	var extra := clampf((_radius + SOFT_BAND - clearance[c]) / SOFT_BAND, 0.0, 1.0)
	return 1.0 + 4.0 * extra * extra


func _astar(s: int, g: int) -> PackedInt32Array:
	var n := width * depth
	var g_cost := PackedFloat32Array()
	g_cost.resize(n)
	g_cost.fill(INF)
	var came := PackedInt32Array()
	came.resize(n)
	came.fill(-1)
	var closed := PackedByteArray()
	closed.resize(n)
	var goal_pos := Vector2(g % width, g / width)
	var heap := _Heap.new()
	g_cost[s] = 0.0
	heap.push(0.0, s)
	while not heap.is_empty():
		var c := heap.pop()
		if closed[c]:
			continue
		if c == g:
			break
		closed[c] = 1
		var cp := Vector2(c % width, c / width)
		for nb in _neighbours(c):
			if closed[nb] or not _can_enter(c, nb):
				continue
			var np := Vector2(nb % width, nb / width)
			var step := cp.distance_to(np) * _cost(nb)
			var cost := g_cost[c] + step
			if cost < g_cost[nb]:
				g_cost[nb] = cost
				came[nb] = c
				heap.push(cost + np.distance_to(goal_pos), nb)
	if came[g] < 0 and g != s:
		return PackedInt32Array()
	var path := PackedInt32Array()
	var c := g
	while c >= 0:
		path.append(c)
		c = came[c]
	path.reverse()
	return path


## Drops waypoints that a straight line through free cells can skip.
func _string_pull(cells: PackedInt32Array) -> PackedInt32Array:
	var out := PackedInt32Array([cells[0]])
	var i := 0
	while i < cells.size() - 1:
		var j := cells.size() - 1
		while j > i + 1 and not _line_free(cells[i], cells[j]):
			j -= 1
		out.append(cells[j])
		i = j
	return out


func _line_free(a: int, b: int) -> bool:
	var pa := Vector2(a % width, a / width)
	var pb := Vector2(b % width, b / width)
	var steps := int(ceil(pa.distance_to(pb) * 2.0))
	var prev := a
	for k in range(1, steps + 1):
		var p := pa.lerp(pb, float(k) / steps)
		var c := roundi(p.y) * width + roundi(p.x)
		if c != prev:
			# Shortcuts keep half the soft band of clearance, except while backing out of it.
			var roomy := clearance[c] >= _radius + SOFT_BAND * 0.5
			if not roomy and not (_can_enter(prev, c) and clearance[c] >= clearance[prev]):
				return false
			prev = c
	return true


## Minimal binary heap of (priority, cell).
class _Heap:
	var _keys := PackedFloat32Array()
	var _vals := PackedInt32Array()

	func is_empty() -> bool:
		return _vals.is_empty()

	func push(key: float, val: int) -> void:
		_keys.append(key)
		_vals.append(val)
		var i := _vals.size() - 1
		while i > 0:
			var p := (i - 1) / 2
			if _keys[p] <= _keys[i]:
				break
			_swap(i, p)
			i = p

	func pop() -> int:
		var top := _vals[0]
		var last := _vals.size() - 1
		_swap(0, last)
		_keys.resize(last)
		_vals.resize(last)
		var i := 0
		while true:
			var l := i * 2 + 1
			var r := l + 1
			var m := i
			if l < last and _keys[l] < _keys[m]:
				m = l
			if r < last and _keys[r] < _keys[m]:
				m = r
			if m == i:
				break
			_swap(i, m)
			i = m
		return top

	func _swap(a: int, b: int) -> void:
		var k := _keys[a]
		_keys[a] = _keys[b]
		_keys[b] = k
		var v := _vals[a]
		_vals[a] = _vals[b]
		_vals[b] = v
