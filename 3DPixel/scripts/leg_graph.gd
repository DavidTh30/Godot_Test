class_name LegGraph
extends Control
## Foot position history per leg. One horizontal strip per selected axis (X, Y, Z); inside a
## strip every selected leg is drawn in its own color, so legs can be compared axis by axis.
## The planned (target) position can be overlaid as a faint line to see tracking error.

const CAPACITY := 360  ## samples kept (6 s at 60 Hz)
const SAMPLE_HZ := 60.0
const AXIS_TITLES := ["X  side  (+ = right)", "Y  height", "Z  fore-aft  (- = front)"]
const LEG_COLORS := {
	"L1": Color("ef5350"), "L2": Color("ffa726"), "L3": Color("ffee58"),
	"R1": Color("42a5f5"), "R2": Color("66bb6a"), "R3": Color("ce93d8"),
}

var leg_names: Array[String] = []
var leg_visible: Array[bool] = []
var axis_visible: Array[bool] = [true, true, true]
var show_target := true
var relative := true  ## plot each leg around its own average, so legs share one scale
var paused := false

var _actual: Array = []  # per leg: [PackedFloat32Array x, y, z]
var _target: Array = []


static func leg_color(leg_name: String) -> Color:
	return LEG_COLORS.get(leg_name, Color.WHITE)


## Sets the legs to plot. Keeps the per-leg selection when the leg set is unchanged.
func set_legs(names: Array[String]) -> void:
	if names != leg_names:
		leg_names = names.duplicate()
		leg_visible.clear()
		for n in names:
			leg_visible.append(true)
	clear()


func clear() -> void:
	_actual.clear()
	_target.clear()
	for n in leg_names:
		_actual.append([PackedFloat32Array(), PackedFloat32Array(), PackedFloat32Array()])
		_target.append([PackedFloat32Array(), PackedFloat32Array(), PackedFloat32Array()])
	queue_redraw()


func push(actual: Array[Vector3], target: Array[Vector3]) -> void:
	if paused or actual.size() != leg_names.size():
		return
	for i in leg_names.size():
		for a in 3:
			_actual[i][a] = _append(_actual[i][a], actual[i][a])
			_target[i][a] = _append(_target[i][a], target[i][a])


func _append(values: PackedFloat32Array, v: float) -> PackedFloat32Array:
	values.append(v)
	if values.size() > CAPACITY:
		values = values.slice(values.size() - CAPACITY)
	return values


func _draw() -> void:
	var font := get_theme_default_font()
	draw_rect(Rect2(Vector2.ZERO, size), Color(0, 0, 0, 0.35))
	var axes: Array[int] = []
	for a in 3:
		if axis_visible[a]:
			axes.append(a)
	if axes.is_empty() or not leg_visible.has(true):
		draw_string(font, Vector2(12, size.y * 0.5), "Select at least one leg and one axis", HORIZONTAL_ALIGNMENT_LEFT, -1, 14, Color(1, 1, 1, 0.6))
		return

	var gap := 6.0
	var label_w := 56.0
	var strip_h := (size.y - gap * (axes.size() + 1)) / axes.size()
	for si in axes.size():
		var a := axes[si]
		var rect := Rect2(Vector2(label_w, gap + si * (strip_h + gap)), Vector2(size.x - label_w - 8.0, strip_h))
		_draw_strip(a, rect, font)
	draw_string(font, Vector2(label_w, size.y - 1), "last %.0f s" % (CAPACITY / SAMPLE_HZ), HORIZONTAL_ALIGNMENT_LEFT, -1, 10, Color(1, 1, 1, 0.4))


func _draw_strip(a: int, rect: Rect2, font: Font) -> void:
	# Relative mode subtracts each leg's average over the window (its resting spot around the body).
	var offsets := PackedFloat32Array()
	offsets.resize(leg_names.size())
	if relative:
		for i in leg_names.size():
			var values: PackedFloat32Array = _actual[i][a]
			var sum := 0.0
			for v in values:
				sum += v
			offsets[i] = sum / maxi(values.size(), 1)
	# Auto-scale to the selected legs (and targets if shown), at least 2 cm tall.
	var lo := INF
	var hi := -INF
	for i in leg_names.size():
		if not leg_visible[i]:
			continue
		for v in _actual[i][a]:
			lo = minf(lo, v - offsets[i])
			hi = maxf(hi, v - offsets[i])
		if show_target:
			for v in _target[i][a]:
				lo = minf(lo, v - offsets[i])
				hi = maxf(hi, v - offsets[i])
	if lo == INF:
		lo = -0.01
		hi = 0.01
	var span := maxf(hi - lo, 0.02)
	var mid := (lo + hi) * 0.5
	lo = mid - span * 0.55
	hi = mid + span * 0.55

	draw_rect(rect, Color(1, 1, 1, 0.03))
	draw_rect(rect, Color(1, 1, 1, 0.12), false, 1.0)
	for k in 5:
		var value := lerpf(lo, hi, k / 4.0)
		var y := rect.end.y - rect.size.y * k / 4.0
		draw_line(Vector2(rect.position.x, y), Vector2(rect.end.x, y), Color(1, 1, 1, 0.06))
		# Keep the scale labels inside the strip so neighbouring strips don't overlap.
		var label_y := clampf(y + 4.0, rect.position.y + 10.0, rect.end.y - 2.0)
		draw_string(font, Vector2(4, label_y), "%+.1f cm" % (value * 100.0) if relative else "%.1f cm" % (value * 100.0), HORIZONTAL_ALIGNMENT_LEFT, -1, 10, Color(1, 1, 1, 0.5))
	var title: String = AXIS_TITLES[a] + ("   relative to each leg's average" if relative else "")
	draw_string(font, rect.position + Vector2(6, 14), title, HORIZONTAL_ALIGNMENT_LEFT, -1, 12, Color(1, 1, 1, 0.75))

	for i in leg_names.size():
		if not leg_visible[i]:
			continue
		var color := leg_color(leg_names[i])
		if show_target:
			_plot(_target[i][a], offsets[i], rect, lo, hi, Color(color, 0.35), 1.0)
		_plot(_actual[i][a], offsets[i], rect, lo, hi, color, 1.6)


func _plot(values: PackedFloat32Array, offset_value: float, rect: Rect2, lo: float, hi: float, color: Color, width: float) -> void:
	if values.size() < 2:
		return
	var points := PackedVector2Array()
	var offset := CAPACITY - values.size()
	for i in values.size():
		var t := clampf((values[i] - offset_value - lo) / (hi - lo), 0.0, 1.0)
		points.append(Vector2(rect.position.x + rect.size.x * (offset + i) / float(CAPACITY - 1), rect.end.y - rect.size.y * t))
	draw_polyline(points, color, width, true)
