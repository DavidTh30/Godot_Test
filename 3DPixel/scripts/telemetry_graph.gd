class_name TelemetryGraph
extends Control
## Scrolling line chart for a few telemetry series, each with its own fixed range.

const CAPACITY := 300

var _series: Array[Dictionary] = []


func add_series(label: String, color: Color, lo: float, hi: float) -> void:
	_series.append({"label": label, "color": color, "lo": lo, "hi": hi, "values": PackedFloat32Array()})


func push(index: int, value: float) -> void:
	var values: PackedFloat32Array = _series[index]["values"]
	values.append(value)
	if values.size() > CAPACITY:
		values = values.slice(values.size() - CAPACITY)
	_series[index]["values"] = values


func clear() -> void:
	for s in _series:
		s["values"] = PackedFloat32Array()
	queue_redraw()


func _draw() -> void:
	var rect := Rect2(Vector2.ZERO, size)
	draw_rect(rect, Color(0, 0, 0, 0.35))
	for i in range(1, 4):
		var y := size.y * i / 4.0
		draw_line(Vector2(0, y), Vector2(size.x, y), Color(1, 1, 1, 0.06))
	var font := get_theme_default_font()
	for si in _series.size():
		var s := _series[si]
		var values: PackedFloat32Array = s["values"]
		var lo: float = s["lo"]
		var hi: float = s["hi"]
		var color: Color = s["color"]
		if lo < 0.0 and hi > 0.0:
			var zero_y := size.y * (1.0 - (0.0 - lo) / (hi - lo))
			draw_line(Vector2(0, zero_y), Vector2(size.x, zero_y), Color(color, 0.25))
		if values.size() >= 2:
			var points := PackedVector2Array()
			for i in values.size():
				var t := clampf((values[i] - lo) / (hi - lo), 0.0, 1.0)
				points.append(Vector2(size.x * i / float(CAPACITY - 1), size.y * (1.0 - t)))
			draw_polyline(points, color, 1.5, true)
		var latest := values[values.size() - 1] if values.size() > 0 else 0.0
		draw_string(font, Vector2(6, 14 + si * 15), "%s  %.3f" % [s["label"], latest], HORIZONTAL_ALIGNMENT_LEFT, -1, 12, color)
