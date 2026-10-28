extends Node3D
## Spider robot physics lab: builds the world, the robot and a control/telemetry UI.

const TERRAINS := ["Flat", "Slope 10 deg", "Rough", "Steps"]
const TIME_SCALES := [1.0, 0.5, 0.25]
const SPAWN := Vector3(0, 0, 0.8)
## Slider key -> SpiderRobot property, used to restore defaults.
const SLIDER_PROPS := {
	"speed": "speed", "stride": "stride_length", "turn": "turn", "strafe": "strafe",
	"freq": "step_frequency", "lift": "step_height", "height": "stand_height", "reach": "stance_reach",
	"kp": "servo_kp", "torque": "servo_max_torque", "motor_gain": "motor_gain", "bob": "body_bob",
}
const DEFAULT_FRICTION := 1.0
enum MouseMode { CAMERA, GO_TO, PLACE, MOVE, DELETE }
const MOUSE_MODE_NAMES := ["Camera", "Walk to click", "Place", "Move", "Delete"]
const NAV_ARRIVE := 0.12  ## m, the robot stops this close to the clicked point
const NAV_WAYPOINT_REACH := 0.15  ## m, an intermediate waypoint counts as reached this close
const NAV_CLEARANCE := 0.07  ## m kept between the outermost foot and an obstacle
const NAV_CLIMB_MARGIN := 0.02  ## m, rises up to step_height + this are walked over, not around
const NAV_STUCK_TIME := 4.0  ## s without getting closer to the goal before re-planning
const NAV_REPLAN_DELAY := 0.1  ## s after an obstacle edit, so deleted bodies are gone first
const GRID_SHADER := """
shader_type spatial;
uniform vec3 base_color : source_color = vec3(0.22, 0.23, 0.25);
uniform vec3 line_color : source_color = vec3(0.34, 0.35, 0.38);
varying vec3 world_pos;
void vertex() {
	world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}
void fragment() {
	vec2 fine = world_pos.xz / 0.25;
	vec2 g1 = abs(fract(fine - 0.5) - 0.5) / fwidth(fine);
	vec2 g2 = abs(fract(world_pos.xz - 0.5) - 0.5) / fwidth(world_pos.xz);
	float minor = 1.0 - min(min(g1.x, g1.y), 1.0);
	float major = 1.0 - min(min(g2.x, g2.y), 1.0);
	ALBEDO = mix(base_color, line_color, max(minor * 0.45, major));
	ROUGHNESS = 0.92;
}
"""

var robot: SpiderRobot

var _terrain: Node3D
var _terrain_index := 0
var _ground_material := PhysicsMaterial.new()
var _grid_shader := Shader.new()
var _time_scale_index := 0
var _sim_time := 0.0
var _sample_accum := 0.0
var _speed_avg := 0.0

var _camera: Camera3D
var _cam_target := SPAWN
var _cam_yaw := 30.0
var _cam_pitch := -48.0
var _cam_dist := 2.4

var _sliders := {}
var _legs_option: OptionButton
var _gait_option: OptionButton
var _terrain_option: OptionButton
var _walk_button: Button
var _balance_check: CheckBox
var _scan_check: CheckBox
var _targets_check: CheckBox
var _time_option: OptionButton
var _servo_option: OptionButton
var _gyro_check: CheckBox
var _command_option: OptionButton
var _mouse_mode: int = MouseMode.CAMERA
var _mode_buttons: Array[Button] = []
var _mode_hint: Label
var _obstacle_option: OptionButton
var _obstacles: Node3D
var _place_index := 0
var _place_yaw := 0.0
var _ghost: Node3D
var _dragging: CollisionObject3D
var _drag_offset := Vector3.ZERO
var _nav_active := false
var _nav_target := Vector3.ZERO
var _nav_marker: Node3D
var _nav_planner := NavPlanner.new()
var _nav_path := PackedVector3Array()  ## remaining waypoints, the last one is the target
var _nav_grid_valid := false  ## the planner grid matches the current path (for line heights)
var _nav_no_path := false
var _nav_remaining := 0.0
var _nav_best := INF
var _nav_best_time := 0.0
var _nav_replan_in := -1.0
var _nav_stuck_spots: Array[Vector3] = []  ## (x, radius, z) of spots the robot got stuck at
var _nav_line: MeshInstance3D
var _nav_line_mesh := ImmediateMesh.new()
var _avoid_check: CheckBox
var _leg_graph: LegGraph
var _leg_graph_panel: PanelContainer
var _leg_graph_row: HBoxContainer
var _leg_graph_body_frame := true
var _leg_graph_tick := 0
var _telemetry: Label
var _state_label: Label
var _graph: TelemetryGraph
var _leg_row: HBoxContainer
var _leg_styles: Array[StyleBoxFlat] = []
var _leg_labels: Array[Label] = []


func _ready() -> void:
	_grid_shader.code = GRID_SHADER
	_ground_material.friction = 1.0
	_setup_world()
	_build_terrain(0)
	_obstacles = Node3D.new()
	_obstacles.name = "Obstacles"
	add_child(_obstacles)
	_nav_marker = _make_nav_marker()
	add_child(_nav_marker)
	_nav_line = _make_nav_line()
	add_child(_nav_line)
	robot = SpiderRobot.new()
	robot.name = "SpiderRobot"
	add_child(robot)
	_build_ui()
	_reset_robot()


func _process(delta: float) -> void:
	if robot.body == null:
		return
	_cam_target = _cam_target.lerp(robot.body.global_position, clampf(delta * 4.0, 0.0, 1.0))
	var basis := Basis.from_euler(Vector3(deg_to_rad(_cam_pitch), deg_to_rad(_cam_yaw), 0))
	_camera.transform = Transform3D(basis, _cam_target + basis.z * _cam_dist)
	_update_telemetry(delta)
	_update_ghost()
	_draw_nav_path()
	if _leg_graph_panel and _leg_graph_panel.visible:
		_leg_graph.queue_redraw()


func _physics_process(delta: float) -> void:
	_sim_time += delta
	_update_navigation(delta)
	_sample_leg_graph()


## Feeds the leg graph at 60 Hz with each foot's actual and planned position,
## in the body frame (relative to the chassis) or in world space.
func _sample_leg_graph() -> void:
	_leg_graph_tick += 1
	if _leg_graph == null or robot.body == null or _leg_graph_tick % 4 != 0:
		return
	var xf := robot.body.global_transform
	var inv := xf.affine_inverse()
	var actual: Array[Vector3] = []
	var target: Array[Vector3] = []
	for leg in robot.legs:
		if _leg_graph_body_frame:
			actual.append(inv * leg.foot_pos)
			target.append(leg.target)
		else:
			actual.append(leg.foot_pos)
			target.append(xf * leg.target)
	_leg_graph.push(actual, target)


func _unhandled_input(event: InputEvent) -> void:
	if _handle_mouse(event):
		get_viewport().set_input_as_handled()
		return
	if event is InputEventMouseMotion and event.button_mask & MOUSE_BUTTON_MASK_RIGHT:
		_cam_yaw -= event.relative.x * 0.3
		_cam_pitch = clampf(_cam_pitch - event.relative.y * 0.3, -89.0, -5.0)
	elif event is InputEventMouseButton and event.pressed:
		# Wheel over the tool panel belongs to the panel (scrolling), never to the camera.
		if get_viewport().gui_get_hovered_control() != null:
			return
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			_cam_dist = maxf(0.6, _cam_dist * 0.9)
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_cam_dist = minf(12.0, _cam_dist * 1.1)
	elif event is InputEventKey and event.pressed and not event.echo:
		match event.keycode:
			KEY_SPACE:
				if _nav_active:
					_cancel_navigation()
				else:
					_set_walking(not robot.walking)
			KEY_ESCAPE:
				_cancel_navigation()
				_set_mouse_mode(MouseMode.CAMERA)
			KEY_R:
				if event.shift_pressed:
					_reset_robot()
				else:
					_reset_defaults()
			KEY_P:
				robot.push(2.0)
			KEY_W:
				_nudge_forward(1.0)
			KEY_S:
				_nudge_forward(-1.0)
			KEY_A:
				_nudge("turn", 0.1)
			KEY_D:
				_nudge("turn", -0.1)
			KEY_Q:
				_nudge("strafe", -0.02)
			KEY_E:
				_nudge("strafe", 0.02)
			KEY_X:
				for key in ["speed", "stride", "turn", "strafe"]:
					_sliders[key].value = 0.0
			KEY_G:
				_leg_graph_panel.visible = not _leg_graph_panel.visible
			KEY_B:
				_balance_check.button_pressed = not _balance_check.button_pressed
			KEY_T:
				_set_time_scale((_time_scale_index + 1) % TIME_SCALES.size())
			KEY_1, KEY_2, KEY_3:
				var idx: int = event.keycode - KEY_1
				if idx < _gait_option.item_count:
					_gait_option.select(idx)
					robot.gait_index = idx
			_:
				return
		get_viewport().set_input_as_handled()


# --- World ------------------------------------------------------------------------

func _setup_world() -> void:
	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR
	env.background_color = Color("2a2c30")
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color("c9d2e0")
	env.ambient_light_energy = 0.45
	env.tonemap_mode = Environment.TONE_MAPPER_FILMIC
	env.ssao_enabled = true
	env.fog_enabled = true
	env.fog_light_color = Color("2a2c30")
	env.fog_density = 0.03
	var world_env := WorldEnvironment.new()
	world_env.environment = env
	add_child(world_env)

	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-60, 25, 0)
	sun.light_energy = 1.1
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 12.0
	add_child(sun)

	_camera = Camera3D.new()
	_camera.fov = 50.0
	_camera.near = 0.02
	add_child(_camera)
	_camera.current = true


func _build_terrain(kind: int) -> void:
	_terrain_index = kind
	if _terrain:
		remove_child(_terrain)
		_terrain.queue_free()
	_terrain = Node3D.new()
	_terrain.name = "Terrain"
	add_child(_terrain)
	_static_box(Vector3(0, -0.5, 0), Vector3(40, 1, 40), Basis(), Color("37393d"))

	match kind:
		1:
			# Ramp rising toward -Z, then a plateau.
			var angle := deg_to_rad(10.0)
			var length := 3.0
			var tilt := Basis(Vector3.RIGHT, angle)
			var top_mid := Vector3(0, length * 0.5 * sin(angle), -1.0 - length * 0.5 * cos(angle))
			_static_box(top_mid - tilt.y * 0.1, Vector3(3, 0.2, length), tilt, Color("45403a"))
			var plateau_y := length * sin(angle)
			_static_box(Vector3(0, plateau_y * 0.5, -1.0 - length * cos(angle) - 1.5), Vector3(3, plateau_y, 3), Basis(), Color("45403a"))
		2:
			var rng := RandomNumberGenerator.new()
			rng.seed = 3
			for i in 220:
				var h := rng.randf_range(0.012, 0.05)
				var size := Vector3(rng.randf_range(0.08, 0.24), h, rng.randf_range(0.08, 0.24))
				var pos := Vector3(rng.randf_range(-1.6, 1.6), h * 0.5, rng.randf_range(-7.0, 0.3))
				_static_box(pos, size, Basis(Vector3.UP, rng.randf() * TAU), Color("4a4540"))
		3:
			for k in 10:
				var h := 0.035 * (k + 1)
				_static_box(Vector3(0, h * 0.5, -0.8 - 0.35 * (k + 0.5)), Vector3(3, h, 0.35), Basis(), Color("40444a"))
			_static_box(Vector3(0, 0.175, -0.8 - 3.5 - 1.5), Vector3(3, 0.35, 3), Basis(), Color("40444a"))


func _static_box(pos: Vector3, size: Vector3, basis: Basis, color: Color) -> void:
	var sb := StaticBody3D.new()
	sb.collision_layer = SpiderRobot.GROUND_LAYER
	sb.collision_mask = 0
	sb.physics_material_override = _ground_material
	sb.transform = Transform3D(basis, pos)
	var shape := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = size
	shape.shape = box
	sb.add_child(shape)
	var mi := MeshInstance3D.new()
	var mesh := BoxMesh.new()
	mesh.size = size
	mi.mesh = mesh
	var mat := ShaderMaterial.new()
	mat.shader = _grid_shader
	mat.set_shader_parameter("base_color", color)
	mat.set_shader_parameter("line_color", color.lightened(0.18))
	mi.material_override = mat
	sb.add_child(mi)
	_terrain.add_child(sb)


func _set_ground_friction(value: float) -> void:
	_ground_material.friction = value
	for child in _terrain.get_children():
		var sb := child as StaticBody3D
		if sb:
			sb.physics_material_override = null
			sb.physics_material_override = _ground_material


# --- Robot control ----------------------------------------------------------------

func _reset_robot() -> void:
	_cancel_navigation()
	robot.build(Transform3D(Basis(), SPAWN))
	_cam_target = SPAWN
	_sim_time = 0.0
	_speed_avg = 0.0
	if _graph:
		_graph.clear()
	_rebuild_leg_row()
	_set_walking(robot.walking)


## Restores every robot, servo and world setting (except the terrain) to its start-up value,
## then respawns the robot.
func _reset_defaults() -> void:
	var defaults := SpiderRobot.new()
	_set_walking(false)
	robot.leg_count = defaults.leg_count
	robot.gait_index = defaults.gait_index
	_legs_option.select(0 if defaults.leg_count == 4 else 1)
	_refresh_gaits()
	robot.balance = defaults.balance
	robot.terrain_scan = defaults.terrain_scan
	robot.show_targets = defaults.show_targets
	robot.servo_mode = defaults.servo_mode
	_set_command_mode(0 if defaults.command_mode == 1 else 1)
	_command_option.select(0 if defaults.command_mode == 1 else 1)
	robot.gyro_feedback = defaults.gyro_feedback
	_gyro_check.set_pressed_no_signal(defaults.gyro_feedback)
	_servo_option.select(defaults.servo_mode)
	_balance_check.set_pressed_no_signal(defaults.balance)
	_scan_check.set_pressed_no_signal(defaults.terrain_scan)
	_targets_check.set_pressed_no_signal(defaults.show_targets)
	for key in SLIDER_PROPS:
		var value: float = defaults.get(SLIDER_PROPS[key])
		robot.set(SLIDER_PROPS[key], value)
		_sliders[key].value = value
	_sliders["friction"].value = DEFAULT_FRICTION
	_set_ground_friction(DEFAULT_FRICTION)
	_time_option.select(0)
	_set_time_scale(0)
	defaults.free()
	_cam_yaw = 30.0
	_cam_pitch = -48.0
	_cam_dist = 2.4
	_reset_robot()


## Option index 0 = stride length, 1 = speed. Dims the slider that is not in use.
func _set_command_mode(index: int) -> void:
	robot.command_mode = 1 if index == 0 else 0
	_sliders["stride"].modulate.a = 1.0 if robot.command_mode == 1 else 0.35
	_sliders["speed"].modulate.a = 0.35 if robot.command_mode == 1 else 1.0


func _set_walking(on: bool) -> void:
	robot.walking = on
	if _walk_button:
		_walk_button.text = "Stop  (Space)" if on else "Walk  (Space)"


func _set_leg_count(index: int) -> void:
	robot.leg_count = 4 if index == 0 else 6
	robot.gait_index = 0
	_refresh_gaits()
	_reset_robot()


func _refresh_gaits() -> void:
	_gait_option.clear()
	for gait_name in SpiderRobot.gait_names(robot.leg_count):
		_gait_option.add_item(gait_name)
	_gait_option.select(robot.gait_index)


func _set_time_scale(index: int) -> void:
	_time_scale_index = index
	if _time_option:
		_time_option.select(index)
	Engine.time_scale = TIME_SCALES[index]


## W/S: change the stride length or the speed, whichever is the active command.
func _nudge_forward(direction: float) -> void:
	if robot.command_mode == 1:
		_nudge("stride", 0.01 * direction)
	else:
		_nudge("speed", 0.02 * direction)


func _nudge(key: String, amount: float) -> void:
	var slider: HSlider = _sliders[key]
	slider.value += amount


# --- UI ---------------------------------------------------------------------------

func _build_ui() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)

	var panel := PanelContainer.new()
	panel.custom_minimum_size = Vector2(300, 0)
	panel.anchor_bottom = 1.0
	panel.offset_bottom = 0
	layer.add_child(panel)
	var scroll := ScrollContainer.new()
	scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	panel.add_child(scroll)
	var margin := MarginContainer.new()
	margin.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	for side in ["left", "right", "top", "bottom"]:
		margin.add_theme_constant_override("margin_" + side, 12)
	scroll.add_child(margin)
	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation", 6)
	margin.add_child(box)

	var title := Label.new()
	title.text = "Spider Robot Physics Lab"
	title.add_theme_font_size_override("font_size", 18)
	box.add_child(title)

	_header(box, "ROBOT")
	_legs_option = _option(box, "Legs", ["4 (quadruped)", "6 (hexapod)"], 1, _set_leg_count)
	_gait_option = _option(box, "Gait", [], 0, func(i: int) -> void: robot.gait_index = i)
	_refresh_gaits()
	var row := HBoxContainer.new()
	box.add_child(row)
	_walk_button = _button(row, "Walk  (Space)", func() -> void: _set_walking(not robot.walking))
	_button(row, "Push (P)", func() -> void: robot.push(2.0))
	var reset_row := HBoxContainer.new()
	box.add_child(reset_row)
	var reset_all := _button(reset_row, "Reset all (R)", _reset_defaults)
	reset_all.tooltip_text = "Restore all default settings and respawn the robot"
	var respawn := _button(reset_row, "Respawn (Shift+R)", _reset_robot)
	respawn.tooltip_text = "Put the robot back at the start, keeping the current settings"
	_balance_check = CheckBox.new()
	_balance_check.text = "IMU balance  (B)"
	_balance_check.button_pressed = robot.balance
	_balance_check.focus_mode = Control.FOCUS_NONE
	_balance_check.toggled.connect(func(on: bool) -> void: robot.balance = on)
	box.add_child(_balance_check)
	_scan_check = CheckBox.new()
	var scan := _scan_check
	scan.text = "Terrain scan (foothold raycast)"
	scan.button_pressed = robot.terrain_scan
	scan.focus_mode = Control.FOCUS_NONE
	scan.toggled.connect(func(on: bool) -> void: robot.terrain_scan = on)
	box.add_child(scan)
	_gyro_check = CheckBox.new()
	_gyro_check.text = "Gyro turn-rate feedback"
	_gyro_check.button_pressed = robot.gyro_feedback
	_gyro_check.focus_mode = Control.FOCUS_NONE
	_gyro_check.toggled.connect(func(on: bool) -> void: robot.gyro_feedback = on)
	box.add_child(_gyro_check)
	_targets_check = CheckBox.new()
	var targets := _targets_check
	targets.text = "Show foot targets"
	targets.button_pressed = robot.show_targets
	targets.focus_mode = Control.FOCUS_NONE
	targets.toggled.connect(func(on: bool) -> void: robot.show_targets = on)
	box.add_child(targets)

	_header(box, "COMMAND  (W/S  A/D  Q/E,  X = zero)")
	_command_option = _option(box, "Command by", ["Stride length", "Speed"], 0 if robot.command_mode == 1 else 1, _set_command_mode)
	_slider(box, "stride", "Stride length", -0.2, 0.2, 0.005, robot.stride_length, "%.3f m", func(v: float) -> void: robot.stride_length = v)
	_slider(box, "speed", "Forward speed", -0.3, 0.3, 0.01, robot.speed, "%.2f m/s", func(v: float) -> void: robot.speed = v)
	_set_command_mode(_command_option.selected)
	_slider(box, "turn", "Turn rate", -1.0, 1.0, 0.05, robot.turn, "%.2f rad/s", func(v: float) -> void: robot.turn = v)
	_slider(box, "strafe", "Strafe", -0.2, 0.2, 0.01, robot.strafe, "%.2f m/s", func(v: float) -> void: robot.strafe = v)

	_header(box, "GAIT")
	_slider(box, "freq", "Step frequency", 0.3, 4.0, 0.05, robot.step_frequency, "%.2f Hz", func(v: float) -> void: robot.step_frequency = v)
	_slider(box, "lift", "Step height", 0.0, 0.1, 0.005, robot.step_height, "%.3f m", func(v: float) -> void: robot.step_height = v)
	_slider(box, "height", "Body height", 0.06, 0.26, 0.005, robot.stand_height, "%.3f m", func(v: float) -> void: robot.stand_height = v)
	_slider(box, "reach", "Stance reach", 0.2, 0.4, 0.005, robot.stance_reach, "%.3f m", func(v: float) -> void: robot.stance_reach = v)

	_slider(box, "bob", "Body bob", 0.0, 0.02, 0.001, robot.body_bob, "%.3f m", func(v: float) -> void: robot.body_bob = v)

	_header(box, "SERVOS")
	_servo_option = _option(box, "Servo model", ["PD torque (soft)", "Motor (stiff)"], robot.servo_mode, func(i: int) -> void: robot.servo_mode = i)
	_slider(box, "motor_gain", "Motor gain", 5.0, 30.0, 1.0, robot.motor_gain, "%.0f /s", func(v: float) -> void: robot.motor_gain = v)
	_slider(box, "kp", "PD stiffness (kp)", 10.0, 150.0, 1.0, robot.servo_kp, "%.0f Nm/rad", func(v: float) -> void: robot.servo_kp = v)
	_slider(box, "torque", "Max torque", 0.5, 15.0, 0.1, robot.servo_max_torque, "%.1f Nm", func(v: float) -> void: robot.servo_max_torque = v)

	_header(box, "WORLD")
	_terrain_option = _option(box, "Terrain", TERRAINS, 0, func(i: int) -> void:
		_build_terrain(i)
		_reset_robot())
	_slider(box, "friction", "Ground friction", 0.05, 1.5, 0.05, 1.0, "%.2f", _set_ground_friction)
	_time_option = _option(box, "Time scale  (T)", ["1x", "0.5x", "0.25x"], 0, _set_time_scale)

	var help := Label.new()
	help.text = "Right drag: orbit   Wheel: zoom\n1/2/3: gait   Space: walk/stop\nG: foot position graph"
	help.add_theme_font_size_override("font_size", 12)
	help.modulate = Color(1, 1, 1, 0.55)
	box.add_child(help)

	# Telemetry panel (top right).
	var tele := PanelContainer.new()
	tele.anchor_left = 1.0
	tele.anchor_right = 1.0
	tele.offset_left = -360
	tele.offset_top = 10
	tele.offset_right = -10
	layer.add_child(tele)
	var tele_margin := MarginContainer.new()
	for side in ["left", "right", "top", "bottom"]:
		tele_margin.add_theme_constant_override("margin_" + side, 10)
	tele.add_child(tele_margin)
	var tele_box := VBoxContainer.new()
	tele_margin.add_child(tele_box)
	_state_label = Label.new()
	_state_label.add_theme_font_size_override("font_size", 18)
	tele_box.add_child(_state_label)
	_telemetry = Label.new()
	_telemetry.add_theme_font_size_override("font_size", 13)
	tele_box.add_child(_telemetry)
	_leg_row = HBoxContainer.new()
	_leg_row.add_theme_constant_override("separation", 4)
	tele_box.add_child(_leg_row)
	var legend := Label.new()
	legend.text = "green = foot touching ground,  ^ = planned swing"
	legend.add_theme_font_size_override("font_size", 11)
	legend.modulate = Color(1, 1, 1, 0.55)
	tele_box.add_child(legend)
	_graph = TelemetryGraph.new()
	_graph.custom_minimum_size = Vector2(330, 130)
	_graph.add_series("speed m/s", Color("4fc3f7"), -0.3, 0.3)
	_graph.add_series("height m", Color("ffb74d"), 0.0, 0.3)
	_graph.add_series("tilt /90deg", Color("ef5350"), 0.0, 1.0)
	tele_box.add_child(_graph)

	_build_leg_graph_panel(layer)
	_build_toolbar(layer)


## Bottom panel: foot X/Y/Z history with leg, axis and frame selectors.
func _build_leg_graph_panel(layer: CanvasLayer) -> void:
	_leg_graph_panel = PanelContainer.new()
	_leg_graph_panel.anchor_left = 0.0
	_leg_graph_panel.anchor_right = 1.0
	_leg_graph_panel.anchor_top = 1.0
	_leg_graph_panel.anchor_bottom = 1.0
	_leg_graph_panel.offset_left = 310
	_leg_graph_panel.offset_right = -10
	_leg_graph_panel.offset_top = -330
	_leg_graph_panel.offset_bottom = -10
	layer.add_child(_leg_graph_panel)
	var margin := MarginContainer.new()
	for side in ["left", "right", "top", "bottom"]:
		margin.add_theme_constant_override("margin_" + side, 8)
	_leg_graph_panel.add_child(margin)
	var box := VBoxContainer.new()
	margin.add_child(box)

	var controls := HBoxContainer.new()
	controls.add_theme_constant_override("separation", 6)
	box.add_child(controls)
	var title := Label.new()
	title.text = "Foot position"
	controls.add_child(title)
	controls.add_child(VSeparator.new())
	_leg_graph_row = HBoxContainer.new()
	controls.add_child(_leg_graph_row)
	_small_button(controls, "All", false, func(_on: bool) -> void: _set_all_legs(true))
	_small_button(controls, "None", false, func(_on: bool) -> void: _set_all_legs(false))
	controls.add_child(VSeparator.new())
	for a in 3:
		var axis_button := _small_button(controls, "XYZ"[a], true, _set_graph_axis.bind(a))
		axis_button.set_pressed_no_signal(true)
	controls.add_child(VSeparator.new())
	var frame := OptionButton.new()
	frame.focus_mode = Control.FOCUS_NONE
	frame.add_item("Body frame")
	frame.add_item("World")
	frame.tooltip_text = "Body frame: foot relative to the chassis. World: absolute position."
	frame.item_selected.connect(_set_graph_frame)
	controls.add_child(frame)
	var target := CheckBox.new()
	target.text = "Target"
	target.tooltip_text = "Overlay the planned foot position as a faint line"
	target.button_pressed = true
	target.focus_mode = Control.FOCUS_NONE
	target.toggled.connect(_set_graph_target)
	controls.add_child(target)
	var relative := CheckBox.new()
	relative.text = "Relative"
	relative.tooltip_text = "Plot each leg around its own average position so all legs share one scale"
	relative.button_pressed = true
	relative.focus_mode = Control.FOCUS_NONE
	relative.toggled.connect(_set_graph_relative)
	controls.add_child(relative)
	_small_button(controls, "Pause", true, func(on: bool) -> void: _leg_graph.paused = on)
	_small_button(controls, "Hide (G)", false, func(_on: bool) -> void: _leg_graph_panel.visible = false)

	_leg_graph = LegGraph.new()
	_leg_graph.size_flags_vertical = Control.SIZE_EXPAND_FILL
	_leg_graph.custom_minimum_size = Vector2(0, 200)
	box.add_child(_leg_graph)


func _small_button(parent: Control, text: String, toggle: bool, callback: Callable) -> Button:
	var b := Button.new()
	b.text = text
	b.toggle_mode = toggle
	b.focus_mode = Control.FOCUS_NONE
	if toggle:
		b.toggled.connect(callback)
	else:
		b.pressed.connect(callback.bind(true))
	parent.add_child(b)
	return b


func _set_graph_axis(on: bool, axis: int) -> void:
	_leg_graph.axis_visible[axis] = on
	_leg_graph.queue_redraw()


func _set_graph_leg(on: bool, index: int) -> void:
	_leg_graph.leg_visible[index] = on
	_leg_graph.queue_redraw()


func _set_graph_frame(index: int) -> void:
	_leg_graph_body_frame = index == 0
	_leg_graph.clear()


func _set_graph_relative(on: bool) -> void:
	_leg_graph.relative = on
	_leg_graph.queue_redraw()


func _set_graph_target(on: bool) -> void:
	_leg_graph.show_target = on
	_leg_graph.queue_redraw()


func _rebuild_leg_graph_legs() -> void:
	if _leg_graph == null:
		return
	var names: Array[String] = []
	for leg in robot.legs:
		names.append(leg.name)
	_leg_graph.set_legs(names)
	for child in _leg_graph_row.get_children():
		_leg_graph_row.remove_child(child)
		child.queue_free()
	for i in names.size():
		var color := LegGraph.leg_color(names[i])
		var b := _small_button(_leg_graph_row, names[i], true, _set_graph_leg.bind(i))
		b.set_pressed_no_signal(_leg_graph.leg_visible[i])
		b.custom_minimum_size.x = 40
		for state in ["font_color", "font_pressed_color", "font_hover_color", "font_hover_pressed_color"]:
			b.add_theme_color_override(state, color)
		var on_style := StyleBoxFlat.new()
		on_style.bg_color = Color(color, 0.25)
		on_style.border_color = color
		on_style.set_border_width_all(2)
		on_style.set_corner_radius_all(4)
		b.add_theme_stylebox_override("pressed", on_style)
		b.add_theme_stylebox_override("hover_pressed", on_style)


func _set_all_legs(on: bool) -> void:
	for i in _leg_graph.leg_visible.size():
		_leg_graph.leg_visible[i] = on
	for child in _leg_graph_row.get_children():
		(child as Button).set_pressed_no_signal(on)
	_leg_graph.queue_redraw()


func _rebuild_leg_row() -> void:
	_rebuild_leg_graph_legs()
	if _leg_row == null:
		return
	for child in _leg_row.get_children():
		_leg_row.remove_child(child)
		child.queue_free()
	_leg_styles.clear()
	_leg_labels.clear()
	for leg in robot.legs:
		var label := Label.new()
		label.text = leg.name
		label.custom_minimum_size = Vector2(46, 26)
		label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
		label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
		var style := StyleBoxFlat.new()
		style.set_corner_radius_all(4)
		label.add_theme_stylebox_override("normal", style)
		_leg_row.add_child(label)
		_leg_styles.append(style)
		_leg_labels.append(label)


func _update_telemetry(delta: float) -> void:
	var fallen := robot.is_fallen()
	if fallen:
		_state_label.text = "FALLEN  -  press R to reset"
		_state_label.modulate = Color("ef5350")
	elif _nav_active:
		_state_label.text = "WALK TO TARGET  -  %.2f m left%s" % [_nav_remaining, "  (no path, going straight)" if _nav_no_path else ""]
		_state_label.modulate = Color("ffb74d")
	elif robot.walking:
		_state_label.text = "WALKING  -  %s gait" % _gait_option.get_item_text(robot.gait_index)
		_state_label.modulate = Color("81c784")
	else:
		_state_label.text = "STANDING"
		_state_label.modulate = Color("e0e0e0")

	# The number shows a ~0.25 s average (like a filtered IMU reading); the graph keeps the raw
	# signal so the per-step speed ripple stays visible.
	var raw_speed := robot.forward_speed()
	_speed_avg = lerpf(_speed_avg, raw_speed, 1.0 - exp(-delta / 0.25))
	var pos := robot.body.global_position
	var travelled := Vector2(pos.x - SPAWN.x, pos.z - SPAWN.z).length()
	var contacts := 0
	for leg in robot.legs:
		contacts += 1 if leg.contact else 0
	_telemetry.text = "\n".join([
		"Speed      %6.3f m/s   (planned %.2f, 0.25 s avg)" % [0.0 if absf(_speed_avg) < 0.0005 else _speed_avg, robot.planned_speed()],
		"Stride     %6.1f cm      (%s)" % [robot.planned_speed() * robot.GAITS[robot.leg_count][robot.gait_index][1] / robot.cadence() * 100.0, "set" if robot.command_mode == 1 else "from speed"],
		"Distance   %6.2f m      Heading %5.0f deg" % [travelled, robot.heading_deg()],
		"Height     %6.3f m      Tilt %5.1f deg" % [pos.y, robot.tilt_deg()],
		"Joint err  %6.2f deg    Servo load %s" % [robot.joint_error_rms_deg(), "n/a (motor)" if robot.servo_load() < 0.0 else "%3.0f%%" % (robot.servo_load() * 100.0)],
		"Feet down  %d / %d        Cadence %.2f Hz" % [contacts, robot.legs.size(), robot.cadence()],
		"Sim time   %.1f s  (%sx)" % [_sim_time, str(TIME_SCALES[_time_scale_index])],
	])

	for i in mini(robot.legs.size(), _leg_styles.size()):
		var leg := robot.legs[i]
		_leg_styles[i].bg_color = Color("2e7d32") if leg.contact else Color("424242")
		_leg_labels[i].text = leg.name + ("" if leg.stance else " ^")

	_sample_accum += delta
	if _sample_accum >= 1.0 / 30.0:
		_sample_accum = 0.0
		_graph.push(0, raw_speed)
		_graph.push(1, pos.y)
		_graph.push(2, robot.tilt_deg() / 90.0)
		_graph.queue_redraw()


func _header(parent: Control, text: String) -> void:
	var label := Label.new()
	label.text = text
	label.add_theme_font_size_override("font_size", 12)
	label.modulate = Color(1, 1, 1, 0.55)
	parent.add_child(label)


func _button(parent: Control, text: String, callback: Callable) -> Button:
	var b := Button.new()
	b.text = text
	b.focus_mode = Control.FOCUS_NONE
	b.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	b.pressed.connect(callback)
	parent.add_child(b)
	return b


func _option(parent: Control, text: String, items: Array, selected: int, callback: Callable) -> OptionButton:
	var row := HBoxContainer.new()
	parent.add_child(row)
	var label := Label.new()
	label.text = text
	label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	row.add_child(label)
	var option := OptionButton.new()
	option.focus_mode = Control.FOCUS_NONE
	option.custom_minimum_size.x = 150
	for item in items:
		option.add_item(str(item))
	if selected < items.size():
		option.select(selected)
	option.item_selected.connect(callback)
	row.add_child(option)
	return option


func _slider(parent: Control, key: String, text: String, lo: float, hi: float, step: float, value: float, fmt: String, callback: Callable) -> void:
	var head := HBoxContainer.new()
	parent.add_child(head)
	var label := Label.new()
	label.text = text
	label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	head.add_child(label)
	var value_label := Label.new()
	value_label.text = fmt % value
	value_label.modulate = Color("90caf9")
	head.add_child(value_label)
	var slider := HSlider.new()
	slider.min_value = lo
	slider.max_value = hi
	slider.step = step
	slider.value = value
	slider.focus_mode = Control.FOCUS_NONE
	slider.scrollable = false  # the wheel scrolls the panel instead of nudging values
	slider.value_changed.connect(func(v: float) -> void:
		value_label.text = fmt % v
		callback.call(v))
	parent.add_child(slider)
	_sliders[key] = slider


# --- Mouse modes: click-to-walk and obstacle editing --------------------------------

func _build_toolbar(layer: CanvasLayer) -> void:
	var bar := PanelContainer.new()
	bar.position = Vector2(310, 10)
	layer.add_child(bar)
	var margin := MarginContainer.new()
	for side in ["left", "right", "top", "bottom"]:
		margin.add_theme_constant_override("margin_" + side, 6)
	bar.add_child(margin)
	var box := VBoxContainer.new()
	margin.add_child(box)
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 4)
	box.add_child(row)
	var label := Label.new()
	label.text = "Mouse:"
	row.add_child(label)
	var group := ButtonGroup.new()
	for i in MOUSE_MODE_NAMES.size():
		var b := Button.new()
		b.text = MOUSE_MODE_NAMES[i]
		b.toggle_mode = true
		b.button_group = group
		b.focus_mode = Control.FOCUS_NONE
		b.button_pressed = i == _mouse_mode
		b.pressed.connect(_set_mouse_mode.bind(i))
		row.add_child(b)
		_mode_buttons.append(b)
	row.add_child(VSeparator.new())
	_obstacle_option = OptionButton.new()
	_obstacle_option.focus_mode = Control.FOCUS_NONE
	for obstacle_name in Obstacles.names():
		_obstacle_option.add_item(obstacle_name)
	_obstacle_option.tooltip_text = "Obstacle to place"
	_obstacle_option.item_selected.connect(func(i: int) -> void:
		_place_index = i
		_set_mouse_mode(MouseMode.PLACE))
	row.add_child(_obstacle_option)
	var clear := Button.new()
	clear.text = "Clear obstacles"
	clear.focus_mode = Control.FOCUS_NONE
	clear.pressed.connect(_clear_obstacles)
	row.add_child(clear)
	row.add_child(VSeparator.new())
	_avoid_check = CheckBox.new()
	_avoid_check.text = "Avoid obstacles"
	_avoid_check.tooltip_text = "Walk to click: plan a path around obstacles too tall to step over"
	_avoid_check.button_pressed = true
	_avoid_check.focus_mode = Control.FOCUS_NONE
	_avoid_check.toggled.connect(func(_on: bool) -> void:
		if _nav_active:
			_nav_stuck_spots.clear()
			_plan_path())
	row.add_child(_avoid_check)
	_mode_hint = Label.new()
	_mode_hint.add_theme_font_size_override("font_size", 12)
	_mode_hint.modulate = Color(1, 1, 1, 0.6)
	box.add_child(_mode_hint)
	_set_mouse_mode(_mouse_mode)


func _set_mouse_mode(mode: int) -> void:
	_mouse_mode = mode
	_release_drag()
	if mode < _mode_buttons.size():
		_mode_buttons[mode].button_pressed = true
	var hints := [
		"Right drag: orbit   Wheel: zoom",
		"Left click the ground: the robot walks there (Space or Esc cancels)",
		"Left click: place  -  Wheel: rotate  -  pick the obstacle in the list",
		"Left drag an obstacle to move it  -  Wheel while dragging: rotate",
		"Left click an obstacle to delete it",
	]
	if _mode_hint:
		_mode_hint.text = hints[mode]
	var cursors := [Input.CURSOR_ARROW, Input.CURSOR_CROSS, Input.CURSOR_CROSS, Input.CURSOR_MOVE, Input.CURSOR_FORBIDDEN]
	Input.set_default_cursor_shape(cursors[mode])
	_rebuild_ghost()


## Handles left clicks / wheel / drags in the 3D view for the current mouse mode.
## Returns true when the event was used.
func _handle_mouse(event: InputEvent) -> bool:
	if event is InputEventMouseButton:
		var mb := event as InputEventMouseButton
		if mb.pressed and get_viewport().gui_get_hovered_control() != null:
			return false
		if mb.button_index == MOUSE_BUTTON_LEFT:
			if not mb.pressed:
				if _dragging:
					_release_drag()
					return true
				return false
			var hit := _mouse_ray([], mb.position)
			match _mouse_mode:
				MouseMode.GO_TO:
					if not hit.is_empty():
						_start_navigation(hit["position"])
					return true
				MouseMode.PLACE:
					if not hit.is_empty():
						_place_obstacle(hit["position"])
					return true
				MouseMode.MOVE:
					var obstacle := _obstacle_from_hit(hit)
					if obstacle:
						_dragging = obstacle
						_drag_offset = obstacle.global_position - (hit["position"] as Vector3)
						_drag_offset.y = 0.0
						if obstacle is RigidBody3D:
							(obstacle as RigidBody3D).freeze = true
					return true
				MouseMode.DELETE:
					var obstacle := _obstacle_from_hit(hit)
					if obstacle:
						obstacle.queue_free()
						_request_replan()
					return true
		elif mb.pressed and (mb.button_index == MOUSE_BUTTON_WHEEL_UP or mb.button_index == MOUSE_BUTTON_WHEEL_DOWN):
			var step := deg_to_rad(15.0) * (1.0 if mb.button_index == MOUSE_BUTTON_WHEEL_UP else -1.0)
			if _dragging:
				_dragging.rotate_y(step)
				_request_replan()
				return true
			if _mouse_mode == MouseMode.PLACE:
				_place_yaw += step
				return true
	elif event is InputEventMouseMotion and _dragging:
		var hit := _mouse_ray([_dragging.get_rid()], (event as InputEventMouseMotion).position)
		if not hit.is_empty():
			_dragging.global_position = (hit["position"] as Vector3) + _drag_offset
			_request_replan()
		return true
	return false


func _mouse_ray(exclude: Array[RID], screen_pos: Vector2) -> Dictionary:
	var mouse := screen_pos
	var from := _camera.project_ray_origin(mouse)
	var to := from + _camera.project_ray_normal(mouse) * 100.0
	var query := PhysicsRayQueryParameters3D.create(from, to, SpiderRobot.GROUND_LAYER, exclude)
	return get_world_3d().direct_space_state.intersect_ray(query)


func _obstacle_from_hit(hit: Dictionary) -> CollisionObject3D:
	if hit.is_empty():
		return null
	var collider: Object = hit["collider"]
	if collider is CollisionObject3D and (collider as Node).is_in_group(Obstacles.GROUP):
		return collider as CollisionObject3D
	return null


func _place_obstacle(at: Vector3) -> void:
	var obstacle := Obstacles.build(_place_index, _ground_material)
	_obstacles.add_child(obstacle)
	obstacle.global_position = at
	obstacle.rotation.y = _place_yaw
	_request_replan()


func _release_drag() -> void:
	if _dragging and is_instance_valid(_dragging):
		if _dragging is RigidBody3D:
			var rb := _dragging as RigidBody3D
			rb.freeze = false
			rb.linear_velocity = Vector3.ZERO
			rb.angular_velocity = Vector3.ZERO
		_request_replan()
	_dragging = null


func _clear_obstacles() -> void:
	_release_drag()
	for child in _obstacles.get_children():
		child.queue_free()
	_request_replan()


## Translucent preview of the obstacle under the mouse in Place mode.
func _rebuild_ghost() -> void:
	if _ghost:
		_ghost.queue_free()
		_ghost = null
	if _mouse_mode != MouseMode.PLACE:
		return
	_ghost = Node3D.new()
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.3, 1.0, 0.6, 0.4)
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	for v in Obstacles.visuals(_place_index):
		(v as MeshInstance3D).material_override = mat
		(v as MeshInstance3D).cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		_ghost.add_child(v)
	_ghost.visible = false
	add_child(_ghost)


func _update_ghost() -> void:
	if _ghost == null:
		return
	var hit := {}
	if get_viewport().gui_get_hovered_control() == null:
		hit = _mouse_ray([], get_viewport().get_mouse_position())
	_ghost.visible = not hit.is_empty()
	if _ghost.visible:
		_ghost.global_position = hit["position"]
		_ghost.rotation.y = _place_yaw


func _make_nav_marker() -> Node3D:
	var marker := Node3D.new()
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color("ff9800")
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	var pole := MeshInstance3D.new()
	var pole_mesh := CylinderMesh.new()
	pole_mesh.top_radius = 0.006
	pole_mesh.bottom_radius = 0.006
	pole_mesh.height = 0.3
	pole.mesh = pole_mesh
	pole.position.y = 0.15
	var flag := MeshInstance3D.new()
	var flag_mesh := BoxMesh.new()
	flag_mesh.size = Vector3(0.1, 0.06, 0.004)
	flag.mesh = flag_mesh
	flag.position = Vector3(0.05, 0.27, 0)
	var ring := MeshInstance3D.new()
	var ring_mesh := TorusMesh.new()
	ring_mesh.inner_radius = NAV_ARRIVE - 0.01
	ring_mesh.outer_radius = NAV_ARRIVE + 0.01
	ring.mesh = ring_mesh
	ring.position.y = 0.005
	for mi in [pole, flag, ring]:
		(mi as MeshInstance3D).material_override = mat
		(mi as MeshInstance3D).cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		marker.add_child(mi)
	marker.visible = false
	return marker


func _make_nav_line() -> MeshInstance3D:
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color("ffb74d")
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	var line := MeshInstance3D.new()
	line.mesh = _nav_line_mesh
	line.material_override = mat
	line.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	return line


func _start_navigation(target: Vector3) -> void:
	_nav_target = target
	_nav_active = true
	_nav_marker.global_position = target
	_nav_marker.visible = true
	_nav_stuck_spots.clear()
	_plan_path()


func _cancel_navigation() -> void:
	if not _nav_active:
		return
	_nav_active = false
	_nav_marker.visible = false
	_nav_path.clear()
	_nav_stuck_spots.clear()
	_nav_replan_in = -1.0
	_nav_line_mesh.clear_surfaces()
	robot.turn = _sliders["turn"].value
	robot.drive_scale = 1.0
	_set_walking(false)


## Obstacles changed: plan again shortly (deleted bodies leave the physics space at frame end).
func _request_replan() -> void:
	if _nav_active and _nav_replan_in < 0.0:
		_nav_replan_in = NAV_REPLAN_DELAY


## Plans the waypoints to the target. With "Avoid obstacles" off, or when no path exists,
## the robot heads straight for the target as before.
func _plan_path() -> void:
	_nav_replan_in = -1.0
	_nav_no_path = false
	_nav_grid_valid = false
	_nav_path = PackedVector3Array([_nav_target])
	if _avoid_check == null or _avoid_check.button_pressed:
		var path := _nav_planner.plan(get_world_3d().direct_space_state, robot.body.global_position, _nav_target,
				robot.step_height + NAV_CLIMB_MARGIN, _nav_robot_radius(), _nav_stuck_spots)
		if path.is_empty():
			_nav_no_path = true
		else:
			_nav_path = path
			_nav_grid_valid = true
	_nav_best = INF
	_nav_best_time = _sim_time


## Planning radius: the farthest a foot gets from the body center (neutral foot spot plus half a
## stride, about 0.6 m for the default hexapod) plus some clearance.
func _nav_robot_radius() -> float:
	var reach := 0.0
	for leg in robot.legs:
		reach = maxf(reach, Vector2(leg.neutral.x, leg.neutral.z).length())
	return reach + absf(robot.stride_length) * 0.5 + SpiderRobot.FOOT_RADIUS + NAV_CLEARANCE


## Thin line on the ground from the robot through the remaining waypoints.
func _draw_nav_path() -> void:
	_nav_line_mesh.clear_surfaces()
	if not _nav_active or _nav_path.is_empty() or robot.body == null:
		return
	var points := PackedVector3Array([robot.body.global_position])
	points.append_array(_nav_path)
	_nav_line_mesh.surface_begin(Mesh.PRIMITIVE_LINE_STRIP)
	for i in points.size() - 1:
		var a := points[i]
		var b := points[i + 1]
		var steps := maxi(1, ceili(Vector2(b.x - a.x, b.z - a.z).length() / NavPlanner.CELL))
		for k in steps + 1:
			if k == 0 and i > 0:
				continue
			var p := a.lerp(b, float(k) / steps)
			p.y = _nav_planner.height_at(p) if _nav_grid_valid else _nav_target.y
			_nav_line_mesh.surface_add_vertex(p + Vector3(0, 0.012, 0))
	_nav_line_mesh.surface_end()


## Follows the planned waypoints: turn rate from the heading error to the current waypoint, forward
## speed reduced while it is off to the side (turns on the spot when it is behind) and when the
## target is close. The robot's normal stride / speed setting is the top speed. Re-plans when
## obstacles change or when the robot stops getting closer for NAV_STUCK_TIME.
func _update_navigation(delta: float) -> void:
	if not _nav_active or robot.body == null:
		return
	if robot.is_fallen():
		_cancel_navigation()
		return
	if _nav_replan_in >= 0.0:
		_nav_replan_in -= delta
		if _nav_replan_in < 0.0:
			_plan_path()
	var pos := robot.body.global_position
	var flat := Vector3(pos.x, 0.0, pos.z)
	while _nav_path.size() > 1 and flat.distance_to(Vector3(_nav_path[0].x, 0.0, _nav_path[0].z)) < NAV_WAYPOINT_REACH:
		_nav_path.remove_at(0)
	var waypoint := _nav_path[0]
	var to := Vector3(waypoint.x - pos.x, 0.0, waypoint.z - pos.z)
	var dist := to.length()
	_nav_remaining = dist
	for i in range(1, _nav_path.size()):
		_nav_remaining += Vector2(_nav_path[i].x - _nav_path[i - 1].x, _nav_path[i].z - _nav_path[i - 1].z).length()
	if _nav_path.size() == 1 and dist < NAV_ARRIVE:
		_cancel_navigation()
		return
	var forward := -robot.body.global_basis.z
	forward.y = 0.0
	forward = forward.normalized()
	var angle := forward.signed_angle_to(to / dist, Vector3.UP)
	robot.turn = clampf(angle * 2.0, -1.0, 1.0)
	var align := clampf(cos(angle), 0.0, 1.0)
	robot.drive_scale = align * align * clampf(_nav_remaining / 0.6, 0.1, 1.0)
	if not robot.walking:
		_set_walking(true)

	if _nav_remaining < _nav_best - 0.05:
		_nav_best = _nav_remaining
		_nav_best_time = _sim_time
	elif _sim_time - _nav_best_time > NAV_STUCK_TIME:
		# Walking forward but getting nowhere: something the grid missed is in the way.
		if absf(angle) < 0.5 and _avoid_check.button_pressed:
			var ahead := pos + forward * 0.3
			_nav_stuck_spots.append(Vector3(ahead.x, 0.12, ahead.z))
		_plan_path()
