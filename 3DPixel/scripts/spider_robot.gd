class_name SpiderRobot
extends Node3D
## Physics-simulated spider robot.
##
## Chassis and leg segments are RigidBody3D nodes joined by HingeJoint3D. Every leg has 3 DOF:
## coxa (yaw), femur (pitch) and tibia (pitch). Each joint is driven like a hobby servo: a PD
## controller measures the real joint angle from the two bodies and applies equal and opposite
## torques, clamped to the servo's max torque. A gait generator plans foot targets in the body
## frame and analytic inverse kinematics turns them into joint angle targets.
##
## The robot node itself must stay at the identity transform; parts are placed in world space.

const COXA_LEN := 0.07
const FEMUR_LEN := 0.18
const TIBIA_LEN := 0.26
const FOOT_RADIUS := 0.022
const BODY_SIZE := Vector3(0.34, 0.09, 0.46)
const BODY_MASS := 2.0
const MAX_STRIDE := 0.22
const SERVO_DAMPING := 0.42  ## kd = SERVO_DAMPING * sqrt(I * kp); 0.3x the old value, see _make_leg
const MIN_SWING_TIME := 0.12  ## seconds; slow-swing gaits get a lower cadence so legs can keep up
const ROBOT_LAYER := 2
const GROUND_LAYER := 1

## Per leg count: [name, duty factor, phase offset per leg]. Leg order matches LAYOUTS.
const GAITS := {
	4: [
		["Trot", 0.55, [0.0, 0.5, 0.5, 0.0]],
		["Creep", 0.8, [0.25, 0.5, 0.75, 0.0]],
	],
	6: [
		["Tripod", 0.55, [0.0, 0.5, 0.0, 0.5, 0.0, 0.5]],
		["Ripple", 2.0 / 3.0, [2.0 / 3.0, 1.0 / 3.0, 0.0, 1.0 / 6.0, 5.0 / 6.0, 0.5]],
		["Wave", 5.0 / 6.0, [5.0 / 6.0, 4.0 / 6.0, 0.5, 2.0 / 6.0, 1.0 / 6.0, 0.0]],
	],
}

## Per leg count: [name, mount point on the chassis (body frame), mount yaw in degrees].
## Yaw 0 points along +X (right side); the robot's front is -Z.
const LAYOUTS := {
	4: [
		["L1", Vector3(-0.15, 0, -0.19), 135.0],
		["L2", Vector3(-0.15, 0, 0.19), 225.0],
		["R1", Vector3(0.15, 0, -0.19), 45.0],
		["R2", Vector3(0.15, 0, 0.19), -45.0],
	],
	6: [
		["L1", Vector3(-0.15, 0, -0.18), 135.0],
		["L2", Vector3(-0.17, 0, 0.0), 180.0],
		["L3", Vector3(-0.15, 0, 0.18), 225.0],
		["R1", Vector3(0.15, 0, -0.18), 45.0],
		["R2", Vector3(0.17, 0, 0.0), 0.0],
		["R3", Vector3(0.15, 0, 0.18), -45.0],
	],
}


class Servo:
	var parent: RigidBody3D
	var child: RigidBody3D
	var axis: Vector3  ## Hinge axis in the parent's frame.
	var rest: Basis  ## Child orientation relative to parent at angle 0.
	var yaw_axis: bool
	var damping: float  ## PD mode: kd = damping * sqrt(kp)
	var target := 0.0
	var angle := 0.0
	var torque := 0.0

	func measure() -> float:
		var rel := rest.inverse() * (parent.global_basis.orthonormalized().inverse() * child.global_basis.orthonormalized())
		if yaw_axis:
			return atan2(-rel.x.z, rel.x.x)
		return atan2(rel.x.y, rel.x.x)

	var integral := 0.0
	var joint: HingeJoint3D
	var _prev_target := 0.0
	var _has_prev := false

	## Stiff geared-servo model: the physics engine's hinge motor is solved inside the constraint
	## solver (implicit), so it holds position under load without the sag and vibration of an
	## explicit torque loop. Commanded joint speed = planned speed + gain * angle error.
	## The engine limits it to max_torque. Jolt's motor turns the child the opposite way to the
	## joint's +Z, and max_impulse is a per-step impulse, so torque = impulse * tick rate.
	func drive_motor(gain: float, max_speed: float, max_torque: float, dt: float) -> void:
		angle = measure()
		var target_rate := 0.0
		if _has_prev:
			target_rate = wrapf(target - _prev_target, -PI, PI) / dt
		_prev_target = target
		_has_prev = true
		var err := wrapf(target - angle, -PI, PI)
		var speed := clampf(target_rate + gain * err, -max_speed, max_speed)
		joint.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR, true)
		joint.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY, -speed)
		joint.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, max_torque * dt)
		torque = 0.0
		integral = 0.0

	## PD(+I) servo. With feedforward, the damping term acts on the error between the planned
	## joint speed and the real one, instead of braking every motion the gait asks for.
	func drive(kp: float, max_torque: float, dt: float, feedforward: bool, ki: float) -> void:
		joint.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR, false)
		angle = measure()
		var axis_w := (parent.global_basis * axis).normalized()
		var rel_speed := (child.angular_velocity - parent.angular_velocity).dot(axis_w)
		var target_rate := 0.0
		if feedforward and _has_prev:
			target_rate = wrapf(target - _prev_target, -PI, PI) / dt
		_prev_target = target
		_has_prev = true
		var err := wrapf(target - angle, -PI, PI)
		if ki > 0.0:
			# Anti-windup: the integral may contribute at most half of the torque limit.
			var limit := max_torque * 0.5 / (kp * ki)
			integral = clampf(integral + err * dt, -limit, limit)
		else:
			integral = 0.0
		torque = kp * (err + ki * integral) + damping * sqrt(kp) * (target_rate - rel_speed)
		torque = clampf(torque, -max_torque, max_torque)
		child.apply_torque(axis_w * torque)
		parent.apply_torque(-axis_w * torque)


class Leg:
	var name: String
	var mount_pos: Vector3
	var mount_basis: Basis
	var neutral := Vector3.ZERO
	var coxa: RigidBody3D
	var femur: RigidBody3D
	var tibia: RigidBody3D
	var servos: Array[Servo] = []
	var target := Vector3.ZERO  ## Foot target in the body frame.
	var level := 0.0  ## Height correction from the balance controller.
	var ground_y := FOOT_RADIUS  ## World height of the foot center when it last stood on the ground.
	var contact_y := 0.0
	var foot_pos := Vector3.ZERO
	var touched := false
	var stance := true
	var contact := false
	var marker: MeshInstance3D


# Tunable parameters (the lab UI writes these directly).
var leg_count := 6
var gait_index := 0
var walking := false
var balance := true
var terrain_scan := true  ## raycast the landing spot so swing feet aim at the real ground height
var speed := 0.18  ## m/s forward (speed command mode)
## 0 = speed: stride length follows speed / frequency (faster stepping = shorter steps).
## 1 = stride length: each step covers stride_length, so speed = stride * frequency / duty
##     (faster stepping = faster walking with the same long steps).
var command_mode := 1
var stride_length := 0.12  ## m of foot travel per stance (stride mode); negative walks backward
var drive_scale := 1.0  ## 0..1 multiplier on forward speed (used by click-to-walk to slow down)
var strafe := 0.0  ## m/s to the right
var turn := 0.0  ## rad/s, positive turns left
var step_frequency := 1.5  ## gait cycles per second
var step_height := 0.05
var stand_height := 0.145  ## body center above the ground
var stance_reach := 0.32  ## horizontal distance from mount to neutral foot position
var servo_kp := 115.0  ## N*m per rad
var servo_max_torque := 8.0  ## N*m
var show_targets := true
var body_bob := 0.0  ## m, optional body dip/rise twice per gait cycle (0 = flat ride)
var velocity_feedforward := false
var servo_ki := 0.0  ## 1/s, integral gain relative to kp (0 = plain PD)
## Integral correction of body height. Off by default: with stiff motor servos and enough solver
## iterations (project.godot: jolt velocity_steps = 45) the body already holds its height, and
## this extra loop made the standing robot oscillate ~3 mm at 8 Hz. Useful with soft PD servos.
var height_hold := false
var height_hold_gain := 3.0  ## 1/s
var servo_mode := 1  ## 0 = PD torque loop (soft), 1 = engine motor (stiff geared servo)
var motor_gain := 15.0  ## 1/s, joint speed per radian of error in motor mode
var servo_max_speed := 10.0  ## rad/s, like a hobby servo's no-load speed
var stance_press := 0.004  ## m, stance feet aim this far below the ground they found
var servo_lead := 0.03  ## s, gait commands run this far ahead to cover servo lag
var ground_speed_matching := true  ## swing feet leave/meet the ground at stance speed
var gyro_feedback := true  ## correct the turn command from the measured yaw rate (holds heading when turn = 0)

var body: RigidBody3D
var legs: Array[Leg] = []
var phase := 0.0

var _stride_blend := 0.0
var _height_bias := 0.0
var _height_err_avg := 0.0
var debug_body_ref := 0.0  ## last body height reference (for tests / telemetry)
var _yaw_rate_avg := 0.0
var _turn_trim := 0.0
var _turn_cmd := 0.0
var _materials := {}


func build(spawn: Transform3D) -> void:
	for child in get_children():
		remove_child(child)
		child.queue_free()
	legs.clear()
	phase = 0.0
	_stride_blend = 0.0
	_height_bias = 0.0
	_yaw_rate_avg = 0.0
	_turn_trim = 0.0
	if not GAITS.has(leg_count):
		leg_count = 6
	gait_index = clampi(gait_index, 0, GAITS[leg_count].size() - 1)

	var body_xf := spawn
	body_xf.origin.y += stand_height + 0.01
	body = _make_body()
	body.transform = body_xf
	add_child(body)
	for def in LAYOUTS[leg_count]:
		legs.append(_make_leg(def, body_xf))


static func gait_names(count: int) -> Array[String]:
	var names: Array[String] = []
	for g in GAITS[count]:
		names.append(g[0])
	return names


func _physics_process(delta: float) -> void:
	if body == null or legs.is_empty():
		return
	var gait: Array = GAITS[leg_count][gait_index]
	var duty: float = gait[1]
	var offsets: Array = gait[2]
	_stride_blend = move_toward(_stride_blend, 1.0 if walking else 0.0, delta * 3.0)
	if _stride_blend > 0.0:
		phase = fposmod(phase + delta * cadence(), 1.0)
	_update_balance(delta)
	_update_turn(delta)

	# The body is held level, stand_height above the average ground the feet have found. Foot
	# targets assume that pose, so any tilt or sag is pushed back by the legs.
	var ground_mean := 0.0
	for leg in legs:
		ground_mean += leg.ground_y
	var body_ref_y := ground_mean / legs.size() - FOOT_RADIUS + stand_height
	# Slight bob, twice per gait cycle: the body dips as feet plant and rises as they push off.
	body_ref_y += body_bob * cos(TAU * 2.0 * phase) * _stride_blend
	# Integral correction so the body holds its height under load instead of sagging.
	if not height_hold:
		_height_bias = 0.0
	elif body.global_basis.y.y > 0.5:
		_height_err_avg = lerpf(_height_err_avg, body_ref_y - body.global_position.y, 1.0 - exp(-delta / 0.4))
		_height_bias = clampf(_height_bias + _height_err_avg * height_hold_gain * delta, -0.05, 0.05)
	body_ref_y += _height_bias
	debug_body_ref = body_ref_y

	var body_xf := body.global_transform
	var space := get_world_3d().direct_space_state
	for i in legs.size():
		var leg := legs[i]
		_sense_ground(space, leg)
		leg.neutral = _neutral(leg)
		var foot_y := _plan_foot(leg, fposmod(phase + offsets[i] + servo_lead * cadence() * _stride_blend, 1.0), duty, delta, space, body_xf)
		leg.target = Vector3(leg.target.x, foot_y - body_ref_y + leg.level, leg.target.z)
		var angles := _ik(leg, leg.target)
		leg.servos[0].target = angles.x
		leg.servos[1].target = angles.y
		leg.servos[2].target = angles.z
		for servo in leg.servos:
			if servo_mode == 1:
				servo.drive_motor(motor_gain, servo_max_speed, servo_max_torque, delta)
			else:
				servo.drive(servo_kp, servo_max_torque, delta, velocity_feedforward, servo_ki)
		leg.marker.visible = show_targets
		leg.marker.position = body_xf * leg.target


## Ground contact = ground within a few mm below the foot sphere. (Jolt's reported contacts include
## speculative ones a couple of cm away, which would mark lifted feet as touching.)
func _sense_ground(space: PhysicsDirectSpaceState3D, leg: Leg) -> void:
	leg.foot_pos = leg.tibia.global_transform * Vector3(TIBIA_LEN, 0, 0)
	var query := PhysicsRayQueryParameters3D.create(leg.foot_pos + Vector3.UP * 0.01, leg.foot_pos + Vector3.DOWN * (FOOT_RADIUS + 0.006), GROUND_LAYER)
	var hit := space.intersect_ray(query)
	leg.contact = not hit.is_empty()
	if leg.contact:
		leg.contact_y = hit["position"].y + FOOT_RADIUS


# --- Gait and kinematics --------------------------------------------------------

func _neutral(leg: Leg) -> Vector3:
	var p := leg.mount_pos + leg.mount_basis * Vector3.RIGHT * stance_reach
	p.y = -stand_height + FOOT_RADIUS
	return p


## Plans the foot at gait phase p. Writes the horizontal target (body frame) into leg.target and
## returns the wanted world height of the foot center.
## Stance: the foot slides backward along the velocity it must cancel, pushing the body forward,
## pressed onto the ground height it found (and reaching lower if the ground drops away).
## Swing: it arcs forward above the last ground height and stops descending as soon as it touches.
func _plan_foot(leg: Leg, p: float, duty: float, delta: float, space: PhysicsDirectSpaceState3D, body_xf: Transform3D) -> float:
	var cycle := 1.0 / cadence()
	# Velocity of the ground under this foot, seen from the body: translation plus turn (w x r).
	var vel := Vector3(strafe, 0, -planned_speed()) + Vector3(leg.neutral.z, 0, -leg.neutral.x) * _turn_cmd
	var stride := (vel * duty * cycle * _stride_blend).limit_length(MAX_STRIDE)
	var offset: Vector3
	var foot_y: float
	if p < duty:
		leg.stance = true
		offset = stride * (0.5 - p / duty)
		if leg.contact:
			leg.ground_y = leg.contact_y
		elif leg.foot_pos.y <= leg.ground_y + 0.003:
			# The foot already reached the expected ground height and still touches nothing, so the
			# ground is lower here: reach down. (Probing while the foot is merely late to land would
			# wrongly lower the ground estimate and sink the body on every step.)
			leg.ground_y = maxf(leg.ground_y - 0.25 * delta, body.global_position.y - 0.4)
		foot_y = leg.ground_y - stance_press
	else:
		if leg.stance:
			leg.touched = false
		leg.stance = false
		var u := (p - duty) / (1.0 - duty)
		# Ground-speed matching: the swing starts and ends with the same backward speed the foot had
		# in stance, so it leaves and meets the ground without sliding (a cubic Hermite curve from
		# -0.5 to +0.5 stride). Sliding at lift-off/touchdown was what braked and dipped the body.
		var carry := _swing_carry(u, duty) if ground_speed_matching else _min_jerk(clampf((u - 0.08) / 0.77, 0.0, 1.0))
		offset = stride * (carry - 0.5)
		# Early touchdown only counts near the end of the swing; earlier the foot keeps its arc
		# so it isn't dragged forward into the side of an obstacle.
		if u > 0.8 and leg.contact:
			leg.touched = true
			leg.ground_y = leg.contact_y
		# Glide from the take-off height to the scanned landing height as the foot moves forward.
		foot_y = lerpf(leg.ground_y, _landing_height(leg, stride, space, body_xf), clampf(carry, 0.0, 1.0))
		if leg.touched:
			foot_y = leg.ground_y
		else:
			var rise := _min_jerk(clampf(u / 0.25, 0.0, 1.0))
			var fall := _min_jerk(clampf((u - 0.6) / 0.4, 0.0, 1.0))
			foot_y += step_height * (rise - fall) * _stride_blend
	leg.target = leg.neutral + offset
	return foot_y


## World height (foot center) of the ground where this swing will land, or the take-off height
## when terrain scanning is off or nothing is found.
func _landing_height(leg: Leg, stride: Vector3, space: PhysicsDirectSpaceState3D, body_xf: Transform3D) -> float:
	if not terrain_scan:
		return leg.ground_y
	var land := leg.neutral + stride * 0.5
	var above := body_xf * Vector3(land.x, 0.0, land.z)
	above.y = maxf(above.y, leg.ground_y) + 0.15
	var query := PhysicsRayQueryParameters3D.create(above, above + Vector3.DOWN * 0.8, GROUND_LAYER)
	var hit := space.intersect_ray(query)
	if hit.is_empty():
		return leg.ground_y
	return hit["position"].y + FOOT_RADIUS


## Forward progress of a swing foot (0 = back of the stride, 1 = front) at swing time u (0..1).
## Hermite curve whose end slopes equal the stance speed, so foot speed is continuous at lift-off and touchdown.
static func _swing_carry(u: float, duty: float) -> float:
	var m := -(1.0 - duty) / duty  # stance slope in swing-time units
	var u2 := u * u
	var u3 := u2 * u
	return (-2.0 * u3 + 3.0 * u2) + (u3 - 2.0 * u2 + u) * m + (u3 - u2) * m


## Smooth 0..1 ramp with zero velocity and acceleration at both ends (quintic).
static func _min_jerk(t: float) -> float:
	return t * t * t * (10.0 + t * (-15.0 + 6.0 * t))


## Analytic 3-DOF IK. Returns (coxa yaw, femur pitch, tibia pitch) for a body-frame foot target.
## Knee-up solution: the femur rises from the hip and the tibia bends down to the foot.
func _ik(leg: Leg, target: Vector3) -> Vector3:
	var local := leg.mount_basis.inverse() * (target - leg.mount_pos)
	var yaw := atan2(-local.z, local.x)
	var r := Vector2(local.x, local.z).length() - COXA_LEN
	var h := local.y
	var d := clampf(sqrt(r * r + h * h), 0.05, FEMUR_LEN + TIBIA_LEN - 0.001)
	var hip := atan2(h, r) + acos(clampf((FEMUR_LEN * FEMUR_LEN + d * d - TIBIA_LEN * TIBIA_LEN) / (2.0 * FEMUR_LEN * d), -1.0, 1.0))
	var knee := acos(clampf((FEMUR_LEN * FEMUR_LEN + TIBIA_LEN * TIBIA_LEN - d * d) / (2.0 * FEMUR_LEN * TIBIA_LEN), -1.0, 1.0))
	return Vector3(yaw, hip, -(PI - knee))


## Gyro turn-rate loop. While turning, the swinging legs' momentum makes the stance feet slip
## and the body over-rotates (~40% with stiff servos); when walking straight it slowly drifts.
## An integral trim on the turn command makes the measured (0.3 s averaged) yaw rate match it.
func _update_turn(delta: float) -> void:
	var yaw_rate := body.angular_velocity.dot(body.global_basis.y)
	_yaw_rate_avg = lerpf(_yaw_rate_avg, yaw_rate, 1.0 - exp(-delta / 0.3))
	if gyro_feedback and _stride_blend > 0.5 and body.global_basis.y.y > 0.5:
		_turn_trim = clampf(_turn_trim + (turn - _yaw_rate_avg) * 1.5 * delta, -1.0, 1.0)
	elif not gyro_feedback:
		_turn_trim = move_toward(_turn_trim, 0.0, delta)
	_turn_cmd = turn + _turn_trim


## IMU-style leveling: legs under a low corner of the chassis extend, legs under a high corner retract.
func _update_balance(delta: float) -> void:
	var b := body.global_basis
	var upright := b.y.y > 0.5
	var mean := 0.0
	for leg in legs:
		if balance and upright:
			var drop := (b * leg.mount_pos).y
			leg.level = clampf(leg.level + drop * 6.0 * delta, -0.06, 0.06)
		else:
			leg.level = move_toward(leg.level, 0.0, delta * 0.1)
		mean += leg.level
	mean /= legs.size()
	for leg in legs:
		leg.level -= mean


# --- Telemetry --------------------------------------------------------------------

## Forward speed the gait is planned for: the speed command, or stride * cadence / duty in stride mode.
func planned_speed() -> float:
	if command_mode == 1:
		var duty: float = GAITS[leg_count][gait_index][1]
		return stride_length * cadence() / duty * drive_scale
	return speed * drive_scale


## Effective gait cycles per second: the step frequency, capped so swings last at least MIN_SWING_TIME.
func cadence() -> float:
	var duty: float = GAITS[leg_count][gait_index][1]
	return minf(step_frequency, (1.0 - duty) / MIN_SWING_TIME)


func forward_speed() -> float:
	var forward := -body.global_basis.z
	forward.y = 0.0
	if forward.length() < 0.01:
		return 0.0
	return body.linear_velocity.dot(forward.normalized())


func heading_deg() -> float:
	var forward := -body.global_basis.z
	return rad_to_deg(atan2(-forward.x, -forward.z))


func tilt_deg() -> float:
	return rad_to_deg(acos(clampf(body.global_basis.y.y, -1.0, 1.0)))


func is_fallen() -> bool:
	return body.global_basis.y.y < 0.35


func joint_error_rms_deg() -> float:
	var sum := 0.0
	var n := 0
	for leg in legs:
		for servo in leg.servos:
			var err := wrapf(servo.target - servo.angle, -PI, PI)
			sum += err * err
			n += 1
	return rad_to_deg(sqrt(sum / maxi(n, 1)))


## Average servo torque as a fraction of the torque limit (0..1). Only measurable in PD mode:
## the engine motor's torque is solved internally and not reported, so this returns -1 there.
func servo_load() -> float:
	if servo_mode == 1:
		return -1.0
	var sum := 0.0
	var n := 0
	for leg in legs:
		for servo in leg.servos:
			sum += absf(servo.torque)
			n += 1
	return sum / maxi(n, 1) / servo_max_torque


func push(strength: float) -> void:
	var dir := Vector3(randf_range(-1, 1), 0, randf_range(-1, 1)).normalized()
	body.apply_central_impulse(dir * strength + Vector3.UP * strength * 0.3)


# --- Construction -----------------------------------------------------------------

func _make_body() -> RigidBody3D:
	var rb := RigidBody3D.new()
	rb.name = "Chassis"
	rb.mass = BODY_MASS
	rb.collision_layer = ROBOT_LAYER
	rb.collision_mask = GROUND_LAYER
	rb.can_sleep = false
	rb.physics_material_override = _physics_material(0.5)
	var shape := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = BODY_SIZE
	shape.shape = box
	rb.add_child(shape)
	_visual_box(rb, BODY_SIZE, Vector3.ZERO, "frame")
	_visual_box(rb, Vector3(BODY_SIZE.x * 0.86, 0.025, BODY_SIZE.z * 0.84), Vector3(0, BODY_SIZE.y * 0.5 + 0.012, 0), "shell")
	_visual_box(rb, Vector3(0.12, 0.03, 0.16), Vector3(0, BODY_SIZE.y * 0.5 + 0.035, 0.04), "board")
	_visual_box(rb, Vector3(0.14, 0.04, 0.02), Vector3(0, 0.0, -BODY_SIZE.z * 0.5 - 0.01), "frame")
	for x: float in [-0.035, 0.035]:
		var eye := MeshInstance3D.new()
		var sphere := SphereMesh.new()
		sphere.radius = 0.012
		sphere.height = 0.024
		eye.mesh = sphere
		eye.material_override = _material("eye")
		eye.position = Vector3(x, 0.005, -BODY_SIZE.z * 0.5 - 0.022)
		rb.add_child(eye)
	return rb


func _make_leg(def: Array, body_xf: Transform3D) -> Leg:
	var leg := Leg.new()
	leg.name = def[0]
	leg.mount_pos = def[1]
	leg.mount_basis = Basis(Vector3.UP, deg_to_rad(def[2]))
	leg.neutral = _neutral(leg)
	leg.target = leg.neutral
	leg.ground_y = body_xf.origin.y - stand_height + FOOT_RADIUS
	var a := _ik(leg, leg.neutral)

	# Spawn the segments already in the standing pose so the robot starts on its feet.
	var mount_xf := body_xf * Transform3D(leg.mount_basis, leg.mount_pos)
	var coxa_xf := mount_xf * Transform3D(Basis(Vector3.UP, a.x), Vector3.ZERO)
	var femur_xf := coxa_xf * Transform3D(Basis(Vector3.BACK, a.y), Vector3(COXA_LEN, 0, 0))
	var tibia_xf := femur_xf * Transform3D(Basis(Vector3.BACK, a.z), Vector3(FEMUR_LEN, 0, 0))
	leg.coxa = _segment(leg.name + "_coxa", coxa_xf, COXA_LEN, 0.05, 0.08, true, false)
	leg.femur = _segment(leg.name + "_femur", femur_xf, FEMUR_LEN, 0.035, 0.15, false, false)
	leg.tibia = _segment(leg.name + "_tibia", tibia_xf, TIBIA_LEN, 0.03, 0.12, false, true)

	# Damping from rough inertia estimates of what each joint moves (kg*m^2). The factor stays well
	# below critical (2*sqrt(I)): with explicit torques at 240 Hz, more damping over-corrects every
	# step on the light segments and the legs chatter at ~65 Hz, which shakes the whole robot.
	leg.servos.append(_servo(body, leg.coxa, Vector3.UP, leg.mount_basis, true, SERVO_DAMPING * sqrt(0.015)))
	leg.servos.append(_servo(leg.coxa, leg.femur, Vector3.BACK, Basis(), false, SERVO_DAMPING * sqrt(0.006)))
	leg.servos.append(_servo(leg.femur, leg.tibia, Vector3.BACK, Basis(), false, SERVO_DAMPING * sqrt(0.002)))
	leg.servos[0].target = a.x
	leg.servos[1].target = a.y
	leg.servos[2].target = a.z

	leg.marker = MeshInstance3D.new()
	var dot := SphereMesh.new()
	dot.radius = 0.014
	dot.height = 0.028
	leg.marker.mesh = dot
	leg.marker.material_override = _material("marker")
	leg.marker.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(leg.marker)
	return leg


func _servo(parent: RigidBody3D, child: RigidBody3D, axis: Vector3, rest: Basis, yaw: bool, damping: float) -> Servo:
	var servo := Servo.new()
	servo.parent = parent
	servo.child = child
	servo.axis = axis
	servo.rest = rest
	servo.yaw_axis = yaw
	servo.damping = damping

	# HingeJoint3D rotates around its own local Z axis.
	var axis_w := (parent.transform.basis * axis).normalized()
	var side := axis_w.cross(Vector3.UP)
	if side.length() < 0.1:
		side = axis_w.cross(Vector3.RIGHT)
	side = side.normalized()
	var joint := HingeJoint3D.new()
	joint.transform = Transform3D(Basis(side, axis_w.cross(side), axis_w), child.transform.origin)
	add_child(joint)
	joint.node_a = joint.get_path_to(parent)
	joint.node_b = joint.get_path_to(child)
	servo.joint = joint
	return servo


## A leg segment: its origin is the proximal joint and it extends along local +X.
func _segment(seg_name: String, xf: Transform3D, length: float, thickness: float, mass: float, vertical_hub: bool, foot: bool) -> RigidBody3D:
	var rb := RigidBody3D.new()
	rb.name = seg_name
	rb.mass = mass
	rb.collision_layer = ROBOT_LAYER
	rb.collision_mask = GROUND_LAYER
	rb.can_sleep = false
	rb.physics_material_override = _physics_material(1.2 if foot else 0.6)
	rb.transform = xf

	var shape := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = Vector3(length, thickness, thickness)
	shape.shape = box
	shape.position.x = length * 0.5
	rb.add_child(shape)
	_visual_box(rb, Vector3(length, thickness, thickness * 1.2), Vector3(length * 0.5, 0, 0), "leg")

	# Servo housing at the joint.
	var hub := MeshInstance3D.new()
	var cylinder := CylinderMesh.new()
	cylinder.top_radius = thickness * 0.8
	cylinder.bottom_radius = thickness * 0.8
	cylinder.height = thickness * 1.7
	hub.mesh = cylinder
	hub.material_override = _material("servo")
	if not vertical_hub:
		hub.rotation.x = PI * 0.5
	rb.add_child(hub)

	if foot:
		var foot_shape := CollisionShape3D.new()
		var sphere := SphereShape3D.new()
		sphere.radius = FOOT_RADIUS
		foot_shape.shape = sphere
		foot_shape.position.x = length
		rb.add_child(foot_shape)
		var tip := MeshInstance3D.new()
		var tip_mesh := SphereMesh.new()
		tip_mesh.radius = FOOT_RADIUS
		tip_mesh.height = FOOT_RADIUS * 2.0
		tip.mesh = tip_mesh
		tip.position.x = length
		tip.material_override = _material("rubber")
		rb.add_child(tip)

	add_child(rb)
	return rb


func _visual_box(parent: Node3D, size: Vector3, pos: Vector3, mat_name: String) -> void:
	var mi := MeshInstance3D.new()
	var mesh := BoxMesh.new()
	mesh.size = size
	mi.mesh = mesh
	mi.position = pos
	mi.material_override = _material(mat_name)
	parent.add_child(mi)


func _material(mat_name: String) -> StandardMaterial3D:
	if _materials.has(mat_name):
		return _materials[mat_name]
	var m := StandardMaterial3D.new()
	match mat_name:
		"frame":
			m.albedo_color = Color("3a3d42")
			m.metallic = 0.6
			m.roughness = 0.5
		"shell":
			m.albedo_color = Color("6b3f2c")
			m.roughness = 0.6
		"board":
			m.albedo_color = Color("2f5d3a")
			m.roughness = 0.7
		"leg":
			m.albedo_color = Color("b9bec6")
			m.metallic = 0.7
			m.roughness = 0.35
		"servo":
			m.albedo_color = Color("1f2124")
			m.roughness = 0.6
		"rubber":
			m.albedo_color = Color("111214")
			m.roughness = 0.95
		"eye":
			m.albedo_color = Color("ff3b2f")
			m.emission_enabled = true
			m.emission = Color("ff3b2f")
			m.emission_energy_multiplier = 2.0
		"marker":
			m.albedo_color = Color(0.2, 1.0, 0.5, 0.8)
			m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
			m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
			m.no_depth_test = true
	_materials[mat_name] = m
	return m


func _physics_material(friction: float) -> PhysicsMaterial:
	var key := "phys_%s" % friction
	if _materials.has(key):
		return _materials[key]
	var pm := PhysicsMaterial.new()
	pm.friction = friction
	_materials[key] = pm
	return pm
