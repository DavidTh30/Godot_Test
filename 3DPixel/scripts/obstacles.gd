class_name Obstacles
extends RefCounted
## Catalog of placeable obstacles and a builder for them. Every obstacle's origin is the center
## of its base, so it can be dropped straight onto the point the mouse ray hits.
## Static ones are StaticBody3D; "movable" ones are RigidBody3D the robot can push around.

const GROUP := "obstacle"

const CATALOG := [
	{"name": "Bump 3 cm", "shape": "box", "size": Vector3(0.3, 0.03, 0.3), "color": Color("6d7a88")},
	{"name": "Step 6 cm", "shape": "box", "size": Vector3(0.4, 0.06, 0.4), "color": Color("7d6e5a")},
	{"name": "Block 12 cm", "shape": "box", "size": Vector3(0.25, 0.12, 0.25), "color": Color("8a5a4a")},
	{"name": "Wall 30 cm", "shape": "box", "size": Vector3(0.8, 0.3, 0.06), "color": Color("9a4040")},
	{"name": "Pipe", "shape": "pipe", "radius": 0.035, "length": 0.8, "color": Color("a0a6ad")},
	{"name": "Ramp 9 deg", "shape": "ramp", "size": Vector3(0.5, 0.08, 0.4), "color": Color("5a7d62")},
	{"name": "Crate (movable)", "shape": "box", "size": Vector3(0.2, 0.2, 0.2), "color": Color("b5894f"), "mass": 1.0},
	{"name": "Ball (movable)", "shape": "sphere", "radius": 0.08, "color": Color("d0a030"), "mass": 0.3},
]


static func names() -> Array[String]:
	var out: Array[String] = []
	for def in CATALOG:
		out.append(def["name"])
	return out


## Builds a physics body for catalog entry `index`, collision layer = ground (the robot's feet
## sense and stand on it), at the given base position and yaw.
static func build(index: int, ground_material: PhysicsMaterial) -> CollisionObject3D:
	var def: Dictionary = CATALOG[index]
	var body: CollisionObject3D
	if def.has("mass"):
		var rb := RigidBody3D.new()
		rb.mass = def["mass"]
		rb.collision_mask = SpiderRobot.GROUND_LAYER | SpiderRobot.ROBOT_LAYER
		var pm := PhysicsMaterial.new()
		pm.friction = 0.6
		rb.physics_material_override = pm
		body = rb
	else:
		var sb := StaticBody3D.new()
		sb.collision_mask = 0
		sb.physics_material_override = ground_material
		body = sb
	body.collision_layer = SpiderRobot.GROUND_LAYER
	body.add_to_group(GROUP)
	body.set_meta("catalog_index", index)
	body.name = String(def["name"]).validate_node_name()

	var shape := CollisionShape3D.new()
	for mesh_instance in visuals(index):
		var mi := mesh_instance as MeshInstance3D
		var mat := StandardMaterial3D.new()
		mat.albedo_color = def["color"]
		mat.roughness = 0.7
		mi.material_override = mat
		body.add_child(mi)
		shape.transform = mi.transform
		match String(def["shape"]):
			"ramp":
				shape.shape = (mi.mesh as PrismMesh).create_convex_shape()
			_:
				shape.shape = _collision_shape(def)
	body.add_child(shape)
	return body


## Mesh nodes for an obstacle (also used for the translucent placement preview).
static func visuals(index: int) -> Array[Node3D]:
	var def: Dictionary = CATALOG[index]
	var mi := MeshInstance3D.new()
	match String(def["shape"]):
		"box":
			var box := BoxMesh.new()
			box.size = def["size"]
			mi.mesh = box
			mi.position.y = box.size.y * 0.5
		"sphere":
			var sphere := SphereMesh.new()
			sphere.radius = def["radius"]
			sphere.height = sphere.radius * 2.0
			mi.mesh = sphere
			mi.position.y = sphere.radius
		"pipe":
			var cyl := CylinderMesh.new()
			cyl.top_radius = def["radius"]
			cyl.bottom_radius = def["radius"]
			cyl.height = def["length"]
			mi.mesh = cyl
			mi.rotation.z = PI * 0.5  # lying along X
			mi.position.y = cyl.top_radius
		"ramp":
			var prism := PrismMesh.new()
			prism.size = def["size"]
			prism.left_to_right = 1.0  # right-angle wedge: vertical face at +X
			mi.mesh = prism
			mi.position.y = prism.size.y * 0.5
			mi.rotation.y = PI * 0.5  # rises toward -Z (the robot's walking direction)
	var out: Array[Node3D] = [mi]
	return out


static func _collision_shape(def: Dictionary) -> Shape3D:
	match String(def["shape"]):
		"box":
			var box := BoxShape3D.new()
			box.size = def["size"]
			return box
		"sphere":
			var sphere := SphereShape3D.new()
			sphere.radius = def["radius"]
			return sphere
		"pipe":
			var cyl := CylinderShape3D.new()
			cyl.radius = def["radius"]
			cyl.height = def["length"]
			return cyl
	return BoxShape3D.new()
