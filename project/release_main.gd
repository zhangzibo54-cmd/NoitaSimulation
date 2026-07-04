extends Node2D

var world
var info_label: Label
var param_labels := {}
var brush_material := 2 # 0 air, 1 rock, 2 water
var brush_radius := 7.0

func _ready() -> void:
	world = ClassDB.instantiate("NoitaWorld")
	if world == null:
		push_error("NoitaWorld was not registered. Rebuild the GDExtension DLL and restart Godot.")
		return
	world.position = Vector2(20, 36)
	world.set_world_size(320, 180)
	world.set_display_scale(3.0)
	world.set_pressure_iterations(8)
	world.set_pressure_release_radius(1)
	world.set_pressure_distance_decay(0.1)
	world.set_pressure_impulse_strength(1.0)
	world.set_gravity(0.28)
	world.set_simulation_speed(1.0)
	world.generate_basin()
	add_child(world)

	var layer := CanvasLayer.new()
	add_child(layer)

	info_label = Label.new()
	info_label.position = Vector2(12, 8)
	info_label.add_theme_color_override("font_color", Color(0.88, 0.94, 1.0))
	layer.add_child(info_label)

	_create_debug_panel(layer)

func _create_debug_panel(layer: CanvasLayer) -> void:
	var panel := PanelContainer.new()
	panel.position = Vector2(12, 135)
	layer.add_child(panel)

	var box := VBoxContainer.new()
	box.custom_minimum_size = Vector2(450, 0)
	panel.add_child(box)

	var title := Label.new()
	title.text = "C++ NoitaWorld debug sliders"
	box.add_child(title)

	_add_slider(box, "gravity", 0.0, 1.2, 0.01, world.get_gravity())
	_add_slider(box, "pressure_iterations", 0.0, 64.0, 1.0, world.get_pressure_iterations())
	_add_slider(box, "release_radius", 1.0, 16.0, 1.0, world.get_pressure_release_radius())
	_add_slider(box, "distance_decay", 0.0, 1.0, 0.01, world.get_pressure_distance_decay())
	_add_slider(box, "pressure_impulse", 0.0, 5.0, 0.01, world.get_pressure_impulse_strength())
	_update_param_labels()

func _add_slider(parent: VBoxContainer, key: String, min_value: float, max_value: float, step_value: float, value: float) -> void:
	var label := Label.new()
	parent.add_child(label)
	param_labels[key] = label

	var slider := HSlider.new()
	slider.min_value = min_value
	slider.max_value = max_value
	slider.step = step_value
	slider.value = value
	slider.custom_minimum_size = Vector2(420, 0)
	slider.value_changed.connect(_on_param_slider_changed.bind(key))
	parent.add_child(slider)

func _on_param_slider_changed(value: float, key: String) -> void:
	if world == null:
		return
	match key:
		"gravity":
			world.set_gravity(value)
		"pressure_iterations":
			world.set_pressure_iterations(int(round(value)))
		"release_radius":
			world.set_pressure_release_radius(int(round(value)))
		"distance_decay":
			world.set_pressure_distance_decay(value)
		"pressure_impulse":
			world.set_pressure_impulse_strength(value)
	_update_param_labels()

func _update_param_labels() -> void:
	if world == null:
		return
	if param_labels.has("gravity"):
		param_labels["gravity"].text = "gravity: %.2f cells/tick^2" % [world.get_gravity()]
	if param_labels.has("pressure_iterations"):
		param_labels["pressure_iterations"].text = "pressure iterations/step: %d" % [world.get_pressure_iterations()]
	if param_labels.has("release_radius"):
		param_labels["release_radius"].text = "pressure release radius: %d cells" % [world.get_pressure_release_radius()]
	if param_labels.has("distance_decay"):
		param_labels["distance_decay"].text = "distance decay: %.2f  weight = diff / (1 + dist * decay)" % [world.get_pressure_distance_decay()]
	if param_labels.has("pressure_impulse"):
		param_labels["pressure_impulse"].text = "pressure impulse strength: %.2f" % [world.get_pressure_impulse_strength()]

func _process(_delta: float) -> void:
	if world == null:
		return
	_handle_keyboard()
	_handle_mouse_paint()
	_update_param_labels()
	_update_info()

func _handle_keyboard() -> void:
	if Input.is_action_just_pressed("ui_accept"):
		world.set_paused(not world.is_paused())
	if Input.is_action_just_pressed("ui_cancel"):
		world.generate_basin()
	if Input.is_key_pressed(KEY_1):
		brush_material = 2 # water
	if Input.is_key_pressed(KEY_2):
		brush_material = 1 # rock
	if Input.is_key_pressed(KEY_3):
		brush_material = 0 # air/erase
	if Input.is_key_pressed(KEY_EQUAL):
		brush_radius = min(30.0, brush_radius + 0.25)
	if Input.is_key_pressed(KEY_MINUS):
		brush_radius = max(1.0, brush_radius - 0.25)
	if Input.is_key_pressed(KEY_BRACKETLEFT):
		world.set_pressure_iterations(max(0, world.get_pressure_iterations() - 1))
	if Input.is_key_pressed(KEY_BRACKETRIGHT):
		world.set_pressure_iterations(world.get_pressure_iterations() + 1)

func _handle_mouse_paint() -> void:
	var paint := false
	var material := brush_material
	if Input.is_mouse_button_pressed(MOUSE_BUTTON_LEFT):
		paint = true
		material = brush_material
	elif Input.is_mouse_button_pressed(MOUSE_BUTTON_RIGHT):
		paint = true
		material = 0
	if not paint:
		return
	var local: Vector2 = (get_viewport().get_mouse_position() - world.global_position) / float(world.get_display_scale())
	world.paint_circle(local.x, local.y, brush_radius, material)

func _update_info() -> void:
	var mat_name := "Water"
	if brush_material == 1:
		mat_name = "Rock"
	elif brush_material == 0:
		mat_name = "Air"

	var text := "NoitaWorld C++ mass/momentum/pressure prototype\n"
	text += "FPS: %d | target sim: %.0f steps/s | total sim steps: %d\n" % [int(Engine.get_frames_per_second()), 60.0 * world.get_simulation_speed(), world.get_step_count()]
	text += "last step: %.3f ms | gravity: %.2f | iters: %d | radius: %d | decay: %.2f | impulse: %.2f\n" % [world.get_last_step_ms(), world.get_gravity(), world.get_pressure_iterations(), world.get_pressure_release_radius(), world.get_pressure_distance_decay(), world.get_pressure_impulse_strength()]
	text += "water mass: %.1f | avg mass/water cell: %.3f | water cells: %d\n" % [world.get_total_water_mass(), world.get_average_water_mass(), world.get_water_cell_count()]
	text += "paused: %s | Controls: Space pause, Esc reset, 1 water, 2 rock, 3 air, LMB paint, RMB erase\n" % [str(world.is_paused())]
	text += "+/- brush, [/] pressure iters | Brush: %s radius %.1f" % [mat_name, brush_radius]
	info_label.text = text
