extends Node2D

var world
var info_label: Label
var param_labels := {}
var brush_material := 2 # 0 air, 1 rock, 2 water
var brush_radius := 6.0
var inject_horizontal_speed := 0.0

func _ready() -> void:
	world = ClassDB.instantiate("MacWorld")
	if world == null:
		push_error("MacWorld was not registered. Rebuild the GDExtension DLL and restart Godot.")
		return

	world.position = Vector2(20, 36)
	world.set_world_size(320, 180)
	world.set_display_scale(3.0)
	world.set_dt(0.18)
	world.set_gravity(1.00)
	world.set_viscosity(0.04)
	world.set_pressure_iterations(20)
	if world.has_method("set_pressure_active_mass"):
		world.set_pressure_active_mass(0.30)
	if world.has_method("set_density_correction_strength"):
		world.set_density_correction_strength(0.20)
	if world.has_method("set_underfill_correction_strength"):
		world.set_underfill_correction_strength(0.40)
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
	call_deferred("_enter_fullscreen")

func _enter_fullscreen() -> void:
	var window := get_window()
	if window != null:
		window.borderless = true
		window.mode = Window.MODE_EXCLUSIVE_FULLSCREEN
	DisplayServer.window_set_flag(DisplayServer.WINDOW_FLAG_BORDERLESS, true)
	DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_EXCLUSIVE_FULLSCREEN)
	await get_tree().process_frame
	if window != null:
		window.mode = Window.MODE_EXCLUSIVE_FULLSCREEN
	DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_EXCLUSIVE_FULLSCREEN)
	await get_tree().process_frame
	_fit_world_to_viewport()

func _notification(what: int) -> void:
	if what == NOTIFICATION_WM_SIZE_CHANGED:
		_fit_world_to_viewport()

func _fit_world_to_viewport() -> void:
	if world == null:
		return
	var margin := Vector2(20.0, 36.0)
	world.position = margin
	var available := get_viewport_rect().size - margin - Vector2(20.0, 20.0)
	var sx := available.x / float(world.get_width())
	var sy := available.y / float(world.get_height())
	world.set_display_scale(max(0.25, min(sx, sy)))

func _create_debug_panel(layer: CanvasLayer) -> void:
	var panel := PanelContainer.new()
	panel.position = Vector2(12, 135)
	layer.add_child(panel)

	var box := VBoxContainer.new()
	box.custom_minimum_size = Vector2(470, 0)
	panel.add_child(box)

	var title := Label.new()
	title.text = "MAC grid + PCG pressure projection"
	box.add_child(title)

	_add_slider(box, "dt", 0.02, 0.5, 0.01, world.get_dt())
	_add_slider(box, "gravity", 0.0, 2.5, 0.01, world.get_gravity())
	_add_slider(box, "viscosity", 0.0, 0.25, 0.005, world.get_viscosity())
	_add_slider(box, "pressure_iterations", 1.0, 250.0, 1.0, world.get_pressure_iterations())
	if world.has_method("get_pressure_active_mass"):
		_add_slider(box, "pressure_active_mass", 0.01, 1.0, 0.01, world.get_pressure_active_mass())
	if world.has_method("get_density_correction_strength"):
		_add_slider(box, "density_correction", 0.0, 2.0, 0.01, world.get_density_correction_strength())
	if world.has_method("get_underfill_correction_strength"):
		_add_slider(box, "underfill_correction", 0.0, 2.0, 0.01, world.get_underfill_correction_strength())
	_add_slider(box, "inject_horizontal_speed", -6.0, 6.0, 0.1, inject_horizontal_speed)
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
	slider.custom_minimum_size = Vector2(440, 0)
	slider.value_changed.connect(_on_param_slider_changed.bind(key))
	parent.add_child(slider)

func _on_param_slider_changed(value: float, key: String) -> void:
	if world == null:
		return
	match key:
		"dt":
			world.set_dt(value)
		"gravity":
			world.set_gravity(value)
		"viscosity":
			world.set_viscosity(value)
		"pressure_iterations":
			world.set_pressure_iterations(int(round(value)))
		"pressure_active_mass":
			if world.has_method("set_pressure_active_mass"):
				world.set_pressure_active_mass(value)
		"density_correction":
			if world.has_method("set_density_correction_strength"):
				world.set_density_correction_strength(value)
		"underfill_correction":
			if world.has_method("set_underfill_correction_strength"):
				world.set_underfill_correction_strength(value)
		"inject_horizontal_speed":
			inject_horizontal_speed = value
	_update_param_labels()

func _update_param_labels() -> void:
	if world == null:
		return
	if param_labels.has("dt"):
		param_labels["dt"].text = "dt: %.3f tick" % [world.get_dt()]
	if param_labels.has("gravity"):
		param_labels["gravity"].text = "gravity: %.2f cells/tick^2" % [world.get_gravity()]
	if param_labels.has("viscosity"):
		param_labels["viscosity"].text = "viscosity nu: %.3f" % [world.get_viscosity()]
	if param_labels.has("pressure_iterations"):
		param_labels["pressure_iterations"].text = "PCG max iterations: %d" % [world.get_pressure_iterations()]
	if param_labels.has("pressure_active_mass"):
		param_labels["pressure_active_mass"].text = "pressure active mass threshold: %.2f" % [_get_pressure_active_mass()]
	if param_labels.has("density_correction"):
		param_labels["density_correction"].text = "density correction strength: %.2f  target div = k * max(mass-1,0)/dt" % [_get_density_correction_strength()]
	if param_labels.has("underfill_correction"):
		param_labels["underfill_correction"].text = "internal underfill strength: %.2f  only internal mass<1 gets negative div" % [_get_underfill_correction_strength()]
	if param_labels.has("inject_horizontal_speed"):
		param_labels["inject_horizontal_speed"].text = "LMB water initial horizontal speed: %.1f" % [inject_horizontal_speed]

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
	if Input.is_key_pressed(KEY_F11):
		_enter_fullscreen()
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
		world.set_pressure_iterations(max(1, world.get_pressure_iterations() - 1))
	if Input.is_key_pressed(KEY_BRACKETRIGHT):
		world.set_pressure_iterations(world.get_pressure_iterations() + 1)

func _handle_mouse_paint() -> void:
	var use_tool := false
	var material := brush_material
	if Input.is_mouse_button_pressed(MOUSE_BUTTON_LEFT):
		use_tool = true
		material = brush_material
	elif Input.is_mouse_button_pressed(MOUSE_BUTTON_RIGHT):
		use_tool = true
		material = 0
	if not use_tool:
		return

	var local: Vector2 = (get_viewport().get_mouse_position() - world.global_position) / float(world.get_display_scale())
	if material == 2 and abs(inject_horizontal_speed) > 0.001:
		world.inject_water(local.x, local.y, brush_radius, 1.0, inject_horizontal_speed, 0.0)
	else:
		world.paint_circle(local.x, local.y, brush_radius, material)

func _update_info() -> void:
	var mat_name := "Water"
	if brush_material == 1:
		mat_name = "Rock"
	elif brush_material == 0:
		mat_name = "Air"

	var text := "MacWorld: MAC grid + explicit advection/viscosity + PCG pressure projection\n"
	text += "FPS: %d | target sim: %.0f steps/s | total sim steps: %d | last step: %.3f ms\n" % [int(Engine.get_frames_per_second()), 60.0 * world.get_simulation_speed(), world.get_step_count(), world.get_last_step_ms()]
	text += "dt: %.3f | gravity: %.2f | viscosity: %.3f | active: %.2f | over: %.2f | under: %.2f | PCG: %d/%d residual %.2e\n" % [world.get_dt(), world.get_gravity(), world.get_viscosity(), _get_pressure_active_mass(), _get_density_correction_strength(), _get_underfill_correction_strength(), world.get_last_pcg_iterations(), world.get_pressure_iterations(), world.get_last_pcg_residual()]
	text += "water mass: %.1f | avg mass/water cell: %.3f | water cells: %d\n" % [world.get_total_water_mass(), world.get_average_water_mass(), world.get_water_cell_count()]
	text += "paused: %s | Controls: Space pause, Esc reset, 1 water, 2 rock, 3 air, LMB paint/inject, RMB erase\n" % [str(world.is_paused())]
	text += "+/- brush, [/] PCG iters | Brush: %s radius %.1f | LMB water vx %.1f" % [mat_name, brush_radius, inject_horizontal_speed]
	info_label.text = text

func _get_pressure_active_mass() -> float:
	if world != null and world.has_method("get_pressure_active_mass"):
		return float(world.get_pressure_active_mass())
	return 0.30

func _get_density_correction_strength() -> float:
	if world != null and world.has_method("get_density_correction_strength"):
		return float(world.get_density_correction_strength())
	return 0.0

func _get_underfill_correction_strength() -> float:
	if world != null and world.has_method("get_underfill_correction_strength"):
		return float(world.get_underfill_correction_strength())
	return 0.0
