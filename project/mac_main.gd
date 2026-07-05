extends Node2D

var world
var info_label: Label
var param_labels := {}
var brush_material := 2 # 0 air, 1 rock, 2 water, 3 sand, 4 smoke, 5 toxic, 6 oil, 7 fire, 8 steam, 9 toxic gas, 10 flammable gas
var brush_radius := 6.0
var inject_horizontal_speed := 0.0
var rigid_dragging := false
var rigid_test_key_was_down := false
var rigid_paint_stroke := false

func _ready() -> void:
	world = ClassDB.instantiate("MacWorld")
	if world == null:
		push_error("MacWorld was not registered. Rebuild the GDExtension DLL and restart Godot.")
		return

	world.position = Vector2(20, 36)
	world.set_world_size(320, 180)
	world.set_display_scale(3.0)
	world.set_dt(1.0 / 30.0)
	world.set_gravity(120.0)
	world.set_viscosity(0.04)
	world.set_pressure_iterations(20)
	if world.has_method("set_pressure_active_mass"):
		world.set_pressure_active_mass(0.10)
	if world.has_method("set_density_correction_strength"):
		world.set_density_correction_strength(0.02)
	if world.has_method("set_underfill_correction_strength"):
		world.set_underfill_correction_strength(0.02)
	if world.has_method("set_rigid_liquid_impulse_strength"):
		world.set_rigid_liquid_impulse_strength(0.45)
	world.set_simulation_speed(1.0)
	if world.has_method("generate_rigid_collision_test"):
		world.generate_rigid_collision_test()
	else:
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
	panel.position = Vector2(12, max(210.0, get_viewport_rect().size.y - 410.0))
	var panel_style := StyleBoxFlat.new()
	panel_style.bg_color = Color(0.02, 0.025, 0.035, 0.20)
	panel_style.border_color = Color(0.45, 0.70, 1.0, 0.18)
	panel_style.set_border_width_all(1)
	panel_style.set_corner_radius_all(6)
	panel.add_theme_stylebox_override("panel", panel_style)
	layer.add_child(panel)

	var box := VBoxContainer.new()
	box.custom_minimum_size = Vector2(280, 0)
	panel.add_child(box)

	var title := Label.new()
	title.text = "MAC grid + PCG pressure projection"
	title.add_theme_color_override("font_color", Color(0.84, 0.92, 1.0, 0.82))
	title.add_theme_font_size_override("font_size", 11)
	box.add_child(title)

	_add_slider(box, "dt", 1.0 / 120.0, 1.0 / 15.0, 0.001, world.get_dt())
	_add_slider(box, "gravity", 0.0, 300.0, 1.0, world.get_gravity())
	_add_slider(box, "viscosity", 0.0, 0.25, 0.005, world.get_viscosity())
	_add_slider(box, "pressure_iterations", 1.0, 250.0, 1.0, world.get_pressure_iterations())
	if world.has_method("get_pressure_active_mass"):
		_add_slider(box, "pressure_active_mass", 0.01, 1.0, 0.01, world.get_pressure_active_mass())
	if world.has_method("get_density_correction_strength"):
		_add_slider(box, "density_correction", 0.0, 2.0, 0.01, world.get_density_correction_strength())
	if world.has_method("get_underfill_correction_strength"):
		_add_slider(box, "underfill_correction", 0.0, 2.0, 0.01, world.get_underfill_correction_strength())
	if world.has_method("get_rigid_liquid_impulse_strength"):
		_add_slider(box, "rigid_liquid_impulse", 0.0, 3.0, 0.01, world.get_rigid_liquid_impulse_strength())
	_add_slider(box, "inject_horizontal_speed", -60.0, 60.0, 1.0, inject_horizontal_speed)
	_update_param_labels()

func _add_slider(parent: VBoxContainer, key: String, min_value: float, max_value: float, step_value: float, value: float) -> void:
	var label := Label.new()
	label.add_theme_color_override("font_color", Color(0.86, 0.92, 1.0, 0.72))
	label.add_theme_font_size_override("font_size", 10)
	parent.add_child(label)
	param_labels[key] = label

	var slider := HSlider.new()
	slider.min_value = min_value
	slider.max_value = max_value
	slider.step = step_value
	slider.value = value
	slider.custom_minimum_size = Vector2(260, 0)
	slider.modulate = Color(0.78, 0.88, 1.0, 0.62)
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
		"rigid_liquid_impulse":
			if world.has_method("set_rigid_liquid_impulse_strength"):
				world.set_rigid_liquid_impulse_strength(value)
		"inject_horizontal_speed":
			inject_horizontal_speed = value
	_update_param_labels()

func _update_param_labels() -> void:
	if world == null:
		return
	if param_labels.has("dt"):
		param_labels["dt"].text = "fluid dt / target interval: %.4f s (%.1f Hz)" % [world.get_dt(), 1.0 / max(world.get_dt(), 0.0001)]
	if param_labels.has("gravity"):
		param_labels["gravity"].text = "gravity: %.1f cells/s^2" % [world.get_gravity()]
	if param_labels.has("viscosity"):
		param_labels["viscosity"].text = "viscosity nu: %.3f" % [world.get_viscosity()]
	if param_labels.has("pressure_iterations"):
		param_labels["pressure_iterations"].text = "PCG max iterations: %d" % [world.get_pressure_iterations()]
	if param_labels.has("pressure_active_mass"):
		param_labels["pressure_active_mass"].text = "pressure active volume threshold: %.2f" % [_get_pressure_active_mass()]
	if param_labels.has("density_correction"):
		param_labels["density_correction"].text = "density correction strength: %.2f  target div = k * max(volume-1,0)/dt" % [_get_density_correction_strength()]
	if param_labels.has("underfill_correction"):
		param_labels["underfill_correction"].text = "internal underfill strength: %.2f  only internal volume<1 gets negative div" % [_get_underfill_correction_strength()]
	if param_labels.has("rigid_liquid_impulse"):
		param_labels["rigid_liquid_impulse"].text = "rigid-liquid splash impulse: %.2f  displaced liquid gets outward momentum" % [_get_rigid_liquid_impulse_strength()]
	if param_labels.has("inject_horizontal_speed"):
		param_labels["inject_horizontal_speed"].text = "LMB water initial horizontal speed: %.1f cells/s" % [inject_horizontal_speed]

func _process(_delta: float) -> void:
	if world == null:
		return
	_handle_keyboard()
	_handle_rigid_drag()
	_handle_mouse_paint()
	_update_param_labels()
	_update_info()

func _handle_keyboard() -> void:
	if Input.is_action_just_pressed("ui_accept"):
		world.set_paused(not world.is_paused())
	if Input.is_action_just_pressed("ui_cancel"):
		world.generate_basin()
	var t_down := Input.is_key_pressed(KEY_T)
	if t_down and not rigid_test_key_was_down and world.has_method("generate_rigid_collision_test"):
		world.generate_rigid_collision_test()
	rigid_test_key_was_down = t_down
	if Input.is_key_pressed(KEY_F11):
		_enter_fullscreen()
	if Input.is_key_pressed(KEY_1):
		brush_material = 2 # water
	if Input.is_key_pressed(KEY_2):
		brush_material = 1 # rock
	if Input.is_key_pressed(KEY_3):
		brush_material = 0 # air/erase
	if Input.is_key_pressed(KEY_4):
		brush_material = 3 # sand
	if Input.is_key_pressed(KEY_5):
		brush_material = 4 # smoke
	if Input.is_key_pressed(KEY_6):
		brush_material = 5 # toxic
	if Input.is_key_pressed(KEY_7):
		brush_material = 6 # oil
	if Input.is_key_pressed(KEY_8):
		brush_material = 7 # fire
	if Input.is_key_pressed(KEY_9):
		brush_material = 8 # steam
	if Input.is_key_pressed(KEY_0):
		brush_material = 9 # toxic gas
	if Input.is_key_pressed(KEY_F):
		brush_material = 10 # flammable gas
	if Input.is_key_pressed(KEY_EQUAL):
		brush_radius = min(30.0, brush_radius + 0.25)
	if Input.is_key_pressed(KEY_MINUS):
		brush_radius = max(1.0, brush_radius - 0.25)
	if Input.is_key_pressed(KEY_BRACKETLEFT):
		world.set_pressure_iterations(max(1, world.get_pressure_iterations() - 1))
	if Input.is_key_pressed(KEY_BRACKETRIGHT):
		world.set_pressure_iterations(world.get_pressure_iterations() + 1)


func _mouse_local_position() -> Vector2:
	return (get_viewport().get_mouse_position() - world.global_position) / float(world.get_display_scale())

func _handle_rigid_drag() -> void:
	if world == null or not world.has_method("start_rigid_drag"):
		return
	var local := _mouse_local_position()
	if Input.is_mouse_button_pressed(MOUSE_BUTTON_MIDDLE):
		if not rigid_dragging:
			rigid_dragging = bool(world.start_rigid_drag(local.x, local.y))
		elif world.has_method("update_rigid_drag"):
			var rotate := Input.is_key_pressed(KEY_SHIFT)
			world.update_rigid_drag(local.x, local.y, rotate)
	elif rigid_dragging:
		rigid_dragging = false
		if world.has_method("end_rigid_drag"):
			world.end_rigid_drag()

func _handle_mouse_paint() -> void:
	var lmb_down := Input.is_mouse_button_pressed(MOUSE_BUTTON_LEFT)
	if rigid_paint_stroke and (not lmb_down or brush_material != 1):
		rigid_paint_stroke = false
		if world.has_method("end_rigid_paint_stroke"):
			world.end_rigid_paint_stroke()

	var use_tool := false
	var material := brush_material
	if lmb_down:
		use_tool = true
		material = brush_material
	elif Input.is_mouse_button_pressed(MOUSE_BUTTON_RIGHT):
		use_tool = true
		material = 0
	if not use_tool:
		return

	if rigid_dragging or Input.is_mouse_button_pressed(MOUSE_BUTTON_MIDDLE):
		return
	var local: Vector2 = _mouse_local_position()
	if material == 1 and lmb_down and world.has_method("begin_rigid_paint_stroke"):
		if not rigid_paint_stroke:
			rigid_paint_stroke = true
			world.begin_rigid_paint_stroke()
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
	elif brush_material == 3:
		mat_name = "Sand"
	elif brush_material == 4:
		mat_name = "Smoke"
	elif brush_material == 5:
		mat_name = "Toxic"
	elif brush_material == 6:
		mat_name = "Oil"
	elif brush_material == 7:
		mat_name = "Fire"
	elif brush_material == 8:
		mat_name = "Steam"
	elif brush_material == 9:
		mat_name = "Toxic gas"
	elif brush_material == 10:
		mat_name = "Flammable gas"

	var text := "MacWorld: MAC grid + explicit advection/viscosity + PCG pressure projection\n"
	text += "FPS: %d | target fluid: %.1f steps/s | budget: 4.00 ms/frame | total sim steps: %d | last fluid step: %.2f ms\n" % [int(Engine.get_frames_per_second()), (1.0 / max(world.get_dt(), 0.0001)) * world.get_simulation_speed(), world.get_step_count(), world.get_last_step_ms()]
	if world.has_method("get_last_sim_ms"):
		var frame_steps: int = 1
		var frame_sim_ms: float = float(world.get_last_sim_ms())
		if world.has_method("get_last_frame_sim_steps"):
			frame_steps = int(world.get_last_frame_sim_steps())
			frame_sim_ms = float(world.get_last_frame_sim_ms())
		text += "timing: sim %.2f ms | frame sim %d steps %.2f ms | fill RGBA %.2f ms | texture %.2f ms | measured total %.2f ms\n" % [world.get_last_sim_ms(), frame_steps, frame_sim_ms, world.get_last_fill_ms(), world.get_last_texture_ms(), frame_sim_ms + world.get_last_fill_ms() + world.get_last_texture_ms()]
	if world.has_method("get_last_predict_ms"):
		text += "fluid phases: predict %.2f | build %.2f | pcg %.2f | project %.2f | advect %.2f | clamp %.2f | total %.2f ms\n" % [world.get_last_predict_ms(), world.get_last_build_ms(), world.get_last_pcg_ms(), world.get_last_project_ms(), world.get_last_advect_ms(), world.get_last_clamp_ms(), world.get_last_step_ms()]
	if world.has_method("get_active_region_pad"):
		text += "fluid active rect: (%d,%d)-(%d,%d) pad %d | max speed %.2f cells/tick\n" % [world.get_active_region_min_x(), world.get_active_region_min_y(), world.get_active_region_max_x(), world.get_active_region_max_y(), world.get_active_region_pad(), world.get_active_region_max_speed()]
	text += "dt: %.3f | gravity: %.2f | viscosity: %.3f | active: %.2f | over: %.2f | under: %.2f | splash: %.2f | PCG: %d/%d residual %.2e\n" % [world.get_dt(), world.get_gravity(), world.get_viscosity(), _get_pressure_active_mass(), _get_density_correction_strength(), _get_underfill_correction_strength(), _get_rigid_liquid_impulse_strength(), world.get_last_pcg_iterations(), world.get_pressure_iterations(), world.get_last_pcg_residual()]
	text += "liquid volume: %.1f | avg volume/liquid cell: %.3f | liquid cells: %d\n" % [world.get_total_water_mass(), world.get_average_water_mass(), world.get_water_cell_count()]
	if world.has_method("get_rigid_body_count"):
		var awake_count: int = int(world.get_rigid_awake_count()) if world.has_method("get_rigid_awake_count") else -1
		var sleeping_count: int = int(world.get_rigid_sleeping_count()) if world.has_method("get_rigid_sleeping_count") else -1
		text += "rigid bodies: %d | awake: %d | sleeping: %d | MMB drag body, Shift+MMB rotate picked body\n" % [world.get_rigid_body_count(), awake_count, sleeping_count]
	text += "paused: %s | Controls: Space pause, Esc basin, T rigid collision test, 1 water, 2 rock, 3 air, 4 sand, 5 smoke, 6 toxic, 7 oil, 8 fire, 9 steam, 0 toxic gas, F flammable gas, LMB paint/inject, RMB erase, MMB drag rigid, Shift+MMB rotate\n" % [str(world.is_paused())]
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

func _get_rigid_liquid_impulse_strength() -> float:
	if world != null and world.has_method("get_rigid_liquid_impulse_strength"):
		return float(world.get_rigid_liquid_impulse_strength())
	return 0.45
