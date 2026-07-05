extends Node2D

var worlds := []
var labels := []

func _ready() -> void:
	_create_threaded_world(Vector2(18, 44), 0)
	_create_threaded_world(Vector2(18 + 260 * 2.2, 44), 1)
	var info := Label.new()
	info.position = Vector2(12, 8)
	info.text = "Threaded MacWorld test: left and right worlds each run simulation on its own C++ std::thread; main thread only uploads textures."
	info.add_theme_color_override("font_color", Color(0.86, 0.94, 1.0))
	add_child(info)

func _exit_tree() -> void:
	for w in worlds:
		if w != null and w.has_method("set_threaded_simulation_enabled"):
			w.set_threaded_simulation_enabled(false)

func _create_threaded_world(pos: Vector2, variant: int) -> void:
	var w = ClassDB.instantiate("MacWorld")
	if w == null:
		push_error("MacWorld was not registered. Rebuild the GDExtension DLL and restart Godot.")
		return
	w.position = pos
	w.set_world_size(260, 150)
	w.set_display_scale(2.2)
	w.set_dt(1.0 / 30.0)
	w.set_gravity(120.0)
	w.set_viscosity(0.04)
	w.set_pressure_iterations(20)
	if w.has_method("set_pressure_active_mass"):
		w.set_pressure_active_mass(0.10)
	if w.has_method("set_density_correction_strength"):
		w.set_density_correction_strength(0.02)
	if w.has_method("set_underfill_correction_strength"):
		w.set_underfill_correction_strength(0.02)
	if w.has_method("set_rigid_liquid_impulse_strength"):
		w.set_rigid_liquid_impulse_strength(0.45)

	# Two independent places / simulations.  They deliberately start with
	# different content so it is visible that they are not sharing state.
	if variant == 0:
		w.generate_rigid_collision_test()
		for i in range(10):
			w.inject_water(70 + i * 5, 34, 4, 1.0, 16.0, 0.0)
	else:
		w.generate_basin()
		for i in range(18):
			w.paint_circle(135 + sin(float(i)) * 38.0, 30 + i * 2, 5.0, 2) # water
		for i in range(8):
			w.paint_circle(120 + i * 7, 48, 4.0, 6) # oil

	add_child(w)
	if w.has_method("set_threaded_simulation_enabled"):
		w.set_threaded_simulation_enabled(true)
	worlds.append(w)

	var l := Label.new()
	l.position = pos + Vector2(0, 150 * 2.2 + 8)
	l.add_theme_color_override("font_color", Color(0.82, 0.92, 1.0))
	add_child(l)
	labels.append(l)

func _process(_delta: float) -> void:
	for i in range(worlds.size()):
		var w = worlds[i]
		var l = labels[i]
		if w == null or l == null:
			continue
		var threaded := false
		if w.has_method("is_threaded_simulation_enabled"):
			threaded = w.is_threaded_simulation_enabled()
		var steps := int(w.get_step_count()) if w.has_method("get_step_count") else 0
		var sim_ms := float(w.get_last_sim_ms()) if w.has_method("get_last_sim_ms") else 0.0
		var fill_ms := float(w.get_last_fill_ms()) if w.has_method("get_last_fill_ms") else 0.0
		var tex_ms := float(w.get_last_texture_ms()) if w.has_method("get_last_texture_ms") else 0.0
		l.text = "World %d | threaded: %s | steps: %d | sim %.2f ms | fill %.2f | texture %.2f" % [i + 1, str(threaded), steps, sim_ms, fill_ms, tex_ms]
