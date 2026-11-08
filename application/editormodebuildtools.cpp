/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormodebuildtools.h"
#include "application/editoruilayer.h"
#include "model/AnimModel.h"
#include "scene/scenelayers.h"

namespace
{

// the tracks changed by the last Edits steps of the History which changed tracks, those of the Kind of action, each once
template <typename Steps, typename Action>
std::vector<TTrack *> tracks_of_edits(Steps const &History, std::size_t const Edits, Action const Kind)
{
	std::vector<TTrack *> tracks;
	std::size_t found{0};
	for (auto step{History.rbegin()}; step != History.rend() && found < Edits; ++step)
	{
		if (step->action != Kind)
			continue;
		++found;
		for (auto const &[track, step_state] : step->tracks)
			tracks.push_back(track);
		tracks.insert(tracks.end(), step->created.begin(), step->created.end());
		tracks.insert(tracks.end(), step->removed.begin(), step->removed.end());
	}
	std::erase(tracks, nullptr);
	std::ranges::sort(tracks);
	tracks.erase(std::ranges::unique(tracks).begin(), tracks.end());
	return tracks;
}

} // namespace

void editor_mode::make_build_tools()
{
	editor_services &services{*this};
	editor_build_services &build{*this};
	m_build_tools = std::make_unique<build_tools>(services, build);
	m_tools.push_back(&m_build_tools->sweep);
	m_tools.push_back(&m_build_tools->bend);
	// nothing selected in the road window at the start
	m_build_tools->road.road_select(nullptr);
}

road_tool &editor_mode::road_state()
{
	return m_roadtool;
}

std::vector<double> editor_mode::ground_heights_at(std::vector<glm::dvec3> const &Points, bool const Fresh, std::vector<char> *Found) const
{
	return ground_heights(Points, Fresh, Found);
}

sweep_tool &editor_mode::sweep_state()
{
	return m_sweep;
}

void editor_mode::push_sweep_toggled(sweep_node &Sweep)
{
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::Other;
	snap.node_name = Sweep.name();
	snap.sweeps_toggled.push_back(&Sweep);
	trim_history();
	m_history.push_back(std::move(snap));
	g_redo.clear();
}

void editor_mode::push_sweep_change(sweep_node &Sweep, sweep_node::state Before)
{
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::Other;
	snap.node_name = Sweep.name();
	snap.sweeps.emplace_back(&Sweep, std::move(Before));
	trim_history();
	m_history.push_back(std::move(snap));
	g_redo.clear();
}

void editor_mode::push_bend_made(sweep_node &Sweep, bend_source const &Source)
{
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::Bend;
	snap.sweeps_toggled.push_back(&Sweep);
	snap.layer = Source.layer;
	if (Source.model != nullptr)
	{
		auto *model{Source.model};
		model->export_as_text(snap.serialized);
		snap.node_name = model->name();
		snap.position = model->location();
		snap.rotation = model->Angles();
		snap.scale = model->Scale();
		snap.uuid = model->uuid;
		m_node = nullptr;
		m_dragging = false;
		ui()->set_node(nullptr);
		erase_model(model);
	}
	else
	{
		snap.instance = Source.instance;
		snap.node_name = Source.file;
		scene::Layers.removed(Source.instance, true);
		select_include(0);
	}
	trim_history();
	m_history.push_back(std::move(snap));
	g_redo.clear();
}

editor_ui *editor_mode::build_ui() const
{
	return ui();
}

scene::basic_node *editor_mode::current_node() const
{
	return m_node;
}

scene::instance_handle editor_mode::current_instance() const
{
	return m_instance;
}

void editor_mode::drop_node_selection()
{
	m_node = nullptr;
	m_dragging = false;
	select_include(0);
	ui()->set_node(nullptr);
}

bool editor_mode::right_button_held() const
{
	return m_input.mouse.button(GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
}

profile_state &editor_mode::vertical_profile()
{
	return m_profile;
}

std::vector<stored_profile> &editor_mode::stored_profiles()
{
	return m_profiles;
}

bool &editor_mode::profiles_loaded()
{
	return m_profiles_loaded;
}

nmt_state &editor_mode::terrain_state()
{
	return m_nmt_state;
}

terrain_streamer &editor_mode::ground_streamer()
{
	return m_streamer;
}

float const &editor_mode::terrain_spacing() const
{
	return m_terrain_spacing;
}

int const &editor_mode::track_edits() const
{
	return m_track_edits;
}

void editor_mode::history_take_profiles(std::vector<stored_profile> Library)
{
	m_history.back().profiles = true;
	m_history.back().profile_library = std::move(Library);
}

std::vector<TTrack *> editor_mode::tracks_edited(std::size_t const Edits) const
{
	return tracks_of_edits(m_history, Edits, EditorSnapshot::Action::TrackEdit);
}

void editor_mode::terrain_step_joined()
{
	auto const before{m_history.empty() ? EditorSnapshot::Action::Other : m_history.back().action};
	terrain_history_step();
	if (before != EditorSnapshot::Action::Terrain && false == m_history.empty() && m_history.back().action == EditorSnapshot::Action::Terrain)
		m_history.back().joined = true;
}

TCamera &editor_mode::view_camera()
{
	return Camera;
}

void editor_mode::release_focus()
{
	m_focus_active = false;
}

void editor_mode::forget_ground_cache()
{
	forget_ground();
}

bool editor_mode::terrain_ready()
{
	return ensure_terrain();
}

void editor_mode::open_terrain_workspace(bool const Open)
{
	terrain_workspace(Open);
}

std::size_t editor_mode::shape_tracks_on_terrain(std::vector<TTrack *> const *Only)
{
	return m_build_tools->terrain.nmt_build.nmt_shape_tracks(Only);
}

void editor_mode::join_last_history_steps()
{
	join_last_steps();
}

std::vector<TTrack *> editor_mode::selected_track_list()
{
	return selected_tracks();
}

TTrack *editor_mode::grade_line_pick_aimed(int &Index) const
{
	if (m_intent.what != track_intent::kind::grade_line_pick || m_intent.track == nullptr)
		return nullptr;
	Index = m_intent.index;
	return m_intent.track;
}

bool editor_mode::tracks_editable_for(std::vector<TTrack *> const &Tracks, std::string &Reason) const
{
	return tracks_editable(Tracks, Reason);
}

void editor_mode::render_ground_conversion_panel()
{
	render_ground_conversion();
}

editor_orthophoto &editor_mode::orthophoto()
{
	return m_orthophoto;
}

std::vector<std::unique_ptr<editor_orthophoto>> &editor_mode::imagery_layers()
{
	return m_wms;
}

editor_map &editor_mode::terrain_map()
{
	return m_nmt_map;
}

area_drawing &editor_mode::area_state()
{
	return m_area_drawing;
}

bool &editor_mode::area_paused()
{
	return m_area_paused;
}

lay_tool &editor_mode::lay_state()
{
	return m_lay;
}

extend_tool &editor_mode::extend_state()
{
	return m_extend;
}

int &editor_mode::terrain_add_count()
{
	return m_terrain_add_count;
}

bool &editor_mode::terrain_open()
{
	return m_terrain_open;
}

float &editor_mode::terrain_base_height()
{
	return m_terrain_baseheight;
}

bool &editor_mode::orthophoto_chosen()
{
	return m_orthophoto_chosen;
}

glm::dvec2 &editor_mode::orthophoto_origin_edit()
{
	return m_orthophoto_origin_edit;
}

terrain_streamer::change const *editor_mode::terrain_step_marked()
{
	terrain_history_step();
	return m_history.empty() ? nullptr : m_history.back().terrain.get();
}

void editor_mode::terrain_step_join_tracks(terrain_streamer::change const *Top)
{
	terrain_history_step();
	auto const count{m_history.size()};
	if (auto const of_tracks = [](EditorSnapshot const &Step) { return Step.action == EditorSnapshot::Action::TrackEdit || (Step.action == EditorSnapshot::Action::Terrain && Step.joined); };
	    count >= 2 && m_history.back().action == EditorSnapshot::Action::Terrain && m_history.back().terrain.get() != Top && of_tracks(m_history[count - 2]))
		m_history.back().joined = true;
}

void editor_mode::terrain_step()
{
	terrain_history_step();
}

std::vector<TTrack *> const &editor_mode::select_grown_track_set(std::vector<TTrack *> const &Tracks)
{
	m_track_set = Tracks;
	track_set_grow();
	select_tracks(m_track_set);
	return m_track_set;
}

std::vector<TTrack const *> editor_mode::tracks_made_since(std::size_t const From) const
{
	std::vector<TTrack const *> made;
	for (auto step{From}; step < m_history.size(); ++step)
		made.insert(made.end(), m_history[step].created.begin(), m_history[step].created.end());
	return made;
}

bool editor_mode::terrain_is_work_area() const
{
	return current_work_area() == work_area::terrain;
}

std::size_t editor_mode::history_size() const
{
	return m_history.size();
}

void editor_mode::join_last_step()
{
	m_history.back().joined = true;
}

void editor_mode::join_history_from(std::size_t const Step)
{
	for (auto step{Step}; step < m_history.size(); ++step)
		m_history[step].joined = true;
}

void editor_mode::raise_model_joined(TAnimModel &Model, double const Rise)
{
	push_snapshot(&Model, EditorSnapshot::Action::Move);
	m_history.back().joined = true;
	m_editor.translate(&Model, Model.location() + glm::dvec3{0.0, Rise, 0.0}, true);
}

void editor_mode::forget_ground_area(glm::dvec2 const &Min, glm::dvec2 const &Max)
{
	forget_ground_in(Min, Max);
}

void editor_mode::select_track_list(std::vector<TTrack *> const &Tracks)
{
	select_tracks(Tracks);
}

void editor_mode::select_track_node(scene::basic_node *Node)
{
	select_track(Node);
}

void editor_mode::toggle_orthographic()
{
	toggle_ortho();
}

bool editor_mode::orthophoto_settings_shown() const
{
	return orthophoto_settings_open();
}

void editor_mode::save_imagery_settings()
{
	save_orthophoto_settings();
}

glm::dvec3 editor_mode::cursor_ground_point() const
{
	return cursor_ground();
}

route_design &editor_mode::route_state()
{
	return m_route;
}

straights_state &editor_mode::straight_lines()
{
	return m_straights;
}

bool &editor_mode::route_dragging()
{
	return m_route_dragging;
}

infra_window &editor_mode::infra_state()
{
	return m_infra;
}

void editor_mode::route_recompute()
{
	route_update();
}

alignment::limits editor_mode::lay_norms() const
{
	return m_build_tools->laying.planner.laying_norms();
}

void editor_mode::undo_last_step()
{
	undo_last();
}

std::string editor_mode::refusal(std::string Text)
{
	return refused(std::move(Text));
}

void editor_mode::records_into_history()
{
	records_to_history();
}

void editor_mode::record_switch_shape(TTrack const &Track, editor_track::switch_template const &Shape) const
{
	record_switch(Track, Shape);
}

std::optional<curved_crossover> editor_mode::crossover_at(TTrack const &Switch) const
{
	return crossover_of(Switch);
}

bool editor_mode::lay_crossover_at(int const Shape, glm::dvec3 const &Press, glm::dvec3 const &Release, double const Reach, std::vector<std::string> const &Names)
{
	return lay_crossover(Shape, Press, Release, Reach, Names);
}

int editor_mode::infra_bound_total() const
{
	return m_build_tools->binder.infra_bound_count();
}

std::string editor_mode::infra_carried_status()
{
	return m_build_tools->binder.infra_carried_note();
}

void editor_mode::infra_recognise_objects()
{
	m_build_tools->binder.infra_recognize();
}

void editor_mode::infra_bind_chosen_objects()
{
	m_build_tools->binder.infra_bind_chosen();
}

glm::dvec3 editor_mode::cursor_at_level(double const Height) const
{
	return cursor_level(Height);
}

bool editor_mode::handle_drag_moved(glm::dvec3 &Position, TTrack const *Ground)
{
	return handle_drag_step(Position, Ground);
}

bool editor_mode::take_out_crossover_at(curved_crossover const &Crossover, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed, TTrack *&First, TTrack *&Second)
{
	return take_out_crossover(Crossover, States, Created, Removed, First, Second);
}

int editor_mode::crossover_template_of(switch_store::record const &Record)
{
	return crossover_template(Record);
}

void editor_mode::neighbour_straights_found()
{
	m_build_tools->routes.straight_edit.find_neighbour_straights();
}

void editor_mode::focus_on_node(scene::basic_node *Node)
{
	start_focus(Node);
}

std::vector<TTrack *> &editor_mode::tracks_dragged()
{
	return m_track_drag;
}

void editor_mode::track_drag_moved()
{
	m_track_dirty = true;
}

void editor_mode::track_drag_commit(bool const Force)
{
	commit_track_drag(Force);
}

void editor_mode::track_tab_set(editor_track_tab const Tab)
{
	m_track_tab = Tab;
}

std::vector<double> editor_mode::typed_numbers() const
{
	return typed_values();
}

void editor_mode::typed_turn_taken()
{
	typed_turn();
}

bool editor_mode::following_preview() const
{
	return m_follow_preview;
}

void editor_mode::follow_preview_set(bool const Following)
{
	m_follow_preview = Following;
}

double editor_mode::lay_design_speed(TTrack const *From, TTrack const *To) const
{
	return m_build_tools->laying.planner.design_speed(From, To);
}

double &editor_mode::join_radius_chosen()
{
	return m_join_radius;
}

int &editor_mode::join_variant_chosen()
{
	return m_join_variant;
}

alignment::join_result &editor_mode::join_result_last()
{
	return m_join_last;
}

std::optional<alignment::lay_result> &editor_mode::eased_result_last()
{
	return m_eased_last;
}

std::string &editor_mode::typed_text()
{
	return m_typed;
}

editor_track::layout &editor_mode::laying_layout()
{
	return m_track_layout;
}

double editor_mode::joint_kink() const
{
	return m_joints.kink_angle();
}

void editor_mode::join_target_aimed(editor_track::snap_target const &Target)
{
	join_target(Target);
}

void editor_mode::tutorial_open_at(char const *Lesson)
{
	m_first_steps_open = true;
	m_tutorial_goto = Lesson;
}

editor_track::point_ref &editor_mode::track_point_chosen()
{
	return m_track_point;
}

glm::dvec3 editor_mode::cursor_on_terrain(double const Height) const
{
	return cursor_terrain(Height);
}

editor_track::snap_target editor_mode::free_end_snapped(glm::dvec3 const &Near, glm::vec2 const &Screen, TTrack const *Self, int const Category, std::vector<TTrack const *> const &Exclude) const
{
	return snap_free_end(Near, Screen, Self, Category, Exclude);
}

bool editor_mode::free_end_of(TTrack &Track, int const Path, bool const Atend, extend_tool &Tool) const
{
	return free_end(Track, Path, Atend, Tool);
}

double editor_mode::bed_height_at(glm::dvec3 const &Point, glm::dvec2 const &Direction, TTrack const *Style) const
{
	return bed_height(Point, Direction, Style);
}

glm::dvec3 editor_mode::lay_bed_point(glm::dvec3 const &Point) const
{
	return lay_bed(Point);
}

glm::dvec3 editor_mode::typed_lay_point(glm::dvec3 const &Mouse) const
{
	return typed_lay(Mouse);
}

void editor_mode::ground_followed(std::vector<segment_data> &Pieces, double const Height0, double const Grade0, double const Height1, double const Grade1, TTrack const *Style) const
{
	follow_ground(Pieces, Height0, Grade0, Height1, Grade1, Style);
}

bool editor_mode::branch_link_planned(editor_track::branch_link &Link, std::string &Reason) const
{
	return plan_branch_link(Link, Reason);
}

void editor_mode::branch_link_finished(editor_track::branch_link const &Link)
{
	finish_branch_link(Link);
}

bool editor_mode::crossover_laid_from_extend()
{
	crossover_plan plan;
	if (false == plan_crossover(plan))
		return false;
	finish_crossover(plan);
	return true;
}

track_hover editor_mode::track_hovered(TTrack const *Only, float const Pixels) const
{
	return track_under_cursor(Only, Pixels);
}

std::uint64_t editor_mode::texture_image_of(int const Kind, std::string const &Name)
{
	return track_texture_image(Kind, Name);
}

bool editor_mode::texture_picked(int const Kind, std::string const &Current, bool const Mixed, std::string &Chosen)
{
	return track_texture_picker(Kind, Current, Mixed, Chosen);
}

bool editor_mode::material_dropped(std::string &Name) const
{
	return material_drop(Name);
}

std::vector<infra::object_state> &editor_mode::infra_states_before()
{
	return m_infra_buffer;
}

std::vector<infra::object_state> const *editor_mode::track_edit_infra() const
{
	if (false == m_history.empty() && m_history.back().action == EditorSnapshot::Action::TrackEdit)
		return &m_history.back().infra;
	return nullptr;
}

std::optional<std::string> editor_mode::signal_read_of(std::string const &File) const
{
	auto const *signal{signal_template_of(File)};
	if (signal == nullptr)
		return std::nullopt;
	return signal->read;
}

void editor_mode::include_directive_set(scene::instance_handle Instance, std::string const &Directive)
{
	set_include_directive(Instance, Directive);
}
