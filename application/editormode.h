/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/applicationmode.h"
#include "application/editoruipanels.h"
#include "input/editormouseinput.h"
#include "input/editorkeyboardinput.h"
#include "vehicle/Camera.h"
#include "scene/sceneeditor.h"
#include "scene/scenenode.h"
#include "editor/editorTerrain.hpp"
#include "editor/editorTerrainStreamer.hpp"
#include "editor/editorOrthophoto.hpp"
#include "editor/editorTrack.hpp"
#include "editor/editorAlignment.hpp"
#include "editor/editorRoad.hpp"

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class TAnimModel;
class TTrack;

class editor_mode : public application_mode
{

  public:
	// constructors
	editor_mode();
	// methods
	// initializes internal data structures of the mode. returns: true on success, false otherwise
	bool init() override;
	// mode-specific update of simulation data. returns: false on error, true otherwise
	bool update() override;
	// maintenance method, called when the mode is activated
	void enter() override;
	// maintenance method, called when the mode is deactivated
	void exit() override;
	// input handlers
	void on_key(int Key, int Scancode, int Action, int Mods) override;
	void on_cursor_pos(double Horizontal, double Vertical) override;
	void on_mouse_button(int Button, int Action, int Mods) override;
	void on_scroll(double const Xoffset, double const Yoffset) override;
	void on_window_resize(int w, int h) override
	{
		;
	}
	void on_event_poll() override;
	bool is_command_processor() const override;
	void undo_last();
	static bool focus_active();
	static void  set_focus_active(bool isActive);
	static TCamera& get_camera() { return Camera; }
	static glm::mat4 projection_matrix(float const Aspect);
	void toggle_ortho();
	static bool change_history() { return m_change_history; }
	static void set_change_history(bool enabled) { m_change_history = enabled; }
	static bool settings_open() { return m_settings_open; }
	static void set_settings_open(bool enabled) { m_settings_open = enabled; }
  private:
	// types
	struct editormode_input
	{

		editormouse_input mouse;
		editorkeyboard_input keyboard;

		bool init();
		void poll();
	};

	struct state_backup
	{

		TCamera camera;
		bool freefly;
		bool picking;
	};

	struct EditorSnapshot
	{
		enum class Action { Move, Rotate, Scale, Add, Delete, TrackEdit, Other, RoadEdit };

		Action action{Action::Other};
		std::string node_name;          // node identifier (basic_node::name())
		// direct pointer to node when available; used for in-memory undo/redo lookup
		scene::basic_node *node_ptr{nullptr};
		std::string serialized;         // full text for recreate (used for Add/Delete)
		glm::dvec3 position{0.0, 0.0, 0.0};
		glm::vec3 rotation{0.0f, 0.0f, 0.0f};
		glm::vec3 scale{1.0f, 1.0f, 1.0f};
		UID uuid; // node UUID for reference, used as fallback lookup for deleted/recreated nodes
		scene::layer_handle layer{null_handle}; // scenery layer of the node, so a recreated node returns to it
		scene::instance_handle instance{0}; // include of a scenery template which was placed or removed, instead of a node
		std::vector<std::pair<TTrack *, editor_track::state>> tracks;
		std::vector<TTrack *> created;
		std::vector<TTrack *> removed;
		editor_road::record roads; // what a change of the roads comes to

	};
	void push_snapshot(scene::basic_node *node, EditorSnapshot::Action Action = EditorSnapshot::Action::Move, std::string const &Serialized = std::string());

	std::vector<EditorSnapshot> m_history; // history of changes to nodes, used for undo functionality
	std::vector<EditorSnapshot> g_redo;
	// methods
	void update_camera(double const Deltatime);

	editor_ui *ui() const;
	void redo_last();
	void handle_brush_mouse_hold(int Action, int Button);
	void apply_rotation_for_new_node(scene::basic_node *node, int rotation_mode, float fixed_rotation_value);
	// members
	state_backup m_statebackup; // helper, cached variables to be restored on mode exit
	editormode_input m_input;
	static TCamera Camera;

	// focus (smooth camera fly-to) state
	static bool m_focus_active;
	glm::dvec3 m_focus_start_pos{0.0,0.0,0.0};
	glm::dvec3 m_focus_target_pos{0.0,0.0,0.0};
	glm::vec3 m_focus_start_angle{0.0f};   // camera pitch/yaw/roll at focus start
	glm::vec3 m_focus_target_angle{0.0f};  // camera pitch/yaw/roll facing the focused object
	double m_focus_time{0.0};
	double m_focus_duration{0.6};

	double fTime50Hz{0.0}; // bufor czasu dla komunikacji z PoKeys
	scene::basic_editor m_editor;
	scene::basic_node *m_node{nullptr}; // currently selected scene node
	scene::instance_handle m_instance{0}; // currently selected include of a scenery template; these are handled as a whole
	include_selection m_include; // parameters of the selected include, changed with the gizmo and in the node properties
	bool m_include_gesture{false}; // a change of the selected include is under way, and has its undo snapshot already
	std::vector<std::pair<TAnimModel *, int>> m_retired; // models replaced after changes of includes, with the frames left until they're destroyed
	bool m_takesnapshot{true}; // helper, hints whether snapshot of selected node(s) should be taken before modification
	bool m_dragging = false;
	glm::dvec3 oldPos; // world position of the last brush placement
	bool m_brush_has_last{false}; // false at the start of a brush stroke, so the first placement ignores spacing
	bool mouseHold{false};
	float kMaxPlacementDistance = 200.0f;
	static bool m_change_history;
	static bool m_settings_open;

	double m_ortho_pitch{0.0};
	// camera fly-mode (right mouse button held); used to flush motion when it's released
	command_relay m_camera_relay;
	bool m_camera_flying{false};

	// UI/history settings
	int m_max_history_size{200};
	int m_selected_history_idx{-1};
	glm::dvec3 clamp_mouse_offset_to_max(const glm::dvec3 &offset);
	// the ground under a point the cursor landed on, for things which are to stand on the ground
	glm::dvec3 placement_on_ground(glm::dvec3 Location);
	// puts every instance of the model the selected node shows on the ground under it
	void drop_model_instances();
	void render_object_menu();

	// focus camera smoothly on specified node
	void start_focus(scene::basic_node *node, double duration = 0.6);

	// drops the node straight down onto the nearest surface below (terrain or another object)
	void snap_to_ground(scene::basic_node *node);

	// editable terrain patches created in the editor
	void render_terrain_ui();
	// creates a large terrain as a grid of adjacent chunks (each its own editable patch)
	void create_chunked_terrain();
	// manual grid-aligned chunks: add/remove single chunks for fine control
	float chunk_grid_size() const { return m_terrain_cells * m_terrain_cellsize; }
	void add_grid_chunk(int Cx, int Cz);
	void remove_grid_chunk(int Cx, int Cz);
	// handles a click in chunk-edit mode (add a neighbour, or Shift = delete the clicked chunk)
	void handle_chunk_edit_click(bool DeleteMode);
	// commits authored terrain to disk, enables streaming, and exports the scenery
	void save_scene_with_terrain();
	// commits authored terrain to disk and enables streaming
	void commit_terrain();
	// writes the changes to the files of scenery opened for editing (Ctrl+S)
	void save();
	// places include of specified scenery template at the cursor, in the active layer
	void place_include(std::string const &File, int RotationMode, float FixedRotation);
	// undo and redo of placing or removing an include: brings removed include back, or removes it
	void toggle_include(scene::instance_handle Instance);
	// makes specified include the selected one, 0 clears the selection
	void select_include(scene::instance_handle Instance);
	// writes parameter values of the selected include to its directive, and shows the outcome in the scene
	void apply_include();
	// replaces directive of specified include, and the models shown for it
	void set_include_directive(scene::instance_handle Instance, std::string const &Directive);
	// undo and redo of an include snapshot
	void restore_include(EditorSnapshot &Snapshot);
	// gizmo for the selected include, limited to what the parameters of its template can express
	void render_include_gizmo(glm::mat4 const &View, glm::mat4 const &Projection, glm::dvec3 const &Camerapos);
	// exports the scenery in legacy (text) format, with layers hidden in the editor left out of the picture
	void export_scenery();
	// raises/lowers terrain under the cursor while the left mouse button is held in sculpt mode
	void handle_terrain_sculpt(double Deltatime);
	// returns the terrain patch (if any) whose footprint covers the given world point
	editor_terrain *terrain_at(double X, double Z);
	// gathers every active terrain patch: manually-created ones plus streamed chunks
	std::vector<editor_terrain *> active_terrains();
	// samples the selected model instance's geometry into a new editable terrain patch, then removes it
	void capture_terrain();
	std::vector<std::unique_ptr<editor_terrain>> m_terrains;
	// grid-aligned manual chunks, keyed by (cx,cz) on the global chunk grid
	std::map<std::pair<int, int>, std::unique_ptr<editor_terrain>> m_grid_chunks;
	bool m_terrain_sculpt{false};     // when true, LMB sculpts terrain instead of picking
	bool m_chunk_edit{false};         // when true, LMB adds/removes whole chunks
	int m_terrain_cells{32};          // grid resolution (quads per side)
	int m_terrain_chunks{4};          // chunks per side for a chunked terrain
	float m_terrain_cellsize{2.0f};   // metres per quad
	float m_terrain_baseheight{0.0f}; // flat starting height
	float m_terrain_brush_radius{12.0f};
	float m_terrain_brush_strength{4.0f}; // metres per second while held (one-shot for the buttons)
	float m_terrain_simplify_error{0.5f}; // flatness tolerance (m) for mesh simplification
	bool m_terrain_auto_optimize{false};  // auto-simplify edited chunks after sculpting settles
	double m_terrain_idle{0.0};           // seconds since the last sculpt edit (debounce timer)
	char m_terrain_texture[128]{""};  // optional ground texture name

	// streaming terrain that follows the camera (open-world); the editor shares the single
	// simulation-level instance so authored terrain also renders in the driver / other modes
	terrain_streamer &m_streamer{EditorTerrain};
	int m_stream_radius{2};
	bool m_stream_persist{true}; // save edited chunks to disk and load them back

	// geoportal orthophoto layer drawn under the other viewport overlays
	void render_orthophoto_ui();
	void draw_orthophoto();
	// picks up the stored settings and the origin of the current scenery
	void load_orthophoto_settings();
	void save_orthophoto_settings();
	editor_orthophoto m_orthophoto;
	std::string m_orthophoto_scenery;            // scenery whose origin is loaded
	glm::dvec2 m_orthophoto_origin_edit{0.0};    // origin being typed in (northing, easting); applied when editing ends

	// hierarchy management
	void add_to_hierarchy(scene::basic_node *node);
	void remove_from_hierarchy(scene::basic_node *node);
	scene::basic_node* find_in_hierarchy(const std::string &uuid_str);
	scene::basic_node* find_node_by_any(scene::basic_node *node_ptr, const std::string &uuid_str, const std::string &name);

	// clear history/redo pointers that reference the given node (prevent dangling pointers)
	void nullify_history_pointers(scene::basic_node *node);
	void render_change_history();
	void render_settings();

	// area fill: scatters models from a package over a polygon outlined in the viewport
	void render_area_fill();
	void draw_area_fill_outline() const;
	void add_area_fill_point();
	void run_area_fill();
	void undo_last_area_fill();
	std::vector<glm::dvec3> m_fill_points; // outline vertices (world space) in click order
	std::vector<TAnimModel *> m_fill_last; // instances created by the most recent fill, for "Undo last fill"
	std::vector<std::string> m_fill_custom; // user-assembled package of node templates
	int m_fill_custom_idx{-1};
	model_set_ref m_fill_source; // manual = m_fill_custom, otherwise a user set or node bank group
	float m_fill_density{200.0f}; // objects per hectare
	float m_fill_min_spacing{2.0f}; // minimal distance between placed objects (m)
	bool m_fill_random_rotation{true}; // random yaw; otherwise the Functions panel rotation settings apply
	float m_fill_scale_min{1.0f};
	float m_fill_scale_max{1.0f};
	bool m_fill_models_as_ground{true}; // large model instances (terrain tiles) count as ground
	std::string m_fill_status; // result of the last fill, shown in the panel

	// ImGuizmo-based transform gizmo for the selected node
	enum class gizmo_operation { translate, rotate, scale };
	void render_gizmo();
	// gizmo settings, drawn in the toolset window
	void render_gizmo_options();
	bool m_gizmo_enabled{true};                                  // master switch for the in-viewport gizmo
	bool m_gizmo_using{false};                                   // tracks an ongoing drag, so a single undo snapshot is taken per drag
	bool m_gizmo_local{false};                                   // manipulate in the object's local space instead of world space
	gizmo_operation m_gizmo_op{gizmo_operation::translate};      // current transform mode (translate/rotate/scale)
	float m_gizmo_snap{1.0f};                                    // translation snap step (metres) applied while Ctrl is held

	TTrack *selected_track() const;
	void render_track_ui();
	void render_path_ui();
	void draw_track_overlay() const;
	bool pick_track_handle();
	void select_track(scene::basic_node *Node);
	void render_track_gizmo();
	void commit_track_drag(bool const Force);
	void push_track_snapshot(std::vector<std::pair<TTrack *, editor_track::state>> States, std::vector<TTrack *> Created = {});
	void restore_track_snapshot(EditorSnapshot const &Snapshot, std::vector<EditorSnapshot> &Opposite, bool const Undo);
	editor_track::point_ref m_track_point;
	std::vector<TTrack *> m_track_drag;
	std::vector<std::pair<TTrack *, editor_track::point_ref>> m_track_drag_points;
	editor_track::snap_target m_track_snap;
	glm::dvec3 m_track_pivot{0.0};
	glm::mat4 m_track_gizmo{1.0f};
	bool m_track_gizmo_using{false};
	bool m_track_dirty{false};
	std::chrono::steady_clock::time_point m_track_last_commit;
	editor_track::state m_track_field_before;
	std::array<std::array<char, 256>, 3> m_track_materials{};
	struct route_design
	{
		TTrack *from{nullptr};
		TTrack *to{nullptr};
		editor_track::chain chain;
		std::string error;
		std::string status;
		alignment::design design;
		alignment::result result;
		int vertex{-1};
		int grip{-1};
	};
	void render_route_ui();
	void route_reset();
	bool route_from_curve(TTrack &Track);
	void route_update() { m_route.result = alignment::compute(m_route.design); }
	void route_apply();
	void route_recommend(alignment::vertex &Vertex) const;
	void draw_route_overlay() const;
	bool pick_route_vertex();
	void render_route_gizmo();
	glm::dvec3 route_vertex_position(int const Vertex) const;
	bool route_active() const;
	route_design m_route;

	struct straights_state
	{
		editor_track::straight_tolerance tolerance;
		double minimum_length{50.0};
		std::vector<editor_track::straight> found;
		int listed{-1};
		TTrack *current_for{nullptr};
		editor_track::straight current;
		std::vector<editor_track::straight> neighbours;
		std::vector<double> spacings{3.5, 4.0, 4.5, 4.75, 5.0, 5.5, 6.0};
		char filter[32]{};
		int handle{-1};
		bool dragging{false};
		editor_track::straight drag_line;
		std::vector<editor_track::straight> set;
		std::vector<editor_track::straight> drag_lines;
		bool fitting{false};
		editor_track::curve fit_before;
		editor_track::curve fit_after;
		std::vector<std::pair<TTrack *, editor_track::state>> drag_states;
		glm::dvec3 preview_start{0.0};
		glm::dvec3 preview_end{0.0};
		glm::mat4 gizmo{1.0f};
		glm::dvec3 edit_start{0.0};
		glm::dvec3 edit_end{0.0};
		double edit_length{0.0};
		double edit_azimuth{0.0};
		double distance{4.0};
		double curve_radius{0.0};
		bool auto_transitions{true};
		double transition{0.0};
		int tool{0};
		bool tool_mouse{false};
		std::vector<glm::dvec3> detour;
		editor_track::straight detour_line;
		glm::dvec3 detour_mouse{0.0};
		bool tool_placed{false};
		bool tool_dragging{false};
		double tool_at{0.0};
		glm::dvec3 tool_point{0.0};
		glm::dvec3 tool_handle{0.0};
		editor_track::straight tool_line;
		glm::mat4 tool_gizmo{1.0f};
		std::string status;
	};
	void render_straights_ui();
	void draw_straights_overlay() const;
	editor_track::straight const &current_straight();
	bool straights_active();
	bool pick_straight_handle();
	void render_straight_gizmo();
	void straight_apply(editor_track::straight const &Line, glm::dvec3 const &Start, glm::dvec3 const &End);
	void straights_apply(std::vector<editor_track::straight> const &Lines, std::vector<std::pair<glm::dvec3, glm::dvec3>> const &Ends);
	bool in_straight_set(editor_track::straight const &Line) const;
	bool place_straight_tool();
	void render_straight_tool_gizmo();
	void apply_straight_tool();
	double straight_tool_radius(editor_track::straight const &Line) const;
	void straight_reshape(editor_track::straight const &Line, double const From, double const To, std::function<glm::dvec3(glm::dvec3 const &)> const &Tail, double const Radius);
	void straight_refresh();
	bool start_curve_fit(editor_track::straight const &Line);
	void update_curve_fit(glm::dvec3 const &Start, glm::dvec3 const &End);
	void add_detour_point();
	void apply_detour();
	std::vector<glm::dvec3> detour_outline() const;
	void find_neighbour_straights();
	glm::dvec3 snap_straight_offset(editor_track::straight const &Line, glm::dvec3 const &Offset) const;
	glm::dvec3 snap_straight_direction(glm::dvec3 const &Pivot, glm::dvec3 const &Moved) const;
	straights_state m_straights;

	struct curve_sample
	{
		double station{0.0};
		glm::dvec3 position{0.0};
		glm::dvec2 tangent{0.0, 1.0};
		float roll{0.f};
		float radius{0.f};
	};
	struct switch_tool
	{
		bool curved{false};
		std::vector<curve_sample> frame;
		std::vector<TTrack *> frame_tracks;
		std::vector<editor_track::switch_template> templates;
		bool collected{false};
		int armed{-1};
		bool placing{false};
		double along{0.0};
		glm::dvec3 point{0.0};
		glm::dvec3 mouse{0.0};
		editor_track::straight line;
	};
	switch_tool m_switch;
	int m_turnout_template{0};
	struct extend_tool
	{
		bool active{false};
		TTrack *track{nullptr};
		glm::dvec3 point{0.0};
		glm::dvec2 direction{0.0, 1.0};
		double grade{0.0};
		glm::dvec3 mouse{0.0};
		int path{0};
		bool atend{true};
	};
	extend_tool m_extend;
	void render_switch_ui();
	bool start_switch_placement();
	void finish_switch_placement();
	std::vector<segment_data> switch_preview() const;
	void insert_switch(editor_track::straight const &Line, double const Along, int const Direction, int const Side);
	static std::vector<curve_sample> sample_chain(editor_track::chain const &Chain);
	static curve_sample sample_at(std::vector<curve_sample> const &Frame, double const Station);
	static double nearest_station(std::vector<curve_sample> const &Frame, glm::dvec3 const &Point);
	static std::vector<segment_data> curved_switch_paths(editor_track::switch_template const &Shape, std::vector<curve_sample> const &Frame, double const Station, int const Direction, int const Side);
	void insert_curved_switch(int const Direction, int const Side);
	bool place_switch_on_straight(editor_track::straight const &Line, editor_track::switch_template const &Shape, TTrack const *Style, double const Along, int const Direction, int const Side, bool const Snap, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed);
	struct crossover_plan
	{
		editor_track::straight target;
		editor_track::switch_template shape;
		double along{0.0};
		int direction{1};
		int side{1};
		segment_data insert;
		std::vector<segment_data> paths;
	};
	bool plan_crossover(crossover_plan &Plan) const;
	bool cut_straight(editor_track::straight const &Line, double const From, double const To, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed, TTrack **Style);
	bool insert_double_slip(editor_track::straight const &Line, glm::dvec3 const &Point, editor_track::switch_template const &Shape);
	bool replace_double_slip(TTrack &Part, editor_track::switch_template const &Shape);
	std::vector<TTrack *> build_double_slip(glm::dvec3 const &PointA, glm::dvec3 const &PointB, glm::dvec3 const &PointC, glm::dvec3 const &PointD, glm::dvec2 const &Crossing, double const Height, glm::dvec2 const &First, glm::dvec2 const &Second, double const Angle, double const Radius, TTrack &Style);
	bool start_extend();
	void finish_extend();
	std::vector<segment_data> extend_pieces() const;
	bool extend_snap(editor_track::snap_target &Target) const;
	void update_build_tools();
	void draw_build_overlay() const;
	enum class track_tab { straights, route, path, turnout };
	bool m_track_window_open{false};
	void render_track_window();
	void render_turnout_ui();
	void render_path_parameters(TTrack &Track);
	void render_straight_ui();
	track_tab m_track_tab{track_tab::straights};
	int m_track_tab_request{-1};
	void delete_selected_track();
	void start_straight_gesture(int const Tool);
	void finish_straight_gesture();
	void toggle_straight_set(TTrack &Track);
	void draw_track_hints();
	bool m_route_tab{true};
	bool m_route_gizmo_using{false};
	glm::mat4 m_route_gizmo{1.0f};
	float m_track_snap_radius{1.0f};
	bool m_track_align_tangent{true};
	bool m_track_drag_connected{true};

	// roads: a menu and a window of their own, independent from the edit modes of the toolset
	// what a click of the build tool would make
	struct road_plan
	{
		std::vector<segment_data> pieces; // axes of the pieces, in the order they're laid
		editor_road::loose_end snap; // loose end of a road or arm of a junction the road is led to
		editor_road::branch start; // junction made, or given another arm, for a road led out of the side of a road or out of a junction
		bool hasstart{false};
		editor_road::branch end; // the same for a road led into the side of a road or into a junction
		bool hasend{false};
		std::string error; // why there's nothing to make
	};
	struct road_tool
	{
		bool window{false}; // the road window is open, the left mouse button belongs to its tools
		bool lanes{true}; // lanes of the roads are drawn as trajectories
		int tool{0}; // 0: select, 1: build, 2: place
		road_node *selected{nullptr};
		junction_node *junction{nullptr}; // selected junction
		roadpoint_node *marker{nullptr}; // selected level crossing or traffic point
		std::vector<glm::dvec3> points; // selected points where the pieces meet or end
		road_node::state settings; // layout shown in the window: of the selected piece, or of the pieces about to be built
		bool whole{true}; // layout changes go to every piece of the road the selected one belongs to
		bool varied{false}; // the selected piece is shown with lanes of its own for its end
		bool chain{false}; // the point to build from is set
		glm::dvec3 point{0.0}; // where the next piece starts
		glm::dvec2 direction{0.0, 1.0}; // the way it has to leave that point
		bool hasdirection{false};
		bool reversed{false}; // the pieces are laid from their ends, to carry on a road from its start
		double grade{0.0}; // slope the next piece has to start with, to carry on what it's attached to
		bool hasgrade{false};
		// the road being built leaves the side of a road, or a junction between its roads. the point to build from is the centre
		// of the junction, which gets made, or gets another arm, along with the first piece
		road_node *branchroad{nullptr};
		double branchat{0.0}; // value of the curve parameter of the axis of that road
		junction_node *branchjunction{nullptr};
		float offset{0.1f}; // height above the ground the roads are built at
		bool follow{true}; // built roads are led over the ground between the clicked points
		float crossingspeed{30.f}; // speed limit on the junctions being made
		float heightstep{0.1f}; // how much the selected points are raised or lowered at a time
		bool ring{false}; // the build tool makes roundabouts
		float ringradius{15.f}; // of the middle of the road going around
		std::string island{"none"}; // what the middle of a roundabout is covered with
		float bankgrade{1.5f}; // metres a bank led to the ground runs for each metre it drops
		// level crossings and traffic points
		int placekind{0}; // what the place tool puts down: 0: level crossing, 1: spawn point, 2: removal point, 3: pedestrian crossing
		roadpoint_node::state placing[4]; // what the next point of each kind is going to be like
		float stopmargin{5.f}; // distance from the outermost track of a new crossing to the places the vehicles stop at
		roadpoint_node::state target; // the point a click of the place tool would make at the moment
		bool hastarget{false};
		glm::dvec3 targetdirection{0.0}; // the way the traffic goes there
		std::string targetnote; // what's worth knowing about that point, or why there's none
		glm::dvec3 aimed{0.0}; // where the cursor was when the target was worked out, and for what kind of point
		int aimedkind{-1};
		char vehicles[4096]{}; // vehicles to add to the set of a spawn point, written the way they're put in a scenery
		road_plan plan; // what a click would make at the moment, worked out once a frame for display
		glm::dvec3 hover{0.0}; // point of a road or a junction under the cursor a road can be led out of
		bool hashover{false};
		glm::dvec3 mouse{0.0};
		std::string status;
		char surface[128]{};
		char sides[2][128]{};
		char filter[64]{"asph"};
		char kerbtext[128]{};
		char mediantext[128]{};
		char banktext[128]{};
		char islandtext[128]{"none"};
	};
	void render_road_menu();
	void render_road_window();
	bool render_road_layout(road_node::state &State);
	bool render_road_material(char const *Label, char *Buffer, std::size_t const Size, std::string &Value);
	void render_junction_layout(junction_node &Junction);
	void junction_apply(junction_node &Junction, junction_node::state const &State);
	// leads the banks of the corners of a junction to the ground beside it
	void junction_bank_to_ground(junction_node &Junction);
	// how far from specified point, and how far down, a slope of the grade set for the banks gets to the ground.
	// Outwards: the way the slope leads, seen from above
	std::pair<float, float> bank_reach(glm::dvec3 const &Edge, glm::dvec3 const &Outwards);
	void draw_road_overlay() const;
	void update_road_tool();
	void road_click();
	void road_select(road_node *Road);
	void junction_select(junction_node *Junction);
	void road_apply();
	bool road_delete();
	bool road_split();
	void road_cancel();
	// sets the width of the road at the selected points, or the height of each of them
	void road_point_width(float const Width);
	void road_point_heights(std::vector<double> const &Heights);
	bool road_points_apply(std::vector<std::pair<road_node *, road_node::state>> const &Changes, std::vector<std::pair<junction_node *, junction_node::state>> const &Junctions);
	// takes the selected points out of their roads
	bool road_points_delete();
	// puts a roundabout around specified point
	void road_roundabout(glm::dvec3 const &Ground);
	// leads the banks of the selected piece, or of the whole road, to the ground beside it
	void road_bank_to_ground();
	// level crossings and traffic points
	void roadpoint_select(roadpoint_node *Point);
	// works out the point a click of the place tool would make. Fresh: even if the cursor is where it was the last time
	void roadpoint_aim(bool const Fresh);
	void roadpoint_place();
	void roadpoint_apply(roadpoint_node &Point, roadpoint_node::state const &State);
	// Point: the point the controls are for, nullptr for the one about to be made
	bool render_roadpoint_layout(roadpoint_node::state &State, roadpoint_node const *Point);
	// Fresh: the ground which wasn't looked up yet is gathered on the spot, instead of a bit of it each frame
	road_plan road_preview(bool const Fresh);
	// Firstgrade, Lastgrade: slopes the road has to start and end with, nullptr for none
	void road_profile(std::vector<segment_data> &Pieces, double const *Firstgrade, double const *Lastgrade, bool const Fresh);
	// height of the ground under each of the points, with the roads not taken for the ground. a point with nothing under it keeps its height
	std::vector<double> ground_heights(std::vector<glm::dvec3> const &Points, bool const Fresh);
	void push_road_snapshot(editor_road::record Record);
	void restore_road_snapshot(EditorSnapshot const &Snapshot, std::vector<EditorSnapshot> &Opposite, bool const Undo);
	road_tool m_roadtool;
};
