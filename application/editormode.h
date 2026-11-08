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
#include "application/editormodegauge.h"
#include "application/editormodejoints.h"
#include "application/editormodelineside.h"
#include "application/editormodeparallel.h"
#include "application/editormodespeed.h"
#include "application/editormodesrk.h"
#include "application/editormodekm.h"
#include "application/editormodevehicle.h"
#include "application/editortool.h"
#include "application/editormodebuildservices.h"
#include "application/editormoderailservices.h"
#include "scene/sceneeditor.h"
#include "scene/scenenode.h"
#include "editor/editorTerrain.hpp"
#include "editor/editorTerrainStreamer.hpp"
#include "editor/editorOrthophoto.hpp"
#include "editor/editorNmt.hpp"
#include "editor/editorGeoMap.hpp"
#include "editor/editorTrack.hpp"
#include "editor/editorConnections.hpp"
#include "editor/editorAlignment.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorSwitchStore.hpp"
#include "editor/editorSwitchCurve.hpp"
#include "editor/editorKilometrage.hpp"
#include "editor/editorSpeed.hpp"
#include "editor/editorRoad.hpp"
#include "editor/editorProfile.hpp"
#include "editor/editorGroundFollow.hpp"
#include "editor/editorInfra.hpp"
#include "editor/editorGauge.hpp"
#include "editor/editorClips.hpp"
#include "editor/editorIcons.hpp"
#include "application/scriptedinput.h"
#include "world/Sweep.h"

#include <array>
#include <map>
#include <tuple>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <sstream>
#include <limits>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

class TAnimModel;
// a tool of a field of work of the editor, a button in the toolbar and over the 3d view
struct editor_tool_button;
class TTrack;
struct build_tools;
struct rail_tools;

class editor_mode : public application_mode, private editor_track::observer, private editor_services, private editor_build_services, private editor_rail_services
{

  public:
	// constructors
	editor_mode();
	~editor_mode() override;

	// the rail tools (editormoderailtools.h), made by make_rail_tools()
	std::unique_ptr<rail_tools> m_rail_tools;
	void make_rail_tools();
	// rail services: overrides
	// rail services: end

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
	// takes the last change back; changes made together (e.g. to the models of a selection) are taken back together
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
		enum class Action { Move, Rotate, Scale, Add, Delete, TrackEdit, Other, RoadEdit, Array, Bend, Terrain, Kilometrage };
		// copy of a model an array made
		struct array_copy
		{
			TAnimModel *model{nullptr}; // nullptr while the copy is out of the scene
			UID uuid;
			// what it takes to make the copy again, filled in when it's taken out of the scene
			std::string serialized;
			std::string name;
			glm::dvec3 position{0.0};
			glm::vec3 rotation{0.0f};
			glm::vec3 scale{1.0f};
		};

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
		std::vector<scene::instance_handle> instances; // includes placed or removed together
		std::vector<std::pair<scene::instance_handle, std::string>> directives; // includes changed together, with the directives to go back to
		std::vector<std::pair<sweep_node *, sweep_node::state>> sweeps; // models along curves changed, with the definitions to go back to
		std::vector<sweep_node *> sweeps_toggled; // models along curves made or removed: undo flips whether they're there
		std::vector<std::pair<TTrack *, editor_track::state>> tracks;
		std::vector<TTrack *> created;
		std::vector<TTrack *> removed;
		editor_road::record roads; // what a change of the roads comes to
		std::vector<infra::object_state> infra; // objects which followed the paths
		bool profiles{false}; // the stored grade line designs changed too, these were the ones before
		std::vector<stored_profile> profile_library;
		std::optional<std::vector<alignment_store::record>> alignments; // the kept horizontal alignments changed too, these were the ones before
		std::optional<std::vector<switch_store::record>> switch_records; // the kept switches changed too, these were the ones before
		std::optional<kilometrage::state> kilometrage; // the kilometrage before the change (Action::Kilometrage)
		std::vector<array_copy> copies; // models an array added to the one it was made of
		bool joined{false}; // undone and redone together with the change before it
		std::shared_ptr<terrain_streamer::change const> terrain; // the heightmap terrain from before the change (Action::Terrain)
		std::uint64_t serial{next_serial()}; // of each step made, for the revision of the history
		static std::uint64_t next_serial()
		{
			static std::uint64_t counter{0};
			return ++counter;
		}
	};
	void push_snapshot(scene::basic_node *node, EditorSnapshot::Action Action = EditorSnapshot::Action::Move, std::string const &Serialized = std::string());

	std::vector<EditorSnapshot> m_history; // history of changes to nodes, used for undo functionality
	std::vector<EditorSnapshot> g_redo;
	int m_track_edits{0};
	// methods
	void update_camera(double const Deltatime);

	editor_ui *ui() const;
	void redo_last();
	// a single step of the history, see undo_last()
	void undo_one();
	void drop_step(std::size_t const Step);
	std::string refused(std::string Text);
	void redo_one();
	// the changes of the heightmap terrain since the last step become a step of the history: a stroke of the brush,
	// or what a frame did; the next change is recorded from now on
	void terrain_history_step();
	// puts the terrain back as the step recorded it, the step left with the state it replaced
	void restore_terrain(EditorSnapshot &Snapshot);
	// swaps the kilometrage with the one the snapshot keeps
	void restore_kilometrage(EditorSnapshot &Snapshot) const;
	bool m_history_busy{false}; // undo or redo under way
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
	// the last place and angles of the camera which were numbers: a camera gone to no finite place goes back there
	glm::dvec3 m_camera_finite_pos{0.0, 0.0, 0.0};
	glm::vec3 m_camera_finite_angle{0.0f};
	bool m_camera_finite{false};

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
	bool m_ortho_pan{false};
	glm::dvec2 m_ortho_pan_cursor{0.0};
	void ortho_pan_to(double const Horizontal, double const Vertical);
	void ortho_pan_stop();
	// camera fly-mode (right mouse button held); used to flush motion when it's released
	command_relay m_camera_relay;
	bool m_camera_flying{false};

	// UI/history settings
	int m_max_history_size{200};
	int m_selected_history_idx{-1};
	glm::dvec3 clamp_mouse_offset_to_max(const glm::dvec3 &offset);
	// the ground under a point the cursor landed on, for things which are to stand on the ground
	glm::dvec3 placement_on_ground(glm::dvec3 Location) override;
	// puts every instance of the model the selected node shows on the ground under it
	void drop_model_instances();
	void render_object_menu();
	// undo, redo and what can be done with the selection, with their keys
	void render_edit_menu();
	// the list of the keyboard shortcuts, opened from the help menu or by F1
	void render_help_menu();
	void render_shortcuts_window();
	bool m_shortcuts_open{false};
	using first_step = editor_first_step;
	// the tutorial: a window of lessons, each with a clip or a sketch, the mouse and the keys it teaches and the steps to try
	struct tutorial_lesson;
	static std::vector<tutorial_lesson> const &tutorial_lessons();
	// a lesson is done once all its steps are
	bool tutorial_done(tutorial_lesson const &Lesson) const;
	void render_first_steps();
	// the steps reached by what the editor is doing now
	void note_first_steps();
	// the lesson of the tool in use: the first of its lessons not done yet, or none
	char const *tutorial_of_tool();
	// the lesson the tool in use or a button of a panel asks for opened, and the clip of the lesson open loaded
	void follow_tutorial();
	void render_tutorial_media(tutorial_lesson const &Lesson, float const Width);
	void render_tutorial_goals(tutorial_lesson const &Lesson);
	// the button under a lesson, opening the tool it's about or switching the view
	void render_tutorial_trial(tutorial_lesson const &Lesson);
	// the buttons to the lesson before and after, and a dot for each lesson, green when it's done
	void render_tutorial_pages();
	void first_step_done(first_step const Step) override;
	bool m_first_steps_open{false};
	std::array<bool, static_cast<std::size_t>(first_step::count)> m_first_steps_done{};
	int m_tutorial_page{0};
	char const *m_tutorial_goto{nullptr}; // the lesson of this title opened by a button of a panel
	std::string m_tutorial_followed; // the lesson the tool in use asked for last: it's opened when that changes
	double m_tutorial_since{0.0};
	editor_clips::clip m_tutorial_clip;
	// deletes what's selected: a road piece or point, a signal, an include of a scenery template, a model, tracks (Del)
	void delete_selected();
	void delete_model(TAnimModel *Model, bool const Joined);

	// more models selected at once, in the surroundings: m_node is the one the gizmo is on, the others are in m_selection.
	// Shift+LMB adds a model or takes it out; a model of a group stands for the whole group
	std::vector<scene::basic_node *> m_selection;
	std::vector<scene::basic_node *> selected_nodes() const; // m_node first
	std::vector<scene::basic_node *> selection_with_groups() const; // and the other models of their groups
	void select_node(scene::basic_node *Node, bool const Additive);
	void draw_selection_overlay();
	// the other models of the selection follow m_node moved or turned by the gizmo
	void selection_snapshots(EditorSnapshot::Action const Action);
	void selection_follow(glm::dvec3 const &Moved, glm::vec3 const &Turned, glm::dvec3 const &Pivot);
	// clipboard (Ctrl+C, Ctrl+V): definitions of the models, placed around the cursor as they were around the selected one
	struct clipboard_entry
	{
		std::string definition;
		glm::dvec3 offset{0.0};
		glm::vec3 angles{0.0f};
		glm::vec3 scale{1.0f};
		int group{-1}; // models copied with their group are grouped again, each group anew
	};
	std::vector<clipboard_entry> m_clipboard;
	void copy_selection();
	void paste_clipboard(glm::dvec3 const &Location);
	glm::dvec3 cursor_ground(); // the point of the ground under the cursor
	// groups (Ctrl+G, Ctrl+Shift+G), written to the scenery as group ... endgroup
	void group_selection();
	void ungroup_selection();
	void drop_selection_to_ground();
	// menu at the cursor, Ctrl+RMB
	bool m_contextopen{false};
	glm::dvec3 m_contextpoint{0.0};
	void render_context_menu();

	// focus camera smoothly on specified node
	void start_focus(scene::basic_node *node, double duration = 0.6);

	// drops the node straight down onto the nearest surface below (terrain or another object). its group goes along with it,
	// unless Alone: then the node goes by itself. Joined: undone together with the change before
	void snap_to_ground(scene::basic_node *node, bool const Alone = false, bool const Joined = false);

	// the heightmap terrain of the scenery (terrain/<name>/): chunks, sculpting, painting, water
	void render_terrain_ui();
	// the terrain as a field of work of its own: the terrain tools and the orthophoto in the tool options window
	void render_terrain_tool_options();
	// opens or leaves the terrain; leaving it puts the terrain tools away, so the mouse doesn't stay with them
	void terrain_workspace(bool const Open);
	// opens the heightmap terrain of the scenery for editing, named after the scenery, if the scenery has none yet. returns: true if it's open
	bool ensure_terrain();
	// adds chunks around the camera, Count along a side, at the height of the ground under each grid point where there's any
	void add_chunks_around(int const Count);
	// adds a chunk of the terrain shaped as the ground under it; where there's no ground, carried on level with the nearest point
	// of it, or at the base height. returns: false if there's a chunk already
	bool add_chunk_from_ground(heightmap::chunk_key const &Key, float const Spacing, std::uint16_t const Layer = 0);

	// the terrain from the digital terrain model of GUGiK (editormodenmt.cpp)
	using nmt_place = ::nmt_place;
	editor_nmt m_nmt;
	nmt_state m_nmt_state;
	editor_map m_nmt_map;
	std::vector<nmt_place> nmt_track_places(double const Step) const override;
	void nmt_locate() override;
	// the same, else the height of the model with the offset of the scenery where its square is in: the ground where there's no
	// terrain made yet, as under the base map of the top view
	bool nmt_ground(double const X, double const Z, double &Height) const;
	// the terrain of the selected tracks in the panel of the tracks
	void render_track_terrain(std::vector<TTrack *> const &Tracks);
	// the grade line the route shares, by most of its paths; -1 when it shares none
	int route_grade_line();
	// the run of the track clicked onto the grade line of the route by hand, or off it if it's on it; a route sharing none makes one with it
	void grade_line_pick(TTrack &Track, bool const Alone);
	// the tool of the terrain the left mouse button works
	enum class terrain_tool
	{
		none, // picks the models, as in the surroundings
		chunks, // adds a chunk, Shift: removes it
		sculpt, // raises the ground, Shift: lowers it
		smooth,
		level, // leads the ground to the target height, Ctrl: picks the target height under the cursor
		paint, // paints the chosen material, Shift: the first material of the palette
		spacing, // selects chunks (Ctrl: adds or takes away one), whose point spacing is then converted
		water, // outlines a body of water
		restore, // takes the touch-ups over the modifiers back
		fill // outlines an area filled with a material
	};
	terrain_tool m_terrain_tool{terrain_tool::none};
	bool terrain_brush() const
	{
		return m_terrain_tool == terrain_tool::sculpt || m_terrain_tool == terrain_tool::smooth || m_terrain_tool == terrain_tool::level || m_terrain_tool == terrain_tool::paint ||
		       m_terrain_tool == terrain_tool::restore;
	}
	// converts the point spacing of the selected chunks to m_terrain_convert_spacing
	void convert_selected_chunks();
	// the chunks selected with the spacing tool taken out of the terrain, in one step of the history
	void remove_selected_chunks();
	// takes the height of whatever is under the cursor (terrain, track, model) for the target of the level brush. returns: false if there's nothing
	bool pick_terrain_target();
	bool terrain_clicking() const { return m_terrain_tool == terrain_tool::chunks || m_terrain_tool == terrain_tool::spacing || m_terrain_tool == terrain_tool::water || m_terrain_tool == terrain_tool::fill; }
	// fills the outlined area with the material of the chosen palette entry, turned by m_paint_rotation
	void fill_terrain_area();
	// the brush, the hovered chunk and the outlines of water drawn over the viewport
	void draw_terrain_overlay();
	// forgets what's known about the ground, after the terrain changed
	void forget_ground();
	// the ground gathered where the terrain changed in the rectangle (x, z) gathered again when asked for, the rest kept
	void forget_ground_in(glm::dvec2 const &Min, glm::dvec2 const &Max);
	// conversion of the ground of the terrain files of the scenery (terrain directive, .txtf/.btf) to heightmap terrain
	struct ground_conversion
	{
		struct source
		{
			std::string file; // as the terrain directive names it
			std::uint16_t reference{0}; // see scene::terrain_file::reference_of()
			bool text{false}; // read from the text file, rather than the binary one
			bool good{false};
			bool chosen{true};
			std::string message;
			std::size_t first{0}, count{0}; // its triangles
		};
		bool open{false};
		std::vector<source> files;
		std::vector<std::array<glm::dvec3, 3>> triangles;
		std::vector<std::uint16_t> materials; // of the triangles: index of the name
		std::vector<std::string> names; // of the materials
		std::set<std::size_t> chosen; // materials to convert
		std::string status;
	} m_ground_conversion;
	// reads the triangles of the terrain files of the scenery for the conversion
	void load_ground_files();
	// the folder of the textures as a browser, in the window of the node bank while the terrain is edited: the textures and the
	// materials can be dragged onto the fields which take a material
	struct texture_browser
	{
		struct item
		{
			std::string name; // as the materials are named: relative to the folder of the textures, without the extension
			std::string label; // the file, without the folder
			bool material{false}; // there's a .mat file of the name
		};
		std::string folder; // relative to the folder of the textures, empty or with a trailing slash
		std::vector<std::string> folders; // under the current one
		std::vector<item> items;
		bool listed{false};
		char filter[64]{};
		float size{96.f}; // of the images, pixels
		std::string selected;
		std::map<std::string, int> thumbnails; // texture shown for the item of the name, 0: none
		int loads{0}; // textures read in the frame, a few at most so the browser doesn't stall the editor
		int loadsframe{-1};
	} m_textures;
	void render_texture_browser();
	// texture of the image of the item of the browser, 0 while there's none
	std::uint64_t texture_thumbnail(texture_browser::item const &Item);
	// takes a material dragged from the texture browser onto the item drawn last. returns: true if one was dropped, its name in Name
	static bool material_drop(std::string &Name);
	// the same, for a text field
	static bool material_drop(char *Buffer, std::size_t Size);
	// replaces the chosen triangles of the chosen terrain files with chunks of heightmap terrain of their shape, painted with
	// their materials; the files lose them on save. returns: summary for the user
	std::string convert_ground();
	void render_ground_conversion();
	// handles a click of the chunk, spacing and water tools (Shift: the other way round)
	void handle_terrain_click(bool const Shift);
	// writes the changes to the files of scenery opened for editing (Ctrl+S)
	bool save();
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
	// undo and redo of the includes placed, removed or changed together
	void restore_includes(EditorSnapshot &Snapshot);
	// gizmo for the selected include, limited to what the parameters of its template can express
	void render_include_gizmo(glm::mat4 const &View, glm::mat4 const &Projection, glm::dvec3 const &Camerapos);
	// exports the scenery in legacy (text) format, with layers hidden in the editor left out of the picture
	void export_scenery();
	// works the brush of the terrain under the cursor while the left mouse button is held
	void handle_terrain_brush(double Deltatime);
	// the chunk of the terrain (if any) covering the given world point
	editor_terrain *terrain_at(double X, double Z);
	// the chunks of the terrain held in memory
	std::vector<editor_terrain *> active_terrains() const;
	// samples the selected model instance's geometry into the chunks of the terrain it covers, then removes it
	void capture_terrain();
	bool m_terrain_open{false};       // the terrain is the field of work, chosen in the toolbar
	float m_terrain_spacing{2.0f};    // point spacing of the chunks made and of the spacing tool
	float m_terrain_baseheight{0.0f}; // height of chunks made where there's no ground
	int m_terrain_add_count{4};       // chunks along a side added around the camera
	float m_terrain_brush_radius{12.0f};
	float m_terrain_brush_strength{4.0f}; // metres per second while held; for the paint, how fast it covers
	// the level brush
	float m_terrain_target{0.0f}; // height the ground is led to
	int m_terrain_level_mode{0}; // 0: up and down, 1: only raises, 2: only lowers
	bool m_terrain_target_from_stroke{false}; // the target is the height where a stroke starts
	bool m_terrain_stroke{false}; // a stroke of the brush is under way
	bool m_terrain_picking{false}; // the next click picks the target height
	int m_terrain_layer{0};               // entry of the palette painted
	char m_terrain_material[128]{"grass"}; // material added to the palette
	float m_terrain_material_size{0.0f};   // metres its textures repeat at, 0: given by the material
	float m_terrain_layer_size{0.0f};      // repeat of the chosen palette entry, while it's dragged
	bool m_terrain_layer_size_dragged{false};
	float m_terrain_layer_rotation{0.0f};  // angle of the chosen palette entry, while it's dragged
	bool m_terrain_layer_rotation_dragged{false};
	std::vector<glm::dvec3> m_water_points; // outline of a body of water being drawn
	float m_water_level{0.0f};
	bool m_water_level_edited{false}; // the level was given by hand, the points clicked don't change it
	char m_water_material[128]{""};
	int m_water_selected{-1};
	char m_water_selected_material[128]{""}; // material of the chosen water, while it's edited
	int m_water_material_of{-1}; // the water whose material is in the field above
	float m_water_selected_size{0.0f}; // metres its texture repeats at, 0: given by the material
	std::set<heightmap::chunk_key> m_terrain_selection; // chunks chosen with the spacing tool
	float m_terrain_convert_spacing{1.0f}; // point spacing the selected chunks are converted to
	bool m_terrain_convert_paint{true}; // chunks converted to 0.5 m get paint samples every 0.5 m too
	std::vector<glm::dvec3> m_paint_outline; // of the area filled with a material
	float m_paint_feather{0.0f}; // metres the filled material blends in over across the outline, 0: sharp
	float m_paint_rotation{0.0f}; // degrees the textures of the filled material are turned by
	float m_water_selected_level{0.0f}; // level of the chosen water, while it's edited
	bool m_water_level_dragged{false};
	std::string m_terrain_status;

	// the heightmap terrain, shared with the scenery loader so it shows in every mode
	terrain_streamer &m_streamer{EditorTerrain};

	// geoportal orthophoto layer drawn under the other viewport overlays
	void render_orthophoto_ui();
	void render_map_menu();
	bool m_newscenery_asked{false};
	void render_new_scenery_popup();
	bool restart_for_new_scenery();
	bool restart_editor(std::string const &Scenery);
	bool m_openscenery_asked{false};
	bool m_opened_at_track{false};
	bool m_turntable_included{false};
	bool turntable_controls(std::vector<std::string> &Rootstatements);
	bool m_nose_included{false};
	bool nose_controls(std::vector<std::string> &Rootstatements);
	void starter_trainset(std::vector<std::string> &Trailing);
	std::optional<std::pair<glm::dvec3, glm::vec3>> m_saved_camera;
	char m_openscenery_search[64]{};
	std::vector<std::string> m_openscenery_list;
	std::string m_openscenery_choice;
	void render_open_scenery_popup();
	// the settings of the orthophoto, among the base layers in the layers window: asked to be opened there, the frame they were last
	// drawn open in
	bool m_orthophoto_window{false};
	// the user turned the orthophoto on or off, or the scenery keeps it so: the terrain model made, or the area drawn, leaves it as it
	// is then, it turns it on only where nobody chose
	bool m_orthophoto_chosen{false};
	// the user turns the orthophoto on or off: kept with the editor, for every scenery
	void choose_orthophoto(bool const On);
	int m_orthophoto_frame{-10};
	bool orthophoto_settings_open() const;
	// view menu: the top view, the lanes of the roads, the structure gauge, the environment
	void render_view_menu();
	// time of day, day of the year, visibility, clouds and air temperature, as the scenery is seen in the editor
	void render_environment_window();
	bool m_environment_open{false};
	// over the 3d view: the tools of the field of work along its left edge, the modes of the gizmo at its top, the axes of the
	// world and the top view in its top right corner
	void render_viewport_overlays();
	// the tools of the field of work, in the order of the toolbar
	void work_area_tools(std::vector<editor_tool_button> &Buttons);
	// the tools of the tracks and of the terrain, the ones behind More too while they're shown or one of them is in use
	void track_area_tools(std::vector<editor_tool_button> &Buttons);
	void terrain_area_tools(std::vector<editor_tool_button> &Buttons);
	// the tools of the field of work in the toolbar, as icons with their names
	void render_toolbar_tools();
	// what the row of the toolbar holds, written at its left end: the fields of work, the tools
	static void render_toolbar_caption(char const *Text);
	void render_viewport_gizmo();
	void render_view_axes();
	// the camera looking along an axis of the world (0: x, 1: y, 2: z) from its positive or negative side
	void look_along_axis(int const Axis, bool const Positive);
	// origin of the scenery in PUWG 1992 the scenery file gives in its //$g line, if it does
	bool m_georeference_read{false};
	bool m_georeference{false};
	glm::dvec2 m_georeference_origin{0.0}; // northing, easting
	std::string m_georeference_line;
	void read_georeference();
	void draw_orthophoto();
	void draw_ortho_compass() const;
	// picks up the stored settings and the origin of the current scenery, and the offset of the terrain model the scenery has
	void load_orthophoto_settings();
	void save_orthophoto_settings();
	// the settings of the terrain model changed, kept with the editor, and the offset of the model with the scenery's
	void keep_nmt_settings();
	editor_orthophoto m_orthophoto;
	// the WMS layers of the user's over the orthophoto, the top one first, as the settings of the editor keep them; their tiles are
	// kept in memory only
	std::vector<std::unique_ptr<editor_orthophoto>> m_wms;
	// a WMS layer being added: the address typed, what its capabilities tell, read on a thread of its own, the layers ticked
	struct wms_adding
	{
		std::array<char, 512> url{};
		std::array<char, 64> filter{};
		std::future<editor_orthophoto::service_info> reading;
		std::jthread reader; // reads them, the future above gets what it found; joined before the future goes
		std::optional<editor_orthophoto::service_info> info;
		std::string service; // the address the capabilities were read of
		std::vector<char> ticked;
	};
	wms_adding m_wms_adding;
	// the orthophoto and the WMS layers of the user's at the bottom of the layers window, with their settings
	void render_base_layers();
	void render_wms_adding();
	// the WMS layers as the settings have them, laid out by the origin and on the ground of the orthophoto, around the camera
	void update_wms_layers();
	// the ground the imagery is fitted onto, the same for the orthophoto and the WMS layers
	void ground_sources(editor_orthophoto &Layer);
	std::size_t m_ortho_resident{0}; // chunks of the terrain in memory, as the imagery last heard of them
	std::size_t m_ortho_waiting{0}; // squares of the terrain model asked for and not in yet, the same
	std::string m_orthophoto_scenery;            // scenery whose origin is loaded
	std::string m_orthophoto_mark;
	std::vector<std::string> m_orthophoto_area_mark; // the lines of the download area as the scenery keeps them
	// the download area being drawn on the map: its points in the world (x, z), the one dragged
	area_drawing m_area_drawing;
	bool m_area_paused{false}; // the download of the area held by the button; out of the editor it's held anyway
	glm::dvec2 m_orthophoto_origin_edit{0.0};    // origin being typed in (northing, easting); applied when editing ends

	// hierarchy management
	void add_to_hierarchy(scene::basic_node *node);
	void remove_from_hierarchy(scene::basic_node *node);
	void erase_model(TAnimModel *model);
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

	// array: copies of the selected model set out in a row, after the array modifier of Blender
	struct array_settings
	{
		int fit{0}; // 0: fixed count, 1: as many as the length takes
		int count{2}; // the model itself included
		float length{10.0f}; // m
		bool relative{true};
		glm::vec3 relative_offset{1.0f, 0.0f, 0.0f}; // of the size of the model, along its axes
		bool constant{false};
		glm::vec3 constant_offset{1.0f, 0.0f, 0.0f}; // m, along the axes of the model
		float turn{0.0f}; // degrees around the vertical axis, each copy against the one before it
		bool ground{false}; // the copies are put as high over the ground as the model is
		bool operator==(array_settings const &) const = default;
	};
	struct array_tool
	{
		static constexpr int limit = 1000; // models in an array, so a slip of the finger doesn't freeze the editor
		static constexpr double minimal_step = 0.001; // m, copies any closer would sit in one another
		array_settings settings;
		// the array made last, which follows the settings until something else is selected or changed
		TAnimModel *base{nullptr}; // model it was made of, nullptr if there's none
		std::size_t step{0}; // size of the undo history with the array at its top
		array_settings applied; // what the copies were set out with
		glm::dvec3 location{0.0}; // of the model then
		glm::vec3 angles{0.0f};
		glm::vec3 scale{1.0f};
		// size of the model asked about last
		TModel3d const *sized{nullptr};
		glm::vec3 size{0.0f};
		std::string status; // outcome of the last operation, shown in the panel
	};
	array_tool m_array;
	void render_array();
	// size of the model of specified instance along its axes, the scale of the instance left out
	glm::vec3 array_size(TAnimModel const &Base);
	// offset of each copy from the one before it, along the axes of the model
	glm::dvec3 array_offset(TAnimModel const &Base);
	// number of the models in the array, the one it's made of included. Step: length of the offset
	int array_count(double const Step) const;
	// location and angles of each copy in turn
	std::vector<std::pair<glm::dvec3, glm::vec3>> array_placements(TAnimModel &Base);
	// true while the array made last follows the settings
	bool array_live() const;
	void make_array();
	// brings the copies of the array made last to what the settings ask for
	void shape_array();
	void update_array();
	void restore_array_snapshot(EditorSnapshot Snapshot, std::vector<EditorSnapshot> &Opposite, bool const Undo);

	// ImGuizmo-based transform gizmo for the selected node
	enum class gizmo_operation { translate, rotate, scale };
	void render_gizmo();
	// gizmo settings, drawn in the toolset window
	void render_gizmo_options();
	// how the top view draws the tracks: as a combo with the colour of the rails, or as items of a menu
	void render_top_tracks_choice(bool const Combo);
	// the tools of the field of work in the toolbar under the menu
	void render_toolbar();
	// an edit mode of the node bank as the field of work, from the toolbar or its key (1-5)
	void choose_edit_mode(nodebank_panel::edit_mode const Mode);
	// fields of work of the editor, in the row above the toolbar: the models around, the tracks, the roads, the terrain
	enum class work_area
	{
		surroundings,
		tracks,
		roads,
		terrain
	};
	work_area current_work_area() const;
	void choose_work_area(work_area const Area);
	void render_workspaces();
	bool m_gizmo_enabled{true};                                  // master switch for the in-viewport gizmo
	bool m_track_gizmo_on{false};                                // the gizmo in the tracks: off until Q, W or E, so a track isn't moved by chance
	bool gizmo_shown() const;                                    // the switch of the field of work in use
	bool m_gizmo_using{false};                                   // tracks an ongoing drag, so a single undo snapshot is taken per drag
	bool m_gizmo_local{false};                                   // manipulate in the object's local space instead of world space
	gizmo_operation m_gizmo_op{gizmo_operation::translate};      // current transform mode (translate/rotate/scale)
	float m_gizmo_snap{1.0f};                                    // translation snap step (metres) applied while Ctrl is held

	TTrack *selected_track() const override;
	bool track_editing() const override;
	std::size_t history_revision() const override;
	glm::dvec3 camera_position() const override;
	track_hover const &hovered_track() const override;
	std::vector<TTrack *> const &track_set() const override;
	editor_track_tab current_track_tab() const override;
	void render_path_ui();
	// a field of the path of the Track edited in the inspector: the state Before kept as it's taken, the step of the undo history and the
	// track built again as it's let go
	void path_field_done(TTrack &Track, editor_track::state const &Before);
	// the points of the path Index of the Track to pick: its ends, (+) joined or (-) free, and its control points
	void render_path_points(TTrack const &Track, int const Index);
	// the exact position of the point picked of the path Index of the Track, or its control vector; the state Before for the undo history
	void render_point_fields(TTrack &Track, int const Index, editor_track::state const &Before);
	// the cant and the radius of the Path of the Track, and its straightening; the state Before for the undo history
	void render_cant_radius(TTrack &Track, segment_data &Path, editor_track::state const &Before);
	void draw_track_overlay() const;
	editor_track::point_ref track_handle_hit() const;
	void select_track(scene::basic_node *Node);
	void render_track_gizmo();
	void commit_track_drag(bool const Force);
	void cancel_track_tools();
	void push_track_snapshot(std::vector<std::pair<TTrack *, editor_track::state>> States, std::vector<TTrack *> Created = {}, std::vector<TTrack *> Removed = {}) override;
	void push_kilometrage_snapshot() override;
	void set_current_node(scene::basic_node *Node) override;
	alignment::limits const &route_norms() const override;
	void clear_track_point() override;
	editor_nmt &terrain_model() override;
	double terrain_model_offset() const override;
	bool terrain_model_fitted() const override;
	// false, with the reason, when one of the paths can't be changed, its file not taking the changes back
	static bool tracks_editable(std::vector<TTrack *> const &Tracks, std::string &Reason);
	void trim_history();
	void restore_track_snapshot(EditorSnapshot const &Snapshot, std::vector<EditorSnapshot> &Opposite, bool const Undo);
	// changes of the kept alignments and switches go into the last step of the history, made just before them
	void records_to_history();
	// the paths a drag of the gizmo takes, to tell after it what became of the kept alignments and switches
	struct record_watch
	{
		bool active{false};
		std::vector<TTrack *> moving;
		std::vector<std::pair<glm::dvec3, glm::dvec3>> before; // ends of the first path of each moving track
		std::map<int, std::vector<TTrack *>> on; // the kept alignments and the moving paths on them
		std::set<int> whole; // the kept alignments all of whose paths move
	};
	record_watch m_record_watch;
	void records_watch_begin();
	void records_watch_end();
	editor_track::point_ref m_track_point;
	std::vector<TTrack *> m_track_drag;
	std::vector<std::pair<TTrack *, editor_track::point_ref>> m_track_drag_points;
	std::vector<editor_track::joint> m_track_joints; // of the neighbours, following the whole track dragged
	editor_track::snap_target m_track_snap;
	glm::dvec3 m_track_pivot{0.0};
	glm::mat4 m_track_gizmo{1.0f};
	bool m_track_gizmo_using{false};
	struct track_gizmo_drag
	{
		gizmo_operation operation{gizmo_operation::translate};
		bool on_control{false};
		double angle{0.0};
		int path{-1};
		bool atend{true};
		std::size_t pieces{1};
		std::vector<segment_data> paths;
		editor_track::point_ref point;
		glm::dvec3 origin{0.0};
		glm::dvec3 axis{0.0};
		double grabbed{0.0};
		double length{0.0};
		double stretch{0.0};
		std::vector<std::pair<TTrack *, editor_track::point_ref>> partners;
		glm::dvec3 fixed{0.0};
		std::vector<std::pair<TTrack *, std::vector<int>>> held;
		std::vector<TTrack *> extra;
		int ring{-1};
		glm::dmat3 basis{1.0};
		glm::dvec3 pivot{0.0};
		double grade{0.0};
		double roll{0.0};
		double roll0{0.0};
		double pivot_chainage{0.0};
		std::vector<double> chainages;
		std::vector<double> lengths;
		struct switch_joint
		{
			TTrack *track{nullptr};
			editor_track::point_ref own;
			editor_track::point_ref theirs;
			glm::dvec3 point{0.0};
			glm::dvec3 tangent{0.0};
			double roll{0.0};
			bool moved{false};
			std::vector<std::pair<TTrack *, std::vector<segment_data>>> cluster;
			std::vector<std::pair<TTrack *, std::vector<editor_track::joint>>> joints;
		};
		std::vector<switch_joint> switches;
	};
	track_gizmo_drag m_track_gizmo_drag;
	gizmo_operation track_gizmo_operation(TTrack const &Track, bool const Pointmode) const;
	glm::mat4 track_gizmo_frame(TTrack const &Track, bool const Pointmode, gizmo_operation const Operation) const;
	void track_gizmo_hold();
	void track_gizmo_settle();
	void track_gizmo_stretch(TTrack &Track, double const Length);
	void track_gizmo_control(TTrack &Track, glm::dvec3 Control);
	void track_gizmo_point(TTrack &Track, glm::dvec3 const &Position) const;
	void track_gizmo_switches(TTrack const &Track);
	void track_gizmo_conform(TTrack &Track) const;
	bool track_gizmo_clustered(TTrack const *Track) const;
	void track_gizmo_spill(TTrack &Track, std::size_t const Pieces);
	void track_gizmo_carry(track_gizmo_drag::switch_joint const &Link, glm::dvec3 const &Point, glm::dvec3 const &From, glm::dvec3 const &To, double const Grade) const;
	void track_gizmo_roll_switch(track_gizmo_drag::switch_joint const &Link, double const Roll) const;
	void track_gizmo_take_joints(TTrack const &Track);
	void track_gizmo_take_points(TTrack const &Track);
	void track_gizmo_take_partners(TTrack &Track);
	void track_gizmo_take_switch_joints(TTrack const &Track, bool const Controlmode);
	// the joint of the track with a switch, the switches of the cluster going with it given
	track_gizmo_drag::switch_joint track_gizmo_link(TTrack const &Track, editor_track::point_ref const &Own, std::pair<TTrack *, editor_track::point_ref> const &Connection, std::vector<TTrack *> const &Members);
	void track_gizmo_take_switches(TTrack &Track);
	// the other switch of a double switch and the track between them go with the moved or turned switch as a whole
	void track_gizmo_take_double(TTrack &Track);
	void track_gizmo_measure_rotation(TTrack &Track, bool const Pointmode);
	void track_gizmo_measure(TTrack &Track, bool const Pointmode, gizmo_operation const Operation);
	void track_gizmo_begin(TTrack &Track, bool const Pointmode, gizmo_operation const Operation);
	void track_gizmo_turn_direction(TTrack &Track, double const Angle);
	void track_gizmo_turn_grade(TTrack &Track, double const Angle);
	void track_gizmo_turn_cant(TTrack &Track, double const Angle);
	void track_gizmo_turn(TTrack &Track, int const Axis, double const Angle);
	void track_gizmo_rotated(TTrack &Track);
	void track_gizmo_scaled(TTrack &Track);
	void track_gizmo_moved_point(TTrack &Track, glm::dvec3 const &Position);
	void track_gizmo_length(TTrack &Track, double Length);
	void track_gizmo_end(TTrack &Track);
	bool m_track_dirty{false};
	bool m_track_drag_on_the_way{false}; // committed, but not as the end of the drag
	std::chrono::steady_clock::time_point m_track_last_commit;
	editor_track::state m_track_field_before;
	// the event to add to the path, typed
	TTrack const *m_track_events_for{nullptr};
	std::array<char, 128> m_track_event{};
	int m_track_event_list{0};
	std::string m_track_events_missing;
	void render_path_events(TTrack &Track);
	// the events of the list List of the Track, one taken off with x where they're Editable; the state Before for the undo history
	void render_event_list(TTrack &Track, int const List, bool const Editable, editor_track::state const &Before);
	std::string m_track_mode_notice;
	void render_route_ui();
	void route_trace_start();
	void route_trace_fit();
	void route_trace_end(bool const Keep);
	int route_trace_hit() const;
	glm::dvec3 route_trace_cursor() const;
	void route_bind_ends();
	alignment::vertex route_vertex_of(editor_track::curve const &Curve) const;
	bool route_load(TTrack &Track);
	void route_take_stored(int const Index);
	void record_switch(TTrack const &Track, editor_track::switch_template const &Shape) const;
	void record_double(TTrack const &First, TTrack const &Second, editor_track::switch_template const &Shape) const;
	// the two switches of a double switch and the plain track between them, found from any of the three
	struct double_switch
	{
		TTrack *first{nullptr};
		TTrack *link{nullptr};
		TTrack *second{nullptr};
	};
	std::optional<double_switch> double_of(TTrack const &Track) const;
	// the double switch the switch is kept as a part of
	static std::optional<double_switch> double_of_switch(TTrack const &Switch);
	// a slip of the scenery the track is a part of: its switches, all its tracks, the switches among them, and its ends where it joins
	// the tracks around, A and B of its first track through, C and D of the other one, with the way into the slip at each
	struct slip_set
	{
		switch_store::record record;
		std::vector<TTrack *> switches;
		std::vector<TTrack *> parts;
		std::array<glm::dvec3, 4> ends{};
		std::array<glm::dvec2, 4> into{};
	};
	std::optional<slip_set> slip_of(TTrack const &Track) const;
	// the two tracks of the slip through it from end to end, as they cross where it's taken out
	static std::array<segment_data, 2> slip_throughs(slip_set const &Slip);
	// the slip taken out with its record, nothing of it put in the history yet; with Throughs its two tracks laid crossing in its
	// place, given back
	std::vector<TTrack *> slip_taken_out(slip_set const &Slip, bool const Throughs) const;
	// the record of the crossing of two plain tracks the track is one of
	std::optional<switch_store::record> crossing_record(TTrack const &Track) const;
	// the paths of the set in the order editor_track::place_switch gives them
	static std::vector<segment_data> double_paths(double_switch const &Set);
	// the switch of the template, or the two switches of a double one with the plain track between them, committed and recorded
	std::vector<TTrack *> create_switch_set(editor_track::switch_template const &Shape, std::vector<segment_data> const &Paths, TTrack const &Style, std::string const &Name);
	// with no Style, as the lay tool lays its tracks
	std::vector<TTrack *> create_switch_set(editor_track::switch_template const &Shape, std::vector<segment_data> const &Paths, TTrack const *Style, std::string const &Name);
	void record_slip(std::span<TTrack *const> const Parts, std::string_view const Kind, std::string_view const Type, double const Radius, double const Angle, std::string_view const Base, double const Middle = 0.0) const;
	void render_switch_record(TTrack const &Track) const;
	bool digitize_drawn(std::vector<TTrack *> const &Created, bool const Alone = true) override;
	// the lines kept, the paths left out
	void digitize_all(int *Kept = nullptr, int *Failed = nullptr);
	// the lines, the switches, the crossovers and the objects along the tracks at one go; without Write only counts them
	std::string digitize_scenery(bool const Write);
	void route_update()
	{
		m_route.design.norms.checked = true;
		m_route.result = alignment::compute(m_route.design);
		++m_route.revision;
	}
	void route_apply();
	glm::dvec3 route_vertex_position(int const Vertex) const;
	bool route_active() const;
	route_design m_route;

	void render_straights_ui();
	editor_track::straight const &current_straight();
	bool place_straight_tool();
	double straight_tool_radius(editor_track::straight const &Line) const;
	void straight_refresh() override;
	void add_detour_point();
	std::vector<glm::dvec3> detour_outline() const;
	parallel_offer straight_parallel_offer(editor_track::straight const &Line, glm::dvec3 const &Offset) const;
	straights_state m_straights;

	struct curve_sample
	{
		double station{0.0};
		glm::dvec3 position{0.0};
		glm::dvec2 tangent{0.0, 1.0};
		float roll{0.f};
		float radius{0.f};
	};
	// the line leaving an end of a switch off its main straight, taken before the switch moves: its paths up to the end of
	// the straight past the curve next to the switch. When the switch moves only the paths up to a joint on that straight
	// are laid again, so the curve keeps to the end of the switch and the rest of the network stays where it is
	struct switch_branch
	{
		std::size_t joint{0};
		editor_track::chain chain;
		std::size_t straight{0}; // the first path of the straight past the curve
		alignment::design design; // as the paths lie
		alignment::result result;
		// made for the new end of the switch
		editor_track::chain part; // the paths laid again
		std::vector<segment_data> pieces;
		alignment::design laid; // of the whole chain
		double factor{1.0}; // of the radius and the transitions of the curve
	};
	struct switch_tool
	{
		std::vector<std::string> keep_names; // the names the switches laid next take, before the ones of the tool
		std::string keep_slip; // the name the slip or the crossing laid next takes, of the one it's laid in place of
		bool curved{false};
		// a switch placed where there's no track: started on the ground, or at the free end it goes on from in its direction; the direction
		// of its main track taken from the drag once it's long enough, then the drag sideways gives the side of the diverging one
		bool free{false};
		editor_track::snap_target free_end;
		std::optional<glm::dvec2> free_heading;
		// from a free end, dragged on past its end: bent, its diverging track turning the other way than the main one, the same way with
		// Tab (one-sided)
		bool one_sided{false};
		std::vector<curve_sample> frame;
		std::vector<TTrack *> frame_tracks;
		std::vector<editor_track::switch_template> templates;
		bool collected{false};
		int armed{-1};
		int last{0}; // template armed the last time
		bool placing{false};
		// where it was pressed on the screen: let go no farther from there, the press places nothing, a click doesn't lay a switch by chance
		glm::vec2 pressed_at{0.f};
		double along{0.0};
		double pressed{0.0}; // along a curve: where the press was, the start of the switch drawn from it to the ends of the cant ramps
		glm::dvec3 point{0.0};
		glm::dvec3 mouse{0.0};
		editor_track::straight line;
		std::string status;
		char name[64]{};
		struct drive_family
		{
			std::string name;
			std::array<std::string, 4> files;
			std::array<double, 4> heights{};
			bool preferred{false};
		};
		std::vector<drive_family> drives;
		int drives_revision{-1}; // of the include bank the drives were read from
		int drive{-1};
		int drive_side{0};
		struct slide_state
		{
			bool active{false};
			TTrack *track{nullptr};
			glm::vec2 pressed{0.f};
			double grip{0.0};
			double low{0.0};
			double high{0.0};
			double distance{0.0};
			double length{0.0};
			std::vector<double> joints;
			// the lines leaving the switch off its straight: the joint they leave at, the path of the switch in the shown ones
			struct branch_line
			{
				editor_track::joint joint;
				std::size_t path{0};
				switch_branch branch;
			};
			std::vector<branch_line> branches;
			// a switch bent into a curve slides along it: the curve its main track and the tracks next to it make, where the switch
			// starts on it; low and high the distances it may go back and on
			std::vector<curve_sample> frame;
			double station{0.0};
		};
		slide_state slide;
		double slide_typed{0.0};
		int connection{-1};
		int connection_kind{0}; // the kind of connection chosen last, taken again when the connections are chosen
		std::string rail_pick{"60E1"}; // the rail whose templates are listed first where the track doesn't tell its rail
		int connection_second{-1};
		double connection_radius{0.0};
		double connection_gap{6.0};
		bool connection_together{false};
		std::string connection_spacings{"4.75;4.75;5.0"};
		double connection_stub{25.0};
		// the connection a switch dragged makes, as where it's let go tells it, in connection then: let go on the other track a
		// crossover, across the tracks between them a ladder; by a crossover there already turned the other way a double crossover
		// with it, over it a scissors one
		bool recognised{false};
		TTrack *partner{nullptr}; // a switch of the crossover there already
	};
	switch_tool m_switch;
	// what the switch tool places: a switch into a straight or a curve, a slip switch or a crossing at a crossing of two
	// straights, or a connection of tracks laid whole
	enum class switch_kind
	{
		single,
		slip,
		crossing,
		connection
	};
	switch_kind current_switch_kind() const;
	void choose_switch_kind(switch_kind const Kind);
	bool switch_kind_offered(switch_kind const Kind) const;
	bool plan_connection(connection::request &Request, connection::result &Result) const;
	// the lines of the straights a ladder from the line of the tool onto the target crosses, the nearest straight at each distance
	std::vector<connection::line> ladder_lines(connection::line const &Target, glm::dvec2 const &Travel, std::vector<TTrack const *> const &Exclude) const;
	// what the drag of the switch being placed makes, from where the cursor is: sets connection and partner of the tool
	void recognise_connection();
	// along a curve: whether the switch is let go on a track running beside it
	bool let_go_beside_curve() const;
	// the crossover on the two lines nearest the press, turned the other way, made the partner of the tool with the connection they make
	void recognise_partner(connection::line const &Line, connection::line const &Otherline);
	// the double or the scissors crossover of the partner and the crossover dragged, on the lines of their tracks
	bool plan_partner_connection(connection::request &Request, connection::result &Result) const;
	void insert_partner_connection();
	// the switches and the inserts of the connection planned laid, one step of the history. false if they don't fit
	bool lay_planned_connection(connection::request const &Request, connection::result const &Plan, TTrack *Style);
	// a switch of the connection laid where the plan puts it, onto the straight there or by itself; nullptr if it can't be, with the
	// problem, or none when the switch tool tells it
	TTrack *lay_connection_switch(connection::placed const &Entry, connection::request const &Request, TTrack const &Style, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created,
	                              std::vector<TTrack *> &Removed, char const *&Problem);
	void insert_connection();
	void render_connection_options();
	// the name of a kind of connection of tracks, translated
	static char const *connection_name(int const Kind);
	// the name of a switch template for the lists: where it comes from and its name without the lengths
	static std::string switch_template_label(editor_track::switch_template const &Template);
	// the name of a switch template for the user: translated, with the speed on its diverging track
	static std::string template_name(editor_track::switch_template const &Template);
	// the template of the catalogue a record of the switches was laid of, -1 if none: Lengths compares a and b too, which the
	// record of a connection keeps for its spacing; Movable: the nose of the frog, either when not given
	int matching_template(switch_store::record const &Record, bool const Lengths, std::optional<bool> const Movable = std::nullopt) const;
	// the template the switch was laid of, -1 if none: a switch of the scenery of its own shape, say
	int switch_template_index(TTrack const &Switch) const;
	// why the switch can't take another template here, empty when it can
	std::string template_change_blocked(TTrack const &Switch) const;
	// the template of the catalogue a slip or a crossing was laid of, of the rail before the others; -1 if none
	int slip_template_index(switch_store::record const &Record, std::string_view const Rail) const;
	// the switch of the template of the index in place: its start and its side kept, the tracks it joins following; a switch of a
	// crossover takes the crossover with it, laid again, a slip or a crossing is laid again where its tracks cross. One step of the
	// history; the status says how it went
	bool change_switch_template(TTrack &Switch, int const Index);
	bool change_slip_template(TTrack const &Part, int const Index);
	// the switch, or the double switch it's a part of, given the shape in place, straight or bent into the curve it lies in; Flip:
	// the diverging track to the other side. Notes: for the user
	bool apply_switch_template(TTrack &Switch, editor_track::switch_template const &Shape, bool const Flip, std::string &Notes);
	// the same along the straight, the start and the direction of the switch kept; and bent into the curve it lies in, its start kept
	// where it is along the curve
	bool reapply_straight_switch(TTrack &First, std::optional<double_switch> const &Set, editor_track::switch_template const &Shape, bool const Flip, std::string &Notes);
	bool reapply_bent_switch(TTrack &First, editor_track::switch_template const &Shape, bool const Flip, std::string &Notes);
	// the template to choose: the speeds on the diverging track as chips, the templates of the rail by their skos, all the others
	// folded under. Fits: of each template by its index, 1 for those of the kind wanted; Rail: of the track, empty when it doesn't
	// tell; Popup: inside a combo, with no windows of its own. Gives the index picked, -1 while none is
	int render_template_picker(char const *Id, std::span<char const> const Fits, std::string const &Rail, int const Chosen, bool const Popup);
	// of each template of the switch tool by its index, 1 for those that fit
	template <typename Fit_> std::vector<char> templates_fitting(Fit_ const &Fit) const
	{
		std::vector<char> fitting;
		fitting.reserve(m_switch.templates.size());
		for (auto const &entry : m_switch.templates)
			fitting.push_back(Fit(entry) ? 1 : 0);
		return fitting;
	}
	// the template of the selected switch, slip or crossing, changed with a pick: in a combo for the inspector, else in full; nothing
	// for another track
	void render_switch_template(TTrack &Track, bool const Combo);
	// what a switch, a slip or a crossing is offered the templates by: those that fit it, its rail, the template it has, its record
	struct template_offer
	{
		TTrack const *first{nullptr}; // of a double switch, its first switch
		std::optional<switch_store::record> known;
		std::function<bool(editor_track::switch_template const &)> fits;
		std::string rail;
		int current{-1};
	};
	// what the track is offered the templates by; false for a track that isn't a switch, a slip or a crossing
	bool template_offer_of(TTrack const &Track, template_offer &Offer) const;
	// the name of the template the track has, or of its own shape
	std::string offered_name(template_offer const &Offer) const;
	// the template in a combo of the inspector: the one picked, -1 while none is
	int render_template_combo(template_offer const &Offer, std::string const &Blocked);
	std::string connection_readout(connection::result const &Plan) const;
	editor_track::straight straight_near(glm::dvec3 const &Point, std::vector<TTrack const *> const &Exclude, double const Reach) const;
	editor_track::straight straight_through(glm::dvec3 const &Point, glm::dvec2 const &Direction) const;
	void record_connection(connection::kind const Kind, std::vector<TTrack *> const &Switches, connection::result const &Plan, editor_track::switch_template const &Shape) const;
	bool switch_room(TTrack &Track, double &Low, double &High, std::string &Reason) const;
	bool switch_vacate(TTrack const &Track, glm::dvec3 const &From, glm::dvec3 const &To, double const Shift, editor_track::straight const &Line, editor_track::edit &Edit, std::string &Status);
	bool move_switch_paths(TTrack &Track, std::vector<segment_data> const &Paths, std::string &Status);
	std::optional<switch_branch> take_switch_branch(TTrack const &Track, editor_track::joint const &Joint, std::size_t const Index) const;
	// for the path of the switch the line leaves, as it'll lie
	bool plan_switch_branch(segment_data const &Path, editor_track::joint const &Joint, switch_branch &Branch) const;
	void lay_switch_branch(switch_branch const &Branch, editor_track::edit &Edit) const;
	// of the joints off the main straight of the switch, the neighbours in Skip left out
	std::vector<switch_branch> switch_branches(TTrack const &Track, std::vector<editor_track::joint> const &Joints, std::vector<TTrack const *> const &Skip = {}) const;
	// for the paths the switch has now; the joints of the branches which can be laid are handled, the others are left out. Gives the notes for the user
	std::string plan_switch_branches(TTrack const &Track, std::vector<editor_track::joint> const &Joints, std::vector<switch_branch> &Branches, std::vector<bool> &Handled) const;
	// room on the straight of the whole double switch along the axis of its first switch, and the length of the set
	bool double_room(double_switch const &Set, double &Low, double &High, double &Length, std::string &Reason);
	bool move_double_paths(double_switch const &Set, std::vector<segment_data> const &Paths, std::string &Status);
	bool slide_switch(TTrack &Track, double const Distance);
	bool switch_slides(TTrack &Track) const;
	// a slip slid along its first track: the slip and the tracks its crossing track goes on into with their ends free move whole, the
	// straights of its first track before it and past it give and take the length. Low and High: how far back and on it may go
	struct slip_slide
	{
		glm::dvec3 shift{0.0}; // along its first track, a metre of it
		std::vector<TTrack *> movers;
		// where it joins the tracks of its first track before it and past it: at its ends A and B, or at the joints of its template
		// with the straights out to them going along; those tracks, the end of each there
		std::array<glm::dvec3, 2> joints{};
		std::array<std::pair<TTrack *, editor_track::point_ref>, 2> ends{};
		double low{0.0};
		double high{0.0};
	};
	bool slip_room(slip_set const &Slip, slip_slide &Slide, std::string &Reason) const;
	bool slide_slip(slip_set const &Slip, double const Distance);
	// a switch bent into a curve, its main track going on into a plain track at each end: the curve the three make, the two tracks,
	// where the switch starts on it; false for any other
	bool bent_switch_frame(TTrack &Track, std::vector<curve_sample> &Frame, std::array<TTrack *, 2> &Sides, double &Station) const;
	// the paths of the switch moved along the curve by the distance, shaped as the curve is there
	std::vector<segment_data> bent_switch_paths(TTrack const &Track, std::vector<curve_sample> const &Frame, double const Station, double const Distance) const;
	bool slide_bent_switch(TTrack &Track, double const Distance);
	// the switch bent into a curve given the paths, along the curve of the bent_switch_frame: the tracks next to it laid along it up to
	// its new ends, one taken up whole going, the curve past the diverging track following. Notes: for the user
	bool lay_bent_switch(TTrack &Track, std::vector<curve_sample> const &Frame, std::array<TTrack *, 2> const &Sides, std::vector<segment_data> const &Paths, std::string &Notes);
	// a track next to a bent switch, its Path, laid along the Frame up to the new End of the switch on it, the station there; Joint: the
	// old end of the switch it joined, Corner: the new one it's laid right up to
	static segment_data side_along_frame(segment_data const &Path, std::vector<curve_sample> const &Frame, double const End, glm::dvec3 const &Joint, glm::dvec3 const &Corner);
	void switch_slide_start(TTrack &Track);
	// switch_slide_start for a slip, false for a track which isn't one
	bool switch_slide_start_slip(TTrack const &Track);
	// switch_slide_start for a switch bent into a curve, false for any other
	bool switch_slide_start_bent(TTrack &Track);
	void switch_slide_update();
	void switch_slide_finish();
	void mouse_left_release();
	void draw_switch_slide() const;
	// reads the drives from the include bank once it changes; Wait: for the templates being looked for. true if it read them
	bool scan_switch_drives(bool Wait = true);
	std::string switch_name_for_new() const;
	bool place_switch_drive(TTrack &Switch);
	bool flip_switch_drive(TTrack &Switch);
	void advance_switch_name();
	void join_last_steps();
	bool render_switch_drive_choice();
	struct turntable_template
	{
		std::string file;
		std::string name;
		double length{0.0};
		int angles{0};
		int positions{0};
		int track{0};
		int yaw{0};
		std::vector<double> fixed;
	};
	struct turntable_fit
	{
		editor_track::snap_target end;
		double heading{0.0};
		double radius{0.0};
		double length{0.0};
		std::vector<segment_data> pieces;
		std::string issue;
		bool chosen{true};
		TTrack *replaces{nullptr};
	};
	struct turntable_tool
	{
		std::vector<turntable_template> templates;
		int revision{-1}; // of the include bank the templates were read from
		int chosen{-1};
		char name[64]{};
		bool placing{false};
		glm::dvec3 centre{0.0};
		double yaw{0.0};
		glm::vec2 pressed{0.f};
		editor_track::snap_target snap;
		TTrack *table{nullptr};
		scene::instance_handle include{0};
		int kind{-1};
		double length{0.0};
		double table_yaw{0.0};
		std::vector<double> exits;
		float reach{80.f};
		float radius{150.f};
		float stub{25.f};
		float step{0.f};
		std::vector<turntable_fit> fits;
		int hovered{-1};
		double heading{0.0};
		double exit_length{0.0};
		std::string status;
		std::string error;
	};
	turntable_tool m_turntable;
	bool scan_turntables(bool Wait = true);
	std::string turntable_name_for_new() const;
	bool turntable_read(TTrack *Table);
	std::vector<scene::instance_handle> turntable_includes(TTrack const &Table);
	glm::dvec3 turntable_centre() const;
	std::vector<double> turntable_lines() const;
	double turntable_snap(double const Heading) const;
	void turntable_finish_placement();
	void turntable_place(glm::dvec3 const &Centre, double const Yaw);
	bool turntable_fit_end(editor_track::snap_target const &End, turntable_fit &Fit) const;
	void turntable_find_fits();
	void turntable_build(std::vector<turntable_fit const *> const &Fits);
	bool turntable_exit_at(glm::dvec3 const &Ground, double &Heading, double &Length) const;
	void turntable_store_exits();
	void render_turntable_ui();
	void draw_turntable_overlay() const;
	extend_tool m_extend;
	lay_tool m_lay;
	alignment::limits laying_norms() const;
	void lay_after_history(bool const Undo);
	void render_lay_ui();
	void lay_click();
	void lay_finish(editor_track::snap_target const &End);
	void lay_cancel();
	void render_switch_ui();
	void render_switch_settings(switch_kind const Kind);
	void render_switch_timbers();
	void render_rail_profile_hint(TTrack const &Track);
	bool start_switch_placement();
	// start_switch_placement on a Track which isn't on a straight: into the curve it's in, pressed at the Ground
	bool start_curved_switch_placement(TTrack &Track, glm::dvec3 const &Ground);
	void finish_switch_placement();
	// finish_switch_placement for a connection of two switches
	void finish_connection_placement();
	// a switch with no track under it: a press on the ground starts it at the free end of a track within reach, none with no free end near
	editor_track::snap_target free_switch_end(glm::dvec3 const &Ground) const;
	bool free_switch_offered() const;
	void start_free_switch(glm::dvec3 const &Ground);
	// the start of the switch, the way its main track goes and the side of its diverging one, as the drag gives them so far
	void free_switch_pose(glm::dvec3 &Start, glm::dvec2 &Heading, int &Side, double &Grade) const;
	// a switch laid on from a free end, dragged on past its end: bent, its main track turning toward the cursor
	struct free_bend
	{
		double radius{0.0}; // of the main track
		int turn{1}; // the way the main track turns, 1 to the left
		int side{1}; // of the diverging track
		std::vector<curve_sample> frame; // the curve the switch is bent into
		std::vector<segment_data> paths;
		switch_curve::switch_report report;
		// a free end the cursor is at, which the main track leads into, the diverging one with Shift
		editor_track::snap_target target;
		bool diverging{false};
		editor_track::branch_link link;
		std::string problem; // why it can't lead into the target
	};
	// false where it isn't bent: near the switch, along the straight, or not from a free end
	bool free_switch_bend(free_bend &Bend) const;
	bool free_switch_bend_now(free_bend &Bend) const;
	// the track into the free end the switch bent from a free end, Bend, is dragged to
	void free_bend_link(free_bend &Bend) const;
	// laid on from an end of a switch: what the norms say of the change of the curvature from that switch to the tracks of this one
	std::vector<std::string> free_switch_exits(std::vector<segment_data> const &Paths) const;
	std::vector<segment_data> free_switch_paths() const;
	void insert_free_switch();
	std::vector<segment_data> switch_preview() const;
	std::vector<segment_data> switch_preview_now() const;
	// where a switch can't go, or doesn't go where it's dragged: what's in the way blinking in the view, each point told what's there,
	// why by the cursor; while it's dragged, and a while after it's refused
	struct switch_problem
	{
		std::string text;
		std::vector<segment_data> paths;
		std::vector<std::pair<glm::dvec3, std::string>> marks;
	};
	// what's at the end of the straight: a curve beginning, a switch, the track ending
	std::string straight_end_note(editor_track::straight const &Line, bool const Far) const;
	std::optional<switch_problem> switch_drag_problem() const;
	std::optional<switch_problem> straight_switch_problem(editor_track::straight const &Line, double const Length, double const Along, int const Direction) const;
	void flash_switch_problem(switch_problem Problem);
	void flash_switch_problem(std::string const &Text, std::vector<segment_data> Paths, std::vector<std::pair<glm::dvec3, std::string>> Marks = {});
	void draw_switch_problem(switch_problem const &Problem) const;
	switch_problem m_switch_flash;
	double m_switch_flash_until{-1.0};
	// a preview worked out once a frame for the drawing and the readouts which all show it: the frame and the state of the tool it
	// was worked out for
	template <typename Result> struct frame_memo
	{
		int frame{-1};
		std::vector<double> key;
		Result result{};
		bool fresh(std::vector<double> const &Key) const { return frame == ImGui::GetFrameCount() && key == Key; }
		void keep(std::vector<double> Key, Result const &Value)
		{
			frame = ImGui::GetFrameCount();
			key = std::move(Key);
			result = Value;
		}
	};
	std::vector<double> switch_memo_key() const;
	mutable frame_memo<std::vector<segment_data>> m_switch_memo;
	mutable frame_memo<std::pair<bool, free_bend>> m_bend_memo;
	mutable frame_memo<std::vector<segment_data>> m_extend_memo;
	void insert_switch(editor_track::straight const &Line, double const Along, int const Direction, int const Side);
	void switch_placed(std::vector<TTrack *> const &Created);
	static std::vector<curve_sample> sample_chain(editor_track::chain const &Chain);
	static curve_sample sample_at(std::vector<curve_sample> const &Frame, double const Station);
	// the curve a switch goes into as the norms look at it: its headings and its cant along it
	std::vector<switch_curve::track_point> frame_points(std::vector<curve_sample> const &Frame) const;
	// the way the switch being placed into a curve goes along it and the side its diverging track leaves to, from the drag
	void curved_switch_pose(int &Direction, int &Side) const;
	// the start of the switch being placed into a curve: where it was pressed, drawn to where the switch meets an end of a cant ramp or
	// keeps 6 m off it; What receives which
	double curved_switch_start(int const Direction, std::string *What = nullptr) const;
	// the switch made of the paths, laid into the curve at the station, as ST-T1-A6 sees it
	switch_curve::switch_report curved_switch_report(std::vector<segment_data> const &Paths, std::vector<curve_sample> const &Frame, double const Station, int const Direction) const;
	static double nearest_station(std::vector<curve_sample> const &Frame, glm::dvec3 const &Point);
	static std::vector<segment_data> curved_switch_paths(editor_track::switch_template const &Shape, std::vector<curve_sample> const &Frame, double const Station, int const Direction, int const Side);
	// the path of a switch template laid into the curve: x of the template across the main track to the Side, z along it from the
	// Station on, going Direction along the curve; past Bent along it going on straight as the curve heads there. The heights and the
	// cant are those of the line where the point is along it
	static segment_data frame_path(segment_data const &Local, std::vector<curve_sample> const &Frame, double const Station, int const Direction, int const Side, std::optional<double> const &Bent = std::nullopt);
	void insert_curved_switch(int const Direction, int const Side);
	// a crossover between two tracks running side by side along a curve (editormodeconnectioncurve.cpp)
	struct curve_mapping;
	// the steps of laying a crossover along a curve, with what each of them finds and makes
	struct curved_laying;
	// Beyond: the straights past the ends of the curve go with it up to that far, and a straight track that near the curve is taken
	// as its part
	static bool curve_of(TTrack &Track, editor_track::straight_tolerance const &Tolerance, double const Gauge, std::vector<curve_sample> &Frame, std::vector<TTrack *> &Tracks, double const Beyond = 0.0);
	static void cut_curve(std::vector<TTrack *> &Candidates, std::vector<curve_sample> const &Frame, glm::dvec3 const &First, glm::dvec3 const &Second, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed);
	void insert_curved_connection();
	// the single crossover the switch is a part of, along a curve or straights
	std::optional<curved_crossover> crossover_of(TTrack const &Switch) const;
	bool take_out_crossover(curved_crossover const &Crossover, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed, TTrack *&First, TTrack *&Second);
	bool slide_curved_connection(TTrack const &Switch, double const Distance, int const Shape = -1);
	// the template a crossover was laid of, -1 if it's not among the templates
	int crossover_template(switch_store::record const &Record);
	// a crossover laid again from the press on one track to the release by the other, as by the mouse. false if it doesn't fit
	// Names: of the switch at the press and of the one at the release, kept when a crossover is laid again
	bool lay_crossover(int const Shape, glm::dvec3 const &Press, glm::dvec3 const &Release, double const Reach = 3.0, std::vector<std::string> const &Names = {});
	// after the grade line of the open route went on its paths: the line on the other side of each crossover of the route
	// takes the same change of the heights where it runs beside, and its design with it. returns: how many lines followed
	std::size_t follow_crossovers(std::vector<editor_track::route_sample> const &Before);
	// its steps: whether the line across a crossover follows, the line following and its design
	bool crossover_followed(TTrack const &Other, editor_track::route const &Route, std::vector<TTrack const *> const &Done);
	bool follow_crossover(TTrack &Other, std::vector<editor_track::route_sample> const &Before, std::vector<editor_track::route_sample> const &After, profile::line const &Line);
	void follow_design(profile::line const &Designed, std::vector<double> const &Breaks, std::vector<editor_track::route_sample> const &Samples, std::vector<double> const &Changes);
	bool place_switch_on_straight(editor_track::straight const &Line, editor_track::switch_template const &Shape, TTrack const *Style, double const Along, int const Direction, int const Side, bool const Snap, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed, std::string const &Name = {});
	struct crossover_plan
	{
		editor_track::straight target;
		editor_track::switch_template shape;
		double along{0.0};
		int direction{1};
		int side{1};
		std::vector<segment_data> inserts; // a straight one, or an aligning curve between straights onto a track not parallel
		std::vector<segment_data> paths;
		connection::result connection; // as it's kept with the two switches
	};
	bool plan_crossover(crossover_plan &Plan) const;
	struct switch_link
	{
		editor_track::snap_target target;
		editor_track::branch_link link;
		double along{0.0};
		int forward{1};
		int side{1};
		double moved{0.0};
		std::vector<segment_data> paths;
		std::string reason;
	};
	bool plan_switch_link(switch_link &Plan) const;
	void insert_linked_switch(switch_link const &Plan);
	bool plan_branch_link(editor_track::branch_link &Link, std::string &Reason) const;
	void finish_branch_link(editor_track::branch_link const &Link);
	std::string link_problem(editor_track::snap_target const &Target, editor_track::branch_link const &Link) const;
	bool stretchable(editor_track::snap_target const &Target) const;
	std::vector<TTrack *> link_tracks(editor_track::snap_target const &Target, editor_track::branch_link const &Link, TTrack const &Style, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created);
	bool cut_straight(editor_track::straight const &Line, double const From, double const To, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed, TTrack **Style);
	// where a slip or a crossing goes for the cursor at Point on the straight Line: the crossing of Line with the rail track
	// nearest the cursor, or the double slip there to be replaced. With no track crossing there and Side given (1 to the left, -1 to
	// the right of Line), the slip brings the crossing track along at its own angle, turning off to that side ahead
	struct slip_plan
	{
		std::string problem;
		std::string note;
		bool lone{false}; // no track crosses Line by the cursor
		TTrack *replaces{nullptr};
		TTrack *other{nullptr}; // the crossing track, none where the slip brings it along
		glm::dvec2 crossing{0.0};
		glm::dvec2 first{0.0, 1.0};
		glm::dvec2 direction{0.0, 1.0};
		double angle{0.0};
		double alongfirst{0.0};
		double half{0.0};
		// where the slip brings its crossing track along: from the crossing to the joints of its template, past the frogs of the diamond,
		// the straights of both tracks laid out that far; the ends of the straights there, in the order of the points
		double reach{0.0};
		std::array<glm::dvec3, 4> ends{};
		editor_track::slip_curve curve;
		std::array<glm::dvec3, 4> points{};
	};
	bool plan_double_slip(editor_track::straight const &Line, glm::dvec3 const &Point, editor_track::switch_template const &Shape, slip_plan &Plan, int const Side = 0) const;
	// a slip is made for its skos: where the tracks cross at another one, the refusal, which tells the template made for theirs; empty
	// where it fits, and for a crossing
	std::string slip_skew(editor_track::switch_template const &Shape, double const Angle) const;
	// the cursor where it points at the straight, at the height of the straight there
	glm::dvec3 cursor_on_straight(editor_track::straight const &Line) const;
	bool insert_double_slip(editor_track::straight const &Line, glm::dvec3 const &Point, editor_track::switch_template const &Shape, int const Side = 0);
	// the side a slip dragged out on a straight with no track crossing it brings its crossing track to: the one through the quarter of
	// the cursor
	int lone_slip_side() const;
	bool replace_double_slip(TTrack const &Part, editor_track::switch_template const &Shape);
	// the drives of the four switches of a double slip replaced, Old, taken out, the switches of the New one given theirs
	void slip_drives_renewed(std::array<TTrack *, 4> const &Old, std::vector<TTrack *> const &New);
	// Bend: what a path of the slip laid out as on straights turns into where it's bent into a curve
	std::vector<TTrack *> build_double_slip(glm::dvec3 const &PointA, glm::dvec3 const &PointB, glm::dvec3 const &PointC, glm::dvec3 const &PointD, glm::dvec2 const &Crossing, double const Height, glm::dvec2 const &First, glm::dvec2 const &Second, double const Angle, editor_track::slip_curve const &Curve, TTrack const &Style, std::string const &Type = "Rkpd",
	                                        std::function<segment_data(segment_data const &)> const &Bend = {});
	// a slip switch bent into a curve of a side track (ST-T1-A6 11.4 pkt 4b): laid out on the tangent to the curve at its crossing, where
	// it was pressed, and bent onto the curve as a switch laid into it is, its crossing track coming along with its ends free. Direction
	// and Side as of a switch dragged into a curve: the crossing track goes ahead to that side, through the quarter of the cursor, and the
	// slip curve of a single slip lies on that side
	struct curved_slip
	{
		std::string problem;
		std::string snapped; // what the ends of the slip were drawn to
		double middle{0.0}; // the station of the crossing along the curve
		double half{0.0}; // from the crossing to the ends of the slip
		double reach{0.0}; // from the crossing to the joints of the template, the straights of both tracks laid out that far
		int direction{1};
		double angle{0.0};
		editor_track::slip_curve curve;
		// as the slip is laid out on the tangent, before it's bent: its crossing, its tracks going Direction along the curve, its ends
		glm::dvec2 crossing{0.0};
		glm::dvec2 first{0.0, 1.0};
		glm::dvec2 second{0.0, 1.0};
		std::array<glm::dvec3, 4> points{};
	};
	bool plan_curved_slip(editor_track::switch_template const &Shape, int const Direction, int const Side, curved_slip &Plan) const;
	// the path of the slip laid out on the tangent, bent onto the curve
	segment_data bent_slip_path(curved_slip const &Plan, segment_data const &Path) const;
	// the two tracks through the slip and its slip curves, each from end to end, bent: the first track, the slip curve leaving it at its
	// start, the second track, the other slip curve of a double slip
	std::vector<segment_data> curved_slip_lines(curved_slip const &Plan, std::string_view const Type) const;
	// the straights of both tracks from the ends of the slip out to the joints of the template, bent
	std::vector<segment_data> curved_slip_reaches(curved_slip const &Plan) const;
	switch_curve::slip_report curved_slip_report(curved_slip const &Plan, std::string const &Type) const;
	// the radii of the slip and its cant, as the view and the status tell them
	static std::vector<std::string> slip_report_lines(switch_curve::slip_report const &Report);
	void insert_curved_slip(int const Direction, int const Side);
	bool start_extend(editor_track::snap_target const &End);
	void finish_extend();
	void finish_crossover(crossover_plan const &Plan);
	// the same for the track shown while it's dragged: its grade line fitted on the thread of the fitter, the frame not waiting for it
	std::vector<segment_data> extend_preview() const;
	// the free end of the path as the start of a new track; false if it isn't free or has no direction
	static bool free_end(TTrack &Track, int const Path, bool const Atend, extend_tool &Tool);
	using track_layout = editor_track::layout;
	// the shape dragged from a free end, as the mode of laying has it: a straight in the one of the straights, an arc and a straight in
	// the one of a curve
	track_layout m_track_layout{track_layout::straight};
	double m_join_radius{0.0};
	int m_join_variant{0};
	TTrack const *m_join_track{nullptr};
	editor_track::point_ref m_join_point;
	mutable alignment::join_result m_join_last;
	mutable std::optional<alignment::lay_result> m_eased_last;
	bool track_wheel(double const Offset);
	// the parts of it: the connection of a switch dragged out, the speed laid for, the radius of the curve laid or joining
	void wheel_connection(double const Offset);
	std::string track_shaping(bool const Joined) const;
	double design_speed(TTrack const *From, TTrack const *To) const;
	// the tracks the laying under way goes on from and to
	std::pair<TTrack const *, TTrack const *> laying_ends() const;
	std::string design_speed_of(TTrack const *From, TTrack const *To) const;
	// what ST-T1-A6 says of the sudden changes of the cant deficiency right behind a switch the pieces laid come out of at Start (From),
	// or go into at End (To): of the end of its curve and the first sudden change of the pieces (9.11, tables 9.15, 9.16)
	std::vector<std::string> switch_exit_warnings(TTrack const *From, glm::dvec3 const &Start, TTrack const *To, glm::dvec3 const &End, std::vector<segment_data> const &Pieces) const;
	bool extend_snap(editor_track::snap_target &Target) const;
	void update_build_tools();
	// the parts of it: the track and the point the cursor joins, the end of a track dragged on, the switch dragged out, the line being
	// laid; the cursor taken where it is for the line, or kept where its fits were asked for; the line worked out through the points
	void join_target(editor_track::snap_target const &Target);
	void update_switch_placing();
	void draw_build_overlay() const;
	// the lines told of the switch or the slip being bent into a curve, the Preview of its paths: what its start was drawn to, the radii,
	// the cant, the notes and the warnings; the Station they're told at, the middle of a slip or the start of a switch
	std::vector<std::pair<std::string, ImU32>> curved_switch_lines(std::vector<segment_data> const &Preview, double &Station) const;
	// the joint before the blades of the switch being placed or the one selected and the tips of its blades
	void draw_blade_tips() const;
	// modes of the track editor, one at a time: path is the plain selection
	using track_tab = editor_track_tab;
	struct track_mode
	{
		char const *label;
		char const *key;
		track_tab tab;
		char const *tooltip;
		editor_icons::icon picture;
		int family; // 0 builds the track, 1 equips it, 2 checks it
	};
	static std::array<track_mode, 12> const &track_modes();
	bool track_plan_tab() const { return m_track_tab == track_tab::straights || m_track_tab == track_tab::route; }
	track_tab plan_tab_for(TTrack const *Track) const;
	// the objects along the track: a tab for each tool
	int m_lineside_tab{0};
	void render_lineside_ui();
	void arm_switch();
	// the window of the switch templates: the catalogue and the own ones, made of straights, arcs and transition curves
	struct switch_designer
	{
		bool open{false};
		bool loaded{false};
		std::vector<switch_design::design> catalogue;
		std::vector<switch_design::design> own;
		bool from_catalogue{true};
		int selected{-1};
		bool dirty{false};
		bool stretched{true};
		double quick_radius{300.0};
		double quick_ratio{9.0};
		double quick_a{0.0};
		double quick_b{0.0};
		int fit_kind{3}; // of switch_design::tangent_fit
		double quick_first{0.0}; // the radius of the first arc of an arc, a straight and an arc, 0: the R
		bool picking{false}; // the switch clicked off the view, the orthophoto under it
		std::vector<glm::dvec3> clicks;
		bool on_view{true}; // the template shown in the view where its first points were clicked
		switch_design::design measured; // what the points clicked so far make
		bool measured_ready{false};
		std::string measured_text;
		int pointed{-1}; // the value under the mouse, its dimension shown on the drawing
		int pointed_test{-1};
		double blade_length{5.0};
		double blade_gap{0.15};
		std::string status;
	};
	switch_designer m_switch_designer;
	void switch_designs_load();
	// the list of the switch tool built again with the own templates, keeping the armed one or arming the one whose label starts with Arm
	void switch_templates_refresh(std::string const &Arm = {});
	void render_switch_designer();
	void render_switch_designer_list();
	void render_switch_designer_clicking();
	void render_switch_designer_edit();
	bool render_switch_designer_quick(switch_design::design &Entry, int &Pointing);
	void render_switch_designer_actions(switch_design::design const &Entry, bool const Editable, bool const Buildable);
	void switch_designer_click(glm::dvec3 const &Point);
	void switch_designer_measure();
	void switch_designer_finish();
	void draw_switch_designer_clicks() const;
	void draw_cursor_ring(glm::dvec3 const &Point) const;
	static std::string grade_line_key(TTrack const &Track);
	static std::string grade_line_key(std::string const &Name, TTrack const &Track);
	bool m_track_window_open{false};
	// the inspector is pinned to the right edge of the screen, the profile strip to the bottom one, both resized from the inner edge
	void render_track_inspector();
	// the track tools in the tool options window, and the selected path in the inspector
	void render_track_tool_options();
	void render_track_selection();
	void show_track_tab(track_tab const Wanted) override;
	// the route tool opened on a Track not in its chain: the line read from it, or the notice why it can't be
	void open_route_on(TTrack &Track);
	bool track_analysis_tab() const { return m_track_tab == track_tab::profile || m_track_tab == track_tab::speed || m_track_tab == track_tab::joints || m_track_tab == track_tab::infra; }
	bool track_shortcut(int const Key);
	bool track_busy() const; // something going on in the mode, which Esc drops before it leaves the mode
	std::string track_mode_name() const;
	void render_turnout_ui();
	// the drive of the Switch, under its name where it's Named, a switch of a double one: the drive standing by it to change, the events
	// switching it, or a drive to put by it
	void render_switch_drive(TTrack &Switch, bool const Named);
	// the drive standing by the Switch, of the Family and in the Variant for its side: another one put in its place, or turned to the
	// other side
	void render_standing_drive(TTrack &Switch, int const Family, int const Variant);
	// the drives to pick in place of the drive of the Switch, of the Family now
	void render_drive_combo(TTrack const &Switch, int const Family);
	// the Track, a switch bent into a curve, slid along it by the distance typed, the tracks next to it keeping a metre
	void render_bent_switch_slide(TTrack &Track);
	struct track_name_edit
	{
		TTrack const *track{nullptr};
		std::string synced;
		char text[128]{};
		editor_track::name_uses uses;
		std::string status;
	};
	track_name_edit m_track_name;
	void render_track_name(TTrack &Track);
	void render_straight_ui();
	track_tab m_track_tab{track_tab::path};
	void delete_selected_track();
	// the double switch Set deleted, its main track left in its place unless Shift asks for all of it
	void delete_double_switch(double_switch const &Set);
	// the Slip deleted whole, its two tracks left crossing in its place unless Shift asks for all of it
	void delete_slip(slip_set const &Slip);
	// the switch thrown over to its other path in the editor, its blades going over as in the simulation; the scenery keeps no state of it
	void throw_switch(TTrack &Switch);
	// the blades of the switches thrown in the editor moved on by the time of the frame: the time of the simulation stands still in it
	void move_thrown_switches();
	std::vector<TTrack *> m_thrown_switches;
	std::string designs_trimmed(std::vector<TTrack *> const &Removed) const;
	void start_straight_gesture(int const Tool);
	void toggle_straight_set(TTrack &Track);
	struct key_hint
	{
		char const *key;
		std::string action;
		char const *raw{nullptr}; // the key before it's translated, its gesture drawn from it
	};
	std::vector<key_hint> track_key_hints(bool const All) const;
	// the Hints of the keys of a tool under way: laying, a detour of a straight, a track extended, a switch placed; false where none is
	bool tool_key_hints(std::vector<key_hint> &Hints) const;
	// the Hints of the keys while laying, by the points clicked so far
	void lay_key_hints(std::vector<key_hint> &Hints) const;
	// the Hints of the keys while a switch is dragged out on a straight: a connection recognised, one dragged onto the other track, or a
	// switch alone
	void placing_key_hints(std::vector<key_hint> &Hints) const;
	// the Hints of the keys of the lay, the switch and the profile tabs, with the gizmo of the Selected track told of; false in another tab
	bool tab_key_hints(std::vector<key_hint> &Hints, TTrack const *Selected, bool &Gizmo) const;
	// the Hints of the keys of the switch tab by the kind of switch, the gizmo with a track Selected: whether the gizmo is told of
	bool turnout_key_hints(std::vector<key_hint> &Hints, TTrack const *Selected) const;
	// the Hints of the keys of the tools of the track window; false with the window closed or another tool in it
	bool window_key_hints(std::vector<key_hint> &Hints) const;
	// the Hints of the keys of the signalling: a signal moved or placed, or the stations
	void srk_key_hints(std::vector<key_hint> &Hints) const;
	// the Hints of the keys of the plan and the route tabs; false in another tab
	bool plan_key_hints(std::vector<key_hint> &Hints) const;
	// the Hints of the keys of the plan tab, by the tool and the straight dragged
	void straights_key_hints(std::vector<key_hint> &Hints) const;
	// the Hints of the keys of the Selected track, its point or a set of tracks: whether the gizmo is told of
	bool selection_key_hints(std::vector<key_hint> &Hints, TTrack const *Selected) const;
	// what the gizmo does to the point picked of the Selected track, by its operation
	char const *point_gizmo_action(TTrack const *Selected) const;
	// what the gizmo does to the whole Selected track, by its operation
	char const *track_gizmo_action(TTrack const *Selected) const;
	void render_track_guide();
	std::string track_readout() const;
	// track_readout of a track extended from its end: its length and radius, what it joins, the warnings
	std::string extend_readout() const;
	// track_readout while laying: the shape laid, what it joins, the warnings and the notices
	std::string lay_readout() const;
	// the shape laid told by the cursor: the start, the curves of the plan or the pieces of the preview
	std::string lay_shape_text() const;
	// the curve of the plan laid the cursor makes now, the points traced, the speed of the design and the grades
	std::string lay_plan_text() const;
	// the pieces of the preview laid: their length and radius, the speed of the design and the grades
	std::string lay_pieces_text() const;
	// track_readout of the tools of the plan placed: the turn of a straight broken, the shift of a detour
	std::string straight_tool_readout() const;
	// track_readout of a straight dragged: the curve fitted, its shift and the parallel offered, or its length and azimuth
	std::string straight_drag_readout() const;
	// track_readout of the gizmo dragging the Track or its point
	std::string gizmo_readout(TTrack const &Track) const;
	// track_readout of the gizmo dragging a control vector of the Track
	std::string control_readout(TTrack const &Track) const;
	void draw_track_hints();
	// the readout of the change in progress in a box by the cursor, with the number typed
	void draw_track_readout() const;
	// a scenery with no tracks yet: in the middle of the view, how the first one is laid
	void draw_empty_scenery_hint() const;
	// path under the cursor, outlined before a click so it's known which one the click takes
	track_hover m_hover;
	track_hover track_under_cursor(TTrack const *Only = nullptr, float const Pixels = 0.0f) const;
	track_hover split_spot() const;
	void update_track_hover();
	void draw_track_lines() const;
	// what the colours of the tracks in the view tell
	enum class track_colouring { plain, radius, speed, grade };
	// the colour draw_track_lines gives the path: by its radius, or by the speed along it, else Plain
	static ImU32 track_line_colour(track_colouring const Colouring, segment_data const &Path, double const Velocity, ImU32 const Plain);
	track_colouring m_track_colouring{track_colouring::plain};
	void draw_track_legend() const;
	// the radii and lengths of the curves and the names and types of the switches written by them, from above, as on a plan
	bool m_track_labels{true};
	void draw_track_labels() const;
	void draw_track_hover() const;
	struct track_intent
	{
		enum class kind { nothing, gizmo, select, point, box, straight_set, straight_break, detour, straight_tool, straight_handle, route_vertex, route_grip, switch_at, switch_free, lay, lay_end, sweep, turntable_place, turntable_exit, turntable_fit, spread, extend_end, signal_place, signal_move, switch_slide, pick_style, grade_line_pick, fouling, measure, route_trace, kilometrage, srk_boundary };
		kind what{kind::nothing};
		TTrack *track{nullptr};
		int path{0};
		editor_track::point_ref point;
		int index{-1};
		glm::dvec3 position{0.0};
		editor_track::snap_target snap;
		std::string action;
	};
	track_intent m_intent;
	// the measure between two points (M): on the ground, at a free end or on the axis of the track under the cursor
	struct tape_state
	{
		bool active{false};
		std::vector<glm::dvec3> points;
		glm::dvec3 mouse{0.0};
	};
	tape_state m_tape;
	void tape_update();
	std::string tape_readout() const;
	void draw_tape() const;
	track_intent track_intent_at(int const Mods);
	bool track_gesture_on() const;
	void update_track_intent();
	void track_press(track_intent const &Intent);
	void turntable_press(track_intent const &Intent);
	track_intent turntable_intent(glm::dvec3 const &Ground, bool const Shift);
	void draw_track_intent() const;
	editor_track::snap_target snap_free_end(glm::dvec3 const &Near, glm::vec2 const &Screen, TTrack const *Self, int const Category, std::vector<TTrack const *> const &Exclude) const;
	void draw_free_ends(TTrack const *Self, int const Category, std::vector<TTrack const *> const &Exclude) const;
	// the free ends of the rail tracks around the camera, looked for again after an edit or once the camera leaves the place
	struct free_end_cache
	{
		int edits{-1};
		glm::dvec3 centre{0.0};
		double reach{0.0};
		std::vector<glm::dvec3> points;
	};
	mutable free_end_cache m_free_end_cache;
	std::vector<glm::dvec3> const &free_end_points(glm::dvec3 const &Centre, double const Reach) const;
	struct point_drag
	{
		bool active{false};
		bool moved{false};
		glm::vec2 from{0.f};
		double height{0.0};
		std::size_t step{0};
	};
	point_drag m_point_drag;
	struct handle_drag
	{
		bool active{false};
		bool moved{false};
		glm::vec2 from{0.f};
		double height{0.0};
		glm::dvec3 offset{0.0};
		glm::dvec3 position{0.0};
	};
	handle_drag m_handle_drag;
	void handle_drag_start(glm::dvec3 const &Anchor);
	// Ground: the handle goes onto the ground, at the height of the ballast of the track, unless Shift is held
	bool handle_drag_step(glm::dvec3 &Position, TTrack const *Ground = nullptr);
	void point_drag_start(editor_track::point_ref const &Point);
	void point_drag_update();
	// the drag started once the cursor went far enough, false while it hasn't or the track can't be changed; where the point goes
	bool point_drag_begin(TTrack &Track, glm::vec2 const &Mouse);
	void point_drag_place(TTrack &Track, glm::vec2 const &Mouse, glm::dvec3 &Position);
	void point_drag_finish();
	bool route_hit(int &Vertex, int &Grip) const;
	int straight_handle_hit();
	bool switch_reach(glm::dvec3 const &Ground, glm::dvec3 &Point);
	glm::dvec3 cursor_ground() const;
	glm::dvec3 cursor_level(double const Height) const;
	glm::dvec3 cursor_terrain(double const Height) const;
	double bed_height(glm::dvec3 const &Point, glm::dvec2 const &Direction, TTrack const *Style) const;
	double nearby_ground(glm::dvec3 const &Point) const;
	static std::pair<double, double> trackbed(TTrack const *Style);
	std::vector<double> bed_heights(std::vector<glm::dvec3> const &Points, std::vector<glm::dvec2> const &Directions, TTrack const *Style) const;
	void follow_ground(std::vector<segment_data> &Pieces, double const Height0, double const Grade0, double const Height1, double const Grade1, TTrack const *Style) const;
	ground_follow::grading laying_grading(TTrack const *Style) const;
	// the grade lines of the tracks shown while they're laid are fitted on a thread of their own, not to hold up the frames
	mutable ground_follow::fitter m_fitter;
	mutable bool m_follow_preview{false};
	glm::dvec3 lay_bed(glm::dvec3 const &Point) const;
	std::optional<glm::dvec3> m_cursor_override;
	bool sweep_grabs(glm::dvec3 const &Point);
	// menu of the path a click of the right mouse button lands on
	struct track_context
	{
		track_hover target;
		std::chrono::steady_clock::time_point pressed;
		glm::vec3 angle{0.f};
		glm::vec2 mouse{0.f};
		bool armed{false};
		bool pending{false};
	};
	track_context m_track_context;
	void track_context_press();
	void track_context_release();
	void render_track_context();
	void split_track_at(TTrack &Track, glm::dvec3 const &Point);
	void focus_track(TTrack &Track, int const Path, glm::dvec3 const &Point) override;
	// the camera flies over Duration seconds to the place Distance back from the target along Look, looking along it
	void fly_to(glm::dvec3 const &Target, glm::dvec3 const &Look, double const Distance, double const Duration = 0.6) override;
	// paths found by their names
	struct track_search
	{
		char text[64]{};
		std::string searched;
		std::vector<TTrack *> found;
		std::size_t total{0};
		bool focus{false};
	};
	track_search m_track_search;
	void render_track_search();
	// paths selected together: Shift+LMB adds or takes one out, Shift+drag adds those in the box
	std::vector<TTrack *> m_track_set;
	struct track_box
	{
		bool active{false};
		bool replace{false}; // the tracks in the box become the selection, else they join it
		glm::vec2 from{0.f};
		glm::vec2 to{0.f};
		TTrack *hovered{nullptr};
	};
	track_box m_track_box;
	glm::mat4 m_track_set_gizmo{1.0f};
	bool m_track_set_using{false};
	glm::dvec3 m_track_set_pivot{0.0};
	std::vector<std::pair<TTrack *, std::vector<editor_track::joint>>> m_track_set_joints;
	bool in_track_set(TTrack const *Track) const;
	// the track is selected: alone or in the set
	bool track_selected(TTrack const *Track) const;
	// a click in the view: the track under the cursor becomes the selection, or nothing
	void track_click_select(TTrack *Hovered);
	bool track_set_begin(bool const Rotate);
	void track_set_turn(double const Angle);
	void track_set_move(glm::dvec3 const &Offset);
	void track_set_end();
	glm::dvec3 track_set_centre() const;
	void track_set_toggle(TTrack &Track);
	void track_set_prune();
	void start_track_box(bool const Replace = false);
	void finish_track_box();
	void draw_track_set() const;
	void render_track_set_ui();
	void render_track_set_gizmo();
	void track_set_delete();
	struct track_spread
	{
		int stop{0};
		bool same_rails{false};
		bool same_bed{false};
		bool same_speed{false};
		float reach{0.f};
		TTrack const *from{nullptr};
		int signature{-1};
		std::vector<TTrack *> preview;
		double length{0.0};
	};
	mutable track_spread m_spread;
	std::vector<TTrack *> track_spread_from(TTrack &Start) const;
	std::vector<TTrack *> const &track_spread_preview(TTrack &Start) const;
	void track_spread_apply(TTrack &Start);
	void track_set_grow();
	void render_track_spread_ui();
	void draw_track_spread() const;
	// the properties of the selected tracks in the inspector, one sheet for one track and for many; see editormodetrackprops.cpp
	// the set when there's one, the selected track otherwise
	std::vector<TTrack *> selected_tracks();
	void select_tracks(std::vector<TTrack *> const &Tracks);
	// changes each track which can be changed, in one step of the undo; false from Change: the track stays as it was
	std::size_t tracks_apply(std::vector<TTrack *> const &Tracks, std::function<bool(TTrack &)> const &Change, bool const Rebuild, char const *What);
	// the model of the timbers, with the fastenings where it has them, set on all the switches and crossings of the scenery, in one
	// step of the undo; empty: the one of eu07.ini. returns: the tracks changed
	std::size_t set_switch_timbers(std::string const &Model);
	int track_property_index(std::string_view const Key) const;
	std::size_t track_property_apply(int const Property, std::vector<TTrack *> const &Tracks, std::string const &Text);
	void track_field_finish();
	void track_field_settle();
	void track_textures_scan();
	// a texture in use, offered first in the picker
	struct texture_use
	{
		material_handle material{null_handle}; // null: its image is read by the name
		std::string name;
		int count{0}; // shown on its tile, 0: nothing
		std::string note; // what's said of it above the tiles
	};
	// what the picker of a texture offers besides every texture of the folder
	struct texture_offer
	{
		char const *title; // the heading of the ones in use
		std::function<std::vector<texture_use>()> used; // gathered when the picker opens
		char const *none_entry; // the entry for no texture, nullptr: there's none
		char const *none_name; // the name that entry gives
	};
	// every texture and material under the folder of the textures, read on the first use of the picker
	void textures_index();
	// the picker of a texture, in an open combo. returns: true when one is chosen, its name in Chosen
	bool texture_picker(texture_offer const &Offer, std::string const &Current, bool const Mixed, std::string &Chosen);
	// the large image of the picker: the tile under the mouse, what Enter would choose or the current texture, with its notes
	void texture_preview(texture_offer const &Offer, std::string const &Current, std::string const &First, bool const Searching, bool const Mixed);
	// the notes next to the large image of a texture: where it's used, whether it's set now, whether Enter chooses it
	void texture_notes(std::string const &Shown, std::string const &First, std::string const &Current, bool const Searching, bool const Mixed) const;
	// the tiles of the picker, those in use and those of the folder. returns: true when one is clicked, its name in Chosen
	bool texture_tiles(texture_offer const &Offer, std::vector<texture_use const *> const &Used, std::vector<std::string const *> const &Files, std::string const &Current, bool const Mixed, std::string &Chosen);
	// one tile of the picker. returns: true when it's clicked
	bool texture_tile(std::string const &Name, int const Count, float const Tile, float const Label, bool const Current);
	// the same, of a texture of the rails (Kind 0) or of the trackbed (1)
	bool track_texture_picker(int const Kind, std::string const &Current, bool const Mixed, std::string &Chosen);
	// image of the texture of the name, 0 while there's none
	std::uint64_t texture_image(std::vector<texture_use> const &Used, std::string const &Name);
	std::uint64_t track_texture_image(int const Kind, std::string const &Name);
	// a field of a texture: its image and its name, a click opens the picker, a material of the texture browser can be dropped on it.
	// returns: true when another one was chosen, in Value
	bool texture_field(char const *Label, float const Width, std::string &Value, texture_offer const &Offer);
	// the same, for a name kept in a buffer
	bool texture_field(char const *Label, float const Width, char *Buffer, std::size_t const Size, texture_offer const &Offer);
	// the textures of the roads and junctions of the scenery
	std::vector<texture_use> road_textures_used() const;
	// the textures of the palette and of the water of the terrain; None: the entry for no texture of its own, nullptr: there's none
	texture_offer terrain_texture_offer(char const *None);
	void render_track_properties(std::vector<TTrack *> const &Tracks);
	// the values of the properties over the tracks gathered again when the selection or the tracks changed
	void property_values_gather(std::vector<TTrack *> const &Tracks, std::size_t const Signature);
	// the notes on the values of the properties made again when the selection or the tracks changed
	void property_notes(std::size_t const Signature);
	// the tooltip of a row of the properties
	void property_row_help(std::size_t const Index, std::size_t const Selected) const;
	// the label of a row of the properties, marked when the selected tracks differ in it
	void property_row_label(std::size_t const Index, std::size_t const Selected) const;
	// the rows of the properties by their kind: a text, a choice, a tick, a texture
	void property_text_row(std::size_t const Index, std::vector<TTrack *> const &Tracks);
	void property_choice_row(std::size_t const Index, std::vector<TTrack *> const &Tracks);
	void property_check_row(std::size_t const Index, std::vector<TTrack *> const &Tracks);
	void property_texture_row(std::size_t const Index, std::vector<TTrack *> const &Tracks);
	// whether any property of the part is shown, More: of those under "more"
	bool property_rows_any(int const Part, bool const More) const;
	// the rows of the properties of the part
	void property_rows(int const Part, bool const More, std::vector<TTrack *> const &Tracks);
	// the notes under a row of the properties
	void property_row_notes(std::size_t const Index) const;
	// the rows under "more" of the part, in a tree node of the id
	void property_more_rows(char const *Id, int const Part, std::vector<TTrack *> const &Tracks);
	// the rows of the look of the track
	void property_look_rows(std::vector<TTrack *> const &Tracks);
	void render_track_set_header(std::vector<TTrack *> const &Tracks);
	// a row of the list of the selected tracks: Removed or Alone set to the track when it's taken out or kept alone
	void render_selected_track_row(TTrack *Track, int const Index, float const Column, TTrack *&Removed, TTrack *&Alone);
	void render_track_select_tools(std::vector<TTrack *> const &Tracks);
	// the popup selecting the tracks of the scenery alike the selected ones
	void render_similar_popup(std::vector<TTrack *> const &Tracks);
	void render_track_style(std::vector<TTrack *> const &Tracks);
	// what the track taken changes on the selected tracks, each with a tick, and the button setting it
	void render_style_taken(std::vector<TTrack *> const &Tracks);
	// a line of what the track taken changes: its tick and how it changes. returns: true when it's ticked
	bool render_style_line(int const Property, std::string const &Now, std::string const &Taken, std::size_t const Differ, std::size_t const Selected);
	void copy_track_style(TTrack const &Track);
	std::size_t paste_track_style(std::vector<TTrack *> const &Tracks);
	// which groups of the properties are pasted or put on the tracks by a preset, in the order of the groups of editormodetrackprops.cpp
	using track_groups = std::array<bool, 11>;
	// the values of the properties of the chosen groups put on the tracks, in one step of the undo. returns: the number of the
	// tracks changed, none when no value is of the chosen groups
	std::optional<std::size_t> apply_track_values(std::vector<std::pair<std::string, std::string>> const &Values, track_groups const &Groups, std::vector<TTrack *> const &Tracks, char const *What);
	// presets of the properties of the tracks (editorTrackPresets), made from a track and put on the selected ones
	void render_track_presets(std::vector<TTrack *> const &Tracks);
	// the combo choosing the preset
	void render_preset_combo();
	// the buttons applying the preset and making a new one from the track
	void render_preset_new(std::vector<TTrack *> const &Tracks, bool const None, float const Width);
	// the name of the preset of the id, renamed by Enter or the button
	void render_preset_name(int const Id);
	// the ticks of what the preset of the id overwrites
	void render_preset_groups(int const Id);
	// the buttons updating the preset of the id from the track and deleting it
	void render_preset_update(std::vector<TTrack *> const &Tracks, int const Id, float const Width);
	void render_track_presets_menu();
	std::size_t apply_track_preset(int const Id, std::vector<TTrack *> const &Tracks);
	std::vector<TTrack *> similar_tracks(int const Property, std::vector<TTrack *> const &Tracks) const;
	// values of the properties over the selected tracks, each with the number of the tracks having it
	struct track_values
	{
		std::size_t signature{0};
		int generation{0};
		int generation_seen{-1};
		std::size_t history{0};
		int frame{-100};
		std::vector<std::vector<std::pair<std::string, int>>> values;
		std::vector<std::size_t> applies;
		std::size_t unsaved{0}; // of the tracks, changed since the file was written
	};
	track_values m_track_values;
	// how the last value given in the sheet went: on how many tracks, and why not on the others
	struct track_outcome
	{
		int property{-1};
		std::string text;
		bool refused{false}; // some tracks couldn't take it
		std::size_t signature{0}; // of the tracks it was given to
		std::size_t history{0};
		double time{0.0};
	};
	track_outcome m_track_outcome;
	// the lines under the rows of the sheet
	struct track_note
	{
		enum class kind
		{
			typing,
			done,
			refused
		};
		int property{-1};
		std::string text;
		kind type{kind::done};
	};
	std::vector<track_note> m_track_notes;
	// the field being typed in, with the tracks it was started on
	struct track_field
	{
		int property{-1};
		std::vector<TTrack *> tracks;
		std::array<char, 256> text{};
		std::string initial;
		int frame{0};
	};
	track_field m_track_field;
	std::array<std::vector<texture_use>, 2> m_track_textures; // of the rails, of the trackbed
	std::vector<texture_use> m_texture_used; // the ones in use the open picker offers
	std::array<char, 128> m_texture_filter{};
	struct texture_index_entry
	{
		std::string name; // as the materials are named: relative to the folder of the textures, without the extension
		std::string lowered;
		bool material{false}; // there's a .mat file of the name
	};
	std::vector<texture_index_entry> m_texture_index; // sorted by the name
	bool m_texture_indexed{false};
	std::string m_texture_hovered; // the tile under the mouse in the picker, shown large above them
	// properties taken from a track, to be set on the selected ones
	struct track_style
	{
		std::string from;
		std::vector<std::pair<std::string, std::string>> values;
		// ticked at first by their groups: speed, environment, overhead line, track circuits, rails, trackbed, its shape, sleepers, ride and
		// sound, frogs, sleeper placement
		track_groups groups{false, false, false, false, true, true, true, true, false, true, true};
		std::map<std::string, bool, std::less<>> ticks; // changed by hand, by the keys of the properties
		bool picking{false}; // the next click on a track in the view takes its properties
		bool reveal{false};  // its section of the inspector opens, a track having been taken
	};
	track_style m_track_style;
	// a property of the track taken which the selected ones have otherwise
	struct track_style_line
	{
		int property{-1};
		std::string now;       // as the selected tracks have it, or how many values they have
		std::string taken;     // as the track taken has it
		std::size_t differ{0}; // of the selected tracks, the ones it changes
	};
	std::vector<track_style_line> track_style_lines(std::vector<TTrack *> const &Tracks) const;
	bool track_style_takes(std::string const &Key) const;
	struct track_presets_panel
	{
		int selected{0}; // id of the preset chosen in the inspector
		int nameid{0}; // the preset whose name is in the field
		std::array<char, 128> name{};
	};
	track_presets_panel m_track_presets;
	std::vector<std::vector<TTrack *>> m_track_similar;
	TTrack const *m_track_list_hover{nullptr}; // in the list of the selected tracks
	int m_track_list_hover_frame{-1};
	using template_item = ::template_item;
	using standing_template = ::standing_template;
	// includes of the templates whose file names the test accepts, with the location in p2..p4 and the rotation in p5
	std::vector<standing_template> standing_templates(std::function<bool(std::string const &)> const &Accept) const override;
	struct switch_drive
	{
		standing_template standing;
		int family{-1};
		int index{0};
	};
	std::optional<switch_drive> find_switch_drive(TTrack const &Switch, bool Wait = true);
	// the drives standing in the scenery by the switch each is set to, read from the files once a frame for all the switches
	std::map<std::string, switch_drive, std::less<>> const &switch_drives_standing(bool Wait = true);
	struct drives_standing
	{
		std::map<std::string, switch_drive, std::less<>> by_switch;
		int frame{-1};
		int revision{-1}; // of the drives scanned
		std::size_t instances{0};
	};
	std::optional<drives_standing> m_drives_standing; // emptied when an include changes
	std::string missing_drive_variant(int const Family, bool const Rightswitch, bool const Driveright) const;
	bool replace_switch_drive(TTrack const &Switch, int Family, bool Flip);
	void apply_switch_drive_choice(TTrack &Switch);
	void switch_drive_captured(TTrack const &Track);
	void switch_drives_committed(std::span<TTrack *const> const Tracks);
	// the drives of the switches deleted go with them, in the step of the undo history the deletion made last
	void drives_deleted(std::vector<TTrack *> const &Switches);
	// the drives of all the switches of the scenery set at the tips of their blades, the ones the earlier versions of the editor set at the
	// joint before them too: once, as the scenery is opened, with no step of the undo history; how many moved
	std::size_t drives_to_blade_tips();
	bool m_drives_checked{false};
	scene::layer_handle output_layer(std::string const &Part) override;
	// places them in the active layer or the given one, as one step of the undo. returns: how many were placed
	std::size_t place_templates(std::vector<template_item> const &Items, std::string &Error, scene::layer_handle Layer = scene::layer_handle{}) override;
	// moves them, as one step of the undo
	std::size_t move_templates(std::vector<std::pair<standing_template, template_item>> const &Moves) override;
	// takes them out, as one step of the undo
	std::size_t remove_templates(std::vector<scene::instance_handle> const &Instances) override;
	bool standing_signals(std::vector<standing_signal> &Signals, bool const Wait) override;
	std::size_t signal_batch(signal_changes const &Changes, std::vector<std::pair<TTrack *, editor_track::state>> States, std::vector<TTrack *> Created, std::string &Error) override;
	// the parts of a batch of signal changes, on its state defined where it's used: the signals placed, renamed, given other values or
	// another template, taken away; the step of the batch put into the history
	struct signal_batch_state;
	void signal_batch_place(signal_changes const &Changes, std::vector<template_item> &Items, signal_batch_state &Batch, std::string &Error);
	void signal_batch_rename(signal_rename const &Rename, bool const Temporary, signal_batch_state &Batch);
	void signal_batch_values(signal_changes const &Changes, signal_batch_state &Batch);
	void signal_batch_file(scene::instance_handle const Instance, std::string const &Wanted, signal_batch_state &Batch);
	std::vector<scene::instance_handle> signal_batch_remove(signal_changes const &Changes, std::unordered_set<TTrack const *> const &Created, signal_batch_state &Batch);
	void signal_batch_commit(signal_batch_state &Batch, std::vector<TTrack *> Created, std::vector<scene::instance_handle> Instances);
	std::string signal_templates_missing(std::vector<signal_order> const &Orders) override;
	bool signal_heads(std::vector<signal_head> &Heads, bool const Wait) override;
	std::string signal_mount(std::string const &Kind, std::string const &File) override;
	std::size_t signal_reattach(std::vector<scene::instance_handle> const &Instances) override;
	void select_signal(scene::instance_handle const Instance) override;
	void render_signal_templates() override;

	struct signal_template
	{
		std::string file;
		std::string name;
		std::string description;
		std::string kind;
		std::string mount;
		std::string lamps;
		std::string lean;
		std::string read;
		int parameters{0};
		bool plate{false};
		bool linked{false};
		std::vector<int> speeds;
		int aspects{0}; // of a block signal, 3 or 4
		int arms{0}; // of a semaphore, 1 or 2, from its description
		// the aspects its description lists, by name (Sr1, Od2...) with the event giving each; empty for a template not described so
		std::vector<std::pair<std::string, std::string>> aspect_events;
		// of a warning disc: its event for each speed class of the signal after it, stop, vmax, 100, 60, 40
		std::vector<std::pair<std::string, std::string>> repeats;
	};
	struct signal_spot
	{
		TTrack *track{nullptr};
		int path{0};
		glm::dvec3 axis{0.0};
		glm::dvec2 travel{0.0, 1.0};
		int side{1};
		int list{2};
		double offset{0.0};
		glm::dvec3 position{0.0};
		double yaw{0.0};
	};
	struct signal_standing
	{
		standing_template include;
		int kind{-1};
		std::string name;
		std::string read;
		TTrack *track{nullptr};
		int list{0};
	};
	struct signal_tool
	{
		std::vector<signal_template> templates;
		int revision{-1};
		int kind{0};
		std::array<int, 7> chosen{-1, -1, -1, -1, -1, -1, -1};
		bool armed{false}; // a click by a track places the chosen template
		char filter[64]{};
		char name[64]{"A"};
		char plate[64]{};
		char linked[64]{"none"};
		float height{0.0f};
		bool placing{false};
		signal_spot press;
		glm::vec2 pressed{0.f};
		int facing{0};
		std::vector<signal_standing> standing;
		std::size_t standing_history{static_cast<std::size_t>(-1)};
		std::size_t standing_redo{static_cast<std::size_t>(-1)};
		double standing_time{-1.0};
		int hover{-1};
		scene::instance_handle selected{0};
		bool moving{false};
		glm::dvec2 grab{0.0};
		signal_spot home;
		bool home_valid{false};
		signal_spot moved;
		bool moved_valid{false};
		std::string status;
	};
	signal_tool m_signal;
	bool signal_scan(bool Wait = true);
	signal_template const *signal_current() const;
	signal_template const *signal_armed() const;
	signal_template const *signal_template_of(std::string const &File) const;
	// the template it is placed by: the one of its file if that is of its kind, else the one chosen for its kind. returns: nullptr if none
	signal_template const *signal_template_for(signal_order const &Order) const;
	bool signal_spot_at(TTrack &Track, int const Path, glm::dvec3 const &Point, glm::dvec3 const &Cursor, int const Facing, int const Side, signal_template const &Template, signal_spot &Spot) const;
	void signal_refresh(bool const Force = false);
	// the signals standing, read by the tracks as they are now, the templates as found so far
	std::vector<signal_standing> signal_standing_now() const;
	int signal_hit() const;
	void signal_press(track_intent const &Intent);
	void signal_release();
	bool signal_place(signal_spot const &Spot, signal_template const &Template);
	void signal_move_update();
	bool signal_standing_spot(signal_standing const &Standing, signal_spot &Spot) const;
	bool signal_drop_at(signal_standing const &Standing, TTrack &Track, int const Path, glm::dvec3 const &Point, int const Side, signal_spot &Spot) const;
	bool signal_move_to(signal_standing const &Standing, signal_spot const &Spot, char const *Read, char const *Unread);
	bool signal_flip(scene::instance_handle const Instance);
	bool signal_turn(scene::instance_handle const Instance);
	bool signal_swap(scene::instance_handle const Instance, std::string const &File);
	// the signal set to the aspect in the editor, as its event sets it in the game: only how it looks, nothing saved
	bool signal_show_aspect(scene::instance_handle const Instance, std::string const &Aspect, std::string const &Event);
	signal_standing const *signal_standing_of(scene::instance_handle const Instance) const;
	bool signal_delete(scene::instance_handle const Instance);
	void signal_detach_events(std::string const &Read, std::vector<std::pair<TTrack *, editor_track::state>> &States);
	void signal_renamed(std::string const &Before, std::string const &After, EditorSnapshot &Snapshot);
	void signal_select(scene::instance_handle const Instance);
	std::string signal_next_name(std::string const &Name) const;
	void render_signal_ui();
	void render_signal_palette();
	void render_signal_selected();
	// of the signal selected: its template and another one for it, the aspects it can be set to; true when it got another one
	bool render_signal_template(signal_standing const &Found, signal_template const &Item);
	// the list of the templates the signal can take instead, Across those of another mount too: the file picked, empty for none
	std::string signal_swap_combo(signal_template const &Item, bool const Across) const;
	void render_signal_aspects(signal_standing const &Found, signal_template const &Item);
	void draw_signal_overlay() const;
	std::string signal_intent_label(signal_template const &Template, signal_spot const &Spot) const;
	std::string signal_variant(signal_template const &Template, signal_spot const &Spot) const;
	// what a deleted track carried and no other one took, its read events and track circuits, for a track laid in its place
	struct orphan_record
	{
		TTrack *track{nullptr}; // retired; the record waits while it stays so
		std::vector<std::pair<std::string, int>> events; // the name, the list it was on
		std::vector<std::string> circuits;
		// the tracks given the circuits: while one stays the circuits don't wait, once a step other than an undo removes one they
		// are its own
		std::vector<TTrack *> given;
		bool fresh{false}; // made by the step being taken
	};
	std::vector<orphan_record> m_orphans;
	std::string m_orphan_note; // shown by the cursor after the step which made it
	void orphans_register(std::span<TTrack *const> const Removed, std::span<TTrack *const> const Kept);
	// Created: made by the step, their circuits the ones of the tracks they were made from; Changed: the others it changed
	void orphans_take_over(std::span<TTrack *const> const Created, std::span<TTrack *const> const Changed);
	// the state of the taking over, defined where it's used; the signal reading the event of the name, nullptr for none; the event
	// of the old track put onto the candidate lying in its place, false where none lies there; a record taken over as far as it goes
	struct orphan_takeover;
	signal_standing const *orphan_reader(orphan_takeover &Takeover, std::string const &Name);
	bool orphan_take_event(orphan_takeover &Takeover, TTrack const &Old, std::string const &Name, int const List, signal_standing const *Standing) const;
	void orphan_take_record(orphan_takeover &Takeover, orphan_record &Record);

	sweep_tool m_sweep;
	bool sweep_press();
	void restore_sweeps(EditorSnapshot &Snapshot);

	void restore_bend(EditorSnapshot &Snapshot, bool const Undo);

	// script of editor actions given in the EU07_EDITOR_SELFTEST file, run a step at a time, for testing without a person
	struct selftest
	{
		bool loaded{false};
		std::vector<std::string> lines;
		std::size_t next{0};
		int wait{0};
		bool quitting{false};
		scripted_input input;
		std::string badge; // the mouse or the keys drawn over the view, for the clips of the tutorial
		glm::vec2 badge_at{0.0f}; // its bottom left corner, in the pixels of the window
	};
	selftest m_selftest;
	using selftest_result = editor_selftest_result;
	selftest_result selftest_scene(std::string const &command, std::istringstream &words);
	// the scene commands of the self-test by the group: the view and the tracks, ahead of the scripted input; the files, the nodes
	// and the rest after it; and those of them with more to them
	selftest_result selftest_scene_view(std::string_view const command, std::istringstream &words);
	selftest_result selftest_scene_files(std::string_view const command, std::istringstream &words);
	selftest_result selftest_scene_nodes(std::string_view const command, std::istringstream &words);
	selftest_result selftest_scene_other(std::string_view const command, std::istringstream &words);
	selftest_result selftest_select_at(std::istringstream &words);
	selftest_result selftest_track_ends(std::istringstream &words) const;
	selftest_result selftest_kbreak(std::istringstream &words);
	selftest_result selftest_ortho(std::istringstream &words);
	selftest_result selftest_map_state() const;
	selftest_result selftest_select_include(std::istringstream &words);
	selftest_result selftest_model_box() const;
	selftest_result selftest_key(std::istringstream &words);
	selftest_result selftest_sleepers(std::string const &command, std::istringstream &words);
	selftest_result selftest_sweep(std::string const &command, std::istringstream &words);
	selftest_result selftest_bend(std::string const &command, std::istringstream &words);
	selftest_result selftest_track(std::string const &command, std::istringstream &words);
	// the track commands of the self-test by the group: the lineside sweeps, the routes, their traces, the alignments, the
	// grade lines, the checks of the tracks; and those of them with more to them
	selftest_result selftest_track_sweep(std::string const &command, std::istringstream &words);
	selftest_result selftest_track_routes(std::string const &command, std::istringstream &words);
	selftest_result selftest_track_traces(std::string const &command, std::istringstream &words);
	selftest_result selftest_track_alignment(std::string const &command, std::istringstream &words);
	selftest_result selftest_track_profiles(std::string const &command, std::istringstream &words);
	selftest_result selftest_track_checks(std::string const &command, std::istringstream &words);
	selftest_result selftest_track_probe(std::istringstream &words);
	selftest_result selftest_track_lay(std::istringstream &words);
	selftest_result selftest_track_analyse(std::istringstream &words);
	selftest_result selftest_track_route(std::istringstream &words);
	selftest_result selftest_track_route_vertex(std::string const &command, std::istringstream &words);
	selftest_result selftest_track_trace_sample(std::istringstream &words);
	selftest_result selftest_track_trace_step(std::string const &command, std::istringstream &words);
	selftest_result selftest_track_alignments() const;
	selftest_result selftest_track_backward_curve(std::istringstream &words);
	selftest_result selftest_track_delete(std::string const &command, std::istringstream &words);
	selftest_result selftest_track_split_kind(std::string const &command, std::istringstream &words);
	void selftest_track_nudge(TTrack &Chosen, glm::dvec3 const &Offset);
	selftest_result selftest_track_vprofile_model() const;
	selftest_result selftest_track_vprofile_zoom(std::istringstream &words);
	selftest_result selftest_track_vprofile(std::istringstream &words);
	selftest_result selftest_track_tape(std::istringstream &words);
	selftest_result selftest_track_vprofile_run(std::string const &command, std::istringstream &words);
	void selftest_track_log_grade_line(std::string const &command) const;
	selftest_result selftest_track_vprofile_apply(std::istringstream &words);
	void selftest_track_infrascan(std::istringstream &words);
	selftest_result selftest_switch(std::string const &command, std::istringstream &words);
	// the commands of selftest_switch by the group: placing the switches, the switches there are, their drives; each command of them
	selftest_result selftest_switch_placing(std::string const &command, std::istringstream &words);
	selftest_result selftest_switch_checks(std::string const &command, std::istringstream &words);
	selftest_result selftest_switch_drives(std::string const &command, std::istringstream &words);
	selftest_result selftest_blade_tips(std::string const &command);
	selftest_result selftest_curvedswitch(std::istringstream &words);
	selftest_result selftest_designer(std::istringstream &words);
	selftest_result selftest_designerpick(std::istringstream &words);
	selftest_result selftest_release();
	selftest_result selftest_hover(std::istringstream &words);
	selftest_result selftest_layhold(std::istringstream &words);
	selftest_result selftest_switchkind(std::istringstream &words);
	selftest_result selftest_connection(std::string const &command, std::istringstream &words);
	void selftest_connection_drag();
	void selftest_connection_plan(std::string const &command, std::string const &kindword);
	selftest_result selftest_keyh();
	selftest_result selftest_switchdrive(std::istringstream &words);
	selftest_result selftest_drivechoice(std::istringstream &words);
	selftest_result selftest_drives(std::istringstream &words);
	selftest_result selftest_driveflip(std::istringstream &words);
	selftest_result selftest_drive_put(std::string const &command, std::istringstream &words);
	selftest_result selftest_switchfree(std::istringstream &words);
	selftest_result selftest_switchproblem(std::istringstream &words);
	selftest_result selftest_switch_place(std::string const &command, std::istringstream &words);
	bool selftest_switch_into_curve(TTrack &nearest, glm::dvec3 const &point, int const direction, int const side, std::string const &snap);
	void selftest_log_switch_frame() const;
	selftest_result selftest_switch_link(std::string const &command, std::istringstream &words);
	selftest_result selftest_switch_slide(std::string const &command, std::istringstream &words);
	void selftest_switch_slide_by(TTrack &track, std::string const &name, std::istringstream &words);
	int selftest_template_shown(TTrack const &Track) const;
	void selftest_switch_replace(TTrack &track, std::string const &name, std::istringstream &words);
	selftest_result selftest_switch_pieces(std::string const &command, std::istringstream &words);
	selftest_result selftest_switches() const;
	selftest_result selftest_rails(std::string const &command, std::istringstream &words);
	selftest_result selftest_join(std::string const &command, std::istringstream &words);
	// the commands of selftest_join by the group: the checks and the settings of the laying, the laying itself; each command of them
	selftest_result selftest_join_checks(std::string const &command, std::istringstream &words);
	selftest_result selftest_join_laying(std::string const &command, std::istringstream &words);
	selftest_result selftest_join_or_lay_to(std::string const &command, std::istringstream &words);
	selftest_result selftest_fitcheck(std::istringstream &words) const;
	selftest_result selftest_cantcheck(std::istringstream &words) const;
	selftest_result selftest_fittime(std::istringstream &words) const;
	selftest_result selftest_layspeed(std::istringstream &words);
	selftest_result selftest_laymode(std::istringstream &words);
	selftest_result selftest_laytrace(std::istringstream &words);
	selftest_result selftest_layp(std::istringstream &words);
	selftest_result selftest_layreport(std::istringstream &words);
	selftest_result selftest_laywalk(std::istringstream &words);
	selftest_result selftest_laybends(std::istringstream &words);
	selftest_result selftest_lay_key(std::string const &command);
	selftest_result selftest_laysample(std::istringstream &words);
	selftest_result selftest_laydrag(std::istringstream &words);
	selftest_result selftest_tracktab(std::istringstream &words);
	selftest_result selftest_laychain(std::istringstream &words);
	selftest_result selftest_laystate() const;
	selftest_result selftest_laymonkey(std::istringstream &words);
	selftest_result selftest_layclicks(std::istringstream &words);
	selftest_result selftest_gizmo(std::istringstream &words);
	// the gizmo commands of the self-test by the group: what it works on, its drag on the selected track begun and gone on with
	static gizmo_operation gizmo_operation_named(std::string_view const Name);
	void selftest_gizmo_set(std::string const &action, std::istringstream &words);
	void selftest_gizmo_begin(TTrack &track, std::istringstream &words);
	void selftest_gizmo_drag(std::string const &action, TTrack &track, std::istringstream &words);
	// the point clicked by the lay tool, as a self-test gives it; the line the points clicked make now logged
	void selftest_lay_point(glm::dvec3 Point);
	void selftest_lay_report(std::string const &Label);
	void selftest_click_logged(glm::dvec3 Point);
	selftest_result selftest_signal(std::string const &command, std::istringstream &words);
	// the self-tests of the descriptions of the signals: the templates with what the descriptions give, a signal set to an aspect
	selftest_result selftest_signal_templates(std::istringstream &words);
	selftest_result selftest_signal_aspect(std::istringstream &words);
	// the signal commands of the self-test with more to them, on the values given with the command
	void selftest_signal_place(std::map<std::string, std::string, std::less<>> &values);
	void selftest_signal_move(std::map<std::string, std::string, std::less<>> &values);
	void selftest_signal_select(std::string const &command, std::map<std::string, std::string, std::less<>> &values);
	void selftest_signal_probe();
	selftest_result selftest_terrain(std::string const &command, std::istringstream &words);
	// the terrain commands of the self-test by the group, and those of them with more to them
	selftest_result selftest_terrain_imagery(std::string_view const command, std::istringstream &words);
	selftest_result selftest_terrain_model(std::string_view const command, std::istringstream &words);
	selftest_result selftest_terrain_fit(std::string_view const command, std::istringstream &words);
	selftest_result selftest_terrain_edit(std::string_view const command, std::istringstream &words);
	selftest_result selftest_terrain_told(std::string_view const command, std::istringstream &words);
	selftest_result selftest_terrain_reports(std::string_view const command, std::istringstream &words);
	selftest_result selftest_orthophoto(std::istringstream &words);
	selftest_result selftest_orthoarea(std::istringstream &words);
	selftest_result selftest_orthosettings(std::istringstream &words);
	selftest_result selftest_nmtsettings(std::istringstream &words);
	selftest_result selftest_wmsstats() const;
	selftest_result selftest_orthostats() const;
	selftest_result selftest_nmtmake(std::istringstream &words);
	selftest_result selftest_nmtpending();
	selftest_result selftest_nmtoffer(std::istringstream &words);
	selftest_result selftest_nmtgrow(std::istringstream &words);
	selftest_result selftest_nmtwait();
	selftest_result selftest_nmtcheck(std::istringstream &words) const;
	std::string turntable_include_text(std::string const &Name) const;
	void log_turntable_end(nmt_state::turntable const &Table, TTrack *Track, int const Path, bool const Atend) const;
	selftest_result selftest_turntables() const;
	selftest_result selftest_fittedstate(std::istringstream &words);
	void log_nmt_probe(nmt_place const &Place, nmt_place const &Next) const;
	selftest_result selftest_nmtprobe(std::istringstream &words) const;
	selftest_result selftest_trackterrain(std::istringstream &words);
	selftest_result selftest_terrain_brush(std::string_view const command, std::istringstream &words);
	selftest_result selftest_terrainfill(std::istringstream &words);
	selftest_result selftest_terrainrestore(std::istringstream &words);
	selftest_result selftest_terrainbake(std::istringstream &words);
	selftest_result selftest_terrainselect(std::istringstream &words);
	selftest_result selftest_terrainwater(std::string_view const command, std::istringstream &words);
	selftest_result selftest_terrainseams() const;
	selftest_result selftest_danglers() const;
	selftest_result selftest_terrainhash() const;
	selftest_result selftest_terrainprobe(std::istringstream &words) const;
	selftest_result selftest_terrainconvert();
	int track_grade_line(TTrack const &Track) const;
	selftest_result selftest_fitreport(std::istringstream &words) const;
	selftest_result selftest_gradeline(std::istringstream &words);
	void gradeline_part(std::istringstream &words, double const value);
	void gradeline_bump(std::istringstream &words, double const value);
	bool gradeline_action(std::string const &action, std::string const &word, double const value, std::istringstream &words);
	void gradeline_move(std::string const &word, std::istringstream &words);
	void gradeline_other(std::string const &action, std::string const &word, double const value, std::istringstream &words);
	void gradeline_of(std::string const &action, std::string const &name, std::istringstream &words);
	void gradeline_pick(double const x, std::istringstream &words);
	selftest_result selftest_trackheights(std::istringstream &words);
	TTrack const *profile_span_track(std::size_t const Span) const;
	selftest_result selftest_profiledump(std::istringstream &words) const;
	int beside_line(TTrack const &Track) const;
	selftest_result selftest_besidedump(std::istringstream &words);
	void log_profile_off(double const Departure, TTrack const *From, TTrack const *To) const;
	void profile_audit(std::size_t const Index, std::size_t &Gone, std::size_t &Retaken, std::size_t &Failing);
	selftest_result selftest_profileaudit();
	selftest_result selftest_gradelines(std::istringstream &words) const;
	selftest_result selftest_props(std::string const &command, std::istringstream &words);
	// the commands of selftest_props, one each
	selftest_result selftest_tracksel(std::istringstream &words);
	selftest_result selftest_trackprop(std::istringstream &words);
	selftest_result selftest_trackprops();
	selftest_result selftest_trackstyle(std::istringstream &words);
	void selftest_trackstyle_list();
	selftest_result selftest_tracksimilar(std::istringstream &words);
	selftest_result selftest_monkey(std::string const &command, std::istringstream &words);
	// monkey <seed> <steps>: the random steps written into the script after this line
	selftest_result selftest_monkey_run(std::istringstream &words);
	// monkeyview <seed>, or monkeyat <seed> with View false
	selftest_result selftest_monkey_view(bool const View, std::istringstream &words);
	selftest_result selftest_undoall();
	selftest_result selftest_dispatch(std::string const &command, std::istringstream &words);
	void selftest_step();
	// a value typed during a gesture, taken instead of the one the cursor gives
	std::string m_typed;
	// what the typed value sets in the gesture under way, nullptr when the gesture takes none
	char const *typed_meaning() const;
	std::vector<double> typed_values() const;
	bool typed_key(int const Key);
	glm::dvec3 typed_lay(glm::dvec3 const &Mouse) const;
	void typed_turn();
	bool m_route_tab{true};

	profile_state m_profile;
	std::vector<stored_profile> m_profiles;
	bool m_profiles_loaded{false};
	void profile_library_mark(std::vector<scene::layer_handle> Layers);
	bool profile_restore();
	// the radii of its neighbours were for the breaks of the grade it made, they follow the limits after it
	void profile_remove_point(int const Index);
	void render_profile_body();
	void render_profile_strip();
	void profile_open(TTrack *From, TTrack *To);
	void profile_open_run(TTrack &Track);
	// depth of the foot of the ballast under the rail top, and its distance from the axis
	static std::pair<double, double> profile_ballast(TTrack const &Track);
	void profile_check();
	// Terrain: the heightmap terrain follows the tracks changed. Bridges: stretches of the route over a bridge or a viaduct, the
	// paths over them cut off at their ends and set to the bridge environment
	void profile_apply(bool const Terrain = false, std::vector<profile::departure> const &Bridges = {}, double const Fewest = 0.0);
	void profile_after_undo();

	infra_window m_infra;
	std::vector<infra::binding> m_bindings;
	std::vector<std::pair<scene::layer_handle, std::string>> m_bindings_unresolved; // lines kept as they are
	bool m_bindings_loaded{false};
	std::vector<infra::object_state> m_infra_buffer; // states of the objects before the change in progress
	std::vector<std::pair<scene::instance_handle, std::string>> m_drive_buffer;
	bool m_infra_suspended{false};
	std::vector<point_object> m_point_objects;
	std::size_t m_point_objects_revision{static_cast<std::size_t>(-1)};
	std::size_t m_point_objects_count{0};
	void infra_adopt(TTrack &Track);
	// its steps: the point objects gathered again, one of them taken by the track
	void point_objects_gather();
	void infra_adopt_object(TTrack &Track, point_object const &Object);
	int infra_bound_count() const; // by the infra tool, not the ones a change of their track took along
	std::string infra_carried_note(); // a line of the status telling how many, empty for none; clears them
	void infra_load();
	void infra_store();
	void infra_forget(scene::basic_node const *Node);
	std::vector<TTrack *> infra_scope() const;
	void infra_recognize();
	void infra_bind_chosen();
	void infra_unbind(std::vector<std::size_t> Indices);
	infra::binding *infra_find(infra::binding const &Binding);
	infra::object_state infra_state_of(infra::binding const &Binding) const;
	void infra_restore_state(infra::object_state const &State);
	void infra_buffer(infra::binding const &Binding, bool const Refresh);
	// editor_track::observer
	void track_captured(TTrack const &Track) override;
	void tracks_committed(std::vector<TTrack *> const &Tracks) override;
	void track_retired(TTrack &Track) override;
	void track_split(TTrack &Original, TTrack &Created, segment_data const &First) override;
	void track_renamed(TTrack const &Track, std::string const &Old) override;
	void infra_move(infra::binding &Binding);
	void infra_attach(EditorSnapshot &Snapshot);
	void render_infra_body();

	gauge_tool m_gauge{*this, *this, *this, *this};
	speed_tool m_speed{*this, *this, *this, *this};
	joints_tool m_joints{*this, *this, *this, *this};
	vehicle_tool m_vehicle{*this, *this};
	hekto_tool m_hekto{*this, *this, *this, *this, *this, *this};
	fouling_tool m_fouling{*this, *this, *this, *this, *this};
	parallel_tool m_parallel{*this, *this, *this, *this};
	srk_tool m_srk{*this, *this, *this, *this, *this};
	km_tool m_km{*this, *this, *this, *this};
	std::vector<editor_tool *> m_tools{&m_gauge, &m_speed, &m_joints, &m_vehicle, &m_hekto, &m_fouling, &m_parallel, &m_srk, &m_km};

	// the build tools (editormodebuildtools.h), made by make_build_tools()
	std::unique_ptr<build_tools> m_build_tools;
	void make_build_tools();
	// build services: overrides
	road_tool &road_state() override;
	std::vector<double> ground_heights_at(std::vector<glm::dvec3> const &Points, bool const Fresh, std::vector<char> *Found) const override;
	void push_road_snapshot(editor_road::record Record) override;
	bool render_road_material(char const *Label, std::string &Value) override;
	sweep_tool &sweep_state() override;
	void sweep_curve() override;
	void push_sweep_toggled(sweep_node &Sweep) override;
	void push_sweep_change(sweep_node &Sweep, sweep_node::state Before) override;
	void push_bend_made(sweep_node &Sweep, bend_source const &Source) override;
	bool bend_source_of_selection(bend_source &Source) const override;
	editor_ui *build_ui() const override;
	scene::basic_node *current_node() const override;
	scene::instance_handle current_instance() const override;
	void drop_node_selection() override;
	bool right_button_held() const override;
	profile_state &vertical_profile() override;
	std::vector<stored_profile> &stored_profiles() override;
	bool &profiles_loaded() override;
	nmt_state &terrain_state() override;
	terrain_streamer &ground_streamer() override;
	float const &terrain_spacing() const override;
	int const &track_edits() const override;
	// the last step takes the kept grade lines back to the library given, as they were before it changed them
	void profiles_to_history(std::vector<stored_profile> Library) override;
	void history_take_profiles(std::vector<stored_profile> Library) override;
	std::vector<TTrack *> tracks_edited(std::size_t const Edits) const override;
	void terrain_step_joined() override;
	TCamera &view_camera() override;
	void release_focus() override;
	void forget_ground_cache() override;
	bool terrain_ready() override;
	void open_terrain_workspace(bool const Open) override;
	std::size_t shape_tracks_on_terrain(std::vector<TTrack *> const *Only) override;
	void join_last_history_steps() override;
	std::vector<TTrack *> selected_track_list() override;
	TTrack *grade_line_pick_aimed(int &Index) const override;
	bool tracks_editable_for(std::vector<TTrack *> const &Tracks, std::string &Reason) const override;
	std::size_t follow_crossovers_of(std::vector<editor_track::route_sample> const &Before) override;
	void render_ground_conversion_panel() override;
	editor_orthophoto &orthophoto() override;
	std::vector<std::unique_ptr<editor_orthophoto>> &imagery_layers() override;
	editor_map &terrain_map() override;
	area_drawing &area_state() override;
	bool &area_paused() override;
	lay_tool &lay_state() override;
	extend_tool &extend_state() override;
	int &terrain_add_count() override;
	bool &terrain_open() override;
	float &terrain_base_height() override;
	bool &orthophoto_chosen() override;
	glm::dvec2 &orthophoto_origin_edit() override;
	terrain_streamer::change const *terrain_step_marked() override;
	void terrain_step_join_tracks(terrain_streamer::change const *Top) override;
	void terrain_step() override;
	std::vector<TTrack *> const &select_grown_track_set(std::vector<TTrack *> const &Tracks) override;
	std::vector<TTrack const *> tracks_made_since(std::size_t const From) const override;
	bool terrain_is_work_area() const override;
	std::size_t history_size() const override;
	void join_last_step() override;
	void join_history_from(std::size_t const Step) override;
	void raise_model_joined(TAnimModel &Model, double const Rise) override;
	void forget_ground_area(glm::dvec2 const &Min, glm::dvec2 const &Max) override;
	void select_track_list(std::vector<TTrack *> const &Tracks) override;
	void select_track_node(scene::basic_node *Node) override;
	void toggle_orthographic() override;
	bool orthophoto_settings_shown() const override;
	void save_imagery_settings() override;
	glm::dvec3 cursor_ground_point() const override;
	route_design &route_state() override;
	straights_state &straight_lines() override;
	bool &route_dragging() override;
	infra_window &infra_state() override;
	void route_recompute() override;
	alignment::limits lay_norms() const override;
	void undo_last_step() override;
	std::string refusal(std::string Text) override;
	void records_into_history() override;
	void record_switch_shape(TTrack const &Track, editor_track::switch_template const &Shape) const override;
	std::optional<curved_crossover> crossover_at(TTrack const &Switch) const override;
	bool lay_crossover_at(int const Shape, glm::dvec3 const &Press, glm::dvec3 const &Release, double const Reach, std::vector<std::string> const &Names) override;
	int infra_bound_total() const override;
	std::string infra_carried_status() override;
	void infra_recognise_objects() override;
	void infra_bind_chosen_objects() override;
	glm::dvec3 cursor_at_level(double const Height) const override;
	bool handle_drag_moved(glm::dvec3 &Position, TTrack const *Ground) override;
	bool take_out_crossover_at(curved_crossover const &Crossover, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed, TTrack *&First, TTrack *&Second) override;
	int crossover_template_of(switch_store::record const &Record) override;
	void neighbour_straights_found() override;
	void focus_on_node(scene::basic_node *Node) override;
	std::vector<TTrack *> &tracks_dragged() override;
	void track_drag_moved() override;
	void track_drag_commit(bool const Force) override;
	void track_tab_set(editor_track_tab const Tab) override;
	std::vector<double> typed_numbers() const override;
	void typed_turn_taken() override;
	bool following_preview() const override;
	void follow_preview_set(bool const Following) override;
	double lay_design_speed(TTrack const *From, TTrack const *To) const override;
	double &join_radius_chosen() override;
	int &join_variant_chosen() override;
	alignment::join_result &join_result_last() override;
	std::optional<alignment::lay_result> &eased_result_last() override;
	std::string &typed_text() override;
	editor_track::layout &laying_layout() override;
	double joint_kink() const override;
	void join_target_aimed(editor_track::snap_target const &Target) override;
	void tutorial_open_at(char const *Lesson) override;
	editor_track::point_ref &track_point_chosen() override;
	glm::dvec3 cursor_on_terrain(double const Height) const override;
	editor_track::snap_target free_end_snapped(glm::dvec3 const &Near, glm::vec2 const &Screen, TTrack const *Self, int const Category, std::vector<TTrack const *> const &Exclude) const override;
	bool free_end_of(TTrack &Track, int const Path, bool const Atend, extend_tool &Tool) const override;
	double bed_height_at(glm::dvec3 const &Point, glm::dvec2 const &Direction, TTrack const *Style) const override;
	glm::dvec3 lay_bed_point(glm::dvec3 const &Point) const override;
	glm::dvec3 typed_lay_point(glm::dvec3 const &Mouse) const override;
	void ground_followed(std::vector<segment_data> &Pieces, double const Height0, double const Grade0, double const Height1, double const Grade1, TTrack const *Style) const override;
	bool branch_link_planned(editor_track::branch_link &Link, std::string &Reason) const override;
	void branch_link_finished(editor_track::branch_link const &Link) override;
	bool crossover_laid_from_extend() override;
	track_hover track_hovered(TTrack const *Only, float const Pixels) const override;
	std::uint64_t texture_image_of(int const Kind, std::string const &Name) override;
	bool texture_picked(int const Kind, std::string const &Current, bool const Mixed, std::string &Chosen) override;
	bool material_dropped(std::string &Name) const override;
	// build services: end


	void infra_rebase();
	void render_infra_search();
	void render_infra_candidates();
	void render_infra_bound();
	void draw_infra_overlay() const;

	bool m_route_dragging{false};
	float m_track_snap_radius{1.0f};
	bool m_track_align_tangent{true};
	bool m_track_drag_connected{true};

	// height of the ground under each of the points, with the roads not taken for the ground. a point with nothing under it keeps its height
	// Found: set for the points which have the ground under them
	std::vector<double> ground_heights(std::vector<glm::dvec3> const &Points, bool const Fresh, std::vector<char> *Found = nullptr) const;
	void restore_road_snapshot(EditorSnapshot const &Snapshot, std::vector<EditorSnapshot> &Opposite, bool const Undo);
	road_tool m_roadtool;
};
