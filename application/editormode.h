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
#include <memory>
#include <optional>
#include <sstream>
#include <limits>
#include <set>
#include <string>
#include <vector>

class TAnimModel;
// a tool of a field of work of the editor, a button in the toolbar and over the 3d view
struct editor_tool_button;
class TTrack;

class editor_mode : public application_mode, private editor_track::observer, private editor_services
{

  public:
	// constructors
	editor_mode();
	~editor_mode() override;
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

	struct stored_profile;
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
	void restore_kilometrage(EditorSnapshot &Snapshot);
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
	void render_first_steps();
	void first_step_done(first_step const Step) override;
	bool m_first_steps_open{false};
	std::array<bool, static_cast<std::size_t>(first_step::count)> m_first_steps_done{};
	int m_tutorial_page{0};
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
	struct nmt_state
	{
		double offset{0.0}; // added to the heights of the model, so they meet the tracks of the scenery
		bool fitted{false};
		bool fitting{false}; // waiting for the model under the places of the tracks the offset is worked out from
		std::vector<nmt_place> fit;
		double spread{0.0}; // how much the tracks differ from the model about the offset, m
		std::size_t samples{0};
		double band{300.0}; // m from the tracks the chunks made along them reach
		bool shape{true}; // the formation and the slopes of the tracks as modifiers of the terrain
		bool grow{true}; // while a track is laid, the terrain is made around the cursor where there's none
		bool laying{false}; // the tracks committed are being laid: they leave the terrain as it is
		bool follow{true}; // a track with its formation edited takes it along, the formation led anew
		bool grown{false}; // the chunks waiting were all asked for by growing: the formations aren't made anew after them
		bool reshape{false}; // the formations of the tracks laid anew once the chunks waiting are made, as the tracks are put onto the terrain
		bool unfitted{false}; // the tracks put onto the terrain are only the ones whose grade line isn't fitted yet
		double grow_time{-10.0};
		double slope{1.5};
		double reach{60.0};
		std::vector<heightmap::chunk_key> pending; // chunks to make once their model is in
		std::set<heightmap::chunk_key> given_up; // chunks the growing gave up, next to terrain not from the model: not asked for again
		std::size_t made{0};
		std::size_t empty{0}; // chunks not made: the model has no data there
		std::size_t foreign{0}; // chunks not made ahead: the terrain beside isn't of the model
		std::optional<double> ahead; // how much the terrain made ahead lies over the model
		std::set<TTrack *> changed; // tracks whose formations are to follow them
		std::set<std::string> gone; // names of the tracks removed, whose formations go with them
		double changed_time{0.0};
		bool restoring{false}; // the history is taken back or forward: the terrain comes back with it
		std::size_t shaped{0};
		std::string status;
		std::string error;
		// the tracks of a scenery built at any height put onto the terrain model: the whole scenery lifted to it, then the grade
		// line of each line of tracks fitted to the model, the longest first, then the terrain made along them
		enum class automatic
		{
			idle,
			fetching, // the model under the tracks
			fitting // the grade lines, a few each frame
		};
		automatic stage{automatic::idle};
		std::vector<nmt_place> probes; // places of the tracks whose model is waited for
		std::size_t tracks{0}; // rail tracks when the grade lines began, and the paths the grade lines split off since
		std::size_t split{0};
		std::vector<TTrack *> left; // tracks whose grade line is still to be fitted
		std::vector<TTrack *> done; // tracks of the lines fitted already: the lines fitted later end on them and don't change them
		// the rail tops of the lines fitted already, in squares of the plan: the tracks of a station lie in one plane, a line laid
		// near them takes their height for its ground too, the more closely the nearer it is, and a line crossing one of them
		// on its level its height on the crossing
		struct laid_point
		{
			glm::dvec3 position{0.0};
			glm::dvec2 direction{0.0, 1.0};
			double grade{0.0};
			double was{0.0}; // the height before the line was fitted
			double rails{0.0}; // the height of the rails of the track
			TTrack const *track{nullptr};
		};
		std::map<std::pair<int, int>, std::vector<laid_point>> laid;
		// the points of all the paths as they were laid, for the crossings
		std::map<std::pair<int, int>, std::vector<laid_point>> paths;
		// ends which meet in the plan unlinked, laid apart in height, that the joining of the ends couldn't join as neither of
		// them can be changed alone, as of two switches: each with the other one, which a line ending there takes over
		std::map<std::tuple<TTrack const *, int, bool>, std::tuple<TTrack *, int, bool>> loose;
		// the points of the lines over a bridge or a viaduct, the model falling away under them: no formation is laid there
		std::map<std::pair<int, int>, std::vector<glm::dvec3>> bridged;
		std::size_t lines{0}; // grade lines fitted and applied
		std::size_t refused{0}; // grade lines the checks didn't let apply
		std::size_t departures{0}; // grade lines parting from the terrain model by much
		double lifted{0.0}; // m the scenery went up or down
		// the terrain made under the tracks as they are: the model drawn to them by the difference found at the places along them,
		// fading out away from them into the offset of the whole scenery
		bool under{false};
		std::vector<glm::dvec3> anchors; // x, z of a place of the tracks, the formation there less the model
		std::map<std::pair<int, int>, std::vector<std::size_t>> anchor_cells; // the anchors in squares of the plan
		bool offered{false}; // the terrain was offered to the scenery begun without it
		bool prompt{false}; // the offer is shown
		bool moving{false}; // the place being picked again
		char place[96]{}; // typed: degrees or PUWG 1992
		int shown{-1}; // the frame the terrain panel showed how far the work got
	};
	editor_nmt m_nmt;
	nmt_state m_nmt_state;
	editor_map m_nmt_map;
	std::vector<nmt_place> nmt_track_places(double const Step) const override;
	void nmt_locate() override;
	void nmt_fit();
	void nmt_finish_fit();
	// the terrain under the tracks, the tracks left as they are: the model drawn to each of them, its relief kept
	void nmt_under_start();
	void nmt_under_anchors();
	// added to the height of the model at the place: the offset of the scenery, drawn to the tracks near them
	double nmt_offset_at(double const X, double const Z) const;
	std::vector<heightmap::chunk_key> nmt_keys_around() const;
	std::vector<heightmap::chunk_key> nmt_keys_along_tracks() const;
	// only the chunks there are none of yet: a chunk there is, made from the model or by hand, edited or not, is never made anew.
	// Reshape: the formations of the tracks laid anew as the tracks lie now, as when they moved
	void nmt_make(std::vector<heightmap::chunk_key> const &Keys, bool const Reshape = false);
	enum class nmt_chunk
	{
		waiting, // for its squares of the model
		made,
		empty // the model has no data there
	};
	// Offset: the heights of the model go up or down by it in the whole chunk, in place of the offsets of the fit
	nmt_chunk add_chunk_from_nmt(heightmap::chunk_key const &Key, std::optional<double> const Offset = std::nullopt);
	// how much the terrain lies over the model at the points: none where there's too little terrain at them, nor where it
	// doesn't follow the model (Foreign), as one made by hand
	std::optional<double> nmt_terrain_offset(std::vector<glm::dvec2> const &Points, bool &Foreign) const;
	// the same along the edges the chunk shares with the chunks beside it in memory
	std::optional<double> nmt_seam_offset(heightmap::chunk_key const &Key, bool &Foreign) const;
	// the terrain beside the formation of the track is the one made from the model; Waiting: its squares of the model aren't in yet
	// the tracks changed by hand are told to the terrain: their formations laid anew, of the tracks removed taken away, a
	// track laid onto the terrain of the model given its own, once the mouse lets go; in the step of the history of the change
	void nmt_track_changed(std::vector<TTrack *> const &Tracks);
	void nmt_track_gone(TTrack const &Track);
	void nmt_follow_tracks();
	// the height the ground will have where the terrain is made ahead of the track laid and isn't there yet
	bool nmt_ahead(double const X, double const Z, double &Height) const;
	// the same, else the height of the model with the offset of the scenery where its square is in: the ground where there's no
	// terrain made yet, as under the base map of the top view
	bool nmt_ground(double const X, double const Z, double &Height) const;
	// the squares of the model under the rectangle are in, none of them waiting to be asked for again
	bool nmt_ready(glm::dvec2 const &Min, glm::dvec2 const &Max);
	// Only: the paths of these tracks are shaped anew, the modifiers of those removed taken away; null: all of them. the number
	// of the modifiers laid
	std::size_t nmt_shape_tracks(std::vector<TTrack *> const *Only = nullptr);
	void nmt_update();
	void nmt_grow();
	void render_nmt_ui();
	// where the scenery lies: the map to click on, the place typed; true once it's placed
	bool render_nmt_place(bool const Moving, bool const Button);
	// the work under way: what's left of it and the button to stop it; Compact: one line, for the 3d view
	void render_nmt_progress(bool const Compact);
	// the formations of the tracks led into the terrain, down or up to it, or taken out of it; one step of the history each
	void nmt_pull_terrain(std::vector<TTrack *> const &Tracks);
	void nmt_restore_terrain(std::vector<TTrack *> const &Tracks);
	// the terrain of the selected tracks in the panel of the tracks
	void render_track_terrain(std::vector<TTrack *> const &Tracks);
	// the chunks waiting for the terrain model and the tiles of the imagery asked for outlined where they will be, so it's seen
	// what's on its way
	void draw_nmt_pending() const;
	// over the 3d view while the terrain panel isn't seen
	void render_nmt_banner();
	bool nmt_busy() const;
	// the terrain made so far stays, what's waiting is dropped, the grade lines not fitted yet are left
	void nmt_stop();
	// a scenery begun from nothing, with no tracks nor terrain, is asked as the first track is laid whether it wants the terrain
	void nmt_offer();
	void render_nmt_prompt();
	// the place picked in the terrain window: the terrain model and the orthophoto take it
	void nmt_place_scenery(glm::dvec2 const &Point);
	void nmt_accept_offer();
	// the whole scenery: tracks, includes and models, moved up or down as one undo step
	void nmt_lift_scenery(double const Offset);
	std::size_t nmt_join_ends();
	// Unfitted: only the tracks whose grade line isn't fitted yet, the scenery left at its height
	void nmt_auto_start(bool const Unfitted = false);
	void nmt_auto_step();
	// the grade line of the line of tracks through the track fitted to the terrain model and applied; false if it couldn't be.
	// The line runs over the tracks not Fitted, the ones fitted stay as they are and the line meets them. Where the model falls
	// away under the line by the abutments the tracks go over on a bridge; Laid: the tracks just laid, the formation fills it and
	// they are cut only where they can't follow the line otherwise
	bool nmt_fit_line(TTrack &Track, std::function<bool(TTrack const *)> const &Fitted, bool const Laid);
	// the tracks laid in one go since the history step From put onto the terrain model as one line, as the tracks go onto the
	// terrain: the track ending at End then the one the laying goes on from
	bool nmt_fit_laid(std::size_t const From, glm::dvec3 const End);
	// the tracks laid one after another on one straight or one arc, cut by the grade lines fitted before, made one again
	void nmt_merge_laid(std::unordered_set<TTrack const *> &Laid);
	// the points of the paths of the track about a metre apart, in squares of the plan
	static void nmt_sample_paths(TTrack const &Track, std::map<std::pair<int, int>, std::vector<nmt_state::laid_point>> &Points);
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
	// takes the height of whatever is under the cursor (terrain, track, model) for the target of the level brush. returns: false if there's nothing
	bool pick_terrain_target();
	bool terrain_clicking() const { return m_terrain_tool == terrain_tool::chunks || m_terrain_tool == terrain_tool::spacing || m_terrain_tool == terrain_tool::water || m_terrain_tool == terrain_tool::fill; }
	// fills the outlined area with the material of the chosen palette entry, turned by m_paint_rotation
	void fill_terrain_area();
	// the brush, the hovered chunk and the outlines of water drawn over the viewport
	void draw_terrain_overlay();
	// forgets what's known about the ground, after the terrain changed
	void forget_ground();
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
	char m_water_default_material[128]{""}; // of the waters which don't name their own, while it's edited
	bool m_water_default_edited{false};
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
	bool m_orthophoto_window{false};
	void render_orthophoto_window();
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
	// picks up the stored settings and the origin of the current scenery
	void load_orthophoto_settings();
	void save_orthophoto_settings();
	editor_orthophoto m_orthophoto;
	std::size_t m_ortho_resident{0}; // chunks of the terrain in memory, as the imagery last heard of them
	std::size_t m_ortho_waiting{0}; // squares of the terrain model asked for and not in yet, the same
	std::string m_orthophoto_scenery;            // scenery whose origin is loaded
	std::string m_orthophoto_mark;
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
	} m_record_watch;
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
		bool control{false};
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
	void track_gizmo_point(TTrack &Track, glm::dvec3 const &Position);
	void track_gizmo_switches(TTrack &Track);
	void track_gizmo_conform(TTrack &Track) const;
	bool track_gizmo_clustered(TTrack const *Track) const;
	void track_gizmo_spill(TTrack &Track, std::size_t const Pieces);
	void track_gizmo_carry(track_gizmo_drag::switch_joint &Link, glm::dvec3 const &Point, glm::dvec3 const &From, glm::dvec3 const &To, double const Grade);
	void track_gizmo_roll_switch(track_gizmo_drag::switch_joint const &Link, double const Roll);
	void track_gizmo_take_joints(TTrack &Track);
	void track_gizmo_take_points(TTrack &Track);
	void track_gizmo_take_partners(TTrack &Track);
	void track_gizmo_take_switch_joints(TTrack &Track, bool const Controlmode);
	track_gizmo_drag::switch_joint track_gizmo_link(TTrack &Track, editor_track::point_ref const &Own, std::pair<TTrack *, editor_track::point_ref> const &Connection, std::function<bool(TTrack const *)> const &Movable);
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
	std::chrono::steady_clock::time_point m_track_last_commit;
	editor_track::state m_track_field_before;
	// the event to add to the path, typed
	TTrack const *m_track_events_for{nullptr};
	std::array<char, 128> m_track_event{};
	int m_track_event_list{0};
	std::string m_track_events_missing;
	void render_path_events(TTrack &Track);
	std::string m_track_mode_notice;
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
		bool single{true}; // one curve between its straights, instead of the whole line
		bool relaying{false}; // the crossovers of the line were taken out, they go back once it's laid
		// crossovers taken out with a line which didn't fit it laid again, tried again as the other lines are laid
		struct crossover_out
		{
			int shape{-1};
			glm::dvec3 press{0.0}, release{0.0};
			std::string name;
			std::vector<std::string> names; // of the switches at the press and at the release
		};
		std::vector<crossover_out> crossovers_out;
		double origin{0.0}; // km of the start, m
		int designed{0}; // vertices whose parameters came from the alignment kept in the scenery
		int departed{-1}; // alignment kept in the scenery near the fragment, which its paths no longer follow
		double departure{0.0};
		// the band of the curvature in the panel: the element under the mouse, the point of it, and what's dragged in it
		struct curvature_band
		{
			int hot{-1};
			double at{-1.0};
			int vertex{-1};
			int part{0}; // 1 the radius, 2 the transition in, 3 the transition out
			double value{0.0};
			ImVec2 press{0.0f, 0.0f};
			float metres{1.0f}; // pixels of a metre and of a unit of the curvature, kept as the drag goes
			float curvature{1.0f};
		} band;
		// the part of a curve whose field the mouse is over in the panel, or which is being edited there, shown in the view
		struct curve_focus
		{
			enum class part
			{
				whole,
				entry,
				exit,
				arc, // of the radius `arc`, the first one 0
				between // the transition leading into the arc of the radius `arc`
			};
			int vertex{-1};
			part what{part::whole};
			int arc{0};
			bool held{false}; // by a field being edited, the mouse over the others doesn't move it
		} focus;
		// the points clicked when the fragment was laid, kept with its design: the curves keep to them as their transitions and cants
		// change, while hold is on
		std::vector<glm::dvec3> clicks;
		bool hold{true};
		struct click_state
		{
			double offset{0.0}; // from the line, m
			int vertex{-1}; // whose curve it's a point of
			bool corner{false}; // a bend of straights clicked, the vertex itself rather than a point of its curve
		};
		std::vector<click_state> click_states; // as the line was at the revision measured
		int revision{0}; // of the result, counted as it's computed
		int measured{-1};
		// points clicked along the middle of the track seen on the orthophoto, the radii of the curves and their shares fitted to them after each click
		struct trace_state
		{
			bool active{false};
			int radii{0}; // 0: as many as the points need, up to three
			std::vector<glm::dvec3> points;
			alignment::design before; // taken back by Esc
			double rms{0.0};
			double worst{0.0};
		} trace;
	};
	void render_route_ui();
	bool render_route_band();
	bool render_route_parameters();
	bool render_route_vertices();
	bool render_route_vertex(int const Index);
	void render_route_result();
	void render_route_trace();
	// the camera over the stretch of the line between the chainages, a curve seen whole
	void route_show_curve(double const From, double const To);
	// the departures of the points clicked from the line and the curves they belong to, measured again when the line changed
	void route_measure_clicks();
	// the points clicked along the curve of the vertex, in the plan, the bends of straights left out
	std::vector<glm::dvec2> route_clicks_of(int const Vertex) const;
	// the radii of the curve of the vertex fitted again to the points clicked along it, its transitions and cants as they are
	bool route_hold_clicks(int const Vertex);
	// the elements of the part of the curve in focus in the panel, in their order along the line
	std::vector<std::size_t> route_focused() const;
	void route_trace_start();
	void route_trace_fit();
	void route_trace_end(bool const Keep);
	int route_trace_hit() const;
	glm::dvec3 route_trace_cursor() const;
	void route_reset();
	void route_bind_ends();
	bool route_from_curve(TTrack &Track);
	bool route_from_line(TTrack &Track);
	alignment::vertex route_vertex_of(editor_track::curve const &Curve) const;
	bool route_load(TTrack &Track);
	void route_read_stored(TTrack &Track);
	void route_take_stored(int const Index);
	void record_switch(TTrack &Track, editor_track::switch_template const &Shape);
	void record_double(TTrack &First, TTrack &Second, editor_track::switch_template const &Shape);
	// the two switches of a double switch and the plain track between them, found from any of the three
	struct double_switch
	{
		TTrack *first{nullptr};
		TTrack *link{nullptr};
		TTrack *second{nullptr};
	};
	std::optional<double_switch> double_of(TTrack const &Track) const;
	// the paths of the set in the order editor_track::place_switch gives them
	static std::vector<segment_data> double_paths(double_switch const &Set);
	// the switch of the template, or the two switches of a double one with the plain track between them, committed and recorded
	std::vector<TTrack *> create_switch_set(editor_track::switch_template const &Shape, std::vector<segment_data> const &Paths, TTrack const &Style, std::string const &Name);
	void record_slip(std::vector<TTrack *> const &Parts, std::string const &Kind, std::string const &Type, double const Radius, double const Angle, std::string const &Base, double const Middle = 0.0);
	void render_switch_record(TTrack const &Track) const;
	void route_store();
	bool digitize_drawn(std::vector<TTrack *> const &Created, bool const Alone = true) override;
	bool digitize_chain(editor_track::chain const &Chain, std::string *Reason = nullptr, TTrack **Culprit = nullptr);
	int digitize_parts(editor_track::chain const &Chain, std::string *Reason, int &Skipped);
	// the lines kept, the paths left out
	void digitize_all(int *Kept = nullptr, int *Failed = nullptr);
	// what the digitization of the scenery did, or would do
	struct digitize_report
	{
		int lines{0};
		int lines_failed{0};
		int switches{0};
		int known{0}; // kept in the scenery already
		int designed{0}; // of a template of the catalogue
		int measured{0};
		int slips{0};
		int unnamed{0}; // no record finds them
		int odd{0};
		int recorded{0};
		int crossovers{0};
		int crossovers_known{0};
		int crossovers_recorded{0};
		int crossovers_odd{0};
		int found{0};
		int bound{0};
		std::map<std::string, int> types; // of the catalogue
		std::map<std::string, int> shapes; // measured
		std::vector<std::string> notes;
	};
	// the switches without a type kept get the one of the catalogue they are of, or the size measured from their shape
	void digitize_switches(digitize_report &Report, bool const Write);
	// two switches facing each other from two tracks, their diverging tracks joined by a plain one, are kept as a crossover
	void digitize_crossovers(digitize_report &Report, bool const Write);
	// the lines, the switches, the crossovers and the objects along the tracks at one go; without Write only counts them
	std::string digitize_scenery(bool const Write);
	void render_route_stored();
	void route_update()
	{
		m_route.design.norms.checked = true;
		m_route.result = alignment::compute(m_route.design);
		++m_route.revision;
	}
	void route_apply();
	void route_lay();
	bool route_relay_through_switches(editor_track::chain const &Chain, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, TTrack *&Last);
	void route_recommend(alignment::vertex &Vertex) const;
	void draw_route_overlay() const;
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
		bool fit_has_before{false};
		bool fit_has_after{false};
		std::vector<std::pair<TTrack *, editor_track::state>> drag_states;
		std::size_t drag_step{0};
		glm::dvec3 preview_start{0.0};
		glm::dvec3 preview_end{0.0};
		glm::dvec3 edit_start{0.0};
		glm::dvec3 edit_end{0.0};
		double edit_length{0.0};
		double edit_azimuth{0.0};
		double distance{4.0};
		double curve_radius{0.0};
		bool auto_transitions{true};
		double transition{0.0};
		int tool{0};
		int tool_before{0};
		bool tool_mouse{false};
		std::vector<glm::dvec3> detour;
		editor_track::straight detour_line;
		glm::dvec3 detour_mouse{0.0};
		bool tool_placed{false};
		double tool_at{0.0};
		glm::dvec3 tool_point{0.0};
		glm::dvec3 tool_handle{0.0};
		editor_track::straight tool_line;
		std::string status;
	};
	void render_straights_ui();
	void draw_straights_overlay() const;
	editor_track::straight const &current_straight();
	bool straights_active();
	void render_straight_gizmo();
	void straight_apply(editor_track::straight const &Line, glm::dvec3 const &Start, glm::dvec3 const &End);
	void straights_apply(std::vector<editor_track::straight> const &Lines, std::vector<std::pair<glm::dvec3, glm::dvec3>> const &Ends);
	bool in_straight_set(editor_track::straight const &Line) const;
	bool place_straight_tool();
	void apply_straight_tool();
	double straight_tool_radius(editor_track::straight const &Line) const;
	void straight_reshape(editor_track::straight const &Line, double const From, double const To, std::function<glm::dvec3(glm::dvec3 const &)> const &Tail, double const Radius);
	void straight_refresh() override;
	bool start_curve_fit(editor_track::straight const &Line, int const Handle);
	void update_curve_fit(glm::dvec3 const &Start, glm::dvec3 const &End);
	void add_detour_point();
	void apply_detour();
	std::vector<glm::dvec3> detour_outline() const;
	void find_neighbour_straights();
	// listed spacing of a neighbouring straight the line can snap to, if it is close enough
	struct parallel_offer
	{
		bool nearby{false};
		bool snaps{false};
		double spacing{0.0};
		double distance{0.0};
		double angle{0.0}; // deg between the two straights
		double correction{0.0}; // m along the line normal to reach the spacing
		editor_track::straight const *neighbour{nullptr};
	};
	parallel_offer straight_parallel_offer(editor_track::straight const &Line, glm::dvec3 const &Offset) const;
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
		bool curved{false};
		std::vector<curve_sample> frame;
		std::vector<TTrack *> frame_tracks;
		std::vector<editor_track::switch_template> templates;
		bool collected{false};
		int armed{-1};
		int last{0}; // template armed the last time
		bool placing{false};
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
		};
		slide_state slide;
		double slide_typed{0.0};
		int connection{-1};
		int connection_kind{0}; // the kind of connection chosen last, taken again when the connections are chosen
		bool rail_only{true}; // the templates listed only for the rail of the selected track, where its rail profile tells it
		int connection_second{-1};
		double connection_radius{0.0};
		double connection_gap{6.0};
		bool connection_together{false};
		char connection_spacings[64]{"4.75;4.75;5.0"};
		double connection_stub{25.0};
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
	void insert_connection();
	void render_connection_options();
	// the name of a kind of connection of tracks, translated
	static char const *connection_name(int const Kind);
	// the name of a switch template for the lists: where it comes from and its name without the lengths
	static std::string switch_template_label(editor_track::switch_template const &Template);
	std::string connection_readout(connection::result const &Plan) const;
	editor_track::straight straight_near(glm::dvec3 const &Point, std::vector<TTrack const *> const &Exclude, double const Reach) const;
	editor_track::straight straight_through(glm::dvec3 const &Point, glm::dvec2 const &Direction) const;
	void record_connection(connection::kind const Kind, std::vector<TTrack *> const &Switches, connection::result const &Plan, editor_track::switch_template const &Shape);
	bool switch_room(TTrack &Track, double &Low, double &High, std::string &Reason);
	bool switch_vacate(TTrack &Track, glm::dvec3 const &From, glm::dvec3 const &To, double const Shift, editor_track::straight const &Line, editor_track::edit &Edit, std::string &Status);
	bool move_switch_paths(TTrack &Track, std::vector<segment_data> const &Paths, std::string &Status);
	std::optional<switch_branch> take_switch_branch(TTrack const &Track, editor_track::joint const &Joint, std::size_t const Index) const;
	// for the path of the switch the line leaves, as it'll lie
	bool plan_switch_branch(segment_data const &Path, editor_track::joint const &Joint, switch_branch &Branch) const;
	void lay_switch_branch(switch_branch const &Branch, editor_track::edit &Edit);
	// of the joints off the main straight of the switch, the neighbours in Skip left out
	std::vector<switch_branch> switch_branches(TTrack const &Track, std::vector<editor_track::joint> const &Joints, std::vector<TTrack const *> const &Skip = {}) const;
	// for the paths the switch has now; the joints of the branches which can be laid are handled, the others are left out. Gives the notes for the user
	std::string plan_switch_branches(TTrack const &Track, std::vector<editor_track::joint> const &Joints, std::vector<switch_branch> &Branches, std::vector<bool> &Handled) const;
	// room on the straight of the whole double switch along the axis of its first switch, and the length of the set
	bool double_room(double_switch const &Set, double &Low, double &High, double &Length, std::string &Reason);
	bool move_double_paths(double_switch const &Set, std::vector<segment_data> const &Paths, std::string &Status);
	bool slide_switch(TTrack &Track, double const Distance);
	bool switch_slides(TTrack const &Track) const;
	void switch_slide_start(TTrack &Track);
	void switch_slide_update();
	void switch_slide_finish();
	void draw_switch_slide() const;
	// reads the drives from the include bank once it changes; Wait: for the templates being looked for. true if it read them
	bool scan_switch_drives(bool Wait = true);
	std::string switch_name_for_new() const;
	bool place_switch_drive(TTrack &Switch);
	bool flip_switch_drive(TTrack &Switch);
	void advance_switch_name();
	void join_last_steps();
	bool render_switch_drive_choice();
	int m_turnout_template{0};
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
		editor_track::snap_target origin;
		editor_track::snap_target snap;
		glm::vec2 pressed{0.f};
		std::vector<TTrack *> line;
	};
	extend_tool m_extend;
	// what a point clicked while laying is: a bend of the straights, eased with a large curve, or a point along the middle of a curve
	// seen on the photo, the last two of these on the straight after it
	enum class lay_mode
	{
		straights,
		curve
	};
	// a curve traced on the photo as it's laid, how far the points clicked along it are from it
	struct lay_trace
	{
		std::vector<glm::dvec3> points;
		std::vector<double> offsets;
		double rms{0.0};
		double worst{0.0};
		std::size_t curves{0};
	};
	// the line the points clicked make: its design, the bends among its vertices, the curves traced
	struct lay_line_plan
	{
		alignment::design design;
		alignment::result shape;
		std::vector<char> corners;
		std::vector<lay_trace> traces;
		// the straights of a fixed direction the next bend slides along: from a free end, out of a curve traced
		std::vector<std::pair<glm::dvec3, glm::dvec3>> guides;
		// where the grade breaks from point to point: the bends kept as such, at the heights clicked; a curve traced, or bends made
		// one curve, goes at one grade from end to end
		std::vector<glm::dvec3> knots;
	};
	// lays new track through the clicked points, with no path needed to start from
	struct lay_tool
	{
		bool active{false};
		std::vector<glm::dvec3> points;
		std::vector<lay_mode> modes; // of the points, as each was clicked
		lay_mode mode{lay_mode::straights};
		int radii{0}; // of a curve traced: 0 as many as the points need, up to three
		bool straights_between{true}; // a curve traced may be two with a straight between them
		lay_line_plan plan; // of the line shown
		editor_track::snap_target start; // free end the track starts from, if any
		glm::dvec3 mouse{0.0};
		editor_track::snap_target mouse_snap;
		std::vector<segment_data> preview;
		std::string preview_error;
		double preview_length{0.0};
		double radius{500.0};
		double piece_length{100.0};
		bool copy_selected{false};
		editor_track::path_style style;
		bool own_speed{false}; // the speed of the style also where the track goes on from one with its own, the new tracks given it
		std::string status;
		double status_time{-100.0};
		std::string skew; // what Alt turned the straight to
		double escape_time{-100.0}; // when Esc was pressed over points not laid: the second Esc drops them
		struct step
		{
			editor_track::snap_target start;
			glm::dvec3 point{0.0};
		};
		std::vector<step> chain;
		std::size_t history{0}; // the step of the history the chain was begun at
	};
	lay_tool m_lay;
	alignment::limits laying_norms() const;
	glm::dvec3 lay_angle_snap(glm::dvec3 const &Mouse, std::string &Label) const;
	// the straight track Alt turns the laid straight to: the nearest one the straight from From to To crosses, or one along From
	TTrack const *skew_reference(glm::dvec3 const &From, glm::dvec3 const &To, double const Reach) const;
	void lay_after_history(bool const Undo);
	void lay_back();
	void lay_escape();
	std::size_t lay_pending() const; // the points clicked and not laid yet
	void render_lay_ui();
	void render_lay_status();
	void lay_click();
	void lay_finish(editor_track::snap_target const &End);
	void lay_cancel();
	void lay_enter();
	// Tab: the bends of straights or the curve traced, for the points clicked from now on
	void lay_switch_mode();
	// the points the same in number as their modes, a point with none a bend of straights
	void lay_trim();
	// the track through the points clicked, each the way its mode says; End: the free end it finishes at, the last point then
	std::vector<segment_data> lay_pieces(std::vector<glm::dvec3> const &Points, std::vector<lay_mode> const &Modes, editor_track::snap_target const &End, double &Length, std::string &Error,
	                                     lay_line_plan *Plan = nullptr) const;
	bool lay_line(std::vector<glm::dvec3> const &Points, std::vector<lay_mode> const &Modes, editor_track::snap_target const &End, lay_line_plan &Plan, std::string &Error) const;
	// the curves of a stretch traced, the ones fitted lately kept for the same points
	// none while the stretch the cursor moves along is still being fitted; Meanwhile: the fit done last of the same points but the one
	// at the cursor until then
	std::optional<alignment::trace_stretch> traced_stretch(alignment::design const &Base, std::vector<glm::dvec2> const &Points, int const Radii, bool const Straights, bool const Meanwhile = true) const;
	// the curve through a row of bends clicked close along one, by traced_stretch
	alignment::stretch_fit bends_fit() const
	{
		return [this](alignment::design const &Base, std::vector<glm::dvec2> const &Points) { return traced_stretch(Base, Points, 0, false, false); };
	}
	mutable std::vector<std::pair<std::vector<double>, alignment::trace_stretch>> m_stretch_cache;
	// the stretch the cursor moves along is fitted on a thread, the frames drawn meanwhile with the fit done last
	mutable alignment::stretch_fitter m_stretch_fitter;
	void render_switch_ui();
	void render_switch_timbers();
	void render_rail_profile_hint(TTrack &Track);
	bool start_switch_placement();
	void finish_switch_placement();
	std::vector<segment_data> switch_preview() const;
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
	switch_curve::switch_report curved_switch_report(std::vector<segment_data> const &Paths, double const Station, int const Direction) const;
	static double nearest_station(std::vector<curve_sample> const &Frame, glm::dvec3 const &Point);
	static std::vector<segment_data> curved_switch_paths(editor_track::switch_template const &Shape, std::vector<curve_sample> const &Frame, double const Station, int const Direction, int const Side);
	void insert_curved_switch(int const Direction, int const Side);
	// a crossover between two tracks running side by side along a curve (editormodeconnectioncurve.cpp)
	struct curve_mapping;
	static bool curve_of(TTrack &Track, editor_track::straight_tolerance const &Tolerance, double const Gauge, std::vector<curve_sample> &Frame, std::vector<TTrack *> &Tracks);
	static void cut_curve(std::vector<TTrack *> &Candidates, std::vector<curve_sample> const &Frame, glm::dvec3 const &First, glm::dvec3 const &Second, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed);
	void insert_curved_connection();
	struct curved_crossover
	{
		switch_store::record record;
		TTrack *first{nullptr};
		TTrack *second{nullptr};
		TTrack *insert{nullptr};
	};
	// the single crossover the switch is a part of, along a curve or straights
	std::optional<curved_crossover> crossover_of(TTrack const &Switch) const;
	bool take_out_crossover(curved_crossover const &Crossover, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed, TTrack *&First, TTrack *&Second);
	bool slide_curved_connection(TTrack &Switch, double const Distance);
	// the template a crossover was laid of, -1 if it's not among the templates
	int crossover_template(switch_store::record const &Record);
	// a crossover laid again from the press on one track to the release by the other, as by the mouse. false if it doesn't fit
	// Names: of the switch at the press and of the one at the release, kept when a crossover is laid again
	bool lay_crossover(int const Shape, glm::dvec3 const &Press, glm::dvec3 const &Release, double const Reach = 3.0, std::vector<std::string> const &Names = {});
	// after the grade line of the open route went on its paths: the line on the other side of each crossover of the route
	// takes the same change of the heights where it runs beside, and its design with it. returns: how many lines followed
	std::size_t follow_crossovers(std::vector<editor_track::route_sample> const &Before);
	bool place_switch_on_straight(editor_track::straight const &Line, editor_track::switch_template const &Shape, TTrack const *Style, double const Along, int const Direction, int const Side, bool const Snap, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed, std::string const &Name = {});
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
	// nearest the cursor, or the double slip there to be replaced
	struct slip_plan
	{
		std::string problem;
		std::string note;
		TTrack *replaces{nullptr};
		TTrack *other{nullptr};
		glm::dvec2 crossing{0.0};
		glm::dvec2 first{0.0, 1.0};
		glm::dvec2 direction{0.0, 1.0};
		double angle{0.0};
		double alongfirst{0.0};
		double half{0.0};
		editor_track::slip_curve curve;
		std::array<glm::dvec3, 4> points{};
	};
	bool plan_double_slip(editor_track::straight const &Line, glm::dvec3 const &Point, editor_track::switch_template const &Shape, slip_plan &Plan) const;
	// the cursor where it points at the straight, at the height of the straight there
	glm::dvec3 cursor_on_straight(editor_track::straight const &Line) const;
	bool insert_double_slip(editor_track::straight const &Line, glm::dvec3 const &Point, editor_track::switch_template const &Shape);
	bool replace_double_slip(TTrack &Part, editor_track::switch_template const &Shape);
	std::vector<TTrack *> build_double_slip(glm::dvec3 const &PointA, glm::dvec3 const &PointB, glm::dvec3 const &PointC, glm::dvec3 const &PointD, glm::dvec2 const &Crossing, double const Height, glm::dvec2 const &First, glm::dvec2 const &Second, double const Angle, editor_track::slip_curve const &Curve, TTrack &Style, std::string const &Type = "Rkpd");
	bool start_extend(editor_track::snap_target const &End);
	void finish_extend();
	std::vector<segment_data> extend_pieces() const;
	// the same for the track shown while it's dragged: its grade line fitted on the thread of the fitter, the frame not waiting for it
	std::vector<segment_data> extend_preview() const;
	// the free end of the path as the start of a new track; false if it isn't free or has no direction
	static bool free_end(TTrack &Track, int const Path, bool const Atend, extend_tool &Tool);
	// the free end new pieces are added at: the selected point if it's one, else the end, else the start
	bool chosen_free_end(extend_tool &Tool);
	// adds a straight (Side 0) or a curve to the left (1) or the right (-1) at the free end of the selected track
	void extend_add(int const Side);
	void render_extend_ui();
	using track_layout = editor_track::layout;
	track_layout m_track_layout{track_layout::arc_straight};
	double m_join_radius{0.0};
	int m_join_variant{0};
	TTrack const *m_join_track{nullptr};
	editor_track::point_ref m_join_point;
	mutable alignment::join_result m_join_last;
	// the layout laid on from the end of From with the transition curves of the norms; none where it doesn't fit
	std::optional<alignment::lay_result> eased_layout(TTrack const *From, glm::dvec3 const &Point, glm::dvec2 const &Direction, glm::dvec3 const &Target, double const Radius) const;
	mutable std::optional<alignment::lay_result> m_eased_last;
	mutable std::optional<std::pair<alignment::lay_request, std::optional<alignment::lay_result>>> m_eased_cache;
	bool track_wheel(double const Offset);
	// a touchpad sends fractions of a notch: a choice moves a step for each whole notch gathered, and only one in a gesture when
	// Gesture; a pause, another key or the other way starts over
	struct wheel_state
	{
		double rest{0.0};
		double time{-10.0};
		int keys{0};
		bool spent{false};
	};
	wheel_state m_wheel;
	int wheel_step(double const Offset, bool const Gesture);
	std::string track_shaping(bool const Joined) const;
	void render_layout_combo();
	double design_speed(TTrack const *From, TTrack const *To) const;
	// the track whose speed the design takes, none where it's the one set for the laying
	TTrack const *speed_track(TTrack const *From, TTrack const *To) const;
	// the tracks the laying under way goes on from and to
	std::pair<TTrack const *, TTrack const *> laying_ends() const;
	std::string design_speed_of(TTrack const *From, TTrack const *To) const;
	void give_own_speed(std::vector<TTrack *> const &Created) const;
	std::vector<segment_data> join_pieces(TTrack const *From, editor_track::point_ref const &Frompoint, glm::dvec3 const &Start, glm::dvec2 const &Direction, editor_track::snap_target const &End, double &Length,
	                                      std::string &Error) const;
	double m_extend_length{25.0};
	double m_extend_radius{300.0};
	bool extend_snap(editor_track::snap_target &Target) const;
	void update_build_tools();
	void draw_build_overlay() const;
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
		double quick_straight{0.0};
		double blade_length{5.0};
		double blade_gap{0.15};
		std::string status;
	};
	switch_designer m_switch_designer;
	void switch_designs_load();
	// the list of the switch tool built again with the own templates, keeping the armed one or arming the one whose label starts with Arm
	void switch_templates_refresh(std::string const &Arm = {});
	void render_switch_designer();
	bool m_track_window_open{false};
	// the inspector is pinned to the right edge of the screen, the profile strip to the bottom one, both resized from the inner edge
	void render_track_inspector();
	// the track tools in the tool options window, and the selected path in the inspector
	void render_track_tool_options();
	void render_track_selection();
	void show_track_tab(track_tab const Wanted) override;
	bool track_analysis_tab() const { return m_track_tab == track_tab::profile || m_track_tab == track_tab::speed || m_track_tab == track_tab::joints || m_track_tab == track_tab::infra; }
	bool track_shortcut(int const Key);
	bool track_busy() const; // something going on in the mode, which Esc drops before it leaves the mode
	std::string track_mode_name() const;
	void render_turnout_ui();
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
	void start_straight_gesture(int const Tool);
	void finish_straight_gesture();
	void toggle_straight_set(TTrack &Track);
	struct key_hint
	{
		char const *key;
		std::string action;
		char const *raw{nullptr}; // the key before it's translated, its gesture drawn from it
	};
	std::vector<key_hint> track_key_hints(bool const All) const;
	void render_track_guide();
	std::string track_readout() const;
	void draw_track_hints();
	// a scenery with no tracks yet: in the middle of the view, how the first one is laid
	void draw_empty_scenery_hint();
	// path under the cursor, outlined before a click so it's known which one the click takes
	using track_hover = ::track_hover;
	track_hover m_hover;
	track_hover track_under_cursor(TTrack const *Only = nullptr, float const Pixels = 0.0f) const;
	track_hover split_spot() const;
	void update_track_hover();
	void draw_track_lines() const;
	// what the colours of the tracks in the view tell
	enum class track_colouring { plain, radius, speed, grade };
	track_colouring m_track_colouring{track_colouring::plain};
	void draw_track_legend() const;
	// the radii and lengths of the curves and the names and types of the switches written by them, from above, as on a plan
	bool m_track_labels{true};
	void draw_track_labels() const;
	void draw_track_hover() const;
	struct track_intent
	{
		enum class kind { none, gizmo, select, point, box, straight_set, straight_break, detour, straight_tool, straight_handle, route_vertex, route_grip, switch_at, lay, lay_end, sweep, turntable_place, turntable_exit, turntable_fit, spread, extend_end, signal_place, signal_move, switch_slide, pick_style, fouling, measure, route_trace, kilometrage, srk_boundary };
		kind what{kind::none};
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
	void point_drag_finish();
	bool route_hit(int &Vertex, int &Grip) const;
	int straight_handle_hit();
	bool switch_reach(glm::dvec3 const &Ground, glm::dvec3 &Point);
	glm::dvec3 cursor_ground() const;
	glm::dvec3 cursor_level(double const Height) const;
	glm::dvec3 cursor_terrain(double const Height);
	double bed_height(glm::dvec3 const &Point, glm::dvec2 const &Direction, TTrack const *Style) const;
	double nearby_ground(glm::dvec3 const &Point) const;
	static std::pair<double, double> trackbed(TTrack const *Style);
	std::vector<double> bed_heights(std::vector<glm::dvec3> const &Points, std::vector<glm::dvec2> const &Directions, TTrack const *Style) const;
	void follow_ground(std::vector<segment_data> &Pieces, double const Height0, double const Grade0, double const Height1, double const Grade1, TTrack const *Style) const;
	ground_follow::grading laying_grading(TTrack const *Style) const;
	// the grade lines of the tracks shown while they're laid are fitted on a thread of their own, not to hold up the frames
	mutable ground_follow::fitter m_fitter;
	mutable bool m_follow_preview{false};
	glm::dvec3 lay_bed(glm::dvec3 const &Point);
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
	int track_property_index(std::string const &Key) const;
	std::size_t track_property_apply(int const Property, std::vector<TTrack *> const &Tracks, std::string const &Text);
	void track_field_settle();
	void track_textures_scan();
	// every texture and material under the folder of the textures, read on the first use of the picker
	void track_textures_index();
	// the picker of a texture of the rails (Kind 0) or of the trackbed (1), in an open combo. returns: true when one is chosen, its name in Chosen, "none" for no texture
	bool track_texture_picker(int const Kind, std::string const &Current, bool const Mixed, std::string &Chosen);
	// image of the texture of the name for the picker, 0 while there's none
	std::uint64_t track_texture_image(int const Kind, std::string const &Name);
	void render_track_properties(std::vector<TTrack *> const &Tracks);
	void render_track_set_header(std::vector<TTrack *> const &Tracks);
	void render_track_select_tools(std::vector<TTrack *> const &Tracks);
	void render_track_style(std::vector<TTrack *> const &Tracks);
	void copy_track_style(TTrack const &Track);
	std::size_t paste_track_style(std::vector<TTrack *> const &Tracks);
	// which groups of the properties are pasted or put on the tracks by a preset, in the order of the groups of editormodetrackprops.cpp
	using track_groups = std::array<bool, 11>;
	// the values of the properties of the chosen groups put on the tracks, in one step of the undo. returns: the number of the
	// tracks changed, none when no value is of the chosen groups
	std::optional<std::size_t> apply_track_values(std::vector<std::pair<std::string, std::string>> const &Values, track_groups const &Groups, std::vector<TTrack *> const &Tracks, char const *What);
	// presets of the properties of the tracks (editorTrackPresets), made from a track and put on the selected ones
	void render_track_presets(std::vector<TTrack *> const &Tracks);
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
	};
	track_values m_track_values;
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
	struct track_texture_use
	{
		material_handle material{null_handle};
		std::string name;
		int count{0};
	};
	std::array<std::vector<track_texture_use>, 2> m_track_textures; // of the rails, of the trackbed
	std::array<char, 128> m_track_texture_filter{};
	struct texture_index_entry
	{
		std::string name; // as the materials are named: relative to the folder of the textures, without the extension
		std::string lowered;
		bool material{false}; // there's a .mat file of the name
	};
	std::vector<texture_index_entry> m_track_texture_index; // sorted by the name
	bool m_track_texture_indexed{false};
	std::string m_track_texture_hovered; // the tile under the mouse in the picker, shown large above them
	// properties copied from a track, to be pasted on others
	struct track_style
	{
		std::string from;
		std::vector<std::pair<std::string, std::string>> values;
		// speed, environment, overhead line, track circuits, rails, trackbed, its shape, sleepers, ride and sound, frogs, sleeper placement
		track_groups groups{false, false, false, false, true, true, true, true, false, true, true};
		bool open{false};
		bool picking{false}; // the next click on a track in the view copies its properties
	};
	track_style m_track_style;
	struct track_presets_panel
	{
		int selected{0}; // id of the preset chosen in the inspector
		int nameid{0}; // the preset whose name is in the field
		char name[128]{};
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
	std::string missing_drive_variant(int const Family, bool const Rightswitch, bool const Driveright) const;
	bool replace_switch_drive(TTrack &Switch, int Family, bool Flip);
	void apply_switch_drive_choice(TTrack &Switch);
	void switch_drive_captured(TTrack const &Track);
	void switch_drives_committed(std::vector<TTrack *> const &Tracks);
	scene::layer_handle output_layer(std::string const &Part) override;
	// places them in the active layer or the given one, as one step of the undo. returns: how many were placed
	std::size_t place_templates(std::vector<template_item> const &Items, std::string &Error, scene::layer_handle Layer = scene::layer_handle{}) override;
	// moves them, as one step of the undo
	std::size_t move_templates(std::vector<std::pair<standing_template, template_item>> const &Moves) override;
	// takes them out, as one step of the undo
	std::size_t remove_templates(std::vector<scene::instance_handle> const &Instances) override;
	bool standing_signals(std::vector<standing_signal> &Signals, bool const Wait) override;
	std::size_t signal_batch(signal_changes const &Changes, std::vector<std::pair<TTrack *, editor_track::state>> States, std::vector<TTrack *> Created, std::string &Error) override;
	bool signal_heads(std::vector<signal_head> &Heads, bool const Wait) override;
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
	signal_standing const *signal_standing_of(scene::instance_handle const Instance) const;
	bool signal_delete(scene::instance_handle const Instance);
	void signal_detach_events(std::string const &Read, std::vector<std::pair<TTrack *, editor_track::state>> &States);
	void signal_renamed(std::string const &Before, std::string const &After, EditorSnapshot &Snapshot);
	void signal_select(scene::instance_handle const Instance);
	std::string signal_next_name(std::string const &Name) const;
	void render_signal_ui();
	void render_signal_palette();
	void render_signal_selected();
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
	void orphans_register(std::vector<TTrack *> const &Removed, std::vector<TTrack *> const &Kept);
	// Created: made by the step, their circuits the ones of the tracks they were made from; Changed: the others it changed
	void orphans_take_over(std::vector<TTrack *> const &Created, std::vector<TTrack *> const &Changed);

	// a model laid along the track: bent to follow it, or repeated along it
	struct sweep_tool
	{
		bool open{false};
		bool expand{false};
		char model[256]{};
		char skin[128]{"none"};
		char search[64]{};
		std::vector<std::string> models;
		bool scanned{false};
		int revision{-1}; // of the include bank the templates were taken from
		// parameters of a template laid along the track, with their names
		std::string parameters_for;
		std::vector<std::array<char, 128>> parameters;
		std::vector<std::string> parameter_labels;
		std::vector<bool> parameter_placement; // filled in with zeros: the template is placed in its own space
		std::vector<std::vector<std::string>> parameter_choices; // variants of the textures a parameter is a part of the name of
		int length_mode{2}; // 0: one copy as long as the original, 1: the length given, 2: the whole stretch of the line
		double length{200.0};
		sweep_node::state settings;
		int axis{0}; // 0: the longer one of the model, 1: z, 2: x
		double model_length{0.0};
		std::string measured; // model the length is of
		// the curve: the axis of the track
		TTrack *curve_for{nullptr};
		std::size_t curve_history{0};
		struct piece
		{
			TTrack *track{nullptr};
			int path{0};
			bool forward{true};
		};
		std::vector<piece> curve_pieces;
		std::vector<segment_data> curve;
		double curve_length{0.0};
		std::vector<glm::dvec3> outline; // of the curve, for the preview
		std::vector<double> stations;
		// marking the stretch in the view: press where it starts, the distance from the track sets the offset, drag to the end
		bool marking{false};
		bool dragging{false};
		double anchor{0.0}; // station the drag started at
		sweep_node *edited{nullptr};
		std::string status;
	};
	sweep_tool m_sweep;
	// paths the models along curves were laid on, so they follow the changes of the paths
	struct sweep_link
	{
		sweep_node *sweep{nullptr};
		std::size_t piece{0};
		TTrack *track{nullptr};
		int path{0};
		bool reversed{false};
	};
	std::vector<sweep_link> m_sweep_links;
	void sweep_curve();
	void sweep_outline();
	bool sweep_extend(bool const Atend, glm::dvec3 const &Cursor);
	double sweep_station(glm::dvec3 const &Point, double &Lateral) const;
	void sweep_scan_models();
	void sweep_measure();
	sweep_node::state sweep_definition() const;
	void sweep_create();
	void sweep_apply();
	void sweep_delete(sweep_node &Sweep);
	void sweep_edit(sweep_node &Sweep);
	std::vector<sweep_node *> sweeps_near(TTrack const &Track) const;
	bool sweep_press();
	void sweep_drag();
	void sweep_release();
	// the same at a point of the world: Anywhere takes the press however far from the track it is
	bool sweep_press_at(glm::dvec3 const &Point, bool const Anywhere);
	void sweep_drag_at(glm::dvec3 const &Point);
	void render_sweep_ui();
	void draw_sweep_overlay() const;
	void restore_sweeps(EditorSnapshot &Snapshot);
	void sweeps_captured(TTrack const &Track);
	void sweeps_committed(std::vector<TTrack *> const &Tracks);
	void sweeps_split(TTrack &Original, TTrack &Created);

	struct bend_source
	{
		TAnimModel *model{nullptr};
		scene::instance_handle instance{0};
		std::string file;
		std::string skin{"none"};
		std::vector<std::string> parameters;
		glm::dmat4 transform{1.0};
		glm::dvec3 scale{1.0};
		scene::layer_handle layer{null_handle};
	};
	struct bend_tool
	{
		int axis{0};
		bool turned{false};
		int along_anchor{1};
		int side_anchor{0};
		int height_anchor{0};
		glm::dvec3 point{0.0};
		bool picking{false};
		void const *tracks_for{nullptr};
		glm::dvec3 tracks_at{0.0};
		std::vector<std::pair<TTrack *, double>> tracks;
		int track{0};
		sweep_node *edited{nullptr};
		std::string signature;
		bool marker_valid{false};
		glm::dvec3 marker{0.0};
		std::string status;
		int dragging{0};
		sweep_node::state drag_state;
		glm::dvec3 drag_anchor{0.0};
		double drag_station{0.0};
		std::string drag_label;
		sweep_node *hover{nullptr};
		bool over_marker{false};
		std::string label;
		std::string details;
	};
	bend_tool m_bend;
	bool bend_drag_start(int const Mods);
	void bend_drag_update();
	sweep_node *sweep_under(glm::dvec3 const &Point) const;
	bool bend_click();
	bool bend_shortcut();
	bool bend_source_of_selection(bend_source &Source) const;
	void bend_find_tracks(glm::dvec3 const &Point, void const *Key);
	bool bend_definition(bend_source const &Source, TTrack &Track, sweep_node::state &State, std::vector<TTrack *> &Tracks);
	void bend_selection();
	void bend_edit(sweep_node::state const &State);
	void bend_reanchor(sweep_node::state &State, int const Side, int const Height, glm::dvec3 const &Point) const;
	double bend_fixed(sweep_node const &Sweep, glm::dvec3 const &Point) const;
	void bend_stretch(sweep_node const &Sweep, double const Fixed, double const Length, glm::dvec3 const &Point, sweep_node::state &State) const;
	void bend_pick(glm::dvec3 const &Point);
	glm::dvec3 bend_marker(sweep_node const &Sweep) const;
	void restore_bend(EditorSnapshot &Snapshot, bool const Undo);
	void render_bend();
	void draw_bend_overlay() const;

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
	selftest_result selftest_sleepers(std::string const &command, std::istringstream &words);
	selftest_result selftest_sweep(std::string const &command, std::istringstream &words);
	selftest_result selftest_track(std::string const &command, std::istringstream &words);
	selftest_result selftest_switch(std::string const &command, std::istringstream &words);
	selftest_result selftest_rails(std::string const &command, std::istringstream &words);
	selftest_result selftest_join(std::string const &command, std::istringstream &words);
	selftest_result selftest_signal(std::string const &command, std::istringstream &words);
	selftest_result selftest_terrain(std::string const &command, std::istringstream &words);
	selftest_result selftest_props(std::string const &command, std::istringstream &words);
	selftest_result selftest_monkey(std::string const &command, std::istringstream &words);
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

	// vertical profile (grade line) along a route
	struct profile_state
	{
		bool open{false};
		TTrack *from{nullptr};
		TTrack *to{nullptr};
		TTrack *picked_from{nullptr};
		TTrack *picked_to{nullptr};
		double run_length{std::numeric_limits<double>::infinity()}; // the whole line through the track clicked
		std::vector<TTrack *> followed; // the selection the route was last taken from
		editor_track::route route;
		std::vector<editor_track::route_sample> samples;
		std::vector<TTrack *> created; // the paths the last apply split off the route
		std::vector<double> terrain; // NaN where there's no terrain
		std::vector<double> model; // the ground of the terrain model (NMT) as measured; NaN where it's not known
		bool model_waiting{false}; // squares of the model under the route still to come
		double model_polled{0.0};
		// what the joined ends take their height and grade from, when it's a switch: shown on the plot
		std::string start_inherits, end_inherits;
		// depth of the formation under the grade line, the rail top, of the track of each sample: the formation, not the rails,
		// lies on the ground
		std::vector<double> beds;
		profile::line line;
		profile::context context;
		std::vector<profile::issue> issues;
		double origin{0.0}; // chainage of the start of the route
		double departure{0.0}; // largest difference between the grade line and the track
		profile::ground_fit fit;
		// the heightmap terrain led to the grade line by a modifier: formation under the track, slopes down or up to the ground
		struct earthworks
		{
			bool from_ballast{true};
			double depth{0.7}; // m, formation below the top of the rail
			double half_width{3.5}; // m, from the axis of the route to the edge of the formation
			double slope{1.5}; // horizontal run of the slopes per metre of height
			double reach{60.0}; // m, from the edge of the formation, where the slopes end even if they don't get to the ground
			double rounding{1.5}; // m, over which the slopes bend into the formation and into the ground
			bool generate{true};
			// chunks of terrain made under the route by the last shaping
			std::vector<std::pair<int, int>> generated;
			// the modifier of the terrain made by the last shaping
			std::string modifier;
		} earthworks;
		double view_from{0.0};
		double view_to{100.0};
		double view_centre{0.0};
		float exaggeration{20.0f};
		bool show_terrain{true};
		bool show_model{true};
		bool show_track{true};
		bool camera_follows{true};
		// the part of the route the plot shows: as last seen and when, and as the camera last showed it
		std::pair<double, double> view_seen{0.0, 0.0};
		double view_seen_time{0.0};
		std::pair<double, double> view_framed{0.0, 0.0};
		bool show_plan{true};
		int selected{-1};
		int dragging{-1};
		// where the point was grabbed: the mouse there, the point against the mouse; the point moves once the mouse does
		ImVec2 drag_press{0.0f, 0.0f};
		glm::dvec2 drag_grab{0.0};
		bool drag_moved{false};
		int curve_grip{-1};
		bool panning{false};
		bool changed{false};
		double hover{-1.0};
		// under the cursor in the strip
		int hover_point{-1};
		int hover_grip{-1};
		bool hover_line{false};
		int shown{-2}; // point in the side panel
		bool show_remarks{false};
		float strip_top{-1.0f}; // the top of the strip over the 3d view, -1 while it isn't shown
		int edits{-1}; // of the tracks, when the route was last read
		std::string status;
		std::string error;
	};
	profile_state m_profile;
	// grade lines designed in the editor, kept in the scenery files as "//$p" comment lines
	struct stored_profile
	{
		scene::layer_handle layer{null_handle};
		double length{0.0};
		glm::dvec3 start{0.0}, end{0.0}, middle{0.0}; // of the route, world space
		double origin{0.0};
		profile::line line;
		double departure{-1.0}; // of the track from the design when it was kept; -1 as kept by an older editor
	};
	std::vector<stored_profile> m_profiles;
	bool m_profiles_loaded{false};
	void profile_library_load();
	void profile_library_mark(std::vector<scene::layer_handle> Layers);
	// the tracks whose grade line was fitted, by name, with the shape they had then, kept as "//$f" comment lines: a track
	// changed since, by hand or taken back, isn't fitted any more
	struct fitted_mark
	{
		scene::layer_handle layer{null_handle};
		std::uint64_t shape{0};
	};
	std::map<std::string, fitted_mark> m_fitted;
	bool m_fitted_loaded{false};
	void fitted_load();
	void fitted_set(std::vector<TTrack *> const &Tracks);
	bool grade_fitted(TTrack const &Track);
	static std::uint64_t fitted_shape(TTrack const &Track);
	int profile_find(bool &Reversed) const;
	bool profile_restore();
	bool profile_restore_parts();
	// the route a kept design was laid over, as it runs now; none when its paths moved or are gone
	std::optional<editor_track::route> profile_stored_route(stored_profile const &Entry, TTrack **From = nullptr, TTrack **To = nullptr) const;
	void profile_store();
	void profile_forget();
	void profile_open_stored(std::size_t const Index);
	void profile_edited();
	// the radii of its neighbours were for the breaks of the grade it made, they follow the limits after it
	void profile_remove_point(int const Index);
	void render_profile_body();
	void render_profile_strip();
	void render_profile_header();
	void render_profile_empty();
	void render_profile_toolbar();
	void render_profile_points();
	void render_profile_source();
	bool render_profile_parameters();
	void render_profile_issues();
	void profile_open(TTrack *From, TTrack *To);
	void profile_open_run(TTrack &Track);
	void profile_take(editor_track::route Route, TTrack *From, TTrack *To);
	void profile_open_selection(std::vector<TTrack *> const &Tracks);
	void profile_follow_selection();
	void profile_resample();
	// the ground of the terrain model under the route, as it was measured before any earthworks
	void profile_sample_model();
	void profile_recognize();
	void profile_fit_ground();
	// depth of the foot of the ballast under the rail top, and its distance from the axis
	static std::pair<double, double> profile_ballast(TTrack const &Track);
	// false for a track with no trackbed of its own, as the switches of the older sceneries whose bed is an include
	static bool profile_own_bed(TTrack const &Track);
	void profile_shape_ground();
	std::pair<int, int> profile_generate_ground(std::vector<std::pair<double, double>> const &Beds, double const Reach);
	void profile_restore_ground();
	void render_profile_earthworks();
	void profile_check();
	// Terrain: the heightmap terrain follows the tracks changed. Bridges: stretches of the route over a bridge or a viaduct, the
	// paths over them cut off at their ends and set to the bridge environment
	void profile_apply(bool const Terrain = false, std::vector<profile::departure> const &Bridges = {}, double const Fewest = 0.0);
	// the terrain under the tracks changed by the last Edits steps of the history which changed tracks
	void profile_follow_terrain(std::size_t const Edits);
	void profile_fit_view();
	// the camera shows the part of the route the plot shows, once the plot stops moving
	void profile_follow_view();
	void profile_frame_view();
	void profile_after_undo();
	void profile_follow_edits();
	struct profile_view;
	void render_profile_canvas(float const Height);
	void profile_fit_exaggeration(profile_view const &View);
	void profile_navigate(profile_view const &View, bool const Hovered);
	bool profile_canvas_edit(profile_view const &View, bool const Hovered);
	void draw_profile_canvas(profile_view const &View, ImDrawList &Draw) const;
	void profile_canvas_tooltip() const;
	void render_profile_point();
	void draw_profile_overlay() const;

	// infrastructure along the track: objects bound to the paths follow their changes. bindings are kept in the
	// scenery files as "//$b" comment lines
	struct infra_candidate
	{
		infra::binding binding;
		double offset{0.0}; // sideways from the axis
		std::string reason;
		bool chosen{false};
		bool bound{false};
		TTrack *reader{nullptr};
		std::string event; // of that path
		std::string cell;
		bool loose{false};
		TTrack *nearer{nullptr};
		glm::dvec3 nearer_foot{0.0};
		bool unread{false};
	};
	struct infra_window
	{
		bool open{false};
		int scope{1}; // 0: the selected path, 1: the line through it, 2: the route of the vertical profile
		float reach{1500.0f};
		float corridor{8.0f};
		bool follow{true};
		bool show{true};
		std::vector<infra::rule> rules{infra::default_rules()};
		std::vector<TTrack *> tracks; // of the scope
		std::vector<infra_candidate> candidates;
		int filter{0};
		int read{0};
		int overruled{0};
		int unread{0};
		int elsewhere{0};
		int hovered{-1};
		std::string status;
		std::string error;
	};
	infra_window m_infra;
	std::vector<infra::binding> m_bindings;
	std::vector<std::pair<scene::layer_handle, std::string>> m_bindings_unresolved; // lines kept as they are
	bool m_bindings_loaded{false};
	std::vector<infra::object_state> m_infra_buffer; // states of the objects before the change in progress
	std::vector<std::pair<scene::instance_handle, std::string>> m_drive_buffer;
	bool m_infra_suspended{false};
	// the point objects which go with a path: the signals and the warning discs it reads, the SHP resonators, and the controls of the
	// block sections, the signs and the posts by the path nearest to them
	struct point_object
	{
		scene::instance_handle include{0};
		glm::dvec3 location{0.0};
		TTrack *reader{nullptr};
		TTrack *nearest{nullptr}; // of one no path reads
		infra::category group{infra::category::other};
	};
	std::vector<point_object> m_point_objects;
	std::size_t m_point_objects_revision{static_cast<std::size_t>(-1)};
	std::size_t m_point_objects_count{0};
	void infra_adopt(TTrack &Track);
	int infra_bound_count() const; // by the infra tool, not the ones a change of their track took along
	std::set<std::pair<scene::basic_node const *, scene::instance_handle>> m_infra_carried; // the objects their tracks moved since the tool cleared it
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

	void infra_rebase();
	void render_infra_search();
	void render_infra_candidates();
	void render_infra_bound();
	void draw_infra_overlay() const;

	bool m_route_dragging{false};
	alignment::design m_route_drag_from;
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
	void render_road_tool_options();
	// the change of the road tool
	void road_choose_tool(int const Tool);
	void render_road_selection();
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
	// Found: set for the points which have the ground under them
	std::vector<double> ground_heights(std::vector<glm::dvec3> const &Points, bool const Fresh, std::vector<char> *Found = nullptr) const;
	void push_road_snapshot(editor_road::record Record);
	void restore_road_snapshot(EditorSnapshot const &Snapshot, std::vector<EditorSnapshot> &Opposite, bool const Undo);
	road_tool m_roadtool;
};
