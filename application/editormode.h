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
#include "input/editormouseinput.h"
#include "input/editorkeyboardinput.h"
#include "vehicle/Camera.h"
#include "scene/sceneeditor.h"
#include "scene/scenenode.h"
#include "editor/editorTerrain.hpp"
#include "editor/editorTerrainStreamer.hpp"

#include <memory>

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
	void on_scroll(double const Xoffset, double const Yoffset) override
	{
		;
	}
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
		enum class Action { Move, Rotate, Scale, Add, Delete, Other };

		Action action{Action::Other};
		std::string node_name;          // node identifier (basic_node::name())
		// direct pointer to node when available; used for in-memory undo/redo lookup
		scene::basic_node *node_ptr{nullptr};
		std::string serialized;         // full text for recreate (used for Add/Delete)
		glm::dvec3 position{0.0, 0.0, 0.0};
		glm::vec3 rotation{0.0f, 0.0f, 0.0f};
		glm::vec3 scale{1.0f, 1.0f, 1.0f};
		UID uuid; // node UUID for reference, used as fallback lookup for deleted/recreated nodes

	};
	void push_snapshot(scene::basic_node *node, EditorSnapshot::Action Action = EditorSnapshot::Action::Move, std::string const &Serialized = std::string());

	std::vector<EditorSnapshot> m_history; // history of changes to nodes, used for undo functionality
	std::vector<EditorSnapshot> g_redo;
	// methods
	void update_camera(double Deltatime);

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
	bool m_takesnapshot{true}; // helper, hints whether snapshot of selected node(s) should be taken before modification
	bool m_dragging = false;
	glm::dvec3 oldPos;
	bool mouseHold{false};
	float kMaxPlacementDistance = 200.0f;
	static bool m_change_history;
	static bool m_settings_open;

	// camera fly-mode (right mouse button held); used to flush motion when it's released
	command_relay m_camera_relay;
	bool m_camera_flying{false};

	// UI/history settings
	int m_max_history_size{200};
	int m_selected_history_idx{-1};
	glm::dvec3 clamp_mouse_offset_to_max(const glm::dvec3 &offset);

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
	// commits authored terrain to disk, enables streaming, and exports the scenery (Ctrl+S)
	void save_scene_with_terrain();
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

	// hierarchy management
	void add_to_hierarchy(scene::basic_node *node);
	void remove_from_hierarchy(scene::basic_node *node);
	scene::basic_node* find_in_hierarchy(const std::string &uuid_str);
	scene::basic_node* find_node_by_any(scene::basic_node *node_ptr, const std::string &uuid_str, const std::string &name);

	// clear history/redo pointers that reference the given node (prevent dangling pointers)
	void nullify_history_pointers(scene::basic_node *node);
	void render_change_history();
	void render_settings();

	// ImGuizmo-based transform gizmo for the selected node
	enum class gizmo_operation { translate, rotate, scale };
	void render_gizmo();
	bool m_gizmo_enabled{true};                                  // master switch for the in-viewport gizmo
	bool m_gizmo_using{false};                                   // tracks an ongoing drag, so a single undo snapshot is taken per drag
	bool m_gizmo_local{false};                                   // manipulate in the object's local space instead of world space
	gizmo_operation m_gizmo_op{gizmo_operation::translate};      // current transform mode (translate/rotate/scale)
	float m_gizmo_snap{1.0f};                                    // translation snap step (metres) applied while Ctrl is held
};
