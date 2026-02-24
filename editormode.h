/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "applicationmode.h"
#include "editormouseinput.h"
#include "editorkeyboardinput.h"
#include "Camera.h"
#include "sceneeditor.h"
#include "scenenode.h"

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
		enum class Action { Move, Rotate, Add, Delete, Other };

		Action action{Action::Other};
		std::string node_name;          // node identifier (basic_node::name())
		// direct pointer to node when available; used for in-memory undo/redo lookup
		scene::basic_node *node_ptr{nullptr};
		std::string serialized;         // full text for recreate (used for Add/Delete)
		glm::dvec3 position{0.0, 0.0, 0.0};
		glm::vec3 rotation{0.0f, 0.0f, 0.0f};
		UID uuid; // node UUID for reference, used as fallback lookup for deleted/recreated nodes

	};
	void push_snapshot(scene::basic_node *node, EditorSnapshot::Action Action = EditorSnapshot::Action::Move, std::string const &Serialized = std::string());

	std::vector<EditorSnapshot> m_history; // history of changes to nodes, used for undo functionality
	std::vector<EditorSnapshot> g_redo;
	// methods
	void update_camera(double const Deltatime);
	bool mode_translation() const;
	bool mode_translation_vertical() const;
	bool mode_rotationY() const;
	bool mode_rotationX() const;
	bool mode_rotationZ() const;
	bool mode_snap() const;

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
	glm::dvec3 m_focus_start_lookat{0.0,0.0,0.0};
	glm::dvec3 m_focus_target_lookat{0.0,0.0,0.0};
	double m_focus_time{0.0};
	double m_focus_duration{0.6};

	double fTime50Hz{0.0}; // bufor czasu dla komunikacji z PoKeys
	scene::basic_editor m_editor;
	scene::basic_node *m_node; // currently selected scene node
	bool m_takesnapshot{true}; // helper, hints whether snapshot of selected node(s) should be taken before modification
	bool m_dragging = false;
	glm::dvec3 oldPos;
	bool mouseHold{false};
	float kMaxPlacementDistance = 200.0f;
	static bool m_change_history;

	// UI/history settings
	int m_max_history_size{200};
	int m_selected_history_idx{-1};
	glm::dvec3 clamp_mouse_offset_to_max(const glm::dvec3 &offset);

	// focus camera smoothly on specified node
	void start_focus(scene::basic_node *node, double duration = 0.6);

	// hierarchy management
	void add_to_hierarchy(scene::basic_node *node);
	void remove_from_hierarchy(scene::basic_node *node);
	scene::basic_node* find_in_hierarchy(const std::string &uuid_str);
	scene::basic_node* find_node_by_any(scene::basic_node *node_ptr, const std::string &uuid_str, const std::string &name);

	// clear history/redo pointers that reference the given node (prevent dangling pointers)
	void nullify_history_pointers(scene::basic_node *node);
	void render_change_history();
};
