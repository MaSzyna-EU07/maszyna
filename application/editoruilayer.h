/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/uilayer.h"
#include "application/editoruipanels.h"

namespace scene
{

class basic_node;

}

struct ImGuiDockNode;
struct ImGuiTabBar;

class editor_ui : public ui_layer
{

  public:
	// constructors
	editor_ui();
	// methods
	// updates state of UI elements
	void update() override;
	void set_node(scene::basic_node *Node);
	// include of a scenery template to show in the node properties in place of a node; nullptr if there's none
	void set_include(include_selection *Include);
	void add_node_template(const std::string &desc);
	float rot_val();
	bool rot_from_last();
	functions_panel::rotation_mode rot_mode();
	const std::string *get_active_node_template(bool bypassRandom = false);
	nodebank_panel::edit_mode mode();
	void set_mode(nodebank_panel::edit_mode const Mode);
	float getSpacing();
	// node bank and model set access (set selection widgets, set contents)
	nodebank_panel &nodebank()
	{
		return m_nodebankpanel;
	}
	// rotation settings widgets, for tools drawn by the editor mode
	void render_rotation_controls();
	// draws the area fill settings in its node bank tab (the fill state lives in the editor mode)
	void set_fill_options(std::function<void()> Renderer)
	{
		m_filloptions = std::move(Renderer);
	}
	// the open track window takes the mouse over, whichever toolset tab is chosen
	void set_track(bool const Track)
	{
		m_track = Track;
	}
	// draws menus of the editor mode in the menu bar, past the ones of the user interface
	void set_menu_options(std::function<void()> Renderer)
	{
		m_menuoptions = std::move(Renderer);
	}
	// draws the tools of the editor mode in the toolbar under the menu
	void set_toolbar_options(std::function<void()> Renderer)
	{
		m_toolbaroptions = std::move(Renderer);
	}
	// draws the edit menu of the editor mode, next to the file menu
	void set_edit_menu(std::function<void()> Renderer)
	{
		m_editmenu = std::move(Renderer);
	}
	// draws the fields of work of the editor mode in a row above the toolbar
	void set_workspace_bar(std::function<void()> Renderer)
	{
		m_workspacebar = std::move(Renderer);
	}
	// draws the gizmo settings in the toolset window (the gizmo state lives in the editor mode)
	void set_gizmo_options(std::function<void()> Renderer)
	{
		m_gizmooptions = std::move(Renderer);
	}
	// draws the array settings in the toolset window (the array state lives in the editor mode)
	void set_array_options(std::function<void()> Renderer)
	{
		m_arrayoptions = std::move(Renderer);
	}
	void set_bend_options(std::function<void()> Renderer)
	{
		m_bendoptions = std::move(Renderer);
	}
	void expand_bend()
	{
		m_bendexpand = true;
	}
	// actions of the file menu, performed by the editor mode
	void set_file_actions(std::function<void()> Save, std::function<void()> Export);
	void set_new_scenery(std::function<void()> New) { m_newscenery = std::move(New); }
	void set_open_scenery(std::function<void()> Open) { m_openscenery = std::move(Open); }
	// part of the main window between the docked windows, where the 3d view is seen; for the overlays placed in it
	static glm::vec2 view_min()
	{
		return m_viewmin;
	}
	static glm::vec2 view_max()
	{
		return m_viewmax;
	}
	// tools of the track mode in the tool options window, the selected path in the inspector
	void set_track_options(std::function<void()> Tools, std::function<void()> Selection)
	{
		m_tracktools = std::move(Tools);
		m_trackselection = std::move(Selection);
	}
	// fields of work of the editor mode besides the edit modes, chosen in its toolbar: their tools are drawn in the tool options window,
	// what they have selected in the inspector. the terrain is shown over the track mode, the roads give way to it
	enum class workspace
	{
		none,
		roads,
		terrain,
		count_
	};
	void set_workspace(workspace const Workspace)
	{
		m_workspace = Workspace;
	}
	void set_workspace_options(workspace const Workspace, std::function<void()> Tools, std::function<void()> Selection = {})
	{
		m_workspacetools[static_cast<std::size_t>(Workspace)] = std::move(Tools);
		m_workspaceselection[static_cast<std::size_t>(Workspace)] = std::move(Selection);
	}
	// selection of a node in the scene window; Focus is set when the camera is to fly to it
	void set_scene_select(std::function<void(scene::basic_node *, bool)> Select)
	{
		m_sceneselect = std::move(Select);
	}
	// pins of the docks: an unpinned dock hides its windows in a strip at the edge of the main window, and they come out over the 3d view
	// when their tab in the strip is pointed at or clicked. the shortcut unpins all docks, or pins them back
	void toggle_docks()
	{
		m_dockstoggle = true;
	}
	// the unpinned docks are kept in imgui.ini along with the layout; the handler has to be there before the file is read
	static void register_settings();
	// shows outcome of an operation in the layers window
	void set_status(std::string const &Status, bool const Error = false);

  private:
	// methods
	void render_mode_options(nodebank_panel::edit_mode const Mode);
	// windows of the editor drawn by the user interface itself
	void render_tool_options();
	void render_inspector();
	void render_scene();
	void scene_rebuild();
	void scene_row(std::size_t const Entry);
	void render_menu_contents() override;
	void render_windows_menu() override;
	int palette() const override;
	void render_dockspace() override;
	void render_() override;
	// docks the windows of the editor in their default places
	void build_default_layout(unsigned int const Dockspace);
	// unpinned docks: at the start of the frame the windows are hidden, docked back or placed over the view, at its end the strips and pins are drawn
	void autohide_begin();
	void autohide_place();
	void autohide_strips();
	void autohide_pins();
	void autohide_show(unsigned int const Window, bool const Focus);
	void autohide_pin(std::size_t const Group);
	static void autohide_node(ImGuiDockNode *Node);
	static std::size_t autohide_group_of(unsigned int const Window);
	static void autohide_menu(ImGuiContext *Context, ImGuiDockNode *Node, ImGuiTabBar *TabBar);
	// members
	itemproperties_panel m_itempropertiespanel{"Node Properties", true}; // not a window of its own, drawn in the toolset window
	functions_panel m_functionspanel{"Functions", true}; // not a window of its own, its settings are drawn in the toolset tabs
	nodebank_panel m_nodebankpanel{"Node bank", true}; // templates to insert, paint or fill with
	layers_panel m_layerspanel{"Layers", true};
	includes_panel m_includespanel{"Include database", false};
	std::function<void()> m_save;
	std::function<void()> m_export;
	std::function<void()> m_newscenery;
	std::function<void()> m_openscenery;
	brush_object_list m_brushobjects;
	bool m_insertrandom{false}; // insert mode picks a random template from m_insertset
	model_set_ref m_insertset;
	std::function<void()> m_filloptions;
	std::function<void()> m_menuoptions;
	std::function<void()> m_toolbaroptions;
	std::function<void()> m_workspacebar;
	std::function<void()> m_editmenu;
	bool m_layoutreset{false}; // the default layout is to be built again on the next frame
	bool m_tooloptionsopen{true};
	bool m_inspectoropen{true};
	bool m_sceneopen{true};
	std::function<void(scene::basic_node *, bool)> m_sceneselect;
	std::function<void()> m_tracktools;
	std::function<void()> m_trackselection;
	int m_toolmode{-1}; // edit mode or field of work the tool options were drawn for the last time
	workspace m_workspace{workspace::none};
	std::array<std::function<void()>, static_cast<std::size_t>(workspace::count_)> m_workspacetools;
	std::array<std::function<void()>, static_cast<std::size_t>(workspace::count_)> m_workspaceselection;
	// scene window: the models of the scenery by layer and group, rebuilt when their number changes
	struct scene_entry
	{
		std::string uuid; // key in scene::Hierarchy, the node is looked up by it when drawn
		scene::layer_handle layer;
		scene::group_handle group;
		std::string name;
	};
	std::vector<scene_entry> m_sceneentries;
	std::size_t m_scenesize{static_cast<std::size_t>(-1)};
	char m_scenefilter[64]{};
	std::string m_scenefilterused;
	std::vector<std::size_t> m_scenematches;
	static glm::vec2 m_viewmin;
	static glm::vec2 m_viewmax;
	std::function<void()> m_gizmooptions;
	std::function<void()> m_arrayoptions;
	std::function<void()> m_bendoptions;
	bool m_bendexpand{false};
	scene::basic_node *m_node{nullptr}; // currently bound scene node, if any
	bool m_track{false};
	// windows of an unpinned dock, and the edge they're hidden at
	struct autohide_group
	{
		int edge{0}; // ImGuiDir
		float size{300.0f}; // width of the side docks, height of the top and bottom ones, in pixels
		bool first{false}; // the node was above or left of the other one in its split, and took this share of it
		float share{0.5f};
		int depth{0}; // of the column of docks the node was in, in the tree of the dockspace; outer docks are put back first
		std::vector<unsigned int> windows;
	};
	static std::vector<autohide_group> m_autohide;
	static unsigned int m_dockspace;
	unsigned int m_autohideshown{0}; // window of an unpinned dock shown over the view
	bool m_autohideappear{false}; // the shown window is to be brought forward
	bool m_autohidefocus{false}; // and given the focus, it was clicked in the strip
	unsigned int m_autohidehovered{0}; // tab in the strips under the cursor, and since when
	double m_autohidehovertime{0.0};
	double m_autohideleft{-1.0}; // when the cursor left the shown window, -1 while it's over it
	bool m_autohidestrip{false}; // the cursor was over a tab of the strips in the last frame
	std::vector<std::size_t> m_autohidepin; // groups to be docked back at the start of the next frame
	bool m_dockstoggle{false};
};
