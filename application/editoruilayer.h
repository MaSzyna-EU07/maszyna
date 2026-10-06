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
	// actions of the file menu, performed by the editor mode
	void set_file_actions(std::function<void()> Save, std::function<void()> Export);
	// shows outcome of an operation in the layers window
	void set_status(std::string const &Status, bool const Error = false);

  private:
	// methods
	void render_mode_options(nodebank_panel::edit_mode const Mode);
	void render_header_sections();
	void render_menu_contents() override;
	// members
	itemproperties_panel m_itempropertiespanel{"Node Properties", true}; // not a window of its own, drawn in the toolset window
	functions_panel m_functionspanel{"Functions", true}; // not a window of its own, its settings are drawn in the toolset tabs
	nodebank_panel m_nodebankpanel{"Toolset", true}; // main editor window: gizmo, node properties, edit modes and the node bank
	layers_panel m_layerspanel{"Layers", true};
	includes_panel m_includespanel{"Include database", false};
	std::function<void()> m_save;
	std::function<void()> m_export;
	brush_object_list m_brushobjects;
	bool m_insertrandom{false}; // insert mode picks a random template from m_insertset
	model_set_ref m_insertset;
	std::function<void()> m_filloptions;
	std::function<void()> m_menuoptions;
	std::function<void()> m_gizmooptions;
	std::function<void()> m_arrayoptions;
	scene::basic_node *m_node{nullptr}; // currently bound scene node, if any
	bool m_track{false};
};
