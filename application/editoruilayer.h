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
	void add_node_template(const std::string &desc);
	float rot_val();
	bool rot_from_last();
	functions_panel::rotation_mode rot_mode();
	const std::string *get_active_node_template(bool bypassRandom = false);
	nodebank_panel::edit_mode mode();
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
	// draws the gizmo settings in the toolset window (the gizmo state lives in the editor mode)
	void set_gizmo_options(std::function<void()> Renderer)
	{
		m_gizmooptions = std::move(Renderer);
	}

  private:
	// methods
	void render_mode_options(nodebank_panel::edit_mode const Mode);
	void render_header_sections();
	// members
	itemproperties_panel m_itempropertiespanel{"Node Properties", true}; // not a window of its own, drawn in the toolset window
	functions_panel m_functionspanel{"Functions", true}; // not a window of its own, its settings are drawn in the toolset tabs
	nodebank_panel m_nodebankpanel{"Toolset", true}; // main editor window: gizmo, node properties, edit modes and the node bank
	brush_object_list m_brushobjects;
	bool m_insertrandom{false}; // insert mode picks a random template from m_insertset
	model_set_ref m_insertset;
	std::function<void()> m_filloptions;
	std::function<void()> m_gizmooptions;
	scene::basic_node *m_node{nullptr}; // currently bound scene node, if any
};
