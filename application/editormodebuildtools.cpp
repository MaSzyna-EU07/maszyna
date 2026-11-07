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
