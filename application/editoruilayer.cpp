/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editoruilayer.h"

#include "utilities/Globals.h"
#include "utilities/utilities.h"
#include "scene/scenenode.h"
#include "scene/scenelayers.h"
#include "rendering/renderer.h"
#include "utilities/translation.h"
#include "imgui/imgui_internal.h"

glm::vec2 editor_ui::m_viewmin{0.0f};
glm::vec2 editor_ui::m_viewmax{0.0f};

editor_ui::editor_ui()
{

	clear_panels();
	// bind the panels with ui object. maybe not the best place for this but, eh

	add_external_panel(&m_nodebankpanel);
	add_external_panel(&m_layerspanel);
	add_external_panel(&m_includespanel);

	m_nodebankpanel.mode_options = [this](nodebank_panel::edit_mode const Mode) { render_mode_options(Mode); };
	m_nodebankpanel.header_sections = [this]() { render_header_sections(); };
	// the menu is a part of the layout of the editor, not a pop-up as in the driver mode
	m_menu_always = true;
}

void editor_ui::render_dockspace()
{
	// windows of the editor are docked around the 3d view, which is left free in the middle
	auto *viewport{ImGui::GetMainViewport()};
	auto const dockspace{ImGui::GetID("###editordockspace")};
	if (m_layoutreset || ImGui::DockBuilderGetNode(dockspace) == nullptr)
	{
		m_layoutreset = false;
		build_default_layout(dockspace);
	}
	ImGui::DockSpaceOverViewport(dockspace, viewport, ImGuiDockNodeFlags_PassthruCentralNode);
	if (auto const *central{ImGui::DockBuilderGetCentralNode(dockspace)}; central != nullptr)
	{
		m_viewmin = {central->Pos.x, central->Pos.y};
		m_viewmax = {central->Pos.x + central->Size.x, central->Pos.y + central->Size.y};
	}
	else
	{
		m_viewmin = {viewport->WorkPos.x, viewport->WorkPos.y};
		m_viewmax = {viewport->WorkPos.x + viewport->WorkSize.x, viewport->WorkPos.y + viewport->WorkSize.y};
	}
}

void editor_ui::build_default_layout(unsigned int const Dockspace)
{
	auto *viewport{ImGui::GetMainViewport()};
	ImGui::DockBuilderRemoveNode(Dockspace);
	ImGui::DockBuilderAddNode(Dockspace, ImGuiDockNodeFlags_DockSpace | ImGuiDockNodeFlags_PassthruCentralNode);
	ImGui::DockBuilderSetNodePos(Dockspace, viewport->WorkPos);
	ImGui::DockBuilderSetNodeSize(Dockspace, viewport->WorkSize);
	// widths in pixels the current windows need, as parts of the room there is
	auto const width{std::max(viewport->WorkSize.x, 1.0f)};
	auto const height{std::max(viewport->WorkSize.y, 1.0f)};
	auto const left_width{std::min(440.0f * Global.ui_scale, width * 0.3f)};
	auto const right_width{std::min(460.0f * Global.ui_scale, width * 0.3f)};
	auto const bottom_height{std::min(300.0f * Global.ui_scale, height * 0.35f)};
	ImGuiID center{Dockspace};
	auto const left{ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, left_width / width, nullptr, &center)};
	auto const right{ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, right_width / (width - left_width), nullptr, &center)};
	auto const bottom{ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, bottom_height / height, nullptr, &center)};
	// windows are found by the stable part of their names, after ###
	for (auto const *window : {"###Toolset", "###Layers", "###scenehierarchy"})
		ImGui::DockBuilderDockWindow(window, left);
	for (auto const *window : {"###trackinspector", "###roadeditor", "###editorsettings", "###Include database", "###orthophoto", "###structuregauge"})
		ImGui::DockBuilderDockWindow(window, right);
	for (auto const *window : {"###profilestrip", "###editorhistory", "###modelsets"})
		ImGui::DockBuilderDockWindow(window, bottom);
	ImGui::DockBuilderFinish(Dockspace);
}

void editor_ui::render_()
{
	auto *viewport{ImGui::GetMainViewport()};
	auto const flags{ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_MenuBar};
	// toolbar under the menu: the tools of the editor mode
	if (ImGui::BeginViewportSideBar("##editortoolbar", viewport, ImGuiDir_Up, ImGui::GetFrameHeight(), flags))
	{
		if (ImGui::BeginMenuBar())
		{
			if (m_toolbaroptions)
				m_toolbaroptions();
			ImGui::EndMenuBar();
		}
	}
	ImGui::End();
	// status bar: the outcome of the last operation, and the selected node
	if (ImGui::BeginViewportSideBar("##editorstatusbar", viewport, ImGuiDir_Down, ImGui::GetFrameHeight(), flags))
	{
		if (ImGui::BeginMenuBar())
		{
			if (false == m_layerspanel.status.empty())
			{
				if (m_layerspanel.status_error)
					ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.4f, 1.0f), "%s", m_layerspanel.status.c_str());
				else
					ImGui::TextUnformatted(m_layerspanel.status.c_str());
			}
			if (m_node != nullptr)
			{
				auto const name{m_node->name().empty() ? std::string{"(none)"} : m_node->name()};
				auto const label{std::string{STR_C("Selected:")} + " " + name};
				ImGui::SameLine(std::max(ImGui::GetCursorPosX(), ImGui::GetWindowWidth() - ImGui::CalcTextSize(label.c_str()).x - ImGui::GetStyle().ItemSpacing.x * 2.0f));
				ImGui::TextUnformatted(label.c_str());
			}
			ImGui::EndMenuBar();
		}
	}
	ImGui::End();
}

// updates state of UI elements
void editor_ui::update()
{

	set_tooltip("");

	if (Global.ControlPicking && DebugModeFlag)
	{
		const auto sceneryNode = GfxRenderer->Pick_Node();
		const std::string content = sceneryNode ? sceneryNode->tooltip() : "";
		set_tooltip(content);
	}

	ui_layer::update();
	m_itempropertiespanel.update(m_node);
	m_functionspanel.update(m_node);
}

void editor_ui::render_mode_options(nodebank_panel::edit_mode const Mode)
{
	switch (Mode)
	{
	case nodebank_panel::MODIFY:
		ImGui::TextDisabled(STR_C("LMB: select node   F: focus   End: drop to ground   Del: delete"));
		break;
	case nodebank_panel::COPY:
		ImGui::TextDisabled(STR_C("LMB: copy the clicked model to the node bank"));
		break;
	case nodebank_panel::ADD:
	{
		ImGui::TextDisabled(STR_C("LMB: insert a model at the cursor"));
		ImGui::Checkbox(STR_C("Random model from set"), &m_insertrandom);
		if (m_insertrandom)
		{
			m_nodebankpanel.set_combo(STR_C("Set##insert"), m_insertset, nullptr);
			auto const count{m_nodebankpanel.set_entries(m_insertset).size()};
			ImGui::TextDisabled(STR_C("%zu templates in set%s"), count, count == 0 ? ", the node bank selection is used" : "");
		}
		else
		{
			ImGui::TextDisabled(STR_C("Inserts the template selected in the node bank"));
		}
		m_functionspanel.render_controls();
		break;
	}
	case nodebank_panel::BRUSH:
		ImGui::TextDisabled(STR_C("Hold LMB: paint models along the cursor path"));
		m_brushobjects.render_options(m_nodebankpanel);
		m_functionspanel.render_controls();
		break;
	case nodebank_panel::FILL:
		if (m_filloptions)
			m_filloptions();
		break;
	default:
		break;
	}
}

void editor_ui::render_header_sections()
{
	if (ImGui::CollapsingHeader(STR_C("Gizmo"), ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (m_gizmooptions)
			m_gizmooptions();
	}
	if (ImGui::CollapsingHeader(STR_C("Node properties"), ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::Indent();
		m_itempropertiespanel.render_body();
		ImGui::Unindent();
	}
	if (ImGui::CollapsingHeader(STR_C("Array")))
	{
		if (m_arrayoptions)
			m_arrayoptions();
	}
	if (m_bendexpand)
	{
		ImGui::SetNextItemOpen(true);
		m_bendexpand = false;
	}
	if (ImGui::CollapsingHeader(STR_C("Bend along the track")))
	{
		if (m_bendoptions)
			m_bendoptions();
	}
}

void editor_ui::render_menu_contents()
{
	if (ImGui::BeginMenu(STR_C("File")))
	{
		if (ImGui::MenuItem(STR_C("New scenery...")) && m_newscenery)
			m_newscenery();
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("The editor starts again with the wizard of a new scenery: its name and its centre on the map"));
		if (ImGui::MenuItem(STR_C("Open scenery...")) && m_openscenery)
			m_openscenery();
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("The editor starts again with another scenery of the scenery folder"));
		// changes go to the scenery files only if the scenery was loaded with its sources tracked
		auto const editsession{false == scene::Layers.empty()};
		if (ImGui::MenuItem(STR_C("Save"), STR_C("Ctrl+S"), false, editsession) && m_save)
		{
			m_save();
		}
		if (false == editsession && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
		{
			ImGui::SetTooltip("%s", STR_C("Start the simulator with -edit <scenery file> to save changes to the scenery files"));
		}
		if (ImGui::MenuItem(STR_C("Export scenery dump"), STR_C("Ctrl+Shift+F11")) && m_export)
		{
			m_export();
		}
		if (ImGui::IsItemHovered())
		{
			ImGui::SetTooltip("%s", STR_C("Writes the whole scene to _export files next to the scenario, the scenery files are left as they are"));
		}
		ImGui::EndMenu();
	}

	ui_layer::render_menu_contents();

	if (ImGui::BeginMenu(STR_C("Mode windows")))
	{
		if (ImGui::MenuItem(STR_C("Reset window layout")))
			m_layoutreset = true;
		ImGui::Separator();
		ImGui::MenuItem(STR_C("Toolset"), nullptr, &m_nodebankpanel.is_open);
		ImGui::MenuItem(STR_C("Layers"), nullptr, &m_layerspanel.is_open);
		ImGui::MenuItem(STR_C("Include database"), nullptr, &m_includespanel.is_open);
		ImGui::EndMenu();
	}

	if (m_menuoptions)
		m_menuoptions();
}

void editor_ui::set_file_actions(std::function<void()> Save, std::function<void()> Export)
{
	m_save = Save;
	m_export = std::move(Export);
	m_layerspanel.save = std::move(Save);
}

void editor_ui::set_status(std::string const &Status, bool const Error)
{
	m_layerspanel.status = Status;
	m_layerspanel.status_error = Error;
}

void editor_ui::render_rotation_controls()
{
	m_functionspanel.render_controls();
}

void editor_ui::set_node(scene::basic_node *Node)
{
	m_node = Node;
	m_itempropertiespanel.update(m_node);
	m_functionspanel.update(m_node);
}

void editor_ui::set_include(include_selection *Include)
{
	m_itempropertiespanel.include(Include);
}

void editor_ui::add_node_template(const std::string &desc)
{
	m_nodebankpanel.add_template(desc);
}

std::string const *editor_ui::get_active_node_template(bool bypassRandom)
{
	if (!bypassRandom)
	{
		// random pick from the active tool's set; an empty set falls back to the node bank selection
		std::string const *pick{nullptr};
		if (m_nodebankpanel.mode == nodebank_panel::BRUSH && m_brushobjects.useRandom)
		{
			if (m_brushobjects.source.kind == model_set_ref::source::manual)
				pick = m_brushobjects.Objects.empty() ? nullptr : m_brushobjects.GetRandomObject();
			else
				pick = m_nodebankpanel.random_template(m_brushobjects.source);
		}
		else if (m_nodebankpanel.mode == nodebank_panel::ADD && m_insertrandom)
		{
			pick = m_nodebankpanel.random_template(m_insertset);
		}
		if (pick != nullptr)
			return pick;
	}
	return m_nodebankpanel.get_active_template();
}

nodebank_panel::edit_mode editor_ui::mode()
{
	return m_track ? nodebank_panel::TRACK : m_nodebankpanel.mode;
}
void editor_ui::set_mode(nodebank_panel::edit_mode const Mode)
{
	m_nodebankpanel.requested_mode = Mode;
	m_nodebankpanel.mode = Mode;
	m_nodebankpanel.is_open = true;
}
float editor_ui::getSpacing()
{
	return m_brushobjects.spacing;
}

functions_panel::rotation_mode editor_ui::rot_mode()
{
	return m_functionspanel.rot_mode;
}
float editor_ui::rot_val()
{
	return m_functionspanel.rot_value;
}
bool editor_ui::rot_from_last()
{
	return m_functionspanel.rot_from_last;
}