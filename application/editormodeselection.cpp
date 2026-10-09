/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// more models selected at once, the clipboard, groups and the menu at the cursor of the scenery editor

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"
#include "editor/editorFormat.hpp"

#include "model/AnimModel.h"
#include "rendering/renderer.h"
#include "scene/scenelayers.h"
#include "scene/scenenodegroups.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/translation.h"
#include "imgui/imgui.h"

#include <glm/gtx/rotate_vector.hpp>
#include <algorithm>
#include <map>

namespace
{

// the models of the selection can be grouped, copied and dropped onto the ground; tracks and other nodes are left out
TAnimModel *as_model(scene::basic_node *Node)
{
	return Node != nullptr ? dynamic_cast<TAnimModel *>(Node) : nullptr;
}

} // namespace

std::vector<scene::basic_node *> editor_mode::selected_nodes() const
{
	std::vector<scene::basic_node *> nodes;
	if (m_node == nullptr)
		return nodes;
	nodes.push_back(m_node);
	// the selection of more models is a thing of the surroundings
	if (ui()->mode() != nodebank_panel::MODIFY)
		return nodes;
	for (auto *node : m_selection)
		if (node != nullptr && std::find(nodes.begin(), nodes.end(), node) == nodes.end())
			nodes.push_back(node);
	return nodes;
}

std::vector<scene::basic_node *> editor_mode::selection_with_groups() const
{
	auto nodes{selected_nodes()};
	auto const count{nodes.size()};
	for (std::size_t index = 0; index < count; ++index)
	{
		// NOTE: the groups up to the first one stand for no group, see scene::basic_editor
		if (nodes[index]->group() <= 1)
			continue;
		for (auto *member : scene::Groups.group(nodes[index]->group()).nodes)
			if (std::find(nodes.begin(), nodes.end(), member) == nodes.end())
				nodes.push_back(member);
	}
	return nodes;
}

void editor_mode::select_node(scene::basic_node *Node, bool const Additive)
{
	if (false == Additive)
	{
		m_selection.clear();
		m_node = Node;
		ui()->set_node(m_node);
		return;
	}
	if (Node == nullptr)
		return;
	if (m_node == nullptr)
	{
		m_node = Node;
	}
	else if (Node == m_node)
	{
		// taken out; the one selected before it takes the gizmo
		if (m_selection.empty())
		{
			m_node = nullptr;
		}
		else
		{
			m_node = m_selection.back();
			m_selection.pop_back();
		}
	}
	else if (auto const found{std::find(m_selection.begin(), m_selection.end(), Node)}; found != m_selection.end())
	{
		m_selection.erase(found);
	}
	else
	{
		// the model selected last gets the gizmo
		m_selection.push_back(m_node);
		m_node = Node;
	}
	ui()->set_node(m_node);
	auto const count{selected_nodes().size()};
	if (count > 1)
		ui()->set_status(format(STR_C("%zu models selected; Shift+LMB adds or takes one out"), count));
}

void editor_mode::draw_selection_overlay()
{
	if (ui()->mode() != nodebank_panel::MODIFY || m_node == nullptr)
		return;
	auto const nodes{selection_with_groups()};
	if (nodes.size() < 2)
		return;
	// a ring around each model of the selection, the one with the gizmo brighter, the models of their groups dimmer
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	auto const direct{selected_nodes()};
	for (auto *node : nodes)
	{
		ImVec2 point;
		if (false == projection.project(node->location(), point))
			continue;
		auto const chosen{std::find(direct.begin(), direct.end(), node) != direct.end()};
		auto const color{node == m_node ? overlay_color::highlight : chosen ? overlay_color::selected : overlay_color::marked};
		drawlist->AddCircle(point, node == m_node ? 9.0f : 7.0f, color, 16, 2.0f);
	}
}

void editor_mode::selection_snapshots(EditorSnapshot::Action const Action)
{
	// one step of the history for all of them
	auto joined{false};
	for (auto *node : selection_with_groups())
	{
		push_snapshot(node, Action);
		if (false == m_history.empty())
			m_history.back().joined = joined;
		joined = true;
	}
}

void editor_mode::selection_follow(glm::dvec3 const &Moved, glm::vec3 const &Turned, glm::dvec3 const &Pivot)
{
	// a group is moved by any of its models, once; the one of m_node is moved with it already
	std::vector<scene::group_handle> moved;
	if (m_node != nullptr && m_node->group() > 1)
		moved.push_back(m_node->group());
	for (auto *node : selected_nodes())
	{
		if (node == m_node)
			continue;
		if (node->group() > 1)
		{
			if (std::find(moved.begin(), moved.end(), node->group()) != moved.end())
				continue;
			moved.push_back(node->group());
		}
		if (Turned != glm::vec3(0.0f))
		{
			// turned in place, then carried around the turning point
			m_editor.rotate(node, Turned, 0.0f);
			auto const location{Pivot + glm::rotateY(node->location() - Pivot, glm::radians<double>(Turned.y))};
			m_editor.translate(node, location, true);
		}
		if (Moved != glm::dvec3(0.0))
			m_editor.translate(node, node->location() + Moved, true);
	}
}

glm::dvec3 editor_mode::cursor_ground()
{
	return placement_on_ground(Camera.Pos + clamp_mouse_offset_to_max(GfxRenderer->Mouse_Position()));
}

void editor_mode::copy_selection()
{
	auto const nodes{selection_with_groups()};
	auto const *anchor{as_model(m_node)};
	if (anchor == nullptr)
	{
		ui()->set_status(STR("Select a model to copy"), true);
		return;
	}
	m_clipboard.clear();
	std::map<scene::group_handle, int> groups;
	for (auto *node : nodes)
	{
		auto *model{as_model(node)};
		if (model == nullptr)
			continue;
		clipboard_entry entry;
		model->export_as_text(entry.definition);
		entry.offset = model->location() - anchor->location();
		entry.angles = model->Angles();
		entry.scale = model->Scale();
		if (model->group() > 1)
			entry.group = groups.emplace(model->group(), static_cast<int>(groups.size())).first->second;
		m_clipboard.push_back(std::move(entry));
	}
	ui()->set_status(format(STR_C("%zu models copied; Ctrl+V puts them at the cursor"), m_clipboard.size()));
}

void editor_mode::paste_clipboard(glm::dvec3 const &Location)
{
	if (m_clipboard.empty())
	{
		ui()->set_status(STR("Nothing copied yet; Ctrl+C copies the selected models"), true);
		return;
	}
	// one step of the history, the way an array is: the copies are taken out together
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::Array;
	std::vector<TAnimModel *> made;
	std::map<int, std::vector<scene::basic_node *>> groups;
	for (auto const &entry : m_clipboard)
	{
		// NOTE: no names for the copies, same as with the brush and the array
		auto *model{simulation::State.create_model(entry.definition, std::string{}, Location + entry.offset)};
		if (model == nullptr)
			continue;
		model->location(Location + entry.offset);
		model->Angles(entry.angles);
		model->Scale(entry.scale);
		add_to_hierarchy(model);
		EditorSnapshot::array_copy copy;
		copy.model = model;
		copy.uuid = model->uuid;
		snap.copies.push_back(std::move(copy));
		snap.layer = model->layer();
		made.push_back(model);
		if (entry.group >= 0)
			groups[entry.group].push_back(model);
	}
	if (made.empty())
		return;
	m_history.push_back(std::move(snap));
	g_redo.clear();
	for (auto const &group : groups)
		if (group.second.size() > 1)
			scene::Groups.make(group.second);
	// the copies are the selection now
	m_selection.assign(made.begin() + 1, made.end());
	m_node = made.front();
	ui()->set_node(m_node);
	ui()->set_status(format(STR_C("%zu models pasted; Ctrl+Z takes them back"), made.size()));
}

void editor_mode::group_selection()
{
	auto nodes{selection_with_groups()};
	nodes.erase(std::remove_if(nodes.begin(), nodes.end(), [](scene::basic_node *Node) { return as_model(Node) == nullptr; }), nodes.end());
	if (nodes.size() < 2)
	{
		ui()->set_status(STR("Select two models or more to group them: Shift+LMB adds one to the selection"), true);
		return;
	}
	for (auto *node : nodes)
	{
		std::string reason;
		if (node->from_template() || false == scene::Layers.editable(node, &reason))
		{
			ui()->set_status("\"" + (node->name().empty() ? std::string{"(unnamed node)"} : node->name()) + "\" can't be grouped: " + reason, true);
			return;
		}
	}
	// a group is written to the scenery as one block, in the layer of the model with the gizmo
	for (auto *node : nodes)
		if (node->layer() != m_node->layer())
			scene::Layers.move(node, m_node->layer());
	scene::Groups.make(nodes);
	m_selection.clear();
	ui()->set_status(format(STR_C("%zu models grouped: a click on one selects them all; Ctrl+Shift+G takes the group apart"), nodes.size()));
}

void editor_mode::ungroup_selection()
{
	std::vector<scene::group_handle> groups;
	for (auto *node : selected_nodes())
	{
		if (node->group() <= 1 || std::find(groups.begin(), groups.end(), node->group()) != groups.end())
			continue;
		// the groups of the scenery templates are their includes, they can't be taken apart here
		auto const &members{scene::Groups.group(node->group()).nodes};
		if (std::any_of(members.begin(), members.end(), [](scene::basic_node const *Member) { return Member->from_template(); }))
			continue;
		groups.push_back(node->group());
	}
	if (groups.empty())
	{
		ui()->set_status(STR("The selected model isn't in a group"), true);
		return;
	}
	for (auto const group : groups)
		scene::Groups.dissolve(group);
	ui()->set_status(format(STR_C("%zu groups taken apart"), groups.size()));
}

void editor_mode::drop_selection_to_ground()
{
	// each model by itself, a group follows the ground under its models
	auto joined{false};
	for (auto *node : selection_with_groups())
	{
		auto const size{m_history.size()};
		snap_to_ground(node, true, joined);
		joined = joined || m_history.size() != size;
	}
}

void editor_mode::render_context_menu()
{
	if (m_contextopen)
	{
		ImGui::OpenPopup("##editorcontext");
		m_contextopen = false;
	}
	if (false == ImGui::BeginPopup("##editorcontext"))
		return;
	auto const selected{m_node != nullptr};
	auto const model{as_model(m_node) != nullptr};
	auto const count{selected_nodes().size()};
	if (count > 1)
		ImGui::TextDisabled(STR_C("%zu models selected"), count);
	else if (selected)
		ImGui::TextDisabled("%s", m_node->name().empty() ? "(unnamed node)" : m_node->name().c_str());
	else
		ImGui::TextDisabled("%s", STR_C("Nothing selected"));
	ImGui::Separator();
	auto const item = [](char const *Label, char const *Keys, bool const Enabled, char const *Tooltip) {
		auto const chosen{ImGui::MenuItem(Label, Keys, false, Enabled)};
		if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
			ImGui::SetTooltip("%s", Tooltip);
		return chosen;
	};
	if (item(STR_C("Copy"), "Ctrl+C", model, STR_C("Copies the selected models with their groups")))
		copy_selection();
	if (item(STR_C("Paste here"), "Ctrl+V", false == m_clipboard.empty(), STR_C("Puts the copied models here, around this point as they were around the selected one")))
		paste_clipboard(m_contextpoint);
	ImGui::Separator();
	if (item(STR_C("Group"), "Ctrl+G", model && count > 1, STR_C("The selected models move together from now on; a click on one selects them all")))
		group_selection();
	if (item(STR_C("Ungroup"), "Ctrl+Shift+G", selected && m_node->group() > 1, STR_C("The models of the group move each by itself again")))
		ungroup_selection();
	if (item(STR_C("Drop to the ground"), STR_C("End"), selected, STR_C("Puts each selected model, also each model of their groups, down onto the terrain or the object under it")))
		drop_selection_to_ground();
	ImGui::Separator();
	if (item(STR_C("Fly to the selected"), "F", selected, STR_C("The camera flies to the selected model and looks at it")))
		start_focus(m_node, 0.6);
	if (item(STR_C("Delete"), STR_C("Del"), selected, STR_C("Deletes the selected models")))
		delete_selected();
	ImGui::EndPopup();
}
