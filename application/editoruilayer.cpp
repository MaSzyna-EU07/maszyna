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

editor_ui::editor_ui()
{

	clear_panels();
	// bind the panels with ui object. maybe not the best place for this but, eh

	add_external_panel(&m_nodebankpanel);

	m_nodebankpanel.mode_options = [this](nodebank_panel::edit_mode const Mode) { render_mode_options(Mode); };
	m_nodebankpanel.header_sections = [this]() { render_header_sections(); };
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
		ImGui::TextDisabled("LMB: select node   F: focus   End: drop to ground   Del: delete");
		break;
	case nodebank_panel::COPY:
		ImGui::TextDisabled("LMB: copy the clicked model to the node bank");
		break;
	case nodebank_panel::ADD:
	{
		ImGui::TextDisabled("LMB: insert a model at the cursor");
		ImGui::Checkbox("Random model from set", &m_insertrandom);
		if (m_insertrandom)
		{
			m_nodebankpanel.set_combo("Set##insert", m_insertset, nullptr);
			auto const count{m_nodebankpanel.set_entries(m_insertset).size()};
			ImGui::TextDisabled("%zu templates in set%s", count, count == 0 ? ", the node bank selection is used" : "");
		}
		else
		{
			ImGui::TextDisabled("Inserts the template selected in the node bank");
		}
		m_functionspanel.render_controls();
		break;
	}
	case nodebank_panel::BRUSH:
		ImGui::TextDisabled("Hold LMB: paint models along the cursor path");
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
	if (ImGui::CollapsingHeader("Gizmo", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (m_gizmooptions)
			m_gizmooptions();
	}
	if (ImGui::CollapsingHeader("Node properties", ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::Indent();
		m_itempropertiespanel.render_body();
		ImGui::Unindent();
	}
	if (ImGui::CollapsingHeader("Layers"))
	{
		ImGui::Indent();
		render_layers();
		ImGui::Unindent();
	}
}

void editor_ui::render_layers()
{
	if (scene::Layers.empty())
	{
		// layers are established only when the scenery is loaded for editing, regular load skips the bookkeeping
		ImGui::TextDisabled("Scenery layers are available in an edit session.\nStart the simulator with: -edit <scenery file>");
		return;
	}

	ImGui::TextDisabled("visible, locked, layer file (click: make active)");

	std::pair<char const *, scene::layer_item> const itemlabels[] = {{"models", scene::layer_item::model},
	                                                                 {"tracks", scene::layer_item::track},
	                                                                 {"traction", scene::layer_item::traction},
	                                                                 {"power sources", scene::layer_item::powersource},
	                                                                 {"memory cells", scene::layer_item::memcell},
	                                                                 {"event launchers", scene::layer_item::launcher},
	                                                                 {"events", scene::layer_item::event},
	                                                                 {"vehicles", scene::layer_item::vehicle},
	                                                                 {"sounds", scene::layer_item::sound},
	                                                                 {"terrain shapes", scene::layer_item::shape},
	                                                                 {"lines", scene::layer_item::lines}};

	for (std::size_t idx = 1; idx <= scene::Layers.size(); ++idx)
	{
		auto const handle{static_cast<scene::layer_handle>(idx)};
		auto const &layer{scene::Layers.layer(handle)};
		auto const isactive{handle == scene::Layers.active()};

		ImGui::PushID(static_cast<int>(idx));
		// active layer receives new nodes, so it can't be hidden nor locked
		if (isactive)
		{
			ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
		}
		auto visible{layer.visible};
		if (ImGui::Checkbox("##visible", &visible) && false == isactive)
		{
			scene::Layers.visible(handle, visible);
		}
		if (ImGui::IsItemHovered())
		{
			ImGui::SetTooltip("%s", isactive ? "Active layer is always visible" : "Show or hide models, tracks and traction of the layer");
		}
		ImGui::SameLine();
		auto locked{layer.locked};
		if (ImGui::Checkbox("##locked", &locked) && false == isactive)
		{
			scene::Layers.locked(handle, locked);
		}
		if (ImGui::IsItemHovered())
		{
			ImGui::SetTooltip("%s", isactive ? "Active layer can't be locked" : "Nodes of locked layer can't be selected in the viewport");
		}
		if (isactive)
		{
			ImGui::PopStyleVar();
		}
		ImGui::SameLine();
		// layers are listed in the order the files were first opened, so indentation alone shows the include tree
		auto const label{std::string(2 * scene::Layers.depth(handle), ' ') + Bezogonkow(layer.name) + " (" + (layer.binary ? "binary terrain" : std::to_string(layer.item_count())) + ")"};
		if (ImGui::Selectable(label.c_str(), isactive))
		{
			scene::Layers.active(handle);
		}
		if (ImGui::IsItemHovered())
		{
			std::string content{isactive ? "Active layer, nodes created in the editor are placed here" : "Click to make this the active layer"};
			if (layer.binary)
			{
				content += "\nThe file wasn't parsed, its content was loaded from binary terrain file";
			}
			for (auto const &itemlabel : itemlabels)
			{
				auto const count{layer.items[static_cast<std::size_t>(itemlabel.second)]};
				if (count > 0)
				{
					content += "\n" + std::string{itemlabel.first} + ": " + std::to_string(count);
				}
			}
			ImGui::SetTooltip("%s", content.c_str());
		}
		ImGui::PopID();
	}
}

void editor_ui::render_rotation_controls()
{
	m_functionspanel.render_controls();
}

void editor_ui::set_node(scene::basic_node *Node)
{
	m_node = Node;
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
	return m_nodebankpanel.mode;
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