/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "widgets/vehiclegraph.h"
#include "simulation/simulation.h"
#include "vehicle/Train.h"
#include "vehicle/graph/graph_catalog.h"
#include "vehicle/graph/graph_instance.h"
#include "utilities/translation.h"

namespace ui
{

vehiclegraph_panel::vehiclegraph_panel() : ui_panel(STR("Vehicle graph"), false)
{
	size_min = {400, 200};
}

void vehiclegraph_panel::render_contents()
{
	auto *train{simulation::Train};
	auto *graph{train != nullptr ? train->Dynamic()->MoverParameters->Graph.get() : nullptr};
	if (graph == nullptr)
	{
		ImGui::TextUnformatted(STR_C("The occupied vehicle isn't defined by a vehicle graph"));
		return;
	}
	auto const &definition{graph->definition()};
	auto const &body{graph->body()};
	ImGui::Text("%s  t = %.1f s", definition.name().c_str(), graph->time());
	ImGui::Text("Ft %.1f kN  Fb %.1f kN  I %.0f A", body.tractive_force / 1000.0, body.brake_force / 1000.0, body.current);
	ImGui::InputText(STR_C("Filter"), m_filterbuffer, sizeof(m_filterbuffer));
	m_filter = m_filterbuffer;
	ImGui::InputDouble(STR_C("Forced value"), &m_forcevalue);
	ImGui::Separator();

	auto const &nodes{definition.nodes()};
	for (std::size_t nodeindex = 0; nodeindex < nodes.size(); ++nodeindex)
	{
		auto const &node{nodes[nodeindex]};
		if (false == m_filter.empty() && node.id.find(m_filter) == std::string::npos && node.type->id.find(m_filter) == std::string::npos)
		{
			continue;
		}
		auto const label{node.id + " (" + node.type->id + ")" + (node.title.empty() ? "" : " - " + node.title)};
		if (false == ImGui::TreeNode(label.c_str()))
		{
			continue;
		}
		for (std::size_t portindex = 0; portindex < node.ports.size(); ++portindex)
		{
			auto const &port{node.ports[portindex]};
			auto const slot{node.slots[portindex]};
			auto const direction{port.direction == vgraph::port_direction::in ? "in " : "out"};
			if (port.dom == vgraph::domain::signal)
			{
				auto const forced{graph->is_forced(slot)};
				ImGui::Text("%s %-16s %12.4f %s", direction, port.key.c_str(), graph->signal(slot), port.unit.c_str());
				if (port.direction == vgraph::port_direction::out)
				{
					// forcing a signal output keeps it at the value regardless of its block
					ImGui::SameLine();
					auto const button{std::string{forced ? STR_C("release") : STR_C("force")} + "##" + node.id + "." + port.key};
					if (ImGui::SmallButton(button.c_str()))
					{
						if (forced)
						{
							graph->release(slot);
						}
						else
						{
							graph->force(slot, m_forcevalue);
						}
					}
				}
			}
			else
			{
				auto const &link{graph->link(slot)};
				auto const &domain{vgraph::info(port.dom)};
				ImGui::Text("%s %-16s %12.4f %s / %.4f %s", direction, port.key.c_str(), link.forward, std::string{domain.forward_unit}.c_str(), link.back,
				            std::string{domain.back_unit}.c_str());
			}
		}
		auto const &block{graph->node_block(nodeindex)};
		for (std::size_t stateindex = 0; stateindex < block.state_count(); ++stateindex)
		{
			ImGui::Text("    %-16s %12.4f", block.state_name(stateindex), block.state_value(stateindex));
		}
		ImGui::TreePop();
	}
}

} // namespace ui
