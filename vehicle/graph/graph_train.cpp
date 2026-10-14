/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/Train.h"
#include "vehicle/graph/graph_instance.h"
#include "vehicle/graph/blocks/cab.h"
#include "model/Model3d.h"
#include "utilities/Logs.h"

// cab of a graph vehicle: its controls and indicators are the cab blocks of the graph. the animated submodels are found
// in the models of the vehicle the same way as for the controls of the .mmd files, and registered with the "graph:<id>"
// labels, which the mouse input turns into the vehiclegraphcontrol command
namespace
{

TSubModel *find_submodel(TDynamicObject const &Vehicle, std::string const &Name)
{
	std::array<TModel3d *, 3> const sources{Vehicle.mdKabina, Vehicle.mdLowPolyInt, Vehicle.mdModel};
	for (auto *source : sources)
	{
		if (source == nullptr)
		{
			continue;
		}
		if (auto *submodel{source->GetFromName(Name)})
		{
			return submodel;
		}
	}
	return nullptr;
}

// whether a block placed in the specified cab ("cab" parameter: 0 any, 1, 2) belongs to the cab with the specified index
// placement from the cab parameter: 0 any cab, 1 or 2, -1 the engine room (cab index 0)
bool in_cab(int const Placement, int const Cabindex)
{
	return Placement == 0 || (Placement < 0 ? Cabindex == 0 : Placement == Cabindex);
}

user_command find_command(std::string const &Name)
{
	if (Name.empty())
	{
		return user_command::none;
	}
	auto const lookup{simulation::commandMap.find(Name)};
	return lookup != simulation::commandMap.end() ? lookup->second : user_command::none;
}

} // namespace

void TTrain::initialize_graph_controls(int const Cabindex)
{
	m_graphcommands.clear();
	auto *graph{mvOccupied->Graph.get()};
	if (graph == nullptr)
	{
		return;
	}
	// controls and indicators of the .mmd aren't used by graph vehicles
	clear_cab_controls();
	Cabine[Cabindex].ClearControls();
	m_controlmapper.clear();

	auto const &nodes{graph->definition().nodes()};
	for (std::size_t idx = 0; idx < nodes.size(); ++idx)
	{
		auto &block{graph->node_block(idx)};
		auto const &id{nodes[idx].id};
		if (auto *control{dynamic_cast<vgraph::cab_switch *>(&block)})
		{
			// commands of the simulator operate the control in every cab
			auto const bind = [&](std::string const &Name, int const Operation) {
				auto const command{find_command(Name)};
				if (command == user_command::none)
				{
					if (false == Name.empty())
					{
						ErrorLog("Bad vehicle graph: unknown command \"" + Name + "\" bound to control \"" + id + "\"");
					}
					return;
				}
				m_graphcommands[command].push_back({control, Operation});
			};
			bind(control->command(), 0);
			bind(control->key_increase(), 1);
			bind(control->key_decrease(), -1);

			if (control->submodel().empty() || false == in_cab(control->cab(), Cabindex))
			{
				continue;
			}
			auto *submodel{find_submodel(*DynamicObject, control->submodel())};
			if (submodel == nullptr)
			{
				ErrorLog("Bad model: failed to locate sub-model \"" + control->submodel() + "\" of control \"" + id + "\" in 3d model(s) of \"" + DynamicObject->name() + "\"",
				         logtype::model);
				continue;
			}
			// the animation is given in degrees or millimetres per position
			auto const rotate{control->animation() == vgraph::cab_switch::animation_type::rotate};
			auto const unit{rotate ? 360.0 : 1000.0};
			auto &gauge{Cabine[Cabindex].Gauge(-1)};
			gauge.AssignSounds(control->sound_increase(), control->sound_decrease(), control->sound_positions(), DynamicObject);
			gauge.Init(submodel, nullptr, rotate ? TGaugeAnimation::gt_Rotate : TGaugeAnimation::gt_Move, static_cast<float>(control->step() / unit),
			           static_cast<float>(control->offset() / unit), static_cast<float>(control->smoothing()), *control->display_state());
			gauge.AssignFloat(control->display_state());
			m_controlmapper.insert(gauge, "graph:" + id);
		}
		else if (auto *lamp{dynamic_cast<vgraph::cab_lamp *>(&block)})
		{
			if (lamp->submodel().empty() || false == in_cab(lamp->cab(), Cabindex))
			{
				continue;
			}
			auto &button{Cabine[Cabindex].Button(-1)};
			std::array<TModel3d *, 3> const sources{DynamicObject->mdKabina, DynamicObject->mdLowPolyInt, DynamicObject->mdModel};
			auto found{false};
			for (auto *source : sources)
			{
				if (button.Init(lamp->submodel(), source, false))
				{
					found = true;
					break;
				}
			}
			if (false == found)
			{
				ErrorLog("Bad model: failed to locate sub-model \"" + lamp->submodel() + "\" of lamp \"" + id + "\" in 3d model(s) of \"" + DynamicObject->name() + "\"",
				         logtype::model);
			}
			button.AssignBool(lamp->lit_state());
			button.AssignSounds(lamp->sound_on(), lamp->sound_off(), DynamicObject);
		}
		else if (auto *indicator{dynamic_cast<vgraph::cab_gauge *>(&block)})
		{
			if (false == in_cab(indicator->cab(), Cabindex))
			{
				continue;
			}
			// the needle turns from angle_min at min to angle_max at max
			auto const range{indicator->max() - indicator->min()};
			auto const scale{range != 0.0 ? (indicator->angle_max() - indicator->angle_min()) / 360.0 / range : 0.0};
			auto const offset{indicator->angle_min() / 360.0 - indicator->min() * scale};
			for (std::size_t needle = 0; needle < (indicator->two_needles() ? 2 : 1); ++needle)
			{
				auto const &name{indicator->submodel(needle)};
				if (name.empty())
				{
					continue;
				}
				auto *submodel{find_submodel(*DynamicObject, name)};
				if (submodel == nullptr)
				{
					ErrorLog("Bad model: failed to locate sub-model \"" + name + "\" of gauge \"" + id + "\" in 3d model(s) of \"" + DynamicObject->name() + "\"",
					         logtype::model);
					continue;
				}
				auto &gauge{Cabine[Cabindex].Gauge(-1)};
				gauge.Init(submodel, nullptr, TGaugeAnimation::gt_Rotate, static_cast<float>(scale), static_cast<float>(offset), static_cast<float>(indicator->smoothing()),
				           *indicator->value_state(needle));
				gauge.AssignFloat(indicator->value_state(needle));
			}
		}
	}
}

bool TTrain::graph_command(command_data const &Command)
{
	if (Command.command == user_command::vehiclegraphcontrol)
	{
		// mouse on a control of the graph, or the hardware desk
		auto *control{dynamic_cast<vgraph::cab_switch *>(mvOccupied->Graph->find_block(Command.payload))};
		if (control == nullptr)
		{
			return true;
		}
		auto const increase{Command.param1 >= 0.0};
		if (Command.action == GLFW_RELEASE)
		{
			control->release();
		}
		else if (control->behaviour() == vgraph::cab_switch::behaviour_type::multi)
		{
			// levers move with every press and repeat
			if (increase)
			{
				control->increase();
			}
			else
			{
				control->decrease();
			}
		}
		else if (Command.action == GLFW_PRESS)
		{
			control->press();
		}
		return true;
	}
	auto const lookup{m_graphcommands.find(Command.command)};
	if (lookup == m_graphcommands.end())
	{
		return false;
	}
	for (auto const &binding : lookup->second)
	{
		auto &control{*binding.control};
		if (Command.action == GLFW_RELEASE)
		{
			control.release();
			continue;
		}
		switch (binding.operation)
		{
		case 1:
		{
			control.increase();
			break;
		}
		case -1:
		{
			control.decrease();
			break;
		}
		default:
		{
			if (Command.action == GLFW_PRESS)
			{
				control.press();
			}
			break;
		}
		}
	}
	return true;
}

void TTrain::export_graph_values(dictionary_source &Dictionary) const
{
	auto const *graph{mvOccupied->Graph.get()};
	auto const &nodes{graph->definition().nodes()};
	for (std::size_t idx = 0; idx < nodes.size(); ++idx)
	{
		auto const *item{dynamic_cast<vgraph::cab_export const *>(&graph->node_block(idx))};
		if (item == nullptr || item->name().empty())
		{
			continue;
		}
		switch (item->kind())
		{
		case vgraph::cab_export::kind_type::integer:
		{
			Dictionary.insert(item->name(), static_cast<int>(std::lround(item->value())));
			break;
		}
		case vgraph::cab_export::kind_type::boolean:
		{
			Dictionary.insert(item->name(), item->value() > 0.5);
			break;
		}
		default:
		{
			Dictionary.insert(item->name(), item->value());
			break;
		}
		}
	}
}
