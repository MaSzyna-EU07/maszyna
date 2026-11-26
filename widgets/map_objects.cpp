#include "stdafx.h"
#include "widgets/map_objects.h"
#include "simulation/simulation.h"
#include "model/AnimModel.h"
#include "utilities/Globals.h"
#include "vehicle/DynObj.h"
#include "world/EvLaunch.h"
#include "world/Track.h"

#include <algorithm>
#include <cctype>
#include <ranges>
#include <unordered_map>
#include <unordered_set>

map::map_objects map::MapObjects;

namespace {

struct scene_map_group
{
	std::vector<scene::basic_node *> nodes;
	std::vector<basic_event *> events;
};

bool event_belongs_to_signal(std::string const &event_name, std::string const &signal_name)
{
	if (!event_name.starts_with(signal_name))
		return false;
	if (event_name.size() == signal_name.size())
		return true;

	char const separator = event_name[signal_name.size()];
	return separator == '_' || separator == '-' || separator == '+' || separator == ':';
}

void collect_signal_models(
	std::vector<basic_event *> const &root_events,
	std::vector<TAnimModel *> &models)
{
	std::vector<basic_event *> pending_events(root_events);
	std::unordered_set<basic_event *> visited_events;
	std::unordered_set<TAnimModel *> visited_models;

	while (!pending_events.empty())
	{
		basic_event *event = pending_events.back();
		pending_events.pop_back();
		if (!event || !visited_events.insert(event).second)
			continue;

		if (auto *lights = dynamic_cast<lights_event *>(event))
		{
			for (scene::basic_node *target : lights->target_nodes())
			{
				auto *model = dynamic_cast<TAnimModel *>(target);
				if (model && model->iNumLights > 0 && !model->name().ends_with("w24") &&
				    visited_models.insert(model).second)
					models.push_back(model);
			}
			continue;
		}

		if (auto *multi = dynamic_cast<multi_event *>(event))
		{
			auto children = multi->children();
			pending_events.insert(pending_events.end(), children.begin(), children.end());
		}
	}
}

bool is_shp_event(basic_event const *event)
{
	if (!event)
		return false;

	std::string const &event_name = event->name();
	if (event_name.size() < 4)
		return false;

	std::size_t const suffix_start = event_name.size() - 3;
	return event_name[suffix_start - 1] == '_' &&
	       std::tolower(static_cast<unsigned char>(event_name[suffix_start])) == 's' &&
	       std::tolower(static_cast<unsigned char>(event_name[suffix_start + 1])) == 'h' &&
	       std::tolower(static_cast<unsigned char>(event_name[suffix_start + 2])) == 'p';
}

bool is_shp_launcher(basic_event const *first_event, basic_event const *second_event)
{
	return is_shp_event(first_event) || is_shp_event(second_event);
}

bool assign_cross_switch(map::track_switch_map_object &sw, std::string const &switch_name, std::string const &id, std::size_t index)
{
	sw.action[index] = simulation::Events.FindEvent(switch_name + ":" + id);
	if (!sw.action[index])
		sw.action[index] = simulation::Events.FindEvent(switch_name + id);
	if (!sw.action[index])
		return false;

	auto *multi = dynamic_cast<multi_event *>(sw.action[index]);
	if (!multi)
		return false;

	auto names = multi->dump_children_names();
	for (auto &name : names)
	{
		name = name.substr(switch_name.size());
		if (name.size() > 4)
			continue;

		auto const position_of = [&name](char const character) {
			auto const position = name.find_last_of(character);
			return position == std::string::npos ? -1 : static_cast<int>(position);
		};
		int const pos_a = position_of('a');
		int const pos_b = position_of('b');
		int const pos_c = position_of('c');
		int const pos_d = position_of('d');
		int const pos_0 = position_of('0');
		int const pos_1 = position_of('1');

		int position = 3;
		if (pos_a > pos_b && pos_a > pos_c && pos_a > pos_d)
			position = 0;
		else if (pos_b > pos_a && pos_b > pos_c && pos_b > pos_d)
			position = 1;
		else if (pos_c > pos_a && pos_c > pos_b && pos_c > pos_d)
			position = 2;
		sw.preview[index][position] = pos_0 > pos_1 ? '0' : '1';
	}
	return true;
}

template <typename NodeType>
void collect_group_nodes(
	std::unordered_map<scene::group_handle, scene_map_group> &groups,
	std::deque<NodeType *> const &items)
{
	for (NodeType *node : items)
	{
		if (node && node->group() != null_handle)
			groups[node->group()].nodes.push_back(node);
	}
}

} // namespace

void map::map_objects::prepare()
{
	MapObjects.clear();

	std::unordered_map<scene::group_handle, scene_map_group> groups;
	collect_group_nodes(groups, simulation::Instances.sequence());
	collect_group_nodes(groups, simulation::Memory.sequence());
	collect_group_nodes(groups, simulation::Paths.sequence());

	for (TEventLauncher *launcher : simulation::Events.launchers())
	{
		if (launcher && launcher->group() != null_handle)
			groups[launcher->group()].nodes.push_back(launcher);
	}
	for (basic_event *event : simulation::Events.sequence())
	{
		if (event && event->group() != null_handle)
			groups[event->group()].events.push_back(event);
	}

	std::unordered_map<std::string, std::shared_ptr<track_switch_map_object>> last_switch_map;
	for (auto const &group_entry : groups)
	{
		auto const &group = group_entry.second;
		std::vector<TAnimModel *> named_models;
		std::unordered_map<std::string, TMemCell *> signal_cells;
		for (scene::basic_node *node : group.nodes)
		{
			if (auto *model = dynamic_cast<TAnimModel *>(node); model && !model->name().empty())
				named_models.push_back(model);
			if (typeid(*node) == typeid(TMemCell) && node->name().ends_with("_sem_mem"))
			{
				std::string signal_name = node->name().substr(0, node->name().size() - 8);
				signal_cells.emplace(std::move(signal_name), static_cast<TMemCell *>(node));
			}
		}

		std::unordered_map<std::string, std::shared_ptr<semaphore_map_object>> signals_by_name;
		std::vector<std::shared_ptr<semaphore_map_object>> semaphore_signals;
		for (scene::basic_node *node : group.nodes)
		{
			if (typeid(*node) != typeid(TMemCell) || !node->name().ends_with("_sem_mem"))
				continue;

			std::string const signal_name = node->name().substr(0, node->name().size() - 8);
			TMemCell *memcell = signal_cells.at(signal_name);
			bool const derailer_model = std::ranges::any_of(
				named_models, [&signal_name](TAnimModel const *model) {
					return event_belongs_to_signal(model->name(), signal_name) &&
					       model->name().ends_with("_wk_el");
				});
			bool const has_signal_model = std::ranges::any_of(
				named_models, [&signal_name](TAnimModel const *model) {
					return event_belongs_to_signal(model->name(), signal_name) &&
					       !model->name().ends_with("_wk_el");
				});
			bool const has_shunting_board_model = std::ranges::any_of(
				named_models, [&signal_name](TAnimModel const *model) {
					if (!event_belongs_to_signal(model->name(), signal_name) || !model->Model())
						return false;
					std::string const model_path = model->Model()->NameGet();
					return model_path.find("sem/tm_") != std::string::npos ||
					       model_path.find("sem/karzelki/ktm") != std::string::npos ||
					       model_path.find("pkp/tmk_") != std::string::npos ||
					       model_path.ends_with("pkp/mk1un.t3d");
				});
			if (signal_name.ends_with("_wk_el") || (derailer_model && !has_signal_model))
				continue;

			auto signal = std::make_shared<semaphore_map_object>();
			signal->location = memcell->location();
			signal->memcell = memcell;
			signal->name = signal_name;
			signal->shunting =
				signal_name.starts_with("Tm") || signal_name.starts_with("tm_") ||
				has_shunting_board_model;
			signals_by_name.emplace(signal_name, signal);
			semaphore_signals.push_back(signal);
			MapObjects.push_back(std::move(signal));
		}

		for (auto const &signal : semaphore_signals)
		{
			std::vector<basic_event *> root_events;
			for (basic_event *event : group.events)
			{
				if (!event_belongs_to_signal(event->name(), signal->name))
					continue;
				if (event->name().substr(signal->name.size()).find("sem") != std::string::npos)
					continue;
				root_events.push_back(event);
			}

			std::vector<TAnimModel *> signal_models;
			collect_signal_models(root_events, signal_models);
			std::vector<TAnimModel *> ordered_signal_models;
			for (TAnimModel *model : named_models)
			{
				if (model->name().starts_with(signal->name))
					ordered_signal_models.push_back(model);
			}
			for (TAnimModel *model : signal_models)
			{
				if (std::ranges::find(ordered_signal_models, model) == ordered_signal_models.end())
					ordered_signal_models.push_back(model);
			}

			for (TAnimModel *model : ordered_signal_models)
			{
				if (model->name().ends_with("_wk_el"))
					continue;

				auto [signal_it, inserted] = signals_by_name.try_emplace(model->name());
				if (inserted)
				{
					auto repeated_signal = std::make_shared<semaphore_map_object>();
					repeated_signal->location = model->location();
					repeated_signal->name = model->name();
					repeated_signal->shunting =
						repeated_signal->name.starts_with("Tm") || repeated_signal->name.starts_with("tm_");
					if (auto const cell = signal_cells.find(repeated_signal->name); cell != signal_cells.end())
						repeated_signal->memcell = cell->second;
					MapObjects.push_back(repeated_signal);
					signal_it->second = std::move(repeated_signal);
				}

				auto &models = signal_it->second->models;
				if (std::ranges::find(models, model) == models.end())
					models.push_back(model);
				if (!signal_it->second->memcell)
					signal_it->second->location = model->location();
			}
		}

		for (auto const &[signal_name, signal] : signals_by_name)
		{
			for (basic_event *event : group.events)
			{
				if (!event_belongs_to_signal(event->name(), signal_name))
					continue;
				if (event->name().substr(signal_name.size()).find("sem") != std::string::npos)
					continue;
				if (std::ranges::find(signal->events, event) == signal->events.end())
					signal->events.push_back(event);
			}
		}

		for (scene::basic_node *node : group.nodes)
		{
			if (Global.map_manualswitchcontrol)
			{
				auto *track = dynamic_cast<TTrack *>(node);
				if (!track || track->eType != tt_Switch)
					continue;

				basic_event *switch_plus = simulation::Events.FindEvent(track->name() + "+");
				basic_event *switch_minus = simulation::Events.FindEvent(track->name() + "-");
				if (switch_plus && switch_minus)
				{
					auto map_switch = std::make_shared<track_switch_map_object>();
					MapObjects.push_back(map_switch);
					map_switch->location = node->location();
					map_switch->name = node->name();
					map_switch->action[0] = switch_plus;
					map_switch->action[1] = switch_minus;
					map_switch->track[0] = track;
					map_switch->preview[0][0] = '0';
					map_switch->preview[1][0] = '1';
					continue;
				}

				std::string switch_name = track->name();
				if (switch_name.size() <= 2)
					continue;
				char const last_character = switch_name.back();
				switch_name.pop_back();
				if (switch_name.back() == '_')
					switch_name.pop_back();
				if (!simulation::Events.FindEvent(switch_name + ":ac") &&
				    !simulation::Events.FindEvent(switch_name + "ac"))
					continue;

				if (last_character < 'a' || last_character > 'd')
					continue;
				auto [switch_it, inserted] = last_switch_map.try_emplace(switch_name);
				if (inserted)
				{
					switch_it->second = std::make_shared<track_switch_map_object>();
					switch_it->second->name = switch_name;
				}
				auto const &cross_switch = switch_it->second;
				cross_switch->track[last_character - 'a'] = track;
				if (!std::ranges::all_of(cross_switch->track, [](TTrack const *item) { return item != nullptr; }))
					continue;
				if (!assign_cross_switch(*cross_switch, switch_name, "ac", 0) ||
				    !assign_cross_switch(*cross_switch, switch_name, "ad", 1) ||
				    !assign_cross_switch(*cross_switch, switch_name, "bc", 2) ||
				    !assign_cross_switch(*cross_switch, switch_name, "bd", 3))
					continue;

				cross_switch->location = (cross_switch->track[0]->location() + cross_switch->track[1]->location() +
				                          cross_switch->track[2]->location() + cross_switch->track[3]->location()) / 4.0;
				MapObjects.push_back(cross_switch);
				last_switch_map.erase(switch_name);
				continue;
			}

			auto *launcher_node = dynamic_cast<TEventLauncher *>(node);
			if (!launcher_node || !launcher_node->Event1 || !launcher_node->Event2 ||
			    is_shp_launcher(launcher_node->Event1, launcher_node->Event2))
				continue;

			auto map_launcher = std::make_shared<event_launcher_map_object>();
			MapObjects.push_back(map_launcher);
			map_launcher->location = node->location();
			map_launcher->name = node->name();
			map_launcher->first_event = launcher_node->Event1;
			map_launcher->second_event = launcher_node->Event2;
			if (map_launcher->name.empty())
				map_launcher->name = launcher_node->Event1->name();
			map_launcher->type = launcher_node->Event1->name().find_first_of("-+:") != std::string::npos
			                         ? event_launcher_map_object::track_switch
			                         : event_launcher_map_object::level_crossing;
		}
	}
}

map::floats_map_objects_map map::map_objects::find_in_range(glm::vec3 from, float distance) const
{
	floats_map_objects_map items;

	float max_distance2 = distance * distance;

	for (auto const entry : MapObjects)
	{
		glm::vec3 entry_location = entry->location;
		glm::vec3 search_point = from;

		if (glm::isnan(from.y))
		{
			entry_location.y = 0.0f;
			search_point.y = 0.0f;
		}

		float dist = glm::distance2(entry_location, search_point);
		if (dist < max_distance2)
		{
			items.try_emplace(dist, entry);
		}
	}

	for (TDynamicObject *dynobj : simulation::Vehicles.sequence()) {
		if (!dynobj->Controller || dynobj->Prev())
			continue;

		glm::vec3 entry_location = dynobj->GetPosition();
		glm::vec3 search_point = from;

		if (glm::isnan(from.y))
		{
			entry_location.y = 0.0f;
			search_point.y = 0.0f;
		}

		float dist = glm::distance2(entry_location, search_point);
		if (dist < max_distance2)
		{
			auto entry = std::make_shared<map::vehicle_map_object>();
			entry->dynobj = dynobj;
			entry->name = dynobj->name();
			items.try_emplace(dist, std::move(entry));
		}
	}

	return items;
}
