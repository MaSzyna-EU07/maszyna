/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editoruilayer.h"
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorSwitchStore.hpp"
#include "editor/editorGeometry.hpp"

#include "scene/scene.h"
#include "simulation/simulation.h"
#include "world/Track.h"
#include "utilities/translation.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <map>
#include <sstream>

namespace
{

using geometry::cross;
using geometry::plan_of;

char const *connection_name(int const Kind)
{
	switch (static_cast<connection::kind>(Kind))
	{
	case connection::kind::double_crossover: return STR_C("Double crossover");
	case connection::kind::scissors: return STR_C("Scissors crossover");
	case connection::kind::ladder: return STR_C("Switch ladder");
	default: return STR_C("Crossover");
	}
}

std::vector<double> spacings_of(char const *Text)
{
	std::string text{Text};
	for (auto &letter : text)
		if (letter == ';')
			letter = ' ';
		else if (letter == ',')
			letter = '.';
	std::istringstream words{text};
	std::vector<double> result;
	double value{0.0};
	while (words >> value)
		result.push_back(value);
	return result;
}

std::string switch_label(editor_track::switch_template const &Template)
{
	if (Template.source != nullptr)
		return STR("Scenery: ") + Template.label;
	return (Template.own ? STR("Own: ") : std::string{"PLK: "}) + Template.label;
}

bool usable_in_connection(editor_track::switch_template const &Template)
{
	return false == Template.double_slip && false == Template.twofold();
}

} // namespace

editor_track::straight editor_mode::straight_near(glm::dvec3 const &Point, std::vector<TTrack const *> const &Exclude, double const Reach) const
{
	TTrack *found{nullptr};
	auto best{Reach};
	for (auto *section : simulation::Region->sections(Point, static_cast<float>(std::max(40.0, Reach))))
		for (auto const &cell : section->m_cells)
			for (auto *track : cell.m_paths)
			{
				if (track->eType != tt_Normal || track->m_editorremoved || track->m_paths.empty() || std::find(Exclude.begin(), Exclude.end(), track) != Exclude.end() || false == editor_track::is_straight(*track, m_straights.tolerance))
					continue;
				auto const &path{track->m_paths.front()};
				auto const a{plan_of(path.points[segment_data::point::start])};
				auto const segment{plan_of(path.points[segment_data::point::end]) - a};
				auto const t{std::clamp(glm::dot(plan_of(Point) - a, segment) / std::max(1e-9, glm::dot(segment, segment)), 0.0, 1.0)};
				auto const distance{glm::distance(plan_of(Point), a + segment * t)};
				if (distance < best)
				{
					best = distance;
					found = track;
				}
			}
	return found != nullptr ? editor_track::find_straight(*found, m_straights.tolerance) : editor_track::straight{};
}

editor_track::straight editor_mode::straight_through(glm::dvec3 const &Point, glm::dvec2 const &Direction) const
{
	for (auto *section : simulation::Region->sections(Point, 40.f))
		for (auto const &cell : section->m_cells)
			for (auto *track : cell.m_paths)
			{
				if (track->eType != tt_Normal || track->m_editorremoved || track->m_paths.empty() || false == editor_track::is_straight(*track, m_straights.tolerance))
					continue;
				auto const &path{track->m_paths.front()};
				auto const a{plan_of(path.points[segment_data::point::start])};
				auto const segment{plan_of(path.points[segment_data::point::end]) - a};
				auto const length{glm::length(segment)};
				if (length < 1e-6 || std::abs(cross(segment / length, Direction)) > 1e-3 || std::abs(cross(segment / length, plan_of(Point) - a)) > 0.02)
					continue;
				auto const t{glm::dot(plan_of(Point) - a, segment) / (length * length)};
				if (t >= -1e-6 && t <= 1.0 + 1e-6)
					return editor_track::find_straight(*track, m_straights.tolerance);
			}
	return {};
}

bool editor_mode::plan_connection(connection::request &Request, connection::result &Result) const
{
	auto const &tool{m_switch};
	Result = {};
	if (false == tool.placing || tool.curved || tool.connection < 0 || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()) || tool.line.tracks.empty())
		return false;
	auto const &line{tool.line};
	Request = {};
	Request.type = static_cast<connection::kind>(tool.connection);
	Request.from = connection::of(line);
	Request.first = tool.templates[tool.armed];
	auto const second{tool.connection_second >= 0 && tool.connection_second < static_cast<int>(tool.templates.size()) ? tool.connection_second : tool.armed};
	Request.second = tool.templates[second];
	Request.radius = tool.connection_radius;
	Request.gap = tool.connection_gap;
	Request.together = tool.connection_together;
	Request.stub = tool.connection_stub;
	Request.spacings = spacings_of(tool.connection_spacings);
	Request.along = tool.along;
	glm::dvec2 const offset{plan_of(tool.mouse - tool.point)};
	Request.forward = glm::dot(offset, line.direction) >= 0.0 ? 1 : -1;
	glm::dvec2 const travel{line.direction * static_cast<double>(Request.forward)};
	Request.side = cross(travel, offset) >= 0.0 ? 1 : -1;
	std::vector<TTrack const *> const exclude(line.tracks.begin(), line.tracks.end());
	auto const other{straight_near(tool.mouse, exclude, 8.0)};
	if (false == other.tracks.empty())
	{
		auto const target{connection::of(other)};
		if (Request.type != connection::kind::ladder)
		{
			Request.to.push_back(target);
			Request.target = target.along(tool.mouse);
		}
		else if (std::abs(cross(target.direction, line.direction)) < 1e-3)
		{
			auto const side{cross(travel, plan_of(target.point(target.along(tool.point)) - tool.point)) > 0.0 ? 1.0 : -1.0};
			auto const lateral = [&](glm::dvec3 const &Point) { return side * cross(travel, plan_of(Point - tool.point)); };
			auto const outermost{lateral(target.point(target.along(tool.point)))};
			std::map<long, std::pair<double, TTrack *>> fan;
			auto const better = [](double const One, double const Two) { return (One < 0.0) != (Two < 0.0) ? One >= 0.0 : std::abs(One) < std::abs(Two); };
			auto const centre{(tool.point + tool.mouse) * 0.5};
			for (auto *section : simulation::Region->sections(centre, static_cast<float>(glm::length(offset) * 0.5 + 200.0)))
				for (auto const &cell : section->m_cells)
					for (auto *track : cell.m_paths)
					{
						if (track->eType != tt_Normal || track->m_editorremoved || track->m_paths.empty() || std::find(exclude.begin(), exclude.end(), track) != exclude.end() || false == editor_track::is_straight(*track, m_straights.tolerance))
							continue;
						auto const &path{track->m_paths.front()};
						auto const a{path.points[segment_data::point::start]};
						auto const b{path.points[segment_data::point::end]};
						auto const direction{plan_of(b - a)};
						if (glm::length(direction) < 1e-6 || std::abs(cross(glm::normalize(direction), line.direction)) > 1e-3)
							continue;
						auto const distance{lateral(a)};
						auto const along1{glm::dot(travel, plan_of(a - tool.point))};
						auto const along2{glm::dot(travel, plan_of(b - tool.point))};
						if (distance < 1.0 || distance > outermost + 0.05 || std::max(along1, along2) <= 0.0)
							continue;
						auto const nearer{std::min(along1, along2)};
						auto &entry{fan[std::lround(distance * 20.0)]};
						if (entry.second == nullptr || better(nearer, entry.first))
							entry = {nearer, track};
					}
			for (auto const &entry : fan)
				Request.to.push_back(connection::of(editor_track::find_straight(*entry.second.second, m_straights.tolerance)));
		}
	}
	Result = connection::plan(Request);
	return Result.valid;
}

std::string editor_mode::connection_readout(connection::result const &Plan) const
{
	if (false == Plan.valid)
		return Plan.error;
	auto text{format(STR_C("%s: %d switches, spacing %.2f m"), connection_name(m_switch.connection), static_cast<int>(Plan.switches.size()), Plan.spacing)};
	if (false == Plan.straights.empty())
	{
		text += STR_C(", straight inserts");
		for (auto const length : Plan.straights)
			text += format(" %.2f", length);
		text += " m";
	}
	if (Plan.points > 0.0)
		text += format(STR_C(", mathematical points %.2f m apart"), Plan.points);
	if (Plan.radius > 0.0)
		text += format(STR_C(", aligning curve R %.0f m"), Plan.radius);
	if (Plan.crossing && Plan.angle > 1e-6)
		text += format(STR_C(", crossing 1:%.2f"), 1.0 / std::tan(Plan.angle));
	if (false == Plan.fan.empty())
	{
		text += STR_C(", tracks at");
		for (auto const offset : Plan.fan)
			text += format(" %.2f", offset);
		text += " m";
	}
	if (false == Plan.warning.empty())
		text += "\n" + Plan.warning;
	return text;
}

void editor_mode::insert_connection()
{
	auto &tool{m_switch};
	connection::request request;
	connection::result plan;
	auto const planned{plan_connection(request, plan)};
	tool.placing = false;
	if (false == planned)
	{
		tool.status = plan.error.empty() ? std::string{STR_C("Drag onto the other track")} : plan.error;
		ui()->set_status(tool.status, true);
		return;
	}
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	auto *style{tool.line.tracks.front()};
	auto const fail = [&](std::string const &Reason) {
		for (auto *track : created)
			editor_track::retire(*track);
		for (auto entry{states.rbegin()}; entry != states.rend(); ++entry)
		{
			editor_track::apply(*entry->first, entry->second);
			editor_track::commit({entry->first});
		}
		for (auto *track : removed)
		{
			track->m_editorremoved = false;
			editor_track::commit({track});
		}
		if (auto before{switch_store::take_changes()})
			switch_store::restore(std::move(*before));
		if (auto before{alignment_store::take_changes()})
			alignment_store::restore(std::move(*before));
		straight_refresh();
		tool.status = Reason;
		ui()->set_status(Reason, true);
	};
	std::vector<TTrack *> switches;
	for (auto const &entry : plan.switches)
	{
		auto const &shape{entry.second ? request.second : request.first};
		auto const name{switch_name_for_new()};
		auto const before{created.size()};
		auto const start{entry.paths[0].points[segment_data::point::start]};
		auto const end{entry.paths[0].points[segment_data::point::end]};
		if (entry.line >= 0)
		{
			auto const &target{entry.line == 0 ? request.from : request.to[entry.line - 1]};
			auto const found{straight_through((start + end) * 0.5, target.direction)};
			if (found.tracks.empty())
			{
				fail(STR_C("A switch of the connection goes where its track isn't a plain straight"));
				return;
			}
			auto const forward{glm::dot(found.direction, plan_of(end - start)) > 0.0 ? 1 : -1};
			if (false == place_switch_on_straight(found, shape, nullptr, found.along(start), forward, entry.side, false, states, created, removed, name))
			{
				fail(tool.status);
				return;
			}
		}
		else
		{
			auto const made{create_switch_set(shape, entry.paths, *style, name)};
			if (made.empty())
			{
				fail(tool.status);
				return;
			}
			created.insert(created.end(), made.begin(), made.end());
		}
		auto const made{std::find_if(created.begin() + static_cast<std::ptrdiff_t>(before), created.end(), [](TTrack const *Track) { return Track != nullptr && Track->eType == tt_Switch; })};
		if (made == created.end() || glm::distance((*made)->m_paths.front().points[segment_data::point::start], start) > 0.01)
		{
			fail(STR_C("A switch of the connection doesn't fit where it was planned, its straight is too short"));
			return;
		}
		switches.push_back(*made);
		advance_switch_name();
	}
	std::vector<TTrack *> inserts;
	for (auto const &insert : plan.inserts)
		if (auto *track{editor_track::create_path(*style, insert)})
			inserts.push_back(track);
	editor_track::commit(inserts);
	digitize_drawn(inserts);
	created.insert(created.end(), inserts.begin(), inserts.end());
	if (plan.crossing)
	{
		auto const first{glm::normalize(plan_of(plan.ends[1] - plan.ends[0]))};
		auto second{glm::normalize(plan_of(plan.ends[3] - plan.ends[2]))};
		if (glm::dot(first, second) < 0.0)
			second = -second;
		auto const parts{build_double_slip(plan.ends[0], plan.ends[1], plan.ends[2], plan.ends[3], plan_of(plan.centre), plan.centre.y, first, second, plan.angle, 0.0, *style, "Skrz")};
		editor_track::commit(parts);
		created.insert(created.end(), parts.begin(), parts.end());
	}
	record_connection(request.type, switches, plan, request.first);
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	first_step_done(first_step::turnout);
	m_track_point = {};
	m_node = switches.front();
	ui()->set_node(m_node);
	std::string names;
	for (auto *track : switches)
	{
		if (place_switch_drive(*track))
			join_last_steps();
		names += (names.empty() ? "" : ", ") + track->name();
	}
	tool.status = format(STR_C("%s laid, switches %s"), connection_name(tool.connection), names.c_str()) + "\n" + connection_readout(plan);
	ui()->set_status(tool.status, false);
}

void editor_mode::render_connection_ui()
{
	auto &tool{m_switch};
	ImGui::PushItemWidth(220.0f);
	if (ImGui::BeginCombo(STR_C("Connection##switchconnection"), tool.connection < 0 ? STR_C("Single switch") : connection_name(tool.connection)))
	{
		if (ImGui::Selectable(STR_C("Single switch"), tool.connection < 0))
			tool.connection = -1;
		for (int i = 0; i < connection::kind_count; ++i)
			if (ImGui::Selectable(connection_name(i), tool.connection == i))
				tool.connection = i;
		ImGui::EndCombo();
	}
	item_tooltip("A connection of tracks laid whole: press on a straight and drag onto the other track. Its switches are cut into the tracks with the inserts between them and get their drives; Shift+wheel while dragging: another kind, Ctrl+wheel: R of the aligning curve");
	if (tool.connection < 0)
	{
		ImGui::PopItemWidth();
		return;
	}
	auto const label = [&](int const Index) { return Index < 0 || Index >= static_cast<int>(tool.templates.size()) ? std::string{STR_C("The same as the first one")} : switch_label(tool.templates[Index]); };
	if (ImGui::BeginCombo(STR_C("Other switches##connectionsecond"), label(tool.connection_second).c_str()))
	{
		if (ImGui::Selectable(STR_C("The same as the first one"), tool.connection_second < 0))
			tool.connection_second = -1;
		for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
			if (usable_in_connection(tool.templates[i]) && ImGui::Selectable((switch_label(tool.templates[i]) + "##connectionsecond" + std::to_string(i)).c_str(), tool.connection_second == i))
				tool.connection_second = i;
		ImGui::EndCombo();
	}
	item_tooltip("Template of the switches on the other track and along a ladder; the first switch is the one chosen on the list above");
	ImGui::InputDouble(STR_C("Aligning curve R, m##connectionradius"), &tool.connection_radius, 10.0, 100.0, "%.0f");
	tool.connection_radius = std::clamp(tool.connection_radius, 0.0, 10000.0);
	item_tooltip("Of the curve between switches of different angles or on tracks that aren't parallel, and at the end of a ladder; 0: the radius of the sharper switch");
	if (tool.connection == static_cast<int>(connection::kind::double_crossover))
	{
		ImGui::InputDouble(STR_C("Gap between the switches, m##connectiongap"), &tool.connection_gap, 0.5, 1.0, "%.2f");
		tool.connection_gap = std::clamp(tool.connection_gap, 0.0, 100.0);
		item_tooltip("Straight between the starts of the two switches meeting on one track");
		ImGui::Checkbox(STR_C("The switches meet on this track##connectiontogether"), &tool.connection_together);
		item_tooltip("Otherwise the two crossovers meet on the other track; Alt+wheel while dragging switches it");
	}
	if (tool.connection == static_cast<int>(connection::kind::ladder))
	{
		ImGui::InputText(STR_C("Spacings, m##connectionspacings"), tool.connection_spacings, sizeof(tool.connection_spacings));
		item_tooltip("Distances between the tracks of the fan from this track on, such as 4.75;4.75;5.0; used when the drag doesn't end on a parallel track, otherwise the parallel tracks up to that one make the fan");
		ImGui::InputDouble(STR_C("New tracks, m##connectionstub"), &tool.connection_stub, 5.0, 25.0, "%.0f");
		tool.connection_stub = std::clamp(tool.connection_stub, 1.0, 1000.0);
		item_tooltip("Length of the new tracks of the fan where there are none to lead onto");
	}
	ImGui::PopItemWidth();
}
