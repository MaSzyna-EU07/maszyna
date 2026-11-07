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
#include "application/editormodetrackshared.h"

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
using geometry::plan_distance;
using geometry::plan_of;

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

bool usable_in_connection(editor_track::switch_template const &Template)
{
	return false == Template.double_slip && false == Template.twofold();
}

// m: a switch let go this near the other track is dragged onto it, at least this far from its own track
double constexpr kOntoTrack{1.5};
double constexpr kOffTrack{2.5};
// m: the furthest the switches of two crossovers meeting on one track are apart for a double crossover
double constexpr kDoubleGap{50.0};

// whether the main track of the switch runs along the line
bool on_line(TTrack const &Switch, connection::line const &Line)
{
	auto const &main{Switch.m_paths.front()};
	return std::abs(Line.lateral(main.points[segment_data::point::start])) < 0.05 && std::abs(Line.lateral(main.points[segment_data::point::end])) < 0.05;
}

// the line along the main track of the switch, reaching far past both of its ends
connection::line line_through(TTrack const &Switch)
{
	double constexpr reach{2000.0};
	auto const &main{Switch.m_paths.front()};
	auto const start{main.points[segment_data::point::start]};
	auto const end{main.points[segment_data::point::end]};
	connection::line result;
	result.direction = glm::normalize(plan_of(end - start));
	result.grade = (end.y - start.y) / std::max(1e-9, glm::length(plan_of(end - start)));
	result.start = start - glm::dvec3{result.direction.x, result.grade, result.direction.y} * reach;
	result.length = 2.0 * reach;
	return result;
}

} // namespace

char const *editor_mode::connection_name(int const Kind)
{
	switch (static_cast<connection::kind>(Kind))
	{
	case connection::kind::double_crossover: return STR_C("Double crossover");
	case connection::kind::scissors: return STR_C("Scissors crossover");
	case connection::kind::ladder: return STR_C("Switch ladder");
	default: return STR_C("Crossover");
	}
}

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

void editor_mode::recognise_connection()
{
	auto &tool{m_switch};
	// a connection chosen by a script stays as it was chosen
	if (false == tool.recognised && tool.connection >= 0)
		return;
	tool.connection = -1;
	tool.recognised = false;
	tool.partner = nullptr;
	if (false == tool.placing || tool.free || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()) || tool.templates[tool.armed].double_slip || tool.templates[tool.armed].twofold())
		return;
	auto const mark = [&](connection::kind const Kind) {
		tool.connection = static_cast<int>(Kind);
		tool.recognised = true;
	};
	// let go at a free end, the switch joins it
	auto const own{tool.curved ? std::vector<TTrack const *>(tool.frame_tracks.begin(), tool.frame_tracks.end()) : std::vector<TTrack const *>(tool.line.tracks.begin(), tool.line.tracks.end())};
	if (editor_track::find_free_end(nullptr, ::track_mode::kRailCategory, tool.mouse, ::track_mode::kLaySnapRadius, own).track != nullptr)
		return;
	if (tool.curved)
	{
		// along a curve a crossover only, let go on the track running beside it
		if (tool.frame.size() < 2 || plan_distance(sample_at(tool.frame, nearest_station(tool.frame, tool.mouse)).position, tool.mouse) < kOffTrack)
			return;
		for (auto *section : simulation::Region->sections(tool.mouse, 40.f))
			for (auto const &cell : section->m_cells)
				for (auto *track : cell.m_paths)
					if (track->eType == tt_Normal && false == track->m_editorremoved && false == track->m_paths.empty() && std::find(tool.frame_tracks.begin(), tool.frame_tracks.end(), track) == tool.frame_tracks.end() &&
					    plan_distance(editor_track::point_at(*track, editor_track::nearest_parameter(*track, tool.mouse)), tool.mouse) < kOntoTrack)
					{
						mark(connection::kind::crossover);
						return;
					}
		return;
	}
	if (tool.line.tracks.empty())
		return;
	auto const line{connection::of(tool.line)};
	if (std::abs(line.lateral(tool.mouse)) < kOffTrack)
		return;
	std::vector<TTrack const *> const exclude(tool.line.tracks.begin(), tool.line.tracks.end());
	auto const other{straight_near(tool.mouse, exclude, kOntoTrack)};
	if (other.tracks.empty())
		return;
	auto const otherline{connection::of(other)};
	mark(connection::kind::crossover);
	// a crossover between the two tracks turned the other way: behind the press, or past it on the other track, a double crossover
	// with it; over it a scissors one
	auto const forward{glm::dot(plan_of(tool.mouse - tool.point), line.direction) >= 0.0 ? 1.0 : -1.0};
	auto const press{line.along(tool.point)};
	auto nearest{kDoubleGap};
	for (auto *section : simulation::Region->sections(tool.point, 200.f))
		for (auto const &cell : section->m_cells)
			for (auto *track : cell.m_paths)
			{
				if (track->eType != tt_Switch || track->m_editorremoved || track->m_paths.size() < 2 || false == on_line(*track, line))
					continue;
				auto const crossover{crossover_of(*track)};
				if (false == crossover.has_value())
					continue;
				auto const *opposite{crossover->first == track ? crossover->second : crossover->first};
				if (false == on_line(*opposite, otherline))
					continue;
				auto const &main{track->m_paths.front()};
				auto const way{glm::dot(plan_of(main.points[segment_data::point::end] - main.points[segment_data::point::start]), line.direction) >= 0.0 ? 1.0 : -1.0};
				if (way == forward)
					continue;
				// along this track in the way the crossover there runs from it to the other one: its switch here starts at 0, the one on
				// the other track at Span; the one dragged, turned the other way, crosses it from 0 to twice that
				auto const span{way * (line.along(opposite->m_paths.front().points[segment_data::point::start]) - line.along(main.points[segment_data::point::start]))};
				auto const at{way * (press - line.along(main.points[segment_data::point::start]))};
				if (span <= 0.0)
					continue;
				double gap;
				bool together{false};
				if (at < 0.0)
				{
					gap = -at;
					together = true;
				}
				else if (at > 2.0 * span)
					gap = at - 2.0 * span;
				else
					gap = 0.0;
				if (gap >= nearest)
					continue;
				nearest = gap;
				tool.partner = track;
				tool.connection = static_cast<int>(at >= 0.0 && at <= 2.0 * span ? connection::kind::scissors : connection::kind::double_crossover);
				tool.connection_together = together;
				tool.connection_gap = gap;
			}
	if (tool.partner != nullptr)
		return;
	// across tracks running parallel between the two, a ladder onto all of them
	if (std::abs(cross(other.direction, tool.line.direction)) < 1e-3)
	{
		tool.connection = static_cast<int>(connection::kind::ladder);
		connection::request request;
		connection::result plan;
		plan_connection(request, plan);
		if (request.to.size() < 2)
			tool.connection = static_cast<int>(connection::kind::crossover);
	}
}

bool editor_mode::plan_partner_connection(connection::request &Request, connection::result &Result) const
{
	auto const &tool{m_switch};
	Result = {};
	auto const crossover{tool.partner != nullptr ? crossover_of(*tool.partner) : std::nullopt};
	if (false == crossover.has_value() || tool.connection < 0)
		return false;
	auto *opposite{crossover->first == tool.partner ? crossover->second : crossover->first};
	auto const shape{editor_track::template_of_switch(tool.partner)};
	if (false == shape.has_value())
		return false;
	// on lines through the crossover there, the double or the scissors one starting with it
	Request = {};
	Request.type = static_cast<connection::kind>(tool.connection);
	Request.from = line_through(*tool.partner);
	Request.to = {line_through(*opposite)};
	Request.along = Request.from.along(tool.partner->m_paths.front().points[segment_data::point::start]);
	Request.forward = 1;
	Request.target = Request.to.front().along(opposite->m_paths.front().points[segment_data::point::start]);
	Request.first = *shape;
	Request.second = *shape;
	Request.radius = tool.connection_radius;
	Request.gap = tool.connection_gap;
	Request.together = tool.connection_together;
	Result = connection::plan(Request);
	return Result.valid;
}

void editor_mode::insert_partner_connection()
{
	auto &tool{m_switch};
	tool.placing = false;
	connection::request request;
	connection::result plan;
	auto const fail = [&](std::string const &Reason) {
		tool.status = Reason;
		ui()->set_status(Reason, true);
		flash_switch_problem(Reason, {}, {{tool.partner->location(), ""}});
	};
	if (false == plan_partner_connection(request, plan))
		return fail(plan.error.empty() ? std::string{STR_C("The connection doesn't fit with the crossover there")} : plan.error);
	auto const crossover{crossover_of(*tool.partner)};
	auto *here{tool.partner};
	auto *opposite{crossover->first == here ? crossover->second : crossover->first};
	auto const armed{tool.armed};
	auto const shape{crossover_template(crossover->record)};
	tool.armed = armed;
	if (shape < 0)
		return fail(STR_C("The template the crossover was laid of isn't among the templates"));
	auto const press{here->m_paths.front().points[segment_data::point::start]};
	auto const heading{plan_of(here->m_paths.front().points[segment_data::point::end] - press)};
	auto const release{opposite->m_paths.front().points[segment_data::point::start]};
	std::vector<std::string> const names{here->name(), opposite->name()};
	// the crossover there taken out, then laid again with the other one, one step of the history
	profile_library_load();
	auto library{m_profiles};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	TTrack *first{nullptr};
	TTrack *second{nullptr};
	if (false == take_out_crossover(*crossover, states, created, removed, first, second))
		return fail(STR_C("The crossover there can't be taken out"));
	auto *nearplain{crossover->first == here ? first : second};
	auto *farplain{crossover->first == here ? second : first};
	switch_store::follow();
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	records_to_history();
	if (crossover->record.lifted())
		profiles_to_history(std::move(library));
	auto const from{editor_track::find_straight(*nearplain, m_straights.tolerance)};
	auto const to{editor_track::find_straight(*farplain, m_straights.tolerance)};
	if (from.tracks.empty() || to.tracks.empty())
	{
		undo_last();
		return fail(STR_C("The tracks of the crossover there aren't straights"));
	}
	request.from = connection::of(from);
	request.to = {connection::of(to)};
	request.along = request.from.along(press);
	request.forward = glm::dot(request.from.direction, heading) >= 0.0 ? 1 : -1;
	request.target = request.to.front().along(release);
	request.first = tool.templates[shape];
	request.second = tool.templates[shape];
	plan = connection::plan(request);
	tool.keep_names = names;
	auto const laid{plan.valid && lay_planned_connection(request, plan, from.tracks.front())};
	tool.keep_names.clear();
	if (false == laid)
	{
		auto const reason{plan.valid ? tool.status : plan.error};
		undo_last();
		return fail(reason);
	}
	join_last_steps();
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
	// the switches as they'd go, blinking where they don't fit
	auto const preview{planned ? std::vector<segment_data>{} : switch_preview()};
	tool.placing = false;
	if (false == planned)
	{
		tool.status = plan.error.empty() ? std::string{STR_C("Drag onto the other track")} : plan.error;
		ui()->set_status(tool.status, true);
		flash_switch_problem(tool.status, preview, {{tool.mouse, ""}});
		return;
	}
	lay_planned_connection(request, plan, tool.line.tracks.front());
}

bool editor_mode::lay_planned_connection(connection::request const &Request, connection::result const &Plan, TTrack *Style)
{
	auto &tool{m_switch};
	auto const &request{Request};
	auto const &plan{Plan};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	auto *style{Style};
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
			editor_track::revive(*track);
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
				return false;
			}
			auto const forward{glm::dot(found.direction, plan_of(end - start)) > 0.0 ? 1 : -1};
			if (false == place_switch_on_straight(found, shape, nullptr, found.along(start), forward, entry.side, false, states, created, removed, name))
			{
				fail(tool.status);
				return false;
			}
		}
		else
		{
			auto const made{create_switch_set(shape, entry.paths, *style, name)};
			if (made.empty())
			{
				fail(tool.status);
				return false;
			}
			created.insert(created.end(), made.begin(), made.end());
		}
		auto const made{std::find_if(created.begin() + static_cast<std::ptrdiff_t>(before), created.end(), [](TTrack const *Track) { return Track != nullptr && Track->eType == tt_Switch; })};
		// in the plan: the height is the track's there, which needn't keep to one grade as the line of the plan does
		if (made == created.end() || plan_distance((*made)->m_paths.front().points[segment_data::point::start], start) > 0.01)
		{
			fail(STR_C("A switch of the connection doesn't fit where it was planned, its straight is too short"));
			return false;
		}
		switches.push_back(*made);
		advance_switch_name();
	}
	// the inserts meet the switches at the heights they were put at
	std::vector<glm::dvec3> ends;
	for (auto const *track : switches)
		for (auto const &path : track->m_paths)
			for (auto const index : {segment_data::point::start, segment_data::point::end})
				ends.push_back(path.points[index]);
	std::vector<TTrack *> inserts;
	for (auto insert : plan.inserts)
	{
		for (auto const index : {segment_data::point::start, segment_data::point::end})
			for (auto const &end : ends)
				if (plan_distance(insert.points[index], end) < 0.05)
				{
					insert.points[index].y = end.y;
					break;
				}
		if (auto *track{editor_track::create_path(*style, insert)})
			inserts.push_back(track);
	}
	editor_track::commit(inserts);
	digitize_drawn(inserts);
	created.insert(created.end(), inserts.begin(), inserts.end());
	if (plan.crossing)
	{
		auto const first{glm::normalize(plan_of(plan.ends[1] - plan.ends[0]))};
		auto second{glm::normalize(plan_of(plan.ends[3] - plan.ends[2]))};
		if (glm::dot(first, second) < 0.0)
			second = -second;
		auto const parts{build_double_slip(plan.ends[0], plan.ends[1], plan.ends[2], plan.ends[3], plan_of(plan.centre), plan.centre.y, first, second, plan.angle, editor_track::slip_curve{}, *style, "Skrz")};
		editor_track::commit(parts);
		created.insert(created.end(), parts.begin(), parts.end());
	}
	record_connection(request.type, switches, plan, request.first);
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
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
	tool.status = format(STR_C("%s laid, switches %s"), connection_name(static_cast<int>(request.type)), names.c_str()) + "\n" + connection_readout(plan);
	ui()->set_status(tool.status, false);
	return true;
}

void editor_mode::render_connection_options()
{
	auto &tool{m_switch};
	// of the connections a switch dragged onto the track beside makes
	if (false == begin_fields("##connectionoptions"))
		return;
	auto const label = [&](int const Index) { return Index < 0 || Index >= static_cast<int>(tool.templates.size()) ? std::string{STR_C("The same as the first one")} : switch_template_label(tool.templates[Index]); };
	field_row("Other switches of a connection##connectionsecond");
	if (ImGui::BeginCombo("##connectionsecond", label(tool.connection_second).c_str()))
	{
		if (ImGui::Selectable(STR_C("The same as the first one"), tool.connection_second < 0))
			tool.connection_second = -1;
		for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
			if (usable_in_connection(tool.templates[i]) && ImGui::Selectable((switch_template_label(tool.templates[i]) + "##connectionsecond" + std::to_string(i)).c_str(), tool.connection_second == i))
				tool.connection_second = i;
		ImGui::EndCombo();
	}
	item_tooltip("Template of the switches on the other track and along a ladder, when a switch is dragged onto the track beside; the first switch is the one chosen on the list above");
	field_row("Aligning curve R, m##connectionradius");
	ImGui::InputDouble("##connectionradius", &tool.connection_radius, 10.0, 100.0, "%.0f");
	tool.connection_radius = std::clamp(tool.connection_radius, 0.0, 10000.0);
	item_tooltip("Of the curve between switches of different angles or on tracks that aren't parallel, and at the end of a ladder; 0: the radius of the sharper switch");
	ImGui::EndTable();
}
