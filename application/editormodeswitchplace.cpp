/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http:
*/


#include "stdafx.h"
#include "application/editormode.h"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorIncludeInfo.hpp"

#include "utilities/Globals.h"
#include "rendering/renderer.h"
#include "world/Track.h"
#include "scene/scene.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"

#include "imgui/imgui.h"
#include "imgui/ImGuizmo.h"
#include "application/editormodetrackshared.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <functional>

using namespace track_mode;

std::vector<segment_data> editor_mode::switch_preview() const
{
	auto const &tool{m_switch};
	if (false == tool.placing || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return {};
	if (tool.curved)
	{
		auto const sample{sample_at(tool.frame, tool.along)};
		glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
		auto const forward{glm::dot(offset, sample.tangent) >= 0.0 ? 1 : -1};
		glm::dvec2 const direction{sample.tangent * static_cast<double>(forward)};
		auto const side{direction.x * offset.y - direction.y * offset.x >= 0.0 ? 1 : -1};
		return curved_switch_paths(tool.templates[tool.armed], tool.frame, tool.along, forward, side);
	}
	auto const &line{tool.line};
	glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
	auto const forward{glm::dot(offset, line.direction) >= 0.0 ? 1 : -1};
	glm::dvec2 const direction{line.direction * static_cast<double>(forward)};
	auto const side{direction.x * offset.y - direction.y * offset.x >= 0.0 ? 1 : -1};
	return editor_track::place_switch(tool.templates[tool.armed], tool.point, direction, side, line.grade * forward);
}

bool editor_mode::start_switch_placement()
{
	auto &tool{m_switch};
	if (tool.armed < 0 || m_track_tab != track_tab::turnout)
		return false;
	// with no plain track selected, or a press away from it, the click selects
	auto *selected{selected_track()};
	if (selected == nullptr || selected->eType != tt_Normal || selected->m_paths.empty())
		return false;
	glm::dvec3 const ground{cursor_level(selected->m_paths.front().points[segment_data::point::start].y)};
	tool.curved = false;
	tool.status.clear();
	if (current_straight().tracks.empty())
	{
		auto *track{selected};
		if (tool.templates[tool.armed].double_slip)
		{
			tool.status = STR_C("A double slip goes only into a straight crossing another one");
			return false;
		}
		editor_track::curve curve;
		editor_track::chain chain;
		std::string error;
		if (false == editor_track::find_curve(*track, m_straights.tolerance, m_route.design.norms.gauge, curve))
		{
			tool.status = STR_C("The selected track is neither a recognized straight nor a curve");
			return false;
		}
		if (false == editor_track::find_chain(curve.from, curve.to, chain, error, true))
		{
			tool.status = error.empty() ? std::string{STR_C("The curve can't be followed")} : error;
			return false;
		}
		for (auto const &part : editor_track::chain_parts(chain))
			if (std::find(part.tracks.begin(), part.tracks.end(), track) != part.tracks.end())
			{
				chain = part;
				break;
			}
		tool.frame = sample_chain(chain);
		tool.frame_tracks = chain.tracks;
		if (tool.frame.size() < 2)
			return false;
		tool.curved = true;
		tool.along = nearest_station(tool.frame, ground);
		tool.point = sample_at(tool.frame, tool.along).position;
		if (plan_distance(tool.point, ground) > kSwitchReach)
			return false;
		tool.mouse = ground;
		tool.placing = true;
		return true;
	}
	auto const &line{current_straight()};
	auto const along{std::clamp(line.along(ground), 0.0, line.length)};
	auto const point{line.start + glm::dvec3{line.direction.x, line.grade, line.direction.y} * along};
	if (plan_distance(point, ground) > kSwitchReach)
		return false;
	if (tool.templates[tool.armed].double_slip)
	{
		insert_double_slip(line, ground, tool.templates[tool.armed]);
		return true;
	}
	tool.along = along;
	tool.point = point;
	tool.mouse = ground;
	tool.line = line;
	tool.placing = true;
	return true;
}

void editor_mode::finish_switch_placement()
{
	auto &tool{m_switch};
	tool.placing = false;
	if (tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return;
	if (tool.curved)
	{
		auto const sample{sample_at(tool.frame, tool.along)};
		glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
		auto const forward{glm::dot(offset, sample.tangent) >= 0.0 ? 1 : -1};
		glm::dvec2 const direction{sample.tangent * static_cast<double>(forward)};
		auto const side{direction.x * offset.y - direction.y * offset.x >= 0.0 ? 1 : -1};
		insert_curved_switch(forward, side);
		return;
	}
	glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
	auto const forward{glm::dot(offset, tool.line.direction) >= 0.0 ? 1 : -1};
	glm::dvec2 const direction{tool.line.direction * static_cast<double>(forward)};
	auto const side{direction.x * offset.y - direction.y * offset.x >= 0.0 ? 1 : -1};
	insert_switch(tool.line, tool.along, forward, side);
}

void editor_mode::insert_switch(editor_track::straight const &Line, double const Along, int const Direction, int const Side)
{
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	if (false == place_switch_on_straight(Line, m_switch.templates[m_switch.armed], nullptr, Along, Direction, Side, true, states, created, removed, switch_name_for_new()))
		return;
	auto const placed{created};
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	switch_placed(placed);
}

void editor_mode::switch_placed(std::vector<TTrack *> const &Created)
{
	auto const found{std::find_if(Created.begin(), Created.end(), [](TTrack const *Track) { return Track != nullptr && Track->eType == tt_Switch; })};
	if (found == Created.end())
		return;
	auto const slip{std::count_if(Created.begin(), Created.end(), [](TTrack const *Track) { return Track != nullptr && Track->eType == tt_Switch; }) > 1};
	m_switch.armed = -1;
	m_switch.placing = false;
	m_track_point = {};
	m_node = *found;
	ui()->set_node(m_node);
	if (slip)
		return;
	if (m_switch.drive >= 0)
		place_switch_drive(**found);
	std::string name{m_switch.name};
	auto digits{name.size()};
	while (digits > 0 && std::isdigit(static_cast<unsigned char>(name[digits - 1])))
		--digits;
	if (digits < name.size() && name.size() - digits < 9)
		name = name.substr(0, digits) + std::to_string(std::stoi(name.substr(digits)) + 1);
	else
		name.clear();
	std::snprintf(m_switch.name, sizeof(m_switch.name), "%s", name.c_str());
}

std::string editor_mode::switch_name_for_new() const
{
	if (m_switch.name[0] != '\0')
		return m_switch.name;
	for (int i = 1;; ++i)
	{
		auto const name{"z" + std::to_string(i)};
		if (simulation::Paths.find(name) == nullptr && simulation::Events.FindEvent(name + "+") == nullptr)
			return name;
	}
}

bool editor_mode::cut_straight(editor_track::straight const &Line, double const From, double const To, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed, TTrack **Style)
{
	auto const from{From};
	auto const to{To};
	std::string reason;
	if (false == tracks_editable(Line.tracks, reason))
	{
		m_switch.status = reason;
		ui()->set_status(reason, true);
		return false;
	}
	struct span
	{
		TTrack *track;
		int path;
		double from, to;
		bool forward;
	};
	std::vector<span> spans;
	for (std::size_t i = 0; i < Line.tracks.size(); ++i)
	{
		auto const &path{Line.tracks[i]->m_paths[Line.paths[i]]};
		auto const a{Line.along(path.points[segment_data::point::start])};
		auto const b{Line.along(path.points[segment_data::point::end])};
		spans.push_back({Line.tracks[i], Line.paths[i], std::min(a, b), std::max(a, b), a < b});
	}
	std::sort(spans.begin(), spans.end(), [](span const &A, span const &B) { return A.from < B.from; });
	int first{-1}, last{-1};
	for (int i = 0; i < static_cast<int>(spans.size()); ++i)
	{
		if (spans[i].to > from + 1e-3 && spans[i].from < to - 1e-3)
		{
			if (first < 0)
				first = i;
			last = i;
		}
	}
	if (first < 0)
		return false;
	editor_track::chain chain;
	for (int i = first; i <= last; ++i)
	{
		if (spans[i].track->eType != tt_Normal)
			return false;
		chain.tracks.push_back(spans[i].track);
		chain.forward.push_back(spans[i].forward);
	}
	if (Style != nullptr)
		*Style = spans[first].track;
	auto const before{std::max(0.0, from - spans[first].from)};
	auto const after{std::max(0.0, spans[last].to - to)};
	bool const hasbefore{before > 0.01};
	bool const hasafter{after > 0.01};
	auto const point = [&](double const Distance) { return Line.start + glm::dvec3{Line.direction.x, Line.grade, Line.direction.y} * Distance; };
	if (false == hasbefore && false == hasafter)
	{
		for (auto *track : chain.tracks)
			editor_track::retire(*track);
		Removed.insert(Removed.end(), chain.tracks.begin(), chain.tracks.end());
		return true;
	}
	auto const count{std::max<std::size_t>(chain.tracks.size(), (hasbefore ? 1 : 0) + (hasafter ? 1 : 0))};
	std::size_t countbefore{0};
	if (hasbefore && hasafter)
		countbefore = std::clamp<std::size_t>(static_cast<std::size_t>(std::round(count * before / (before + after))), 1, count - 1);
	else if (hasbefore)
		countbefore = count;
	std::vector<segment_data> pieces;
	auto const straight = [&](double const Start, double const End, std::size_t const Count) {
		for (std::size_t i = 0; i < Count; ++i)
		{
			segment_data path;
			path.points[segment_data::point::start] = point(Start + (End - Start) * i / Count);
			path.points[segment_data::point::end] = point(Start + (End - Start) * (i + 1) / Count);
			pieces.push_back(path);
		}
	};
	if (hasbefore)
		straight(spans[first].from, from, countbefore);
	if (hasafter)
		straight(to, spans[last].to, count - countbefore);
	for (auto *track : chain.tracks)
		States.emplace_back(track, editor_track::capture(*track));
	auto const relaid{editor_track::relay(chain, pieces)};
	Created.insert(Created.end(), relaid.begin(), relaid.end());
	return true;
}

bool editor_mode::place_switch_on_straight(editor_track::straight const &Line, editor_track::switch_template const &Shape, TTrack const *Style, double const Along, int const Direction, int const Side, bool const Snap, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed, std::string const &Name)
{
	auto const &shape{Shape};
	if (shape.length > Line.length - 0.02)
	{
		m_switch.status = format(STR_C("The straight is %.1f m long, too short for the %.1f m switch"), Line.length, shape.length);
		return false;
	}
	auto from{std::clamp(Direction > 0 ? Along : Along - shape.length, 0.0, Line.length - shape.length)};
	if (Snap)
	{
		std::vector<double> joints{0.0, Line.length};
		for (std::size_t i = 0; i < Line.tracks.size(); ++i)
		{
			auto const &path{Line.tracks[i]->m_paths[Line.paths[i]]};
			for (auto const index : {segment_data::point::start, segment_data::point::end})
				joints.push_back(Line.along(path.points[index]));
		}
		auto const nearest = [&](double const Value) {
			double best{Value};
			for (auto const joint : joints)
				if (std::abs(joint - Value) < std::abs(best - Value) || best == Value)
					best = std::abs(joint - Value) < 1.0 ? joint : best;
			return best;
		};
		auto const snappedstart{nearest(from)};
		auto const snappedend{nearest(from + shape.length)};
		if (snappedstart != from)
			from = snappedstart;
		else if (snappedend != from + shape.length)
			from = snappedend - shape.length;
		from = std::clamp(from, 0.0, Line.length - shape.length);
	}
	auto const to{from + shape.length};
	auto const origin{Direction > 0 ? from : to};
	TTrack *first{nullptr};
	if (false == cut_straight(Line, from, to, States, Created, Removed, &first))
	{
		m_switch.status = STR_C("The switch would overlap another switch, move it onto plain track");
		return false;
	}
	auto const point = [&](double const Distance) { return Line.start + glm::dvec3{Line.direction.x, Line.grade, Line.direction.y} * Distance; };
	glm::dvec2 const direction{Line.direction * static_cast<double>(Direction)};
	auto const paths{editor_track::place_switch(shape, point(origin), direction, Side, Line.grade * Direction)};
	auto *track{editor_track::create_switch(shape, paths, Style != nullptr ? *Style : *first, Name)};
	if (track != nullptr)
	{
		editor_track::commit({track});
		Created.push_back(track);
		m_switch.status = STR_C("Switch ") + track->name() + STR_C(" inserted");
	}
	else
		m_switch.status = STR_C("The switch couldn't be created from this template");
	return true;
}

bool editor_mode::plan_crossover(crossover_plan &Plan) const
{
	auto const &tool{m_extend};
	if (false == tool.active || tool.track == nullptr || tool.track->eType != tt_Switch || tool.track->m_paths.size() < 2 || tool.path != 1 || false == tool.atend || ImGui::GetIO().KeyShift)
		return false;
	auto const &own{*tool.track};
	auto const &main{own.m_paths[0]};
	auto const &diverging{own.m_paths[1]};
	auto const origin{main.points[segment_data::point::start]};
	auto const axis{glm::normalize(plan_of(main.points[segment_data::point::end] - origin))};
	glm::dvec2 const perpendicular{-axis.y, axis.x};
	auto const &frog{diverging.points[segment_data::point::end]};
	auto const lateral{glm::dot(plan_of(frog - origin), perpendicular)};
	if (std::abs(lateral) < 0.01)
		return false;
	auto const side{lateral > 0.0 ? 1.0 : -1.0};
	auto const normal{perpendicular * side};
	auto const offset{std::abs(lateral)};
	auto const &control{diverging.points[segment_data::point::control2]};
	auto const tangent{glm::normalize(control != glm::dvec3{} ? -plan_of(control) : plan_of(frog - origin))};
	auto const angle{std::atan2(std::abs(cross(axis, tangent)), glm::dot(axis, tangent))};
	if (angle < 1e-4)
		return false;

	auto const ground{tool.mouse};
	TTrack *target{nullptr};
	double best{8.0};
	auto const sections{simulation::Region->sections(ground, 40.f)};
	for (auto *section : sections)
	{
		for (auto const &cell : section->m_cells)
		{
			for (auto *track : cell.m_paths)
			{
				if (track == &own || track->eType != tt_Normal || track->m_editorremoved || false == editor_track::is_straight(*track, m_straights.tolerance))
					continue;
				auto const &path{track->m_paths.front()};
				auto const a{plan_of(path.points[segment_data::point::start])};
				auto const b{plan_of(path.points[segment_data::point::end])};
				auto const segment{b - a};
				auto const t{std::clamp(glm::dot(plan_of(ground) - a, segment) / std::max(1e-9, glm::dot(segment, segment)), 0.0, 1.0)};
				auto const distance{glm::distance(plan_of(ground), a + segment * t)};
				if (distance < best)
				{
					best = distance;
					target = track;
				}
			}
		}
	}
	if (target == nullptr)
		return false;
	Plan.target = editor_track::find_straight(*target, m_straights.tolerance);
	auto const &line{Plan.target};
	if (line.tracks.empty() || std::abs(cross(line.direction, axis)) > std::sin(glm::radians(1.0)))
		return false;
	auto const distance{glm::dot(plan_of(line.start) - plan_of(origin), normal)};
	auto const remaining{distance - 2.0 * offset};
	if (remaining < 0.01)
		return false;
	auto const length{remaining / std::sin(angle)};
	glm::dvec2 const crossing{axis * std::cos(angle) + normal * std::sin(angle)};
	auto const alongfrog{glm::dot(plan_of(frog - origin), axis)};
	auto const otherfrog{plan_of(frog) + crossing * length};
	auto const otherorigin{otherfrog + axis * alongfrog + normal * offset};
	Plan.along = glm::dot(otherorigin - plan_of(line.start), line.direction);
	if (Plan.along < 0.0 || Plan.along > line.length)
		return false;
	Plan.direction = glm::dot(-axis, line.direction) > 0.0 ? 1 : -1;
	glm::dvec2 const direction{line.direction * static_cast<double>(Plan.direction)};
	Plan.side = glm::dot(glm::dvec2{-direction.y, direction.x}, -normal) > 0.0 ? 1 : -1;

	editor_track::switch_template shape;
	shape.source = tool.track;
	shape.length = glm::dot(plan_of(main.points[segment_data::point::end] - origin), axis);
	for (int i = 0; i < 2; ++i)
	{
		auto const &path{own.m_paths[i]};
		auto &local{shape.local[i]};
		local = path;
		auto const tolocal = [&](glm::dvec3 const &Vector) { return glm::dvec3{glm::dot(plan_of(Vector), perpendicular) * side, Vector.y, glm::dot(plan_of(Vector), axis)}; };
		local.points[segment_data::point::start] = tolocal(path.points[segment_data::point::start] - origin);
		local.points[segment_data::point::end] = tolocal(path.points[segment_data::point::end] - origin);
		local.points[segment_data::point::control1] = tolocal(path.points[segment_data::point::control1]);
		local.points[segment_data::point::control2] = tolocal(path.points[segment_data::point::control2]);
		if (side < 0.0)
			local.rolls = {-local.rolls[0], -local.rolls[1]};
	}
	Plan.shape = shape;
	auto const station{line.start + glm::dvec3{line.direction.x, line.grade, line.direction.y} * Plan.along};
	Plan.paths = editor_track::place_switch(shape, station, direction, Plan.side, line.grade * Plan.direction);
	Plan.insert = segment_data{};
	Plan.insert.points[segment_data::point::start] = frog;
	Plan.insert.points[segment_data::point::end] = Plan.paths[1].points[segment_data::point::end];
	return true;
}

std::vector<TTrack *> editor_mode::build_double_slip(glm::dvec3 const &PointA, glm::dvec3 const &PointB, glm::dvec3 const &PointC, glm::dvec3 const &PointD, glm::dvec2 const &Crossing, double const Height, glm::dvec2 const &First, glm::dvec2 const &Second, double const Angle, double const Radius, TTrack &Style)
{
	auto const at = [](glm::dvec2 const &Planar, double const Y) { return glm::dvec3{Planar.x, Y, Planar.y}; };
	auto const straight = [](glm::dvec3 const &Start, glm::dvec3 const &End) {
		segment_data path;
		path.points[segment_data::point::start] = Start;
		path.points[segment_data::point::end] = End;
		return path;
	};
	// the ends of the two diagonals at the crossing are kept apart further than the simulation joins the ends, 2 cm
	auto const halfsine{std::sqrt(std::max(1e-6, (1.0 - glm::dot(First, Second)) * 0.5))};
	auto const gap{std::max(0.25, 0.06 / halfsine)};
	auto const side{cross(First, Second) > 0.0 ? 1.0 : -1.0};
	auto const arc = [&](glm::dvec3 const &Start, glm::dvec2 const &Heading, double const Turn, glm::dvec3 const &Finish, double const From, double const To) {
		glm::dvec2 const centre{plan_of(Start) + glm::dvec2{-Heading.y, Heading.x} * (Turn * Radius)};
		auto const radial{plan_of(Start) - centre};
		auto const p0{centre + turned(radial, Turn * From)};
		auto const p3{centre + turned(radial, Turn * To)};
		auto const d0{turned(Heading, Turn * From)};
		auto const d3{turned(Heading, Turn * To)};
		auto const handle{4.0 / 3.0 * std::tan((To - From) / 4.0) * Radius};
		auto const y0{Start.y + (Finish.y - Start.y) * From / Angle};
		auto const y3{Start.y + (Finish.y - Start.y) * To / Angle};
		segment_data path;
		path.points[segment_data::point::start] = {p0.x, y0, p0.y};
		path.points[segment_data::point::end] = {p3.x, y3, p3.y};
		path.points[segment_data::point::control1] = {d0.x * handle, 0.0, d0.y * handle};
		path.points[segment_data::point::control2] = {-d3.x * handle, 0.0, -d3.y * handle};
		path.radius = static_cast<float>(Radius);
		return path;
	};
	auto const reversed = [](segment_data const &Path) {
		segment_data result{Path};
		result.points[segment_data::point::start] = Path.points[segment_data::point::end];
		result.points[segment_data::point::end] = Path.points[segment_data::point::start];
		result.points[segment_data::point::control1] = Path.points[segment_data::point::control2];
		result.points[segment_data::point::control2] = Path.points[segment_data::point::control1];
		result.rolls = {-Path.rolls[1], -Path.rolls[0]};
		return result;
	};
	auto const middle{Angle * 0.5};
	auto const split{gap / Radius};
	auto const centreA{at(Crossing - First * gap, Height)};
	auto const centreB{at(Crossing + First * gap, Height)};
	auto const centreC{at(Crossing - Second * gap, Height)};
	auto const centreD{at(Crossing + Second * gap, Height)};
	std::string base;
	for (int i = 1; base.empty() || simulation::Paths.find(base + "_a") != nullptr; ++i)
		base = "editor_dks" + std::to_string(i);
	editor_track::switch_template shape;
	shape.length = glm::distance(plan_of(PointA), Crossing);
	std::vector<TTrack *> parts;
	parts.push_back(editor_track::create_switch(shape, {straight(PointA, centreA), arc(PointA, First, side, PointD, 0.0, middle - split)}, Style, base + "_a"));
	parts.push_back(editor_track::create_switch(shape, {straight(PointB, centreB), reversed(arc(PointC, Second, -side, PointB, middle + split, Angle))}, Style, base + "_b"));
	parts.push_back(editor_track::create_switch(shape, {straight(PointC, centreC), arc(PointC, Second, -side, PointB, 0.0, middle - split)}, Style, base + "_c"));
	parts.push_back(editor_track::create_switch(shape, {straight(PointD, centreD), reversed(arc(PointA, First, side, PointD, middle + split, Angle))}, Style, base + "_d"));
	parts.push_back(editor_track::create_path(Style, straight(centreA, centreB)));
	parts.push_back(editor_track::create_path(Style, straight(centreC, centreD)));
	parts.push_back(editor_track::create_path(Style, arc(PointA, First, side, PointD, middle - split, middle + split)));
	parts.push_back(editor_track::create_path(Style, arc(PointC, Second, -side, PointB, middle - split, middle + split)));
	parts.erase(std::remove(parts.begin(), parts.end(), nullptr), parts.end());
	return parts;
}

bool editor_mode::insert_double_slip(editor_track::straight const &Line, glm::dvec3 const &Point, editor_track::switch_template const &Shape)
{
	auto &state{m_straights};
	state.status.clear();
	{
		auto const sections{simulation::Region->sections(Point, 40.f)};
		for (auto *section : sections)
			for (auto const &cell : section->m_cells)
				for (auto *track : cell.m_paths)
					if (track->eType == tt_Switch && false == track->m_editorremoved && track->DoubleSlip() && glm::distance(plan_of(track->location()), plan_of(Point)) < 25.0)
						return replace_double_slip(*track, Shape);
	}
	TTrack *other{nullptr};
	glm::dvec2 crossing{0.0};
	double best{40.0};
	auto const sections{simulation::Region->sections(Point, 60.f)};
	for (auto *section : sections)
	{
		for (auto const &cell : section->m_cells)
		{
			for (auto *track : cell.m_paths)
			{
				if (std::find(Line.tracks.begin(), Line.tracks.end(), track) != Line.tracks.end() || track->eType != tt_Normal || track->m_editorremoved || false == editor_track::is_straight(*track, m_straights.tolerance))
					continue;
				auto const &path{track->m_paths.front()};
				auto const a{plan_of(path.points[segment_data::point::start])};
				auto const b{plan_of(path.points[segment_data::point::end])};
				auto const segment{b - a};
				auto const determinant{cross(Line.direction, segment)};
				if (std::abs(determinant) < 1e-9)
					continue;
				auto const offset{a - plan_of(Line.start)};
				auto const along{cross(offset, segment) / determinant};
				auto const fraction{cross(offset, Line.direction) / determinant};
				if (fraction < -0.01 || fraction > 1.01 || along < 0.0 || along > Line.length)
					continue;
				auto const position{plan_of(Line.start) + Line.direction * along};
				auto const distance{glm::distance(position, plan_of(Point))};
				if (distance < best)
				{
					best = distance;
					other = track;
					crossing = position;
				}
			}
		}
	}
	if (other == nullptr)
	{
		state.status = STR_C("No straight crossing the selected one near the click");
		return false;
	}
	auto const second{editor_track::find_straight(*other, m_straights.tolerance)};
	if (second.tracks.empty())
		return false;
	auto const first{Line.direction};
	auto direction{second.direction};
	if (glm::dot(first, direction) < 0.0)
		direction = -direction;
	auto const angle{std::atan2(std::abs(cross(first, direction)), glm::dot(first, direction))};
	if (angle < glm::radians(0.5) || angle > glm::radians(30.0))
	{
		state.status = STR_C("The straights cross at an angle unsuitable for a double slip");
		return false;
	}
	auto const alongfirst{glm::dot(crossing - plan_of(Line.start), Line.direction)};
	auto const alongsecond{glm::dot(crossing - plan_of(second.start), second.direction)};
	auto const available{std::min({alongfirst, Line.length - alongfirst, alongsecond, second.length - alongsecond}) - 0.5};
	auto radius{Shape.radius};
	auto half{radius * std::tan(angle * 0.5)};
	if (half > available)
	{
		half = available;
		radius = half / std::tan(angle * 0.5);
		if (radius < 50.0)
		{
			state.status = STR_C("Not enough straight track around the crossing");
			return false;
		}
		state.status = STR_C("R reduced to ") + std::to_string(static_cast<int>(radius)) + STR_C(" m, the straights end ") + std::to_string(static_cast<int>(available)) + STR_C(" m from the crossing");
	}

	auto const heightfirst = [&](double const Along) { return Line.start.y + Line.grade * Along; };
	auto const heightsecond = [&](double const Along) { return second.start.y + second.grade * Along; };
	auto const sign{glm::dot(second.direction, direction) > 0.0 ? 1.0 : -1.0};
	auto const at = [&](glm::dvec2 const &Planar, double const Height) { return glm::dvec3{Planar.x, Height, Planar.y}; };
	auto const pointA{at(crossing - first * half, heightfirst(alongfirst - half))};
	auto const pointB{at(crossing + first * half, heightfirst(alongfirst + half))};
	auto const pointC{at(crossing - direction * half, heightsecond(alongsecond - sign * half))};
	auto const pointD{at(crossing + direction * half, heightsecond(alongsecond + sign * half))};

	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	TTrack *style{nullptr};
	if (false == cut_straight(Line, alongfirst - half, alongfirst + half, states, created, removed, &style))
		return false;
	auto const secondline{editor_track::find_straight(*other, m_straights.tolerance)};
	auto const secondalong{glm::dot(crossing - plan_of(secondline.start), secondline.direction)};
	if (secondline.tracks.empty() || false == cut_straight(secondline, secondalong - half, secondalong + half, states, created, removed, nullptr))
	{
		for (auto *track : created)
			editor_track::retire(*track);
		for (auto const &entry : states)
		{
			editor_track::apply(*entry.first, entry.second);
			editor_track::commit({entry.first});
		}
		for (auto *track : removed)
		{
			track->m_editorremoved = false;
			editor_track::commit({track});
		}
		return false;
	}
	auto const parts{build_double_slip(pointA, pointB, pointC, pointD, crossing, heightfirst(alongfirst), first, direction, angle, radius, style != nullptr ? *style : *Line.tracks.front())};
	editor_track::commit(parts);
	created.insert(created.end(), parts.begin(), parts.end());
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	switch_placed(parts);
	return true;
}

bool editor_mode::replace_double_slip(TTrack &Part, editor_track::switch_template const &Shape)
{
	auto &state{m_straights};
	auto const base{Part.name().substr(0, Part.name().size() - 2)};
	std::array<TTrack *, 4> switches{};
	for (int i = 0; i < 4; ++i)
	{
		switches[i] = simulation::Paths.find(base + "_" + static_cast<char>('a' + i));
		if (switches[i] == nullptr || switches[i]->m_editorremoved || switches[i]->eType != tt_Switch || switches[i]->m_paths.size() < 2)
		{
			state.status = STR_C("The double slip isn't complete, it can't be replaced");
			return false;
		}
	}
	struct end
	{
		TTrack *track;
		glm::dvec3 point;
		glm::dvec2 inward;
		TTrack *outer;
	};
	std::vector<end> ends;
	std::vector<TTrack *> parts(switches.begin(), switches.end());
	for (auto *track : switches)
	{
		segment_data const *straight{nullptr};
		for (auto const &path : track->m_paths)
			if (path.points[segment_data::point::control1] == glm::dvec3{} && path.points[segment_data::point::control2] == glm::dvec3{})
				straight = &path;
		if (straight == nullptr)
			straight = &track->m_paths.front();
		auto const start{straight->points[segment_data::point::start]};
		ends.push_back({track, start, glm::normalize(plan_of(straight->points[segment_data::point::end] - start)), track->SwitchExtension->pPrevs[0]});
		for (int i = 0; i < 2; ++i)
		{
			auto *inner{track->SwitchExtension->pNexts[i]};
			if (inner != nullptr && inner->eType == tt_Normal && std::find(parts.begin(), parts.end(), inner) == parts.end() && inner->Length() < 5.0)
				parts.push_back(inner);
		}
	}
	int partner{-1};
	for (int i = 1; i < 4; ++i)
		if (glm::dot(ends[0].inward, ends[i].inward) < -0.999)
			partner = i;
	if (partner < 0)
	{
		state.status = STR_C("The double slip geometry isn't recognized");
		return false;
	}
	auto const &a{ends[0]};
	auto const &b{ends[partner]};
	std::vector<int> others;
	for (int i = 1; i < 4; ++i)
		if (i != partner)
			others.push_back(i);
	if (glm::dot(ends[0].inward, ends[others[0]].inward) < 0.0)
		std::swap(others[0], others[1]);
	auto const &c{ends[others[0]]};
	auto const &d{ends[others[1]]};
	auto const first{a.inward};
	auto direction{c.inward};
	auto const determinant{cross(first, direction)};
	if (std::abs(determinant) < 1e-9)
		return false;
	auto const t{cross(plan_of(c.point) - plan_of(a.point), direction) / determinant};
	glm::dvec2 const crossing{plan_of(a.point) + first * t};
	auto const angle{std::atan2(std::abs(cross(first, direction)), glm::dot(first, direction))};
	auto const oldhalf{glm::distance(plan_of(a.point), crossing)};
	auto radius{Shape.radius};
	auto half{radius * std::tan(angle * 0.5)};
	auto const height{(a.point.y + b.point.y) * 0.5};

	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	if (half > oldhalf)
	{
		for (auto const *e : {&a, &b, &c, &d})
		{
			if (e->outer == nullptr || false == editor_track::is_straight(*e->outer, m_straights.tolerance))
			{
				half = oldhalf;
				radius = half / std::tan(angle * 0.5);
				state.status = STR_C("R limited to ") + std::to_string(static_cast<int>(radius)) + STR_C(" m, a neighbouring path isn't straight");
				break;
			}
			auto const line{editor_track::find_straight(*e->outer, m_straights.tolerance)};
			auto const reach{std::abs(glm::dot(plan_of(line.end) - crossing, -e->inward))};
			auto const reachstart{std::abs(glm::dot(plan_of(line.start) - crossing, -e->inward))};
			if (std::max(reach, reachstart) - 0.5 < half)
			{
				half = std::min(half, std::max(reach, reachstart) - 0.5);
				radius = half / std::tan(angle * 0.5);
				state.status = STR_C("R reduced to ") + std::to_string(static_cast<int>(radius)) + STR_C(" m to fit the straights");
			}
		}
	}
	for (auto *track : parts)
		editor_track::retire(*track);
	removed.insert(removed.end(), parts.begin(), parts.end());
	TTrack *style{a.outer != nullptr && a.outer->eType == tt_Normal ? a.outer : switches[0]};
	std::array<glm::dvec3, 4> points;
	std::array<end const *, 4> order{&a, &b, &c, &d};
	for (int i = 0; i < 4; ++i)
	{
		auto const &e{*order[i]};
		auto const outward{-e.inward};
		auto const target{crossing + outward * half};
		points[i] = {target.x, e.point.y, target.y};
		if (half < oldhalf - 0.01)
		{
			auto *filler{editor_track::create_path(*style, [&] {
				segment_data path;
				path.points[segment_data::point::start] = points[i];
				path.points[segment_data::point::end] = e.point;
				return path;
			}())};
			created.push_back(filler);
		}
		else if (half > oldhalf + 0.01 && e.outer != nullptr)
		{
			auto const line{editor_track::find_straight(*e.outer, m_straights.tolerance)};
			auto const alongold{glm::dot(plan_of(e.point) - plan_of(line.start), line.direction)};
			auto const alongnew{glm::dot(target - plan_of(line.start), line.direction)};
			cut_straight(line, std::min(alongold, alongnew), std::max(alongold, alongnew), states, created, removed, nullptr);
		}
	}
	auto const parts2{build_double_slip(points[0], points[1], points[2], points[3], crossing, height, first, direction, angle, radius, *style)};
	std::vector<TTrack *> commitlist(parts2.begin(), parts2.end());
	for (auto *track : created)
		if (std::find(commitlist.begin(), commitlist.end(), track) == commitlist.end())
			commitlist.push_back(track);
	editor_track::commit(commitlist);
	created.insert(created.end(), parts2.begin(), parts2.end());
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	return true;
}

std::vector<editor_mode::curve_sample> editor_mode::sample_chain(editor_track::chain const &Chain)
{
	std::vector<curve_sample> result;
	double station{0.0};
	for (std::size_t i = 0; i < Chain.tracks.size(); ++i)
	{
		auto const &path{Chain.tracks[i]->m_paths.front()};
		auto const forward{Chain.forward[i]};
		bezier const curve{path};
		auto const length{glm::distance(curve.p0, curve.p3)};
		int const steps{std::max(8, static_cast<int>(std::ceil(length / 0.5)))};
		for (int k = (result.empty() ? 0 : 1); k <= steps; ++k)
		{
			auto const t{forward ? static_cast<double>(k) / steps : 1.0 - static_cast<double>(k) / steps};
			auto const position{curve.point(t)};
			auto const derivative{forward ? curve.first(t) : -curve.first(t)};
			if (false == result.empty())
				station += plan_distance(position, result.back().position);
			curve_sample sample;
			sample.station = station;
			sample.position = position;
			sample.tangent = glm::normalize(plan_of(derivative));
			auto const roll{path.rolls[0] + (path.rolls[1] - path.rolls[0]) * t};
			sample.roll = static_cast<float>(forward ? roll : -roll);
			sample.radius = path.radius;
			result.push_back(sample);
		}
	}
	return result;
}

editor_mode::curve_sample editor_mode::sample_at(std::vector<curve_sample> const &Frame, double const Station)
{
	if (Frame.empty())
		return {};
	if (Station <= Frame.front().station)
		return Frame.front();
	if (Station >= Frame.back().station)
		return Frame.back();
	auto const next{std::lower_bound(Frame.begin(), Frame.end(), Station, [](curve_sample const &Sample, double const Value) { return Sample.station < Value; })};
	auto const previous{std::prev(next)};
	auto const span{std::max(1e-9, next->station - previous->station)};
	auto const f{(Station - previous->station) / span};
	curve_sample result;
	result.station = Station;
	result.position = glm::mix(previous->position, next->position, f);
	result.tangent = glm::normalize(glm::mix(previous->tangent, next->tangent, f));
	result.roll = static_cast<float>(previous->roll + (next->roll - previous->roll) * f);
	result.radius = previous->radius;
	return result;
}

double editor_mode::nearest_station(std::vector<curve_sample> const &Frame, glm::dvec3 const &Point)
{
	double best{std::numeric_limits<double>::max()};
	double station{0.0};
	for (std::size_t i = 0; i + 1 < Frame.size(); ++i)
	{
		glm::dvec2 const a{Frame[i].position.x, Frame[i].position.z};
		glm::dvec2 const b{Frame[i + 1].position.x, Frame[i + 1].position.z};
		auto const segment{b - a};
		auto const t{std::clamp(glm::dot(plan_of(Point) - a, segment) / std::max(1e-12, glm::dot(segment, segment)), 0.0, 1.0)};
		auto const distance{glm::distance(plan_of(Point), a + segment * t)};
		if (distance < best)
		{
			best = distance;
			station = Frame[i].station + (Frame[i + 1].station - Frame[i].station) * t;
		}
	}
	return station;
}

std::vector<segment_data> editor_mode::curved_switch_paths(editor_track::switch_template const &Shape, std::vector<curve_sample> const &Frame, double const Station, int const Direction, int const Side)
{
	auto const map = [&](glm::dvec3 const &Local, glm::dvec2 &Tangent, glm::dvec2 const &Localtangent) {
		auto const sample{sample_at(Frame, Station + Direction * Local.z)};
		auto const along{sample.tangent * static_cast<double>(Direction)};
		glm::dvec2 const across{glm::dvec2{-along.y, along.x} * static_cast<double>(Side)};
		Tangent = glm::normalize(along * Localtangent.y + across * Localtangent.x);
		auto const planar{plan_of(sample.position) + across * Local.x};
		auto const cant{glm::radians(static_cast<double>(sample.roll) * Direction)};
		auto const cone{-Local.x * Side * std::tan(cant)};
		return glm::dvec3{planar.x, sample.position.y + Local.y + cone, planar.y};
	};
	auto const hermite = [](glm::dvec3 const &Start, glm::dvec2 const &Startdirection, glm::dvec3 const &End, glm::dvec2 const &Enddirection) {
		auto const handle{plan_distance(Start, End) / 3.0};
		segment_data path;
		path.points[segment_data::point::start] = Start;
		path.points[segment_data::point::end] = End;
		path.points[segment_data::point::control1] = {Startdirection.x * handle, (End.y - Start.y) / 3.0, Startdirection.y * handle};
		path.points[segment_data::point::control2] = {-Enddirection.x * handle, -(End.y - Start.y) / 3.0, -Enddirection.y * handle};
		return path;
	};
	std::vector<segment_data> result;
	for (auto const &local : Shape.local)
	{
		auto const &start{local.points[segment_data::point::start]};
		auto const &end{local.points[segment_data::point::end]};
		auto const &control1{local.points[segment_data::point::control1]};
		auto const &control2{local.points[segment_data::point::control2]};
		glm::dvec2 const chord{end.x - start.x, end.z - start.z};
		auto const localstart{glm::normalize(control1 != glm::dvec3{} ? plan_of(control1) : chord)};
		auto const localend{glm::normalize(control2 != glm::dvec3{} ? -plan_of(control2) : chord)};
		glm::dvec2 startdirection, enddirection;
		auto const p0{map(start, startdirection, localstart)};
		auto const p3{map(end, enddirection, localend)};
		auto path{hermite(p0, startdirection, p3, enddirection)};
		{
			auto const cone = [&](double const T) {
				auto const point{bezier{path}.point(T)};
				auto const sample{sample_at(Frame, nearest_station(Frame, point))};
				auto const lateral{glm::dot(plan_of(point) - plan_of(sample.position), glm::dvec2{-sample.tangent.y, sample.tangent.x})};
				return sample.position.y - lateral * std::tan(glm::radians(static_cast<double>(sample.roll)));
			};
			auto const h1{cone(1.0 / 3.0)};
			auto const h2{cone(2.0 / 3.0)};
			auto const y1{3.0 * h1 - 1.5 * h2 - 5.0 / 6.0 * p0.y + p3.y / 3.0};
			auto const y2{3.0 * h2 - 1.5 * h1 + p0.y / 3.0 - 5.0 / 6.0 * p3.y};
			path.points[segment_data::point::control1].y = y1 - p0.y;
			path.points[segment_data::point::control2].y = y2 - p3.y;
		}
		auto const rollstart{sample_at(Frame, Station + Direction * start.z).roll * Direction};
		auto const rollend{sample_at(Frame, Station + Direction * end.z).roll * Direction};
		path.rolls = {static_cast<float>(rollstart), static_cast<float>(rollend)};
		path.radius = local.radius;
		{
			bezier const curve{path};
			auto const begin{plan_of(curve.first(0.0))};
			auto const finish{plan_of(curve.first(1.0))};
			auto const turn{glm::length(begin) > 1e-9 && glm::length(finish) > 1e-9 ? geometry::signed_angle(begin, finish) : 0.0};
			if (local.radius != 0.f && std::abs(turn) > 1e-6)
				path.radius = static_cast<float>(std::copysign(curve.plan_length() / std::abs(turn), static_cast<double>(local.radius)));
		}
		result.push_back(path);
	}
	if (false == result.empty())
		result.front().radius = sample_at(Frame, Station).radius;
	return result;
}

void editor_mode::insert_curved_switch(int const Direction, int const Side)
{
	auto &tool{m_switch};
	auto const &shape{tool.templates[tool.armed]};
	auto const total{tool.frame.back().station};
	if (shape.length > total - 0.02)
	{
		tool.status = format(STR_C("The curve is %.1f m long, too short for the %.1f m switch"), total, shape.length);
		return;
	}
	auto const station{std::clamp(tool.along, Direction > 0 ? 0.0 : shape.length, Direction > 0 ? total - shape.length : total)};
	auto const from{Direction > 0 ? station : station - shape.length};
	auto const to{from + shape.length};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	std::vector<TTrack *> candidates{tool.frame_tracks};
	for (auto const cut : {from, to})
	{
		auto const point{sample_at(tool.frame, cut).position};
		for (auto *candidate : std::vector<TTrack *>(candidates))
		{
			if (candidate->m_editorremoved || candidate->eType != tt_Normal)
				continue;
			auto const t{editor_track::nearest_parameter(*candidate, point)};
			auto const found{editor_track::point_at(*candidate, t)};
			if (plan_distance(found, point) > 0.05)
				continue;
			if (t > 0.002 && t < 0.998)
			{
				if (std::find(created.begin(), created.end(), candidate) == created.end() && std::none_of(states.begin(), states.end(), [&](auto const &Entry) { return Entry.first == candidate; }))
					states.emplace_back(candidate, editor_track::capture(*candidate));
				if (auto *second{editor_track::split_path(*candidate, t)})
				{
					created.push_back(second);
					candidates.push_back(second);
				}
			}
			break;
		}
	}
	for (auto *candidate : candidates)
	{
		if (candidate->m_editorremoved)
			continue;
		auto const middle{editor_track::point_at(*candidate, 0.5)};
		auto const where{nearest_station(tool.frame, middle)};
		if (where > from + 0.01 && where < to - 0.01)
		{
			editor_track::retire(*candidate);
			removed.push_back(candidate);
		}
	}
	auto const paths{curved_switch_paths(shape, tool.frame, station, Direction, Side)};
	auto *track{editor_track::create_switch(shape, paths, *tool.frame_tracks.front(), switch_name_for_new())};
	if (track != nullptr)
	{
		editor_track::commit({track});
		created.push_back(track);
		tool.status = STR_C("Switch ") + track->name() + STR_C(" inserted into the curve");
	}
	else
		tool.status = STR_C("The switch couldn't be created from this template");
	auto const placed{created};
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	switch_placed(placed);
}
