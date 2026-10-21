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
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorSwitchStore.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorIncludeInfo.hpp"

#include "utilities/Globals.h"
#include "rendering/renderer.h"
#include "world/Track.h"
#include "world/TrackFeatures.h"
#include "scene/scene.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"

#include "imgui/imgui.h"
#include "imgui/ImGuizmo.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <functional>
#include <iterator>
#include <set>
#include "application/editormodetrackshared.h"

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
	if (switch_link plan; plan_switch_link(plan))
	{
		plan.paths.insert(plan.paths.end(), plan.link.pieces.begin(), plan.link.pieces.end());
		return plan.paths;
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
	auto const cursor{cursor_on_straight(line)};
	auto const along{std::clamp(line.along(cursor), 0.0, line.length)};
	auto const point{line.start + glm::dvec3{line.direction.x, line.grade, line.direction.y} * along};
	if (plan_distance(point, cursor) > kSwitchReach)
		return false;
	if (tool.templates[tool.armed].double_slip)
	{
		insert_double_slip(line, cursor, tool.templates[tool.armed]);
		return true;
	}
	tool.along = along;
	tool.point = point;
	tool.mouse = cursor;
	tool.line = line;
	tool.placing = true;
	return true;
}

void editor_mode::finish_switch_placement()
{
	auto &tool{m_switch};
	if (tool.connection >= 0)
	{
		if (false == tool.curved)
		{
			insert_connection();
			return;
		}
		insert_curved_connection();
		return;
	}
	switch_link plan;
	auto const linked{plan_switch_link(plan)};
	tool.placing = false;
	if (tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return;
	if (linked)
	{
		insert_linked_switch(plan);
		return;
	}
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
	if (false == plan.reason.empty())
		ui()->set_status(plan.reason, true);
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
	m_switch.placing = false;
	first_step_done(first_step::turnout);
	m_track_point = {};
	m_node = *found;
	ui()->set_node(m_node);
	if (slip)
	{
		auto const known{switch_store::named(**found)};
		if (false == known.has_value() || (known->kind != "double" && known->kind != "slip"))
			return;
		std::vector<std::string> driven;
		for (auto *track : Created)
			if (track != nullptr && track->eType == tt_Switch && place_switch_drive(*track))
			{
				join_last_steps();
				driven.push_back(track->name());
			}
		// a slip has a drive at each of its switches
		if (known->kind == "slip")
		{
			if (false == driven.empty())
			{
				std::string names;
				for (auto const &name : driven)
					names += (names.empty() ? "" : ", ") + name;
				m_switch.status = format(STR_C("Slip with drives at its switches %s: the events of the name of a switch with + and - switch it, also in the map"), names.c_str());
				ui()->set_status(m_switch.status, false);
			}
			return;
		}
		if (driven.size() == 2)
		{
			m_switch.status = format(STR_C("Double switch %s + %s with drives: events %s+, %s- and %s+, %s- switch them, also in the map"), driven[0].c_str(), driven[1].c_str(), driven[0].c_str(), driven[0].c_str(), driven[1].c_str(), driven[1].c_str());
			ui()->set_status(m_switch.status, false);
		}
		advance_switch_name();
		return;
	}
	if (place_switch_drive(**found))
		join_last_steps();
	advance_switch_name();
}

void editor_mode::advance_switch_name()
{
	if (false == m_switch.keep_names.empty())
	{
		m_switch.keep_names.erase(m_switch.keep_names.begin());
		return;
	}
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
	if (false == m_switch.keep_names.empty() && simulation::Paths.find(m_switch.keep_names.front()) == nullptr)
		return m_switch.keep_names.front();
	if (m_switch.name[0] != '\0')
		return m_switch.name;
	for (int i = 1;; ++i)
	{
		auto const name{"z" + std::to_string(i)};
		if (simulation::Paths.find(name) == nullptr && simulation::Events.FindEvent(name + "+") == nullptr)
			return name;
	}
}

void editor_mode::join_last_steps()
{
	if (m_history.size() < 2)
		return;
	auto &last{m_history.back()};
	// a step of more than the includes placed, as the paths laid after others were taken out, is undone together with the one before
	auto const more{false == last.tracks.empty() || false == last.created.empty() || false == last.removed.empty() || false == last.directives.empty() || false == last.sweeps.empty() || false == last.sweeps_toggled.empty() ||
	                false == last.infra.empty() || last.alignments.has_value() || last.switch_records.has_value() || last.profiles || false == last.copies.empty() || last.terrain != nullptr};
	if (more)
	{
		last.joined = true;
		return;
	}
	auto instances{std::move(last.instances)};
	m_history.pop_back();
	auto &previous{m_history.back()};
	previous.instances.insert(previous.instances.end(), instances.begin(), instances.end());
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
		double from;
		double to;
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
	int first{-1};
	int last{-1};
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
	auto const from{switch_from(Line, shape.length, Along, Direction, Snap)};
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
	auto const made{create_switch_set(shape, paths, Style != nullptr ? *Style : *first, Name)};
	Created.insert(Created.end(), made.begin(), made.end());
	return true;
}

std::vector<TTrack *> editor_mode::create_switch_set(editor_track::switch_template const &Shape, std::vector<segment_data> const &Paths, TTrack const &Style, std::string const &Name)
{
	if (Shape.twofold() && Paths.size() >= 5)
	{
		auto firstshape{Shape};
		firstshape.length = glm::distance(Paths[0].points[segment_data::point::start], Paths[0].points[segment_data::point::end]);
		auto secondshape{Shape};
		secondshape.length = Shape.length - Shape.stagger;
		auto *one{editor_track::create_switch(firstshape, Paths, Style, Name)};
		auto *two{one != nullptr ? editor_track::create_switch(secondshape, {Paths[2], Paths[3]}, Style, Name + "b") : nullptr};
		auto *link{two != nullptr ? editor_track::create_path(Style, Paths[4]) : nullptr};
		if (link == nullptr)
		{
			m_switch.status = STR_C("The switch couldn't be created from this template");
			return {};
		}
		editor_track::commit({one, two, link});
		record_double(*one, *two, Shape);
		m_switch.status = format(STR_C("Double switch %s with its second switch %s inserted"), one->name().c_str(), two->name().c_str());
		return {one, two, link};
	}
	auto *track{editor_track::create_switch(Shape, Paths, Style, Name)};
	if (track == nullptr)
	{
		m_switch.status = STR_C("The switch couldn't be created from this template");
		return {};
	}
	editor_track::commit({track});
	record_switch(*track, Shape);
	m_switch.status = format(STR_C("Switch %s inserted, Ctrl+Z takes it back"), track->name().c_str());
	return {track};
}


bool editor_mode::plan_crossover(crossover_plan &Plan) const
{
	auto const &tool{m_extend};
	if (false == tool.active || tool.track == nullptr || tool.track->eType != tt_Switch || tool.track->m_paths.size() < 2 || tool.path != 1 || false == tool.atend || m_track_layout == track_layout::straight)
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
	auto const &control{(diverging.pieces.empty() ? diverging : diverging.pieces.back()).points[segment_data::point::control2]};
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
	editor_track::switch_template shape;
	shape.source = tool.track;
	shape.length = glm::dot(plan_of(main.points[segment_data::point::end] - origin), axis);
	for (int i = 0; i < 2; ++i)
	{
		auto const &path{own.m_paths[i]};
		auto &local{shape.local[i]};
		local = path;
		auto const tolocal = [&](glm::dvec3 const &Vector) { return glm::dvec3{glm::dot(plan_of(Vector), perpendicular) * side, Vector.y, glm::dot(plan_of(Vector), axis)}; };
		local.transform([&](glm::dvec3 const &Point) { return tolocal(Point - origin); }, tolocal);
		if (side < 0.0)
		{
			local.rolls = {-local.rolls[0], -local.rolls[1]};
			for (auto &piece : local.pieces)
				piece.rolls = {-piece.rolls[0], -piece.rolls[1]};
		}
	}
	Plan.shape = shape;
	auto const faced{connection::face(frog, tangent, connection::of(line), 1, shape)};
	if (false == faced.valid)
		return false;
	Plan.along = faced.partner.along;
	Plan.direction = faced.partner.forward;
	Plan.side = faced.partner.side;
	Plan.paths = faced.partner.paths;
	Plan.insert = faced.insert;
	return true;
}

std::string editor_mode::link_problem(editor_track::snap_target const &Target, editor_track::branch_link const &Link) const
{
	auto const name{Target.track != nullptr && false == Target.track->name().empty() ? Target.track->name() : std::string{STR_C("(no name)")}};
	if (false == Link.valid)
		return Link.shortfall > 0.0 ? format(STR_C("The branch can't lead into %s: the R %.0f m curve needs %.2f m more room"), name.c_str(), Link.radius, Link.shortfall) :
		                              format(STR_C("The branch can't lead into %s"), name.c_str());
	if (Link.trim > 0.0 && (false == stretchable(Target) || geometry::chord_length(Target.track->m_paths[Target.point.path]) < Link.trim + 1.0))
		return format(STR_C("%s would have to be shortened by %.2f m, it's too short or not straight"), name.c_str(), Link.trim);
	return {};
}

bool editor_mode::stretchable(editor_track::snap_target const &Target) const
{
	return Target.track != nullptr && Target.track->eType == tt_Normal && Target.track->m_paths.size() == 1 && editor_track::is_straight(*Target.track, m_straights.tolerance);
}

bool editor_mode::plan_switch_link(switch_link &Plan) const
{
	auto const &tool{m_switch};
	if (false == tool.placing || tool.curved || tool.connection >= 0 || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()) || tool.line.tracks.empty() || tool.templates[tool.armed].double_slip)
		return false;
	auto const &shape{tool.templates[tool.armed]};
	auto const &line{tool.line};
	std::vector<TTrack const *> const exclude(line.tracks.begin(), line.tracks.end());
	auto const category{line.tracks.front()->iCategoryFlag & 15};
	auto const orient = [&](glm::dvec3 const &Toward) {
		glm::dvec2 const offset{plan_of(Toward - tool.point)};
		Plan.forward = glm::dot(offset, line.direction) >= 0.0 ? 1 : -1;
		Plan.side = cross(line.direction * static_cast<double>(Plan.forward), offset) >= 0.0 ? 1 : -1;
	};
	auto const place = [&](double const Along) {
		auto const point{line.start + glm::dvec3{line.direction.x, line.grade, line.direction.y} * Along};
		return editor_track::place_switch(shape, point, line.direction * static_cast<double>(Plan.forward), Plan.side, line.grade * Plan.forward);
	};
	auto const &io{ImGui::GetIO()};
	if (false == m_cursor_override.has_value())
		Plan.target = snap_free_end(tool.mouse, {io.MousePos.x, io.MousePos.y}, nullptr, category, exclude);
	if (Plan.target.track == nullptr)
		Plan.target = editor_track::find_free_end(nullptr, category, tool.mouse, kLaySnapRadius, exclude);
	if (Plan.target.track != nullptr)
		orient(Plan.target.position);
	else
	{
		orient(tool.mouse);
		auto const paths{place(tool.along)};
		if (paths.size() < 2)
			return false;
		Plan.target = editor_track::find_free_end(nullptr, category, paths[1].points[segment_data::point::end], kLaySnapRadius, exclude);
		if (Plan.target.track == nullptr)
			return false;
	}
	if (shape.length > line.length - 0.02)
		return false;
	auto const kink{glm::radians(m_joints.kink_angle() * 0.5)};
	auto const plan = [&](double const Along) {
		Plan.along = Along;
		Plan.paths = place(Along);
		if (Plan.paths.size() < 2)
			return false;
		auto const radius{editor_track::branch_radius(Plan.paths[1])};
		Plan.link = editor_track::link_branch(branch_start(Plan.paths[1]), radius, branch_target(Plan.target), kink,
		                                      editor_track::reverse_insert(std::min(editor_track::diverging_speed(radius), design_speed(nullptr, Plan.target.track))), stretchable(Plan.target));
		Plan.reason = link_problem(Plan.target, Plan.link);
		return Plan.reason.empty();
	};
	auto const from{switch_from(line, shape.length, tool.along, Plan.forward, true)};
	auto const snapped{Plan.forward > 0 ? from : from + shape.length};
	auto const paths{place(snapped)};
	if (paths.size() < 2)
		return false;
	if (auto const shift{editor_track::branch_shift(branch_start(paths[1]), line.direction * static_cast<double>(Plan.forward), editor_track::branch_radius(paths[1]), branch_target(Plan.target), kink)})
	{
		auto const along{snapped + *shift * Plan.forward};
		auto const begin{Plan.forward > 0 ? along : along - shape.length};
		Plan.moved = std::abs(*shift);
		if (begin >= -1e-6 && begin + shape.length <= line.length + 1e-6 && plan(along) && false == Plan.link.reverse)
			return true;
	}
	Plan.moved = 0.0;
	return plan(snapped);
}

void editor_mode::insert_linked_switch(switch_link const &Plan)
{
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	if (false == place_switch_on_straight(m_switch.line, m_switch.templates[m_switch.armed], nullptr, Plan.along, Plan.forward, Plan.side, false, states, created, removed, switch_name_for_new()))
		return;
	auto const placed{created};
	link_tracks(Plan.target, Plan.link, Plan.target.track->eType == tt_Normal ? *Plan.target.track : *m_switch.line.tracks.front(), states, created);
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	m_switch.status.clear();
	switch_placed(placed);
	auto summary{link_summary(Plan.target, Plan.link, editor_track::branch_radius(Plan.paths[1]))};
	if (Plan.moved >= 0.005)
		summary += format(STR_C(", the switch moved %.2f m"), Plan.moved);
	m_switch.status = summary + (m_switch.status.empty() ? std::string{} : "\n" + m_switch.status);
	ui()->set_status(m_switch.status, false);
}

bool editor_mode::plan_branch_link(editor_track::branch_link &Link, std::string &Reason) const
{
	auto const &tool{m_extend};
	editor_track::snap_target target;
	Reason.clear();
	if (false == extend_snap(target) || tool.track == nullptr || tool.track->eType != tt_Switch || tool.track->m_paths.size() != 2 || tool.path != 1 || false == tool.atend)
		return false;
	if (std::find(tool.line.begin(), tool.line.end(), target.track) != tool.line.end())
		return false;
	auto const &diverging{tool.track->m_paths[1]};
	auto const radius{editor_track::branch_radius(diverging)};
	Link = editor_track::link_branch(branch_start(diverging), radius, branch_target(target), glm::radians(m_joints.kink_angle() * 0.5),
	                                 editor_track::reverse_insert(std::min(editor_track::diverging_speed(radius), design_speed(tool.track, target.track))), stretchable(target));
	Reason = link_problem(target, Link);
	return Reason.empty();
}

void editor_mode::finish_branch_link(editor_track::branch_link const &Link)
{
	auto *own{m_extend.track};
	auto const target{m_extend.snap};
	m_extend.active = false;
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	auto const &style{target.track->eType == tt_Normal || m_extend.line.empty() ? *target.track : *m_extend.line.front()};
	auto const linked{link_tracks(target, Link, style, states, created)};
	push_track_snapshot(std::move(states), std::move(created));
	m_node = linked.empty() ? own : linked.back();
	ui()->set_node(m_node);
	straight_refresh();
	ui()->set_status(link_summary(target, Link, editor_track::branch_radius(own->m_paths[1])), false);
}

std::vector<TTrack *> editor_mode::link_tracks(editor_track::snap_target const &Target, editor_track::branch_link const &Link, TTrack const &Style, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created)
{
	auto *target{Target.track};
	if (Link.trim != 0.0)
	{
		States.emplace_back(target, editor_track::capture(*target));
		editor_track::move_point(*target, Target.point, Link.landing);
		auto &path{target->m_paths[Target.point.path]};
		path.points[segment_data::point::control1] = glm::dvec3{0.0};
		path.points[segment_data::point::control2] = glm::dvec3{0.0};
		editor_track::commit({target});
	}
	std::vector<TTrack *> pieces;
	for (auto const &piece : Link.pieces)
		pieces.push_back(editor_track::create_path(Style, piece));
	editor_track::commit(pieces);
	digitize_drawn(pieces);
	Created.insert(Created.end(), pieces.begin(), pieces.end());
	return pieces;
}

std::vector<TTrack *> editor_mode::build_double_slip(glm::dvec3 const &PointA, glm::dvec3 const &PointB, glm::dvec3 const &PointC, glm::dvec3 const &PointD, glm::dvec2 const &Crossing, double const Height, glm::dvec2 const &First, glm::dvec2 const &Second, double const Angle, editor_track::slip_curve const &Curve, TTrack &Style, std::string const &Type)
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
	auto const whole{Curve.length()};
	// the part of the slip curve from Start along Heading, turning by Turn to Finish, between the distances From and To along it;
	// where it changes the radius made of pieces, with one curve from end to end for the versions which know no pieces
	auto const arc = [&](glm::dvec3 const &Start, glm::dvec2 const &Heading, double const Turn, glm::dvec3 const &Finish, double const From, double const To) {
		auto const place = [&](double const Along) {
			auto const local{Curve.point(Along)};
			auto const planar{plan_of(Start) + Heading * local.x + glm::dvec2{-Heading.y, Heading.x} * (Turn * local.y)};
			return glm::dvec3{planar.x, Start.y + (Finish.y - Start.y) * Along / whole, planar.y};
		};
		auto const bend = [&](double const Begin, double const End, double const Radius, double const Handle0, double const Handle1) {
			auto const d0{turned(Heading, Turn * Curve.heading(Begin))};
			auto const d3{turned(Heading, Turn * Curve.heading(End))};
			segment_data path;
			path.points[segment_data::point::start] = place(Begin);
			path.points[segment_data::point::end] = place(End);
			if (Radius > 0.0)
			{
				path.points[segment_data::point::control1] = {d0.x * Handle0, 0.0, d0.y * Handle0};
				path.points[segment_data::point::control2] = {-d3.x * Handle1, 0.0, -d3.y * Handle1};
			}
			path.radius = static_cast<float>(Radius);
			return path;
		};
		std::vector<std::pair<double, double>> zones;
		for (auto const &zone : {std::pair{Curve.lead, 0.0}, {Curve.lead + Curve.ends, Curve.radius}, {Curve.lead + Curve.ends + Curve.between, Curve.middle}, {whole - Curve.lead, Curve.radius}, {whole, 0.0}})
		{
			if (zone.first <= (zones.empty() ? 0.0 : zones.back().first) + 1e-9)
				continue;
			if (false == zones.empty() && zone.second == zones.back().second)
				zones.back().first = zone.first;
			else
				zones.push_back(zone);
		}
		// broken at the tips of the blades of the tables, so the model of the switch finds them there
		if (Curve.tip > 0.0)
			for (auto const cut : {Curve.tip, whole - Curve.tip})
				for (std::size_t i = 0; i < zones.size(); ++i)
					if (cut > (i > 0 ? zones[i - 1].first : 0.0) + 0.05 && cut < zones[i].first - 0.05)
					{
						zones.insert(zones.begin() + i, {cut, zones[i].second});
						break;
					}
		std::vector<segment_data> pieces;
		auto from{From};
		for (auto const &[to, radius] : zones)
		{
			auto const upto{std::min(To, to)};
			if (upto <= from + 1e-6)
				continue;
			auto const handle{radius > 0.0 ? 4.0 / 3.0 * std::tan((upto - from) / radius / 4.0) * radius : 0.0};
			pieces.push_back(bend(from, upto, radius, handle, handle));
			from = upto;
		}
		if (pieces.size() == 1)
			return pieces.front();
		// the handles of the curve from end to end reach 2/3 of the way to the crossing of the tangents
		auto const chord{plan_of(place(To) - place(From))};
		auto const d0{turned(Heading, Turn * Curve.heading(From))};
		auto const d3{turned(Heading, Turn * Curve.heading(To))};
		auto const sine{cross(d0, d3)};
		auto path{bend(From, To, Curve.radius, 2.0 / 3.0 * cross(chord, d3) / sine, 2.0 / 3.0 * cross(d0, chord) / sine)};
		path.pieces = std::move(pieces);
		return path;
	};
	auto const back = [](segment_data const &Path) {
		auto result{reversed(Path)};
		std::reverse(result.pieces.begin(), result.pieces.end());
		for (auto &piece : result.pieces)
			piece = reversed(piece);
		return result;
	};
	auto const middle{whole * 0.5};
	auto const split{gap};
	auto const centreA{at(Crossing - First * gap, Height)};
	auto const centreB{at(Crossing + First * gap, Height)};
	auto const centreC{at(Crossing - Second * gap, Height)};
	auto const centreD{at(Crossing + Second * gap, Height)};
	std::string base;
	auto const prefix{Type == "Skrz" ? "editor_skrz" : Type == "Rkp" ? "editor_dkp" : "editor_dks"};
	// a crossing keeps the names of its tracks, only its record holds the name
	auto const recorded = [](std::string const &Name) {
		auto const &records{switch_store::records()};
		return std::any_of(records.begin(), records.end(), [&](switch_store::record const &Record) { return Record.name == Name; });
	};
	for (int i = 1; base.empty() || simulation::Paths.find(base + "_a") != nullptr || simulation::Paths.find(base + "_d") != nullptr || recorded(base); ++i)
		base = prefix + std::to_string(i);
	std::vector<TTrack *> parts;
	if (Type == "Skrz")
	{
		// the two tracks just cross, their ends far from the crossing
		parts.push_back(editor_track::create_path(Style, straight(PointA, PointB)));
		parts.push_back(editor_track::create_path(Style, straight(PointC, PointD)));
		parts.erase(std::remove(parts.begin(), parts.end(), nullptr), parts.end());
		record_slip(parts, "crossing", Type, 0.0, Angle, base);
		return parts;
	}
	editor_track::switch_template shape;
	shape.length = glm::distance(plan_of(PointA), Crossing);
	bool const single{Type == "Rkp"};
	parts.push_back(editor_track::create_switch(shape, {straight(PointA, centreA), arc(PointA, First, side, PointD, 0.0, middle - split)}, Style, base + "_a"));
	if (single)
		parts.push_back(editor_track::create_path(Style, straight(PointB, centreB)));
	else
		parts.push_back(editor_track::create_switch(shape, {straight(PointB, centreB), back(arc(PointC, Second, -side, PointB, middle + split, whole))}, Style, base + "_b"));
	if (single)
		parts.push_back(editor_track::create_path(Style, straight(PointC, centreC)));
	else
		parts.push_back(editor_track::create_switch(shape, {straight(PointC, centreC), arc(PointC, Second, -side, PointB, 0.0, middle - split)}, Style, base + "_c"));
	parts.push_back(editor_track::create_switch(shape, {straight(PointD, centreD), back(arc(PointA, First, side, PointD, middle + split, whole))}, Style, base + "_d"));
	parts.push_back(editor_track::create_path(Style, straight(centreA, centreB)));
	parts.push_back(editor_track::create_path(Style, straight(centreC, centreD)));
	parts.push_back(editor_track::create_path(Style, arc(PointA, First, side, PointD, middle - split, middle + split)));
	if (false == single)
		parts.push_back(editor_track::create_path(Style, arc(PointC, Second, -side, PointB, middle - split, middle + split)));
	parts.erase(std::remove(parts.begin(), parts.end(), nullptr), parts.end());
	record_slip(parts, "slip", Type, Curve.radius, Angle, base, Curve.middle);
	return parts;
}

glm::dvec3 editor_mode::cursor_on_straight(editor_track::straight const &Line) const
{
	// the height of the straight under the cursor, worked out anew till it settles: on a grade the plane of the start of the
	// straight would put the cursor metres away from where it points at the track
	auto cursor{cursor_level(Line.start.y)};
	for (int i = 0; i < 4; ++i)
		cursor = cursor_level(Line.start.y + Line.grade * std::clamp(Line.along(cursor), 0.0, Line.length));
	return cursor;
}

bool editor_mode::plan_double_slip(editor_track::straight const &Line, glm::dvec3 const &Point, editor_track::switch_template const &Shape, slip_plan &Plan) const
{
	Plan = slip_plan{};
	// the crossing of the straight with the rail track nearest the cursor, whatever that track is: the slip goes there or
	// nowhere, never to another crossing around
	TTrack *nearest{nullptr};
	glm::dvec2 crossing{0.0};
	double best{kSlipReach};
	auto const category{Line.tracks.empty() ? 1 : Line.tracks.front()->iCategoryFlag & 15};
	for (auto *section : simulation::Region->sections(Point, 300.f))
		for (auto const &cell : section->m_cells)
			for (auto *track : cell.m_paths)
			{
				if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 15) != category || std::find(Line.tracks.begin(), Line.tracks.end(), track) != Line.tracks.end())
					continue;
				for (auto const &path : track->m_paths)
				{
					bezier const curve{path};
					auto const straight{path.points[segment_data::point::control1] == glm::dvec3{} && path.points[segment_data::point::control2] == glm::dvec3{}};
					int const pieces{straight ? 1 : 16};
					auto previous{plan_of(path.points[segment_data::point::start])};
					for (int i = 1; i <= pieces; ++i)
					{
						auto const next{straight ? plan_of(path.points[segment_data::point::end]) : plan_of(curve.point(static_cast<double>(i) / pieces))};
						auto const segment{next - previous};
						auto const determinant{cross(Line.direction, segment)};
						if (std::abs(determinant) > 1e-9)
						{
							auto const offset{previous - plan_of(Line.start)};
							auto const along{cross(offset, segment) / determinant};
							auto const fraction{cross(offset, Line.direction) / determinant};
							auto const position{plan_of(Line.start) + Line.direction * along};
							if (fraction >= -0.01 && fraction <= 1.01 && along >= 0.0 && along <= Line.length && glm::distance(position, plan_of(Point)) < best)
							{
								best = glm::distance(position, plan_of(Point));
								nearest = track;
								crossing = position;
							}
						}
						previous = next;
					}
				}
			}
	// a double slip nearer the cursor than any crossing is replaced by the one chosen
	if (Shape.type == "Rkpd")
	{
		TTrack *slip{nullptr};
		double closest{25.0};
		for (auto *section : simulation::Region->sections(Point, 40.f))
			for (auto const &cell : section->m_cells)
				for (auto *track : cell.m_paths)
					if (track->eType == tt_Switch && false == track->m_editorremoved && track->DoubleSlip() && glm::distance(plan_of(track->location()), plan_of(Point)) < closest)
					{
						closest = glm::distance(plan_of(track->location()), plan_of(Point));
						slip = track;
					}
		if (slip != nullptr && (nearest == nullptr || closest < best))
		{
			Plan.replaces = slip;
			return true;
		}
	}
	if (nearest == nullptr)
	{
		Plan.problem = STR_C("No track crosses the selected straight under the cursor");
		return false;
	}
	if (nearest->eType != tt_Normal)
	{
		Plan.problem = STR_C("The track crossing here is a switch: a slip goes only where two straights cross");
		return false;
	}
	if (false == editor_track::is_straight(*nearest, m_straights.tolerance))
	{
		Plan.problem = STR_C("The track crossing here isn't straight: a slip goes only where two straights cross");
		return false;
	}
	auto const second{editor_track::find_straight(*nearest, m_straights.tolerance)};
	if (second.tracks.empty())
	{
		Plan.problem = STR_C("The track crossing here isn't straight: a slip goes only where two straights cross");
		return false;
	}
	auto const first{Line.direction};
	auto direction{second.direction};
	if (glm::dot(first, direction) < 0.0)
		direction = -direction;
	auto const angle{std::atan2(std::abs(cross(first, direction)), glm::dot(first, direction))};
	bool const plain{Shape.type == "Skrz"};
	if (angle < glm::radians(0.5) || angle > glm::radians(plain ? 89.0 : 30.0))
	{
		Plan.problem = STR_C("The straights cross at an angle unsuitable for a double slip");
		return false;
	}
	auto const alongfirst{glm::dot(crossing - plan_of(Line.start), Line.direction)};
	auto const alongsecond{glm::dot(crossing - plan_of(second.start), second.direction)};
	auto const available{std::min({alongfirst, Line.length - alongfirst, alongsecond, second.length - alongsecond}) - 0.5};
	auto const *gauged{Line.tracks.front()};
	auto const blade{switch_design::secant_blade(Shape.rail, Shape.radius, Shape.ratio)};
	auto curve{editor_track::slip_of(Shape.radius, Shape.middle, angle, 0.5 * std::abs(gauged->fTrackWidth) + gauged->rail_dimensions().head + 0.002, blade.first, blade.second)};
	curve.tip = switch_design::cut_tip(Shape.rail, curve.radius);
	auto half{plain ? Shape.a : curve.tangent()};
	if (plain && half > available)
	{
		half = available;
		if (half < 2.0)
		{
			Plan.problem = STR_C("Not enough straight track around the crossing");
			return false;
		}
	}
	else if (half > available)
	{
		curve = curve.scaled(available / half);
		half = available;
		if (curve.radius < 50.0)
		{
			Plan.problem = STR_C("Not enough straight track around the crossing");
			return false;
		}
		Plan.note = format(STR_C("R reduced to %d m, the straights end %d m from the crossing"), static_cast<int>(curve.radius), static_cast<int>(available));
	}
	auto const heightfirst = [&](double const Along) { return Line.start.y + Line.grade * Along; };
	auto const heightsecond = [&](double const Along) { return second.start.y + second.grade * Along; };
	auto const sign{glm::dot(second.direction, direction) > 0.0 ? 1.0 : -1.0};
	auto const at = [&](glm::dvec2 const &Planar, double const Height) { return glm::dvec3{Planar.x, Height, Planar.y}; };
	Plan.other = nearest;
	Plan.crossing = crossing;
	Plan.first = first;
	Plan.direction = direction;
	Plan.angle = angle;
	Plan.alongfirst = alongfirst;
	Plan.half = half;
	Plan.curve = curve;
	Plan.points = {at(crossing - first * half, heightfirst(alongfirst - half)), at(crossing + first * half, heightfirst(alongfirst + half)), at(crossing - direction * half, heightsecond(alongsecond - sign * half)),
	               at(crossing + direction * half, heightsecond(alongsecond + sign * half))};
	return true;
}

bool editor_mode::insert_double_slip(editor_track::straight const &Line, glm::dvec3 const &Point, editor_track::switch_template const &Shape)
{
	auto &state{m_straights};
	state.status.clear();
	slip_plan plan;
	if (false == plan_double_slip(Line, Point, Shape, plan))
	{
		state.status = plan.problem;
		return false;
	}
	if (plan.replaces != nullptr)
		return replace_double_slip(*plan.replaces, Shape);
	state.status = plan.note;
	auto *other{plan.other};
	auto const crossing{plan.crossing};
	auto const first{plan.first};
	auto const direction{plan.direction};
	auto const angle{plan.angle};
	auto const alongfirst{plan.alongfirst};
	auto const half{plan.half};
	auto const curve{plan.curve};
	auto const heightfirst = [&](double const Along) { return Line.start.y + Line.grade * Along; };
	auto const &pointA{plan.points[0]};
	auto const &pointB{plan.points[1]};
	auto const &pointC{plan.points[2]};
	auto const &pointD{plan.points[3]};

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
			editor_track::revive(*track);
			editor_track::commit({track});
		}
		return false;
	}
	auto &styled{style != nullptr ? *style : *Line.tracks.front()};
	// a single slip joins the pair of the tracks on the side of the click
	auto const nearer{glm::distance((plan_of(pointA) + plan_of(pointD)) * 0.5, plan_of(Point)) <= glm::distance((plan_of(pointC) + plan_of(pointB)) * 0.5, plan_of(Point))};
	auto const parts{Shape.type == "Rkp" && false == nearer ? build_double_slip(pointC, pointD, pointA, pointB, crossing, heightfirst(alongfirst), direction, first, angle, curve, styled, Shape.type)
	                                                        : build_double_slip(pointA, pointB, pointC, pointD, crossing, heightfirst(alongfirst), first, direction, angle, curve, styled, Shape.type)};
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
	auto const blade{switch_design::secant_blade(Shape.rail, Shape.radius, Shape.ratio)};
	auto curve{editor_track::slip_of(Shape.radius, Shape.middle, angle, 0.5 * std::abs(switches[0]->fTrackWidth) + switches[0]->rail_dimensions().head + 0.002, blade.first, blade.second)};
	curve.tip = switch_design::cut_tip(Shape.rail, curve.radius);
	auto const full{curve.tangent()};
	auto radius{curve.radius};
	auto half{full};
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
				radius = curve.radius * half / full;
				state.status = format(STR_C("R limited to %d m, a neighbouring track isn't straight"), static_cast<int>(radius));
				break;
			}
			auto const line{editor_track::find_straight(*e->outer, m_straights.tolerance)};
			auto const reach{std::abs(glm::dot(plan_of(line.end) - crossing, -e->inward))};
			auto const reachstart{std::abs(glm::dot(plan_of(line.start) - crossing, -e->inward))};
			if (std::max(reach, reachstart) - 0.5 < half)
			{
				half = std::min(half, std::max(reach, reachstart) - 0.5);
				radius = curve.radius * half / full;
				state.status = format(STR_C("R reduced to %d m to fit the straights"), static_cast<int>(radius));
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
	auto const parts2{build_double_slip(points[0], points[1], points[2], points[3], crossing, height, first, direction, angle, curve.scaled(half / full), *style)};
	std::vector<TTrack *> commitlist(parts2.begin(), parts2.end());
	for (auto *track : created)
		if (std::find(commitlist.begin(), commitlist.end(), track) == commitlist.end())
			commitlist.push_back(track);
	editor_track::commit(commitlist);
	created.insert(created.end(), parts2.begin(), parts2.end());
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	// the drives of the switches replaced go with them, the new ones get theirs
	std::vector<scene::instance_handle> drives;
	for (auto const *track : switches)
		if (auto const drive{find_switch_drive(*track)})
			drives.push_back(drive->standing.instance);
	if (false == drives.empty() && remove_templates(drives) > 0)
		join_last_steps();
	for (auto *track : parts2)
		if (track != nullptr && track->eType == tt_Switch && place_switch_drive(*track))
			join_last_steps();
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
	auto const bend = [&](segment_data const &Local) {
		auto const &start{Local.points[segment_data::point::start]};
		auto const &end{Local.points[segment_data::point::end]};
		auto const &control1{Local.points[segment_data::point::control1]};
		auto const &control2{Local.points[segment_data::point::control2]};
		glm::dvec2 const chord{end.x - start.x, end.z - start.z};
		auto const localstart{glm::normalize(control1 != glm::dvec3{} ? plan_of(control1) : chord)};
		auto const localend{glm::normalize(control2 != glm::dvec3{} ? -plan_of(control2) : chord)};
		glm::dvec2 startdirection;
		glm::dvec2 enddirection;
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
		path.radius = Local.radius;
		{
			bezier const curve{path};
			auto const begin{plan_of(curve.first(0.0))};
			auto const finish{plan_of(curve.first(1.0))};
			auto const turn{glm::length(begin) > 1e-9 && glm::length(finish) > 1e-9 ? geometry::signed_angle(begin, finish) : 0.0};
			if (Local.radius != 0.f && std::abs(turn) > 1e-6)
				path.radius = static_cast<float>(std::copysign(curve.plan_length() / std::abs(turn), static_cast<double>(Local.radius)));
		}
		return path;
	};
	std::vector<segment_data> locals{Shape.local.begin(), Shape.local.end()};
	if (Shape.twofold())
	{
		locals.insert(locals.end(), Shape.second.begin(), Shape.second.end());
		locals.push_back(Shape.link);
	}
	std::vector<segment_data> result;
	for (std::size_t i = 0; i < locals.size(); ++i)
	{
		auto const &local{locals[i]};
		auto path{bend(local)};
		for (auto const &piece : local.pieces)
			path.pieces.push_back(bend(piece));
		// the main tracks follow the curve they are laid into
		if (i % 2 == 0)
			path.radius = sample_at(Frame, Station + Direction * 0.5 * (local.points[segment_data::point::start].z + local.points[segment_data::point::end].z)).radius;
		result.push_back(path);
	}
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
	auto const made{create_switch_set(shape, paths, *tool.frame_tracks.front(), switch_name_for_new())};
	created.insert(created.end(), made.begin(), made.end());
	if (made.size() == 1)
		tool.status = format(STR_C("Switch %s inserted into the curve, Ctrl+Z takes it back"), made.front()->name().c_str());
	auto const placed{created};
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	switch_placed(placed);
}
