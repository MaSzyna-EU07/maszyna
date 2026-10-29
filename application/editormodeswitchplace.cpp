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

// the name of the template in the language of the editor, editormodeswitchtool.cpp
std::string switch_name_translated(std::string Title);

namespace
{

segment_data straight_path(glm::dvec3 const &Start, glm::dvec3 const &End)
{
	segment_data path;
	path.points[segment_data::point::start] = Start;
	path.points[segment_data::point::end] = End;
	return path;
}

} // namespace

std::vector<double> editor_mode::switch_memo_key() const
{
	auto const &tool{m_switch};
	auto const &io{ImGui::GetIO()};
	auto const heading{tool.free_heading.value_or(glm::dvec2{9.0})};
	return {tool.mouse.x, tool.mouse.y, tool.mouse.z, tool.point.x, tool.point.z, io.MousePos.x, io.MousePos.y, heading.x, heading.y, tool.free_end.position.x, tool.free_end.position.z,
	        static_cast<double>(reinterpret_cast<std::uintptr_t>(tool.free_end.track)), tool.along, tool.pressed, static_cast<double>(tool.armed), static_cast<double>(tool.connection),
	        static_cast<double>(tool.placing) + 2.0 * tool.free + 4.0 * tool.curved + 8.0 * tool.one_sided + 16.0 * io.KeyShift + 32.0 * io.KeyCtrl + 64.0 * io.KeyAlt, static_cast<double>(m_history.size())};
}

std::vector<segment_data> editor_mode::switch_preview() const
{
	auto key{switch_memo_key()};
	if (false == m_switch_memo.fresh(key))
		m_switch_memo.keep(std::move(key), switch_preview_now());
	return m_switch_memo.result;
}

std::vector<segment_data> editor_mode::switch_preview_now() const
{
	auto const &tool{m_switch};
	if (false == tool.placing || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return {};
	if (tool.free)
		return free_switch_paths();
	if (tool.curved)
	{
		int forward;
		int side;
		curved_switch_pose(forward, side);
		if (tool.templates[tool.armed].double_slip)
		{
			curved_slip plan;
			if (false == plan_curved_slip(tool.templates[tool.armed], forward, side, plan))
				return {};
			auto lines{curved_slip_lines(plan, tool.templates[tool.armed].type)};
			auto const reaches{curved_slip_reaches(plan)};
			lines.insert(lines.end(), reaches.begin(), reaches.end());
			return lines;
		}
		return curved_switch_paths(tool.templates[tool.armed], tool.frame, curved_switch_start(forward), forward, side);
	}
	if (tool.templates[tool.armed].double_slip)
	{
		// the slip dragged out on a straight: its two straights crossing, the one it brings along through the quarter of the cursor
		slip_plan plan;
		if (false == plan_double_slip(tool.line, tool.point, tool.templates[tool.armed], plan, lone_slip_side()) || plan.replaces != nullptr)
			return {};
		return {straight_path(plan.ends[0], plan.ends[1]), straight_path(plan.ends[2], plan.ends[3])};
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
	tool.free = false;
	tool.status.clear();
	if (current_straight().tracks.empty())
	{
		auto *track{selected};
		// a slip is bent into the curve, in a side track only, a crossing goes only where two straights cross
		if (tool.templates[tool.armed].double_slip && tool.templates[tool.armed].type == "Skrz")
		{
			tool.status = refused(STR_C("A crossing goes only where two straights cross"));
			return false;
		}
		editor_track::curve curve;
		editor_track::chain chain;
		std::string error;
		if (false == editor_track::find_curve(*track, m_straights.tolerance, m_route.design.norms.gauge, curve))
		{
			tool.status = refused(STR_C("The selected track is neither a recognized straight nor a curve"));
			return false;
		}
		if (false == editor_track::find_chain(curve.from, curve.to, chain, error, true))
		{
			tool.status = refused(error.empty() ? std::string{STR_C("The curve can't be followed")} : error);
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
		// a crossover along the curve goes on onto the straights past its ends, its first switch drawn so as it's dragged
		if (tool.connection >= 0)
		{
			std::vector<curve_sample> frame;
			std::vector<TTrack *> tracks;
			if (curve_of(*track, m_straights.tolerance, m_route.design.norms.gauge, frame, tracks, kCrossoverStraights))
			{
				tool.frame = std::move(frame);
				tool.frame_tracks = std::move(tracks);
			}
		}
		if (tool.frame.size() < 2)
			return false;
		tool.curved = true;
		tool.along = nearest_station(tool.frame, ground);
		tool.pressed = tool.along;
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
		// at a crossing, where it was pressed; with no track crossing here it's dragged out like a switch, the crossing track it brings
		// along through the quarter the cursor is let go in. Laid only once dragged, as a switch is
		slip_plan plan;
		auto const lone{false == plan_double_slip(line, cursor, tool.templates[tool.armed], plan) && plan.lone};
		tool.along = along;
		tool.point = lone ? point : cursor;
		tool.mouse = cursor;
		tool.line = line;
		tool.placing = true;
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
	if (tool.free)
	{
		tool.placing = false;
		insert_free_switch();
		return;
	}
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
	if (tool.templates[tool.armed].double_slip && false == tool.curved)
	{
		insert_double_slip(tool.line, tool.point, tool.templates[tool.armed], lone_slip_side());
		return;
	}
	if (linked)
	{
		insert_linked_switch(plan);
		return;
	}
	if (tool.curved)
	{
		int forward;
		int side;
		curved_switch_pose(forward, side);
		if (tool.templates[tool.armed].double_slip)
		{
			insert_curved_slip(forward, side);
			return;
		}
		tool.along = curved_switch_start(forward);
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


bool editor_mode::free_switch_offered() const
{
	auto const &tool{m_switch};
	return m_track_tab == track_tab::turnout && tool.connection < 0 && tool.armed >= 0 && tool.armed < static_cast<int>(tool.templates.size()) && false == tool.templates[tool.armed].double_slip;
}

editor_track::snap_target editor_mode::free_switch_end(glm::dvec3 const &Ground) const
{
	// the free end under the cursor on the screen, however high the tracks are against the ground the cursor is on: a scenery built
	// below or above its terrain; else the nearest one within reach in the plan
	if (false == m_cursor_override.has_value() && ImGui::IsMousePosValid())
	{
		auto const &io{ImGui::GetIO()};
		if (auto const end{snap_free_end(Ground, {io.MousePos.x, io.MousePos.y}, nullptr, kRailCategory, {})}; end.track != nullptr)
			return end;
	}
	editor_track::snap_target result;
	auto nearest{kLaySnapRadius};
	for (auto const &end : editor_track::free_ends(nullptr, kRailCategory, Ground, kLaySnapRadius + 100.0, {}))
		if (auto const distance{plan_distance(end.position, Ground)}; distance < nearest)
		{
			nearest = distance;
			result = end;
		}
	return result;
}

void editor_mode::start_free_switch(glm::dvec3 const &Ground)
{
	auto &tool{m_switch};
	if (false == free_switch_offered())
		return;
	tool.status.clear();
	tool.curved = false;
	tool.line = {};
	tool.free = true;
	tool.free_end = free_switch_end(Ground);
	tool.free_heading.reset();
	tool.one_sided = false;
	if (tool.free_end.track != nullptr)
	{
		// going on from the free end in its direction
		auto const outward{plan_of(-tool.free_end.direction)};
		if (glm::length(outward) > 1e-9)
			tool.free_heading = glm::normalize(outward);
		tool.point = tool.free_end.position;
	}
	else
		tool.point = lay_bed(Ground);
	tool.mouse = tool.point;
	tool.placing = true;
}

void editor_mode::free_switch_pose(glm::dvec3 &Start, glm::dvec2 &Heading, int &Side, double &Grade) const
{
	auto const &tool{m_switch};
	Start = tool.point;
	glm::dvec2 const offset{plan_of(tool.mouse - tool.point)};
	// till the drag is long enough to take the direction from, it follows the cursor, the diverging track to the left
	Heading = tool.free_heading.value_or(glm::length(offset) > 1e-6 ? glm::normalize(offset) : glm::dvec2{0.0, 1.0});
	Side = cross(Heading, offset) >= 0.0 ? 1 : -1;
	Grade = 0.0;
	if (tool.free_end.track != nullptr)
	{
		// the grade of the track it goes on from
		auto const &into{tool.free_end.direction};
		auto const run{glm::length(plan_of(into))};
		Grade = run > 1e-9 ? -into.y / run : 0.0;
	}
}

std::vector<segment_data> editor_mode::free_switch_paths() const
{
	auto const &tool{m_switch};
	if (tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return {};
	glm::dvec3 start;
	glm::dvec2 heading;
	int side;
	double grade;
	free_switch_pose(start, heading, side, grade);
	// dragged on past its end from a free end: bent, with the track into the free end the cursor is at
	if (free_bend bend; free_switch_bend(bend))
	{
		auto paths{bend.paths};
		if (bend.target.track != nullptr && bend.problem.empty())
			paths.insert(paths.end(), bend.link.pieces.begin(), bend.link.pieces.end());
		return paths;
	}
	return editor_track::place_switch(tool.templates[tool.armed], start, heading, side, grade);
}

void editor_mode::insert_free_switch()
{
	auto &tool{m_switch};
	free_bend bend;
	auto const bent{free_switch_bend(bend)};
	tool.free = false;
	// a click with no drag on the ground gives no direction; from a free end it's the end's
	if (tool.free_end.track == nullptr && false == tool.free_heading.has_value())
	{
		tool.status = refused(STR_C("Drag along the main track of the switch, then sideways to the side of the diverging one"));
		return;
	}
	auto const paths{bent ? bend.paths : free_switch_paths()};
	if (paths.size() < 2)
		return;
	auto const made{create_switch_set(tool.templates[tool.armed], paths, tool.free_end.track, switch_name_for_new())};
	if (made.empty())
		return;
	// the track into the free end the cursor was at, in the same step of the history
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created{made};
	auto const linked{bent && bend.target.track != nullptr && bend.problem.empty()};
	if (linked)
		link_tracks(bend.target, bend.link, bend.target.track->eType == tt_Normal ? *bend.target.track : *tool.free_end.track, states, created);
	push_track_snapshot(std::move(states), std::move(created), {});
	straight_refresh();
	switch_placed(made);
	auto text{tool.free_end.track != nullptr ? format(STR_C("Switch %s laid on from %s, its other ends free: lay on from them"), made.front()->name().c_str(), tool.free_end.track->name().c_str())
	                                         : format(STR_C("Switch %s laid with no track, its ends free: lay on from them"), made.front()->name().c_str())};
	if (bent && bend.radius > 0.0)
		text = format(STR_C("Switch %s laid on from %s bent into R %.0f m, %s"), made.front()->name().c_str(), tool.free_end.track->name().c_str(), bend.radius,
		              bend.side == bend.turn ? STR_C("one-sided") : STR_C("two-sided"));
	if (linked)
		text += "\n" + link_summary(bend.target, bend.link, editor_track::branch_radius(bend.paths[bend.diverging ? 1 : 0]));
	else if (bent && false == bend.problem.empty())
		text += "\n" + bend.problem;
	if (bent)
		for (auto const &warning : bend.report.warnings)
			text += "\n" + warning;
	for (auto const &warning : free_switch_exits(paths))
		text += "\n" + warning;
	tool.status = text + (tool.status.empty() ? std::string{} : "\n" + tool.status);
	ui()->set_status(tool.status, false);
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
		m_switch.status = refused(reason);
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
		m_switch.status = refused(format(STR_C("The straight is %.1f m long, too short for the %.1f m switch"), Line.length, shape.length));
		return false;
	}
	auto const from{switch_from(Line, shape.length, Along, Direction, Snap)};
	auto const to{from + shape.length};
	auto const origin{Direction > 0 ? from : to};
	TTrack *first{nullptr};
	if (false == cut_straight(Line, from, to, States, Created, Removed, &first))
	{
		m_switch.status = refused(STR_C("The switch would overlap another switch, move it onto plain track"));
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
	return create_switch_set(Shape, Paths, &Style, Name);
}

std::vector<TTrack *> editor_mode::create_switch_set(editor_track::switch_template const &Shape, std::vector<segment_data> const &Paths, TTrack const *Style, std::string const &Name)
{
	// with no track to copy, styled as the lay tool lays its tracks
	auto const switch_of = [&](editor_track::switch_template const &Template, std::vector<segment_data> const &Each, std::string const &Named) {
		return Style != nullptr ? editor_track::create_switch(Template, Each, *Style, Named) : editor_track::create_switch(Template, Each, m_lay.style, Named);
	};
	if (Shape.twofold() && Paths.size() >= 5)
	{
		auto firstshape{Shape};
		firstshape.length = glm::distance(Paths[0].points[segment_data::point::start], Paths[0].points[segment_data::point::end]);
		auto secondshape{Shape};
		secondshape.length = Shape.length - Shape.stagger;
		auto *one{switch_of(firstshape, Paths, Name)};
		auto *two{one != nullptr ? switch_of(secondshape, {Paths[2], Paths[3]}, Name + "b") : nullptr};
		auto *link{two == nullptr ? nullptr : Style != nullptr ? editor_track::create_path(*Style, Paths[4]) : editor_track::create_path(m_lay.style, Paths[4])};
		if (link == nullptr)
		{
			m_switch.status = refused(STR_C("The switch couldn't be created from this template"));
			return {};
		}
		editor_track::commit({one, two, link});
		record_double(*one, *two, Shape);
		m_switch.status = format(STR_C("Double switch %s with its second switch %s inserted"), one->name().c_str(), two->name().c_str());
		return {one, two, link};
	}
	auto *track{switch_of(Shape, Paths, Name)};
	if (track == nullptr)
	{
		m_switch.status = refused(STR_C("The switch couldn't be created from this template"));
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

namespace
{

// the part of the slip curve from Start along Heading, turning by Turn to Finish, between the distances From and To along it; where it
// changes the radius made of pieces, with one curve from end to end for the versions which know no pieces
segment_data slip_arc(editor_track::slip_curve const &Curve, glm::dvec3 const &Start, glm::dvec2 const &Heading, double const Turn, glm::dvec3 const &Finish, double const From, double const To)
{
	auto const whole{Curve.length()};
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
}

} // namespace

std::vector<TTrack *> editor_mode::build_double_slip(glm::dvec3 const &PointA, glm::dvec3 const &PointB, glm::dvec3 const &PointC, glm::dvec3 const &PointD, glm::dvec2 const &Crossing, double const Height, glm::dvec2 const &First, glm::dvec2 const &Second, double const Angle, editor_track::slip_curve const &Curve, TTrack &Style, std::string const &Type, std::function<segment_data(segment_data const &)> const &Bend)
{
	auto const at = [](glm::dvec2 const &Planar, double const Y) { return glm::dvec3{Planar.x, Y, Planar.y}; };
	// the ends of the two diagonals at the crossing are kept apart further than the simulation joins the ends, 2 cm
	auto const halfsine{std::sqrt(std::max(1e-6, (1.0 - glm::dot(First, Second)) * 0.5))};
	auto const gap{std::max(0.25, 0.06 / halfsine)};
	auto const side{cross(First, Second) > 0.0 ? 1.0 : -1.0};
	auto const whole{Curve.length()};
	auto const arc = [&](glm::dvec3 const &Start, glm::dvec2 const &Heading, double const Turn, glm::dvec3 const &Finish, double const From, double const To) { return slip_arc(Curve, Start, Heading, Turn, Finish, From, To); };
	// laid out on a straight, bent where it goes into a curve
	auto const laid = [&](segment_data const &Path) { return Bend ? Bend(Path) : Path; };
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
	for (int i = 1; base.empty() || simulation::Paths.find(base + "_a") != nullptr || simulation::Paths.find(base + "_d") != nullptr || switch_store::has(base); ++i)
		base = prefix + std::to_string(i);
	std::vector<TTrack *> parts;
	if (Type == "Skrz")
	{
		// the two tracks just cross, their ends far from the crossing
		parts.push_back(editor_track::create_path(Style, laid(straight_path(PointA, PointB))));
		parts.push_back(editor_track::create_path(Style, laid(straight_path(PointC, PointD))));
		parts.erase(std::remove(parts.begin(), parts.end(), nullptr), parts.end());
		record_slip(parts, "crossing", Type, 0.0, Angle, base);
		return parts;
	}
	editor_track::switch_template shape;
	shape.length = glm::distance(plan_of(PointA), Crossing);
	bool const single{Type == "Rkp"};
	parts.push_back(editor_track::create_switch(shape, {laid(straight_path(PointA, centreA)), laid(arc(PointA, First, side, PointD, 0.0, middle - split))}, Style, base + "_a"));
	if (single)
		parts.push_back(editor_track::create_path(Style, laid(straight_path(PointB, centreB))));
	else
		parts.push_back(editor_track::create_switch(shape, {laid(straight_path(PointB, centreB)), laid(back(arc(PointC, Second, -side, PointB, middle + split, whole)))}, Style, base + "_b"));
	if (single)
		parts.push_back(editor_track::create_path(Style, laid(straight_path(PointC, centreC))));
	else
		parts.push_back(editor_track::create_switch(shape, {laid(straight_path(PointC, centreC)), laid(arc(PointC, Second, -side, PointB, 0.0, middle - split))}, Style, base + "_c"));
	parts.push_back(editor_track::create_switch(shape, {laid(straight_path(PointD, centreD)), laid(back(arc(PointA, First, side, PointD, middle + split, whole)))}, Style, base + "_d"));
	parts.push_back(editor_track::create_path(Style, laid(straight_path(centreA, centreB))));
	parts.push_back(editor_track::create_path(Style, laid(straight_path(centreC, centreD))));
	parts.push_back(editor_track::create_path(Style, laid(arc(PointA, First, side, PointD, middle - split, middle + split))));
	if (false == single)
		parts.push_back(editor_track::create_path(Style, laid(arc(PointC, Second, -side, PointB, middle - split, middle + split))));
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

bool editor_mode::plan_double_slip(editor_track::straight const &Line, glm::dvec3 const &Point, editor_track::switch_template const &Shape, slip_plan &Plan, int const Side) const
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
	// a double slip nearer the cursor than any crossing is replaced by the one chosen, one bent into a curve isn't: it's laid out of
	// straights
	if (Shape.type == "Rkpd")
	{
		TTrack *slip{nullptr};
		double closest{25.0};
		auto const bent = [](TTrack const &Track) {
			auto const known{switch_store::named(Track)};
			return known.has_value() && known->main_radius != 0.0;
		};
		for (auto *section : simulation::Region->sections(Point, 40.f))
			for (auto const &cell : section->m_cells)
				for (auto *track : cell.m_paths)
					if (track->eType == tt_Switch && false == track->m_editorremoved && track->DoubleSlip() && glm::distance(plan_of(track->location()), plan_of(Point)) < closest && false == bent(*track))
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
	// the curves of the slip for the angle of the tracks crossing, as long as the straights have room for them
	bool const plain{Shape.type == "Skrz"};
	auto const shaped = [&](double const Angle, double const Available) {
		auto const *gauged{Line.tracks.front()};
		auto const blade{switch_design::secant_blade(Shape.rail, Shape.radius, Shape.ratio)};
		auto curve{editor_track::slip_of(Shape.radius, Shape.middle, Angle, 0.5 * std::abs(gauged->fTrackWidth) + gauged->rail_dimensions().head + 0.002, blade.first, blade.second)};
		curve.tip = switch_design::cut_tip(Shape.rail, curve.radius);
		auto half{plain ? Shape.a : curve.tangent()};
		if (plain && half > Available)
		{
			half = Available;
			if (half < 2.0)
			{
				Plan.problem = STR_C("Not enough straight track around the crossing");
				return false;
			}
		}
		else if (half > Available)
		{
			curve = curve.scaled(Available / half);
			half = Available;
			if (curve.radius < 50.0)
			{
				Plan.problem = STR_C("Not enough straight track around the crossing");
				return false;
			}
			Plan.note = format(STR_C("R reduced to %d m, the straights end %d m from the crossing"), static_cast<int>(curve.radius), static_cast<int>(Available));
		}
		Plan.curve = curve;
		Plan.half = half;
		return true;
	};
	if (nearest == nullptr)
	{
		Plan.lone = true;
		Plan.problem = STR_C("No track crosses the selected straight under the cursor");
		if (Side == 0 || Line.tracks.empty())
			return false;
		// no track crossing here: the slip brings the crossing one along, at its own angle, ahead to the side given, its ends free
		auto const angle{std::atan(1.0 / (Shape.ratio > 0.0 ? Shape.ratio : 9.0))};
		auto const first{Line.direction};
		auto const direction{turned(first, static_cast<double>(Side) * angle)};
		auto const alongfirst{std::clamp(glm::dot(plan_of(Point) - plan_of(Line.start), first), 0.0, Line.length)};
		auto const crossing{plan_of(Line.start) + first * alongfirst};
		if (false == shaped(angle, std::min(alongfirst, Line.length - alongfirst) - 0.5))
			return false;
		Plan.problem.clear();
		auto const half{Plan.half};
		auto const height = [&](double const Along) { return Line.start.y + Line.grade * Along; };
		auto const at = [&](glm::dvec2 const &Planar, double const Height) { return glm::dvec3{Planar.x, Height, Planar.y}; };
		Plan.crossing = crossing;
		Plan.first = first;
		Plan.direction = direction;
		Plan.angle = angle;
		Plan.alongfirst = alongfirst;
		Plan.points = {at(crossing - first * half, height(alongfirst - half)), at(crossing + first * half, height(alongfirst + half)), at(crossing - direction * half, height(alongfirst - half * std::cos(angle))),
		               at(crossing + direction * half, height(alongfirst + half * std::cos(angle)))};
		// the straights of both tracks out to the joints of the template, past the frogs of the diamond, as far as the line has room
		auto const reach{std::max(half, std::min(Shape.a, std::min(alongfirst, Line.length - alongfirst) - 0.5))};
		Plan.reach = reach;
		Plan.ends = {at(crossing - first * reach, height(alongfirst - reach)), at(crossing + first * reach, height(alongfirst + reach)), at(crossing - direction * reach, height(alongfirst - reach * std::cos(angle))),
		             at(crossing + direction * reach, height(alongfirst + reach * std::cos(angle)))};
		return true;
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
	if (angle < glm::radians(0.5) || angle > glm::radians(plain ? 89.0 : 30.0))
	{
		Plan.problem = STR_C("The straights cross at an angle unsuitable for a double slip");
		return false;
	}
	if (auto const skew{slip_skew(Shape, angle)}; false == skew.empty())
	{
		Plan.problem = skew;
		return false;
	}
	auto const alongfirst{glm::dot(crossing - plan_of(Line.start), Line.direction)};
	auto const alongsecond{glm::dot(crossing - plan_of(second.start), second.direction)};
	auto const available{std::min({alongfirst, Line.length - alongfirst, alongsecond, second.length - alongsecond}) - 0.5};
	if (false == shaped(angle, available))
		return false;
	auto const half{Plan.half};
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
	Plan.points = {at(crossing - first * half, heightfirst(alongfirst - half)), at(crossing + first * half, heightfirst(alongfirst + half)), at(crossing - direction * half, heightsecond(alongsecond - sign * half)),
	               at(crossing + direction * half, heightsecond(alongsecond + sign * half))};
	// the tracks crossing are there already, nothing laid out past the slip
	Plan.reach = half;
	Plan.ends = Plan.points;
	return true;
}

std::string editor_mode::slip_skew(editor_track::switch_template const &Shape, double const Angle) const
{
	// rad: the tracks crossing at the skos of the slip to within this, half of what keeps 1:9.51 and 1:10 of the catalogue apart
	auto const tolerance{glm::radians(0.14)};
	auto const fits = [&](editor_track::switch_template const &Template) { return Template.ratio > 0.0 && std::abs(Angle - std::atan(1.0 / Template.ratio)) <= tolerance; };
	if (Shape.type == "Skrz" || false == Shape.double_slip || Shape.ratio <= 0.0 || fits(Shape))
		return {};
	// a slip of the same kind made for the skos of the tracks, of the same rail and radius before the others
	editor_track::switch_template const *fitting{nullptr};
	auto best{-1};
	for (auto const &entry : m_switch.templates)
	{
		if (false == entry.double_slip || entry.type != Shape.type || false == fits(entry))
			continue;
		auto const rank{(entry.rail == Shape.rail ? 2 : 0) + (std::abs(entry.radius - Shape.radius) < 0.5 ? 1 : 0)};
		if (rank > best)
		{
			best = rank;
			fitting = &entry;
		}
	}
	auto const ratio{1.0 / std::tan(Angle)};
	if (fitting == nullptr)
		return format(STR_C("The straights cross at 1:%.2f, %s is made for 1:%.2f and no slip template for that: drag the slip out on one straight, its crossing track comes along at its skos"), ratio, switch_name_translated(Shape.title()).c_str(), Shape.ratio);
	return format(STR_C("The straights cross at 1:%.2f, %s is made for 1:%.2f: %s fits that crossing"), ratio, switch_name_translated(Shape.title()).c_str(), Shape.ratio, switch_name_translated(fitting->title()).c_str());
}

bool editor_mode::insert_double_slip(editor_track::straight const &Line, glm::dvec3 const &Point, editor_track::switch_template const &Shape, int const Side)
{
	auto &state{m_straights};
	state.status.clear();
	slip_plan plan;
	if (false == plan_double_slip(Line, Point, Shape, plan, Side))
	{
		state.status = refused(plan.problem);
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
	// a slip bringing its crossing track along takes the line up to the joints of its template
	auto const reach{other == nullptr ? std::max(half, plan.reach) : half};
	if (false == cut_straight(Line, alongfirst - reach, alongfirst + reach, states, created, removed, &style))
		return false;
	auto const secondline{other != nullptr ? editor_track::find_straight(*other, m_straights.tolerance) : editor_track::straight{}};
	auto const secondalong{other != nullptr ? glm::dot(crossing - plan_of(secondline.start), secondline.direction) : 0.0};
	if (other != nullptr && (secondline.tracks.empty() || false == cut_straight(secondline, secondalong - half, secondalong + half, states, created, removed, nullptr)))
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
	std::vector<TTrack *> made{parts};
	// the straights of both tracks from the ends of the slip out to the joints of the template
	if (reach > half + 0.01)
		for (auto const &[from, to] : {std::pair{plan.ends[0], pointA}, {pointB, plan.ends[1]}, {plan.ends[2], pointC}, {pointD, plan.ends[3]}})
			if (auto *straight{editor_track::create_path(styled, straight_path(from, to))})
				made.push_back(straight);
	editor_track::commit(made);
	created.insert(created.end(), made.begin(), made.end());
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	switch_placed(parts);
	if (other == nullptr)
	{
		state.status = format(STR_C("The slip laid with its crossing track at 1:%.1f, the ends of that track free: lay on from them"), 1.0 / std::tan(angle)) + (plan.note.empty() ? std::string{} : "\n" + plan.note);
		ui()->set_status(state.status, false);
	}
	return true;
}

int editor_mode::lone_slip_side() const
{
	auto const &tool{m_switch};
	glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
	auto const &first{tool.line.direction};
	// the crossing track through the quarter the cursor is in: ahead to the left or behind to the right turns it to the left
	auto const left{cross(first, offset) >= 0.0};
	auto const ahead{glm::dot(first, offset) >= 0.0};
	return left == ahead ? 1 : -1;
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
			state.status = refused(STR_C("The double slip isn't complete, it can't be replaced"));
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
		state.status = refused(STR_C("The double slip geometry isn't recognized"));
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
	if (auto const skew{slip_skew(Shape, angle)}; false == skew.empty())
	{
		state.status = refused(skew);
		return false;
	}
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
		// the first point of a track going on right where the one before ended is that one's last; one past a switch the chain leaves
		// out starts apart from it, and its first point is taken too, the joint there on the frame where it is
		auto const continuous{false == result.empty() && glm::distance(curve.point(forward ? 0.0 : 1.0), result.back().position) < 1e-3};
		for (int k = (result.empty() || false == continuous ? 0 : 1); k <= steps; ++k)
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

std::vector<switch_curve::track_point> editor_mode::frame_points(std::vector<curve_sample> const &Frame) const
{
	auto const gauge{m_route.design.norms.gauge};
	std::vector<switch_curve::track_point> points;
	for (auto const &sample : Frame)
	{
		switch_curve::track_point point;
		point.station = sample.station;
		point.heading = std::atan2(sample.tangent.y, sample.tangent.x);
		// the headings go on through a half turn, so that their differences are the turns between them
		if (false == points.empty())
			point.heading = points.back().heading + std::remainder(point.heading - points.back().heading, 2.0 * glm::pi<double>());
		point.cant = gauge * std::sin(glm::radians(static_cast<double>(sample.roll)));
		points.push_back(point);
	}
	return points;
}

void editor_mode::curved_switch_pose(int &Direction, int &Side) const
{
	auto const &tool{m_switch};
	auto const sample{sample_at(tool.frame, tool.pressed)};
	glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
	Direction = glm::dot(offset, sample.tangent) >= 0.0 ? 1 : -1;
	glm::dvec2 const direction{sample.tangent * static_cast<double>(Direction)};
	Side = direction.x * offset.y - direction.y * offset.x >= 0.0 ? 1 : -1;
}

double editor_mode::curved_switch_start(int const Direction, std::string *What) const
{
	auto const &tool{m_switch};
	if (What != nullptr)
		What->clear();
	if (tool.frame.size() < 2 || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return tool.pressed;
	auto const length{tool.templates[tool.armed].length};
	std::string what;
	auto start{switch_curve::snapped_start(switch_curve::ramp_ends(frame_points(tool.frame)), tool.pressed, length, Direction, 3.0, &what)};
	// as on a straight, an end of the switch within half a metre of a joint of the tracks goes right onto it: no path of a few centimetres
	// left beside it, no gap at it
	constexpr double kJointPull{0.5};
	auto nearest{kJointPull};
	auto shift{0.0};
	auto const first{Direction > 0 ? start : start - length};
	for (auto const *track : tool.frame_tracks)
	{
		if (track == nullptr || track->m_editorremoved || track->m_paths.empty())
			continue;
		for (auto const index : {segment_data::point::start, segment_data::point::end})
		{
			auto const joint{nearest_station(tool.frame, track->m_paths.front().points[index])};
			for (auto const end : {first, first + length})
				if (auto const off{std::abs(joint - end)}; off > 1e-4 && off < nearest)
				{
					nearest = off;
					shift = joint - end;
				}
		}
	}
	if (shift != 0.0)
	{
		start += shift;
		what = STR("The switch drawn to the joint of the tracks");
	}
	// a start which would put the switch off the curve isn't one
	auto const total{tool.frame.back().station};
	if (Direction > 0 ? start < 0.0 || start > total - length : start < length || start > total)
		return tool.pressed;
	if (What != nullptr)
		*What = what;
	return start;
}

namespace
{

// where a switch with a straight frog stops being bent, as the DB bends such switches into a curve: from the start of the switch up to the
// frog, both tracks straight over the frog; along the main track from the start of the switch, where the straight the diverging track
// ends with begins. None for a switch with its curve running through the frog, the ones of table 11.9, nor a double one
std::optional<double> frog_zone(editor_track::switch_template const &Shape)
{
	if (Shape.frog_straight < 0.5 || Shape.twofold() || Shape.double_slip || Shape.type != "Rz")
		return std::nullopt;
	auto const &diverging{Shape.local[1]};
	auto const &last{diverging.pieces.empty() ? diverging : diverging.pieces.back()};
	auto const &end{last.points[segment_data::point::end]};
	auto const &control{last.points[segment_data::point::control2]};
	auto direction{control != glm::dvec3{} ? -plan_of(control) : plan_of(end - last.points[segment_data::point::start])};
	if (glm::length(direction) < 1e-9)
		return std::nullopt;
	direction = glm::normalize(direction);
	// in the plan of a template x runs across the main track, z along it
	auto const along{end.z - Shape.frog_straight * direction.y};
	if (along < 1.0 || along > Shape.length - 0.5)
		return std::nullopt;
	return along;
}

// the pieces of a path of a switch template, one crossing the place along the main track split there
std::vector<segment_data> split_along(segment_data const &Path, double const Along)
{
	std::vector<segment_data> const pieces{Path.pieces.empty() ? std::vector<segment_data>{Path} : Path.pieces};
	std::vector<segment_data> result;
	for (auto const &piece : pieces)
	{
		auto const &start{piece.points[segment_data::point::start]};
		auto const &end{piece.points[segment_data::point::end]};
		if (start.z > Along - 1e-3 || end.z < Along + 1e-3)
		{
			result.push_back(piece);
			continue;
		}
		bezier const curve{piece};
		double low{0.0};
		double high{1.0};
		for (int i = 0; i < 60; ++i)
		{
			auto const middle{0.5 * (low + high)};
			(curve.point(middle).z < Along ? low : high) = middle;
		}
		auto const t{0.5 * (low + high)};
		for (auto [from, to] : {std::pair{0.0, t}, std::pair{t, 1.0}})
		{
			auto part{geometry::part_of(piece, from, to)};
			part.pieces.clear();
			result.push_back(part);
		}
	}
	return result;
}

// a path from its start, in the plan, a point every quarter of a metre
std::vector<switch_curve::track_point> plan_points(segment_data const &Path)
{
	std::vector<switch_curve::track_point> points;
	auto const add = [&](segment_data const &Piece) {
		bezier const curve{Piece};
		auto const steps{std::max(4, static_cast<int>(std::ceil(curve.plan_length() / 0.25)))};
		for (int k = points.empty() ? 0 : 1; k <= steps; ++k)
		{
			auto const t{static_cast<double>(k) / steps};
			auto const position{plan_of(curve.point(t))};
			auto const tangent{plan_of(curve.first(t))};
			switch_curve::track_point point;
			point.heading = std::atan2(tangent.y, tangent.x);
			if (false == points.empty())
			{
				point.station = points.back().station + glm::distance(position, plan_of(curve.point(static_cast<double>(k - 1) / steps)));
				point.heading = points.back().heading + std::remainder(point.heading - points.back().heading, 2.0 * glm::pi<double>());
			}
			points.push_back(point);
		}
	};
	if (Path.pieces.empty())
		add(Path);
	for (auto const &piece : Path.pieces)
		add(piece);
	return points;
}

} // namespace

switch_curve::switch_report editor_mode::curved_switch_report(std::vector<segment_data> const &Paths, std::vector<curve_sample> const &Frame, double const Station, int const Direction) const
{
	auto const &tool{m_switch};
	if (Paths.size() < 2 || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return {};
	auto const &shape{tool.templates[tool.armed]};
	auto const frog{frog_zone(shape)};
	return switch_curve::check(plan_points(Paths[0]), plan_points(Paths[1]), frame_points(Frame), Station, Station + Direction * shape.length, shape.radius, shape.ratio, frog.has_value() ? shape.length - *frog : 0.0);
}

segment_data editor_mode::frame_path(segment_data const &Local, std::vector<curve_sample> const &Frame, double const Station, int const Direction, int const Side, std::optional<double> const &Bent)
{
	auto const map = [&](glm::dvec3 const &Point, glm::dvec2 &Tangent, glm::dvec2 const &Localtangent) {
		auto const bent{Bent.has_value() ? std::min(Point.z, *Bent) : Point.z};
		auto const sample{sample_at(Frame, Station + Direction * bent)};
		auto const along{sample.tangent * static_cast<double>(Direction)};
		glm::dvec2 const across{glm::dvec2{-along.y, along.x} * static_cast<double>(Side)};
		Tangent = glm::normalize(along * Localtangent.y + across * Localtangent.x);
		auto const planar{plan_of(sample.position) + along * (Point.z - bent) + across * Point.x};
		auto const level{Point.z > bent ? sample_at(Frame, Station + Direction * Point.z) : sample};
		auto const cant{glm::radians(static_cast<double>(level.roll) * Direction)};
		auto const cone{-Point.x * Side * std::tan(cant)};
		return glm::dvec3{planar.x, level.position.y + Point.y + cone, planar.y};
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
}

std::vector<segment_data> editor_mode::curved_switch_paths(editor_track::switch_template const &Shape, std::vector<curve_sample> const &Frame, double const Station, int const Direction, int const Side)
{
	// a switch with a straight frog bent up to the frog, beyond it going on along the curve as it heads there; the heights and the cant
	// those of the line where the point is along it
	auto const frog{frog_zone(Shape)};
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
		auto path{frame_path(local, Frame, Station, Direction, Side, frog)};
		// the tracks of a switch bent up to the frog in pieces: the bent one and the straight one over the frog
		auto const pieces{frog.has_value() && i < 2 ? split_along(local, *frog) : local.pieces};
		if (pieces.size() >= 2 || false == local.pieces.empty())
			for (auto const &piece : pieces)
				path.pieces.push_back(frame_path(piece, Frame, Station, Direction, Side, frog));
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
		tool.status = refused(format(STR_C("The curve is %.1f m long, too short for the %.1f m switch"), total, shape.length));
		return;
	}
	auto const station{std::clamp(tool.along, Direction > 0 ? 0.0 : shape.length, Direction > 0 ? total - shape.length : total)};
	auto const from{Direction > 0 ? station : station - shape.length};
	auto const to{from + shape.length};
	editor_track::edit edit;
	// a track of the scenery changed: its state before kept once for Ctrl+Z, none for one made here
	auto const keep = [&](TTrack &Track) {
		if (std::find(edit.created.begin(), edit.created.end(), &Track) == edit.created.end())
			edit.capture(Track);
	};
	auto const frogged{frog_zone(shape).has_value()};
	std::vector<TTrack *> candidates{tool.frame_tracks};
	// a cut this near a joint isn't made: no path of a few centimetres, the end of the track there goes onto the end of the switch instead
	constexpr double kJointSnap{0.05};
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
			auto const &own{candidate->m_paths.front()};
			auto const atjoint{std::min(plan_distance(own.points[segment_data::point::start], point), plan_distance(own.points[segment_data::point::end], point)) < kJointSnap};
			if (false == atjoint && t > 0.002 && t < 0.998)
			{
				keep(*candidate);
				if (auto *second{editor_track::split_path(*candidate, t)})
				{
					edit.created.push_back(second);
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
			edit.removed.push_back(candidate);
		}
	}
	auto const paths{curved_switch_paths(shape, tool.frame, station, Direction, Side)};
	// a joint a few centimetres off an end of the switch, not cut there: the end of the track left beside it goes right onto the end of
	// the switch, so the two join and no gap is left; the end of one bent up to the frog is fitted to it further on
	std::vector<TTrack *> snapped;
	for (auto const index : {segment_data::point::start, segment_data::point::end})
	{
		if (index == segment_data::point::end && frogged)
			continue;
		auto const target{paths.front().points[index]};
		for (auto *candidate : candidates)
		{
			if (candidate->m_editorremoved || candidate->eType != tt_Normal)
				continue;
			for (auto const kind : {editor_track::point_kind::start, editor_track::point_kind::end})
			{
				auto const off{plan_distance(candidate->m_paths.front().points[kind == editor_track::point_kind::start ? segment_data::point::start : segment_data::point::end], target)};
				if (off < 1e-4 || off > kJointSnap)
					continue;
				keep(*candidate);
				editor_track::move_point(*candidate, {0, kind}, target);
				if (std::find(snapped.begin(), snapped.end(), candidate) == snapped.end())
					snapped.push_back(candidate);
			}
		}
	}
	editor_track::commit(snapped);
	auto const report{curved_switch_report(paths, tool.frame, station, Direction)};
	// a switch bent up to the frog leaves the curve straight, a little off it at its end: the track going on from there along the line
	// starts where the switch ends, heading as it does, and meets the line again at its other end
	std::string fitted;
	if (frogged)
	{
		auto const endsample{sample_at(tool.frame, station + Direction * shape.length)};
		auto const ending{endsample.position};
		auto const &main{paths.front()};
		auto const &last{main.pieces.empty() ? main : main.pieces.back()};
		auto const switchend{main.points[segment_data::point::end]};
		auto const outward{-editor_track::end_direction(last, editor_track::point_kind::end)};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Normal || track->m_paths.empty() ||
			    std::find(edit.removed.begin(), edit.removed.end(), track) != edit.removed.end())
				continue;
			auto const &path{track->m_paths.front()};
			auto const atend{plan_distance(path.points[segment_data::point::end], ending) < 0.05};
			if (false == atend && plan_distance(path.points[segment_data::point::start], ending) >= 0.05)
				continue;
			keep(*track);
			auto const reshape = [&](segment_data &Path) {
				auto &point{Path.points[atend ? segment_data::point::end : segment_data::point::start]};
				auto &control{Path.points[atend ? segment_data::point::control2 : segment_data::point::control1]};
				auto const other{Path.points[atend ? segment_data::point::start : segment_data::point::end]};
				auto const handle{glm::length(plan_of(control)) > 1e-6 ? glm::length(plan_of(control)) : plan_distance(point, other) / 3.0};
				// into the track from the end touching the switch
				auto const into{outward * (atend ? -1.0 : 1.0) * handle};
				point = {switchend.x, point.y, switchend.z};
				control = {into.x, control.y, into.y};
			};
			auto &own{track->m_paths.front()};
			reshape(own);
			if (false == own.pieces.empty())
				reshape(atend ? own.pieces.back() : own.pieces.front());
			editor_track::commit({track});
			auto const turn{glm::degrees(std::acos(std::clamp(glm::dot(outward, glm::normalize(endsample.tangent * static_cast<double>(Direction))), -1.0, 1.0)))};
			fitted = "\n" + format(STR_C("Track %s going on from the switch fitted to its end, which leaves the curve straight over the frog: %.1f cm off it, turned by %.2f deg"), track->name().c_str(),
			                100.0 * plan_distance(switchend, ending), turn);
			break;
		}
	}
	auto const made{create_switch_set(shape, paths, *tool.frame_tracks.front(), switch_name_for_new())};
	edit.created.insert(edit.created.end(), made.begin(), made.end());
	if (made.size() == 1)
		tool.status = format(STR_C("Switch %s inserted into the curve, Ctrl+Z takes it back"), made.front()->name().c_str());
	// what the norms say of it stays in the status, after what the drive placed with it tells
	std::string norms;
	for (auto const &warning : report.warnings)
		norms += "\n" + warning;
	norms += fitted;
	tool.status += norms;
	auto const placed{edit.created};
	push_track_snapshot(std::move(edit.states), std::move(edit.created), std::move(edit.removed));
	straight_refresh();
	switch_placed(placed);
	if (false == norms.empty() && tool.status.find(norms) == std::string::npos)
		tool.status += norms;
}

std::vector<std::string> editor_mode::free_switch_exits(std::vector<segment_data> const &Paths) const
{
	auto const &tool{m_switch};
	std::vector<std::string> result;
	if (tool.free_end.track == nullptr || tool.free_end.track->eType != tt_Switch)
		return result;
	// the main and the diverging track of this one, each going on from that switch
	for (std::size_t i = 0; i < std::min<std::size_t>(2, Paths.size()); ++i)
		for (auto const &warning : switch_exit_warnings(tool.free_end.track, tool.free_end.position, nullptr, {}, {Paths[i]}))
			if (std::find(result.begin(), result.end(), warning) == result.end())
				result.push_back(warning);
	return result;
}

bool editor_mode::free_switch_bend(free_bend &Bend) const
{
	auto key{switch_memo_key()};
	if (false == m_bend_memo.fresh(key))
	{
		free_bend bend;
		auto const bent{free_switch_bend_now(bend)};
		m_bend_memo.keep(std::move(key), {bent, bend});
	}
	Bend = m_bend_memo.result.second;
	return m_bend_memo.result.first;
}

bool editor_mode::free_switch_bend_now(free_bend &Bend) const
{
	auto const &tool{m_switch};
	Bend = {};
	if (false == tool.free || tool.free_end.track == nullptr || false == tool.free_heading.has_value() || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return false;
	auto const &shape{tool.templates[tool.armed]};
	if (shape.twofold() || shape.double_slip || shape.radius < 1.0)
		return false;
	glm::dvec3 start;
	glm::dvec2 heading;
	int side;
	double grade;
	free_switch_pose(start, heading, side, grade);
	// a free end the cursor is at, other than the one the switch goes on from
	std::vector<TTrack const *> const exclude{tool.free_end.track};
	auto const &io{ImGui::GetIO()};
	if (false == m_cursor_override.has_value())
		Bend.target = snap_free_end(tool.mouse, {io.MousePos.x, io.MousePos.y}, nullptr, kRailCategory, exclude);
	if (Bend.target.track == nullptr)
		Bend.target = editor_track::find_free_end(nullptr, kRailCategory, tool.mouse, kLaySnapRadius, exclude);
	auto const toward{plan_of((Bend.target.track != nullptr ? Bend.target.position : tool.mouse) - start)};
	auto const along{glm::dot(toward, heading)};
	// near the switch the drag gives the side of its diverging track, as of a switch laid straight; past its end it bends it
	if (along < shape.length)
		return false;
	Bend.diverging = Bend.target.track != nullptr && io.KeyShift;
	// the curvature, positive to the left: of the circle going on along the free end through the cursor, or into the straight of the
	// free end it is at, one curve from the start of the switch
	auto const lateral{cross(heading, toward)};
	auto curvature{2.0 * lateral / (along * along + lateral * lateral)};
	if (Bend.target.track != nullptr && glm::length(plan_of(Bend.target.direction)) > 1e-9)
	{
		auto const into{glm::normalize(plan_of(Bend.target.direction))};
		auto const turn{geometry::signed_angle(heading, into)};
		auto const determinant{cross(heading, into)};
		if (std::abs(turn) <= glm::radians(0.3))
			curvature = 0.0;
		else if (std::abs(determinant) > 1e-9)
		{
			// how far ahead the two straights meet
			auto const ahead{cross(plan_of(Bend.target.position - start), into) / determinant};
			if (ahead > 0.0)
				curvature = std::copysign(std::tan(0.5 * std::abs(turn)) / ahead, turn);
		}
		// the diverging track into it: the switch bent so that its diverging track turns as much, the main track the other way where the
		// curve is gentler than the one of the switch, the same way where it's sharper
		if (Bend.diverging)
		{
			auto const way{curvature >= 0.0 ? 1 : -1};
			curvature = way * (std::abs(curvature) - 1.0 / shape.radius);
			Bend.side = way;
		}
	}
	// gentler than this it isn't bent, the drag giving the side as along the straight
	constexpr double kStraighter{12000.0};
	if (std::abs(curvature) < 1.0 / kStraighter)
	{
		if (Bend.target.track == nullptr)
			return false;
		curvature = 0.0;
	}
	else if (Bend.target.track == nullptr)
	{
		// to the radii a designer takes, the nearest one within 5 %, else to 10 m
		constexpr double kRadii[]{10000.0, 8000.0, 6000.0, 5000.0, 4000.0, 3500.0, 3000.0, 2500.0, 2000.0, 1800.0, 1500.0, 1400.0, 1200.0, 1100.0, 1000.0, 900.0, 800.0, 700.0, 600.0, 500.0, 400.0, 300.0};
		auto const radius{1.0 / std::abs(curvature)};
		auto chosen{std::round(radius / 10.0) * 10.0};
		auto closest{0.05};
		for (auto const typical : kRadii)
			if (auto const off{std::abs(radius / typical - 1.0)}; off < closest)
			{
				closest = off;
				chosen = typical;
			}
		curvature = std::copysign(1.0 / std::max(chosen, std::max(100.0, 0.5 * shape.radius)), curvature);
	}
	Bend.radius = curvature != 0.0 ? 1.0 / std::abs(curvature) : 0.0;
	Bend.turn = curvature >= 0.0 ? 1 : -1;
	if (false == Bend.diverging)
		Bend.side = curvature == 0.0 ? side : tool.one_sided ? Bend.turn : -Bend.turn;
	// the curve it's bent into, a little past its end
	glm::dvec2 const normal{-heading.y, heading.x};
	auto const reach{shape.length + 2.0};
	int const steps{std::max(8, static_cast<int>(std::ceil(reach / 0.5)))};
	for (int i = 0; i <= steps; ++i)
	{
		auto const station{reach * i / steps};
		auto const angle{curvature * station};
		auto const planar{curvature == 0.0 ? plan_of(start) + heading * station : plan_of(start) + (heading * std::sin(angle) + normal * (1.0 - std::cos(angle))) / curvature};
		curve_sample sample;
		sample.station = station;
		sample.position = {planar.x, start.y + grade * station, planar.y};
		sample.tangent = glm::normalize(heading * std::cos(angle) + normal * std::sin(angle));
		sample.radius = static_cast<float>(Bend.radius);
		Bend.frame.push_back(sample);
	}
	Bend.paths = curved_switch_paths(shape, Bend.frame, 0.0, 1, Bend.side);
	if (Bend.paths.size() < 2)
		return false;
	Bend.report = curved_switch_report(Bend.paths, Bend.frame, 0.0, 1);
	if (Bend.target.track != nullptr)
	{
		// the track into it goes on with the curve of the one it leaves, then straight, or with reverse curves
		auto const &branch{Bend.paths[Bend.diverging ? 1 : 0]};
		auto const radius{editor_track::branch_radius(branch)};
		auto speed{design_speed(tool.free_end.track, Bend.target.track)};
		if (Bend.diverging)
			speed = std::min(editor_track::diverging_speed(radius), speed);
		Bend.link = editor_track::link_branch(branch_start(branch), radius > 0.5 ? radius : kStraighter, branch_target(Bend.target), glm::radians(m_joints.kink_angle() * 0.5), editor_track::reverse_insert(speed), stretchable(Bend.target));
		Bend.problem = link_problem(Bend.target, Bend.link);
	}
	return true;
}

bool editor_mode::plan_curved_slip(editor_track::switch_template const &Shape, int const Direction, int const Side, curved_slip &Plan) const
{
	auto const &tool{m_switch};
	Plan = curved_slip{};
	// a crossing has no slip curves to bend, it goes only where two straights cross
	if (Shape.type == "Skrz")
	{
		Plan.problem = STR_C("A crossing goes only where two straights cross");
		return false;
	}
	if (tool.frame.size() < 2 || tool.frame_tracks.empty())
		return false;
	auto const *gauged{tool.frame_tracks.front()};
	auto const angle{std::atan(1.0 / (Shape.ratio > 0.0 ? Shape.ratio : 9.0))};
	auto const blade{switch_design::secant_blade(Shape.rail, Shape.radius, Shape.ratio)};
	auto curve{editor_track::slip_of(Shape.radius, Shape.middle, angle, 0.5 * std::abs(gauged->fTrackWidth) + gauged->rail_dimensions().head + 0.002, blade.first, blade.second)};
	curve.tip = switch_design::cut_tip(Shape.rail, curve.radius);
	auto const half{curve.tangent()};
	auto const total{tool.frame.back().station};
	if (2.0 * half > total - 0.02)
	{
		Plan.problem = format(STR_C("The curve is %.1f m long, too short for the %.1f m switch"), total, 2.0 * half);
		return false;
	}
	// the crossing where it was pressed, the ends of the slip drawn to the ends of the cant ramps as the ends of a switch are, the
	// whole of it on the curve
	auto const points{frame_points(tool.frame)};
	auto const start{switch_curve::snapped_start(switch_curve::ramp_ends(points), tool.pressed - Direction * half, 2.0 * half, Direction, 3.0, &Plan.snapped)};
	auto const middle{std::clamp(start + Direction * half, half, total - half)};
	if (std::abs(middle - (start + Direction * half)) > 1e-6)
		Plan.snapped.clear();
	// bent with a curve as sharp as its slip curves, the outer one would turn the other way
	double sharpest{0.0};
	for (auto s{middle - half}; s <= middle + half + 1e-9; s += 0.5)
		sharpest = std::max(sharpest, std::abs(switch_curve::curvature_at(points, s)));
	if (sharpest * curve.radius > 1.0 - 1e-6)
	{
		Plan.problem = format(STR_C("The curve of R %.0f m is as sharp as the slip curves of R %.0f m or sharper, the slip isn't bent into it"), 1.0 / sharpest, curve.radius);
		return false;
	}
	auto const sample{sample_at(tool.frame, middle)};
	Plan.middle = middle;
	Plan.half = half;
	// the straights of both tracks out to the joints of the template, as far as the curve goes on
	Plan.reach = std::max(half, std::min(Shape.a, std::min(middle, total - middle) - 0.02));
	Plan.direction = Direction;
	Plan.angle = angle;
	Plan.curve = curve;
	Plan.crossing = plan_of(sample.position);
	Plan.first = sample.tangent * static_cast<double>(Direction);
	Plan.second = turned(Plan.first, static_cast<double>(Side) * angle);
	auto const at = [](glm::dvec2 const &Planar) { return glm::dvec3{Planar.x, 0.0, Planar.y}; };
	Plan.points = {at(Plan.crossing - Plan.first * half), at(Plan.crossing + Plan.first * half), at(Plan.crossing - Plan.second * half), at(Plan.crossing + Plan.second * half)};
	return true;
}

segment_data editor_mode::bent_slip_path(curved_slip const &Plan, segment_data const &Path) const
{
	auto const &frame{m_switch.frame};
	// in the plan of a template: x across the tangent, to the left of the way the slip goes along the curve, z along it from the crossing
	glm::dvec2 const left{-Plan.first.y, Plan.first.x};
	auto const local = [&](segment_data const &Piece) {
		auto result{Piece};
		result.pieces.clear();
		for (auto const index : {segment_data::point::start, segment_data::point::end})
		{
			auto const offset{plan_of(Piece.points[index]) - Plan.crossing};
			result.points[index] = {glm::dot(offset, left), Piece.points[index].y, glm::dot(offset, Plan.first)};
		}
		for (auto const index : {segment_data::point::control1, segment_data::point::control2})
			if (auto const &handle{Piece.points[index]}; handle != glm::dvec3{})
				result.points[index] = {glm::dot(plan_of(handle), left), handle.y, glm::dot(plan_of(handle), Plan.first)};
		return result;
	};
	auto const bent = [&](segment_data const &Piece) {
		auto const flat{local(Piece)};
		auto result{frame_path(flat, frame, Plan.middle, Plan.direction, 1)};
		// the straights of the slip follow the curve they're bent with
		if (Piece.radius == 0.f)
			result.radius = sample_at(frame, Plan.middle + Plan.direction * 0.5 * (flat.points[segment_data::point::start].z + flat.points[segment_data::point::end].z)).radius;
		return result;
	};
	auto result{bent(Path)};
	for (auto const &piece : Path.pieces)
		result.pieces.push_back(bent(piece));
	return result;
}

std::vector<segment_data> editor_mode::curved_slip_lines(curved_slip const &Plan, std::string const &Type) const
{
	auto const &[a, b, c, d]{Plan.points};
	auto const side{cross(Plan.first, Plan.second) > 0.0 ? 1.0 : -1.0};
	auto const whole{Plan.curve.length()};
	std::vector<segment_data> lines{bent_slip_path(Plan, straight_path(a, b)), bent_slip_path(Plan, slip_arc(Plan.curve, a, Plan.first, side, d, 0.0, whole)), bent_slip_path(Plan, straight_path(c, d))};
	if (Type != "Rkp")
		lines.push_back(bent_slip_path(Plan, slip_arc(Plan.curve, c, Plan.second, -side, b, 0.0, whole)));
	return lines;
}

std::vector<segment_data> editor_mode::curved_slip_reaches(curved_slip const &Plan) const
{
	if (Plan.reach < Plan.half + 0.01)
		return {};
	auto const at = [](glm::dvec2 const &Planar) { return glm::dvec3{Planar.x, 0.0, Planar.y}; };
	auto const &[a, b, c, d]{Plan.points};
	return {bent_slip_path(Plan, straight_path(at(Plan.crossing - Plan.first * Plan.reach), a)), bent_slip_path(Plan, straight_path(b, at(Plan.crossing + Plan.first * Plan.reach))),
	        bent_slip_path(Plan, straight_path(at(Plan.crossing - Plan.second * Plan.reach), c)), bent_slip_path(Plan, straight_path(d, at(Plan.crossing + Plan.second * Plan.reach)))};
}

switch_curve::slip_report editor_mode::curved_slip_report(curved_slip const &Plan, std::string const &Type) const
{
	auto const lines{curved_slip_lines(Plan, Type)};
	std::vector<std::vector<switch_curve::track_point>> curves{plan_points(lines[1])};
	if (lines.size() > 3)
		curves.push_back(plan_points(lines[3]));
	return switch_curve::check_slip({plan_points(lines[0]), plan_points(lines[2])}, curves, frame_points(m_switch.frame), Plan.middle - Plan.direction * Plan.half, Plan.middle + Plan.direction * Plan.half);
}

std::vector<std::string> editor_mode::slip_report_lines(switch_curve::slip_report const &Report)
{
	// a radius counted positive the way the line turns, the other way with a minus
	auto const radius = [](double const Curvature) { return std::abs(Curvature) < 2e-5 ? std::string{STR_C("straight")} : format("%.0f m", 1.0 / Curvature); };
	std::vector<std::string> lines{format(STR_C("tracks through the slip R %s and %s"), radius(Report.first).c_str(), radius(Report.second).c_str())};
	std::string curves;
	for (auto const curvature : Report.curves)
		curves += (curves.empty() ? "" : ", ") + radius(curvature);
	if (false == curves.empty())
		lines.push_back(format(STR_C("slip curves R %s at their sharpest"), curves.c_str()));
	lines.push_back(format(STR_C("cant %.0f mm at the start, %.0f mm at the end"), Report.cant_start, Report.cant_end));
	return lines;
}

void editor_mode::insert_curved_slip(int const Direction, int const Side)
{
	auto &tool{m_switch};
	auto const &shape{tool.templates[tool.armed]};
	curved_slip plan;
	if (false == plan_curved_slip(shape, Direction, Side, plan))
	{
		tool.status = refused(plan.problem);
		return;
	}
	auto const report{curved_slip_report(plan, shape.type)};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	std::vector<TTrack *> candidates{tool.frame_tracks};
	auto &style{*tool.frame_tracks.front()};
	// the curve taken out up to the joints of the template, the straights of both tracks laid out there
	cut_curve(candidates, tool.frame, sample_at(tool.frame, plan.middle - plan.reach).position, sample_at(tool.frame, plan.middle + plan.reach).position, states, created, removed);
	auto const parts{build_double_slip(plan.points[0], plan.points[1], plan.points[2], plan.points[3], plan.crossing, 0.0, plan.first, plan.second, plan.angle, plan.curve, style, shape.type,
	                                   [&](segment_data const &Path) { return bent_slip_path(plan, Path); })};
	std::vector<TTrack *> made{parts};
	for (auto const &path : curved_slip_reaches(plan))
		if (auto *straight{editor_track::create_path(style, path)})
			made.push_back(straight);
	editor_track::commit(made);
	created.insert(created.end(), made.begin(), made.end());
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	switch_placed(parts);
	// after what the drives placed with it tell: its radii and what the norms say of it
	std::string text{STR_C("Slip bent into the curve, the ends of its crossing track free: lay on from them")};
	for (auto const &line : slip_report_lines(report))
		text += "\n" + line;
	for (auto const &warning : report.warnings)
		text += "\n" + warning;
	for (auto const &note : report.notes)
		text += "\n" + note;
	tool.status += (tool.status.empty() ? "" : "\n") + text;
	ui()->set_status(tool.status, false);
}
