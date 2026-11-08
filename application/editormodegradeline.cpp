/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the grade lines the tracks nearly parallel share: kept in the scenery, followed when the profile tool changes a route of them, a route
// taken off them and back, tracks clicked onto the line of the route and off it by hand

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editormodebuildtools.h"
#include "application/editorprojection.h"
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "simulation/simulation.h"
#include "utilities/translation.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <ranges>
#include <set>
#include <sstream>
#include <unordered_set>

namespace
{
std::string const kGradeLineMark{"//$w"};

// an end of a path by where it lies in the plan, to 5 cm: the ends meeting there have the same
using point_key = std::pair<int, int>;

point_key key_of(glm::dvec3 const &Point)
{
	return std::pair{static_cast<int>(std::round(Point.x * 20.0)), static_cast<int>(std::round(Point.z * 20.0))};
}

// what the route changed by abreast of the point, from the samples Before to the ones After; past the ends of the route the change fades
// from kFollow to kLeave beyond them
double change_abreast(std::vector<editor_track::route_sample> const &After, std::vector<editor_track::route_sample> const &Before, glm::dvec3 const &Point)
{
	double constexpr kFollow{0.0}; // m
	double constexpr kLeave{50.0}; // m
	auto closest{std::numeric_limits<double>::max()};
	auto chainage{0.0};
	auto past{0.0};
	for (std::size_t i = 0; i + 1 < After.size(); ++i)
	{
		auto const a{geometry::plan_of(After[i].position)};
		auto const run{geometry::plan_of(After[i + 1].position) - a};
		auto const along{glm::dot(geometry::plan_of(Point) - a, run) / std::max(1e-12, glm::dot(run, run))};
		auto const t{std::clamp(along, 0.0, 1.0)};
		if (auto const distance{glm::length(geometry::plan_of(Point) - (a + run * t))}; distance < closest)
		{
			closest = distance;
			chainage = After[i].chainage + (After[i + 1].chainage - After[i].chainage) * t;
			// how far along the way of the route it lies past its first or its last point
			past = (i == 0 && along < 0.0) || (i + 2 == After.size() && along > 1.0) ? std::abs(along - t) * glm::length(run) : 0.0;
		}
	}
	return std::clamp((kLeave - past) / (kLeave - kFollow), 0.0, 1.0) * (editor_track::sampled_elevation(After, chainage) - editor_track::sampled_elevation(Before, chainage));
}

// the ends of the paths of the route, those of the switches on it with them, and the tracks of it
void route_ends_of(editor_track::route const &Route, std::set<point_key> &Ends, std::unordered_set<TTrack const *> &Tracks)
{
	for (auto const &span : Route.spans)
	{
		Tracks.insert(span.track);
		for (auto const &path : span.track->m_paths)
			for (auto const end : {segment_data::point::start, segment_data::point::end})
				Ends.insert(key_of(path.points[end]));
	}
}

// the changes the ends of the path beside the route take, but the ends of the route and the ones which have one already; whether any of
// them is worth moving by
bool path_moves(segment_data const &Path, std::set<point_key> const &Ends, std::vector<editor_track::route_sample> const &After, std::vector<editor_track::route_sample> const &Before, std::map<point_key, double> &Moves)
{
	auto any{false};
	for (auto const end : {segment_data::point::start, segment_data::point::end})
	{
		auto const key{key_of(Path.points[end])};
		if (Ends.contains(key) || Moves.contains(key))
			continue;
		auto const move{change_abreast(After, Before, Path.points[end])};
		Moves[key] = move;
		any = any || std::abs(move) >= 0.001;
	}
	return any;
}

// the way along a path from one of its ends to the other, as long as the path in the plan
struct joined_end
{
	point_key to;
	double length;
};

// the ways along the paths of the tracks off the grade line, from each of their ends to the other: the route and the tracks on the line
// left out
std::map<point_key, std::vector<joined_end>> off_line_ways(std::unordered_set<TTrack const *> const &Route, std::unordered_set<TTrack const *> const &Members)
{
	std::map<point_key, std::vector<joined_end>> ways;
	for (auto const *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || Route.contains(track) || Members.contains(track))
			continue;
		for (auto const &path : track->m_paths)
		{
			auto const a{key_of(path.points[segment_data::point::start])};
			auto const b{key_of(path.points[segment_data::point::end])};
			auto const length{glm::length(geometry::plan_of(path.points[segment_data::point::end] - path.points[segment_data::point::start]))};
			ways[a].emplace_back(b, length);
			ways[b].emplace_back(a, length);
		}
	}
	return ways;
}

double constexpr kRampIn{60.0}; // m, 0.3 m in over it is 5 per mille

// a change taken in along the tracks off the grade line
struct ramp
{
	double from; // the change where it begins
	double gone; // m from there
};

double ramp_height(ramp const &Ramp)
{
	return Ramp.from * std::max(0.0, 1.0 - Ramp.gone / kRampIn);
}

// the tracks going on from the ends moved off the grade line take the change in over a length, however short each of them is: the ends
// along the ways from the moved ones get their changes too
void ramp_moves(std::map<point_key, double> &Moves, std::map<point_key, std::vector<joined_end>> const &Ways, std::set<point_key> const &Ends)
{
	std::map<point_key, ramp> ramps;
	std::vector<point_key> waiting;
	for (auto const &[key, move] : Moves)
	{
		ramps[key] = {move, 0.0};
		waiting.push_back(key);
	}
	while (false == waiting.empty())
	{
		auto const here{waiting.back()};
		waiting.pop_back();
		auto const ramp_here{ramps.at(here)};
		auto const found{Ways.find(here)};
		if (found == Ways.end())
			continue;
		for (auto const &way : found->second)
		{
			if (Ends.contains(way.to) || Moves.contains(way.to) || ramp_here.gone + way.length >= kRampIn)
				continue;
			ramp const further{ramp_here.from, ramp_here.gone + way.length};
			if (auto const known{ramps.find(way.to)}; known == ramps.end() || std::abs(ramp_height(further)) > std::abs(ramp_height(known->second)))
			{
				ramps[way.to] = further;
				waiting.push_back(way.to);
			}
		}
	}
	for (auto const &[key, ramped] : ramps)
		Moves[key] = ramp_height(ramped);
}

// whether the middle of a path, going along Way, lies on the curve and along it, On m off it at most
bool lies_along(geometry::bezier const &Theirs, glm::dvec3 const &Middle, glm::dvec2 const &Way, double const On)
{
	int constexpr kSteps{64};
	for (int k = 0; k < kSteps; ++k)
	{
		auto const a{Theirs.point(static_cast<double>(k) / kSteps)};
		auto const b{Theirs.point(static_cast<double>(k + 1) / kSteps)};
		auto const run{geometry::plan_of(b - a)};
		auto const square{glm::dot(run, run)};
		if (square < 1e-12)
			continue;
		auto const t{std::clamp(glm::dot(geometry::plan_of(Middle - a), run) / square, 0.0, 1.0)};
		if (glm::length(geometry::plan_of(Middle - a) - run * t) < On && std::abs(glm::dot(glm::normalize(run), glm::normalize(Way))) > 0.98)
			return true;
	}
	return false;
}

// the key of the track of a line, changed since the last frame, whose first path the first path of the track lies on; nullptr if none
std::string const *replaced_under(TTrack const &Track, std::vector<std::pair<std::string, segment_data>> const &Replaced)
{
	double constexpr kOn{0.3}; // m off the path it lay on
	geometry::bezier const mine{Track.m_paths.front()};
	auto const middle{mine.point(0.5)};
	auto const way{geometry::plan_of(mine.point(0.55) - mine.point(0.45))};
	if (glm::length(way) < 1e-9)
		return nullptr;
	for (auto const &[key, path] : Replaced)
	{
		geometry::bezier const theirs{path};
		if (auto const reach{theirs.plan_length()}; geometry::plan_distance(middle, theirs.point(0.5)) > 0.5 * reach + kOn)
			continue;
		if (lies_along(theirs, middle, way, kOn))
			return &key;
	}
	return nullptr;
}

// the moves of the ends of the paths of the tracks beside the route, by what it changed by abreast of them: true where one of them moves
bool beside_moves(std::vector<TTrack *> const &Beside, std::set<point_key> const &Ends, std::vector<editor_track::route_sample> const &After, std::vector<editor_track::route_sample> const &Before, std::map<point_key, double> &Moves)
{
	auto any{false};
	for (auto const *track : Beside)
		for (auto const &path : track->m_paths)
			if (path_moves(path, Ends, After, Before, Moves))
				any = true;
	return any;
}

// an end of a path of the track moves
bool track_moves(TTrack const &Track, std::map<point_key, double> const &Moves)
{
	return std::ranges::any_of(Track.m_paths, [&Moves](auto const &Path) { return Moves.contains(key_of(Path.points[segment_data::point::start])) || Moves.contains(key_of(Path.points[segment_data::point::end])); });
}

// whatever lies on common timbers with the route and the tracks beside it, on a grade line or not, goes with them: those stay on one
// surface
template <typename Places>
void add_on_timbers(Places const &All, std::unordered_set<TTrack const *> const &Route, std::vector<TTrack *> &Beside)
{
	std::unordered_set<TTrack const *> taken{Route};
	taken.insert(Beside.begin(), Beside.end());
	std::unordered_set<int> surfaces;
	for (auto const &place : All)
		if (taken.contains(place.track))
			surfaces.insert(place.surface);
	for (auto const &place : All)
		if (surfaces.contains(place.surface) && taken.insert(place.track).second && false == place.track->m_editorremoved)
			Beside.push_back(const_cast<TTrack *>(place.track)); // NOSONAR: the places point to the tracks of the simulation, none of which is a const object
}

// the button picking the tracks of the common grade line, or ending it
void render_pick_button(bool &Picking, bool const Shared)
{
	if (Picking)
	{
		ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.55f, 0.1f, 1.0f));
		if (ImGui::Button(STR_C("Click the tracks in the view, Esc: done"), ImVec2(-FLT_MIN, 0.0f)))
			Picking = false;
		ImGui::PopStyleColor();
	}
	else if (ImGui::Button(Shared ? STR_C("Put tracks on it or take them off...") : STR_C("Make a common grade line..."), ImVec2(-FLT_MIN, 0.0f)))
		Picking = true;
}

} // namespace

// the ends of the path up or down by these, what's between them in proportion; After is given the path and each of its pieces then
void grade_line_tool::move_path_heights(segment_data &Path, double const First, double const Second, std::function<void(segment_data &)> const &After)
{
	auto const origin{Path.points[segment_data::point::start]};
	auto const chord{geometry::plan_of(Path.points[segment_data::point::end] - origin)};
	auto const square{std::max(1e-9, glm::dot(chord, chord))};
	auto const by = [&](glm::dvec3 const &Point) { return glm::mix(First, Second, std::clamp(glm::dot(geometry::plan_of(Point - origin), chord) / square, 0.0, 1.0)); };
	auto const lay = [&](segment_data &Part) {
		auto &points{Part.points};
		auto const start{points[segment_data::point::start]};
		auto const end{points[segment_data::point::end]};
		if (points[segment_data::point::control1] != glm::dvec3{} || points[segment_data::point::control2] != glm::dvec3{})
		{
			points[segment_data::point::control1].y += by(start + points[segment_data::point::control1]) - by(start);
			points[segment_data::point::control2].y += by(end + points[segment_data::point::control2]) - by(end);
		}
		points[segment_data::point::start].y += by(start);
		points[segment_data::point::end].y += by(end);
		if (After)
			After(Part);
	};
	for (auto &piece : Path.pieces)
		lay(piece);
	lay(Path);
}

// a track on a grade line is told by its name and where it starts in the plan: scenery files have names used twice, and the height of a
// track on the line changes with it
std::string grade_line_tool::grade_line_key(TTrack const &Track)
{
	return grade_line_key(Track.name(), Track);
}

std::string grade_line_tool::grade_line_key(std::string const &Name, TTrack const &Track)
{
	if (Track.m_paths.empty())
		return Name;
	auto const &start{Track.m_paths.front().points[segment_data::point::start]};
	return format("%s@%lld,%lld", Name.c_str(), std::llround(start.x), std::llround(start.z));
}

// the grade lines shared go to the scenery files as "//$w" comment lines, by those keys
void grade_line_tool::grade_lines_load()
{
	auto &state{m_nmt_state};
	if (state.grade_lines_loaded)
		return;
	state.grade_lines_loaded = true;
	for (auto const layer : scene::Layers.marked_layers(kGradeLineMark))
		for (auto const &text : scene::Layers.marked(layer, kGradeLineMark))
		{
			std::istringstream line{text};
			std::string keyword;
			std::string name;
			std::string hand;
			int number{0};
			line >> keyword >> name >> number;
			if (keyword != "gradeline" || line.fail())
				continue;
			state.grade_lines[name] = number;
			if (line >> hand && hand == "hand")
				state.grade_lines_hand.insert(name);
		}
}

void grade_line_tool::grade_lines_store()
{
	if (scene::Layers.empty())
		return;
	std::map<scene::layer_handle, std::vector<std::string>> lines;
	for (auto const layer : scene::Layers.marked_layers(kGradeLineMark))
		lines[layer];
	for (auto const *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved)
			continue;
		auto const key{grade_line_key(*track)};
		auto const found{m_nmt_state.grade_lines.find(key)};
		if (found == m_nmt_state.grade_lines.end() || found->second == nmt_state::grade_line_own || found->second == nmt_state::grade_line_steep)
			continue;
		auto const hand{found->second >= 0 && m_nmt_state.grade_lines_hand.contains(key)};
		auto layer{scene::Layers.resolve(track->layer())};
		if (false == scene::Layers.valid(layer) || false == scene::Layers.accepts(layer))
			layer = scene::Layers.active();
		auto &those{lines[layer]};
		if (those.empty())
			those.emplace_back("# the tracks sharing a grade line, each with the number of it; the simulation skips these lines");
		those.push_back(format("gradeline %s %d%s", key.c_str(), found->second, hand ? " hand" : ""));
	}
	for (auto &[layer, those] : lines)
		if (scene::Layers.valid(layer))
			scene::Layers.mark(layer, kGradeLineMark, std::move(those));
}

bool grade_line_tool::on_route_grade_line(TTrack const &Track)
{
	grade_lines_load();
	auto const &lines{m_nmt_state.grade_lines};
	auto const found{lines.find(grade_line_key(Track))};
	if (found == lines.end() || found->second < 0)
		return false;
	return std::ranges::any_of(m_profile.route.spans, [&lines, &found](auto const &Span) {
		auto const own{lines.find(grade_line_key(*Span.track))};
		return own != lines.end() && own->second == found->second;
	});
}

std::vector<TTrack *> const &grade_line_tool::grade_line_beside()
{
	double constexpr kBeside{200.0}; // m from the route, the width of a station; farther it has nothing to do with this one
	auto &state{m_profile};
	std::tuple<TTrack const *, std::size_t, std::size_t, std::size_t> const of{state.route.spans.empty() ? nullptr : state.route.spans.front().track, state.route.spans.size(), static_cast<std::size_t>(m_track_edits), m_nmt_state.grade_lines_changes};
	if (of == state.beside_of)
		return state.beside;
	state.beside_of = of;
	state.beside.clear();
	grade_lines_load();
	std::unordered_set<int> numbers;
	std::unordered_set<TTrack const *> route;
	for (auto const &span : state.route.spans)
	{
		route.insert(span.track);
		if (auto const found{m_nmt_state.grade_lines.find(grade_line_key(*span.track))}; found != m_nmt_state.grade_lines.end() && found->second >= 0)
			numbers.insert(found->second);
	}
	if (numbers.empty() || state.samples.size() < 2)
		return state.beside;
	auto const lying_by = [&state](glm::dvec3 const &Point) {
		return std::ranges::any_of(state.samples, [&Point](auto const &Sample) { return glm::length(glm::dvec2{Sample.position.x - Point.x, Sample.position.z - Point.z}) < kBeside; });
	};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || route.contains(track) || track->m_paths.empty())
			continue;
		auto const found{m_nmt_state.grade_lines.find(grade_line_key(*track))};
		if (found != m_nmt_state.grade_lines.end() && numbers.contains(found->second) && (lying_by(track->m_paths.front().points[segment_data::point::start]) || lying_by(track->m_paths.front().points[segment_data::point::end])))
			state.beside.push_back(track);
	}
	// and whatever lies on common timbers with them, on a grade line or not: those stay on one surface
	add_on_timbers(m_nmt_state.surface_places, route, state.beside);
	return state.beside;
}

// the tracks beside the route on the grade line it shares go up and down with it, by what it changed by abreast of them, however far
// across the station: a surface on common timbers stays one. Past the ends of the route the change fades from kFollow to kLeave beyond
// them; a track joined to them from off the grade line takes the change at that end only
std::size_t grade_line_tool::grade_line_follow(std::vector<editor_track::route_sample> const &Before)
{
	auto const &state{m_profile};
	auto const &after{state.samples};
	if (false == state.whole_grade_line || Before.size() < 2 || after.size() < 2)
		return 0;
	auto const beside{grade_line_beside()};
	if (beside.empty())
		return 0;
	// the ends of the route itself lie where they should already, those of the switches on it with them
	std::set<point_key> route_ends;
	std::unordered_set<TTrack const *> route;
	route_ends_of(state.route, route_ends, route);
	std::map<point_key, double> moves;
	if (false == beside_moves(beside, route_ends, after, Before, moves))
		return 0;
	// the tracks going on from them off the grade line take the change in over a length, however short each of them is
	std::unordered_set<TTrack const *> const members(beside.begin(), beside.end());
	ramp_moves(moves, off_line_ways(route, members), route_ends);
	auto const move_of = [&moves](glm::dvec3 const &Point) {
		auto const found{moves.find(key_of(Point))};
		return found != moves.end() ? found->second : 0.0;
	};
	editor_track::edit edit;
	std::vector<TTrack *> moved;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || route.contains(track))
			continue;
		if (false == track_moves(*track, moves))
			continue;
		edit.capture(*track);
		moved.push_back(track);
		for (auto &path : track->m_paths)
			move_path_heights(path, move_of(path.points[segment_data::point::start]), move_of(path.points[segment_data::point::end]));
	}
	if (moved.empty())
		return 0;
	editor_track::commit(moved);
	m_services.push_track_snapshot(std::move(edit.states), {});
	m_build.join_last_history_steps();
	m_library.fitted_set(moved, true);
	return moved.size();
}

void grade_line_tool::grade_line_take_off(bool const Off)
{
	auto &lines{m_nmt_state.grade_lines};
	std::size_t changed{0};
	grade_lines_load();
	for (auto const &span : m_profile.route.spans)
	{
		auto const found{lines.find(grade_line_key(*span.track))};
		if (found == lines.end())
			continue;
		if (auto &line{found->second}; Off && line >= 0)
		{
			line = nmt_state::grade_line_off_number - line;
			m_nmt_state.grade_lines_hand.erase(found->first);
		}
		else if (false == Off && line <= nmt_state::grade_line_off_number)
			line = nmt_state::grade_line_off_number - line;
		else if (false == Off && line == nmt_state::grade_line_off)
			line = nmt_state::grade_line_own;
		else
			continue;
		++changed;
	}
	++m_nmt_state.grade_lines_changes;
	grade_lines_store();
	m_profile.status = format(Off ? STR_P("%zu track taken off the common grade line, it goes on its own now", "%zu tracks taken off the common grade line, they go on their own now", changed) : STR_P("%zu track back on the common grade line", "%zu tracks back on the common grade line", changed), changed);
}

int grade_line_tool::route_grade_line()
{
	grade_lines_load();
	std::map<int, double> lengths;
	for (auto const &span : m_profile.route.spans)
		if (auto const found{m_nmt_state.grade_lines.find(grade_line_key(*span.track))}; found != m_nmt_state.grade_lines.end() && found->second >= 0)
			lengths[found->second] += span.track->Length();
	auto best{-1};
	auto longest{-1.0};
	for (auto const &[number, length] : lengths)
		if (length > longest)
		{
			best = number;
			longest = length;
		}
	return best;
}

std::vector<TTrack *> grade_line_tool::grade_line_run(TTrack &Track, bool const Alone)
{
	std::vector run{&Track};
	if (Alone || Track.eType != tt_Normal)
		return run;
	std::unordered_set<TTrack const *> seen{&Track};
	for (auto const forward : {true, false})
	{
		TTrack const *previous{&Track};
		auto *current{forward ? Track.trNext : Track.trPrev};
		while (current != nullptr && current->eType == tt_Normal && false == current->m_editorremoved && seen.insert(current).second)
		{
			run.push_back(current);
			auto *next{current->trPrev == previous ? current->trNext : current->trPrev};
			previous = current;
			current = next;
		}
	}
	return run;
}

// put on by hand, or taken off, the tracks stay so when the lines are worked out again: a map of fiction has no terrain model to tell
// them by
void grade_line_tool::grade_line_pick(TTrack &Track, bool const Alone)
{
	auto &state{m_profile};
	auto &lines{m_nmt_state.grade_lines};
	auto &hand{m_nmt_state.grade_lines_hand};
	grade_lines_load();
	std::unordered_set<TTrack const *> route;
	for (auto const &span : state.route.spans)
		route.insert(span.track);
	if (route.contains(&Track))
	{
		state.status = STR("This is the route itself: the button above takes it off the common grade line");
		return;
	}
	auto const line_of = [&](TTrack const &Of) {
		auto const found{lines.find(grade_line_key(Of))};
		return found != lines.end() ? found->second : nmt_state::grade_line_own;
	};
	auto const number{route_grade_line()};
	auto const run{grade_line_run(Track, Alone)};
	if (number >= 0 && line_of(Track) == number)
	{
		std::size_t changed{0};
		for (auto const *track : run)
			if (line_of(*track) == number && false == route.contains(track))
			{
				auto const key{grade_line_key(*track)};
				lines[key] = nmt_state::grade_line_off_number - number;
				hand.erase(key);
				++changed;
			}
		state.status = format(STR_P("%zu track taken off the common grade line of the route", "%zu tracks taken off the common grade line of the route", changed), changed);
	}
	else
		grade_line_put_on(run, route, number);
	++m_nmt_state.grade_lines_changes;
	grade_lines_store();
}

void grade_line_tool::grade_line_put_on(std::vector<TTrack *> const &Run, std::unordered_set<TTrack const *> const &Route, int const Number)
{
	auto &lines{m_nmt_state.grade_lines};
	auto &hand{m_nmt_state.grade_lines_hand};
	auto number{Number};
	std::size_t changed{0};
	auto const made{number < 0};
	if (made)
	{
		// a new one, by a number no other line has, nor one a track taken off keeps
		number = 0;
		for (auto const line : std::views::values(lines))
		{
			if (line >= 0)
				number = std::max(number, line + 1);
			else if (line <= nmt_state::grade_line_off_number)
				number = std::max(number, nmt_state::grade_line_off_number - line + 1);
		}
		for (auto const *track : Route)
		{
			auto const key{grade_line_key(*track)};
			lines[key] = number;
			hand.insert(key);
		}
	}
	for (auto const *track : Run)
		if (false == Route.contains(track))
		{
			auto const key{grade_line_key(*track)};
			lines[key] = number;
			hand.insert(key);
			++changed;
		}
	m_profile.status = format(made ? STR_P("A common grade line made of the route and %zu track", "A common grade line made of the route and %zu tracks", changed)
	                               : STR_P("%zu track put on the common grade line of the route", "%zu tracks put on the common grade line of the route", changed),
	                          changed);
}

void grade_line_tool::grade_line_split(TTrack const &Original, TTrack const &Created)
{
	grade_lines_load();
	auto &lines{m_nmt_state.grade_lines};
	auto const found{lines.find(grade_line_key(Original))};
	if (found == lines.end() || Created.m_paths.empty())
		return;
	auto const line{found->second};
	auto const hand{m_nmt_state.grade_lines_hand.contains(found->first)};
	auto const key{grade_line_key(Created)};
	lines[key] = line;
	if (hand)
		m_nmt_state.grade_lines_hand.insert(key);
	++m_nmt_state.grade_lines_changes;
	m_nmt_state.grade_lines_dirty = true;
}

void grade_line_tool::grade_line_captured(TTrack const &Track)
{
	if (scene::Layers.empty())
		return;
	grade_lines_load();
	auto const key{grade_line_key(Track)};
	m_nmt_state.grade_line_keys.try_emplace(&Track, key);
	if (false == Track.m_paths.empty() && m_nmt_state.grade_lines.contains(key))
		m_nmt_state.grade_line_replaced.emplace_back(key, Track.m_paths.front());
}

// what the track had under its key before goes on under the one it has now; the key it had stays too, for an undo bringing it back. A
// track never changed before is new: one lying where a track of a line lay before the change takes its line, so the pieces of it laid
// anew and a switch put into it; one between two tracks of a line at the ends of its first path lies on it, put on by hand if both of
// them are, so the link of a crossover between two switches of it
void grade_line_tool::grade_lines_committed(std::vector<TTrack *> const &Tracks)
{
	auto &state{m_nmt_state};
	if (false == state.grade_lines_loaded)
		return;
	auto &lines{state.grade_lines};
	auto &hand{state.grade_lines_hand};
	auto changed{false};
	auto const carry = [&](std::string const &From, std::string const &To) {
		auto const found{lines.find(From)};
		if (From == To || found == lines.end() || lines.contains(To))
			return;
		auto const line{found->second};
		auto const byhand{hand.contains(From)};
		lines[To] = line;
		if (byhand)
			hand.insert(To);
		changed = true;
	};
	std::vector<TTrack *> fresh;
	for (auto *track : Tracks)
	{
		if (track == nullptr || track->m_editorremoved || track->m_paths.empty())
			continue;
		auto const key{grade_line_key(*track)};
		auto const known{state.grade_line_keys.find(track)};
		if (known == state.grade_line_keys.end())
		{
			if (false == lines.contains(key))
				fresh.push_back(track);
			continue;
		}
		carry(known->second, key);
		known->second = key;
	}
	if (grade_lines_take_replaced(fresh))
		changed = true;
	if (grade_lines_take_joined(fresh))
		changed = true;
	if (changed)
	{
		++state.grade_lines_changes;
		state.grade_lines_dirty = true;
	}
}

bool grade_line_tool::grade_lines_take_replaced(std::vector<TTrack *> &Fresh)
{
	auto &state{m_nmt_state};
	auto &lines{state.grade_lines};
	auto &hand{state.grade_lines_hand};
	auto changed{false};
	for (auto *&track : Fresh)
		if (auto const *was{replaced_under(*track, state.grade_line_replaced)}; was != nullptr && lines.contains(*was))
		{
			auto const line{lines.at(*was)};
			auto const byhand{hand.contains(*was)};
			auto const key{grade_line_key(*track)};
			lines[key] = line;
			if (byhand)
				hand.insert(key);
			state.grade_line_keys.try_emplace(track, key);
			track = nullptr;
			changed = true;
		}
	return changed;
}

std::map<int, bool> grade_line_tool::grade_lines_at(TTrack const &Track, glm::dvec3 const &Point) const
{
	auto const &lines{m_nmt_state.grade_lines};
	auto const &hand{m_nmt_state.grade_lines_hand};
	std::map<int, bool> found;
	for (auto const &[other, end] : editor_track::connected_points(Track, Point))
	{
		if (other == &Track || other->m_editorremoved)
			continue;
		if (auto const entry{lines.find(grade_line_key(*other))}; entry != lines.end() && entry->second >= 0)
			found[entry->second] = found[entry->second] || hand.contains(entry->first);
	}
	return found;
}

std::pair<int, bool> grade_line_tool::grade_line_between(TTrack const &Track) const
{
	auto const &path{Track.m_paths.front()};
	auto const start{grade_lines_at(Track, path.points[segment_data::point::start])};
	auto const end{grade_lines_at(Track, path.points[segment_data::point::end])};
	auto number{-1};
	auto byhand{false};
	for (auto const &[line, lefthand] : start)
	{
		auto const other{end.find(line)};
		if (other == end.end())
			continue;
		// two lines at both ends tell nothing
		number = number < 0 ? line : -2;
		byhand = lefthand && other->second;
	}
	return {number, byhand};
}

bool grade_line_tool::grade_lines_take_joined(std::vector<TTrack *> &Fresh)
{
	auto &state{m_nmt_state};
	auto changed{false};
	// the switches first, the link between two of them after
	for (int pass = 0; pass < 3; ++pass)
	{
		auto any{false};
		for (auto *&track : Fresh)
		{
			if (track == nullptr)
				continue;
			auto const [number, byhand]{grade_line_between(*track)};
			if (number < 0)
				continue;
			auto const key{grade_line_key(*track)};
			state.grade_lines[key] = number;
			if (byhand)
				state.grade_lines_hand.insert(key);
			state.grade_line_keys.try_emplace(track, key);
			track = nullptr;
			any = true;
			changed = true;
		}
		if (false == any)
			break;
	}
	return changed;
}

void grade_line_tool::grade_line_renamed(TTrack const &Track, std::string const &Old)
{
	if (Old.empty() || scene::Layers.empty())
		return;
	grade_lines_load();
	auto &state{m_nmt_state};
	auto const from{grade_line_key(Old, Track)};
	auto const to{grade_line_key(Track)};
	if (auto const found{state.grade_lines.find(from)}; found != state.grade_lines.end() && false == state.grade_lines.contains(to))
	{
		auto const line{found->second};
		auto const byhand{state.grade_lines_hand.contains(from)};
		state.grade_lines[to] = line;
		if (byhand)
			state.grade_lines_hand.insert(to);
		++state.grade_lines_changes;
		state.grade_lines_dirty = true;
	}
	if (auto const known{state.grade_line_keys.find(&Track)}; known != state.grade_line_keys.end())
		known->second = to;
}

std::vector<TTrack const *> const &grade_line_tool::grade_line_members(int const Number)
{
	auto &state{m_profile};
	std::tuple<int, std::size_t, std::size_t> const of{Number, m_nmt_state.grade_lines_changes, static_cast<std::size_t>(m_track_edits)};
	if (of == state.members_of)
		return state.members;
	state.members_of = of;
	state.members.clear();
	if (Number < 0)
		return state.members;
	grade_lines_load();
	for (auto const *track : simulation::Paths.sequence())
		if (track != nullptr && false == track->m_editorremoved)
			if (auto const found{m_nmt_state.grade_lines.find(grade_line_key(*track))}; found != m_nmt_state.grade_lines.end() && found->second == Number)
				state.members.push_back(track);
	return state.members;
}

// while they're picked: the tracks of the grade line of the route wherever they are, the run under the mouse in the colour of what a
// click does with it
void grade_line_tool::draw_grade_line_pick()
{
	if (auto const &state{m_profile}; false == state.open || false == state.picking || state.route.spans.empty())
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	auto const draw = [&](TTrack const &Track, ImU32 const Colour, float const Width) {
		for (auto const &path : Track.m_paths)
		{
			geometry::bezier const curve{path};
			auto const count{std::clamp(static_cast<int>(curve.plan_length() / 3.0), 2, 64)};
			auto previous{curve.point(0.0)};
			for (int k = 1; k <= count; ++k)
			{
				auto const next{curve.point(static_cast<double>(k) / count)};
				projection.line(drawlist, previous, next, Colour, Width);
				previous = next;
			}
		}
	};
	auto const number{route_grade_line()};
	for (auto const *track : grade_line_members(number))
		draw(*track, IM_COL32(70, 170, 255, 220), 3.0f);
	int index{0};
	auto *const aimed{m_build.grade_line_pick_aimed(index)};
	if (aimed == nullptr)
		return;
	auto const found{m_nmt_state.grade_lines.find(grade_line_key(*aimed))};
	auto const leaving{number >= 0 && found != m_nmt_state.grade_lines.end() && found->second == number};
	for (auto const *track : grade_line_run(*aimed, index == 1))
		draw(*track, leaving ? IM_COL32(255, 90, 70, 235) : IM_COL32(90, 230, 110, 235), 5.0f);
}

void grade_line_tool::render_profile_grade_line()
{
	auto &state{m_profile};
	grade_lines_load();
	auto const &lines{m_nmt_state.grade_lines};
	std::size_t shared{0};
	std::size_t off{0};
	for (auto const &span : state.route.spans)
		if (auto const found{lines.find(grade_line_key(*span.track))}; found != lines.end())
		{
			shared += found->second >= 0 ? 1 : 0;
			off += found->second == nmt_state::grade_line_off || found->second <= nmt_state::grade_line_off_number ? 1 : 0;
		}
	auto const number{route_grade_line()};
	if (shared > 0)
		ImGui::TextUnformatted(format(STR_C("Common grade line %d, %zu tracks on it"), number + 1, grade_line_members(number).size()).c_str());
	else
		ImGui::TextDisabled("%s", STR_C("The route shares no grade line with other tracks"));
	render_pick_button(state.picking, shared > 0);
	item_tooltip("Then click the tracks in the view: a click puts the stretch from switch to switch on the common grade line of the route, on a blue one takes it off; Shift+click the track only.\nWhat's set by hand stays when the lines are worked out again from the terrain model; there needn't be one");
	if (shared > 0)
	{
		auto const beside{grade_line_beside().size()};
		ImGui::Checkbox(STR_C("Move the whole common grade line"), &state.whole_grade_line);
		item_tooltip("Applied, the tracks and switches lying on one grade line with this route go up and down with it, the ones on common timbers staying on one surface.\nUnticked, only the route changes");
		ImGui::TextDisabled(STR_P("%zu track beside goes with the route (blue in the view)", "%zu tracks beside go with the route (blue in the view)", beside), beside);
		if (ImGui::SmallButton(STR_C("Take off the common grade line")))
			grade_line_take_off(true);
		item_tooltip("This route gets a grade line of its own: the tracks beside no longer go with it, and it doesn't go with them, also when the lines are worked out again");
	}
	if (off > 0)
	{
		if (ImGui::SmallButton(STR_C("Back on the common grade line")))
			grade_line_take_off(false);
		item_tooltip("The route lies on the common grade line again. Its height doesn't change by this; if it's off the tracks beside, fit it to them with the grade line or by working out the lines anew");
	}
}

// what the rest of the editor calls, kept on editor_mode: done by the build tools

std::string editor_mode::grade_line_key(TTrack const &Track)
{
	return grade_line_tool::grade_line_key(Track);
}

std::string editor_mode::grade_line_key(std::string const &Name, TTrack const &Track)
{
	return grade_line_tool::grade_line_key(Name, Track);
}

int editor_mode::route_grade_line()
{
	return m_build_tools->profiles.grade_lines.route_grade_line();
}

void editor_mode::grade_line_pick(TTrack &Track, bool const Alone)
{
	m_build_tools->profiles.grade_lines.grade_line_pick(Track, Alone);
}
