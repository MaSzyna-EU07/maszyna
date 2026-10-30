/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the grade lines the tracks nearly parallel share: kept in the scenery, followed when the profile tool changes a route of them, a route
// taken off them and back

#include "stdafx.h"
#include "application/editormode.h"
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
#include <set>
#include <sstream>
#include <unordered_set>

namespace
{
std::string const kGradeLineMark{"//$n"};
} // namespace

// the ends of the path up or down by these, what's between them in proportion; After is given the path and each of its pieces then
void editor_mode::move_path_heights(segment_data &Path, double const First, double const Second, std::function<void(segment_data &)> const &After)
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

// the grade lines shared go to the scenery files as "//$n" comment lines, by the names of the tracks
void editor_mode::grade_lines_load()
{
	auto &state{m_nmt_state};
	if (state.grade_lines_loaded)
		return;
	state.grade_lines_loaded = true;
	for (auto const layer : scene::Layers.marked_layers(kGradeLineMark))
		for (auto const &text : scene::Layers.marked(layer, kGradeLineMark))
		{
			std::istringstream line{text};
			std::string keyword, name;
			int number{0};
			line >> keyword >> name >> number;
			if (keyword == "gradeline" && false == line.fail())
				state.grade_lines[name] = number;
		}
}

void editor_mode::grade_lines_store()
{
	if (scene::Layers.empty())
		return;
	std::map<scene::layer_handle, std::vector<std::string>> lines;
	for (auto const layer : scene::Layers.marked_layers(kGradeLineMark))
		lines[layer];
	for (auto const &[name, number] : m_nmt_state.grade_lines)
	{
		auto const *track{simulation::Paths.find(name)};
		if (track == nullptr || track->m_editorremoved || number == nmt_state::grade_line_own || number == nmt_state::grade_line_steep)
			continue;
		auto layer{scene::Layers.resolve(track->layer())};
		if (false == scene::Layers.valid(layer) || false == scene::Layers.accepts(layer))
			layer = scene::Layers.active();
		auto &those{lines[layer]};
		if (those.empty())
			those.emplace_back("# the tracks sharing a grade line, each with the number of it; the simulation skips these lines");
		those.push_back(format("gradeline %s %d", name.c_str(), number));
	}
	for (auto &[layer, those] : lines)
		if (scene::Layers.valid(layer))
			scene::Layers.mark(layer, kGradeLineMark, std::move(those));
}

bool editor_mode::on_route_grade_line(TTrack const &Track)
{
	grade_lines_load();
	auto const &lines{m_nmt_state.grade_lines};
	auto const found{lines.find(Track.name())};
	if (found == lines.end() || found->second < 0)
		return false;
	return std::any_of(m_profile.route.spans.begin(), m_profile.route.spans.end(), [&](auto const &Span) {
		auto const own{lines.find(Span.track->name())};
		return own != lines.end() && own->second == found->second;
	});
}

std::vector<TTrack *> const &editor_mode::grade_line_beside()
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
		if (auto const found{m_nmt_state.grade_lines.find(span.track->name())}; found != m_nmt_state.grade_lines.end() && found->second >= 0)
			numbers.insert(found->second);
	}
	if (numbers.empty() || state.samples.size() < 2)
		return state.beside;
	auto const lying_by = [&](glm::dvec3 const &Point) {
		for (auto const &sample : state.samples)
			if (glm::length(glm::dvec2{sample.position.x - Point.x, sample.position.z - Point.z}) < kBeside)
				return true;
		return false;
	};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || route.contains(track) || track->m_paths.empty())
			continue;
		auto const found{m_nmt_state.grade_lines.find(track->name())};
		if (found != m_nmt_state.grade_lines.end() && numbers.contains(found->second) && (lying_by(track->m_paths.front().points[segment_data::point::start]) || lying_by(track->m_paths.front().points[segment_data::point::end])))
			state.beside.push_back(track);
	}
	// and whatever lies on common timbers with them, on a grade line or not: those stay on one surface
	std::unordered_set<TTrack const *> taken{route};
	taken.insert(state.beside.begin(), state.beside.end());
	std::unordered_set<int> surfaces;
	for (auto const &place : m_nmt_state.surface_places)
		if (taken.contains(place.track))
			surfaces.insert(place.surface);
	for (auto const &place : m_nmt_state.surface_places)
		if (surfaces.contains(place.surface) && taken.insert(place.track).second && false == place.track->m_editorremoved)
			state.beside.push_back(const_cast<TTrack *>(place.track));
	return state.beside;
}

// the tracks beside the route on the grade line it shares go up and down with it, by what it changed by abreast of them, however far
// across the station: a surface on common timbers stays one. Past the ends of the route the change fades from kFollow to kLeave beyond
// them; a track joined to them from off the grade line takes the change at that end only
std::size_t editor_mode::grade_line_follow(std::vector<editor_track::route_sample> const &Before)
{
	double constexpr kFollow{0.0}; // m
	double constexpr kLeave{50.0}; // m
	auto &state{m_profile};
	auto const after{state.samples};
	if (false == state.whole_grade_line || Before.size() < 2 || after.size() < 2)
		return 0;
	auto const beside{grade_line_beside()};
	if (beside.empty())
		return 0;
	auto const change_at = [&](glm::dvec3 const &Point) {
		auto closest{std::numeric_limits<double>::max()};
		auto chainage{0.0};
		auto past{0.0};
		for (std::size_t i = 0; i + 1 < after.size(); ++i)
		{
			auto const a{geometry::plan_of(after[i].position)};
			auto const run{geometry::plan_of(after[i + 1].position) - a};
			auto const along{glm::dot(geometry::plan_of(Point) - a, run) / std::max(1e-12, glm::dot(run, run))};
			auto const t{std::clamp(along, 0.0, 1.0)};
			if (auto const distance{glm::length(geometry::plan_of(Point) - (a + run * t))}; distance < closest)
			{
				closest = distance;
				chainage = after[i].chainage + (after[i + 1].chainage - after[i].chainage) * t;
				// how far along the way of the route it lies past its first or its last point
				past = (i == 0 && along < 0.0) || (i + 2 == after.size() && along > 1.0) ? std::abs(along - t) * glm::length(run) : 0.0;
			}
		}
		return std::clamp((kLeave - past) / (kLeave - kFollow), 0.0, 1.0) * (editor_track::sampled_elevation(after, chainage) - editor_track::sampled_elevation(Before, chainage));
	};
	auto const key_of = [](glm::dvec3 const &Point) { return std::pair{static_cast<int>(std::round(Point.x * 20.0)), static_cast<int>(std::round(Point.z * 20.0))}; };
	// the ends of the route itself lie where they should already, those of the switches on it with them
	std::set<std::pair<int, int>> route_ends;
	std::unordered_set<TTrack const *> route;
	for (auto const &span : state.route.spans)
	{
		route.insert(span.track);
		for (auto const &path : span.track->m_paths)
			for (auto const end : {segment_data::point::start, segment_data::point::end})
				route_ends.insert(key_of(path.points[end]));
	}
	std::map<std::pair<int, int>, double> moves;
	auto any{false};
	for (auto const *track : beside)
		for (auto const &path : track->m_paths)
			for (auto const end : {segment_data::point::start, segment_data::point::end})
				if (auto const key{key_of(path.points[end])}; false == route_ends.contains(key) && false == moves.contains(key))
				{
					auto const move{change_at(path.points[end])};
					moves[key] = move;
					any = any || std::abs(move) >= 0.001;
				}
	if (false == any)
		return 0;
	// the tracks going on from them off the grade line take the change in over a length, however short each of them is
	double constexpr kRampIn{60.0}; // m, 0.3 m in over it is 5 per mille
	std::unordered_set<TTrack const *> const members(beside.begin(), beside.end());
	struct joined
	{
		std::pair<int, int> to;
		double length;
	};
	std::map<std::pair<int, int>, std::vector<joined>> ways;
	for (auto const *track : simulation::Paths.sequence())
		if (track != nullptr && false == track->m_editorremoved && false == route.contains(track) && false == members.contains(track))
			for (auto const &path : track->m_paths)
			{
				auto const a{key_of(path.points[segment_data::point::start])};
				auto const b{key_of(path.points[segment_data::point::end])};
				auto const length{glm::length(geometry::plan_of(path.points[segment_data::point::end] - path.points[segment_data::point::start]))};
				ways[a].push_back({b, length});
				ways[b].push_back({a, length});
			}
	struct ramp
	{
		double from; // the change where it begins
		double gone; // m from there
	};
	std::map<std::pair<int, int>, ramp> ramps;
	std::vector<std::pair<int, int>> waiting;
	for (auto const &[key, move] : moves)
	{
		ramps[key] = {move, 0.0};
		waiting.push_back(key);
	}
	auto const height_of = [](ramp const &Ramp) { return Ramp.from * std::max(0.0, 1.0 - Ramp.gone / kRampIn); };
	while (false == waiting.empty())
	{
		auto const here{waiting.back()};
		waiting.pop_back();
		auto const ramp_here{ramps.at(here)};
		auto const found{ways.find(here)};
		if (found == ways.end())
			continue;
		for (auto const &way : found->second)
		{
			if (route_ends.contains(way.to) || moves.contains(way.to) || ramp_here.gone + way.length >= kRampIn)
				continue;
			ramp const further{ramp_here.from, ramp_here.gone + way.length};
			if (auto const known{ramps.find(way.to)}; known == ramps.end() || std::abs(height_of(further)) > std::abs(height_of(known->second)))
			{
				ramps[way.to] = further;
				waiting.push_back(way.to);
			}
		}
	}
	for (auto const &[key, ramped] : ramps)
		moves[key] = height_of(ramped);
	auto const move_of = [&](glm::dvec3 const &Point) {
		auto const found{moves.find(key_of(Point))};
		return found != moves.end() ? found->second : 0.0;
	};
	editor_track::edit edit;
	std::vector<TTrack *> moved;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || route.contains(track))
			continue;
		auto const touched{std::any_of(track->m_paths.begin(), track->m_paths.end(), [&](auto const &Path) { return moves.contains(key_of(Path.points[segment_data::point::start])) || moves.contains(key_of(Path.points[segment_data::point::end])); })};
		if (false == touched)
			continue;
		edit.capture(*track);
		moved.push_back(track);
		for (auto &path : track->m_paths)
			move_path_heights(path, move_of(path.points[segment_data::point::start]), move_of(path.points[segment_data::point::end]));
	}
	if (moved.empty())
		return 0;
	editor_track::commit(moved);
	push_track_snapshot(std::move(edit.states), {});
	join_last_steps();
	fitted_set(moved);
	return moved.size();
}

void editor_mode::grade_line_take_off(bool const Off)
{
	auto &lines{m_nmt_state.grade_lines};
	std::size_t changed{0};
	grade_lines_load();
	for (auto const &span : m_profile.route.spans)
	{
		auto const found{lines.find(span.track->name())};
		if (found == lines.end())
			continue;
		auto &line{found->second};
		if (Off && line >= 0)
			line = nmt_state::grade_line_off_number - line;
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

void editor_mode::render_profile_grade_line()
{
	auto &state{m_profile};
	grade_lines_load();
	auto const &lines{m_nmt_state.grade_lines};
	std::size_t shared{0}, off{0};
	for (auto const &span : state.route.spans)
		if (auto const found{lines.find(span.track->name())}; found != lines.end())
		{
			shared += found->second >= 0 ? 1 : 0;
			off += found->second == nmt_state::grade_line_off || found->second <= nmt_state::grade_line_off_number ? 1 : 0;
		}
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
