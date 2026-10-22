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
#include "application/editorprojection.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"

#include "world/Track.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>

namespace
{

using geometry::plan_distance;

ImU32 const kGradeLine{IM_COL32(90, 230, 110, 255)};
ImU32 const kTangents{IM_COL32(255, 210, 60, 130)};
ImU32 const kTrack{IM_COL32(190, 190, 190, 255)};
ImU32 const kTerrain{IM_COL32(170, 120, 70, 255)};
ImU32 const kGrid{IM_COL32(255, 255, 255, 22)};
ImU32 const kLabel{IM_COL32(200, 200, 200, 200)};
ImU32 const kSwitch{IM_COL32(160, 100, 220, 45)};
ImU32 const kError{IM_COL32(255, 80, 70, 255)};
ImU32 const kWarning{IM_COL32(255, 200, 60, 255)};
ImU32 const kPlan{IM_COL32(120, 190, 255, 255)};
ImU32 const kSwitchLabel{IM_COL32(200, 160, 255, 220)};
ImU32 const kChip{IM_COL32(14, 15, 19, 210)};

// text on a dark plate, readable over the lines of the plot
void plot_label(ImDrawList &Draw, ImVec2 const At, ImU32 const Colour, std::string const &Text)
{
	auto const size{ImGui::CalcTextSize(Text.c_str())};
	Draw.AddRectFilled(ImVec2(At.x - 3.0f, At.y - 1.0f), ImVec2(At.x + size.x + 3.0f, At.y + size.y + 1.0f), kChip, 3.0f);
	Draw.AddText(At, Colour, Text.c_str());
}

double nice_step(double const Approximate)
{
	auto const power{std::pow(10.0, std::floor(std::log10(std::max(Approximate, 1e-6))))};
	for (auto const factor : {1.0, 2.0, 5.0, 10.0})
		if (power * factor >= Approximate)
			return power * factor;
	return power * 10.0;
}

double route_speed(editor_track::route const &Route)
{
	double speed{-1.0};
	for (auto const &span : Route.spans)
		speed = std::max(speed, editor_track::velocity(*span.track));
	return speed > 0.0 ? speed : 100.0;
}

std::string const kProfileMark{"//$p"};

profile::line reversed_line(profile::line Line, double const Length)
{
	std::reverse(Line.points.begin(), Line.points.end());
	for (auto &point : Line.points)
		point.chainage = Length - point.chainage;
	std::swap(Line.join_start, Line.join_end);
	return Line;
}

bool end_locked(profile::line const &Line, profile::context const &Context, int const Index)
{
	if (Index == 0)
		return Line.join_start && Context.start_joined;
	if (Index + 1 == static_cast<int>(Line.points.size()))
		return Line.join_end && Context.end_joined;
	return false;
}

bool has_errors(std::vector<profile::issue> const &Issues)
{
	return std::any_of(Issues.begin(), Issues.end(), [](profile::issue const &Issue) { return Issue.error; });
}

}

// format of the lines, coordinates in the space of the file:
//   profile <route length> <start x y z> <end x y z> <middle x y z>
//   speed <km/h> origin <chainage of the start> join <start 0/1> <end 0/1>
//   norms <grade max> <grade over switches max> <k of R >= k*V^2> <R min> <change needing a curve> <constant grade min> <fixed tolerance>
//   point <chainage> <elevation> <vertical curve R> <automatic R 0/1>
//   end
void editor_mode::profile_library_load()
{
	if (m_profiles_loaded)
		return;
	m_profiles_loaded = true;
	m_profiles.clear();
	for (auto const layer : scene::Layers.marked_layers(kProfileMark))
	{
		auto const &context{scene::Layers.layer(layer).context_insert()};
		auto const height = [&](double const Local) { return context.to_world({0.0, Local, 0.0}).y; };
		std::optional<stored_profile> current;
		for (auto const &text : scene::Layers.marked(layer, kProfileMark))
		{
			std::istringstream line{text};
			std::string keyword;
			line >> keyword;
			if (keyword == "profile")
			{
				stored_profile entry;
				entry.layer = layer;
				glm::dvec3 start, end, middle;
				line >> entry.length >> start.x >> start.y >> start.z >> end.x >> end.y >> end.z >> middle.x >> middle.y >> middle.z;
				if (line.fail())
				{
					current.reset();
					continue;
				}
				entry.start = context.to_world(start);
				entry.end = context.to_world(end);
				entry.middle = context.to_world(middle);
				current = entry;
				continue;
			}
			if (false == current.has_value())
				continue;
			auto &entry{*current};
			std::string word;
			if (keyword == "speed")
			{
				int start{1}, end{1};
				line >> entry.line.speed >> word >> entry.origin >> word >> start >> end;
				entry.line.join_start = (start != 0);
				entry.line.join_end = (end != 0);
			}
			else if (keyword == "norms")
			{
				auto &norms{entry.line.norms};
				line >> norms.grade_max >> norms.grade_switch_max >> norms.radius_factor >> norms.radius_min >> norms.curve_threshold >> norms.element_min >> norms.fixed_tolerance;
			}
			else if (keyword == "point")
			{
				profile::pvi point;
				int automatic{0};
				line >> point.chainage >> point.elevation >> point.radius >> automatic;
				if (line.fail())
					continue;
				point.elevation = height(point.elevation);
				point.automatic = (automatic != 0);
				entry.line.points.push_back(point);
			}
			else if (keyword == "end")
			{
				if (entry.line.points.size() >= 2 && entry.length > 0.0)
					m_profiles.push_back(entry);
				current.reset();
			}
		}
	}
	if (false == m_profiles.empty())
		WriteLog("Editor: " + std::to_string(m_profiles.size()) + " vertical profile design(s) read from the scenery", logtype::generic);
}

void editor_mode::profile_library_mark(std::vector<scene::layer_handle> Layers)
{
	std::sort(Layers.begin(), Layers.end());
	Layers.erase(std::unique(Layers.begin(), Layers.end()), Layers.end());
	for (auto const layer : Layers)
	{
		if (false == scene::Layers.valid(layer))
			continue;
		auto const &context{scene::Layers.layer(layer).context_insert()};
		auto const local = [&](glm::dvec3 const &Point) {
			auto const point{context.to_local(Point)};
			return format("%.4f %.4f %.4f", point.x, point.y, point.z);
		};
		std::vector<std::string> lines;
		for (auto const &entry : m_profiles)
		{
			if (entry.layer != layer)
				continue;
			if (lines.empty())
				lines.emplace_back("# vertical profiles (grade lines) designed in the track editor, the simulation skips these lines");
			lines.push_back(format("profile %.4f ", entry.length) + local(entry.start) + ' ' + local(entry.end) + ' ' + local(entry.middle));
			lines.push_back(format("speed %.1f origin %.4f join %d %d", entry.line.speed, entry.origin, entry.line.join_start ? 1 : 0, entry.line.join_end ? 1 : 0));
			auto const &norms{entry.line.norms};
			lines.push_back(format("norms %.4f %.4f %.4f %.1f %.4f %.2f %.4f", norms.grade_max, norms.grade_switch_max, norms.radius_factor, norms.radius_min, norms.curve_threshold, norms.element_min, norms.fixed_tolerance));
			for (auto const &point : entry.line.points)
				lines.push_back(format("point %.4f %.4f %.1f %d", point.chainage, context.to_local({0.0, point.elevation, 0.0}).y, point.radius, point.automatic ? 1 : 0));
			lines.emplace_back("end");
		}
		scene::Layers.mark(layer, kProfileMark, std::move(lines));
	}
}

int editor_mode::profile_find(bool &Reversed) const
{
	auto const &state{m_profile};
	Reversed = false;
	if (state.samples.empty())
		return -1;
	auto const start{state.samples.front().position};
	auto const end{state.samples.back().position};
	auto const middle{editor_track::sampled_position(state.samples, state.route.length * 0.5)};
	for (std::size_t i = 0; i < m_profiles.size(); ++i)
	{
		auto const &entry{m_profiles[i]};
		if (std::abs(entry.length - state.route.length) > 0.5 || plan_distance(entry.middle, middle) > 0.5)
			continue;
		if (plan_distance(entry.start, start) <= 0.1 && plan_distance(entry.end, end) <= 0.1)
			return static_cast<int>(i);
		if (plan_distance(entry.start, end) <= 0.1 && plan_distance(entry.end, start) <= 0.1)
		{
			Reversed = true;
			return static_cast<int>(i);
		}
	}
	return -1;
}

bool editor_mode::profile_restore()
{
	profile_library_load();
	auto &state{m_profile};
	bool reversed;
	auto const index{profile_find(reversed)};
	if (index < 0)
		return false;
	auto const &entry{m_profiles[index]};
	state.line = reversed ? reversed_line(entry.line, entry.length) : entry.line;
	state.origin = entry.origin;
	state.selected = -1;
	state.changed = false;
	profile_check();
	state.status = STR_C("Design read from the scenery file ") + scene::Layers.layer(entry.layer).name + format(STR_C(", largest departure from the track %.3f m"), state.departure);
	return true;
}

void editor_mode::profile_store()
{
	auto &state{m_profile};
	if (state.route.spans.empty() || state.samples.empty() || state.line.points.size() < 2)
		return;
	if (scene::Layers.empty())
	{
		state.error = STR_C("The scenery wasn't opened for editing, the design can't be kept in its files");
		return;
	}
	profile_library_load();
	stored_profile entry;
	entry.length = state.route.length;
	entry.start = state.samples.front().position;
	entry.end = state.samples.back().position;
	entry.middle = editor_track::sampled_position(state.samples, state.route.length * 0.5);
	entry.origin = state.origin;
	entry.line = state.line;
	auto &points{entry.line.points};
	points.erase(std::remove_if(points.begin(), points.end(), [](profile::pvi const &Point) { return Point.joint; }), points.end());

	bool reversed;
	auto const index{profile_find(reversed)};
	if (index >= 0)
	{
		entry.layer = m_profiles[index].layer;
		m_profiles[index] = entry;
	}
	else
	{
		auto layer{scene::Layers.resolve(state.route.spans.front().track->layer())};
		if (false == scene::Layers.valid(layer) || false == scene::Layers.accepts(layer))
			layer = scene::Layers.active();
		if (false == scene::Layers.valid(layer))
			layer = 1;
		entry.layer = layer;
		m_profiles.push_back(entry);
	}
	profile_library_mark({entry.layer});
	state.changed = false;
}

void editor_mode::profile_forget()
{
	auto &state{m_profile};
	profile_library_load();
	bool reversed;
	auto const index{profile_find(reversed)};
	if (index < 0)
		return;
	auto const layer{m_profiles[index].layer};
	m_profiles.erase(m_profiles.begin() + index);
	profile_library_mark({layer});
	state.status = STR_C("The design will be removed from ") + scene::Layers.layer(layer).name + STR_C(" on save");
}

void editor_mode::profile_open_stored(std::size_t const Index)
{
	auto &state{m_profile};
	if (Index >= m_profiles.size())
		return;
	auto const entry{m_profiles[Index]};
	auto const touching = [](glm::dvec3 const &Point) {
		std::vector<TTrack *> result;
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || false == editor_track::is_supported(*track))
				continue;
			for (auto const &path : track->m_paths)
				for (auto const index : {segment_data::point::start, segment_data::point::end})
					if (plan_distance(path.points[index], Point) <= 0.1 && std::abs(path.points[index].y - Point.y) < 3.0)
					{
						if (std::find(result.begin(), result.end(), track) == result.end())
							result.push_back(track);
					}
		}
		return result;
	};
	for (auto *from : touching(entry.start))
		for (auto *to : touching(entry.end))
		{
			editor_track::route route;
			std::string error;
			if (false == editor_track::find_route(from, to, route, error) || std::abs(route.length - entry.length) > 0.5)
				continue;
			auto const samples{editor_track::sample_route(route, 1.0)};
			if (samples.empty() || plan_distance(samples.front().position, entry.start) > 0.1 || plan_distance(editor_track::sampled_position(samples, route.length * 0.5), entry.middle) > 0.5)
				continue;
			profile_open(from, to);
			return;
		}
	state.open = true;
	state.error = STR_C("The route of the stored design isn't in the scenery any more (paths moved or removed)");
}

void editor_mode::profile_edited()
{
	m_profile.changed = true;
	profile_check();
}

void editor_mode::profile_open(TTrack *From, TTrack *To)
{
	auto &state{m_profile};
	state.open = true;
	state.status.clear();
	editor_track::route route;
	if (false == editor_track::find_route(From, To, route, state.error))
		return;
	profile_take(std::move(route), From, To);
}

void editor_mode::profile_open_run(TTrack &Track)
{
	auto &state{m_profile};
	state.open = true;
	state.status.clear();
	state.error.clear();
	auto route{editor_track::run_route(Track, state.run_length)};
	if (route.spans.empty())
	{
		state.error = STR_C("The path can't carry a profile");
		return;
	}
	auto *from{route.spans.front().track};
	auto *to{route.spans.back().track};
	profile_take(std::move(route), from, to);
}

void editor_mode::profile_take(editor_track::route Route, TTrack *From, TTrack *To)
{
	auto &state{m_profile};
	state.from = state.picked_from = From;
	state.to = state.picked_to = To;
	state.route = std::move(Route);
	state.selected = -1;
	profile_resample();
	if (false == profile_restore())
		profile_recognize();
	profile_fit_view();
}

void editor_mode::profile_resample()
{
	auto &state{m_profile};
	state.samples = editor_track::sample_route(state.route, 1.0);
	auto const &samples{state.samples};
	state.terrain.assign(samples.size(), std::numeric_limits<double>::quiet_NaN());
	std::vector<glm::dvec3> points;
	points.reserve(samples.size());
	for (auto const &sample : samples)
		points.push_back(sample.position);
	std::vector<char> found;
	auto const heights{ground_heights(points, true, &found)};
	for (std::size_t i = 0; i < samples.size(); ++i)
		if (found[i] != 0)
			state.terrain[i] = heights[i];

	auto &context{state.context};
	context = {};
	context.length = state.route.length;
	if (samples.empty())
		return;
	for (auto const &span : state.route.spans)
		if (span.track->eType != tt_Normal)
			context.switches.push_back({span.from, span.to, span.track->name()});
	context.start_elevation = samples.front().position.y;
	context.end_elevation = samples.back().position.y;
	context.start_joined = editor_track::adjoining_grade(state.route, false, context.start_grade);
	context.end_joined = editor_track::adjoining_grade(state.route, true, context.end_grade);
	profile_check();
}

void editor_mode::profile_recognize()
{
	auto &state{m_profile};
	std::vector<double> chainages, elevations, grades;
	for (auto const &sample : state.samples)
	{
		chainages.push_back(sample.chainage);
		elevations.push_back(sample.position.y);
		grades.push_back(sample.grade);
	}
	auto const norms{state.line.norms};
	state.line = profile::recognize(chainages, elevations, grades, {});
	state.line.norms = norms;
	state.line.speed = route_speed(state.route);
	state.selected = -1;
	profile_check();
	state.status = format(STR_C("Grade line recognized from the track: %zu points, largest departure %.3f m"), state.line.points.size(), state.departure);
}

void editor_mode::profile_fit_ground()
{
	auto &state{m_profile};
	std::vector<double> chainages;
	for (auto const &sample : state.samples)
		chainages.push_back(sample.chainage);
	if (std::none_of(state.terrain.begin(), state.terrain.end(), [](double const Height) { return std::isfinite(Height); }))
	{
		state.error = STR_C("There's no ground under the route");
		return;
	}
	auto settings{state.line};
	settings.speed = route_speed(state.route);
	auto line{profile::fit_ground(chainages, state.terrain, state.context, settings, state.fit)};
	if (line.points.size() < 2)
		return;
	state.line = std::move(line);
	state.selected = -1;
	state.error.clear();
	profile_check();
	double fill{0.0}, cut{0.0};
	for (std::size_t i = 0; i < state.samples.size(); ++i)
	{
		if (false == std::isfinite(state.terrain[i]))
			continue;
		auto const difference{profile::elevation(state.line, state.samples[i].chainage) - state.terrain[i]};
		fill = std::max(fill, difference);
		cut = std::max(cut, -difference);
	}
	state.status = format(STR_C("Grade line fitted to the ground: %zu points, embankment up to %.2f m, cutting up to %.2f m"), state.line.points.size(), fill, cut);
}

std::pair<double, double> editor_mode::profile_ballast(TTrack const &Track)
{
	auto const depth{std::max(0.0, static_cast<double>(Track.fTexHeight1 - Track.fTexHeightOffset))};
	auto const foot{0.5 * std::abs(Track.fTrackWidth) + std::abs(Track.fTexWidth) + std::abs(Track.fTexSlope)};
	return {depth, std::max(0.5, foot)};
}

std::pair<int, int> editor_mode::profile_generate_ground(std::vector<std::pair<double, double>> const &Beds, double const Reach)
{
	auto &state{m_profile};
	auto &works{state.earthworks};
	auto const &samples{state.samples};
	if (samples.empty() || false == ensure_terrain())
		return {0, 0};
	auto const size{heightmap::chunk_size};
	std::set<heightmap::chunk_key> keys;
	for (auto const &sample : samples)
	{
		auto const &p{sample.position};
		auto const low{heightmap::chunk_at(p.x - Reach, p.z - Reach)};
		auto const high{heightmap::chunk_at(p.x + Reach, p.z + Reach)};
		for (auto x = low.first; x <= high.first; ++x)
			for (auto z = low.second; z <= high.second; ++z)
				keys.insert({x, z});
	}
	auto const cells{heightmap::cells_for(m_terrain_spacing)};
	auto const cellsize{size / cells};
	auto const side{static_cast<std::size_t>(cells) + 1};
	int made{0}, skipped{0};
	for (auto const &key : keys)
	{
		if (m_streamer.exists(key))
			continue;
		auto const low{heightmap::chunk_corner(key)};
		glm::dvec2 const middle{low + glm::dvec2{size * 0.5}};
		std::vector<std::size_t> around;
		for (std::size_t i = 0; i < samples.size(); ++i)
			if (std::abs(samples[i].position.x - middle.x) <= size * 0.5 + Reach && std::abs(samples[i].position.z - middle.y) <= size * 0.5 + Reach)
				around.push_back(i);
		if (around.empty())
			continue;
		// where the scenery has its own ground, the chunk isn't made: the two would cover each other
		std::vector<glm::dvec3> probes;
		for (int iz = 1; iz < 10; ++iz)
			for (int ix = 1; ix < 10; ++ix)
				probes.emplace_back(low.x + ix * size / 10.0, profile::elevation(state.line, samples[around.front()].chainage), low.y + iz * size / 10.0);
		std::vector<char> found;
		ground_heights(probes, true, &found);
		if (std::count(found.begin(), found.end(), 1) * 20 > static_cast<long>(probes.size()))
		{
			++skipped;
			continue;
		}
		// the ground of the chunk: the ground under the nearest point of the route, or the formation there
		std::vector<float> heights(side * side);
		for (std::size_t iz = 0; iz < side; ++iz)
			for (std::size_t ix = 0; ix < side; ++ix)
			{
				glm::dvec3 const point{low.x + ix * cellsize, 0.0, low.y + iz * cellsize};
				auto best{around.front()};
				auto distance{std::numeric_limits<double>::max()};
				for (auto const i : around)
				{
					auto const d{plan_distance(point, samples[i].position)};
					if (d < distance)
					{
						distance = d;
						best = i;
					}
				}
				heights[iz * side + ix] = static_cast<float>(best < state.terrain.size() && std::isfinite(state.terrain[best]) ? state.terrain[best] : profile::elevation(state.line, samples[best].chainage) - Beds[best].first);
			}
		auto const added{m_streamer.add_chunk(key.first, key.second, m_terrain_spacing, [&](double const X, double const Z) {
			auto const ix{static_cast<std::size_t>(std::clamp<long>(std::lround((X - low.x) / cellsize), 0, cells))};
			auto const iz{static_cast<std::size_t>(std::clamp<long>(std::lround((Z - low.y) / cellsize), 0, cells))};
			return heights[iz * side + ix];
		})};
		if (false == added)
			continue;
		works.generated.push_back(key);
		++made;
	}
	return {made, skipped};
}

void editor_mode::profile_shape_ground()
{
	auto &state{m_profile};
	auto &works{state.earthworks};
	auto const &samples{state.samples};
	if (samples.size() < 2 || state.line.points.size() < 2)
		return;
	if (false == ensure_terrain())
	{
		state.error = STR_C("The heightmap terrain can't be opened");
		return;
	}
	auto const slope{std::max(0.1, works.slope)};
	std::vector<std::pair<double, double>> beds(samples.size(), {works.depth, works.half_width});
	if (works.from_ballast)
		for (std::size_t i = 0; i < samples.size(); ++i)
			if (samples[i].span < state.route.spans.size())
				if (auto const *track{state.route.spans[samples[i].span].track}; track != nullptr)
					beds[i] = profile_ballast(*track);
	double widest{0.0};
	for (auto const &bed : beds)
		widest = std::max(widest, bed.second);
	auto const reach{widest + works.reach};
	works.generated.clear();
	auto const generated{works.generate ? profile_generate_ground(beds, reach) : std::pair<int, int>{0, 0}};

	// the shaping is a modifier of the terrain: the formation along the axis of the route, followed by the ground
	// around it. shaped again, the route replaces its modifier, and the terrain follows the new grade line
	heightmap::modifier modifier;
	auto const name = [](TTrack const *Track) { return Track != nullptr && false == Track->name().empty() ? Track->name() : std::string{"?"}; };
	modifier.name = "route " + name(state.from) + " - " + name(state.to);
	modifier.slope = slope;
	modifier.reach = works.reach;
	modifier.rounding = std::max(0.0, works.rounding);
	std::vector<heightmap::corridor_point> points;
	points.reserve(samples.size());
	for (std::size_t i = 0; i < samples.size(); ++i)
		points.push_back({{samples[i].position.x, samples[i].position.z}, profile::elevation(state.line, samples[i].chainage) - beds[i].first, beds[i].second});
	// the points the line between their neighbours passes within a few millimetres of are left out
	double constexpr tolerance{0.005};
	modifier.points.push_back(points.front());
	std::size_t anchor{0};
	for (std::size_t next = 2; next < points.size(); ++next)
	{
		auto const &a{points[anchor]};
		auto const &b{points[next]};
		auto const run{b.position - a.position};
		auto const length{glm::length(run)};
		bool straight{true};
		for (auto i = anchor + 1; i < next && straight; ++i)
		{
			auto const t{length > 1e-9 ? glm::dot(points[i].position - a.position, run) / (length * length) : 0.0};
			auto const offset{glm::length(points[i].position - (a.position + run * t))};
			auto const formation{a.formation + (b.formation - a.formation) * t};
			auto const width{a.half_width + (b.half_width - a.half_width) * t};
			straight = offset <= tolerance && std::abs(points[i].formation - formation) <= tolerance && std::abs(points[i].half_width - width) <= tolerance;
		}
		if (false == straight)
		{
			anchor = next - 1;
			modifier.points.push_back(points[anchor]);
		}
	}
	modifier.points.push_back(points.back());

	double fill{0.0}, cut{0.0};
	for (std::size_t i = 0; i < samples.size(); ++i)
	{
		if (i >= state.terrain.size() || false == std::isfinite(state.terrain[i]))
			continue;
		fill = std::max(fill, points[i].formation - state.terrain[i]);
		cut = std::max(cut, state.terrain[i] - points[i].formation);
	}
	if (false == works.modifier.empty() && works.modifier != modifier.name)
		m_streamer.remove_modifier(works.modifier);
	works.modifier = modifier.name;
	auto const count{modifier.points.size()};
	m_streamer.modifier(std::move(modifier));
	forget_ground();

	auto status{format(STR_C("Terrain shaped by the modifier \"%s\" (%zu points): embankment up to %.2f m, cutting up to %.2f m"), works.modifier.c_str(), count, fill, cut)};
	if (generated.first > 0)
		status += format(STR_C("; %d chunk(s) of terrain made under the route"), generated.first);
	if (generated.second > 0)
		status += format(STR_C("; %d chunk(s) left out: the scenery has its own ground there, convert it to shape it"), generated.second);
	state.error.clear();
	profile_resample();
	state.status = status;
	WriteLog("Editor: vertical profile - " + state.status, logtype::generic);
}

void editor_mode::profile_restore_ground()
{
	auto &state{m_profile};
	auto &works{state.earthworks};
	std::size_t restored{0};
	if (false == works.modifier.empty() && m_streamer.remove_modifier(works.modifier))
		++restored;
	works.modifier.clear();
	for (auto const &key : works.generated)
	{
		m_streamer.remove_chunk(key.first, key.second);
		++restored;
	}
	works.generated.clear();
	forget_ground();
	profile_resample();
	state.status = restored > 0 ? STR_C("The terrain is back as it was before the shaping") : STR_C("The shaping of the terrain is gone already");
}

void editor_mode::render_ground_conversion()
{
	auto &conversion{m_ground_conversion};
	ImGui::PushID("ground_conversion");
	if (false == conversion.open)
	{
		if (ImGui::Button(STR_C("Convert the terrain files to heightmap terrain...")))
		{
			conversion.open = true;
			load_ground_files();
		}
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Replaces the triangles of the terrain files of the scenery (terrain directive, .txtf/.btf) with heightmap terrain\n"
			                              "of the same shape, painted with their materials; the files lose these triangles on save"));
		if (false == conversion.status.empty())
			ImGui::TextWrapped("%s", conversion.status.c_str());
		ImGui::PopID();
		return;
	}
	if (conversion.files.empty())
		ImGui::TextWrapped("%s", STR_C("The scenery has no terrain files: no terrain directive names a .txtf or .btf file"));
	else
		ImGui::TextUnformatted(STR_C("Terrain files of the scenery:"));
	for (std::size_t index = 0; index < conversion.files.size(); ++index)
	{
		auto &source{conversion.files[index]};
		ImGui::PushID(static_cast<int>(index));
		ImGui::BeginDisabled(false == source.good);
		ImGui::Checkbox(source.file.c_str(), &source.chosen);
		ImGui::EndDisabled();
		ImGui::SameLine();
		if (source.good)
			ImGui::TextDisabled(source.text ? STR_C("(text, %zu triangles)") : STR_C("(binary, %zu triangles)"), source.count);
		else
			ImGui::TextDisabled("(%s)", source.message.c_str());
		ImGui::PopID();
	}
	// the materials of the chosen files
	std::vector<std::size_t> counts(conversion.names.size(), 0);
	for (auto const &source : conversion.files)
		if (source.good && source.chosen)
			for (auto index = source.first; index < source.first + source.count; ++index)
				++counts[conversion.materials[index]];
	if (false == conversion.names.empty())
		ImGui::TextUnformatted(STR_C("Materials: tick the ones which are the ground"));
	for (std::size_t index = 0; index < conversion.names.size(); ++index)
	{
		if (counts[index] == 0)
			continue;
		bool chosen{conversion.chosen.count(index) != 0};
		ImGui::PushID(static_cast<int>(index) + 0x10000);
		if (ImGui::Checkbox(format(STR_C("%s  (%zu triangles)"), conversion.names[index].c_str(), counts[index]).c_str(), &chosen))
		{
			if (chosen)
				conversion.chosen.insert(index);
			else
				conversion.chosen.erase(index);
		}
		ImGui::PopID();
	}
	ImGui::TextWrapped("%s", STR_C("The chunks are painted with the materials of the triangles, up to 8 of them per chunk, added to the palette of the terrain. "
	                               "Until saved, loading the scenery again takes it all back."));
	auto const ready{std::any_of(conversion.files.begin(), conversion.files.end(), [](auto const &Source) { return Source.good && Source.chosen; }) && false == conversion.chosen.empty()};
	ImGui::BeginDisabled(false == ready);
	if (ImGui::Button(STR_C("Convert")))
	{
		conversion.status = convert_ground();
		conversion.open = false;
		profile_resample();
	}
	ImGui::EndDisabled();
	ImGui::SameLine();
	if (ImGui::Button(STR_C("Cancel")))
		conversion.open = false;
	if (false == conversion.open)
	{
		// the triangles take memory, they're read again the next time
		conversion.triangles = {};
		conversion.materials = {};
	}
	ImGui::PopID();
}

void editor_mode::render_profile_earthworks()
{
	auto &state{m_profile};
	auto &works{state.earthworks};
	if (false == ImGui::TreeNode(STR_C("Shape the terrain to the grade line")))
		return;
	ImGui::TextWrapped("%s", STR_C("Only the heightmap terrain changes: a formation under the track and slopes to the ground, kept as a modifier of the terrain which follows the grade line when it's shaped again; the triangles and models of the scenery stay as they are."));
	render_ground_conversion();
	auto const drag = [](char const *Label, double &Value, float const Speed, char const *Format) { return ImGui::DragScalar(STR_C(Label), ImGuiDataType_Double, &Value, Speed, nullptr, nullptr, Format); };
	ImGui::PushItemWidth(90.0f);
	ImGui::Checkbox(STR_C("Make terrain where the route has none"), &works.generate);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Chunks of heightmap terrain are made along the route where there's none, from the ground around,\nthen shaped. Where the scenery has its own ground triangles the chunk is left out: convert them instead"));
	ImGui::Checkbox(STR_C("Formation under the ballast of the track"), &works.from_ballast);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("The terrain goes as deep under the track as its ballast reaches and as wide as its foot,\nread from each path of the route; off: the values given below"));
	if (works.from_ballast)
	{
		if (false == state.route.spans.empty() && state.route.spans.front().track != nullptr)
		{
			auto const bed{profile_ballast(*state.route.spans.front().track)};
			ImGui::TextDisabled(STR_C("Ballast %.2f m deep, its foot %.2f m from the axis"), bed.first, bed.second);
		}
	}
	else
	{
		drag("Formation below the track (m)", works.depth, 0.01f, "%.2f");
		drag("Half width of the formation (m)", works.half_width, 0.05f, "%.2f");
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("From the axis of the route to the edge of the formation;\nfor two tracks add the distance between them"));
	}
	drag("Slopes 1 : n, n", works.slope, 0.05f, "%.2f");
	drag("Slopes reach at most (m)", works.reach, 1.0f, "%.0f");
	drag("Rounding of the edges (m)", works.rounding, 0.1f, "%.1f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Length over which the slope bends into the formation and into the ground; 0: sharp edges"));
	ImGui::PopItemWidth();
	works.rounding = std::clamp(works.rounding, 0.0, 20.0);
	works.depth = std::clamp(works.depth, 0.0, 5.0);
	works.half_width = std::clamp(works.half_width, 0.5, 30.0);
	works.slope = std::clamp(works.slope, 0.1, 10.0);
	works.reach = std::clamp(works.reach, 1.0, 200.0);
	if (has_errors(state.issues))
		ImGui::TextDisabled("%s", STR_C("Correct the errors of the grade line first"));
	else if (ImGui::Button(STR_C("Shape the terrain")))
		profile_shape_ground();
	if (false == works.modifier.empty() || false == works.generated.empty())
	{
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Restore the terrain")))
			profile_restore_ground();
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Removes the modifier of the route, and takes away the terrain the shaping made"));
	}
	if (m_streamer.active())
	{
		// the terrain tools take the tool options window over, the profile stays open under them
		if (ImGui::Button(STR_C("Sculpting...")))
			terrain_workspace(true);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Opens the terrain tools: raising, lowering and smoothing with a brush"));
	}
	ImGui::TreePop();
}

void editor_mode::profile_check()
{
	auto &state{m_profile};
	auto &points{state.line.points};
	// joint points come and go, the selection follows the point it was made on
	struct mark
	{
		int *index;
		double chainage;
		bool joint;
	};
	std::vector<mark> marks;
	for (auto *index : {&state.selected, &state.dragging, &state.curve_grip})
		if (*index >= 0 && *index < static_cast<int>(points.size()))
			marks.push_back({index, points[*index].chainage, points[*index].joint});
	profile::complete(state.line, state.context);
	for (auto const &mark : marks)
	{
		int best{-1};
		double distance{std::numeric_limits<double>::max()};
		for (int i = 0; i < static_cast<int>(points.size()); ++i)
			if (points[i].joint == mark.joint && std::abs(points[i].chainage - mark.chainage) < distance)
			{
				distance = std::abs(points[i].chainage - mark.chainage);
				best = i;
			}
		*mark.index = (mark.joint || distance < 1e-6) ? best : -1;
	}
	state.issues = profile::check(state.line, state.context);
	state.departure = 0.0;
	for (auto const &sample : state.samples)
		state.departure = std::max(state.departure, std::abs(profile::elevation(state.line, sample.chainage) - sample.position.y));
}

void editor_mode::profile_fit_view()
{
	auto &state{m_profile};
	auto const length{std::max(1.0, state.route.length)};
	state.view_from = -0.02 * length;
	state.view_to = length * 1.02;
	state.exaggeration = 0.0f;
}

void editor_mode::profile_apply()
{
	auto &state{m_profile};
	state.status.clear();
	state.error.clear();
	if (state.route.spans.empty() || state.line.points.size() < 2)
		return;
	if (has_errors(state.issues))
	{
		state.error = STR_C("Correct the errors to apply the grade line");
		return;
	}
	std::vector<TTrack *> tracks;
	for (auto const &span : state.route.spans)
		tracks.push_back(span.track);
	if (false == tracks_editable(tracks, state.error))
	{
		state.error = STR_C("The grade line can't be applied, ") + state.error;
		return;
	}
	auto const line{state.line};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	auto const gaps{editor_track::apply_profile(
	    state.route, [&](double const Chainage) { return profile::elevation(line, Chainage); }, [&](double const Chainage) { return profile::grade(line, Chainage); }, profile::breaks(line), states, created)};
	auto const changed{states.size()};
	auto const added{created.size()};
	// the stored design changes with the paths, an undo takes both back
	profile_library_load();
	auto library{m_profiles};
	bool const pushed{false == states.empty() || false == created.empty()};
	push_track_snapshot(std::move(states), std::move(created));
	if (pushed)
	{
		m_history.back().profiles = true;
		m_history.back().profile_library = std::move(library);
	}
	state.from = state.route.spans.front().track;
	state.to = state.route.spans.back().track;
	profile_resample();
	profile_store();
	state.status = format(STR_C("Grade line applied: %zu paths changed, %zu added, largest departure %.3f m"), changed, added, state.departure);
	if (false == scene::Layers.empty())
		state.status += "\nThe paths and the design (//$p lines) go to the scenery files on save";
	auto const name = [](TTrack const *Track) { return Track->name().empty() ? std::string{"(noname)"} : Track->name(); };
	auto const closed{std::count_if(gaps.begin(), gaps.end(), [](auto const &Gap) { return Gap.closed; })};
	if (closed > 0)
		state.status += "\nPaths off the route brought to the height of the switches:";
	for (auto const &gap : gaps)
		if (gap.closed)
			state.status += format("\n  %s at %s: %+.3f m", name(gap.neighbour).c_str(), name(gap.track).c_str(), -gap.gap);
	if (closed < static_cast<long>(gaps.size()))
		state.status += "\nSwitch ends off the adjoining paths, which can't be changed here:";
	for (auto const &gap : gaps)
		if (false == gap.closed)
			state.status += format("\n  %s -> %s: %+.3f m", name(gap.track).c_str(), name(gap.neighbour).c_str(), gap.gap);
	WriteLog("Editor: vertical profile - " + state.status, logtype::generic);
}

void editor_mode::profile_after_undo()
{
	auto &state{m_profile};
	if (state.route.spans.empty())
		return;
	auto const removed = [](TTrack const *Track) { return Track == nullptr || Track->m_editorremoved; };
	if (removed(state.from) || removed(state.to))
	{
		state.from = state.picked_from;
		state.to = state.picked_to;
	}
	std::string error;
	if (removed(state.from) || removed(state.to) || false == editor_track::find_route(state.from, state.to, state.route, error))
	{
		state.route = {};
		state.samples.clear();
		state.selected = state.dragging = state.curve_grip = -1;
		return;
	}
	profile_resample();
}

void editor_mode::render_profile_body()
{
	auto &state{m_profile};
	render_profile_source();
	if (state.route.spans.empty())
	{
		ImGui::TextWrapped("%s", STR_C("Select a path in the view, then build the route along the line, or set its start and end paths (switches are passed through)"));
		if (state.picked_from != nullptr)
			ImGui::Text(STR_C("Start: %s"), state.picked_from->name().c_str());
		if (false == state.error.empty())
			ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", state.error.c_str());
		return;
	}
	ImGui::Separator();
	ImGui::TextWrapped(STR_C("Route: %zu paths, %.2f m, %s ... %s"), state.route.spans.size(), state.route.length, state.route.spans.front().track->name().c_str(), state.route.spans.back().track->name().c_str());
	{
		bool reversed;
		auto const stored{profile_find(reversed)};
		if (stored >= 0)
		{
			ImGui::TextDisabled(STR_C("Design kept in %s%s"), scene::Layers.layer(m_profiles[stored].layer).name.c_str(), state.changed ? STR_C(", changes go there on save") : "");
			ImGui::SameLine();
			if (ImGui::SmallButton(STR_C("Forget the design")))
				profile_forget();
		}
		else if (state.changed)
			ImGui::TextDisabled(STR_C("The design goes to the scenery file of the route on save"));
	}

	if (render_profile_parameters())
		profile_edited();
}

void editor_mode::render_profile_strip()
{
	auto &state{m_profile};
	auto const &io{ImGui::GetIO()};
	ImGui::SetNextWindowPos(ImVec2(8.0f, io.DisplaySize.y - 360.0f), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(std::max(600.0f, io.DisplaySize.x - 500.0f), 320.0f), ImGuiCond_FirstUseEver);
	auto const title{std::string{STR_C("Vertical profile")} + format("  %.2f m###profilestrip", state.route.length)};
	if (ImGui::Begin(title.c_str(), nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
	{
		auto const side{std::clamp(ImGui::GetContentRegionAvail().x * 0.28f, 280.0f, 420.0f)};
		ImGui::BeginChild("##profileplot", ImVec2(-side, 0.0f), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
		render_profile_toolbar();
		render_profile_canvas(ImGui::GetContentRegionAvail().y);
		ImGui::EndChild();
		ImGui::SameLine();
		ImGui::BeginChild("##profileside", ImVec2(0.0f, 0.0f), false);
		if (ImGui::BeginTabBar("##profiletabs"))
		{
			// a click on a point in the plot brings its tab forward
			bool const picked{state.selected >= 0 && state.selected != state.shown};
			state.shown = state.selected;
			if (ImGui::BeginTabItem(STR_C("Point"), nullptr, picked ? ImGuiTabItemFlags_SetSelected : 0))
			{
				render_profile_point();
				ImGui::EndTabItem();
			}
			if (ImGui::BeginTabItem(format("%s (%zu)###profilepoints", STR_C("Points"), state.line.points.size()).c_str()))
			{
				ImGui::BeginChild("##profilepointlist");
				render_profile_points();
				ImGui::EndChild();
				ImGui::EndTabItem();
			}
			bool const errors{has_errors(state.issues)};
			if (false == state.issues.empty())
				ImGui::PushStyleColor(ImGuiCol_Text, errors ? ImVec4(1.0f, 0.45f, 0.4f, 1.0f) : ImVec4(1.0f, 0.8f, 0.3f, 1.0f));
			bool const remarks{ImGui::BeginTabItem(format("%s (%zu)###profileremarks", STR_C("Remarks"), state.issues.size()).c_str())};
			if (false == state.issues.empty())
				ImGui::PopStyleColor();
			if (remarks)
			{
				render_profile_issues();
				ImGui::EndTabItem();
			}
			ImGui::EndTabBar();
		}
		ImGui::EndChild();
	}
	ImGui::End();
}

void editor_mode::render_profile_source()
{
	auto &state{m_profile};
	auto *selected{selected_track()};
	if (ImGui::Button(STR_C("Along the line from the selected path")) && selected != nullptr)
		profile_open_run(*selected);
	ImGui::SameLine();
	ImGui::PushItemWidth(70.0f);
	ImGui::DragScalar(STR_C("m each way"), ImGuiDataType_Double, &state.run_length, 10.0f, nullptr, nullptr, "%.0f");
	ImGui::PopItemWidth();
	state.run_length = std::clamp(state.run_length, 10.0, 50000.0);
	if (ImGui::Button(STR_C("Start: selected path")) && selected != nullptr)
		state.picked_from = selected;
	ImGui::SameLine();
	if (ImGui::Button(STR_C("End: selected path")) && selected != nullptr && state.picked_from != nullptr)
		profile_open(state.picked_from, selected);
	profile_library_load();
	if (false == m_profiles.empty() && ImGui::TreeNode("##profilestored", STR_C("Designs kept in the scenery (%zu)"), m_profiles.size()))
	{
		for (std::size_t i = 0; i < m_profiles.size(); ++i)
		{
			auto const &entry{m_profiles[i]};
			ImGui::PushID(static_cast<int>(i));
			if (ImGui::SmallButton(STR_C("Open")))
				profile_open_stored(i);
			ImGui::SameLine();
			ImGui::Text(STR_C("%s  %.1f m, %zu points, from (%.1f, %.1f) to (%.1f, %.1f)"), scene::Layers.layer(entry.layer).name.c_str(), entry.length, entry.line.points.size(), entry.start.x, entry.start.z, entry.end.x, entry.end.z);
			ImGui::PopID();
		}
		ImGui::TreePop();
	}
}

bool editor_mode::render_profile_parameters()
{
	auto &state{m_profile};
	auto &line{state.line};
	bool changed{false};
	auto const drag = [](char const *Label, double &Value, float const Speed, char const *Format) { return ImGui::DragScalar(STR_C(Label), ImGuiDataType_Double, &Value, Speed, nullptr, nullptr, Format); };
	ImGui::PushItemWidth(120.0f);
	drag("Chainage of the start (m)", state.origin, 1.0f, "%.2f");
	changed |= drag("Design speed (km/h)", line.speed, 1.0f, "%.0f");
	ImGui::PopItemWidth();
	line.speed = std::max(1.0, line.speed);
	auto const join_tooltip = [] {
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Keeps the elevation of the joint and takes over the grade of the adjoining track,\nwith a vertical curve which starts right at the joint"));
	};
	if (state.context.start_joined)
	{
		changed |= ImGui::Checkbox(STR_C("Join the adjoining track at the start"), &line.join_start);
		join_tooltip();
	}
	if (state.context.end_joined)
	{
		changed |= ImGui::Checkbox(STR_C("Join the adjoining track at the end"), &line.join_end);
		join_tooltip();
	}
	if (ImGui::TreeNode(STR_C("Limits")))
	{
		auto &norms{line.norms};
		ImGui::PushItemWidth(90.0f);
		changed |= drag("Maximum grade (per mille)", norms.grade_max, 0.1f, "%.2f");
		changed |= drag("Maximum grade over switches (per mille)", norms.grade_switch_max, 0.1f, "%.2f");
		changed |= drag("Vertical curve R >= k*V^2, k", norms.radius_factor, 0.01f, "%.2f");
		changed |= drag("Vertical curve R min (m)", norms.radius_min, 10.0f, "%.0f");
		changed |= drag("Vertical curve needed above a change of (per mille)", norms.curve_threshold, 0.1f, "%.2f");
		changed |= drag("Constant grade between curves at least (m)", norms.element_min, 1.0f, "%.0f");
		changed |= drag("Tolerance at fixed points (m)", norms.fixed_tolerance, 0.001f, "%.3f");
		ImGui::PopItemWidth();
		if (ImGui::SmallButton(STR_C("Automatic R at all points")))
		{
			for (auto &point : line.points)
				point.automatic = true;
			changed = true;
		}
		ImGui::TextDisabled(STR_C("Required R for %.0f km/h: %.0f m"), line.speed, profile::required_radius(line));
		ImGui::TreePop();
	}

	ImGui::Separator();
	auto const errors{std::count_if(state.issues.begin(), state.issues.end(), [](profile::issue const &Issue) { return Issue.error; })};
	auto const warnings{static_cast<std::ptrdiff_t>(state.issues.size()) - errors};
	if (errors > 0)
		ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), STR_C("%d errors, %d warnings: see the Remarks in the strip"), static_cast<int>(errors), static_cast<int>(warnings));
	else if (warnings > 0)
		ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), STR_C("%d warnings: see the Remarks in the strip"), static_cast<int>(warnings));
	else
		ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.4f, 1.0f), "%s", STR_C("The grade line meets the limits"));
	ImGui::TextDisabled(STR_C("Largest departure from the track: %.3f m"), state.departure);
	if (ImGui::Button(STR_C("Recognize from the track")))
	{
		profile_recognize();
		state.changed = true;
	}
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Sets the points of the grade line from the existing track"));
	ImGui::SameLine();
	if (ImGui::Button(STR_C("Fit to the ground")))
	{
		profile_fit_ground();
		state.changed = true;
	}
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Places the breaks of the grade line along the ground under the route,\nwithin the limits and with the least earthworks; the ground itself stays as it is"));
	ImGui::PushItemWidth(70.0f);
	ImGui::DragScalar(STR_C("m off the ground##fittolerance"), ImGuiDataType_Double, &state.fit.tolerance, 0.05f, nullptr, nullptr, "%.2f");
	ImGui::PopItemWidth();
	state.fit.tolerance = std::clamp(state.fit.tolerance, 0.05, 50.0);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("How far the grade line may depart from the ground before it breaks;\nmore gives fewer breaks and higher embankments and deeper cuttings"));
	ImGui::SameLine();
	ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.55f, 0.25f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.68f, 0.3f, 1.0f));
	if (ImGui::Button(STR_C("Apply to the scenery"), ImVec2(-1.0f, 0.0f)))
		profile_apply();
	ImGui::PopStyleColor(2);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Sets the heights of the track along the route to the grade line, Ctrl+Z takes it back"));
	ImGui::Separator();
	render_profile_earthworks();
	return changed;
}

void editor_mode::render_profile_issues()
{
	auto &state{m_profile};
	ImGui::BeginChild("##profileissues", ImVec2(0.0f, 0.0f), false);
	if (state.issues.empty())
		ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.4f, 1.0f), "%s", STR_C("The grade line meets the limits"));
	else
		ImGui::TextDisabled("%s", STR_C("LMB on a remark shows its place"));
	for (std::size_t i = 0; i < state.issues.size(); ++i)
	{
		auto const &issue{state.issues[i]};
		ImGui::PushID(static_cast<int>(i));
		ImGui::PushStyleColor(ImGuiCol_Text, issue.error ? ImVec4(1.0f, 0.4f, 0.3f, 1.0f) : ImVec4(1.0f, 0.8f, 0.3f, 1.0f));
		auto const text{format("%s  km %s  %s", issue.error ? "x" : "!", profile::format_chainage(issue.chainage + state.origin).c_str(), issue.text.c_str())};
		if (ImGui::Selectable(text.c_str()))
		{
			if (issue.point >= 0)
				state.selected = issue.point;
			auto const half{(state.view_to - state.view_from) * 0.5};
			state.view_from = issue.chainage - half;
			state.view_to = issue.chainage + half;
		}
		ImGui::PopStyleColor();
		ImGui::PopID();
	}
	if (false == state.status.empty())
		ImGui::TextWrapped("%s", state.status.c_str());
	if (false == state.error.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", state.error.c_str());
	ImGui::EndChild();
}

void editor_mode::render_profile_point()
{
	auto &state{m_profile};
	auto &points{state.line.points};
	bool changed{false};
	auto const count{static_cast<int>(points.size())};
	if (state.selected >= count)
		state.selected = -1;
	auto const drag = [](char const *Label, double &Value, float const Speed, char const *Format) { return ImGui::DragScalar(STR_C(Label), ImGuiDataType_Double, &Value, Speed, nullptr, nullptr, Format); };
	auto const kind_of = [](double const Change) { return Change > 0.0 ? STR_C("sag") : STR_C("crest"); };
	if (state.selected < 0)
	{
		ImGui::TextWrapped("%s", STR_C("No point selected. Click a point of the grade line in the plot, or double click the line to add one."));
	}
	else if (points[state.selected].joint)
	{
		auto const index{state.selected};
		auto const &point{points[index]};
		auto const change{(profile::grade_after(state.line, index) - profile::grade_after(state.line, index - 1)) * 1000.0};
		ImGui::Text(STR_C("Point %d: joint with the adjoining track"), index + 1);
		ImGui::Text(STR_C("km %s   H %.3f m   R %.0f m"), profile::format_chainage(point.chainage + state.origin).c_str(), point.elevation, point.radius);
		ImGui::Text(STR_C("Grade %+.2f -> %+.2f per mille (%s)"), profile::grade_after(state.line, index - 1) * 1000.0, profile::grade_after(state.line, index) * 1000.0, kind_of(change));
		ImGui::TextWrapped("%s", STR_C("Placed automatically, moves with the neighbouring point; untick the joining in the inspector to edit the end freely"));
	}
	else
	{
		auto const index{state.selected};
		auto &point{points[index]};
		bool const end{index == 0 || index == count - 1};
		bool const locked{end_locked(state.line, state.context, index)};
		ImGui::Text(STR_C("Point %d"), index + 1);
		ImGui::SameLine();
		ImGui::TextDisabled(STR_C("km %s%s"), profile::format_chainage(point.chainage + state.origin).c_str(), locked ? STR_C(", joined to the adjoining track") : end ? STR_C(", end of the route") : "");
		ImGui::PushItemWidth(std::max(90.0f, ImGui::GetContentRegionAvail().x * 0.4f));
		auto chainage{point.chainage + state.origin};
		if (false == end && drag("Chainage (m)", chainage, 0.5f, "%.2f"))
		{
			point.chainage = std::clamp(chainage - state.origin, points[index - 1].chainage + 0.5, points[index + 1].chainage - 0.5);
			changed = true;
		}
		if (locked)
			ImGui::Text(STR_C("Elevation %.3f m"), point.elevation);
		else
			changed |= drag("Elevation (m)", point.elevation, 0.01f, "%.3f");
		if (index > 0 && false == locked)
		{
			auto grade{profile::grade_after(state.line, index - 1) * 1000.0};
			if (drag("Grade before (per mille)", grade, 0.05f, "%+.2f"))
			{
				point.elevation = points[index - 1].elevation + grade / 1000.0 * (point.chainage - points[index - 1].chainage);
				changed = true;
			}
		}
		if (index + 1 < count)
		{
			auto grade{profile::grade_after(state.line, index) * 1000.0};
			if (drag("Grade after (per mille)", grade, 0.05f, "%+.2f"))
			{
				points[index + 1].elevation = point.elevation + grade / 1000.0 * (points[index + 1].chainage - point.chainage);
				changed = true;
			}
		}
		if (false == end)
		{
			if (drag("Vertical curve R (m)", point.radius, 10.0f, "%.0f"))
			{
				point.automatic = false;
				changed = true;
			}
			point.radius = std::max(0.0, point.radius);
			ImGui::SameLine();
			changed |= ImGui::Checkbox(STR_C("auto"), &point.automatic);
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("R follows the limits: the required radius where the change of grade needs a curve; 0: no curve"));
			auto const change{(profile::grade_after(state.line, index) - profile::grade_after(state.line, index - 1)) * 1000.0};
			auto const tangent{point.radius * std::abs(change) / 2000.0};
			ImGui::TextDisabled(STR_C("Change of grade %+.2f per mille, %s"), change, kind_of(change));
			if (point.radius > 0.0)
				ImGui::TextDisabled(STR_C("Tangent T %.2f m, sagitta f %.3f m"), tangent, tangent * tangent / (2.0 * point.radius));
		}
		ImGui::PopItemWidth();
		if (index + 1 < count && ImGui::Button(STR_C("Add a point after")))
		{
			profile::pvi inserted;
			inserted.chainage = (point.chainage + points[index + 1].chainage) * 0.5;
			inserted.elevation = (point.elevation + points[index + 1].elevation) * 0.5;
			points.insert(points.begin() + index + 1, inserted);
			state.selected = index + 1;
			changed = true;
		}
		if (false == end)
		{
			ImGui::SameLine();
			if (ImGui::Button(STR_C("Remove  Del")))
			{
				points.erase(points.begin() + index);
				state.selected = -1;
				changed = true;
			}
		}
	}
	if (changed)
		profile_edited();
}

void editor_mode::render_profile_points()
{
	auto &state{m_profile};
	auto const &points{state.line.points};
	ImGui::Columns(4, "##profilepointcolumns", false);
	for (auto const *header : {"#", "km", "H (m)", "R (m)"})
	{
		ImGui::TextDisabled("%s", header);
		ImGui::NextColumn();
	}
	ImGui::Separator();
	for (int i = 0; i < static_cast<int>(points.size()); ++i)
	{
		auto const &point{points[i]};
		if (ImGui::Selectable(format("%d##point%d", i + 1, i).c_str(), state.selected == i, ImGuiSelectableFlags_SpanAllColumns))
		{
			state.selected = i;
			auto const half{(state.view_to - state.view_from) * 0.5};
			if (point.chainage < state.view_from || point.chainage > state.view_to)
			{
				state.view_from = point.chainage - half;
				state.view_to = point.chainage + half;
			}
		}
		ImGui::NextColumn();
		ImGui::TextUnformatted(profile::format_chainage(point.chainage + state.origin).c_str());
		ImGui::NextColumn();
		ImGui::Text("%.3f", point.elevation);
		ImGui::NextColumn();
		if (point.radius > 0.0)
			ImGui::Text("%.0f%s", point.radius, point.joint ? STR_C(" joint") : point.automatic ? STR_C(" auto") : "");
		else
			ImGui::TextDisabled("-");
		ImGui::NextColumn();
	}
	ImGui::Columns(1);
}

// screen mapping of the profile canvas, chainage across and elevation upwards
struct editor_mode::profile_view
{
	ImVec2 corner;
	ImVec2 size;
	float strip;
	float top;
	float height;
	double across;
	double vertical;
	double from;
	double centre;

	profile_view(profile_state const &State, ImVec2 const Corner, ImVec2 const Size) :
	    corner{Corner}, size{Size}, strip{State.show_plan ? 44.0f : 0.0f}, top{18.0f}, height{Size.y - strip - top},
	    across{std::max(1.0, State.view_to - State.view_from) / Size.x}, vertical{across / State.exaggeration}, from{State.view_from}, centre{State.view_centre}
	{
	}
	float x_of(double const Chainage) const { return corner.x + static_cast<float>((Chainage - from) / across); }
	float y_of(double const Height) const { return corner.y + top + height * 0.5f - static_cast<float>((Height - centre) / vertical); }
	double chainage_of(float const X) const { return from + (X - corner.x) * across; }
	double height_of(float const Y) const { return centre - (Y - corner.y - top - height * 0.5f) * vertical; }
	float plot_top() const { return corner.y + top; }
	float plot_bottom() const { return corner.y + top + height; }
	ImVec2 bottom_right() const { return {corner.x + size.x, corner.y + size.y}; }
	float mouse_distance2(double const Chainage, double const Height) const
	{
		auto const &mouse{ImGui::GetIO().MousePos};
		auto const dx{x_of(Chainage) - mouse.x};
		auto const dy{y_of(Height) - mouse.y};
		return dx * dx + dy * dy;
	}
};

void editor_mode::render_profile_toolbar()
{
	auto &state{m_profile};
	auto *draw{ImGui::GetWindowDrawList()};
	auto const swatch = [&](ImU32 const Colour) {
		auto const at{ImGui::GetCursorScreenPos()};
		auto const height{ImGui::GetFrameHeight()};
		draw->AddLine(ImVec2(at.x, at.y + height * 0.5f), ImVec2(at.x + 16.0f, at.y + height * 0.5f), Colour, 3.0f);
		ImGui::Dummy(ImVec2(16.0f, height));
		ImGui::SameLine(0.0f, 4.0f);
	};
	auto const legend = [&](ImU32 const Colour, char const *Label) {
		swatch(Colour);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted(STR_C(Label));
		ImGui::SameLine(0.0f, 14.0f);
	};
	auto const toggle = [&](ImU32 const Colour, char const *Label, bool &Value) {
		swatch(Colour);
		ImGui::Checkbox(STR_C(Label), &Value);
		ImGui::SameLine(0.0f, 14.0f);
	};
	legend(kGradeLine, "Grade line");
	legend(kTangents, "Tangents");
	toggle(kTrack, STR_C("Track"), state.show_track);
	toggle(kTerrain, STR_C("Terrain"), state.show_terrain);
	toggle(kPlan, STR_C("Plan curvature"), state.show_plan);
	legend(kSwitchLabel, "Switches");
	ImGui::SetNextItemWidth(130.0f);
	ImGui::SliderFloat("##exaggeration", &state.exaggeration, 1.0f, 500.0f, STR_C("heights x%.0f"), ImGuiSliderFlags_Logarithmic);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Exaggeration of the heights against the lengths, also Ctrl+wheel over the plot"));
	ImGui::SameLine();
	if (ImGui::Button(STR_C("Fit")))
		profile_fit_view();
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Shows the whole route"));
}

void editor_mode::render_profile_canvas(float const Height)
{
	auto &state{m_profile};
	auto const available{ImGui::GetContentRegionAvail()};
	ImVec2 const size{std::max(200.0f, available.x), std::max(100.0f, Height)};
	ImVec2 const corner{ImGui::GetCursorScreenPos()};
	ImGui::InvisibleButton("##profilecanvas", size);
	bool const hovered{ImGui::IsItemHovered()};

	if (state.exaggeration <= 0.0f)
		profile_fit_exaggeration(profile_view{state, corner, size});
	profile_navigate(profile_view{state, corner, size}, hovered);
	profile_view const view{state, corner, size};
	if (profile_canvas_edit(view, hovered))
		profile_edited();
	state.hover = hovered ? view.chainage_of(ImGui::GetIO().MousePos.x) : -1.0;

	auto *draw{ImGui::GetWindowDrawList()};
	draw->AddRectFilled(corner, view.bottom_right(), IM_COL32(22, 24, 28, 255));
	draw->PushClipRect(corner, view.bottom_right(), true);
	draw_profile_canvas(view, *draw);
	if (hovered && state.hover >= 0.0 && state.hover <= state.route.length && false == state.samples.empty())
	{
		auto const x{view.x_of(state.hover)};
		draw->AddLine(ImVec2(x, view.plot_top()), ImVec2(x, view.bottom_right().y), IM_COL32(255, 255, 255, 90));
		if (state.hover_point < 0 && state.hover_grip < 0 && state.curve_grip < 0)
			profile_canvas_tooltip();
	}

	// what the mouse does over the thing under it
	char const *hint{"wheel: zoom   Ctrl+wheel: heights   RMB/MMB drag: pan   double click on the line: add a point"};
	if (state.dragging >= 0)
		hint = "Ctrl: only the height   Shift: keep the grade before";
	else if (state.curve_grip >= 0 || state.hover_grip >= 0)
		hint = "drag up or down: radius of the vertical curve";
	else if (state.hover_point >= 0)
		hint = "drag: move the point   Ctrl+drag: only the height   Shift+drag: keep the grade before   Del: remove";
	else if (state.hover_line)
		hint = "double click: add a point here";
	if (hovered || state.dragging >= 0 || state.curve_grip >= 0)
	{
		auto const text{std::string{STR_C(hint)}};
		auto const size{ImGui::CalcTextSize(text.c_str())};
		plot_label(*draw, ImVec2(view.bottom_right().x - size.x - 8.0f, view.plot_bottom() - size.y - 6.0f), IM_COL32(255, 255, 255, 230), text);
	}
	draw->PopClipRect();

	auto const &points{state.line.points};
	if (state.dragging >= 0 && state.dragging < static_cast<int>(points.size()))
	{
		auto const index{state.dragging};
		ImGui::BeginTooltip();
		ImGui::Text(STR_C("Point %d"), index + 1);
		ImGui::Text(STR_C("km %s   H %.3f m"), profile::format_chainage(points[index].chainage + state.origin).c_str(), points[index].elevation);
		if (index > 0)
			ImGui::Text(STR_C("before %+.2f per mille"), profile::grade_after(state.line, index - 1) * 1000.0);
		if (index + 1 < static_cast<int>(points.size()))
			ImGui::Text(STR_C("after %+.2f per mille"), profile::grade_after(state.line, index) * 1000.0);
		ImGui::EndTooltip();
	}
	else if (state.curve_grip > 0 && state.curve_grip < static_cast<int>(points.size()))
	{
		ImGui::BeginTooltip();
		ImGui::Text("R %.0f m", points[state.curve_grip].radius);
		ImGui::EndTooltip();
	}
}

void editor_mode::profile_fit_exaggeration(profile_view const &View)
{
	auto &state{m_profile};
	double low{std::numeric_limits<double>::max()}, high{std::numeric_limits<double>::lowest()};
	for (auto const &sample : state.samples)
	{
		low = std::min(low, sample.position.y);
		high = std::max(high, sample.position.y);
	}
	for (auto const &point : state.line.points)
	{
		low = std::min(low, point.elevation);
		high = std::max(high, point.elevation);
	}
	if (low > high)
		low = high = 0.0;
	auto const range{std::max(1.0, (high - low) * 1.3)};
	state.exaggeration = static_cast<float>(std::clamp(View.across / (range / View.height), 1.0, 500.0));
	state.view_centre = (low + high) * 0.5;
}

void editor_mode::profile_navigate(profile_view const &View, bool const Hovered)
{
	auto &state{m_profile};
	ImGuiIO const &io{ImGui::GetIO()};
	if (Hovered && io.MouseWheel != 0.0f)
	{
		if (io.KeyCtrl)
			state.exaggeration = std::clamp(state.exaggeration * std::pow(1.2f, io.MouseWheel), 1.0f, 500.0f);
		else
		{
			auto const anchor{View.chainage_of(io.MousePos.x)};
			auto const factor{std::pow(0.8, static_cast<double>(io.MouseWheel))};
			state.view_from = anchor - (anchor - state.view_from) * factor;
			state.view_to = anchor + (state.view_to - anchor) * factor;
		}
	}
	if (Hovered && (ImGui::IsMouseClicked(1) || ImGui::IsMouseClicked(2)))
		state.panning = true;
	if (state.panning)
	{
		if (ImGui::IsMouseDown(1) || ImGui::IsMouseDown(2))
		{
			state.view_from -= io.MouseDelta.x * View.across;
			state.view_to -= io.MouseDelta.x * View.across;
			state.view_centre += io.MouseDelta.y * View.vertical;
		}
		else
			state.panning = false;
	}
}

bool editor_mode::profile_canvas_edit(profile_view const &View, bool const Hovered)
{
	auto &state{m_profile};
	auto &line{state.line};
	auto &points{line.points};
	ImGuiIO const &io{ImGui::GetIO()};
	ImVec2 const mouse{io.MousePos};

	auto const hit_point = [&]() {
		int hit{-1};
		float best{64.0f};
		for (int i = 0; i < static_cast<int>(points.size()); ++i)
		{
			if (points[i].joint)
				continue;
			if (auto const distance{View.mouse_distance2(points[i].chainage, points[i].elevation)}; distance < best)
			{
				best = distance;
				hit = i;
			}
		}
		return hit;
	};
	auto const hit_grip = [&]() {
		for (auto const &c : profile::curves(line))
		{
			if (points[c.point].joint)
				continue;
			auto const chainage{points[c.point].chainage};
			if (View.mouse_distance2(chainage, profile::elevation(line, chainage)) < 64.0f)
				return static_cast<int>(c.point);
		}
		return -1;
	};
	bool changed{false};
	bool const busy{state.dragging >= 0 || state.curve_grip >= 0 || state.panning};
	state.hover_point = Hovered && false == busy ? hit_point() : -1;
	state.hover_grip = Hovered && false == busy && state.hover_point < 0 ? hit_grip() : -1;
	state.hover_line = false;
	if (Hovered && false == busy && state.hover_point < 0 && state.hover_grip < 0 && points.size() >= 2)
	{
		auto const chainage{View.chainage_of(mouse.x)};
		state.hover_line = chainage > points.front().chainage && chainage < points.back().chainage && std::abs(View.y_of(profile::elevation(line, chainage)) - mouse.y) < 12.0f;
	}
	if (state.dragging >= 0 || state.panning)
		ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
	else if (state.curve_grip >= 0 || state.hover_grip >= 0)
		ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
	else if (state.hover_point >= 0 || state.hover_line)
		ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
	if (Hovered && ImGui::IsMouseClicked(0))
	{
		auto const point{hit_point()};
		auto const grip{point < 0 ? hit_grip() : -1};
		if (point >= 0)
			state.selected = state.dragging = point;
		else if (grip >= 0)
			state.selected = state.curve_grip = grip;
		else
			state.selected = -1;
	}
	if (Hovered && ImGui::IsMouseDoubleClicked(0) && state.dragging < 0 && state.curve_grip < 0 && points.size() >= 2)
	{
		auto const chainage{View.chainage_of(mouse.x)};
		if (chainage > points.front().chainage + 0.5 && chainage < points.back().chainage - 0.5 && std::abs(View.y_of(profile::elevation(line, chainage)) - mouse.y) < 12.0f)
		{
			std::size_t index{0};
			while (index + 2 < points.size() && points[index + 1].chainage < chainage)
				++index;
			profile::pvi inserted;
			inserted.chainage = std::round(chainage * 100.0) / 100.0;
			inserted.elevation = points[index].elevation + profile::grade_after(line, index) * (inserted.chainage - points[index].chainage);
			points.insert(points.begin() + index + 1, inserted);
			state.selected = static_cast<int>(index) + 1;
			changed = true;
		}
	}
	if (state.dragging >= 0 && state.dragging < static_cast<int>(points.size()))
	{
		if (ImGui::IsMouseDown(0))
		{
			auto const index{state.dragging};
			auto &point{points[index]};
			bool const end{index == 0 || index + 1 == static_cast<int>(points.size())};
			auto const incoming{index > 0 ? profile::grade_after(line, index - 1) : 0.0};
			if (false == end && false == io.KeyCtrl)
				point.chainage = std::clamp(std::round(View.chainage_of(mouse.x) * 100.0) / 100.0, points[index - 1].chainage + 0.5, points[index + 1].chainage - 0.5);
			if (end_locked(line, state.context, index))
				;
			else if (io.KeyShift && index > 0)
				point.elevation = points[index - 1].elevation + incoming * (point.chainage - points[index - 1].chainage);
			else
				point.elevation = std::round(View.height_of(mouse.y) * 1000.0) / 1000.0;
			changed = true;
		}
		else
			state.dragging = -1;
	}
	else
		state.dragging = -1;
	if (state.curve_grip > 0 && state.curve_grip + 1 < static_cast<int>(points.size()))
	{
		if (ImGui::IsMouseDown(0))
		{
			auto const sagitta{View.height_of(mouse.y) - points[state.curve_grip].elevation};
			points[state.curve_grip].radius = std::max(10.0, std::round(profile::radius_for_sagitta(line, state.curve_grip, sagitta) / 10.0) * 10.0);
			points[state.curve_grip].automatic = false;
			changed = true;
		}
		else
			state.curve_grip = -1;
	}
	else
		state.curve_grip = -1;
	if ((Hovered || ImGui::IsWindowFocused()) && state.selected > 0 && state.selected + 1 < static_cast<int>(points.size()) && false == points[state.selected].joint && ImGui::IsKeyPressed(ImGuiKey_Delete))
	{
		points.erase(points.begin() + state.selected);
		state.selected = -1;
		changed = true;
	}
	return changed;
}

void editor_mode::draw_profile_canvas(profile_view const &View, ImDrawList &Draw) const
{
	auto const &state{m_profile};
	auto const &line{state.line};
	auto const &points{line.points};
	auto const &samples{state.samples};
	auto const bottomright{View.bottom_right()};
	auto const plottop{View.plot_top()};
	auto const plotbottom{View.plot_bottom()};

	auto const step{nice_step(View.across * 110.0)};
	auto const firsts{std::ceil((state.view_from + state.origin) / step) * step - state.origin};
	for (int i = 0; firsts + i * step <= state.view_to; ++i)
	{
		auto const s{firsts + i * step};
		auto const x{View.x_of(s)};
		Draw.AddLine(ImVec2(x, plottop), ImVec2(x, plotbottom), kGrid);
		Draw.AddText(ImVec2(x + 3.0f, View.corner.y + 2.0f), kLabel, profile::format_chainage(s + state.origin).c_str());
	}
	auto const heightstep{nice_step(View.vertical * 45.0)};
	auto const firsth{std::floor(View.height_of(plotbottom) / heightstep) * heightstep};
	for (int i = 0; firsth + i * heightstep <= View.height_of(plottop); ++i)
	{
		auto const y{View.y_of(firsth + i * heightstep)};
		Draw.AddLine(ImVec2(View.corner.x, y), ImVec2(bottomright.x, y), kGrid);
	}
	for (auto const &zone : state.context.switches)
	{
		Draw.AddRectFilled(ImVec2(View.x_of(zone.from), plottop), ImVec2(std::max(View.x_of(zone.to), View.x_of(zone.from) + 2.0f), plotbottom), kSwitch);
		if (View.x_of(zone.to) - View.x_of(zone.from) > 20.0f || View.across < 0.5)
			Draw.AddText(ImVec2(View.x_of(zone.from) + 2.0f, plotbottom - 16.0f), kSwitchLabel, zone.name.c_str());
	}

	auto const polyline = [&](auto const &Height, ImU32 const Colour, float const Thickness) {
		ImVec2 previous{};
		bool open{false};
		float lastx{-1e9f};
		for (std::size_t i = 0; i < samples.size(); ++i)
		{
			auto const x{View.x_of(samples[i].chainage)};
			auto const h{Height(i)};
			if (std::isnan(h) || x < View.corner.x - 50.0f || x > bottomright.x + 50.0f)
			{
				open = false;
				continue;
			}
			if (open && x - lastx < 1.0f && i + 1 < samples.size())
				continue;
			ImVec2 const next{x, View.y_of(h)};
			if (open)
				Draw.AddLine(previous, next, Colour, Thickness);
			previous = next;
			lastx = x;
			open = true;
		}
	};
	if (state.show_terrain)
	{
		auto const fill{(kTerrain & 0x00FFFFFFu) | 0x30000000u};
		float lastx{-1e9f};
		ImVec2 previous{};
		bool open{false};
		for (std::size_t i = 0; i < samples.size(); ++i)
		{
			auto const x{View.x_of(samples[i].chainage)};
			auto const h{state.terrain[i]};
			if (std::isnan(h) || x < View.corner.x - 50.0f || x > bottomright.x + 50.0f)
			{
				open = false;
				continue;
			}
			if (open && x - lastx < 2.0f && i + 1 < samples.size())
				continue;
			ImVec2 const next{x, std::min(View.y_of(h), plotbottom)};
			if (open)
				Draw.AddQuadFilled(previous, next, ImVec2(next.x, plotbottom), ImVec2(previous.x, plotbottom), fill);
			previous = next;
			lastx = x;
			open = true;
		}
		polyline([&](std::size_t const I) { return state.terrain[I]; }, kTerrain, 1.5f);
	}
	if (state.show_track)
		polyline([&](std::size_t const I) { return samples[I].position.y; }, kTrack, 1.5f);

	for (std::size_t i = 0; i + 1 < points.size(); ++i)
		Draw.AddLine(ImVec2(View.x_of(points[i].chainage), View.y_of(points[i].elevation)), ImVec2(View.x_of(points[i + 1].chainage), View.y_of(points[i + 1].elevation)), kTangents, 1.0f);
	{
		auto const from{std::max(points.front().chainage, View.chainage_of(View.corner.x))};
		auto const to{std::min(points.back().chainage, View.chainage_of(bottomright.x))};
		ImVec2 previous{View.x_of(from), View.y_of(profile::elevation(line, from))};
		auto const firstx{previous.x};
		for (int i = 1; firstx + 2.0f * i <= View.x_of(to) + 2.0f; ++i)
		{
			auto const x{firstx + 2.0f * i};
			auto const chainage{std::min(to, View.chainage_of(x))};
			ImVec2 const next{View.x_of(chainage), View.y_of(profile::elevation(line, chainage))};
			Draw.AddLine(previous, next, kGradeLine, 2.5f);
			previous = next;
		}
	}
	for (std::size_t i = 0; i + 1 < points.size(); ++i)
	{
		auto const middle{(points[i].chainage + points[i + 1].chainage) * 0.5};
		auto const x{View.x_of(middle)};
		if (View.x_of(points[i + 1].chainage) - View.x_of(points[i].chainage) < 70.0f)
			continue;
		auto const text{format("%+.2f o/oo   L %.0f m", profile::grade_after(line, i) * 1000.0, points[i + 1].chainage - points[i].chainage)};
		auto const size{ImGui::CalcTextSize(text.c_str())};
		if (View.x_of(points[i + 1].chainage) - View.x_of(points[i].chainage) < size.x + 30.0f)
			continue;
		plot_label(Draw, ImVec2(x - size.x * 0.5f, View.y_of(profile::elevation(line, middle)) - size.y - 10.0f), kGradeLine, text);
	}
	for (auto const &c : profile::curves(line))
	{
		for (auto const at : {c.start, c.end})
		{
			auto const y{View.y_of(profile::elevation(line, at))};
			Draw.AddLine(ImVec2(View.x_of(at), y - 7.0f), ImVec2(View.x_of(at), y + 7.0f), kGradeLine, 1.5f);
		}
		auto const chainage{points[c.point].chainage};
		ImVec2 const grip{View.x_of(chainage), View.y_of(profile::elevation(line, chainage))};
		bool const lit{static_cast<int>(c.point) == state.curve_grip || static_cast<int>(c.point) == state.hover_grip};
		Draw.AddCircleFilled(grip, lit ? 6.5f : 4.5f, overlay_color::grip, 4);
		if (lit)
			Draw.AddCircle(grip, 10.0f, overlay_color::highlight, 12, 2.0f);
		plot_label(Draw, ImVec2(grip.x + 8.0f, grip.y + 6.0f), overlay_color::grip, format("R %.0f", points[c.point].radius));
	}
	for (int i = 0; i < static_cast<int>(points.size()); ++i)
	{
		ImVec2 const at{View.x_of(points[i].chainage), View.y_of(points[i].elevation)};
		bool const lit{i == state.hover_point || i == state.dragging};
		auto const radius{lit ? 7.0f : 5.5f};
		if (i == 0 || i + 1 == static_cast<int>(points.size()))
			Draw.AddRectFilled(ImVec2(at.x - radius, at.y - radius), ImVec2(at.x + radius, at.y + radius), IM_COL32(70, 140, 255, 255));
		else if (points[i].joint)
			Draw.AddQuadFilled(ImVec2(at.x, at.y - radius), ImVec2(at.x + radius, at.y), ImVec2(at.x, at.y + radius), ImVec2(at.x - radius, at.y), IM_COL32(90, 210, 230, 255));
		else
			Draw.AddCircleFilled(at, radius, overlay_color::marked);
		if (i == state.selected)
			Draw.AddCircle(at, 11.0f, overlay_color::highlight, 16, 2.5f);
		else if (lit)
			Draw.AddCircle(at, 11.0f, IM_COL32(255, 255, 255, 160), 16, 1.5f);
		// the heights only where they're asked for, the numbers of the points always
		auto const text{i == state.selected || lit ? format("%d   H %.3f", i + 1, points[i].elevation) : std::to_string(i + 1)};
		plot_label(Draw, ImVec2(at.x + 8.0f, at.y - 22.0f), overlay_color::marked, text);
	}
	// heights along the left edge, over everything else
	for (int i = 0; firsth + i * heightstep <= View.height_of(plottop); ++i)
		plot_label(Draw, ImVec2(View.corner.x + 4.0f, View.y_of(firsth + i * heightstep) - 8.0f), kLabel, format("%.1f", firsth + i * heightstep));
	for (auto const &issue : state.issues)
	{
		auto const x{View.x_of(issue.chainage)};
		Draw.AddTriangleFilled(ImVec2(x - 5.0f, plottop), ImVec2(x + 5.0f, plottop), ImVec2(x, plottop + 8.0f), issue.error ? kError : kWarning);
	}
	if (state.show_plan && false == samples.empty())
	{
		auto const middle{plotbottom + View.strip * 0.5f};
		Draw.AddLine(ImVec2(View.corner.x, plotbottom), ImVec2(bottomright.x, plotbottom), IM_COL32(255, 255, 255, 60));
		Draw.AddLine(ImVec2(View.corner.x, middle), ImVec2(bottomright.x, middle), IM_COL32(255, 255, 255, 40));
		ImVec2 previous{};
		bool open{false};
		float lastx{-1e9f};
		for (std::size_t i = 0; i < samples.size(); ++i)
		{
			auto const x{View.x_of(samples[i].chainage)};
			if (x < View.corner.x - 50.0f || x > bottomright.x + 50.0f || (open && x - lastx < 1.0f))
				continue;
			ImVec2 const next{x, middle - std::clamp(static_cast<float>(samples[i].curvature * 300.0 * View.strip * 0.4), -View.strip * 0.45f, View.strip * 0.45f)};
			if (open)
				Draw.AddLine(previous, next, IM_COL32(120, 190, 255, 255), 1.5f);
			previous = next;
			lastx = x;
			open = true;
		}
		Draw.AddText(ImVec2(View.corner.x + 4.0f, plotbottom + 1.0f), kLabel, STR_C("plan: curvature (up: left)"));
	}
}

void editor_mode::profile_canvas_tooltip() const
{
	auto const &state{m_profile};
	auto const &samples{state.samples};
	auto const designed{profile::elevation(state.line, state.hover)};
	auto const existing{editor_track::sampled_elevation(samples, state.hover)};
	ImGui::BeginTooltip();
	ImGui::Text(STR_C("km %s"), profile::format_chainage(state.hover + state.origin).c_str());
	ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kGradeLine), STR_C("Grade line %.3f m, %+.2f per mille"), designed, profile::grade(state.line, state.hover) * 1000.0);
	ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kTrack), STR_C("Track %.3f m (%+.3f), %+.2f per mille"), existing, existing - designed, editor_track::sampled_grade(samples, state.hover) * 1000.0);
	auto const next{std::lower_bound(samples.begin(), samples.end(), state.hover, [](editor_track::route_sample const &Sample, double const Value) { return Sample.chainage < Value; })};
	auto const index{static_cast<std::size_t>(std::distance(samples.begin(), std::min(next, std::prev(samples.end()))))};
	if (index < state.terrain.size() && false == std::isnan(state.terrain[index]))
		ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kTerrain), STR_C("Terrain %.3f m, %s %.2f m"), state.terrain[index], designed > state.terrain[index] ? STR_C("fill") : STR_C("cut"), std::abs(designed - state.terrain[index]));
	auto const &span{state.route.spans[samples[index].span]};
	ImGui::TextDisabled("%s%s", span.track->name().c_str(), span.track->eType != tt_Normal ? STR_C(" (switch, tilted as a whole with its branch)") : "");
	ImGui::EndTooltip();
}

void editor_mode::draw_profile_overlay() const
{
	auto const &state{m_profile};
	if (false == state.open || state.samples.size() < 2)
		return;
	screen_projection const projection;
	ImDrawList *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	auto const &samples{state.samples};
	glm::dvec3 previous{samples.front().position};
	glm::dvec3 designedprevious{previous.x, profile::elevation(state.line, 0.0), previous.z};
	for (std::size_t i = 1; i < samples.size(); ++i)
	{
		if (glm::distance(samples[i].position, previous) < 2.0 && i + 1 < samples.size())
			continue;
		auto const &next{samples[i].position};
		projection.line(drawlist, previous, next, IM_COL32(255, 150, 40, 200), 2.5f);
		glm::dvec3 const designed{next.x, profile::elevation(state.line, samples[i].chainage), next.z};
		if (std::abs(designed.y - next.y) > 0.01 || std::abs(designedprevious.y - previous.y) > 0.01)
			projection.line(drawlist, designedprevious, designed, kGradeLine, 2.0f);
		previous = next;
		designedprevious = designed;
	}
	auto const &points{state.line.points};
	for (int i = 0; i < static_cast<int>(points.size()); ++i)
	{
		auto position{editor_track::sampled_position(samples, points[i].chainage)};
		position.y = profile::elevation(state.line, points[i].chainage);
		ImVec2 screen;
		if (false == projection.project(position, screen))
			continue;
		drawlist->AddCircleFilled(screen, i == state.selected ? 7.0f : 5.0f, overlay_color::marked);
		drawlist->AddText(ImVec2(screen.x + 8.0f, screen.y - 16.0f), overlay_color::marked, format("%d", i + 1).c_str());
	}
	if (state.hover >= 0.0 && state.hover <= state.route.length)
	{
		ImVec2 screen;
		if (projection.project(editor_track::sampled_position(samples, state.hover), screen))
			drawlist->AddCircle(screen, 12.0f, IM_COL32(255, 255, 255, 230), 16, 2.5f);
	}
}
