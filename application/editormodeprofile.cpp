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

#include "imgui/imgui.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
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
	state.status = "Design read from the scenery file " + scene::Layers.layer(entry.layer).name + format(", largest departure from the track %.3f m", state.departure);
	return true;
}

void editor_mode::profile_store()
{
	auto &state{m_profile};
	if (state.route.spans.empty() || state.samples.empty() || state.line.points.size() < 2)
		return;
	if (scene::Layers.empty())
	{
		state.error = "The scenery wasn't opened for editing, the design can't be kept in its files";
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
	state.status = "The design will be removed from " + scene::Layers.layer(layer).name + " on save";
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
	state.error = "The route of the stored design isn't in the scenery any more (paths moved or removed)";
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
	state.from = state.picked_from = From;
	state.to = state.picked_to = To;
	state.route = std::move(route);
	state.selected = -1;
	profile_resample();
	if (false == profile_restore())
		profile_recognize();
	profile_fit_view();
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
		state.error = "The path can't carry a profile";
		return;
	}
	state.route = std::move(route);
	state.from = state.picked_from = state.route.spans.front().track;
	state.to = state.picked_to = state.route.spans.back().track;
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
	for (std::size_t i = 0; i < samples.size(); ++i)
	{
		auto const &position{samples[i].position};
		auto *patch{terrain_at(position.x, position.z)};
		if (patch != nullptr && patch->contains(position.x, position.z))
			state.terrain[i] = patch->height_at(position.x, position.z);
	}

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
	state.status = format("Grade line recognized from the track: %zu points, largest departure %.3f m", state.line.points.size(), state.departure);
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
		state.error = "Correct the errors to apply the grade line";
		return;
	}
	auto const line{state.line};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	auto const gaps{editor_track::apply_profile(
	    state.route, [&](double const Chainage) { return profile::elevation(line, Chainage); }, [&](double const Chainage) { return profile::grade(line, Chainage); }, profile::breaks(line), states, created)};
	auto const changed{states.size()};
	auto const added{created.size()};
	push_track_snapshot(std::move(states), std::move(created));
	state.from = state.route.spans.front().track;
	state.to = state.route.spans.back().track;
	profile_resample();
	profile_store();
	state.status = format("Grade line applied: %zu paths changed, %zu added, largest departure %.3f m", changed, added, state.departure);
	if (false == scene::Layers.empty())
		state.status += "\nThe paths and the design (//$p lines) go to the scenery files on save";
	if (false == gaps.empty())
		state.status += "\nSwitch ends off the adjoining paths, to be corrected:";
	for (auto const &gap : gaps)
		state.status += format("\n  %s -> %s: %+.3f m", gap.track->name().c_str(), gap.neighbour->name().c_str(), gap.gap);
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

void editor_mode::render_profile_window()
{
	auto &state{m_profile};
	if (false == state.open)
		return;
	ImGui::SetNextWindowSize(ImVec2(960.0f, 640.0f), ImGuiCond_FirstUseEver);
	if (false == ImGui::Begin("Vertical profile###trackprofile", &state.open))
	{
		ImGui::End();
		return;
	}
	render_profile_source();
	if (state.route.spans.empty())
	{
		ImGui::TextDisabled("Select a path in the viewport, then build the route along the line, or set its start and end paths (switches are passed through)");
		if (state.picked_from != nullptr)
			ImGui::Text("Start: %s", state.picked_from->name().c_str());
		if (false == state.error.empty())
			ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", state.error.c_str());
		ImGui::End();
		return;
	}
	ImGui::Text("Route: %zu paths, %.2f m, %s ... %s", state.route.spans.size(), state.route.length, state.route.spans.front().track->name().c_str(), state.route.spans.back().track->name().c_str());
	{
		bool reversed;
		auto const stored{profile_find(reversed)};
		if (stored >= 0)
		{
			ImGui::TextDisabled("Design kept in %s%s", scene::Layers.layer(m_profiles[stored].layer).name.c_str(), state.changed ? ", changes go there on save" : "");
			ImGui::SameLine();
			if (ImGui::SmallButton("Forget the design"))
				profile_forget();
		}
		else if (state.changed)
			ImGui::TextDisabled("The design goes to the scenery file of the route on save");
	}

	if (render_profile_parameters())
		profile_edited();

	render_profile_canvas();

	ImGui::Columns(2, "##profilecolumns", true);
	render_profile_point();
	ImGui::NextColumn();
	render_profile_issues();
	ImGui::Columns(1);
	ImGui::End();
}

void editor_mode::render_profile_source()
{
	auto &state{m_profile};
	auto *selected{selected_track()};
	if (ImGui::Button("Along the line from the selected path") && selected != nullptr)
		profile_open_run(*selected);
	ImGui::SameLine();
	ImGui::PushItemWidth(70.0f);
	ImGui::InputDouble("m each way##profilerun", &state.run_length, 0.0, 0.0, "%.0f");
	ImGui::PopItemWidth();
	state.run_length = std::clamp(state.run_length, 10.0, 50000.0);
	ImGui::SameLine();
	if (ImGui::Button("Start: selected path") && selected != nullptr)
		state.picked_from = selected;
	ImGui::SameLine();
	if (ImGui::Button("End: selected path") && selected != nullptr && state.picked_from != nullptr)
		profile_open(state.picked_from, selected);
	profile_library_load();
	if (false == m_profiles.empty() && ImGui::TreeNode("##profilestored", "Designs kept in the scenery (%zu)", m_profiles.size()))
	{
		for (std::size_t i = 0; i < m_profiles.size(); ++i)
		{
			auto const &entry{m_profiles[i]};
			ImGui::PushID(static_cast<int>(i));
			if (ImGui::SmallButton("Open"))
				profile_open_stored(i);
			ImGui::SameLine();
			ImGui::Text("%s  %.1f m, %zu points, from (%.1f, %.1f) to (%.1f, %.1f)", scene::Layers.layer(entry.layer).name.c_str(), entry.length, entry.line.points.size(), entry.start.x, entry.start.z, entry.end.x, entry.end.z);
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
	ImGui::PushItemWidth(90.0f);
	ImGui::InputDouble("Chainage of the start (m)", &state.origin, 0.0, 0.0, "%.2f");
	ImGui::SameLine();
	changed |= ImGui::InputDouble("Design speed (km/h)", &line.speed, 0.0, 0.0, "%.0f");
	ImGui::SameLine();
	ImGui::SliderFloat("Exaggeration##profile", &state.exaggeration, 1.0f, 500.0f, "1:%.0f", 2.0f);
	ImGui::PopItemWidth();
	line.speed = std::max(1.0, line.speed);
	ImGui::Checkbox("Track", &state.show_track);
	ImGui::SameLine();
	ImGui::Checkbox("Terrain", &state.show_terrain);
	ImGui::SameLine();
	ImGui::Checkbox("Plan curvature", &state.show_plan);
	ImGui::SameLine();
	if (ImGui::Button("Fit the view"))
		profile_fit_view();
	ImGui::SameLine();
	if (ImGui::Button("Recognize from the track"))
	{
		profile_recognize();
		state.changed = true;
	}
	ImGui::SameLine();
	if (ImGui::Button("Apply to the scenery"))
		profile_apply();
	if (ImGui::TreeNode("Limits##profile"))
	{
		auto &norms{line.norms};
		ImGui::PushItemWidth(90.0f);
		changed |= ImGui::InputDouble("Maximum grade (per mille)", &norms.grade_max, 0.0, 0.0, "%.2f");
		changed |= ImGui::InputDouble("Maximum grade over switches (per mille)", &norms.grade_switch_max, 0.0, 0.0, "%.2f");
		changed |= ImGui::InputDouble("Vertical curve R >= k*V^2, k", &norms.radius_factor, 0.0, 0.0, "%.2f");
		changed |= ImGui::InputDouble("Vertical curve R min (m)", &norms.radius_min, 0.0, 0.0, "%.0f");
		changed |= ImGui::InputDouble("Vertical curve needed above a change of (per mille)", &norms.curve_threshold, 0.0, 0.0, "%.2f");
		changed |= ImGui::InputDouble("Constant grade between curves at least (m)", &norms.element_min, 0.0, 0.0, "%.0f");
		changed |= ImGui::InputDouble("Tolerance at fixed points (m)", &norms.fixed_tolerance, 0.0, 0.0, "%.3f");
		ImGui::PopItemWidth();
		if (ImGui::SmallButton("Automatic R at all points"))
		{
			for (auto &point : line.points)
				point.automatic = true;
			changed = true;
		}
		ImGui::TextDisabled("Required R for %.0f km/h: %.0f m", line.speed, profile::required_radius(line));
		ImGui::TreePop();
	}
	if (state.context.start_joined)
		changed |= ImGui::Checkbox("Join the adjoining track at the start", &line.join_start);
	if (state.context.start_joined && state.context.end_joined)
		ImGui::SameLine();
	if (state.context.end_joined)
		changed |= ImGui::Checkbox("Join the adjoining track at the end", &line.join_end);
	if ((state.context.start_joined || state.context.end_joined) && ImGui::IsItemHovered())
		ImGui::SetTooltip("Keeps the elevation of the joint and takes over the grade of the adjoining track,\n"
		                  "with a vertical curve which starts right at the joint");
	return changed;
}

void editor_mode::render_profile_issues()
{
	auto &state{m_profile};
	ImGui::Text("Largest departure from the track: %.3f m", state.departure);
	ImGui::BeginChild("##profileissues", ImVec2(0.0f, 0.0f), false);
	if (state.issues.empty())
		ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.4f, 1.0f), "The grade line meets the limits");
	for (std::size_t i = 0; i < state.issues.size(); ++i)
	{
		auto const &issue{state.issues[i]};
		ImGui::PushID(static_cast<int>(i));
		ImGui::PushStyleColor(ImGuiCol_Text, issue.error ? ImVec4(1.0f, 0.4f, 0.3f, 1.0f) : ImVec4(1.0f, 0.8f, 0.3f, 1.0f));
		if (ImGui::Selectable(issue.text.c_str()))
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
	if (state.selected < 0)
	{
		ImGui::TextDisabled("LMB: select and drag a point (Ctrl: height only, Shift: keep the grade before)\n"
		                    "Green grip on a curve: drag to set R   Double click on the line: insert a point\n"
		                    "Del: remove the point   Wheel: zoom   Ctrl+wheel: exaggeration   RMB/MMB: pan");
	}
	else if (points[state.selected].joint)
	{
		auto const index{state.selected};
		auto const &point{points[index]};
		auto const change{(profile::grade_after(state.line, index) - profile::grade_after(state.line, index - 1)) * 1000.0};
		ImGui::Text("Point %d: joint with the adjoining track (placed automatically)", index + 1);
		ImGui::Text("km %s  H %.3f  R %.0f", profile::format_chainage(point.chainage + state.origin).c_str(), point.elevation, point.radius);
		ImGui::Text("Grade %+.3f -> %+.3f per mille (%s)", profile::grade_after(state.line, index - 1) * 1000.0, profile::grade_after(state.line, index) * 1000.0, change > 0.0 ? "sag" : "crest");
		ImGui::TextDisabled("Moves with the neighbouring point; untick the joining to edit the end freely");
	}
	else
	{
		auto const index{state.selected};
		auto &point{points[index]};
		bool const end{index == 0 || index == count - 1};
		bool const locked{end_locked(state.line, state.context, index)};
		ImGui::Text("Point %d%s", index + 1, locked ? " (end, joined to the adjoining track)" : end ? " (end of the route)" : "");
		ImGui::PushItemWidth(110.0f);
		auto chainage{point.chainage + state.origin};
		if (ImGui::InputDouble("Chainage (m)", &chainage, 0.0, 0.0, "%.2f") && false == end)
		{
			point.chainage = std::clamp(chainage - state.origin, points[index - 1].chainage + 0.5, points[index + 1].chainage - 0.5);
			changed = true;
		}
		ImGui::SameLine();
		ImGui::TextDisabled("km %s", profile::format_chainage(point.chainage + state.origin).c_str());
		changed |= ImGui::InputDouble("Elevation (m)", &point.elevation, 0.0, 0.0, "%.3f", locked ? ImGuiInputTextFlags_ReadOnly : 0);
		if (index > 0 && false == locked)
		{
			auto grade{profile::grade_after(state.line, index - 1) * 1000.0};
			if (ImGui::InputDouble("Grade from the previous (per mille)", &grade, 0.0, 0.0, "%.3f"))
			{
				point.elevation = points[index - 1].elevation + grade / 1000.0 * (point.chainage - points[index - 1].chainage);
				changed = true;
			}
		}
		if (index + 1 < count)
		{
			auto grade{profile::grade_after(state.line, index) * 1000.0};
			if (ImGui::InputDouble("Grade to the next (per mille)", &grade, 0.0, 0.0, "%.3f"))
			{
				points[index + 1].elevation = point.elevation + grade / 1000.0 * (points[index + 1].chainage - point.chainage);
				changed = true;
			}
		}
		if (false == end)
		{
			if (ImGui::InputDouble("Vertical curve R (m, 0: none)", &point.radius, 0.0, 0.0, "%.0f"))
			{
				point.automatic = false;
				changed = true;
			}
			point.radius = std::max(0.0, point.radius);
			ImGui::SameLine();
			changed |= ImGui::Checkbox("Auto", &point.automatic);
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("R follows the limits: the required radius where the change of grade needs a curve");
			auto const change{(profile::grade_after(state.line, index) - profile::grade_after(state.line, index - 1)) * 1000.0};
			auto const tangent{point.radius * std::abs(change) / 2000.0};
			ImGui::Text("Change of grade %+.3f per mille, %s", change, change > 0.0 ? "sag" : "crest");
			if (point.radius > 0.0)
				ImGui::Text("Tangent T %.2f m, sagitta f %.3f m", tangent, tangent * tangent / (2.0 * point.radius));
		}
		ImGui::PopItemWidth();
		if (index + 1 < count && ImGui::SmallButton("Insert a point halfway to the next"))
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
			if (ImGui::SmallButton("Remove the point"))
			{
				points.erase(points.begin() + index);
				state.selected = -1;
				changed = true;
			}
		}
	}
	ImGui::BeginChild("##profilepoints", ImVec2(0.0f, 0.0f), true);
	for (int i = 0; i < static_cast<int>(points.size()); ++i)
	{
		auto const &point{points[i]};
		auto const label{format("%2d  km %s  H %.3f  %s##point%d", i + 1, profile::format_chainage(point.chainage + state.origin).c_str(), point.elevation, point.radius > 0.0 ? format("R %.0f%s", point.radius, point.joint ? " joint" : point.automatic ? " auto" : "").c_str() : "", i)};
		if (ImGui::Selectable(label.c_str(), state.selected == i))
			state.selected = (state.selected == i ? -1 : i);
	}
	ImGui::EndChild();
	if (changed)
		profile_edited();
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

void editor_mode::render_profile_canvas()
{
	auto &state{m_profile};
	auto const available{ImGui::GetContentRegionAvail()};
	ImVec2 const size{std::max(200.0f, available.x), std::max(240.0f, available.y * 0.6f)};
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
		profile_canvas_tooltip();
	}
	draw->PopClipRect();
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
	if ((Hovered || ImGui::IsWindowFocused()) && state.selected > 0 && state.selected + 1 < static_cast<int>(points.size()) && false == points[state.selected].joint && ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Delete)))
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
	for (auto s = std::ceil((state.view_from + state.origin) / step) * step - state.origin; s <= state.view_to; s += step)
	{
		auto const x{View.x_of(s)};
		Draw.AddLine(ImVec2(x, plottop), ImVec2(x, plotbottom), kGrid);
		Draw.AddText(ImVec2(x + 2.0f, View.corner.y + 2.0f), kLabel, profile::format_chainage(s + state.origin).c_str());
	}
	auto const heightstep{nice_step(View.vertical * 45.0)};
	for (auto h = std::floor(View.height_of(plotbottom) / heightstep) * heightstep; h <= View.height_of(plottop); h += heightstep)
	{
		auto const y{View.y_of(h)};
		Draw.AddLine(ImVec2(View.corner.x, y), ImVec2(bottomright.x, y), kGrid);
		Draw.AddText(ImVec2(View.corner.x + 2.0f, y - 14.0f), kLabel, format("%.1f", h).c_str());
	}
	for (auto const &zone : state.context.switches)
	{
		Draw.AddRectFilled(ImVec2(View.x_of(zone.from), plottop), ImVec2(std::max(View.x_of(zone.to), View.x_of(zone.from) + 2.0f), plotbottom), kSwitch);
		Draw.AddText(ImVec2(View.x_of(zone.from) + 2.0f, plottop + 2.0f), IM_COL32(200, 160, 255, 220), zone.name.c_str());
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
		polyline([&](std::size_t const I) { return state.terrain[I]; }, kTerrain, 1.5f);
	if (state.show_track)
		polyline([&](std::size_t const I) { return samples[I].position.y; }, kTrack, 1.5f);

	for (std::size_t i = 0; i + 1 < points.size(); ++i)
		Draw.AddLine(ImVec2(View.x_of(points[i].chainage), View.y_of(points[i].elevation)), ImVec2(View.x_of(points[i + 1].chainage), View.y_of(points[i + 1].elevation)), kTangents, 1.0f);
	{
		auto const from{std::max(points.front().chainage, View.chainage_of(View.corner.x))};
		auto const to{std::min(points.back().chainage, View.chainage_of(bottomright.x))};
		ImVec2 previous{View.x_of(from), View.y_of(profile::elevation(line, from))};
		for (auto x = previous.x + 2.0f; x <= View.x_of(to) + 2.0f; x += 2.0f)
		{
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
		auto const text{format("%+.2f o/oo  L %.1f", profile::grade_after(line, i) * 1000.0, points[i + 1].chainage - points[i].chainage)};
		Draw.AddText(ImVec2(x - 40.0f, View.y_of(profile::elevation(line, middle)) - 30.0f), kGradeLine, text.c_str());
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
		Draw.AddCircleFilled(grip, 4.5f, overlay_color::grip, 4);
		if (static_cast<int>(c.point) == state.curve_grip)
			Draw.AddCircle(grip, 9.0f, overlay_color::highlight, 12, 2.0f);
		Draw.AddText(ImVec2(grip.x + 7.0f, grip.y + 4.0f), overlay_color::grip, format("R %.0f", points[c.point].radius).c_str());
	}
	for (int i = 0; i < static_cast<int>(points.size()); ++i)
	{
		ImVec2 const at{View.x_of(points[i].chainage), View.y_of(points[i].elevation)};
		if (i == 0 || i + 1 == static_cast<int>(points.size()))
			Draw.AddRectFilled(ImVec2(at.x - 5.0f, at.y - 5.0f), ImVec2(at.x + 5.0f, at.y + 5.0f), IM_COL32(70, 140, 255, 255));
		else if (points[i].joint)
			Draw.AddQuadFilled(ImVec2(at.x, at.y - 6.0f), ImVec2(at.x + 6.0f, at.y), ImVec2(at.x, at.y + 6.0f), ImVec2(at.x - 6.0f, at.y), IM_COL32(90, 210, 230, 255));
		else
			Draw.AddCircleFilled(at, 5.5f, overlay_color::marked);
		if (i == state.selected)
			Draw.AddCircle(at, 10.0f, overlay_color::highlight, 16, 2.0f);
		Draw.AddText(ImVec2(at.x + 7.0f, at.y - 18.0f), overlay_color::marked, format("%d  %.3f", i + 1, points[i].elevation).c_str());
	}
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
		Draw.AddText(ImVec2(View.corner.x + 2.0f, plotbottom + 1.0f), kLabel, "plan: curvature (up: left)");
	}
}

void editor_mode::profile_canvas_tooltip() const
{
	auto const &state{m_profile};
	auto const &samples{state.samples};
	auto const designed{profile::elevation(state.line, state.hover)};
	auto const existing{editor_track::sampled_elevation(samples, state.hover)};
	ImGui::BeginTooltip();
	ImGui::Text("km %s", profile::format_chainage(state.hover + state.origin).c_str());
	ImGui::Text("Grade line %.3f m, %+.2f per mille", designed, profile::grade(state.line, state.hover) * 1000.0);
	ImGui::Text("Track %.3f m (%+.3f), %+.2f per mille", existing, existing - designed, editor_track::sampled_grade(samples, state.hover) * 1000.0);
	auto const next{std::lower_bound(samples.begin(), samples.end(), state.hover, [](editor_track::route_sample const &Sample, double const Value) { return Sample.chainage < Value; })};
	auto const index{static_cast<std::size_t>(std::distance(samples.begin(), std::min(next, std::prev(samples.end()))))};
	if (index < state.terrain.size() && false == std::isnan(state.terrain[index]))
		ImGui::Text("Terrain %.3f m, %s %.2f m", state.terrain[index], designed > state.terrain[index] ? "fill" : "cut", std::abs(designed - state.terrain[index]));
	auto const &span{state.route.spans[samples[index].span]};
	ImGui::TextDisabled("%s%s", span.track->name().c_str(), span.track->eType != tt_Normal ? " (switch, tilted as a whole with its branch)" : "");
	ImGui::EndTooltip();
}

void editor_mode::draw_profile_overlay() const
{
	auto const &state{m_profile};
	if (false == state.open || state.samples.size() < 2)
		return;
	screen_projection const projection;
	ImDrawList *drawlist{ImGui::GetBackgroundDrawList()};
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
