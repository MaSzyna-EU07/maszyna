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
#include "application/editorprojection.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"

#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include "utilities/translation.h"
#include <algorithm>
#include <cmath>

namespace
{

using geometry::bezier;
using geometry::cross;
using geometry::plan_of;

double const kParallelReach{5000.0}; // m each way along the line
double const kSpacings[] = {3.5, 4.0, 4.5, 4.75, 5.0, 6.0};

segment_data reversed(segment_data const &Path)
{
	segment_data result{Path};
	result.points[segment_data::point::start] = Path.points[segment_data::point::end];
	result.points[segment_data::point::end] = Path.points[segment_data::point::start];
	result.points[segment_data::point::control1] = Path.points[segment_data::point::control2];
	result.points[segment_data::point::control2] = Path.points[segment_data::point::control1];
	result.rolls = {-Path.rolls[1], -Path.rolls[0]};
	return result;
}

// the control vectors as they're drawn, also for a straight given without them
std::pair<glm::dvec3, glm::dvec3> controls(segment_data const &Path)
{
	auto const &start{Path.points[segment_data::point::start]};
	auto const &end{Path.points[segment_data::point::end]};
	auto const &control1{Path.points[segment_data::point::control1]};
	auto const &control2{Path.points[segment_data::point::control2]};
	if (control1 == glm::dvec3{} && control2 == glm::dvec3{})
		return {(end - start) / 3.0, (start - end) / 3.0};
	return {control1, control2};
}

// curvature in the plan at the start (false) or the end (true) of the path, 1/m, positive turning left
double curvature(segment_data const &Path, bool const Atend)
{
	auto const [control1, control2] = controls(Path);
	auto const p0{plan_of(Path.points[segment_data::point::start])};
	auto const p3{plan_of(Path.points[segment_data::point::end])};
	auto const p1{p0 + plan_of(control1)};
	auto const p2{p3 + plan_of(control2)};
	auto const first{Atend ? 3.0 * (p3 - p2) : 3.0 * (p1 - p0)};
	auto const second{Atend ? 6.0 * (p1 - 2.0 * p2 + p3) : 6.0 * (p0 - 2.0 * p1 + p2)};
	auto const speed{glm::length(first)};
	return speed > 1e-9 ? cross(first, second) / (speed * speed * speed) : 0.0;
}

glm::dvec2 direction(segment_data const &Path, bool const Atend)
{
	auto const [control1, control2] = controls(Path);
	auto const along{Atend ? -plan_of(control2) : plan_of(control1)};
	return glm::length(along) > 1e-9 ? glm::normalize(along) : glm::dvec2{0.0, 1.0};
}

} // namespace

void editor_mode::parallel_update()
{
	auto &tool{m_parallel};
	auto *track{selected_track()};
	if (track == tool.built_for && tool.spacing == tool.built_spacing && tool.side == tool.built_side && tool.scope == tool.built_scope && m_history.size() == tool.built_history)
		return;
	tool.built_for = track;
	tool.built_spacing = tool.spacing;
	tool.built_side = tool.side;
	tool.built_scope = tool.scope;
	tool.built_history = m_history.size();
	tool.pieces.clear();
	tool.styles.clear();
	tool.length = 0.0;
	tool.error.clear();
	if (track == nullptr)
		return;
	if (track->eType != tt_Normal || false == editor_track::is_supported(*track))
	{
		tool.error = STR_C("A switch has no line of its own: select a plain path");
		return;
	}
	std::vector<std::pair<TTrack *, segment_data>> sources;
	if (tool.scope == 1)
	{
		editor_track::chain run;
		std::string error;
		if (editor_track::find_run(*track, kParallelReach, run, error))
			for (std::size_t i = 0; i < run.tracks.size(); ++i)
				sources.emplace_back(run.tracks[i], run.forward[i] ? run.tracks[i]->m_paths.front() : reversed(run.tracks[i]->m_paths.front()));
	}
	if (sources.empty())
		sources.emplace_back(track, track->m_paths.front());
	auto const offset{tool.spacing * tool.side};
	// the joints are moved once, along the mean direction of the paths meeting there, so the new paths meet too
	std::vector<glm::dvec3> joints;
	for (std::size_t k = 0; k <= sources.size(); ++k)
	{
		bool const first{k == 0};
		bool const last{k == sources.size()};
		auto const &before{sources[last ? k - 1 : (first ? 0 : k - 1)].second};
		auto const &after{sources[last ? k - 1 : k].second};
		auto const point{first ? after.points[segment_data::point::start] : before.points[segment_data::point::end]};
		glm::dvec2 heading{first ? direction(after, false) : last ? direction(before, true) : direction(before, true) + direction(after, false)};
		heading = glm::length(heading) > 1e-9 ? glm::normalize(heading) : glm::dvec2{0.0, 1.0};
		auto const roll{first ? after.rolls[0] : last ? before.rolls[1] : 0.5f * (before.rolls[1] + after.rolls[0])};
		glm::dvec2 const left{-heading.y, heading.x};
		auto const planar{plan_of(point) + left * offset};
		joints.emplace_back(planar.x, point.y - offset * std::tan(glm::radians(static_cast<double>(roll))), planar.y);
	}
	for (std::size_t i = 0; i < sources.size(); ++i)
	{
		auto const &source{sources[i].second};
		auto const [control1, control2] = controls(source);
		auto const startscale{1.0 - curvature(source, false) * offset};
		auto const endscale{1.0 - curvature(source, true) * offset};
		if (startscale < 0.05 || endscale < 0.05)
		{
			tool.pieces.clear();
			tool.styles.clear();
			tool.error = format(STR_C("The spacing is larger than the radius of %s"), sources[i].first->name().c_str());
			return;
		}
		segment_data piece;
		piece.points[segment_data::point::start] = joints[i];
		piece.points[segment_data::point::end] = joints[i + 1];
		piece.points[segment_data::point::control1] = control1 * startscale;
		piece.points[segment_data::point::control2] = control2 * endscale;
		piece.rolls = source.rolls;
		auto const middle{0.5 * (curvature(source, false) + curvature(source, true))};
		if (source.radius != 0.0f && std::abs(middle) > 1e-7)
			piece.radius = static_cast<float>(std::copysign(std::abs((1.0 - middle * offset) / middle), static_cast<double>(source.radius)));
		tool.length += bezier{piece}.plan_length();
		tool.pieces.push_back(piece);
		tool.styles.push_back(sources[i].first);
	}
}

void editor_mode::parallel_build()
{
	auto &tool{m_parallel};
	parallel_update();
	if (tool.pieces.empty())
		return;
	std::vector<TTrack *> created;
	for (std::size_t i = 0; i < tool.pieces.size(); ++i)
		created.push_back(editor_track::create_path(*tool.styles[i], tool.pieces[i]));
	editor_track::commit(created);
	push_track_snapshot({}, created);
	tool.status = format(STR_C("Parallel track of %zu paths, %.1f m, %.2f m away; Ctrl+Z takes it back"), created.size(), tool.length, tool.spacing);
	if (scene::Layers.active() == null_handle)
		tool.status += "\n" + std::string{STR_C("The scenery isn't opened for editing, the new track won't be saved")};
	WriteLog("Editor: " + tool.status, logtype::generic);
	tool.built_for = nullptr;
}

void editor_mode::render_parallel_ui()
{
	auto &tool{m_parallel};
	ImGui::TextDisabled("%s", STR_C("Second track alongside, curves included: radius R minus or plus the spacing,\nthe same transitions and cant, heights on the plane of the cant"));
	ImGui::RadioButton(STR_C("This path"), &tool.scope, 0);
	ImGui::SameLine();
	ImGui::RadioButton(STR_C("The line, up to the switches"), &tool.scope, 1);
	ImGui::RadioButton(STR_C("Left"), &tool.side, -1);
	ImGui::SameLine();
	ImGui::RadioButton(STR_C("Right"), &tool.side, 1);
	ImGui::SameLine();
	ImGui::TextDisabled("%s", STR_C("(of the direction of the line, see the preview)"));
	ImGui::SetNextItemWidth(90.0f);
	ImGui::InputDouble(STR_C("Spacing (m)"), &tool.spacing, 0.0, 0.0, "%.2f");
	tool.spacing = std::clamp(tool.spacing, 1.5, 100.0);
	for (auto const spacing : kSpacings)
	{
		ImGui::SameLine();
		if (ImGui::SmallButton(format("%.2f", spacing).c_str()))
			tool.spacing = spacing;
	}
	parallel_update();
	if (false == tool.error.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "%s", tool.error.c_str());
	else if (false == tool.pieces.empty())
	{
		if (ImGui::Button(format(STR_C("Build %zu paths, %.1f m"), tool.pieces.size(), tool.length).c_str()))
			parallel_build();
	}
	if (false == tool.status.empty())
		ImGui::TextWrapped("%s", tool.status.c_str());
}

void editor_mode::draw_parallel_preview() const
{
	auto const &tool{m_parallel};
	if (false == tool.open || tool.pieces.empty() || selected_track() != tool.built_for)
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList()};
	for (auto const &piece : tool.pieces)
	{
		bezier const curve{piece};
		auto const count{std::clamp(static_cast<int>(curve.plan_length() / 2.0), 4, 128)};
		auto previous{curve.point(0.0)};
		for (int k = 1; k <= count; ++k)
		{
			auto const next{curve.point(static_cast<double>(k) / count)};
			projection.line(drawlist, previous, next, k % 2 ? IM_COL32(90, 255, 120, 230) : IM_COL32(90, 255, 120, 90), 3.0f);
			previous = next;
		}
	}
	ImVec2 screen;
	if (projection.project(tool.pieces.front().points[segment_data::point::start], screen))
		drawlist->AddCircleFilled(screen, 5.0f, IM_COL32(90, 255, 120, 230));
}
