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

std::string name_of(TTrack const &Track)
{
	return Track.name().empty() ? std::string{"(noname)"} : Track.name();
}

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

glm::dvec3 midpoint(TTrack const &Track)
{
	if (Track.m_paths.empty())
		return Track.location();
	bezier const curve{Track.m_paths.front()};
	return curve.point(0.5);
}

// signed offset of Dst from Src, along Src's left normal; positive is the Left radio
double lateral_of(TTrack const &Src, TTrack const &Dst)
{
	if (Src.m_paths.empty())
		return 0.0;
	auto const heading{direction(Src.m_paths.front(), false)};
	glm::dvec2 const left{-heading.y, heading.x};
	return glm::dot(plan_of(midpoint(Dst)) - plan_of(Src.m_paths.front().points[segment_data::point::start]), left);
}

} // namespace

TTrack *editor_mode::parallel_source() const
{
	if (m_track_set.size() >= 2)
		return m_track_set.front();
	return selected_track();
}

TTrack *editor_mode::parallel_target() const
{
	if (m_track_set.size() != 2)
		return nullptr;
	return m_track_set.back() != m_track_set.front() ? m_track_set.back() : nullptr;
}

void editor_mode::parallel_update()
{
	auto &tool{m_parallel};
	auto *track{parallel_source()};
	auto *target{parallel_target()};
	if (track != tool.pair_src || target != tool.pair_dst)
	{
		tool.pair_src = track;
		tool.pair_dst = target;
		if (track != nullptr && target != nullptr)
		{
			auto const lateral{lateral_of(*track, *target)};
			tool.side = lateral >= 0.0 ? 1 : -1;
			tool.measured = std::abs(lateral);
		}
		else
			tool.measured = 0.0;
	}
	else if (track != nullptr && target != nullptr)
		tool.measured = std::abs(lateral_of(*track, *target));

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

void editor_mode::parallel_apply()
{
	auto &tool{m_parallel};
	auto *target{parallel_target()};
	parallel_update();
	if (target == nullptr || tool.pieces.empty())
		return;
	if (tool.pieces.size() != 1)
	{
		tool.error = STR_C("Two selected paths: set the scope to this path, so the other takes its shape at the spacing");
		return;
	}
	std::string reason;
	if (false == editor_track::can_edit_geometry(*target, reason))
	{
		tool.error = reason;
		return;
	}
	auto const joints{editor_track::joints(*target)};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	auto const remember = [&](TTrack *Track) {
		if (Track == nullptr)
			return;
		if (std::any_of(states.begin(), states.end(), [&](auto const &Entry) { return Entry.first == Track; }))
			return;
		states.emplace_back(Track, editor_track::capture(*Track));
	};
	remember(target);
	for (auto const &joint : joints)
		remember(joint.other);
	target->m_paths.front() = tool.pieces.front();
	editor_track::follow(*target, joints);
	std::vector<TTrack *> committed{target};
	for (auto const &joint : joints)
		if (joint.other != nullptr)
			committed.push_back(joint.other);
	editor_track::commit(committed);
	push_track_snapshot(std::move(states));
	tool.status = format(STR_C("%s set parallel to %s, %.2f m away; Ctrl+Z takes it back"), name_of(*target).c_str(), name_of(*parallel_source()).c_str(), tool.spacing);
	WriteLog("Editor: " + tool.status, logtype::generic);
	tool.built_for = nullptr;
	straight_refresh();
}

void editor_mode::render_parallel_controls(bool const Scope)
{
	auto &tool{m_parallel};
	if (Scope)
	{
		ImGui::RadioButton(STR_C("This path"), &tool.scope, 0);
		ImGui::SameLine();
		ImGui::RadioButton(STR_C("The line, up to the switches"), &tool.scope, 1);
	}
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
}

void editor_mode::render_parallel_ui()
{
	auto &tool{m_parallel};
	ImGui::TextDisabled("%s", STR_C("First select a straight path (LMB). Then a second track is laid beside it.\nAlong the line the curves are included: R minus or plus the spacing, same transitions and cant."));
	if (parallel_target() != nullptr)
		ImGui::TextDisabled("%s", STR_C("Two paths selected: set the spacing of the other in Select. The first must be a straight."));
	else if (auto *source{parallel_source()}; source == nullptr)
		ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "%s", STR_C("First select a straight path in the view."));
	else if (source->eType != tt_Normal || false == editor_track::is_straight(*source, m_straights.tolerance))
		ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "%s", STR_C("The selected path isn't a straight. Select a straight first."));
	render_parallel_controls();
	parallel_update();
	if (false == tool.error.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "%s", tool.error.c_str());
	else if (false == tool.pieces.empty())
	{
		if (ImGui::Button(format(STR_C("Lay %zu paths, %.1f m"), tool.pieces.size(), tool.length).c_str()))
			parallel_build();
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("First select a straight. Creates a new track beside it; the selected path stays."));
	}
	if (false == tool.status.empty())
		ImGui::TextWrapped("%s", tool.status.c_str());
}

void editor_mode::render_parallel_set_ui()
{
	auto &tool{m_parallel};
	auto *source{parallel_source()};
	auto *target{parallel_target()};
	if (source == nullptr || target == nullptr)
		return;
	if (false == ImGui::CollapsingHeader(STR_C("Make parallel"), ImGuiTreeNodeFlags_DefaultOpen))
		return;
	ImGui::TextWrapped("%s", STR_C("First selected must be a straight. It stays; the other is moved to this spacing."));
	ImGui::TextWrapped("%s", format(STR_C("%s stays. %s is moved to this spacing, taking its curves."), name_of(*source).c_str(), name_of(*target).c_str()).c_str());
	if (tool.measured > 0.05)
		ImGui::TextDisabled("%s", format(lateral_of(*source, *target) >= 0.0 ? STR_C("Now %.2f m on the right") : STR_C("Now %.2f m on the left"), tool.measured).c_str());
	tool.scope = 0;
	render_parallel_controls(false);
	parallel_update();
	if (false == tool.error.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "%s", tool.error.c_str());
	else if (false == tool.pieces.empty() && ImGui::Button(format(STR_C("Set %s to %.2f m"), name_of(*target).c_str(), tool.spacing).c_str()))
		parallel_apply();
	if (false == tool.status.empty())
		ImGui::TextWrapped("%s", tool.status.c_str());
}

bool editor_mode::parallel_preview_visible() const
{
	if (m_parallel.pieces.empty() || parallel_source() != m_parallel.built_for)
		return false;
	if (m_track_tab == track_tab::lay)
		return m_lay.points.empty();
	if (m_track_tab == track_tab::path)
		return true;
	return m_track_tab == track_tab::lineside && m_parallel.open;
}

void editor_mode::draw_parallel_preview() const
{
	if (false == parallel_preview_visible())
		return;
	auto const &tool{m_parallel};
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
	{
		drawlist->AddCircleFilled(screen, 5.0f, IM_COL32(90, 255, 120, 230));
		auto const label{format("%.2f m", tool.spacing)};
		auto const size{ImGui::CalcTextSize(label.c_str())};
		drawlist->AddRectFilled(ImVec2(screen.x + 8.0f, screen.y - 8.0f), ImVec2(screen.x + 16.0f + size.x, screen.y + 8.0f), IM_COL32(0, 0, 0, 180), 3.0f);
		drawlist->AddText(ImVec2(screen.x + 12.0f, screen.y - 8.0f), IM_COL32(90, 255, 120, 255), label.c_str());
	}
}
