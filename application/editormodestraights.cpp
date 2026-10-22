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

void editor_mode::draw_straights_overlay() const
{
	screen_projection const projection;
	ImDrawList *drawlist = ImGui::GetBackgroundDrawList(ImGui::GetMainViewport());
	glm::dvec3 const camera{Global.pCamera.Pos};
	auto const nearby = [&](editor_track::straight const &Line) {
		auto const offset{camera - Line.start};
		auto const along{std::clamp(glm::dot(plan_of(offset), Line.direction), 0.0, Line.length)};
		auto const closest{Line.start + glm::dvec3{Line.direction.x, 0.0, Line.direction.y} * along};
		return plan_distance(closest, camera) < 3000.0;
	};
	auto const draw = [&](editor_track::straight const &Line, ImU32 const Color, float const Thickness) {
		if (Line.tracks.empty())
			return;
		projection.line(drawlist, Line.start, Line.end, Color, Thickness);
		for (auto const &point : {Line.start, Line.end})
		{
			ImVec2 screen;
			if (projection.project(point, screen))
				drawlist->AddCircleFilled(screen, Thickness + 2.0f, Color);
		}
	};
	for (int i = 0; i < static_cast<int>(m_straights.found.size()); ++i)
	{
		auto const &line{m_straights.found[i]};
		if (nearby(line))
			draw(line, i == m_straights.listed ? overlay_color::selected : IM_COL32(255, 255, 255, 110), i == m_straights.listed ? 4.0f : 2.0f);
	}
	for (auto const &member : m_straights.set)
		draw(member, IM_COL32(255, 150, 30, 255), 4.0f);
	draw(m_straights.current, overlay_color::selected, 4.0f);
	if (m_straights.tool_placed)
	{
		auto const &state{m_straights};
		auto const &line{state.tool_line};
		ImVec2 screen;
		if (projection.project(state.tool_point, screen))
			drawlist->AddCircleFilled(screen, 7.0f, overlay_color::invalid);
		if (projection.project(state.tool_handle, screen))
			drawlist->AddCircleFilled(screen, 7.0f, overlay_color::marked);
		if (state.tool == 1)
		{
			auto const direction{plan_of(state.tool_handle - state.tool_point)};
			if (glm::length(direction) > 1e-3)
			{
				auto const unit{glm::normalize(direction)};
				auto const rest{std::max(0.0, line.length - state.tool_at)};
				projection.line(drawlist, state.tool_point, state.tool_point + glm::dvec3{unit.x, line.grade, unit.y} * rest, overlay_color::marked, 3.0f);
			}
		}
		else
		{
			glm::dvec2 const normal{-line.direction.y, line.direction.x};
			auto const offset{plan_of(state.tool_handle - state.tool_point)};
			auto const shift{glm::dot(offset, normal)};
			auto const length{glm::dot(offset, line.direction)};
			auto const shifted{glm::dvec3{normal.x, 0.0, normal.y} * shift};
			auto const along{glm::dvec3{line.direction.x, 0.0, line.direction.y}};
			projection.line(drawlist, state.tool_point, state.tool_point + along * length + shifted, IM_COL32(255, 210, 60, 160), 2.0f);
			projection.line(drawlist, state.tool_point + along * length + shifted, line.end + shifted, overlay_color::marked, 3.0f);
		}
	}
	if (m_straights.dragging && m_straights.fitting)
	{
		auto const &result{m_route.result};
		if (result.length > 0.0 && result.profile.size() >= 2)
		{
			int const steps{std::clamp(static_cast<int>(result.length / 2.0), 2, 2000)};
			auto previous{alignment::evaluate(result, 0.0).position};
			for (int i = 1; i <= steps; ++i)
			{
				auto const next{alignment::evaluate(result, result.length * i / steps).position};
				projection.line(drawlist, previous, next, result.valid ? overlay_color::marked : overlay_color::invalid, 3.0f);
				previous = next;
			}
		}
	}
	else if (m_straights.dragging)
	{
		for (auto const &member : m_straights.drag_lines)
		{
			for (auto const *track : member.tracks)
			{
				auto const &path{track->m_paths.front()};
				projection.line(drawlist, path.points[segment_data::point::start], path.points[segment_data::point::end], overlay_color::marked, 3.0f);
			}
		}
	}
	else if (false == m_straights.current.tracks.empty())
	{
		auto const &line{m_straights.current};
		glm::dvec3 const handles[] = {line.start, line.end, (line.start + line.end) * 0.5};
		for (int i = 0; i < 3; ++i)
		{
			ImVec2 screen;
			if (false == projection.project(handles[i], screen))
				continue;
			if (i < 2)
				drawlist->AddRectFilled(ImVec2(screen.x - 6.0f, screen.y - 6.0f), ImVec2(screen.x + 6.0f, screen.y + 6.0f), overlay_color::selected);
			else
				drawlist->AddCircleFilled(screen, 6.0f, overlay_color::selected, 4);
			if (i == m_straights.handle)
				drawlist->AddCircle(screen, 12.0f, overlay_color::highlight, 16, 2.5f);
		}
	}
	auto line{m_straights.dragging ? m_straights.drag_line : m_straights.current};
	if (line.tracks.empty())
		return;
	glm::dvec3 const offset{m_straights.dragging ? m_straights.preview_start - m_straights.drag_line.start : glm::dvec3{}};
	if (m_straights.dragging)
	{
		line.start = m_straights.preview_start;
		line.end = m_straights.preview_end;
		auto const planar{plan_of(line.end - line.start)};
		auto const length{glm::length(planar)};
		if (length > 1e-6)
		{
			line.direction = planar / length;
			line.length = length;
		}
	}
	auto const offer{straight_parallel_offer(m_straights.dragging ? m_straights.drag_line : m_straights.current, offset)};
	glm::dvec2 const normal{-line.direction.y, line.direction.x};
	for (auto const &other : m_straights.neighbours)
	{
		auto const across = [&](glm::dvec3 const &Point) { return glm::dot(plan_of(Point - line.start), normal); };
		auto const a0{line.along(other.start)}, a1{line.along(other.end)};
		auto const from{std::max(0.0, std::min(a0, a1))};
		auto const to{std::min(line.length, std::max(a0, a1))};
		if (to <= from)
			continue;
		auto const middle{(from + to) * 0.5};
		auto const fraction{std::abs(a1 - a0) > 1e-6 ? (middle - a0) / (a1 - a0) : 0.0};
		auto const lateral{across(other.start) + (across(other.end) - across(other.start)) * fraction};
		glm::dvec3 const foot{line.start.x + line.direction.x * middle, line.start.y + line.grade * middle, line.start.z + line.direction.y * middle};
		glm::dvec3 const target{foot.x + normal.x * lateral, foot.y, foot.z + normal.y * lateral};
		bool const highlighted{offer.nearby && offer.neighbour == &other};
		auto const tick{highlighted ? (offer.snaps ? IM_COL32(80, 230, 255, 255) : IM_COL32(255, 210, 60, 255)) : IM_COL32(255, 230, 120, 230)};
		if (highlighted)
			projection.line(drawlist, other.start, other.end, tick, 3.0f);
		projection.line(drawlist, foot, target, tick, highlighted ? 3.0f : 1.5f);
		ImVec2 screen;
		if (projection.project((foot + target) * 0.5, screen))
		{
			auto const angle{glm::degrees(std::abs(std::asin(std::clamp(line.direction.x * other.direction.y - line.direction.y * other.direction.x, -1.0, 1.0))))};
			auto const text{highlighted ? (offer.snaps ? format(STR_C("parallel  %.2f m"), offer.spacing) : format("%.2f -> %.2f m  %.1f°", std::abs(lateral), offer.spacing, offer.angle)) :
			                             (angle > 0.05 ? format("%.2f m  %.1f°", std::abs(lateral), angle) : format("%.2f m", std::abs(lateral)))};
			auto const *label{text.c_str()};
			auto const size{ImGui::CalcTextSize(label)};
			drawlist->AddRectFilled(ImVec2(screen.x - 4.0f, screen.y - 2.0f), ImVec2(screen.x + size.x + 4.0f, screen.y + size.y + 2.0f), IM_COL32(0, 0, 0, 190), 3.0f);
			drawlist->AddText(screen, highlighted ? tick : IM_COL32(255, 230, 120, 255), label);
		}
	}
	if (offer.nearby && false == offer.snaps && std::abs(offer.correction) > 0.02)
	{
		auto const shift{glm::dvec3{normal.x, 0.0, normal.y} * offer.correction};
		projection.line(drawlist, line.start + shift, line.end + shift, IM_COL32(80, 230, 255, 210), 3.5f);
	}
}

void editor_mode::find_neighbour_straights()
{
	auto &state{m_straights};
	state.neighbours.clear();
	auto const &line{state.current};
	if (line.tracks.empty())
		return;
	glm::dvec2 const normal{-line.direction.y, line.direction.x};
	auto const middle{(line.start + line.end) * 0.5};
	std::vector<TTrack const *> visited(line.tracks.begin(), line.tracks.end());
	auto const sections{simulation::Region->sections(middle, static_cast<float>(line.length * 0.5 + 60.0))};
	for (auto *section : sections)
	{
		for (auto const &cell : section->m_cells)
		{
			for (auto *track : cell.m_paths)
			{
				if (std::find(visited.begin(), visited.end(), track) != visited.end() || false == editor_track::is_straight(*track, state.tolerance))
					continue;
				auto other{editor_track::find_straight(*track, state.tolerance)};
				visited.insert(visited.end(), other.tracks.begin(), other.tracks.end());
				if (other.tracks.empty() || std::abs(line.direction.x * other.direction.y - line.direction.y * other.direction.x) > std::sin(glm::radians(kParallelFindAngle)))
					continue;
				auto const offset{glm::dvec2{(other.start + other.end).x * 0.5 - line.start.x, (other.start + other.end).z * 0.5 - line.start.z}};
				if (std::abs(glm::dot(offset, normal)) > 60.0)
					continue;
				auto const a0{line.along(other.start)};
				auto const a1{line.along(other.end)};
				if (std::max(a0, a1) <= 0.0 || std::min(a0, a1) >= line.length)
					continue;
				state.neighbours.push_back(std::move(other));
			}
		}
	}
}

editor_mode::parallel_offer editor_mode::straight_parallel_offer(editor_track::straight const &Line, glm::dvec3 const &Offset) const
{
	parallel_offer offer;
	glm::dvec2 const normal{-Line.direction.y, Line.direction.x};
	auto const middle{(Line.start + Line.end) * 0.5 + Offset};
	double best{kParallelHintWindow};
	for (auto const &other : m_straights.neighbours)
	{
		auto const sine{std::abs(Line.direction.x * other.direction.y - Line.direction.y * other.direction.x)};
		if (sine > std::sin(glm::radians(kParallelFindAngle)))
			continue;
		auto const distance{glm::dot(plan_of(other.start - middle), normal)};
		auto const angle{glm::degrees(std::abs(std::asin(std::clamp(sine, 0.0, 1.0))))};
		for (auto const spacing : m_straights.spacings)
		{
			auto const error{std::abs(std::abs(distance) - spacing)};
			if (error < best)
			{
				best = error;
				offer.nearby = true;
				offer.snaps = error < kParallelSnapWindow && angle <= kParallelSnapAngle;
				offer.spacing = spacing;
				offer.distance = std::abs(distance);
				offer.angle = angle;
				offer.correction = distance > 0.0 ? std::abs(distance) - spacing : spacing - std::abs(distance);
				offer.neighbour = &other;
			}
		}
	}
	return offer;
}

glm::dvec3 editor_mode::snap_straight_offset(editor_track::straight const &Line, glm::dvec3 const &Offset) const
{
	auto const offer{straight_parallel_offer(Line, Offset)};
	if (false == offer.snaps)
		return Offset;
	glm::dvec2 const normal{-Line.direction.y, Line.direction.x};
	return Offset + glm::dvec3{normal.x, 0.0, normal.y} * offer.correction;
}

glm::dvec3 editor_mode::snap_straight_direction(glm::dvec3 const &Pivot, glm::dvec3 const &Moved) const
{
	glm::dvec2 const offset{Moved.x - Pivot.x, Moved.z - Pivot.z};
	auto const length{glm::length(offset)};
	if (length < 1e-3)
		return Moved;
	auto const direction{offset / length};
	for (auto const &other : m_straights.neighbours)
	{
		auto const sine{direction.x * other.direction.y - direction.y * other.direction.x};
		if (std::abs(sine) > std::sin(glm::radians(kParallelSnapAngle)))
			continue;
		auto const aligned{other.direction * (glm::dot(direction, other.direction) < 0.0 ? -1.0 : 1.0)};
		return {Pivot.x + aligned.x * length, Moved.y, Pivot.z + aligned.y * length};
	}
	return Moved;
}

void editor_mode::render_straights_ui()
{
	auto &state{m_straights};
	ImGui::PushTextWrapPos(0.0f);
	ImGui::TextDisabled("%s", STR_C("Straights recognized from chains of collinear straight paths.\nLMB: select a path to see the straight it belongs to"));
	ImGui::PopTextWrapPos();
	ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.42f, 0.24f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.52f, 0.30f, 1.0f));
	if (ImGui::Button(STR_C("Recognize straights in the scenery"), ImVec2(-FLT_MIN, ImGui::GetFrameHeight() * 1.4f)))
	{
		state.found = editor_track::find_straights(state.minimum_length, state.tolerance);
		state.listed = -1;
		state.current_for = nullptr;
	}
	ImGui::PopStyleColor(2);
	if (ImGui::TreeNode(STR_C("What counts as one straight")))
	{
		if (begin_fields("##straighttolerance", 0.55f))
		{
			double angle{glm::degrees(state.tolerance.angle)};
			field_row("Angle tolerance (deg)");
			if (ImGui::InputDouble("##straightangle", &angle, 0.0, 0.0, "%.4f"))
				state.tolerance.angle = glm::radians(std::max(0.0, angle));
			item_tooltip("Largest change of the direction between the paths of one straight");
			field_row("Offset tolerance (m)");
			ImGui::InputDouble("##straightoffset", &state.tolerance.offset, 0.0, 0.0, "%.3f");
			item_tooltip("Largest distance of the paths from the common axis of the straight");
			field_row("Treat as straight above R (m)");
			ImGui::InputDouble("##straightradius", &state.tolerance.radius, 0.0, 0.0, "%.0f");
			item_tooltip("Curves of a larger radius count as straight");
			field_row("Minimum length (m)");
			ImGui::InputDouble("##straightminimum", &state.minimum_length, 0.0, 0.0, "%.0f");
			item_tooltip("Shorter straights aren't listed");
			ImGui::EndTable();
		}
		ImGui::TreePop();
	}
	if (false == state.found.empty())
	{
		ImGui::AlignTextToFramePadding();
		ImGui::Text(STR_C("%zu straights"), state.found.size());
		ImGui::SameLine();
		ImGui::SetNextItemWidth(-FLT_MIN);
		ImGui::InputTextWithHint("##straightfilter", STR_C("path name"), state.filter, sizeof(state.filter));
		ImGui::BeginChild("##straights", ImVec2(0.0f, 160.0f), true);
		for (int i = 0; i < static_cast<int>(state.found.size()); ++i)
		{
			auto const &line{state.found[i]};
			if (line.tracks.empty())
				continue;
			if (state.filter[0] != '\0' && std::none_of(line.tracks.begin(), line.tracks.end(), [&](TTrack const *Track) { return Track->name().find(state.filter) != std::string::npos; }))
				continue;
			auto const label{std::to_string(i + 1) + ".  " + describe(line) + "##straight" + std::to_string(i)};
			if (ImGui::Selectable(label.c_str(), state.listed == i))
			{
				state.listed = i;
				m_node = line.tracks[line.tracks.size() / 2];
				ui()->set_node(m_node);
				start_focus(m_node);
			}
		}
		ImGui::EndChild();
	}

}

void editor_mode::render_straight_ui()
{
	auto &state{m_straights};
	auto const &line{current_straight()};
	if (line.tracks.empty())
	{
		if (false == state.status.empty())
			ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", state.status.c_str());
		return;
	}
	metric_tiles({{format("%.2f m", line.length), STR_C("length"), ImGui::GetColorU32(ImGuiCol_Text)},
	              {format("%.2f\xc2\xb0", line.azimuth), STR_C("azimuth"), ImGui::GetColorU32(ImGuiCol_Text)},
	              {format("%+.2f \xe2\x80\xb0", line.grade * 1000.0), STR_C("grade"), ImGui::GetColorU32(ImGuiCol_Text)},
	              {format("%zu", line.tracks.size()), STR_C("paths"), ImGui::GetColorU32(ImGuiCol_TextDisabled)}});
	if (false == m_straights.neighbours.empty())
		ImGui::TextDisabled("%s", STR_C("Nearby straight: the diamond snaps parallel at 4.00 / 4.50 / 4.75 m"));

	// what else can be done with the straight, as tiles with a sketch of it
	auto const gap{std::round(ImGui::GetFontSize() * 0.3f)};
	auto const width{std::floor((ImGui::GetContentRegionAvail().x - 2.0f * gap) / 3.0f)};
	auto const toolbutton = [&](char const *Label, int const Tool, char const *Tooltip, std::function<void(ImDrawList &, ImVec2, ImVec2, ImU32, ImU32)> const &Sketch) {
		bool const active{state.tool == Tool};
		if (sketch_tile(Label, Label, active, width, Sketch))
		{
			state.tool = active ? 0 : Tool;
			state.tool_placed = false;
			state.handle = -1;
		}
		item_tooltip(Tooltip);
	};
	auto const at = [](ImVec2 const Low, ImVec2 const High, float const X, float const Y) { return ImVec2(Low.x + (High.x - Low.x) * X, Low.y + (High.y - Low.y) * Y); };
	toolbutton(STR_C("Break"), 1, STR_C("Turns the rest of the straight from a point, a curve goes at the break (Ctrl+drag in the view does the same)"), [&](ImDrawList &List, ImVec2 Low, ImVec2 High, ImU32 Line, ImU32 Made) {
		List.AddLine(at(Low, High, 0.0f, 0.8f), at(Low, High, 1.0f, 0.8f), (Line & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 60), 1.0f);
		List.AddLine(at(Low, High, 0.0f, 0.8f), at(Low, High, 0.4f, 0.8f), Line, 2.0f);
		List.AddBezierQuadratic(at(Low, High, 0.4f, 0.8f), at(Low, High, 0.62f, 0.8f), at(Low, High, 1.0f, 0.1f), Made, 2.0f);
	});
	ImGui::SameLine(0.0f, gap);
	toolbutton(STR_C("S-curve"), 2, STR_C("Shifts a piece of the straight sideways around an obstacle and back: four clicks on the straight (Shift+click in the view does the same)"), [&](ImDrawList &List, ImVec2 Low, ImVec2 High, ImU32 Line, ImU32 Made) {
		List.AddLine(at(Low, High, 0.0f, 0.8f), at(Low, High, 1.0f, 0.8f), (Line & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 60), 1.0f);
		List.AddLine(at(Low, High, 0.0f, 0.8f), at(Low, High, 0.15f, 0.8f), Line, 2.0f);
		List.AddBezierCubic(at(Low, High, 0.15f, 0.8f), at(Low, High, 0.3f, 0.8f), at(Low, High, 0.25f, 0.2f), at(Low, High, 0.42f, 0.2f), Made, 2.0f);
		List.AddLine(at(Low, High, 0.42f, 0.2f), at(Low, High, 0.58f, 0.2f), Made, 2.0f);
		List.AddBezierCubic(at(Low, High, 0.58f, 0.2f), at(Low, High, 0.75f, 0.2f), at(Low, High, 0.7f, 0.8f), at(Low, High, 0.85f, 0.8f), Made, 2.0f);
		List.AddLine(at(Low, High, 0.85f, 0.8f), at(Low, High, 1.0f, 0.8f), Line, 2.0f);
		List.AddCircleFilled(at(Low, High, 0.5f, 0.8f), 2.5f, IM_COL32(230, 90, 70, 255));
	});
	ImGui::SameLine(0.0f, gap);
	bool const member{in_straight_set(line)};
	auto const grouped{sketch_tile("##group", member ? STR_C("Leave the group") : STR_C("Move with others"), member, width, [&](ImDrawList &List, ImVec2 Low, ImVec2 High, ImU32 Line, ImU32 Made) {
		List.AddLine(at(Low, High, 0.0f, 0.2f), at(Low, High, 1.0f, 0.2f), Line, 2.0f);
		List.AddLine(at(Low, High, 0.0f, 0.8f), at(Low, High, 1.0f, 0.8f), Line, 2.0f);
		for (auto const x : {0.3f, 0.7f})
		{
			List.AddLine(at(Low, High, x, 0.32f), at(Low, High, x, 0.68f), Made, 1.5f);
			List.AddTriangleFilled(at(Low, High, x - 0.08f, 0.38f), at(Low, High, x + 0.08f, 0.38f), at(Low, High, x, 0.26f), Made);
			List.AddTriangleFilled(at(Low, High, x - 0.08f, 0.62f), at(Low, High, x + 0.08f, 0.62f), at(Low, High, x, 0.74f), Made);
		}
	})};
	if (grouped)
	{
		if (member)
			state.set.erase(std::remove_if(state.set.begin(), state.set.end(), [&](editor_track::straight const &Member) { return std::any_of(line.tracks.begin(), line.tracks.end(), [&](TTrack const *Track) { return std::find(Member.tracks.begin(), Member.tracks.end(), Track) != Member.tracks.end(); }); }), state.set.end());
		else
			state.set.push_back(line);
	}
	item_tooltip("Straights of a group move together and stay parallel; Alt+click in the view adds another one");
	if (state.tool != 0)
	{
		ImGui::TextDisabled("%s", STR_C("Curve R (m, 0: automatic)"));
		ImGui::SetNextItemWidth(-FLT_MIN);
		ImGui::InputDouble("##straightcurve", &state.curve_radius, 10.0, 100.0, "%.0f");
		state.curve_radius = std::max(0.0, state.curve_radius);
		item_tooltip("Radius of the curves made by the break and the S-curve; automatic: 1.5 times the minimum radius for the speed of the line");
		value_chips("straightcurves", state.curve_radius, {0.0, 300.0, 600.0, 1000.0, 2000.0}, "%.0f");
		ImGui::TextDisabled("%s", STR_C("Transition curves"));
		int transitions{state.auto_transitions ? 0 : 1};
		if (segmented("straighttransitions", transitions,
		              {m_norms_free ? segment_option{0, STR_C("None"), "Plain arcs: the free design has no transition curves, the PKP PLK norms work them out from the speed of the line"}
		                            : segment_option{0, STR_C("From the line speed"), "Lengths of the transition curves from the speed of the paths and the limits of the curve design"},
		               {1, STR_C("Given length"), "The same length of the transition curves at every curve"}}))
			state.auto_transitions = transitions == 0;
		if (false == state.auto_transitions)
		{
			ImGui::SetNextItemWidth(-FLT_MIN);
			ImGui::InputDouble("##straighttransition", &state.transition, 1.0, 10.0, "%.1f m");
			state.transition = std::max(0.0, state.transition);
		}
	}
	if (false == state.status.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", state.status.c_str());
	if (false == state.set.empty())
	{
		ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.15f, 1.0f), STR_C("Group: %zu straights move together"), state.set.size());
		ImGui::SameLine();
		if (ImGui::SmallButton(STR_C("Clear")))
			state.set.clear();
	}

	if (false == ImGui::IsAnyItemActive())
	{
		state.edit_start = line.start;
		state.edit_end = line.end;
		state.edit_length = line.length;
		state.edit_azimuth = line.azimuth;
	}
	if (ImGui::CollapsingHeader(STR_C("Exact values")))
	{
		if (begin_fields("##straightexact", 0.3f))
		{
			field_row("Start (x, y, z)");
			ImGui::InputScalarN("##straightstart", ImGuiDataType_Double, glm::value_ptr(state.edit_start), 3, nullptr, nullptr, "%.3f");
			field_row("End (x, y, z)");
			ImGui::InputScalarN("##straightend", ImGuiDataType_Double, glm::value_ptr(state.edit_end), 3, nullptr, nullptr, "%.3f");
			ImGui::EndTable();
		}
		if (ImGui::Button(STR_C("Apply ends"), ImVec2(-FLT_MIN, 0.0f)))
			straight_apply(line, state.edit_start, state.edit_end);
		if (begin_fields("##straightmeasures", 0.3f))
		{
			field_row("Length (m)");
			ImGui::InputDouble("##straightlength", &state.edit_length, 0.0, 0.0, "%.3f");
			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(1);
			if (ImGui::Button(STR_C("Set from the start##length"), ImVec2(-FLT_MIN, 0.0f)))
			{
				glm::dvec3 const direction{line.direction.x, line.grade, line.direction.y};
				straight_apply(line, line.start, line.start + direction * std::max(0.1, state.edit_length));
			}
			field_row("Azimuth (deg)");
			ImGui::InputDouble("##straightazimuth", &state.edit_azimuth, 0.0, 0.0, "%.5f");
			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(1);
			if (ImGui::Button(STR_C("Turn around the start"), ImVec2(-FLT_MIN, 0.0f)))
			{
				auto const azimuth{glm::radians(state.edit_azimuth)};
				glm::dvec3 const direction{std::sin(azimuth), line.grade, std::cos(azimuth)};
				straight_apply(line, line.start, line.start + direction * line.length);
			}
			ImGui::EndTable();
		}
	}
}

bool editor_mode::straights_active()
{
	return ui()->mode() == nodebank_panel::TRACK && m_track_tab == track_tab::straights && false == current_straight().tracks.empty();
}

int editor_mode::straight_handle_hit()
{
	if (false == straights_active())
		return -1;
	auto const &line{current_straight()};
	glm::dvec3 const handles[] = {line.start, line.end, (line.start + line.end) * 0.5};
	screen_projection const projection;
	float best = kHandleRadius * kHandleRadius;
	int hit{-1};
	for (int i = 0; i < 3; ++i)
	{
		if (auto const distance{projection.mouse_distance2(handles[i])}; distance < best)
		{
			best = distance;
			hit = i;
		}
	}
	return hit;
}

void editor_mode::straight_refresh()
{
	auto &state{m_straights};
	state.current_for = selected_track();
	state.current = state.current_for != nullptr ? editor_track::find_straight(*state.current_for, state.tolerance) : editor_track::straight{};
	find_neighbour_straights();
	for (auto &line : state.found)
		if (false == line.tracks.empty())
			line = editor_track::find_straight(*line.tracks.front(), state.tolerance);
	for (auto &line : state.set)
		if (false == line.tracks.empty())
			line = editor_track::find_straight(*line.tracks.front(), state.tolerance);
}

bool editor_mode::in_straight_set(editor_track::straight const &Line) const
{
	for (auto const &member : m_straights.set)
		for (auto const *track : Line.tracks)
			if (std::find(member.tracks.begin(), member.tracks.end(), track) != member.tracks.end())
				return true;
	return false;
}

void editor_mode::straight_apply(editor_track::straight const &Line, glm::dvec3 const &Start, glm::dvec3 const &End)
{
	straights_apply({Line}, {{Start, End}});
}

void editor_mode::straights_apply(std::vector<editor_track::straight> const &Lines, std::vector<std::pair<glm::dvec3, glm::dvec3>> const &Ends)
{
	auto &state{m_straights};
	state.status.clear();
	auto const finite = [](glm::dvec3 const &Point) { return std::isfinite(Point.x) && std::isfinite(Point.y) && std::isfinite(Point.z); };
	if (std::any_of(Ends.begin(), Ends.end(), [&](std::pair<glm::dvec3, glm::dvec3> const &End) {
		    return false == finite(End.first) || false == finite(End.second) || false == (glm::distance(End.first, End.second) >= 0.1 && glm::distance(End.first, End.second) <= 100000.0);
	    }))
	{
		state.status = refused(STR_C("The ends have to be numbers, from 0.1 m to 100 km apart"));
		return;
	}
	auto const tracks{editor_track::straight_affected(Lines)};
	if (false == tracks_editable(tracks, state.status))
		return;
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	for (auto *track : tracks)
		states.emplace_back(track, editor_track::capture(*track));
	push_track_snapshot(std::move(states));
	editor_track::move_straights(Lines, Ends);
	editor_track::commit(tracks);
	straight_refresh();
}

void editor_mode::render_straight_gizmo()
{
	auto &state{m_straights};
	if (state.tool_mouse)
	{
		glm::dvec3 const ground{cursor_ground()};
		state.tool_handle = {ground.x, state.tool_handle.y, ground.z};
		typed_turn();
		return;
	}
	auto const &line{state.dragging ? state.drag_line : current_straight()};
	if (state.handle < 0 || line.tracks.empty())
	{
		state.dragging = false;
		return;
	}

	glm::dvec3 const anchors[] = {line.start, line.end, (line.start + line.end) * 0.5};
	glm::dvec3 handled{anchors[state.handle]};
	bool const held{handle_drag_step(handled)};
	if (false == held && false == state.dragging)
		return;

	if (held)
	{
		if (false == state.dragging)
		{
			std::vector<editor_track::straight> lines;
			if (in_straight_set(line))
			{
				for (auto const &member : state.set)
					if (false == member.tracks.empty())
						lines.push_back(editor_track::find_straight(*member.tracks.front(), state.tolerance));
			}
			else
			{
				lines.push_back(line);
			}
			state.fitting = lines.size() == 1 && start_curve_fit(line, state.handle);
			auto const tracks{state.fitting ? std::vector<TTrack *>{} : editor_track::straight_affected(lines)};
			if (false == tracks_editable(tracks, state.status))
			{
				refused(state.status);
				state.fitting = false;
				return;
			}
			state.dragging = true;
			state.drag_line = line;
			state.drag_lines = lines;
			state.drag_states.clear();
			for (auto *track : tracks)
				state.drag_states.emplace_back(track, editor_track::capture(*track));
			if (false == state.fitting)
			{
				push_track_snapshot(state.drag_states);
				state.drag_step = m_history.size();
				m_track_drag = tracks;
			}
		}
		auto const &grabbed{state.drag_line};
		glm::dvec3 moved{handled};
		std::function<glm::dvec3(glm::dvec3 const &)> transform;
		if (state.handle == 2)
		{
			auto offset{snap_straight_offset(grabbed, moved - (grabbed.start + grabbed.end) * 0.5)};
			if (auto const values{typed_values()}; false == values.empty())
				offset = glm::dvec3{-grabbed.direction.y, 0.0, grabbed.direction.x} * values[0];
			transform = [offset](glm::dvec3 const &Point) { return Point + offset; };
		}
		else
		{
			auto const pivot{state.handle == 0 ? grabbed.end : grabbed.start};
			auto const original{state.handle == 0 ? grabbed.start : grabbed.end};
			moved = snap_straight_direction(pivot, moved);
			if (auto const values{typed_values()}; false == values.empty() && values[0] > 0.1)
			{
				glm::dvec2 heading{moved.x - pivot.x, moved.z - pivot.z};
				if (glm::length(heading) < 1e-6)
					heading = {original.x - pivot.x, original.z - pivot.z};
				heading = glm::normalize(heading) * values[0];
				moved.x = pivot.x + heading.x;
				moved.z = pivot.z + heading.y;
			}
			glm::dvec2 const before{original.x - pivot.x, original.z - pivot.z};
			glm::dvec2 const after{moved.x - pivot.x, moved.z - pivot.z};
			auto const lengthbefore{glm::length(before)};
			auto const lengthafter{std::max(0.1, glm::length(after))};
			auto const axis{before / std::max(1e-9, lengthbefore)};
			auto const newaxis{after / lengthafter};
			auto const scale{lengthafter / std::max(1e-9, lengthbefore)};
			auto const rise{moved.y - original.y};
			transform = [=](glm::dvec3 const &Point) {
				glm::dvec2 const offset{Point.x - pivot.x, Point.z - pivot.z};
				auto const along{glm::dot(offset, axis)};
				auto const across{axis.x * offset.y - axis.y * offset.x};
				auto const planar{plan_of(pivot) + newaxis * (along * scale) + glm::dvec2{-newaxis.y, newaxis.x} * across};
				return glm::dvec3{planar.x, Point.y + rise * along / std::max(1e-9, lengthbefore), planar.y};
			};
		}
		std::vector<std::pair<glm::dvec3, glm::dvec3>> ends;
		for (auto const &member : state.drag_lines)
			ends.emplace_back(transform(member.start), transform(member.end));
		state.preview_start = transform(grabbed.start);
		state.preview_end = transform(grabbed.end);
		if (state.fitting)
		{
			update_curve_fit(state.preview_start, state.preview_end);
			return;
		}
		for (auto const &entry : state.drag_states)
			editor_track::apply(*entry.first, entry.second);
		editor_track::move_straights(state.drag_lines, ends);
		m_track_dirty = true;
		commit_track_drag(false);
	}
	else if (state.dragging)
	{
		if (state.fitting)
		{
			state.fitting = false;
			state.dragging = false;
			state.drag_lines.clear();
			if (m_route.result.valid)
				route_apply();
			straight_refresh();
			return;
		}
		commit_track_drag(true);
		state.dragging = false;
		m_track_drag.clear();
		state.drag_states.clear();
		state.drag_lines.clear();
		straight_refresh();
	}
}

void editor_mode::cancel_track_tools()
{
	auto &state{m_straights};
	if (state.dragging)
	{
		// the geometry goes back to what it was before the drag
		if (false == state.fitting)
		{
			for (auto const &entry : state.drag_states)
				editor_track::apply(*entry.first, entry.second);
			m_track_dirty = true;
			commit_track_drag(true);
			m_track_drag.clear();
			drop_step(state.drag_step);
		}
		state.dragging = false;
		state.fitting = false;
		state.drag_states.clear();
		state.drag_lines.clear();
		straight_refresh();
	}
	m_extend.active = false;
	m_turntable.placing = false;
	m_signal.placing = false;
	m_signal.moving = false;
	m_handle_drag = {};
	if (m_route_dragging)
	{
		m_route_dragging = false;
		m_route.design = m_route_drag_from;
		route_update();
	}
	if (m_point_drag.active)
	{
		auto const step{m_point_drag.moved ? m_point_drag.step : 0};
		m_track_snap = {};
		m_record_watch = {};
		m_point_drag = {};
		m_track_drag.clear();
		m_track_drag_points.clear();
		m_track_joints.clear();
		drop_step(step);
	}
	if (m_switch.placing)
	{
		m_switch.placing = false;
		m_switch.status = STR_C("Placing of the switch cancelled");
	}
	else if (m_switch.armed >= 0)
	{
		m_switch.armed = -1;
		m_switch.status.clear();
	}
	m_track_point = {};
	m_route.vertex = -1;
	m_route.grip = -1;
	state.handle = -1;
	state.tool_placed = false;
	state.detour.clear();
}

void editor_mode::straight_reshape(editor_track::straight const &Line, double const From, double const To, std::function<glm::dvec3(glm::dvec3 const &)> const &Tail, double const Radius)
{
	auto &state{m_straights};
	state.status.clear();
	m_infra_carried.clear();
	struct span
	{
		TTrack *track;
		int path;
		double from;
		double to;
		bool forward;
	};
	auto line{Line};
	auto const point = [&](double const Along) {
		auto const planar{plan_of(line.start) + line.direction * Along};
		return glm::dvec3{planar.x, line.start.y + line.grade * Along, planar.y};
	};
	std::vector<span> spans;
	auto const collect = [&]() {
		spans.clear();
		for (std::size_t i = 0; i < line.tracks.size(); ++i)
		{
			auto const &path{line.tracks[i]->m_paths[line.paths[i]]};
			auto const a{line.along(path.points[segment_data::point::start])};
			auto const b{line.along(path.points[segment_data::point::end])};
			spans.push_back({line.tracks[i], line.paths[i], std::min(a, b), std::max(a, b), a < b});
		}
		std::sort(spans.begin(), spans.end(), [](span const &A, span const &B) { return A.from < B.from; });
	};
	int first{-1};
	int last{-1};
	auto const select = [&](double const End) {
		first = -1;
		last = -1;
		for (int i = 0; i < static_cast<int>(spans.size()); ++i)
		{
			if (spans[i].to > From && spans[i].from < End)
			{
				if (first < 0)
					first = i;
				last = i;
			}
		}
	};
	auto const keep_regular = [&]() {
		if (first < 0)
			return;
		auto const centre{(std::max(From, 0.0) + std::min(To, line.length)) * 0.5};
		int middle{first};
		for (int i = first; i <= last; ++i)
			if (spans[i].from <= centre && spans[i].to >= centre)
				middle = i;
		if (spans[middle].track->eType != tt_Normal)
		{
			int best{-1};
			for (int i = first; i <= last; ++i)
				if (spans[i].track->eType == tt_Normal && (best < 0 || std::abs(i - middle) < std::abs(best - middle)))
					best = i;
			if (best < 0)
			{
				first = -1;
				last = -1;
				return;
			}
			middle = best;
		}
		int low{middle};
		int high{middle};
		while (low > first && spans[low - 1].track->eType == tt_Normal)
			--low;
		while (high < last && spans[high + 1].track->eType == tt_Normal)
			++high;
		first = low;
		last = high;
	};
	collect();
	select(To);
	keep_regular();
	if (first < 0)
		return;

	std::vector<std::pair<TTrack *, editor_track::state>> splitstates;
	std::vector<TTrack *> splitcreated;
	auto end{To};
	if (last + 1 >= static_cast<int>(spans.size()))
	{
		auto const final{spans.back()};
		if (final.track->eType != tt_Normal)
			return;
		auto const length{final.to - final.from};
		auto const cut{To < final.to - 1.0 ? std::max(To, final.from + std::min(1.0, length * 0.5)) : final.from + length * 0.7};
		splitstates.emplace_back(final.track, editor_track::capture(*final.track));
		editor_track::chain split;
		split.tracks = {final.track};
		split.forward = {final.forward};
		std::vector<segment_data> pieces(2);
		pieces[0].points[segment_data::point::start] = point(final.from);
		pieces[0].points[segment_data::point::end] = point(cut);
		pieces[1].points[segment_data::point::start] = point(cut);
		pieces[1].points[segment_data::point::end] = point(final.to);
		splitcreated = editor_track::relay(split, pieces);
		line = editor_track::find_straight(*line.tracks.front(), state.tolerance);
		collect();
		end = std::min(To, cut);
		select(end);
		keep_regular();
		if (first < 0 || last + 1 >= static_cast<int>(spans.size()))
		{
			for (auto *track : splitcreated)
				editor_track::retire(*track);
			editor_track::apply(*splitstates.front().first, splitstates.front().second);
			editor_track::commit({splitstates.front().first});
			return;
		}
	}
	auto const undo_split = [&]() {
		for (auto *track : splitcreated)
			editor_track::retire(*track);
		for (auto const &entry : splitstates)
		{
			editor_track::apply(*entry.first, entry.second);
			editor_track::commit({entry.first});
		}
	};

	editor_track::straight tail;
	tail.direction = line.direction;
	for (int i = last + 1; i < static_cast<int>(spans.size()); ++i)
	{
		tail.tracks.push_back(spans[i].track);
		tail.paths.push_back(spans[i].path);
	}
	tail.start = point(spans[last + 1].from);
	tail.end = line.end;
	tail.length = line.length - spans[last + 1].from;
	tail.grade = line.grade;

	std::vector<TTrack *> chainpaths;
	for (int i = first; i <= last; ++i)
		chainpaths.push_back(spans[i].track);
	auto tracks{editor_track::straight_affected({tail})};
	for (auto *track : chainpaths)
		if (std::find(tracks.begin(), tracks.end(), track) == tracks.end())
			tracks.push_back(track);
	if (false == tracks_editable(tracks, m_straights.status))
	{
		undo_split();
		return;
	}
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	for (auto *track : tracks)
		states.emplace_back(track, editor_track::capture(*track));

	editor_track::move_straights({tail}, {{Tail(tail.start), Tail(tail.end)}});

	m_route.from = spans[first].track;
	m_route.to = spans[last].track;
	route_reset();
	if (m_route.chain.tracks.empty() || false == m_route.chain.switches.empty())
	{
		if (false == m_route.chain.switches.empty())
			m_route.error = STR_C("The curve would run through a switch");
		for (auto const &entry : states)
			editor_track::apply(*entry.first, entry.second);
		editor_track::commit(tracks);
		undo_split();
		state.status = refused(m_route.error);
		return;
	}
	m_route.design.start_reserve = 0.0;
	m_route.design.end_reserve = 0.0;
	for (auto &vertex : m_route.design.vertices)
	{
		vertex.radius = Radius;
		route_recommend(vertex);
		if (false == state.auto_transitions)
		{
			vertex.transition_in = state.transition;
			vertex.transition_out = state.transition;
		}
	}
	route_update();
	if (false == m_route.result.valid)
	{
		for (auto const &entry : states)
			editor_track::apply(*entry.first, entry.second);
		editor_track::commit(tracks);
		undo_split();
		state.status = refused(m_route.result.errors.empty() ? STR_C("The curve can't be fitted") : m_route.result.errors.front());
		return;
	}
	std::vector<std::size_t> intervals;
	auto pieces{alignment::pieces_at(m_route.result, m_route.design, m_route.chain.tracks.size(), editor_track::chain_joints(m_route.chain), intervals)};
	editor_track::keep_heights(m_route.chain, pieces);
	auto created{editor_track::relay(m_route.chain, pieces, &m_route.to, &intervals)};
	editor_track::commit(tracks);
	for (auto const &entry : splitstates)
	{
		auto const found{std::find_if(states.begin(), states.end(), [&](auto const &State) { return State.first == entry.first; })};
		if (found != states.end())
			found->second = entry.second;
		else
			states.push_back(entry);
	}
	created.insert(created.begin(), splitcreated.begin(), splitcreated.end());
	{
		auto changed{created};
		for (auto const &entry : states)
			changed.push_back(entry.first);
		digitize_drawn(changed);
	}
	push_track_snapshot(std::move(states), std::move(created));
	std::string error;
	editor_track::find_chain(m_route.from, m_route.to, m_route.chain, error, true);
	straight_refresh();
	state.status = STR_C("Done: click a curve to adjust it; Ctrl+Z takes it back") + infra_carried_note();
}

double editor_mode::straight_tool_radius(editor_track::straight const &Line) const
{
	if (m_straights.curve_radius > 0.0)
		return m_straights.curve_radius;
	double speed{-1.0};
	for (auto const *track : Line.tracks)
		speed = std::max(speed, editor_track::velocity(*track));
	if (speed <= 0.0)
		speed = 100.0;
	auto const recommended{alignment::recommend(speed, 1000.0, m_route.design.norms)};
	return std::max(150.0, std::ceil(recommended.radius_min * 1.5 / 50.0) * 50.0);
}

bool editor_mode::place_straight_tool()
{
	auto &state{m_straights};
	if (ui()->mode() != nodebank_panel::TRACK || m_track_tab != track_tab::straights || state.tool == 0)
		return false;
	if (state.tool == 2)
	{
		add_detour_point();
		return true;
	}
	auto const &line{current_straight()};
	if (line.tracks.empty())
		return false;
	glm::dvec3 const ground{cursor_ground()};
	auto const along{std::clamp(line.along(ground), 0.0, line.length)};
	glm::dvec3 const axis{line.direction.x, line.grade, line.direction.y};
	state.tool_line = line;
	state.tool_at = along;
	state.tool_point = line.start + axis * along;
	auto const reach{std::min(50.0, std::max(1.0, line.length - along))};
	state.tool_handle = state.tool_point + axis * reach;
	state.tool_placed = true;
	state.handle = -1;
	return true;
}

void editor_mode::apply_straight_tool()
{
	auto &state{m_straights};
	auto const line{state.tool_line};
	auto const offset{plan_of(state.tool_handle - state.tool_point)};
	auto const radius{straight_tool_radius(line)};
	if (state.tool == 1)
	{
		if (glm::length(offset) < 1e-3)
			return;
		auto const unit{glm::normalize(offset)};
		auto const angle{std::atan2(line.direction.x * unit.y - line.direction.y * unit.x, glm::dot(line.direction, unit))};
		if (std::abs(angle) < 1e-5)
			return;
		glm::dvec2 const pivot{state.tool_point.x, state.tool_point.z};
		double speed{-1.0};
		for (auto const *track : line.tracks)
			speed = std::max(speed, editor_track::velocity(*track));
		auto const transition{state.auto_transitions ? alignment::recommend(speed > 0.0 ? speed : 100.0, radius, laying_norms()).transition : state.transition};
		auto const reach{radius * std::tan(std::min(std::abs(angle), 3.0) * 0.5) + transition * 0.5 + 10.0};
		straight_reshape(line, state.tool_at - reach, state.tool_at + reach,
			[pivot, angle](glm::dvec3 const &Point) {
				glm::dvec2 const relative{Point.x - pivot.x, Point.z - pivot.y};
				auto const c{std::cos(angle)};
				auto const s{std::sin(angle)};
				return glm::dvec3{pivot.x + relative.x * c - relative.y * s, Point.y, pivot.y + relative.x * s + relative.y * c};
			},
			radius);
	}
	else
	{
		glm::dvec2 const normal{-line.direction.y, line.direction.x};
		auto const shift{glm::dot(offset, normal)};
		auto const length{glm::dot(offset, line.direction)};
		if (std::abs(shift) < 1e-3 || length < 1.0)
			return;
		auto const translation{normal * shift};
		straight_reshape(line, state.tool_at, state.tool_at + length,
			[translation](glm::dvec3 const &Point) { return glm::dvec3{Point.x + translation.x, Point.y, Point.z + translation.y}; },
			radius);
	}
	state.tool_placed = false;
}

void editor_mode::start_straight_gesture(int const Tool)
{
	auto &state{m_straights};
	state.tool_before = state.tool;
	state.tool = Tool;
	if (false == place_straight_tool())
	{
		state.tool = state.tool_before;
		return;
	}
	state.tool_mouse = true;
}

void editor_mode::finish_straight_gesture()
{
	auto &state{m_straights};
	state.tool_mouse = false;
	apply_straight_tool();
	state.tool = state.tool_before;
	state.tool_placed = false;
}

void editor_mode::toggle_straight_set(TTrack &Track)
{
	auto &state{m_straights};
	auto const line{editor_track::find_straight(Track, state.tolerance)};
	if (line.tracks.empty())
		return;
	if (in_straight_set(line))
		state.set.erase(std::remove_if(state.set.begin(), state.set.end(), [&](editor_track::straight const &Member) { return std::any_of(line.tracks.begin(), line.tracks.end(), [&](TTrack const *Other) { return std::find(Member.tracks.begin(), Member.tracks.end(), Other) != Member.tracks.end(); }); }), state.set.end());
	else
		state.set.push_back(line);
	m_node = &Track;
	ui()->set_node(m_node);
	m_track_tab = track_tab::straights;
}

std::vector<glm::dvec3> editor_mode::detour_outline() const
{
	auto const &state{m_straights};
	std::vector<glm::dvec3> result;
	if (state.detour.empty())
		return result;
	auto const &line{state.detour_line};
	glm::dvec2 const normal{-line.direction.y, line.direction.x};
	auto points{state.detour};
	if (points.size() < 4)
		points.push_back(state.detour_mouse);
	auto const offset{points.size() >= 2 ? glm::dot(plan_of(points[1] - line.start), normal) : 0.0};
	auto const place = [&](double const Along, double const Lateral) {
		auto const planar{plan_of(line.start) + line.direction * Along + normal * Lateral};
		return glm::dvec3{planar.x, line.start.y + line.grade * Along, planar.y};
	};
	for (std::size_t i = 0; i < points.size(); ++i)
		result.push_back(place(line.along(points[i]), (i == 1 || i == 2) ? offset : 0.0));
	return result;
}

void editor_mode::add_detour_point()
{
	auto &state{m_straights};
	if (state.detour.empty())
	{
		auto const &line{current_straight()};
		if (line.tracks.empty())
			return;
		state.detour_line = line;
	}
	state.detour.push_back(cursor_ground());
	if (state.detour.size() >= 4)
	{
		apply_detour();
		state.detour.clear();
	}
}

void editor_mode::apply_detour()
{
	auto &state{m_straights};
	auto line{editor_track::find_straight(*state.detour_line.tracks.front(), state.tolerance)};
	if (line.tracks.empty())
		return;
	glm::dvec2 const normal{-line.direction.y, line.direction.x};
	auto const offset{glm::dot(plan_of(state.detour[1] - line.start), normal)};
	std::array<double, 4> stations{line.along(state.detour[0]), line.along(state.detour[1]), line.along(state.detour[2]), line.along(state.detour[3])};
	if (stations[0] > stations[3])
	{
		std::swap(stations[0], stations[3]);
		std::swap(stations[1], stations[2]);
	}
	for (auto &station : stations)
		station = std::clamp(station, 0.0, line.length);
	stations[1] = std::clamp(stations[1], stations[0] + 1.0, stations[3] - 1.0);
	stations[2] = std::clamp(stations[2], stations[1], stations[3] - 1.0);
	if (std::abs(offset) < 0.01 || stations[3] - stations[0] < 3.0)
		return;

	auto const radius{straight_tool_radius(line)};
	auto const slope{std::atan(std::abs(offset) / std::max(1.0, std::min(stations[1] - stations[0], stations[3] - stations[2])))};
	double speed{-1.0};
	for (auto const *track : line.tracks)
		speed = std::max(speed, editor_track::velocity(*track));
	auto const transition{state.auto_transitions ? alignment::recommend(speed > 0.0 ? speed : 100.0, radius, laying_norms()).transition : state.transition};
	auto const reach{radius * std::tan(slope * 0.5) + transition * 0.5 + 5.0};
	auto const from{std::max(0.0, stations[0] - reach)};
	auto const to{std::min(line.length, stations[3] + reach)};

	struct span
	{
		TTrack *track;
		double from;
		double to;
	};
	std::vector<span> spans;
	for (std::size_t i = 0; i < line.tracks.size(); ++i)
	{
		auto const &path{line.tracks[i]->m_paths[line.paths[i]]};
		auto const a{line.along(path.points[segment_data::point::start])};
		auto const b{line.along(path.points[segment_data::point::end])};
		spans.push_back({line.tracks[i], std::min(a, b), std::max(a, b)});
	}
	std::sort(spans.begin(), spans.end(), [](span const &A, span const &B) { return A.from < B.from; });
	int first{-1};
	int last{-1};
	for (int i = 0; i < static_cast<int>(spans.size()); ++i)
	{
		if (spans[i].to > from && spans[i].from < to && spans[i].track->eType == tt_Normal)
		{
			if (first < 0)
				first = i;
			last = i;
		}
		else if (first >= 0 && spans[i].from < to)
		{
			if (spans[i].from > stations[3])
				break;
			first = -1;
			last = -1;
		}
	}
	if (first < 0)
		return;

	m_route.from = spans[first].track;
	m_route.to = spans[last].track;
	route_reset();
	if (m_route.chain.tracks.empty())
		return;
	auto &design{m_route.design};
	auto const startalong{line.along(design.start)};
	auto const endalong{line.along(design.end)};
	auto const place = [&](double const Along, double const Lateral) {
		auto const planar{plan_of(line.start) + line.direction * Along + normal * Lateral};
		return planar;
	};
	alignment::vertex vertex;
	vertex.radius = radius;
	route_recommend(vertex);
	if (false == state.auto_transitions)
	{
		vertex.transition_out = state.transition;
		vertex.transition_in = vertex.transition_out;
	}
	design.vertices.clear();
	auto first_vertex{vertex};
	first_vertex.offset = std::max(0.1, stations[0] - startalong);
	design.vertices.push_back(first_vertex);
	if (stations[2] - stations[1] < 1.0)
	{
		auto middle{vertex};
		middle.position = place((stations[1] + stations[2]) * 0.5, offset);
		design.vertices.push_back(middle);
	}
	else
	{
		auto second{vertex};
		second.position = place(stations[1], offset);
		auto third{vertex};
		third.position = place(stations[2], offset);
		design.vertices.push_back(second);
		design.vertices.push_back(third);
	}
	auto last_vertex{vertex};
	last_vertex.offset = std::max(0.1, endalong - stations[3]);
	design.vertices.push_back(last_vertex);
	route_update();
	if (false == m_route.result.valid)
		return;
	route_apply();
	straight_refresh();
}

bool editor_mode::start_curve_fit(editor_track::straight const &Line, int const Handle)
{
	auto &state{m_straights};
	auto *before{editor_track::outside_neighbour(Line, false)};
	auto *after{editor_track::outside_neighbour(Line, true)};
	auto const curve_at = [&](TTrack *Neighbour, editor_track::curve &Curve) {
		Curve = {};
		return Neighbour != nullptr && Neighbour->eType == tt_Normal && editor_track::find_curve(*Neighbour, state.tolerance, m_route.design.norms.gauge, Curve) && Curve.reversals == 0 && std::abs(Curve.turn) > 1e-4;
	};
	state.fit_has_before = curve_at(before, state.fit_before);
	state.fit_has_after = curve_at(after, state.fit_after);
	if (false == state.fit_has_before && false == state.fit_has_after)
		return false;
	auto const side_follows = [&](bool const Curve, TTrack const *Neighbour, bool const Pivot) { return Curve || Neighbour == nullptr || Pivot; };
	if (false == side_follows(state.fit_has_before, before, Handle == 1) || false == side_follows(state.fit_has_after, after, Handle == 0))
		return false;
	auto const own_end = [&](glm::dvec3 const &Point) -> TTrack * {
		for (auto *track : Line.tracks)
			if (editor_track::touches(*track, Point))
				return track;
		return Line.tracks.empty() ? nullptr : Line.tracks.front();
	};
	auto const other_end = [](editor_track::curve const &Curve, glm::dvec3 const &Joint) { return editor_track::touches(*Curve.from, Joint) ? Curve.to : Curve.from; };
	m_route.from = state.fit_has_before ? other_end(state.fit_before, Line.start) : own_end(Line.start);
	m_route.to = state.fit_has_after ? other_end(state.fit_after, Line.end) : own_end(Line.end);
	route_reset();
	if (m_route.chain.tracks.empty())
		return false;
	return true;
}

void editor_mode::update_curve_fit(glm::dvec3 const &Start, glm::dvec3 const &End)
{
	auto &state{m_straights};
	auto &design{m_route.design};
	glm::dvec2 const point{Start.x, Start.z};
	glm::dvec2 const direction{glm::normalize(plan_of(End - Start))};
	glm::dvec2 const start{design.start.x, design.start.z};
	glm::dvec2 const end{design.end.x, design.end.z};
	auto const startdirection{glm::normalize(design.start_direction)};
	auto const enddirection{glm::normalize(design.end_direction)};
	auto const first{cross(point - start, direction) / cross(startdirection, direction)};
	auto const last{cross(end - point, direction) / cross(enddirection, direction)};
	auto const make = [&](editor_track::curve const &Curve, double const Offset) {
		alignment::vertex vertex;
		vertex.radius = Curve.radius > 0.0 ? std::round(Curve.radius) : 1000.0;
		route_recommend(vertex);
		if (Curve.transition_in > 0.0)
			vertex.transition_in = std::round(Curve.transition_in);
		if (Curve.transition_out > 0.0)
			vertex.transition_out = std::round(Curve.transition_out);
		if (Curve.cant > 0.0)
			vertex.cant = std::round(Curve.cant);
		compound_from(Curve, vertex);
		vertex.offset = std::max(0.1, Offset);
		return vertex;
	};
	design.vertices.clear();
	if (state.fit_has_before)
		design.vertices.push_back(make(state.fit_before, first));
	else
	{
		design.start = Start;
		design.start_direction = direction;
		design.start_reserve = 0.0;
	}
	if (state.fit_has_after)
		design.vertices.push_back(make(state.fit_after, last));
	else
	{
		design.end = End;
		design.end_direction = direction;
		design.end_reserve = 0.0;
	}
	route_update();
}
