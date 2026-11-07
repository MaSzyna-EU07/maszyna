/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormodeparallel.h"
#include "application/editorprojection.h"
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"

#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include "utilities/translation.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace
{

using geometry::bezier;
using geometry::cross;
using geometry::plan_of;
using geometry::signed_angle;

double const kParallelReach{5000.0}; // m each way along the line
double const kSpacings[] = {3.5, 4.0, 4.5, 4.75, 5.0, 6.0};

// the row of the side picker under the mouse: -1 for the new track on the left, 1 on the right, 0 for the line or none
int pointed_side(bool const Hovered, float const Mouse, float const Top, float const Row)
{
	if (false == Hovered)
		return 0;
	if (Mouse < Top + Row)
		return -1;
	return Mouse >= Top + Row * 2.0f ? 1 : 0;
}

// where the side picker draws: the top of its picture, the height of a row, the right edge of the names of the rows, the
// ends of the tracks and their gauge
struct picker_layout
{
	ImDrawList *list{nullptr};
	float top{0.0f};
	float font{0.0f};
	float row{0.0f};
	float names{0.0f};
	float track_left{0.0f};
	float track_right{0.0f};
	float gauge{0.0f};
};

// a track of the side picker as two rails, sleepers under the solid ones
void draw_picker_track(picker_layout const &Layout, float const Y, ImU32 const Colour, bool const Dashed)
{
	auto *list{Layout.list};
	auto const font{Layout.font};
	auto const from{Layout.track_left};
	auto const to{Layout.track_right};
	auto const gauge{Layout.gauge};
	// the sleepers a whole number of pixels apart, never less than one
	auto const sleeper{std::max(1.0f, std::round(font * 0.9f))};
	if (false == Dashed)
		for (int k = 0; from + font * 0.3f + sleeper * static_cast<float>(k) < to; ++k)
		{
			auto const x{from + font * 0.3f + sleeper * static_cast<float>(k)};
			list->AddLine(ImVec2(x, Y - gauge * 0.95f), ImVec2(x, Y + gauge * 0.95f), (Colour & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 90), 2.0f);
		}
	for (auto const rail : {Y - gauge * 0.5f, Y + gauge * 0.5f})
	{
		if (false == Dashed)
		{
			list->AddLine(ImVec2(from, rail), ImVec2(to, rail), Colour, 1.5f);
			continue;
		}
		for (int k = 0; from + font * static_cast<float>(k) < to; ++k)
		{
			auto const x{from + font * static_cast<float>(k)};
			list->AddLine(ImVec2(x, rail), ImVec2(std::min(to, x + font * 0.55f), rail), Colour, 1.0f);
		}
	}
}

// the name of a row of the side picker, right aligned to the names
void draw_picker_label(picker_layout const &Layout, int const Row, char const *Label, bool const Lit)
{
	auto const size{ImGui::CalcTextSize(Label)};
	Layout.list->AddText(ImVec2(Layout.names - size.x, std::round(Layout.top + Layout.row * (static_cast<float>(Row) + 0.5f) - size.y * 0.5f)), ImGui::GetColorU32(Lit ? ImGuiCol_Text : ImGuiCol_TextDisabled), Label);
}

// the spacing, axis to axis: from the line at Middle to the new track at Y, on the side of Sign
void draw_picker_spacing(picker_layout const &Layout, float const Middle, float const Y, float const Sign, double const Spacing)
{
	auto *list{Layout.list};
	auto const x{std::round(Layout.track_left + (Layout.track_right - Layout.track_left) * 0.62f)}; // NOSONAR: std::lerp rounds differently where the ends lie either side of 0 (a panel across the left edge of the main monitor)
	ImU32 const dimension{IM_COL32(255, 210, 60, 255)};
	list->AddLine(ImVec2(x, Middle), ImVec2(x, Y), dimension, 1.5f);
	list->AddTriangleFilled(ImVec2(x, Middle), ImVec2(x - 3.0f, Middle + Sign * 6.0f), ImVec2(x + 3.0f, Middle + Sign * 6.0f), dimension);
	list->AddTriangleFilled(ImVec2(x, Y), ImVec2(x - 3.0f, Y - Sign * 6.0f), ImVec2(x + 3.0f, Y - Sign * 6.0f), dimension);
	auto const text{format("%.2f m", Spacing)};
	auto const size{ImGui::CalcTextSize(text.c_str())};
	ImVec2 const at{x + 6.0f, std::round((Middle + Y) * 0.5f - size.y * 0.5f)};
	list->AddRectFilled(ImVec2(at.x - 3.0f, at.y), ImVec2(at.x + size.x + 3.0f, at.y + size.y), ImGui::GetColorU32(ImGuiCol_FrameBg), 3.0f);
	list->AddText(at, dimension, text.c_str());
}

// the side of the new track as a picture: three rows, the new track on the left of the line, the line with its direction,
// the new track on the right; the side chosen drawn with the spacing marked, the other faint. a click on a row chooses its side
bool side_picker(int &Side, double const Spacing)
{
	auto const font{ImGui::GetFontSize()};
	auto const width{ImGui::GetContentRegionAvail().x};
	auto const row{std::round(font * 1.7f)};
	auto const height{row * 3.0f};
	auto const top{ImGui::GetCursorScreenPos()};
	ImGui::InvisibleButton("##side", ImVec2(width, height));
	auto const hovered{ImGui::IsItemHovered()};
	auto const mouse{ImGui::GetIO().MousePos};
	auto const middle{std::round(top.y + row * 1.5f)};
	auto const pointed{pointed_side(hovered, mouse.y, top.y, row)};
	bool changed{false};
	if (ImGui::IsItemClicked() && pointed != 0 && pointed != Side)
	{
		Side = pointed;
		changed = true;
	}
	// the other side under the mouse, which a click takes
	bool const other_pointed{pointed != 0 && pointed != Side};
	auto *list{ImGui::GetWindowDrawList()};
	ImVec2 const bottom{top.x + width, top.y + height};
	auto const rounding{std::round(font * 0.35f)};
	list->AddRectFilled(top, bottom, ImGui::GetColorU32(ImGuiCol_FrameBg), rounding);
	if (other_pointed)
		list->AddRectFilled(ImVec2(top.x, pointed < 0 ? top.y : top.y + row * 2.0f), ImVec2(bottom.x, pointed < 0 ? top.y + row : bottom.y), ImGui::GetColorU32(ImGuiCol_FrameBgHovered), rounding);
	auto const pad{std::round(font * 0.6f)};
	std::array<char const *, 3> const labels{STR_C("Left"), STR_C("selected"), STR_C("Right")};
	float column{0.0f};
	for (auto const *label : labels)
		column = std::max(column, ImGui::CalcTextSize(label).x);
	auto const track_left{top.x + pad + column + pad};
	auto const track_right{bottom.x - pad * 2.0f};
	picker_layout const layout{.list = list, .top = top.y, .font = font, .row = row, .names = top.x + pad + column, .track_left = track_left, .track_right = track_right, .gauge = std::round(font * 0.36f)};
	auto const line{ImGui::GetColorU32(ImGuiCol_Text, 0.85f)};
	draw_picker_track(layout, middle, line, false);
	auto const arrow{std::round(font * 0.45f)};
	list->AddTriangleFilled(ImVec2(track_right + pad * 1.2f, middle), ImVec2(track_right + pad * 0.2f - arrow * 0.4f, middle - arrow), ImVec2(track_right + pad * 0.2f - arrow * 0.4f, middle + arrow), line);
	draw_picker_label(layout, 1, labels[1], false);
	auto const accent{family_colour(0)};
	for (auto const side : {-1, 1})
	{
		auto const sign{static_cast<float>(side)};
		auto const y{middle + sign * row};
		auto const chosen{side == Side};
		draw_picker_track(layout, y, chosen ? accent : ImGui::GetColorU32(ImGuiCol_TextDisabled, 0.6f), false == chosen);
		auto const label_row{side < 0 ? 0 : 2};
		draw_picker_label(layout, label_row, labels[static_cast<std::size_t>(label_row)], chosen);
		if (chosen)
			draw_picker_spacing(layout, middle, y, sign, Spacing);
	}
	if (hovered)
		ImGui::SetTooltip("%s", other_pointed ? STR_C("A click: the new track on this side") : STR_C("Left and right as seen along the line, the way the arrow points"));
	return changed;
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

std::optional<double> arc_curvature(segment_data const &Path)
{
	auto const [control1, control2] = controls(Path);
	auto const along{plan_of(control1)};
	auto const back{-plan_of(control2)};
	if (glm::length(along) < 1e-9 || glm::length(back) < 1e-9)
		return std::nullopt;
	auto const turn{signed_angle(glm::normalize(along), glm::normalize(back))};
	if (std::abs(turn) < 1e-6)
		return std::nullopt;
	auto const radius{glm::length(plan_of(Path.points[segment_data::point::end] - Path.points[segment_data::point::start])) / (2.0 * std::sin(std::abs(turn) * 0.5))};
	// the length of the control vectors of an arc of the turn
	if (auto const arm{4.0 / 3.0 * std::tan(std::abs(turn) * 0.25) * radius}; std::abs(glm::length(along) - arm) > 1e-4 * arm || std::abs(glm::length(back) - arm) > 1e-4 * arm)
		return std::nullopt;
	return std::copysign(1.0 / radius, turn);
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

using path_sources = std::vector<std::pair<TTrack *, segment_data>>;

// the paths the new track goes beside, each the way the line runs: the line through the track up to the switches (scope 1),
// else the track alone
path_sources line_sources(TTrack &Track, int const Scope)
{
	path_sources sources;
	if (Scope == 1)
	{
		editor_track::chain run;
		std::string error;
		if (editor_track::find_run(Track, kParallelReach, run, error))
			for (std::size_t i = 0; i < run.tracks.size(); ++i)
				sources.emplace_back(run.tracks[i], run.forward[i] ? run.tracks[i]->m_paths.front() : reversed(run.tracks[i]->m_paths.front()));
	}
	if (sources.empty())
		sources.emplace_back(&Track, Track.m_paths.front());
	return sources;
}

// the joint before the path K (the end of the line for K past the last) moved by the offset, along the mean direction of the
// paths meeting there
glm::dvec3 shifted_joint(path_sources const &Sources, std::size_t const K, double const Offset)
{
	bool const first{K == 0};
	bool const last{K == Sources.size()};
	auto const &before{Sources[first ? 0 : K - 1].second};
	auto const &after{Sources[last ? K - 1 : K].second};
	auto const point{first ? after.points[segment_data::point::start] : before.points[segment_data::point::end]};
	glm::dvec2 heading;
	float roll;
	if (first)
	{
		heading = direction(after, false);
		roll = after.rolls[0];
	}
	else if (last)
	{
		heading = direction(before, true);
		roll = before.rolls[1];
	}
	else
	{
		heading = direction(before, true) + direction(after, false);
		roll = 0.5f * (before.rolls[1] + after.rolls[0]);
	}
	heading = glm::length(heading) > 1e-9 ? glm::normalize(heading) : glm::dvec2{0.0, 1.0};
	glm::dvec2 const left_normal{-heading.y, heading.x};
	auto const planar{plan_of(point) + left_normal * Offset};
	return {planar.x, point.y - Offset * std::tan(glm::radians(static_cast<double>(roll))), planar.y};
}

// the path beside the source between the joints moved by the offset, its curves along with R minus or plus the offset; none
// where the offset is larger than the radius
std::optional<segment_data> offset_piece(segment_data const &Source, glm::dvec3 const &Start, glm::dvec3 const &End, double const Offset)
{
	auto const [control1, control2] = controls(Source);
	auto const arc{arc_curvature(Source)};
	auto const startscale{1.0 - (arc.has_value() ? *arc : curvature(Source, false)) * Offset};
	auto const endscale{1.0 - (arc.has_value() ? *arc : curvature(Source, true)) * Offset};
	if (startscale < 0.05 || endscale < 0.05)
		return std::nullopt;
	segment_data piece;
	piece.points[segment_data::point::start] = Start;
	piece.points[segment_data::point::end] = End;
	piece.points[segment_data::point::control1] = control1 * startscale;
	piece.points[segment_data::point::control2] = control2 * endscale;
	piece.rolls = Source.rolls;
	auto const middle{0.5 * (curvature(Source, false) + curvature(Source, true))};
	if (Source.radius != 0.0f && std::abs(middle) > 1e-7)
		piece.radius = static_cast<float>(std::copysign(std::abs(std::abs(static_cast<double>(Source.radius)) - std::copysign(Offset, middle)), static_cast<double>(Source.radius)));
	return piece;
}

} // namespace

TTrack *parallel_tool::parallel_source() const
{
	if (auto const &set{m_selection.track_set()}; set.size() >= 2)
		return set.front();
	return m_selection.selected_track();
}

TTrack *parallel_tool::parallel_target() const
{
	auto const &set{m_selection.track_set()};
	if (set.size() != 2)
		return nullptr;
	return set.back() != set.front() ? set.back() : nullptr;
}

void parallel_tool::parallel_update()
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

	if (track == tool.built_for && tool.spacing == tool.built_spacing && tool.side == tool.built_side && tool.scope == tool.built_scope && m_history.history_revision() == tool.built_history)
		return;
	tool.built_for = track;
	tool.built_spacing = tool.spacing;
	tool.built_side = tool.side;
	tool.built_scope = tool.scope;
	tool.built_history = m_history.history_revision();
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
	auto const sources{line_sources(*track, tool.scope)};
	auto const offset{tool.spacing * tool.side};
	// the joints are moved once, along the mean direction of the paths meeting there, so the new paths meet too
	std::vector<glm::dvec3> joints;
	for (std::size_t k = 0; k <= sources.size(); ++k)
		joints.push_back(shifted_joint(sources, k, offset));
	for (std::size_t i = 0; i < sources.size(); ++i)
	{
		auto const piece{offset_piece(sources[i].second, joints[i], joints[i + 1], offset)};
		if (false == piece.has_value())
		{
			tool.pieces.clear();
			tool.styles.clear();
			tool.error = format(STR_C("The spacing is larger than the radius of %s"), sources[i].first->name().c_str());
			return;
		}
		tool.length += bezier{*piece}.plan_length();
		tool.pieces.push_back(*piece);
		tool.styles.push_back(sources[i].first);
	}
}

void parallel_tool::parallel_build()
{
	auto &tool{m_parallel};
	parallel_update();
	if (tool.pieces.empty())
		return;
	std::vector<TTrack *> created;
	for (std::size_t i = 0; i < tool.pieces.size(); ++i)
		created.push_back(editor_track::create_path(*tool.styles[i], tool.pieces[i]));
	editor_track::commit(created);
	m_design.digitize_drawn(created);
	m_history.push_track_snapshot({}, created);
	tool.status = format(STR_C("Parallel track of %zu paths, %.1f m, %.2f m away; Ctrl+Z takes it back"), created.size(), tool.length, tool.spacing);
	if (scene::Layers.active() == null_handle)
		tool.status += "\n" + std::string{STR_C("The scenery isn't opened for editing, the new track won't be saved")};
	WriteLog("Editor: " + tool.status, logtype::generic);
	tool.built_for = nullptr;
}

void parallel_tool::parallel_apply()
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
	m_history.push_track_snapshot(std::move(states));
	tool.status = format(STR_C("%s set parallel to %s, %.2f m away; Ctrl+Z takes it back"), name_of(*target).c_str(), name_of(*parallel_source()).c_str(), tool.spacing);
	WriteLog("Editor: " + tool.status, logtype::generic);
	tool.built_for = nullptr;
	m_design.straight_refresh();
}

void parallel_tool::render_parallel_controls(bool const Scope)
{
	auto &tool{m_parallel};
	if (Scope)
		segmented("scope", tool.scope, {{0, STR_C("This path"), "Only beside the selected path"}, {1, STR_C("The line, up to the switches"), "Beside the whole line of the selected path, up to the switches: its curves go along with R minus or plus the spacing"}});
	side_picker(tool.side, tool.spacing);
	ImGui::SetNextItemWidth(std::round(ImGui::GetFontSize() * 6.0f));
	ImGui::InputDouble("##spacing", &tool.spacing, 0.0, 0.0, STR_C("%.2f m"));
	item_tooltip("Spacing of the axes of the tracks; the usual ones beside it");
	tool.spacing = std::clamp(tool.spacing, 1.5, 100.0);
	ImGui::SameLine();
	value_chips("spacings", tool.spacing, {kSpacings[0], kSpacings[1], kSpacings[2], kSpacings[3], kSpacings[4], kSpacings[5]}, "%.2f");
}

void parallel_tool::render_options()
{
	auto &tool{m_parallel};
	ImGui::PushTextWrapPos(0.0f);
	if (parallel_target() != nullptr)
		ImGui::TextDisabled("%s", STR_C("Two tracks selected: set the spacing of the other in the Select tool"));
	else if (parallel_source() == nullptr)
		ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "%s", STR_C("Click a track in the view: the new one goes beside it."));
	ImGui::PopTextWrapPos();
	render_parallel_controls();
	parallel_update();
	if (false == tool.error.empty())
	{
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "%s", tool.error.c_str());
		ImGui::PopTextWrapPos();
	}
	else if (false == tool.pieces.empty())
	{
		if (main_button(format(STR_P("Lay %zu track, %.1f m", "Lay %zu tracks, %.1f m", static_cast<long long>(tool.pieces.size())), tool.pieces.size(), tool.length).c_str(), ImVec2(-1.0f, tall_button_height())))
			parallel_build();
		item_tooltip("A new track beside the one selected, as in the preview; the selected stays as it is. Ctrl+Z takes it back");
	}
	if (false == tool.status.empty())
		ImGui::TextWrapped("%s", tool.status.c_str());
}

void parallel_tool::render_set_options()
{
	auto &tool{m_parallel};
	auto *source{parallel_source()};
	auto *target{parallel_target()};
	if (source == nullptr || target == nullptr)
		return;
	if (false == ImGui::CollapsingHeader(STR_C("Make parallel"), ImGuiTreeNodeFlags_DefaultOpen))
		return;
	ImGui::TextWrapped("%s", STR_C("The first selected track must be a straight"));
	ImGui::TextWrapped("%s", format(STR_C("%s stays, %s moves to this spacing with its curves"), name_of(*source).c_str(), name_of(*target).c_str()).c_str());
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

bool parallel_tool::parallel_preview_visible() const
{
	if (m_parallel.pieces.empty() || parallel_source() != m_parallel.built_for)
		return false;
	if (m_view.current_track_tab() == editor_track_tab::parallel)
		return true;
	return m_view.current_track_tab() == editor_track_tab::path && parallel_target() != nullptr;
}

void parallel_tool::draw_overlay() const
{
	if (false == parallel_preview_visible())
		return;
	auto const &tool{m_parallel};
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
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

editor_selftest_result parallel_tool::selftest(std::string const &Command, std::istringstream &Words)
{
	if (Command == "parallel")
	{
		Words >> m_parallel.spacing >> m_parallel.side;
		m_parallel.scope = 1;
		m_parallel.built_for = nullptr;
		parallel_build();
		WriteLog("SELFTEST parallel: " + m_parallel.status + " " + m_parallel.error);
		return editor_selftest_result::handled;
	}
	return editor_selftest_result::unhandled;
}
