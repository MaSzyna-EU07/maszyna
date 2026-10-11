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
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorSwitchStore.hpp"
#include "editor/editorGeometry.hpp"

#include "world/Track.h"
#include "utilities/translation.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <optional>
#include <tuple>

namespace
{

using geometry::bezier;
using geometry::has_controls;
using geometry::plan_of;

struct switch_end_move
{
	glm::dvec3 from{0.0};
	glm::dvec3 to{0.0};
	double shift{0.0};
};

std::string track_label(TTrack const &Track)
{
	return Track.name().empty() ? std::string{"(noname)"} : Track.name();
}

std::optional<std::vector<switch_end_move>> switch_end_moves(TTrack const &Track, segment_data const &Main, bool const Online, std::pair<double, double> const &Room, double const Offset, std::string &Status)
{
	auto const axis{editor_track::main_axis(Track)};
	std::vector<switch_end_move> ends;
	for (auto const index : {segment_data::point::start, segment_data::point::end})
	{
		auto const from{Track.m_paths.front().points[index]};
		auto const to{Main.points[index]};
		if (false == Online || glm::distance(from, to) < 0.005 || axis->lateral(to) > Offset)
			continue;
		auto const target{axis->along(to)};
		if (target < Room.first - 0.01 || target > Room.second + 0.01)
		{
			Status = STR_C("The switch would run into another switch or past the end of the straight");
			return std::nullopt;
		}
		ends.push_back({from, to, index == segment_data::point::start ? axis->along(from) - target : target - axis->along(from)});
	}
	std::stable_sort(ends.begin(), ends.end(), [](switch_end_move const &A, switch_end_move const &B) { return A.shift > B.shift; });
	return ends;
}

std::string switch_rejoin(TTrack &Track, std::vector<editor_track::joint> const &Joints, std::vector<bool> const &Handled, editor_track::edit &Edit)
{
	std::vector<editor_track::joint> following;
	std::string loose;
	for (std::size_t i = 0; i < Joints.size(); ++i)
	{
		auto const &joint{Joints[i]};
		if (Handled[i] || joint.other->m_editorremoved)
			continue;
		auto const &theirs{joint.other->m_paths[joint.theirs.path]};
		auto const shift{glm::distance(editor_track::point_position(Track, joint.own), editor_track::point_position(*joint.other, joint.theirs))};
		if (shift > 0.3 * bezier{theirs}.plan_length())
		{
			loose += (loose.empty() ? "" : ", ") + track_label(*joint.other);
			continue;
		}
		following.push_back(joint);
		Edit.capture(*joint.other);
	}
	editor_track::follow(Track, following);
	return loose;
}

} // namespace

bool editor_mode::switch_room(TTrack &Track, double &Low, double &High, std::string &Reason)
{
	Low = 0.0;
	High = 0.0;
	if (Track.eType != tt_Switch || Track.m_paths.size() < 2 || has_controls(Track.m_paths.front()))
		return false;
	auto const axis{editor_track::main_axis(Track)};
	if (false == axis.has_value())
		return false;
	auto const line{editor_track::find_straight(Track, m_straights.tolerance)};
	if (false == tracks_editable(line.tracks, Reason))
		return false;
	std::vector<std::tuple<double, double, bool>> spans;
	for (std::size_t i = 0; i < line.tracks.size(); ++i)
	{
		if (line.tracks[i] == &Track)
			continue;
		auto const &path{line.tracks[i]->m_paths[line.paths[i]]};
		auto const a{axis->along(path.points[segment_data::point::start])};
		auto const b{axis->along(path.points[segment_data::point::end])};
		spans.emplace_back(std::min(a, b), std::max(a, b), line.tracks[i]->eType == tt_Normal);
	}
	std::sort(spans.begin(), spans.end());
	double low{0.0};
	double high{axis->length};
	for (auto const &[from, to, plain] : spans)
		if (from > axis->length - 0.01)
		{
			if (false == plain || from > high + 0.01)
				break;
			high = std::max(high, to);
		}
	for (auto span{spans.rbegin()}; span != spans.rend(); ++span)
	{
		auto const &[from, to, plain] = *span;
		if (to < 0.01)
		{
			if (false == plain || to < low - 0.01)
				break;
			low = std::min(low, from);
		}
	}
	Low = low;
	High = high;
	return true;
}

bool editor_mode::switch_vacate(TTrack &Track, glm::dvec3 const &From, glm::dvec3 const &To, double const Shift, editor_track::straight const &Line, editor_track::edit &Edit, std::string &Status)
{
	if (Shift > 0.0)
	{
		auto const a{Line.along(From)};
		auto const b{Line.along(To)};
		if (cut_straight(Line, std::min(a, b), std::max(a, b), Edit.states, Edit.created, Edit.removed, nullptr))
			return true;
		Status = STR_C("The switch would overlap another switch, move it onto plain track");
		return false;
	}
	TTrack const *style{nullptr};
	bool filler{false};
	for (auto const &[other, point] : editor_track::connected_points(Track, From))
	{
		if (other->m_editorremoved)
			continue;
		if (other->eType == tt_Normal && std::find(Line.tracks.begin(), Line.tracks.end(), other) != Line.tracks.end())
		{
			Edit.capture(*other);
			editor_track::move_point(*other, point, To);
			continue;
		}
		filler = true;
		if (style == nullptr && other->eType == tt_Normal)
			style = other;
	}
	if (filler)
	{
		segment_data path;
		path.points[segment_data::point::start] = To;
		path.points[segment_data::point::end] = From;
		auto *piece{editor_track::create_path(style != nullptr ? *style : Track, path)};
		Edit.created.push_back(piece);
		Edit.changed.push_back(piece);
	}
	return true;
}

bool editor_mode::move_switch_paths(TTrack &Track, std::vector<segment_data> const &Paths, std::string &Status)
{
	if (Paths.size() < 2 || Track.m_paths.size() < 2 || false == editor_track::main_axis(Track).has_value())
		return false;
	double low{0.0};
	double high{0.0};
	std::string reason;
	bool const online{switch_room(Track, low, high, reason)};
	if (false == online && false == reason.empty())
	{
		Status = reason;
		return false;
	}
	auto const ends{switch_end_moves(Track, Paths.front(), online, {low, high}, m_straights.tolerance.offset, Status)};
	if (false == ends.has_value())
		return false;
	auto const joints{editor_track::joints(Track)};
	std::vector<glm::dvec3> positions;
	for (auto const &joint : joints)
		positions.push_back(editor_track::point_position(Track, joint.own));
	std::vector<bool> handled(joints.size(), false);
	editor_track::edit edit;
	edit.capture(Track);
	auto const line{online ? editor_track::find_straight(Track, m_straights.tolerance) : editor_track::straight{}};
	for (auto const &end : *ends)
	{
		for (std::size_t i = 0; i < joints.size(); ++i)
			if (glm::distance(positions[i], end.from) < geometry::same_point)
				handled[i] = true;
		if (false == switch_vacate(Track, end.from, end.to, end.shift, line, edit, Status))
			return false;
	}
	Track.m_paths.assign(Paths.begin(), Paths.begin() + 2);
	auto const loose{switch_rejoin(Track, joints, handled, edit)};
	editor_track::commit(edit.changed);
	push_track_snapshot(std::move(edit.states), std::move(edit.created), std::move(edit.removed));
	if (false == loose.empty())
		Status = STR_C("Too far from the new ends of the switch, left unjoined: ") + loose;
	return true;
}

bool editor_mode::slide_switch(TTrack &Track, double const Distance)
{
	if (Track.m_paths.size() < 2 || std::abs(Distance) < 0.005)
		return false;
	auto const paths{editor_track::slid(Track, Distance)};
	if (paths.empty())
		return false;
	std::string status;
	if (false == move_switch_paths(Track, paths, status))
	{
		m_switch.status = status;
		ui()->set_status(status, true);
		return false;
	}
	switch_store::follow();
	records_to_history();
	straight_refresh();
	m_switch.status = status.empty() ? format(STR_C("Switch %s moved %.2f m along the straight"), Track.name().c_str(), Distance) : status;
	ui()->set_status(m_switch.status, false == status.empty());
	return true;
}

bool editor_mode::switch_slides(TTrack const &Track) const
{
	return Track.eType == tt_Switch && Track.m_paths.size() == 2 && false == Track.m_editorremoved && false == has_controls(Track.m_paths.front());
}

void editor_mode::switch_slide_start(TTrack &Track)
{
	auto &slide{m_switch.slide};
	slide = {};
	std::string reason;
	if (false == switch_room(Track, slide.low, slide.high, reason))
	{
		if (false == reason.empty())
			ui()->set_status(reason, true);
		return;
	}
	auto const axis{editor_track::main_axis(Track)};
	auto const &mouse{ImGui::GetIO().MousePos};
	slide.active = true;
	slide.track = &Track;
	slide.pressed = {mouse.x, mouse.y};
	slide.grip = axis->along(cursor_level(axis->origin.y));
	auto const line{editor_track::find_straight(Track, m_straights.tolerance)};
	for (std::size_t i = 0; i < line.tracks.size(); ++i)
	{
		if (line.tracks[i] == &Track)
			continue;
		auto const &path{line.tracks[i]->m_paths[line.paths[i]]};
		for (auto const index : {segment_data::point::start, segment_data::point::end})
			slide.joints.push_back(axis->along(path.points[index]));
	}
}

void editor_mode::switch_slide_update()
{
	auto &slide{m_switch.slide};
	if (false == slide.active)
		return;
	auto const axis{slide.track != nullptr && false == slide.track->m_editorremoved && slide.track->m_paths.size() >= 2 ? editor_track::main_axis(*slide.track) : std::nullopt};
	if (false == axis.has_value())
	{
		slide = {};
		return;
	}
	auto const length{axis->length};
	auto const distance{std::clamp(axis->along(cursor_level(axis->origin.y)) - slide.grip, slide.low, slide.high - length)};
	double best{1.0};
	double snapped{distance};
	for (auto const joint : slide.joints)
		for (auto const end : {0.0, length})
			if (auto const gap{std::abs(joint - (end + distance))}; gap < best && std::abs(joint - end) > 0.01)
			{
				best = gap;
				snapped = joint - end;
			}
	slide.distance = std::clamp(snapped, slide.low, slide.high - length);
}

void editor_mode::switch_slide_finish()
{
	auto &slide{m_switch.slide};
	if (false == slide.active)
		return;
	auto *track{slide.track};
	auto const distance{slide.distance};
	auto const &mouse{ImGui::GetIO().MousePos};
	auto const dragged{glm::length(glm::vec2{mouse.x, mouse.y} - slide.pressed) > kDragPixels};
	slide = {};
	if (track != nullptr && false == track->m_editorremoved && dragged)
		slide_switch(*track, distance);
}

void editor_mode::draw_switch_slide() const
{
	auto const &slide{m_switch.slide};
	if (false == slide.active || slide.track == nullptr || slide.track->m_paths.size() < 2)
		return;
	auto const &mouse{ImGui::GetIO().MousePos};
	if (glm::length(glm::vec2{mouse.x, mouse.y} - slide.pressed) <= kDragPixels)
		return;
	auto const paths{editor_track::slid(*slide.track, slide.distance)};
	if (paths.empty())
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	for (auto const &path : paths)
	{
		bezier const curve{path};
		auto previous{curve.point(0.0)};
		for (int i = 1; i <= 24; ++i)
		{
			auto const next{curve.point(i / 24.0)};
			projection.line(drawlist, previous, next, overlay_color::marked, 3.0f);
			previous = next;
		}
	}
	ImVec2 at;
	auto const start{paths.front().points[segment_data::point::start]};
	if (false == projection.project(start, at))
		return;
	auto const km{alignment_store::chainage_at(start)};
	auto const label{km.has_value() ? format("%+.2f m  km %.3f", slide.distance, *km / 1000.0) : format("%+.2f m", slide.distance)};
	auto const size{ImGui::CalcTextSize(label.c_str())};
	drawlist->AddRectFilled(ImVec2(at.x + 10.0f, at.y - 8.0f), ImVec2(at.x + 18.0f + size.x, at.y + 8.0f), IM_COL32(0, 0, 0, 180), 3.0f);
	drawlist->AddText(ImVec2(at.x + 14.0f, at.y - 8.0f), overlay_color::highlight, label.c_str());
}
