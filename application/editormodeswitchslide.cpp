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
#include <array>
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

bool editor_mode::double_room(double_switch const &Set, double &Low, double &High, double &Length, std::string &Reason)
{
	double low{0.0};
	double high{0.0};
	double low2{0.0};
	double high2{0.0};
	auto const axis{editor_track::main_axis(*Set.first)};
	if (false == axis.has_value() || false == switch_room(*Set.first, low, high, Reason) || false == switch_room(*Set.second, low2, high2, Reason))
		return false;
	auto const &main{Set.second->m_paths.front()};
	Low = low;
	High = axis->along(main.points[segment_data::point::start]) + high2;
	Length = axis->along(main.points[segment_data::point::end]);
	return true;
}

bool editor_mode::move_double_paths(double_switch const &Set, std::vector<segment_data> const &Paths, std::string &Status)
{
	auto const axis{editor_track::main_axis(*Set.first)};
	if (Paths.size() < 5 || false == axis.has_value())
		return false;
	double low{0.0};
	double high{0.0};
	double length{0.0};
	std::string reason;
	bool const online{double_room(Set, low, high, length, reason)};
	if (false == online && false == reason.empty())
	{
		Status = reason;
		return false;
	}
	struct outer_end
	{
		TTrack *track{nullptr};
		switch_end_move move;
	};
	std::vector<outer_end> ends;
	auto const outer = [&](TTrack &Track, glm::dvec3 const &From, glm::dvec3 const &To, bool const Atstart) {
		if (false == online || glm::distance(From, To) < 0.005 || axis->lateral(To) > m_straights.tolerance.offset)
			return true;
		auto const target{axis->along(To)};
		if (target < low - 0.01 || target > high + 0.01)
		{
			Status = STR_C("The switch would run into another switch or past the end of the straight");
			return false;
		}
		ends.push_back({&Track, {From, To, Atstart ? axis->along(From) - target : target - axis->along(From)}});
		return true;
	};
	if (false == outer(*Set.first, Set.first->m_paths.front().points[segment_data::point::start], Paths[0].points[segment_data::point::start], true) ||
	    false == outer(*Set.second, Set.second->m_paths.front().points[segment_data::point::end], Paths[2].points[segment_data::point::end], false))
		return false;
	std::stable_sort(ends.begin(), ends.end(), [](outer_end const &A, outer_end const &B) { return A.move.shift > B.move.shift; });
	std::array<TTrack *, 2> const switches{Set.first, Set.second};
	std::array<std::vector<editor_track::joint>, 2> joints;
	std::array<std::vector<glm::dvec3>, 2> positions;
	std::array<std::vector<bool>, 2> handled;
	for (std::size_t k = 0; k < 2; ++k)
	{
		joints[k] = editor_track::joints(*switches[k]);
		for (auto const &joint : joints[k])
		{
			positions[k].push_back(editor_track::point_position(*switches[k], joint.own));
			handled[k].push_back(joint.other == Set.link || joint.other == Set.first || joint.other == Set.second);
		}
	}
	auto const joint{Set.first->m_paths.front().points[segment_data::point::end]};
	auto const &link{Set.link->m_paths.front()};
	bool const reversed{glm::distance(link.points[segment_data::point::start], joint) > glm::distance(link.points[segment_data::point::end], joint)};
	editor_track::edit edit;
	for (auto *track : {Set.first, Set.link, Set.second})
		edit.capture(*track);
	for (auto const &end : ends)
	{
		for (std::size_t k = 0; k < 2; ++k)
			for (std::size_t i = 0; i < joints[k].size(); ++i)
				if (glm::distance(positions[k][i], end.move.from) < geometry::same_point)
					handled[k][i] = true;
		auto const line{editor_track::find_straight(*end.track, m_straights.tolerance)};
		if (false == switch_vacate(*end.track, end.move.from, end.move.to, end.move.shift, line, edit, Status))
			return false;
	}
	Set.first->m_paths.assign(Paths.begin(), Paths.begin() + 2);
	Set.second->m_paths.assign(Paths.begin() + 2, Paths.begin() + 4);
	Set.link->m_paths = {reversed ? geometry::reversed(Paths[4]) : Paths[4]};
	std::string loose;
	for (std::size_t k = 0; k < 2; ++k)
	{
		auto const left{switch_rejoin(*switches[k], joints[k], handled[k], edit)};
		if (false == left.empty())
			loose += (loose.empty() ? "" : ", ") + left;
	}
	editor_track::commit(edit.changed);
	push_track_snapshot(std::move(edit.states), std::move(edit.created), std::move(edit.removed));
	if (false == loose.empty())
		Status = STR_C("Too far from the new ends of the switch, left unjoined: ") + loose;
	return true;
}

bool editor_mode::slide_switch(TTrack &Track, double const Distance)
{
	if (std::abs(Distance) < 0.005)
		return false;
	auto const set{double_of(Track)};
	auto &first{set.has_value() ? *set->first : Track};
	if (first.m_paths.size() < 2)
		return false;
	auto paths{editor_track::slid(first, Distance)};
	if (paths.empty())
		return false;
	std::string status;
	bool moved{false};
	if (set.has_value())
	{
		auto const shift{paths.front().points[segment_data::point::start] - first.m_paths.front().points[segment_data::point::start]};
		paths = double_paths(*set);
		for (auto &path : paths)
			path.translate(shift);
		moved = move_double_paths(*set, paths, status);
	}
	else
		moved = move_switch_paths(Track, paths, status);
	if (false == moved)
	{
		m_switch.status = status;
		ui()->set_status(status, true);
		return false;
	}
	switch_store::follow();
	records_to_history();
	straight_refresh();
	m_switch.status = status.empty() ? format(STR_C("Switch %s moved %.2f m along the straight"), first.name().c_str(), Distance) : status;
	ui()->set_status(m_switch.status, false == status.empty());
	return true;
}

bool editor_mode::switch_slides(TTrack const &Track) const
{
	if (Track.eType == tt_Normal)
	{
		auto const set{double_of(Track)};
		return set.has_value() && switch_slides(*set->first) && switch_slides(*set->second);
	}
	return Track.eType == tt_Switch && Track.m_paths.size() == 2 && false == Track.m_editorremoved && false == has_controls(Track.m_paths.front());
}

void editor_mode::switch_slide_start(TTrack &Track)
{
	auto &slide{m_switch.slide};
	slide = {};
	std::string reason;
	auto const set{double_of(Track)};
	auto &first{set.has_value() ? *set->first : Track};
	auto const axis{editor_track::main_axis(first)};
	if (false == axis.has_value())
		return;
	slide.length = axis->length;
	if (false == (set.has_value() ? double_room(*set, slide.low, slide.high, slide.length, reason) : switch_room(first, slide.low, slide.high, reason)))
	{
		if (false == reason.empty())
			ui()->set_status(reason, true);
		return;
	}
	auto const &mouse{ImGui::GetIO().MousePos};
	slide.active = true;
	slide.track = &first;
	slide.pressed = {mouse.x, mouse.y};
	slide.grip = axis->along(cursor_level(axis->origin.y));
	std::vector<TTrack *> members{&first};
	if (set.has_value())
		members = {set->first, set->link, set->second};
	for (auto *member : members)
	{
		if (member->eType != tt_Switch)
			continue;
		auto const line{editor_track::find_straight(*member, m_straights.tolerance)};
		for (std::size_t i = 0; i < line.tracks.size(); ++i)
		{
			if (std::find(members.begin(), members.end(), line.tracks[i]) != members.end())
				continue;
			auto const &path{line.tracks[i]->m_paths[line.paths[i]]};
			for (auto const index : {segment_data::point::start, segment_data::point::end})
				slide.joints.push_back(axis->along(path.points[index]));
		}
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
	auto const length{slide.length};
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
	auto paths{editor_track::slid(*slide.track, slide.distance)};
	if (paths.empty())
		return;
	if (auto const set{double_of(*slide.track)})
	{
		auto const shift{paths.front().points[segment_data::point::start] - slide.track->m_paths.front().points[segment_data::point::start]};
		paths = double_paths(*set);
		for (auto &path : paths)
			path.translate(shift);
	}
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
