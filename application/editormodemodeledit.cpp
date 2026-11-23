/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// The rest of the editing of a line of the model through the tools of the editor (editor-tracks-domain.md §8; stage E4b), with the setting
// "Editing on the model (trial)" on: the grade line drawn in the profile tool, the curves added to the line and taken out of it in the route
// design, and a track laid alongside it by the parallel tool, each a command of the model, refused with the reasons or kept as one step of
// the history.

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editormodemodel.h"
#include "application/editormodeselftest.h"
#include "application/editorprojection.h"
#include "application/editormodeshared.h"
#include "application/editoruilayer.h"
#include "editor/editorFormat.hpp"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "world/Track.h"

#include "imgui/imgui.h"

#include <algorithm>
#include <cmath>

namespace
{

std::string line_name(layout::line_id const Line)
{
	return format("L%u", layout::number(Line));
}

} // namespace

// the grade line of a line of the model in the profile tool

layout::line_id editor_mode::model_profile_line(bool &Reversed) const
{
	Reversed = false;
	if (false == model_on())
		return layout::line_id::none;
	auto const &spans{m_profile.route.spans};
	if (spans.empty())
		return layout::line_id::none;
	auto const *first{model_track(spans.front().track)};
	if (first == nullptr || layout::owner_of(model_network(), first->line) != nullptr)
		return layout::line_id::none;
	auto const line{first->line};
	auto const tracks{model_line_tracks(line)};
	if (tracks.size() != spans.size())
		return layout::line_id::none;
	// the route over the tracks of the line from end to end: along it, or against it
	Reversed = false == spans.front().forward;
	for (std::size_t i = 0; i < spans.size(); ++i)
		if (spans[i].track != tracks[Reversed ? tracks.size() - 1 - i : i] || spans[i].forward == Reversed)
			return layout::line_id::none;
	return line;
}

bool editor_mode::model_profile_open(TTrack &Track)
{
	// the profile of a line of the model is the one of the whole line, from its start to its end
	auto const *managed{model_track(&Track)};
	if (managed == nullptr || layout::owner_of(model_network(), managed->line) != nullptr)
		return false;
	auto const tracks{model_line_tracks(managed->line)};
	if (tracks.empty())
		return false;
	profile_open(tracks.front(), tracks.back());
	return true;
}

void editor_mode::model_profile_load()
{
	bool reversed{false};
	auto const id{model_profile_line(reversed)};
	auto const *line{model_network().find(id)};
	if (line == nullptr || false == line->solved.valid() || line->solved.profile.size() < 2)
		return;
	auto &state{m_profile};
	// the breaks of the grade line as the model has them, the heights of the ends its nodes'
	auto const length{line->solved.length};
	auto const scale{length > 0.0 ? state.route.length / length : 1.0};
	auto const along = [&](double const Chainage) { return (reversed ? length - Chainage : Chainage) * scale; };
	std::vector<profile::pvi> points;
	for (auto const &point : line->solved.profile)
		points.push_back({along(point.chainage), point.elevation, point.vertical_radius, false, false});
	if (reversed)
		std::ranges::reverse(points);
	points.front().radius = 0.0;
	points.back().radius = 0.0;
	state.line.points = std::move(points);
	state.line.join_start = false;
	state.line.join_end = false;
	state.changed = false;
	state.fitting = false;
	state.selected = -1;
	profile_check();
	state.status = format(STR_C("Line %s of the model: its grade line, %zu breaks"), line_name(id).c_str(), line->shape.grades.size());
}

bool editor_mode::model_profile_apply()
{
	bool reversed{false};
	auto const id{model_profile_line(reversed)};
	auto const *line{model_network().find(id)};
	if (line == nullptr)
		return false;
	auto &state{m_profile};
	auto const &points{state.line.points};
	auto const length{line->solved.length};
	auto const scale{state.route.length > 0.0 ? length / state.route.length : 1.0};
	auto const chainage = [&](double const Along) { return reversed ? length - Along * scale : Along * scale; };
	layout::outcome refusal;
	// the heights of its ends are the nodes', which the other lines meeting there share
	auto const *start{model_network().find(line->start)};
	auto const *end{model_network().find(line->end)};
	if (points.size() < 2 || start == nullptr || end == nullptr)
		return false;
	auto const &first{reversed ? points.back() : points.front()};
	auto const &last{reversed ? points.front() : points.back()};
	double constexpr kEndHeight{0.001}; // m
	for (auto const &[point, node] : {std::pair{&first, start}, std::pair{&last, end}})
		if (std::abs(point->elevation - node->position.y) > kEndHeight)
			refusal.reasons.push_back(format("the end of the line at %.3f m, its node at %.3f m: a line of the model keeps the heights of its nodes", point->elevation, node->position.y));
	layout::set_grades command{id, {}};
	// the radius of a break with none given: the one the line needs, so the grade line stays one the model lays
	auto const needed{std::max(1.0, profile::required_radius(state.line))};
	for (std::size_t i = 1; i + 1 < points.size(); ++i)
		command.grades.push_back({chainage(points[i].chainage), points[i].elevation, points[i].radius > 0.0 ? points[i].radius : needed});
	if (reversed)
		std::ranges::reverse(command.grades);
	if (false == refusal.reasons.empty())
	{
		model_told(refusal, true);
		state.error = model_shown();
		return true;
	}
	if (command.grades == line->shape.grades)
	{
		state.status = STR_C("Nothing changed");
		return true;
	}
	state.error.clear();
	model_context context;
	context.done = [this, id](std::vector<TTrack *> const &) {
		auto const tracks{model_line_tracks(id)};
		auto const shown{model_shown()};
		auto const notices{model_notices()};
		if (false == tracks.empty())
			profile_open(tracks.front(), tracks.back());
		auto &state{m_profile};
		state.status = shown;
		for (auto const &notice : notices)
			state.status += "\n" + notice;
	};
	model_run({command}, context);
	if (model_refused())
		state.error = model_shown();
	WriteLog(format("Editor: model: the grade line of line %s, %zu breaks: %s", line_name(id).c_str(), command.grades.size(), editor_selftest::one_line(model_message()).c_str()), logtype::generic);
	return true;
}

// a handle of the route design dragged on a line of the model: the line as the model lays it and the norms' line, before it's let go

void editor_mode::model_plan_preview()
{
	auto &preview{model().preview};
	auto const line{model_plan_line()};
	if (line == layout::line_id::none || false == m_route_dragging)
	{
		preview = {};
		return;
	}
	std::string key;
	for (auto const &vertex : m_route.design.vertices)
		key += format("%.4f %.4f %.3f %.3f %.3f %.1f;", vertex.position.x, vertex.position.y, vertex.radius, vertex.transition_in, vertex.transition_out, vertex.cant);
	if (key == preview.key)
		return;
	preview = {};
	preview.key = key;
	std::vector<layout::command> commands;
	if (std::string why; false == model_plan_commands(line, commands, why))
	{
		preview.refused = true;
		preview.text = model_tr(why);
		return;
	}
	// tried on a copy of the network, one after another as they'd be carried out
	layout::model copy{model_network()};
	std::vector<std::string> notices;
	for (auto const &command : commands)
	{
		auto const outcome{copy.execute(command)};
		if (false == outcome.done)
		{
			preview.refused = true;
			preview.text = model_reasons_shown(outcome);
			return;
		}
		for (auto const &notice : outcome.notices)
			if (auto const text{model_notice_shown(notice)}; std::ranges::find(notices, text) == notices.end())
				notices.push_back(text);
	}
	preview.text = notices.empty() ? std::string{STR_C("Within the norms")} : format(STR_P("The norms: %zu note", "The norms: %zu notes", static_cast<long long>(notices.size())), notices.size());
	for (auto const &notice : notices)
		preview.text += "\n" + notice;
	if (auto const *laid{copy.state().find(line)}; laid != nullptr && laid->solved.valid())
	{
		auto const steps{std::clamp(static_cast<int>(laid->solved.length / 2.0), 1, 2000)};
		for (int i = 0; i <= steps; ++i)
			preview.points.push_back(laid->solved.point(laid->solved.length * i / steps));
	}
}

void editor_mode::draw_model_plan_preview() const
{
	if (m_model == nullptr)
		return;
	auto const &preview{m_model->preview};
	if (preview.key.empty())
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetForegroundDrawList(ImGui::GetMainViewport())};
	auto const colour{preview.refused ? IM_COL32(255, 90, 70, 230) : IM_COL32(90, 220, 110, 230)};
	for (std::size_t i = 0; i + 1 < preview.points.size(); ++i)
		projection.line(drawlist, preview.points[i], preview.points[i + 1], colour, 2.0f);
	// by the mouse: refused and why, or the norms' line
	auto const &mouse{ImGui::GetIO().MousePos};
	auto const text{(preview.refused ? std::string{STR_C("Refused, nothing changes:")} + "\n" : std::string{}) + preview.text};
	drawlist->AddText(ImVec2(mouse.x + 18.0f, mouse.y + 18.0f), colour, text.c_str());
}
