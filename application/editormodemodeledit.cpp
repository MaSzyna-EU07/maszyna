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
#include "editor/layout/layoutParallel.hpp"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "world/Track.h"

#include "imgui/imgui.h"

#include <algorithm>
#include <cmath>
#include <limits>

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

// the curves of a line of the model added and taken out in the route design

void editor_mode::model_vertex_add(double const Chainage)
{
	auto const id{model_plan_line()};
	auto const *line{model_network().find(id)};
	if (line == nullptr || false == line->solved.valid())
		return;
	auto const at{std::clamp(Chainage, 0.0, line->solved.length)};
	// after the vertex whose curve lies before the point, by the chainages of the tangent points of the curves
	layout::vertex_id after{layout::vertex_id::none};
	auto const &shape{line->shape};
	glm::dvec2 const point{line->solved.position(at)};
	auto const start{model_network().find(line->start)};
	auto const end{model_network().find(line->end)};
	if (start == nullptr || end == nullptr)
		return;
	// the polygon of the tangents: the new vertex goes onto the leg the point lies on, nearest of them
	std::vector<glm::dvec2> corners{{start->position.x, start->position.z}};
	for (auto const &vertex : shape.vertices)
		corners.push_back(vertex.point);
	corners.push_back({end->position.x, end->position.z});
	double best{std::numeric_limits<double>::infinity()};
	std::size_t leg{0};
	for (std::size_t i = 0; i + 1 < corners.size(); ++i)
	{
		auto const a{corners[i]};
		auto const b{corners[i + 1]};
		auto const span{b - a};
		auto const t{glm::dot(span, span) > 0.0 ? std::clamp(glm::dot(point - a, span) / glm::dot(span, span), 0.0, 1.0) : 0.0};
		if (auto const distance{glm::distance(a + span * t, point)}; distance < best)
		{
			best = distance;
			leg = i;
		}
	}
	if (leg > 0)
		after = shape.vertices[leg - 1].id;
	// on the leg, where it runs nearest to the point of the line: no curve until it's moved off the leg
	auto const a{corners[leg]};
	auto const span{corners[leg + 1] - a};
	auto const t{glm::dot(span, span) > 0.0 ? std::clamp(glm::dot(point - a, span) / glm::dot(span, span), 0.0, 1.0) : 0.0};
	layout::vertex made;
	made.point = a + span * t;
	made.radius = m_lay.radius > 0.0 ? m_lay.radius : 600.0;
	made.transition_in = 0.0;
	made.transition_out = 0.0;
	made.cant = 0.0;
	model_context context;
	context.done = [this, id](std::vector<TTrack *> const &) { model_plan_reload(id); };
	model_run({layout::insert_vertex{id, after, made}}, context);
}

void editor_mode::model_vertex_remove(std::size_t const Index)
{
	auto const id{model_plan_line()};
	auto const *line{model_network().find(id)};
	if (line == nullptr || Index >= line->shape.vertices.size())
		return;
	model_context context;
	context.done = [this, id](std::vector<TTrack *> const &) { model_plan_reload(id); };
	model_run({layout::remove_vertex{id, line->shape.vertices[Index].id}}, context);
}

void editor_mode::model_plan_reload(layout::line_id const Line)
{
	// the route design read from the model again, what the command came to kept
	auto const message{model().message};
	auto const notices{model().notices};
	auto const shown{model().shown};
	auto const shown_notices{model().shown_notices};
	auto const refused{model().refused};
	route_from_model(Line);
	model().message = message;
	model().notices = notices;
	model().shown = shown;
	model().shown_notices = shown_notices;
	model().refused = refused;
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
	// the first and the last vertex of two or more are dragged along the tangents of the ends by their offsets, which their positions
	// don't follow (model_plan_commands places them so): without the offset the preview stayed as its first frame showed it
	std::string key;
	for (auto const &vertex : m_route.design.vertices)
		key += format("%.4f %.4f %.4f %.3f %.3f %.3f %.1f;", vertex.position.x, vertex.position.y, vertex.offset, vertex.radius, vertex.transition_in, vertex.transition_out, vertex.cant);
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

void editor_mode::render_model_curves()
{
	auto const line{model_plan_line()};
	auto const *shape{model_network().find(line)};
	if (shape == nullptr)
		return;
	// a handle dragged: the line as the model would lay it
	if (auto const &preview{model().preview}; false == preview.key.empty())
	{
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextColored(preview.refused ? ImVec4(1.0f, 0.35f, 0.3f, 1.0f) : ImVec4(0.45f, 0.85f, 0.55f, 1.0f), "%s", ((preview.refused ? std::string{STR_C("Refused, nothing changes:")} + "\n" : std::string{}) + preview.text).c_str());
		ImGui::PopTextWrapPos();
	}
	// a curve added: a vertex on the tangents at the chainage, bent once its vertex is dragged; one taken out
	auto &at{model().vertex_at};
	if (at < 0.0 || at > shape->solved.length)
		at = std::round(shape->solved.length * 0.5);
	ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6.0f);
	ImGui::InputDouble("##modelvertexat", &at, 0.0, 0.0, "%.1f m");
	ImGui::SameLine();
	if (ImGui::Button(STR_C("Add a curve here")))
	{
		model_vertex_add(at);
		return;
	}
	item_tooltip("A new vertex of the tangents at the distance from the start of the line: no curve until its vertex is dragged off the line, then of the radius of the lay tool");
	for (std::size_t i = 0; i < shape->shape.vertices.size(); ++i)
	{
		if (i > 0)
			ImGui::SameLine();
		ImGui::PushID(static_cast<int>(i));
		if (ImGui::SmallButton(format(STR_C("Take W%zu out"), i + 1).c_str()))
		{
			ImGui::PopID();
			model_vertex_remove(i);
			return;
		}
		item_tooltip("The curve taken out: the tangents of the vertices beside it meet with no curve of it");
		ImGui::PopID();
	}
}

// a track laid alongside a line of the model by the parallel tool

bool editor_mode::model_parallel(TTrack &Source, double const Spacing, int const Side, std::string &Status, std::string &Error)
{
	auto const *managed{model_track(&Source)};
	if (managed == nullptr)
		return false;
	Status.clear();
	Error.clear();
	if (layout::owner_of(model_network(), managed->line) != nullptr)
	{
		Error = STR_C("A turnout has no line of its own: select a track of a line beside it");
		return true;
	}
	// the side of the tool and the left of the model: the same offset of the axis
	auto const plan{layout::parallel_line(model_network(), managed->line, Spacing * static_cast<double>(Side))};
	if (false == plan.valid)
	{
		for (auto const &error : plan.errors)
			Error += (Error.empty() ? "" : "\n") + model_tr(error);
		return true;
	}
	model_context context;
	context.surface = model_surface_of(Source);
	auto const line{managed->line};
	auto const spacings{plan.spacings};
	context.done = [this, &Status, line, Spacing, spacings](std::vector<TTrack *> const &Created) {
		if (false == Created.empty())
		{
			m_node = Created.back();
			ui()->set_node(m_node);
		}
		Status = format(STR_C("A line alongside line %s, %.2f m away"), line_name(line).c_str(), Spacing);
		for (std::size_t i = 0; i < spacings.size(); ++i)
			if (spacings[i] > Spacing + 1e-4)
				Status += "\n" + format(STR_C("In the curve at W%zu: %.3f m"), i + 1, spacings[i]);
		Status += "\n" + model_shown();
		for (auto const &notice : model_notices())
			Status += "\n" + notice;
	};
	model_run({plan.command}, context);
	if (model_refused())
		Error = model_shown();
	for (auto const &note : plan.notes)
		(model_refused() ? Error : Status) += "\n" + model_tr(note);
	WriteLog(format("Editor: model: a line alongside line %s, %.2f m to the %s: %s", line_name(line).c_str(), Spacing, Side > 0 ? "right" : "left", editor_selftest::one_line(model_message()).c_str()), logtype::generic);
	return true;
}
