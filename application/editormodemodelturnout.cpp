/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// The turnouts of the lines of the model laid, moved, changed and taken out through the switch tool (editor-tracks-domain.md §7, §8;
// stage E4b), with the setting "Editing on the model (trial)" on. A switch dragged on a track of a line of the model is a turnout of the
// catalogue inserted into the line (insert_turnout), its switch track made by the generator; moved along the line, given another type or
// the other side, or taken out, a command of the model each; a crossover dragged between two lines of the model is the solver's
// (connect_tracks). A command refused changes nothing and the tool tells why in its red line, with the value, the limit and the paragraph;
// one kept is one step of the history. The switches of the old tracks are laid the old way.

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editormodemodel.h"
#include "application/editorflow.h"
#include "application/editormodeselftest.h"
#include "application/editormodeshared.h"
#include "application/editoruilayer.h"
#include "editor/editorFormat.hpp"
#include "editor/layout/layoutTurnout.hpp"
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

std::string turnout_name(layout::turnout_id const Turnout)
{
	return format("T%u", layout::number(Turnout));
}

double cross(glm::dvec2 const &A, glm::dvec2 const &B)
{
	return A.x * B.y - A.y * B.x;
}

} // namespace

layout::turnout const *editor_mode::model_turnout(TTrack const *Track) const
{
	auto const *managed{model_track(Track)};
	if (managed == nullptr)
		return nullptr;
	return layout::owner_of(model_network(), managed->line);
}

std::string editor_mode::model_type_of(editor_track::switch_template const &Template)
{
	// the label begins with the name of the type of the catalogue, the lengths after it: the longest name it begins with
	std::string best;
	if (Template.own)
		return best;
	for (auto const &name : layout::turnout_types())
		if (name.size() > best.size() && Template.label.compare(0, name.size(), name) == 0 && (Template.label.size() == name.size() || Template.label[name.size()] == ' '))
			best = name;
	return best;
}

bool editor_mode::model_line_at(glm::dvec3 const &Point, std::vector<TTrack *> const &Tracks, layout::line_id &Line, double &Chainage) const
{
	if (false == model_on())
		return false;
	double constexpr kReach{0.5}; // m, of the point from the line
	auto best{kReach};
	auto found{false};
	std::set<layout::line_id> seen;
	for (auto const *track : Tracks)
	{
		auto const *managed{model_track(track)};
		if (managed == nullptr || false == seen.insert(managed->line).second)
			continue;
		auto const *line{model_network().find(managed->line)};
		if (line == nullptr || false == line->solved.valid())
			continue;
		double distance{0.0};
		auto const at{layout::chainage_of(line->solved, {Point.x, Point.z}, &distance)};
		if (distance < best)
		{
			best = distance;
			Line = managed->line;
			Chainage = at;
			found = true;
		}
	}
	return found;
}

std::optional<layout::insert_turnout> editor_mode::model_switch_drag(std::string &Why) const
{
	Why.clear();
	auto const &tool{m_switch};
	if (false == model_on() || tool.free || tool.connection >= 0 || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return std::nullopt;
	glm::dvec3 start{0.0};
	glm::dvec2 heading{0.0, 1.0};
	int side{1};
	std::vector<TTrack *> tracks;
	if (tool.curved)
	{
		if (tool.frame.size() < 2)
			return std::nullopt;
		int forward{1};
		curved_switch_pose(forward, side);
		auto const sample{sample_at(tool.frame, tool.along)};
		start = sample.position;
		heading = sample.tangent * static_cast<double>(forward);
		tracks = tool.frame_tracks;
	}
	else
	{
		if (tool.line.tracks.empty())
			return std::nullopt;
		glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
		auto const forward{glm::dot(offset, tool.line.direction) >= 0.0 ? 1 : -1};
		heading = tool.line.direction * static_cast<double>(forward);
		side = cross(heading, offset) >= 0.0 ? 1 : -1;
		start = tool.point;
		tracks = tool.line.tracks;
	}
	return model_switch_command(tracks, start, heading, side, tool.templates[tool.armed], Why);
}

std::optional<layout::insert_turnout> editor_mode::model_switch_command(std::vector<TTrack *> const &Tracks, glm::dvec3 const &Start, glm::dvec2 const &Heading, int const Side, editor_track::switch_template const &Shape, std::string &Why) const
{
	layout::line_id line{layout::line_id::none};
	double chainage{0.0};
	if (false == model_line_at(Start, Tracks, line, chainage))
		return std::nullopt;
	layout::insert_turnout command;
	command.line = line;
	command.chainage = chainage;
	command.type = model_type_of(Shape);
	if (command.type.empty())
		Why = format(STR_C("%s isn't of the catalogue of the turnouts: a line of the model takes the types of the catalogue"), template_name(Shape).c_str());
	// the side the diverging track leaves to as the switch runs from its start: the side of the drag, in the plan as the model takes it
	command.hand = Side;
	auto const *found{model_network().find(line)};
	command.direction = found != nullptr && glm::dot(Heading, found->solved.tangent(chainage)) < 0.0 ? -1 : 1;
	return command;
}

bool editor_mode::model_switch_place(std::vector<TTrack *> const &Tracks, glm::dvec3 const &Start, glm::dvec2 const &Heading, int const Side)
{
	auto &tool{m_switch};
	if (false == model_on() || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return false;
	std::string why;
	return model_switch_run(model_switch_command(Tracks, Start, Heading, Side, tool.templates[tool.armed], why), why);
}

bool editor_mode::model_switch_finish()
{
	std::string why;
	return model_switch_run(model_switch_drag(why), why);
}

bool editor_mode::model_switch_run(std::optional<layout::insert_turnout> const &Command, std::string const &Why)
{
	auto &tool{m_switch};
	if (false == Command.has_value())
		return false;
	auto const &command{Command};
	auto const why{Why};
	tool.placing = false;
	if (false == why.empty())
	{
		tool.status = switch_refused(why);
		return true;
	}
	model_context context;
	context.done = [this](std::vector<TTrack *> const &Created) { model_switch_done(Created); };
	model_run({*command}, context);
	if (model_refused())
		tool.status = switch_refused(model_shown());
	WriteLog(format("Editor: model: a turnout %s into line %s at %.3f m, hand %d, direction %d: %s", command->type.c_str(), line_name(command->line).c_str(), command->chainage, command->hand, command->direction, editor_selftest::one_line(model_message()).c_str()), logtype::generic);
	return true;
}

void editor_mode::model_switch_done(std::vector<TTrack *> const &Created)
{
	auto &tool{m_switch};
	tool.placing = false;
	ui()->drop_refusal();
	tool.refusal.clear();
	auto const found{std::ranges::find_if(Created, [](TTrack const *Track) { return Track->eType == tt_Switch; })};
	if (found != Created.end())
	{
		first_step_done(first_step::turnout);
		m_track_point = {};
		m_node = *found;
		ui()->set_node(m_node);
	}
	tool.status = model_shown();
	if (auto const *turnout{found != Created.end() ? model_turnout(*found) : nullptr})
		tool.status = format(STR_C("Turnout %s: %s, %s, %.0f km/h on the diverging track"), turnout_name(turnout->id).c_str(), turnout->type.c_str(), turnout->hand > 0 ? STR_C("right") : STR_C("left"), turnout->speed_diverging) + "\n" + model_shown();
	straight_refresh();
}

bool editor_mode::model_switch_slide(TTrack &Switch, double const Distance)
{
	auto const *turnout{model_turnout(&Switch)};
	if (turnout == nullptr)
		return false;
	model_turnout_run(layout::move_turnout{turnout->id, Distance});
	return true;
}

bool editor_mode::model_switch_template(TTrack &Switch, editor_track::switch_template const &Shape)
{
	auto const *turnout{model_turnout(&Switch)};
	if (turnout == nullptr)
		return false;
	auto const type{model_type_of(Shape)};
	if (type.empty())
	{
		m_switch.status = switch_refused(format(STR_C("%s isn't of the catalogue of the turnouts: a line of the model takes the types of the catalogue"), template_name(Shape).c_str()));
		return true;
	}
	model_turnout_run(layout::change_turnout{turnout->id, type, std::nullopt});
	return true;
}

void editor_mode::model_turnout_run(layout::command const &Command)
{
	auto &tool{m_switch};
	model_context context;
	context.done = [this](std::vector<TTrack *> const &Created) { model_switch_done(Created); };
	model_run({Command}, context);
	if (model_refused())
		tool.status = switch_refused(model_shown());
}

void editor_mode::model_turnout_remove(layout::turnout_id const Turnout)
{
	auto const *turnout{model_network().find(Turnout)};
	if (turnout == nullptr)
		return;
	std::vector<std::string> names;
	for (auto const *track : model_line_tracks(turnout->main))
		names.push_back(track->name());
	model_check_users(names, [this, Turnout]() {
		model_context context;
		context.done = [this](std::vector<TTrack *> const &) {
			m_node = nullptr;
			ui()->set_node(nullptr);
			m_track_point = {};
			m_switch.status = model_shown();
		};
		model_run({layout::remove_turnout{Turnout}}, context);
		if (model_refused())
			m_switch.status = switch_refused(model_shown());
	});
}

bool editor_mode::model_connection()
{
	auto &tool{m_switch};
	if (false == model_on() || tool.curved || tool.connection != static_cast<int>(connection::kind::crossover) || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()) || tool.line.tracks.empty())
		return false;
	layout::line_id from{layout::line_id::none};
	double chainage{0.0};
	if (false == model_line_at(tool.point, tool.line.tracks, from, chainage))
		return false;
	// the line beside, where the drag is let go
	std::vector<TTrack const *> const exclude(tool.line.tracks.begin(), tool.line.tracks.end());
	auto const other{straight_near(tool.mouse, exclude, 8.0)};
	layout::line_id to{layout::line_id::none};
	double target{0.0};
	auto const across{other.start + glm::dvec3{other.direction.x, other.grade, other.direction.y} * std::clamp(other.along(tool.mouse), 0.0, other.length)};
	if (other.tracks.empty() || false == model_line_at(across, other.tracks, to, target))
	{
		// the other track an old one: no crossover of the model, the old way
		return false;
	}
	tool.placing = false;
	layout::connect_tracks command;
	command.from = from;
	command.chainage = chainage;
	command.to = to;
	command.type = model_type_of(tool.templates[tool.armed]);
	if (command.type.empty())
	{
		tool.status = switch_refused(format(STR_C("%s isn't of the catalogue of the turnouts: a line of the model takes the types of the catalogue"), template_name(tool.templates[tool.armed]).c_str()));
		return true;
	}
	glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
	auto const *line{model_network().find(from)};
	command.direction = line != nullptr && glm::dot(offset, line->solved.tangent(chainage)) < 0.0 ? -1 : 1;
	// the solver's plan first: refused, its best proposal told with the reasons
	auto const plan{layout::plan_connection(model_network(), command)};
	model_context context;
	context.done = [this](std::vector<TTrack *> const &Created) { model_switch_done(Created); };
	model_run({command}, context);
	if (model_refused())
	{
		auto shown{model_shown()};
		if (false == plan.proposal.empty())
			shown += "\n" + format(STR_C("Proposal: %s"), model_tr(plan.proposal).c_str());
		tool.status = switch_refused(shown);
		model().proposal = plan.proposal;
	}
	WriteLog(format("Editor: model: a crossover %s from line %s at %.3f m to line %s: %s%s", command.type.c_str(), line_name(from).c_str(), chainage, line_name(to).c_str(), editor_selftest::one_line(model_message()).c_str(),
	                plan.proposal.empty() ? "" : (" · proposal: " + plan.proposal).c_str()),
	         logtype::generic);
	return true;
}

std::string editor_mode::model_switch_trial() const
{
	// what the switch dragged on a line of the model comes to as it's let go: the first reason it's refused for, empty where it's laid
	std::string why;
	auto const command{model_switch_drag(why)};
	if (false == command.has_value())
		return {};
	if (false == why.empty())
		return why;
	auto &memo{model().drag};
	auto const key{format("%u %.2f %s %d %d", layout::number(command->line), command->chainage, command->type.c_str(), command->hand, command->direction)};
	if (memo.first != key)
	{
		auto const effect{model().session.trial(*command)};
		memo = {key, effect.outcome.done ? std::string{} : model_reasons_shown(effect.outcome)};
	}
	return memo.second;
}

void editor_mode::render_model_turnout(TTrack &Track)
{
	auto const *turnout{model_turnout(&Track)};
	if (turnout == nullptr)
		return;
	auto &tool{m_switch};
	if (false == tool.collected)
	{
		tool.templates = editor_track::standard_switch_templates();
		tool.collected = true;
	}
	auto const id{turnout->id};
	ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.55f, 1.0f), "%s", format(STR_C("Turnout %s (managed)"), turnout_name(id).c_str()).c_str());
	item_tooltip("A turnout of the model of the track geometry: moved, given another type or side, or taken out by its commands, checked against the norms; its tracks made anew");
	auto summary{format(STR_C("%s · %s · %.0f / %.0f km/h"), turnout->type.c_str(), turnout->hand > 0 ? STR_C("right") : STR_C("left"), turnout->speed_main, turnout->speed_diverging)};
	if (turnout->bent())
		summary += format(STR_C(" · bent R %.0f m"), std::abs(turnout->main_radius));
	ImGui::TextDisabled("%s", summary.c_str());
	// the type: the ones of the catalogue of the same kind
	auto const kind{layout::kind_of(*turnout)};
	int current{-1};
	for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
		if (model_type_of(tool.templates[i]) == turnout->type)
		{
			current = i;
			break;
		}
	auto const fits = [this, kind](editor_track::switch_template const &Template) {
		auto const type{model_type_of(Template)};
		auto const known{type.empty() ? std::nullopt : layout::turnout_type_of(type)};
		return known.has_value() && known->kind == kind;
	};
	if (auto const picked{render_template_picker("##modelturnout", fits, editor_track::rail_of(&Track), current, false)}; picked >= 0 && picked != current)
	{
		model_switch_template(Track, tool.templates[picked]);
		return;
	}
	if (ImGui::Button(STR_C("Other side")))
	{
		model_turnout_run(layout::change_turnout{id, turnout->type, -turnout->hand});
		return;
	}
	item_tooltip("The diverging track to the other side, the start of the turnout where it is");
	ImGui::SameLine();
	auto &distance{model().turnout_move};
	ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5.0f);
	ImGui::InputDouble("##modelturnoutmove", &distance, 0.0, 0.0, "%.1f m");
	ImGui::SameLine();
	if (ImGui::Button(STR_C("Move")) && std::abs(distance) > 0.005)
	{
		model_turnout_run(layout::move_turnout{id, distance});
		return;
	}
	item_tooltip("Moved along its line by the distance, above 0 toward the end of its main track: checked as when it's laid");
	ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f, 0.18f, 0.15f, 1.0f));
	if (ImGui::Button(STR_C("Take the turnout out")))
	{
		ImGui::PopStyleColor();
		model_turnout_remove(id);
		return;
	}
	ImGui::PopStyleColor();
	item_tooltip("Its diverging track goes, the line under it is one line again where it can be. Ctrl+Z brings it back");
	if (false == tool.status.empty())
		editor_flow::message_line(tool.status, tool.status == tool.refusal ? editor_flow::kRefused : ImVec4(0.7f, 0.7f, 0.7f, 1.0f));
}
