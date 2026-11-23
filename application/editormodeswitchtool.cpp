/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the panel of the switch tool: what it places, the templates and the options of that only, and how to place it

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editorgestures.h"
#include "application/editormodeshared.h"
#include "application/editorflow.h"
#include "editor/editorConnections.hpp"
#include "editor/editorFormat.hpp"
#include "editor/editorTrack.hpp"
#include "simulation/simulation.h"
#include "utilities/translation.h"
#include "world/Track.h"
#include "world/TrackSleepers.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <string>

// the name of the template in the language of the editor: the words of the catalogue translated, the kind of a slip or
// a crossing left out where its type says it
std::string switch_name_translated(std::string Title)
{
	auto const replace = [&](std::string const &From, std::string const &To) {
		for (auto at{Title.find(From)}; at != std::string::npos; at = Title.find(From, at + To.size()))
			Title.replace(at, From.size(), To);
	};
	for (auto const *redundant : {" single slip", " double slip", " crossing"})
		replace(redundant, "");
	for (auto const *word : {"the blades outside of the diamond", "movable nose", "long", "industrial"})
		replace(word, STR(word));
	if (auto const at{Title.find("gauge ")}; at != std::string::npos)
		if (auto const gauge{std::atoi(Title.c_str() + at + 6)}; gauge > 0)
		{
			auto const end{Title.find_first_not_of("0123456789", at + 6)};
			Title.replace(at, (end == std::string::npos ? Title.size() : end) - at, format(STR_C("gauge %d"), gauge));
		}
	return Title;
}

namespace
{

// a kind to place as a tile: a sketch of its tracks on the left, its name beside it; true when clicked
bool kind_tile(char const *Id, char const *Label, int const Picture, bool const Chosen, float const Width)
{
	auto const font{ImGui::GetFontSize()};
	auto const height{std::round(font * 2.4f)};
	auto const top{ImGui::GetCursorScreenPos()};
	auto const clicked{ImGui::InvisibleButton(Id, ImVec2(Width, height))};
	auto const hovered{ImGui::IsItemHovered()};
	auto *list{ImGui::GetWindowDrawList()};
	auto const accent{family_colour(0)};
	ImVec2 const bottom{top.x + Width, top.y + height};
	auto const rounding{std::round(font * 0.35f)};
	list->AddRectFilled(top, bottom, Chosen ? (accent & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 46) : ImGui::GetColorU32(hovered ? ImGuiCol_FrameBgHovered : ImGuiCol_FrameBg), rounding);
	list->AddRect(top, bottom, Chosen ? accent : ImGui::GetColorU32(ImGuiCol_Border), rounding, 0, Chosen ? 2.0f : 1.0f);
	auto const pad{std::round(font * 0.4f)};
	auto const w{std::round(font * 2.2f)};
	auto const h{height - 2.0f * pad};
	ImVec2 const o{top.x + pad, top.y + pad};
	auto const at = [&](float const X, float const Y) { return ImVec2(o.x + X * w, o.y + Y * h); };
	auto const line{Chosen ? accent : ImGui::GetColorU32(ImGuiCol_Text, 0.7f)};
	auto const branch{Chosen ? IM_COL32(255, 210, 60, 255) : ImGui::GetColorU32(ImGuiCol_Text, 0.45f)};
	switch (Picture)
	{
	case 0: // a switch: the straight track and the diverging one
		list->AddLine(at(0.0f, 0.75f), at(1.0f, 0.75f), line, 2.0f);
		list->AddBezierQuadratic(at(0.15f, 0.75f), at(0.6f, 0.75f), at(1.0f, 0.1f), branch, 2.0f);
		break;
	case 1: // a slip switch: a crossing with the slip curves inside
		list->AddLine(at(0.0f, 0.85f), at(1.0f, 0.15f), line, 2.0f);
		list->AddLine(at(0.0f, 0.15f), at(1.0f, 0.85f), line, 2.0f);
		list->AddBezierQuadratic(at(0.08f, 0.8f), at(0.5f, 0.45f), at(0.92f, 0.8f), branch, 1.5f);
		list->AddBezierQuadratic(at(0.08f, 0.2f), at(0.5f, 0.55f), at(0.92f, 0.2f), branch, 1.5f);
		break;
	case 2: // a crossing
		list->AddLine(at(0.0f, 0.85f), at(1.0f, 0.15f), line, 2.0f);
		list->AddLine(at(0.0f, 0.15f), at(1.0f, 0.85f), line, 2.0f);
		break;
	case 5: // a double crossover: two crossovers facing each other
		list->AddLine(at(0.0f, 0.2f), at(1.0f, 0.2f), line, 2.0f);
		list->AddLine(at(0.0f, 0.8f), at(1.0f, 0.8f), line, 2.0f);
		list->AddLine(at(0.05f, 0.8f), at(0.45f, 0.2f), branch, 2.0f);
		list->AddLine(at(0.55f, 0.2f), at(0.95f, 0.8f), branch, 2.0f);
		break;
	case 6: // scissors: two crossovers through each other
		list->AddLine(at(0.0f, 0.2f), at(1.0f, 0.2f), line, 2.0f);
		list->AddLine(at(0.0f, 0.8f), at(1.0f, 0.8f), line, 2.0f);
		list->AddLine(at(0.15f, 0.8f), at(0.85f, 0.2f), branch, 2.0f);
		list->AddLine(at(0.15f, 0.2f), at(0.85f, 0.8f), branch, 2.0f);
		break;
	case 7: // a ladder: the tracks of a fan going off one by one
		list->AddLine(at(0.0f, 0.9f), at(1.0f, 0.9f), line, 2.0f);
		for (float const y : {0.6f, 0.35f, 0.1f})
			list->AddLine(at(0.95f - y, y), at(1.0f, y), line, 2.0f);
		list->AddLine(at(0.05f, 0.9f), at(0.85f, 0.1f), branch, 2.0f);
		break;
	default: // a connection of two tracks
		list->AddLine(at(0.0f, 0.2f), at(1.0f, 0.2f), line, 2.0f);
		list->AddLine(at(0.0f, 0.8f), at(1.0f, 0.8f), line, 2.0f);
		list->AddLine(at(0.2f, 0.8f), at(0.8f, 0.2f), branch, 2.0f);
		break;
	}
	auto const text_left{o.x + w + pad};
	auto const wrap{bottom.x - pad - text_left};
	auto const size{ImGui::CalcTextSize(Label, nullptr, false, wrap)};
	list->AddText(nullptr, 0.0f, ImVec2(text_left, std::round(top.y + (height - size.y) * 0.5f)), ImGui::GetColorU32(Chosen ? ImGuiCol_Text : ImGuiCol_TextDisabled), Label, nullptr, wrap);
	return clicked;
}

// whether the template is one of those the kind places
bool of_kind(editor_track::switch_template const &Template, int const Kind)
{
	switch (Kind)
	{
	case 1: return Template.double_slip && Template.type != "Skrz";
	case 2: return Template.double_slip && Template.type == "Skrz";
	case 3: return false == Template.double_slip && false == Template.twofold();
	default: return false == Template.double_slip;
	}
}

} // namespace

std::string editor_mode::switch_template_label(editor_track::switch_template const &Template)
{
	if (Template.source != nullptr)
		return format("%s: %s  (%dx)", STR_C("From the scenery"), Template.title().c_str(), Template.count);
	return (Template.own ? STR("Own: ") : std::string{"PLK: "}) + switch_name_translated(Template.title());
}

editor_mode::switch_kind editor_mode::current_switch_kind() const
{
	auto const &tool{m_switch};
	if (tool.connection >= 0 && false == tool.recognised)
		return switch_kind::connection;
	if (tool.armed >= 0 && tool.armed < static_cast<int>(tool.templates.size()) && tool.templates[tool.armed].double_slip)
		return tool.templates[tool.armed].type == "Skrz" ? switch_kind::crossing : switch_kind::slip;
	return switch_kind::single;
}

bool editor_mode::switch_kind_offered(switch_kind const Kind) const
{
	auto const &templates{m_switch.templates};
	return std::any_of(templates.begin(), templates.end(), [&](editor_track::switch_template const &Template) { return of_kind(Template, static_cast<int>(Kind)); });
}

void editor_mode::choose_switch_kind(switch_kind const Kind)
{
	auto &tool{m_switch};
	tool.connection = (Kind == switch_kind::connection ? std::clamp(tool.connection_kind, 0, connection::kind_count - 1) : -1);
	// the template chosen stays if it's of the kind, else the first one of the kind
	auto const fits = [&](int const Index) { return Index >= 0 && Index < static_cast<int>(tool.templates.size()) && of_kind(tool.templates[Index], static_cast<int>(Kind)); };
	if (fits(tool.armed))
		return;
	for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
		if (fits(i))
		{
			tool.last = i;
			tool.armed = i;
			tool.status.clear();
			return;
		}
	tool.armed = -1;
}

void editor_mode::render_switch_ui()
{
	auto &tool{m_switch};
	auto const kind{current_switch_kind()};
	// dragged: what it lays as it's let go, and what keeps it from being laid
	if (tool.placing)
	{
		render_switch_drag();
		return;
	}
	char const *step{STR_C("Press on a straight and drag along it and to the side")};
	if (kind == switch_kind::slip)
		step = STR_C("Press on a straight or at a crossing and drag");
	else if (kind == switch_kind::crossing)
		step = STR_C("Press at a crossing of two straights and drag");
	else if (kind == switch_kind::connection)
		step = STR_C("Press on a straight and drag onto the other track");
	editor_flow::step_line(step);
	// a switch refused: why, in one message in red under the step, where it's read first
	if (false == tool.status.empty())
		editor_flow::message_line(tool.status, tool.status == tool.refusal ? editor_flow::kRefused : ImVec4(1.0f, 0.7f, 0.2f, 1.0f));
	render_switch_kinds(kind);

	// the template: the rail and the speed on the diverging track as chips, the one they pick as a line which opens the list of them all;
	// the lengths and the rest of the label in the tooltip
	auto const rail{editor_track::rail_of(selected_track())};
	if (false == rail.empty())
		ImGui::TextDisabled(STR_C("rail of the track %s"), rail.c_str());
	else if (auto *track{selected_track()}; track != nullptr && (track->iCategoryFlag & 15) == 1 && track->m_profile1.first.empty())
	{
		ImGui::TextDisabled("%s", STR_C("Template"));
		render_rail_profile_hint(*track);
	}
	if (auto const picked{render_template_picker("##place", [kind](editor_track::switch_template const &Template) { return of_kind(Template, static_cast<int>(kind)); }, rail, tool.armed, false)}; picked >= 0)
	{
		tool.last = picked;
		tool.armed = picked;
	}

	ImGui::PushID("switchmore");
	if (ImGui::TreeNode(STR_C("More settings")))
	{
		if (kind == switch_kind::single || kind == switch_kind::connection)
			render_connection_options();
		// the name and the drive of the next switch; a crossing has no moving parts
		if (kind != switch_kind::crossing)
		{
			if (begin_fields("##switchnamefields", 0.3f))
			{
				field_row("Name##switchname");
				ImGui::InputTextWithHint("##switchname", switch_name_for_new().c_str(), tool.name, sizeof(tool.name));
				item_tooltip("Name of the next switch, its drive events are named after it; empty: the next free z1, z2...; a number at the end goes up with each switch");
				ImGui::EndTable();
			}
			if (render_switch_drive_choice())
				if (auto *track{selected_track()}; track != nullptr && track->eType == tt_Switch && false == track->m_editorremoved)
					apply_switch_drive_choice(*track);
		}
		if (ImGui::Button(STR_C("Collect from the scenery")))
		{
			tool.templates = editor_track::standard_switch_templates();
			auto const found{editor_track::find_switch_templates()};
			tool.templates.insert(tool.templates.end(), found.begin(), found.end());
			tool.collected = true;
			arm_switch();
		}
		item_tooltip("Looks for the switches of the scenery, to use their shapes as the templates");
		render_switch_timbers();
		if (ImGui::Button(STR_C("Template editor...")))
			m_switch_designer.open = true;
		item_tooltip("Own switch templates: diverging tracks of straights, arcs and transition curves, double switches");
		ImGui::TreePop();
	}
	ImGui::PopID();
	render_stray_drives();
}

void editor_mode::render_switch_kinds(switch_kind const Kind)
{
	// what to place, each with the templates and the options of its own only; the slips and the crossings where there are templates of them
	segment_option const single{static_cast<int>(switch_kind::single), STR_C("Switch"), "A switch in a straight or a curve"};
	segment_option const slip{static_cast<int>(switch_kind::slip), STR_C("Slip"),
	                          "Rkp or Rkpd dragged out on one straight with its crossing track, at a crossing of two straights at its skos, or bent into a curve of a side track"};
	segment_option const crossing{static_cast<int>(switch_kind::crossing), STR_C("Crossing"), "A crossing of two straights"};
	auto const slips{switch_kind_offered(switch_kind::slip)};
	auto const crossings{switch_kind_offered(switch_kind::crossing)};
	// a connection of tracks is chosen among the settings: none of these lit then
	int chosen{Kind == switch_kind::connection ? -1 : static_cast<int>(Kind)};
	bool changed{false};
	if (slips && crossings)
		changed = segmented("switchkind", chosen, {single, slip, crossing});
	else if (slips)
		changed = segmented("switchkind", chosen, {single, slip});
	else if (crossings)
		changed = segmented("switchkind", chosen, {single, crossing});
	if (changed)
		choose_switch_kind(static_cast<switch_kind>(chosen));
}

int editor_mode::switch_drag_side(std::string &Refusal) const
{
	auto const &tool{m_switch};
	Refusal.clear();
	if (tool.free)
	{
		if (free_bend bend; free_switch_bend(bend))
		{
			if (false == bend.report.refusals.empty())
				Refusal = bend.report.refusals.front();
			else if (false == bend.problem.empty())
				Refusal = bend.problem;
			return bend.side;
		}
		glm::dvec3 start;
		glm::dvec2 heading;
		int side{1};
		double grade{0.0};
		free_switch_pose(start, heading, side, grade);
		return side;
	}
	if (tool.curved)
	{
		int forward{1};
		int side{1};
		curved_switch_pose(forward, side);
		if (tool.connection >= 0 || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()) || tool.templates[tool.armed].double_slip)
			return side;
		if (auto const preview{switch_preview()}; preview.size() >= 2)
		{
			auto const report{curved_switch_report(preview, tool.frame, curved_switch_start(forward), forward, &tool.frame_tracks)};
			if (false == report.refusals.empty())
				Refusal = report.refusals.front();
		}
		return side;
	}
	glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
	auto const forward{glm::dot(offset, tool.line.direction) >= 0.0 ? 1 : -1};
	glm::dvec2 const direction{tool.line.direction * static_cast<double>(forward)};
	return direction.x * offset.y - direction.y * offset.x >= 0.0 ? 1 : -1;
}

void editor_mode::render_switch_drag()
{
	auto const &tool{m_switch};
	editor_flow::step_line(STR_C("Let go to insert"));
	if (tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return;
	// the length, the side of the diverging track, the speed on it; what keeps it from being laid in one line in red, what it's moved by
	// to stay on its track in orange
	auto const &shape{tool.templates[tool.armed]};
	std::string refusal;
	auto const side{switch_drag_side(refusal)};
	auto line{format("%.1f m · %s", shape.length, side > 0 ? STR_C("right") : STR_C("left"))};
	if (auto const speed{shape.radius > 0.0 ? editor_track::diverging_speed(shape.radius) : 0.0}; speed > 0.0)
		line += format(" · %.0f km/h", speed);
	ImGui::TextUnformatted(line.c_str());
	// on a line of the model: what the model says of the turnout let go here
	if (std::string why; model_switch_drag(why).has_value())
	{
		if (auto const trial{model_switch_trial()}; false == trial.empty())
			editor_flow::message_line(trial, editor_flow::kRefused);
		return;
	}
	if (false == refusal.empty())
		editor_flow::message_line(refusal, editor_flow::kRefused);
	else if (auto const problem{switch_drag_problem()}; problem.has_value())
		editor_flow::message_line(problem->text, ImVec4(1.0f, 0.7f, 0.2f, 1.0f));
	// a branch let go at a track it doesn't fit into: the switch is laid, the branch not led into the track
	if (switch_link plan; false == tool.free && tool.connection < 0 && false == plan_switch_link(plan) && false == plan.reason.empty())
		editor_flow::message_line(plan.reason, ImVec4(1.0f, 0.7f, 0.2f, 1.0f));
}

bool editor_mode::just_laid(TTrack const *Track) const
{
	// the last step of the history, with those made together with it
	for (auto step{m_history.rbegin()}; step != m_history.rend(); ++step)
	{
		if (std::ranges::find(step->created, Track) != step->created.end())
			return true;
		if (false == step->joined)
			break;
	}
	return false;
}

// the timbers of all the switches and crossings of the scenery at once: a button for each model of timbers of the scenery data
void editor_mode::render_switch_timbers()
{
	ImGui::TextDisabled("%s", STR_C("Timbers of all the switches"));
	auto first{true};
	auto const button = [&](std::string const &Model, std::string const &Label) {
		if (false == first)
			ImGui::SameLine();
		first = false;
		if (ImGui::Button((Label + "##switchtimbers" + Model).c_str()))
			m_switch.status = format(STR_C("Timbers %s set on %d tracks"), Label.c_str(), static_cast<int>(set_switch_timbers(Model)));
		item_tooltip("Sets the model of the timbers, with the fastenings where it has them, on all the switches and crossings of the scenery, in one step of the undo");
	};
	for (auto const &model : track_sleepers::pack_models())
		if (model.timber)
			button(model.name, model.name.substr(model.name.find_last_of('/') + 1));
	button({}, STR_C("As in eu07.ini"));
}

std::size_t editor_mode::set_switch_timbers(std::string const &Model)
{
	std::vector<TTrack *> tracks;
	for (auto *track : simulation::Paths.sequence())
		if (track != nullptr && false == track->m_editorremoved && (track->iCategoryFlag & 15) == 1 && (track->eType == tt_Switch || false == track->m_crossings.empty())
		    && track->m_timber_model_name != Model)
			tracks.push_back(track);
	return tracks_apply(tracks, [&](TTrack &Track) { Track.m_timber_model_name = Model; return true; }, true, "Timber model");
}

// a track with no rail profile, as the sceneries of the older editors have them: the templates of all the rails are offered.
// the profile set at once on all the tracks laid with the same rails and no profile, so a scenery is put right in a few clicks
void editor_mode::render_rail_profile_hint(TTrack &Track)
{
	ImGui::SameLine();
	ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", STR_C("Track with no rail profile"));
	item_tooltip("The track doesn't say its rail, so the templates of all the rails are offered.\nSet the profile and the list keeps to the templates of the rail; the rails of the track take its shape");
	ImGui::SameLine();
	if (ImGui::SmallButton(STR_C("Set...##railprofile")))
		ImGui::OpenPopup("##railprofileset");
	if (false == ImGui::BeginPopup("##railprofileset"))
		return;
	auto const material{Track.m_material1};
	auto const same = [&](TTrack const &Other) { return false == Other.m_editorremoved && (Other.iCategoryFlag & 15) == 1 && Other.m_material1 == material && Other.m_profile1.first.empty(); };
	std::vector<TTrack *> tracks;
	for (auto *other : simulation::Paths.sequence())
		if (other != nullptr && same(*other))
			tracks.push_back(other);
	ImGui::TextUnformatted(format(STR_C("Tracks with the rails %s and no profile: %d"), editor_track::material_name(material).c_str(), static_cast<int>(tracks.size())).c_str());
	static std::pair<char const *, char const *> const profiles[] = {{"S60 (60E1)", "s60"}, {"S49 (49E1)", "s49"}, {"S42", "s42"}};
	for (auto const &[label, name] : profiles)
	{
		if (&label != &profiles[0].first)
			ImGui::SameLine();
		if (false == ImGui::Button(label))
			continue;
		auto const changed{tracks_apply(tracks, [&](TTrack &Other) { Other.rail_profile(name); return true; }, true, "Rail profile")};
		m_switch.status = format(STR_C("Rail profile %s set on %d tracks"), label, static_cast<int>(changed));
		ImGui::CloseCurrentPopup();
	}
	ImGui::TextDisabled("%s", STR_C("Other profiles: the properties of the track"));
	ImGui::EndPopup();
}
