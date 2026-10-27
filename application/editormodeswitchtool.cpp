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

std::string template_tooltip(editor_track::switch_template const &Template)
{
	if (Template.source != nullptr)
		return Template.label;
	auto text{switch_name_translated(Template.title())};
	if (Template.double_slip)
	{
		text += "\n";
		text += Template.type == "Rkp" ? STR_C("Single slip (Rkp)") : Template.type == "Rkpd" ? STR_C("Double slip (Rkpd)") : STR_C("Crossing (Skrz)");
		text += "\n" + format(STR_C("a %.3f m from the middle of the crossing to the joint"), Template.a) + "\n";
		if (Template.type == "Skrz")
			text += STR_C("Drag at a crossing of two straights");
		else
			text += Template.type == "Rkp" ? STR_C("Drag at a crossing of two straights at its skos, on the side of the tracks to join") : STR_C("Drag at a crossing of two straights at its skos");
		return text;
	}
	text += "\n" + format(STR_C("a %.3f m to the centre of the switch, b %.3f m on to its end"), Template.a, Template.b);
	if (auto const note{Template.label.rfind(" (")}; false == Template.own && note != std::string::npos && Template.label.back() == ')')
		text += " " + STR(Template.label.substr(note + 1));
	return text;
}

// the angle of the template at a glance, steeper than it is so 1:9 and 1:18.5 differ: the main track and the diverging
// one, or the two crossing tracks of a slip or a crossing
void template_glyph(ImDrawList &Draw, ImVec2 const Low, ImVec2 const High, editor_track::switch_template const &Template, bool const Chosen)
{
	auto const line{ImGui::GetColorU32(ImGuiCol_TextDisabled)};
	auto const made{Chosen ? IM_COL32(255, 255, 255, 255) : IM_COL32(120, 200, 255, 255)};
	auto const ratio{Template.ratio > 0.0 ? Template.ratio : Template.radius > 0.0 && Template.length > 0.0 ? 2.0 * Template.radius / Template.length : 9.0};
	auto const slope{static_cast<float>(std::min(0.9, 3.0 / ratio))};
	auto const width{High.x - Low.x};
	auto const middle{std::round((Low.y + High.y) * 0.5f)};
	if (Template.double_slip)
	{
		auto const rise{std::min(width * slope * 0.5f, (High.y - Low.y) * 0.5f)};
		Draw.AddLine(ImVec2(Low.x, middle), ImVec2(High.x, middle), line, 2.0f);
		Draw.AddLine(ImVec2(Low.x, middle + rise), ImVec2(High.x, middle - rise), made, 2.0f);
		return;
	}
	auto const bottom{std::round(High.y - 2.0f)};
	auto const rise{std::min(width * 0.75f * slope, High.y - Low.y - 4.0f)};
	Draw.AddLine(ImVec2(Low.x, bottom), ImVec2(High.x, bottom), line, 2.0f);
	Draw.AddBezierQuadratic(ImVec2(Low.x + width * 0.15f, bottom), ImVec2(Low.x + width * 0.5f, bottom), ImVec2(High.x, bottom - rise), made, 2.0f);
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
	if (tool.connection >= 0)
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

	// what to place, each with the templates and the options of its own only
	struct choice
	{
		switch_kind kind;
		char const *label;
		char const *tooltip;
	};
	choice const choices[] = {{switch_kind::single, "Switch", "A switch in a straight or a curve"},
	                          {switch_kind::slip, "Slip switch", "Rkp or Rkpd at a crossing of two straights at its skos, or bent into a curve of a side track"},
	                          {switch_kind::crossing, "Crossing", "A crossing of two straights"},
	                          {switch_kind::connection, "Connection", "Crossover, double, scissors, ladder"}};
	{
		auto const gap{std::round(ImGui::GetFontSize() * 0.4f)};
		auto const width{std::floor((ImGui::GetContentRegionAvail().x - gap) * 0.5f)};
		int shown{0};
		for (auto const &entry : choices)
		{
			if (entry.kind != switch_kind::single && false == switch_kind_offered(entry.kind))
				continue;
			if (shown % 2 == 1)
				ImGui::SameLine(0.0f, gap);
			else if (shown > 0)
				ImGui::Dummy(ImVec2(0.0f, gap * 0.25f));
			if (kind_tile(entry.label, STR_C(entry.label), static_cast<int>(&entry - choices), kind == entry.kind, width) && kind != entry.kind)
				choose_switch_kind(entry.kind);
			item_tooltip(entry.tooltip);
			++shown;
		}
	}
	if (kind == switch_kind::connection)
	{
		ImGui::SeparatorText(STR_C("Kind of the connection"));
		auto const gap{std::round(ImGui::GetFontSize() * 0.4f)};
		auto const width{std::floor((ImGui::GetContentRegionAvail().x - gap) * 0.5f)};
		for (int i = 0; i < connection::kind_count; ++i)
		{
			if (i % 2 == 1)
				ImGui::SameLine(0.0f, gap);
			else if (i > 0)
				ImGui::Dummy(ImVec2(0.0f, gap * 0.25f));
			ImGui::PushID(i);
			if (kind_tile("##connectionkind", connection_name(i), 4 + i, tool.connection == i, width))
			{
				tool.connection = i;
				tool.connection_kind = i;
			}
			ImGui::PopID();
		}
	}

	// how to place it with the mouse
	std::vector<std::pair<char const *, char const *>> how;
	switch (kind)
	{
	case switch_kind::slip: how = {{"Drag at a crossing", "of two straights at the skos of the slip: it goes there"}, {"Drag on a curve", "bent into it, in a side track only (11.4 pkt 4b)"}}; break;
	case switch_kind::crossing: how = {{"Drag at a crossing", "of two straights: it goes there"}}; break;
	case switch_kind::connection:
		how = {{"Drag", "from a straight onto the other track"}, {"Shift+wheel", "the kind"}, {"Ctrl+wheel", "R of the aligning curve"}};
		if (tool.connection == static_cast<int>(connection::kind::double_crossover))
			how.push_back({"Alt+wheel", "where they meet"});
		break;
	default: how = {{"Drag on a track", "along: the direction, sideways: the side"}, {"Drag on the ground", "a switch with no track, on from a free end near it"}}; break;
	}
	gesture_rows(how);

	// the templates of the kind, by their names, for the rail of the selected track unless all are asked for; the lengths
	// and the rest of the label in the tooltip
	ImGui::TextDisabled("%s", STR_C(kind == switch_kind::connection ? "The first switch" : "Template"));
	auto const rail{editor_track::rail_of(selected_track())};
	if (false == rail.empty())
	{
		ImGui::SameLine();
		auto const label{format(STR_C("Rail of the track: %s"), rail.c_str())};
		toggle_chips("railonly", {{label.c_str(), &tool.rail_only, "Only the templates of the rail the selected track is laid with"}});
	}
	else if (auto *track{selected_track()}; track != nullptr && (track->iCategoryFlag & 15) == 1 && track->m_profile1.first.empty())
		render_rail_profile_hint(*track);
	auto const shown = [&](editor_track::switch_template const &Template) {
		return of_kind(Template, static_cast<int>(kind)) && (rail.empty() || false == tool.rail_only || Template.source != nullptr || Template.rail == rail || Template.rail == "-");
	};
	// in groups: the switches of the scenery, the own templates, then the catalogue rail by rail
	auto const group_of = [](editor_track::switch_template const &Template) {
		return Template.source != nullptr ? std::string{STR("From the scenery")} : Template.own ? std::string{STR("Own templates")} : "PLK " + (Template.rail == "-" ? std::string{} : Template.rail);
	};
	std::vector<std::string> groups;
	int listed{0};
	for (auto const &entry : tool.templates)
		if (shown(entry))
		{
			++listed;
			if (auto const group{group_of(entry)}; std::find(groups.begin(), groups.end(), group) == groups.end())
				groups.push_back(group);
		}
	auto const row{std::round(ImGui::GetTextLineHeight() * 1.3f)};
	auto const header{groups.size() > 1 ? ImGui::GetTextLineHeightWithSpacing() * static_cast<float>(groups.size()) : 0.0f};
	ImGui::BeginChild("##switchtemplates", ImVec2(0.0f, std::min(std::round(ImGui::GetFontSize() * 18.0f), (listed + 0.5f) * (row + ImGui::GetStyle().ItemSpacing.y) + header + 8.0f)), ImGuiChildFlags_Borders);
	auto const glyph{std::round(row * 1.8f)};
	for (auto const &group : groups)
	{
		if (groups.size() > 1)
			ImGui::SeparatorText(group.c_str());
		for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
		{
			auto const &entry{tool.templates[i]};
			if (false == shown(entry) || group_of(entry) != group)
				continue;
			ImGui::PushID(i);
			auto const low{ImGui::GetCursorScreenPos()};
			if (ImGui::Selectable("##switch", tool.armed == i, ImGuiSelectableFlags_None, ImVec2(0.0f, row)))
			{
				tool.last = i;
				tool.armed = i;
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", template_tooltip(entry).c_str());
			if (ImGui::IsItemVisible())
			{
				auto &draw{*ImGui::GetWindowDrawList()};
				template_glyph(draw, ImVec2(low.x + 2.0f, low.y + 2.0f), ImVec2(low.x + glyph, low.y + row - 2.0f), entry, tool.armed == i);
				auto const label{entry.source != nullptr ? entry.title() + "  (" + std::to_string(entry.count) + "x)" : switch_name_translated(entry.title())};
				draw.AddText(ImVec2(low.x + glyph + ImGui::GetStyle().ItemSpacing.x, std::round(low.y + (row - ImGui::GetTextLineHeight()) * 0.5f)), ImGui::GetColorU32(ImGuiCol_Text), label.c_str());
			}
			ImGui::PopID();
		}
	}
	ImGui::EndChild();
	if (kind == switch_kind::connection && tool.connection == static_cast<int>(connection::kind::ladder) && begin_fields("##connectionladder"))
	{
		field_row("Spacings, m##connectionspacings");
		ImGui::InputText("##connectionspacings", tool.connection_spacings, sizeof(tool.connection_spacings));
		item_tooltip("Distances between the tracks of the fan from this track on, such as 4.75;4.75;5.0; used when the drag doesn't end on a parallel track, otherwise the parallel tracks up to that one make the fan");
		ImGui::EndTable();
	}
	if (kind == switch_kind::connection && tool.connection == static_cast<int>(connection::kind::double_crossover))
	{
		ImGui::TextDisabled("%s", STR_C("Where the two crossovers meet"));
		int together{tool.connection_together ? 0 : 1};
		if (segmented("connectiontogether", together,
		              {{0, STR_C("On this track"), "The two crossovers meet on the track the drag starts on; Alt+wheel while dragging switches it"},
		               {1, STR_C("On the other track"), "The two crossovers meet on the other track; Alt+wheel while dragging switches it"}}))
			tool.connection_together = together == 0;
	}

	ImGui::PushID("switchmore");
	if (ImGui::TreeNode(STR_C("More settings")))
	{
		if (kind == switch_kind::connection)
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
		ImGui::TreePop();
	}
	ImGui::PopID();
	if (false == tool.status.empty())
	{
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", tool.status.c_str());
		ImGui::PopTextWrapPos();
	}

	if (ImGui::Button(STR_C("Template editor...")))
		m_switch_designer.open = true;
	item_tooltip("Own switch templates: diverging tracks of straights, arcs and transition curves, double switches");
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
