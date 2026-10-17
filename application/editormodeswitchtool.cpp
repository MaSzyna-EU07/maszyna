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
#include "application/editormodeshared.h"
#include "editor/editorConnections.hpp"
#include "utilities/translation.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <string>

namespace
{

// the next radio button on the same line as the last item, if it fits there
void same_line_if_fits(char const *Label)
{
	auto const &style{ImGui::GetStyle()};
	auto const width{ImGui::GetFrameHeight() + style.ItemInnerSpacing.x + ImGui::CalcTextSize(Label, nullptr, true).x};
	auto const end{ImGui::GetItemRectMax().x + style.ItemSpacing.x + width};
	if (end <= ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x)
		ImGui::SameLine();
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
		return STR("Scenery: ") + Template.title() + "  (" + std::to_string(Template.count) + "x)";
	return (Template.own ? STR("Own: ") : std::string{"PLK: "}) + Template.title();
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
	choice const choices[] = {{switch_kind::single, "Switch", "A switch put into a straight or a curve"},
	                          {switch_kind::slip, "Slip switch", "A single or double slip switch at a crossing of two straights"},
	                          {switch_kind::crossing, "Crossing", "A crossing of two straights"},
	                          {switch_kind::connection, "Connection", "Switches laid whole between two tracks: a crossover, a double or a scissors crossover, a switch ladder"}};
	ImGui::TextDisabled("%s", STR_C("What to place"));
	auto first{true};
	for (auto const &entry : choices)
	{
		if (entry.kind != switch_kind::single && false == switch_kind_offered(entry.kind))
			continue;
		if (false == first)
			same_line_if_fits(STR_C(entry.label));
		first = false;
		if (ImGui::RadioButton(STR_C(entry.label), kind == entry.kind) && kind != entry.kind)
			choose_switch_kind(entry.kind);
		item_tooltip(entry.tooltip);
	}
	if (kind == switch_kind::connection)
	{
		for (int i = 0; i < connection::kind_count; ++i)
		{
			if (i > 0)
				same_line_if_fits(connection_name(i));
			ImGui::PushID(i);
			if (ImGui::RadioButton(connection_name(i), tool.connection == i))
			{
				tool.connection = i;
				tool.connection_kind = i;
			}
			ImGui::PopID();
		}
	}

	// how to place it with the mouse
	char const *how{nullptr};
	switch (kind)
	{
	case switch_kind::slip: how = "Click at a crossing of two straights; a single slip on the side of the joined tracks"; break;
	case switch_kind::crossing: how = "Click at a crossing of two straights"; break;
	case switch_kind::connection:
		how = tool.connection == static_cast<int>(connection::kind::double_crossover) ? "Press on a straight and drag onto the other track. While dragging: Shift+wheel another kind, Ctrl+wheel R of the aligning curve, Alt+wheel where they meet" :
		                                                                                 "Press on a straight and drag onto the other track. While dragging: Shift+wheel another kind, Ctrl+wheel R of the aligning curve";
		break;
	default: how = "Click a straight or a curve, then press on it and drag: along for the direction, sideways for the side"; break;
	}
	ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.8f, 0.3f, 1.0f));
	ImGui::TextWrapped("%s", STR_C(how));
	ImGui::PopStyleColor();

	// the templates of the kind, by their names; the lengths and the rest of the label in the tooltip
	ImGui::TextDisabled("%s", STR_C(kind == switch_kind::connection ? "The first switch" : "Template"));
	auto const listed{static_cast<int>(std::count_if(tool.templates.begin(), tool.templates.end(), [&](editor_track::switch_template const &Template) { return of_kind(Template, static_cast<int>(kind)); }))};
	ImGui::BeginChild("##switchtemplates", ImVec2(0.0f, std::min(240.0f, (listed + 0.5f) * ImGui::GetTextLineHeightWithSpacing() + 8.0f)), true);
	for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
	{
		auto const &entry{tool.templates[i]};
		if (false == of_kind(entry, static_cast<int>(kind)))
			continue;
		if (ImGui::Selectable((switch_template_label(entry) + "##switch" + std::to_string(i)).c_str(), tool.armed == i))
		{
			tool.last = i;
			tool.armed = i;
		}
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", entry.label.c_str());
	}
	ImGui::EndChild();
	if (kind == switch_kind::connection)
		render_connection_options();

	// the name and the drive of the next switch; a crossing has no moving parts
	if (kind != switch_kind::crossing)
	{
		ImGui::PushItemWidth(120.0f);
		ImGui::InputTextWithHint(STR_C("Name##switchname"), switch_name_for_new().c_str(), tool.name, sizeof(tool.name));
		ImGui::PopItemWidth();
		item_tooltip("Name of the next switch, its drive events are named after it; empty: the next free z1, z2...; a number at the end goes up with each switch");
		if (render_switch_drive_choice())
			if (auto *track{selected_track()}; track != nullptr && track->eType == tt_Switch && false == track->m_editorremoved)
				apply_switch_drive_choice(*track);
	}
	if (false == tool.status.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", tool.status.c_str());

	if (ImGui::SmallButton(STR_C("Collect from the scenery")))
	{
		tool.templates = editor_track::standard_switch_templates();
		auto const found{editor_track::find_switch_templates()};
		tool.templates.insert(tool.templates.end(), found.begin(), found.end());
		tool.collected = true;
		arm_switch();
	}
	item_tooltip("Looks for the switches of the scenery, to use their shapes as the templates");
	ImGui::SameLine();
	if (ImGui::SmallButton(STR_C("Template editor...")))
		m_switch_designer.open = true;
	item_tooltip("Own switch templates: diverging tracks of straights, arcs and transition curves, double switches");
}
