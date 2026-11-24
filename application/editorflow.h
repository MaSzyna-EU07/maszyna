/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// the flow of the panel of a tool: one line of what to do now at its top, the settings folded to a line of their summary while
// they aren't being changed, what's being made as one line of its quality with a badge, the details of it on demand

#include "application/editormodeshared.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>

namespace editor_flow
{

ImVec4 constexpr kGood{0.40f, 0.85f, 0.45f, 1.0f};
ImVec4 constexpr kBelow{1.0f, 0.58f, 0.18f, 1.0f};
ImVec4 constexpr kRefused{1.0f, 0.38f, 0.30f, 1.0f};

// the sign of a warning in the text by the cursor: the font has no glyph of its own for it, a triangle is drawn in its place
inline constexpr std::string_view kWarningMark{" \xE2\x9A\xA0"};

// a pencil, drawn: the sign of a line which opens what it sums up for editing
inline void draw_pencil(ImDrawList &List, ImVec2 const Centre, float const Size, ImU32 const Colour)
{
	auto const half{Size * 0.5f};
	auto const thick{std::max(2.0f, std::round(Size * 0.26f)) * 0.5f};
	float constexpr diagonal{0.70710678f};
	// along the diagonal from the tip at the lower left to the end at the upper right; across it
	ImVec2 const tip{Centre.x - half, Centre.y + half};
	ImVec2 const end{Centre.x + half, Centre.y - half};
	ImVec2 const across{diagonal * thick, diagonal * thick};
	ImVec2 const neck{tip.x + diagonal * Size * 0.32f, tip.y - diagonal * Size * 0.32f};
	List.AddQuadFilled(ImVec2(neck.x - across.x, neck.y - across.y), ImVec2(end.x - across.x, end.y - across.y), ImVec2(end.x + across.x, end.y + across.y), ImVec2(neck.x + across.x, neck.y + across.y), Colour);
	List.AddTriangleFilled(ImVec2(neck.x - across.x, neck.y - across.y), ImVec2(neck.x + across.x, neck.y + across.y), tip, Colour);
}

// the sign of a warning, drawn: a triangle with the mark in it
inline void draw_warning(ImDrawList &List, ImVec2 const Centre, float const Size, ImU32 const Colour)
{
	auto const half{Size * 0.5f};
	List.AddTriangleFilled(ImVec2(Centre.x, Centre.y - half), ImVec2(Centre.x + half * 1.1f, Centre.y + half * 0.85f), ImVec2(Centre.x - half * 1.1f, Centre.y + half * 0.85f), Colour);
	auto const ink{IM_COL32(20, 20, 20, 255)};
	List.AddLine(ImVec2(Centre.x, Centre.y - half * 0.3f), ImVec2(Centre.x, Centre.y + half * 0.3f), ink, std::max(1.0f, Size * 0.13f));
	List.AddCircleFilled(ImVec2(Centre.x, Centre.y + half * 0.58f), std::max(1.0f, Size * 0.07f), ink);
}

// the one line of what to do now, at the top of the panel of the tool: larger and white, with the accent of the tools of building at
// its left; the keys of the step under it, dimmed
inline void step_line(char const *Text, char const *Keys = nullptr)
{
	auto *list{ImGui::GetWindowDrawList()};
	auto const font{ImGui::GetFontSize()};
	auto const top{ImGui::GetCursorScreenPos()};
	auto const bar{std::max(3.0f, std::round(font * 0.22f))};
	auto const indent{bar + std::round(font * 0.5f)};
	ImGui::Indent(indent);
	ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
	ImGui::SetWindowFontScale(1.12f);
	ImGui::TextWrapped("%s", Text);
	ImGui::SetWindowFontScale(1.0f);
	ImGui::PopStyleColor();
	if (Keys != nullptr && *Keys != '\0')
	{
		ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
		ImGui::TextWrapped("%s", Keys);
		ImGui::PopStyleColor();
	}
	auto const bottom{ImGui::GetItemRectMax().y};
	ImGui::Unindent(indent);
	list->AddRectFilled(top, ImVec2(top.x + bar, bottom), family_colour(0), bar * 0.5f);
	ImGui::Spacing();
}

// settings folded to the line of their summary, a pencil at its end: a click on it opens them, another one folds them again; true when
// clicked
inline bool summary_line(char const *Id, std::string const &Text, bool const Open, char const *Tooltip = nullptr)
{
	auto const height{ImGui::GetFrameHeight()};
	auto const width{std::max(1.0f, ImGui::GetContentRegionAvail().x)};
	auto const at{ImGui::GetCursorScreenPos()};
	ImGui::PushID(Id);
	auto const clicked{ImGui::InvisibleButton("##summary", ImVec2(width, height))};
	ImGui::PopID();
	auto const hovered{ImGui::IsItemHovered()};
	if (hovered)
	{
		ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
		if (Tooltip != nullptr && *Tooltip != '\0')
			ImGui::SetTooltip("%s", Tooltip);
	}
	auto *list{ImGui::GetWindowDrawList()};
	list->AddRectFilled(at, ImVec2(at.x + width, at.y + height), ImGui::GetColorU32(hovered ? ImGuiCol_FrameBgHovered : ImGuiCol_FrameBg), std::round(height * 0.3f));
	auto const pad{std::round(ImGui::GetStyle().FramePadding.x)};
	auto const pencil{std::round(height * 0.5f)};
	auto const right{at.x + width - pad - pencil};
	list->PushClipRect(at, ImVec2(right - pad, at.y + height), true);
	list->AddText(ImVec2(at.x + pad, std::round(at.y + (height - ImGui::GetTextLineHeight()) * 0.5f)), ImGui::GetColorU32(ImGuiCol_Text), Text.c_str());
	list->PopClipRect();
	auto const colour{Open ? family_colour(0) : ImGui::GetColorU32(hovered ? ImGuiCol_Text : ImGuiCol_TextDisabled)};
	draw_pencil(*list, ImVec2(right + pencil * 0.5f, at.y + height * 0.5f), pencil, colour);
	return clicked;
}

// a link which opens what it names under it and folds it again, a marker after it pointing right while folded, down while open; true
// when clicked
inline bool fold_link(char const *Id, char const *Text, bool &Open)
{
	auto const size{ImGui::CalcTextSize(Text)};
	auto const mark{std::round(ImGui::GetFontSize() * 0.3f)};
	auto const height{ImGui::GetFrameHeight()};
	auto const at{ImGui::GetCursorScreenPos()};
	ImGui::PushID(Id);
	auto const clicked{ImGui::InvisibleButton("##fold", ImVec2(size.x + mark * 3.5f, height))};
	ImGui::PopID();
	auto const hovered{ImGui::IsItemHovered()};
	if (hovered)
		ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
	auto *list{ImGui::GetWindowDrawList()};
	auto const colour{ImGui::GetColorU32(hovered || Open ? ImGuiCol_Text : ImGuiCol_TextDisabled)};
	auto const y{std::round(at.y + (height - size.y) * 0.5f)};
	list->AddText(ImVec2(at.x, y), colour, Text);
	list->AddLine(ImVec2(at.x, y + size.y), ImVec2(at.x + size.x, y + size.y), (colour & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 110));
	ImVec2 const centre{at.x + size.x + mark * 2.0f, at.y + height * 0.5f};
	if (Open)
		list->AddTriangleFilled(ImVec2(centre.x - mark, centre.y - mark * 0.6f), ImVec2(centre.x + mark, centre.y - mark * 0.6f), ImVec2(centre.x, centre.y + mark * 0.8f), colour);
	else
		list->AddTriangleFilled(ImVec2(centre.x - mark * 0.6f, centre.y - mark), ImVec2(centre.x - mark * 0.6f, centre.y + mark), ImVec2(centre.x + mark * 0.8f, centre.y), colour);
	if (clicked)
		Open = !Open;
	return clicked;
}

// the quality of what's being made at a glance: a badge, green where it meets what it should, orange where it doesn't, and beside it the
// link to the details it sums up; a click on either opens them
inline void quality_badge(char const *Id, char const *Text, bool const Good, char const *Details, bool &Open)
{
	ImGui::PushID(Id);
	if (state_chip(Text, Good ? kGood : kBelow))
		Open = !Open;
	auto const room{ImGui::CalcTextSize(Details).x + ImGui::GetFontSize() * 1.2f};
	same_line_if_fits(room);
	fold_link("##details", Details, Open);
	ImGui::PopID();
}

// one line of a message in its colour, the rest of it in the tooltip of the line; true when the message has more lines than the one
inline bool message_line(std::string_view const Text, ImVec4 const &Colour)
{
	auto const end{Text.find('\n')};
	std::string const first{Text.substr(0, end)};
	ImGui::PushStyleColor(ImGuiCol_Text, Colour);
	ImGui::TextWrapped("%s", first.c_str());
	ImGui::PopStyleColor();
	if (end == std::string_view::npos)
		return false;
	if (ImGui::IsItemHovered())
	{
		std::string const rest{Text.substr(end + 1)};
		ImGui::BeginTooltip();
		ImGui::PushTextWrapPos(std::round(ImGui::GetFontSize() * 32.0f));
		ImGui::TextUnformatted(rest.c_str());
		ImGui::PopTextWrapPos();
		ImGui::EndTooltip();
	}
	return true;
}

} // namespace editor_flow
