/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <initializer_list>
#include <string>
#include <vector>

#include "imgui/imgui.h"
#include "utilities/translation.h"

float constexpr kDragPixels{5.0f};

template <typename Node_> inline std::string name_of(Node_ const &Node)
{
	return Node.name().empty() ? std::string{STR_C("(no name)")} : Node.name();
}

inline float tall_button_height()
{
	return std::round(ImGui::GetFrameHeight() * 1.3f);
}

inline bool main_button(char const *Label, ImVec2 const &Size)
{
	ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.55f, 0.25f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.65f, 0.3f, 1.0f));
	auto const pressed{ImGui::Button(Label, Size)};
	ImGui::PopStyleColor(2);
	return pressed;
}

inline void item_tooltip(char const *Text)
{
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C(Text));
}

// grid points without a height (or another value) take the one of the nearest point which has it (by steps along the grid)
template <typename Value_> inline void fill_from_nearest(std::vector<Value_> &Heights, std::vector<char> &Found, std::size_t const Side)
{
    std::deque<std::size_t> queue;
    for (std::size_t index = 0; index < Found.size(); ++index)
        if (Found[index] != 0)
            queue.push_back(index);
    while (false == queue.empty())
    {
        auto const index = queue.front();
        queue.pop_front();
        auto const ix = index % Side, iz = index / Side;
        auto const visit = [&](std::size_t const Next) {
            if (Found[Next] != 0)
                return;
            Found[Next] = 1;
            Heights[Next] = Heights[index];
            queue.push_back(Next);
        };
        if (ix > 0)
            visit(index - 1);
        if (ix + 1 < Side)
            visit(index + 1);
        if (iz > 0)
            visit(index - Side);
        if (iz + 1 < Side)
            visit(index + Side);
    }
}

// the families of the track tools, each with its colour: building the track, equipping it, checking it
inline ImU32 family_colour(int const Family, float const Alpha = 1.0f)
{
	static ImVec4 const colours[]{{0.30f, 0.60f, 0.95f, 1.0f}, {0.95f, 0.66f, 0.22f, 1.0f}, {0.25f, 0.80f, 0.66f, 1.0f}};
	auto colour{colours[std::clamp(Family, 0, 2)]};
	colour.w = Alpha;
	return ImGui::ColorConvertFloat4ToU32(colour);
}

inline char const *family_name(int const Family)
{
	static char const *const names[]{"Building", "Equipment", "Checks"};
	return STR_C(names[std::clamp(Family, 0, 2)]);
}

// a numbered step of the work in a panel: the number in a circle, its name, a line to the edge of the panel
inline void step_heading(int const Number, char const *Text)
{
	ImGui::Spacing();
	auto *draw{ImGui::GetWindowDrawList()};
	auto const at{ImGui::GetCursorScreenPos()};
	auto const height{ImGui::GetTextLineHeight()};
	auto const radius{std::round(height * 0.6f)};
	ImVec2 const centre{at.x + radius, at.y + radius};
	draw->AddCircleFilled(centre, radius, ImGui::GetColorU32(ImGuiCol_CheckMark), 20);
	char digit[8];
	std::snprintf(digit, sizeof(digit), "%d", Number);
	auto const size{ImGui::CalcTextSize(digit)};
	draw->AddText(ImVec2(std::round(centre.x - size.x * 0.5f), std::round(centre.y - size.y * 0.5f)), IM_COL32(16, 18, 22, 255), digit);
	ImGui::Dummy(ImVec2(radius * 2.0f, radius * 2.0f));
	ImGui::SameLine(0.0f, radius * 0.6f);
	ImGui::SetCursorPosY(ImGui::GetCursorPosY() + radius - height * 0.5f);
	ImGui::TextUnformatted(Text);
	auto const end{ImGui::GetItemRectMax().x + radius * 0.6f};
	auto const edge{ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x};
	if (edge > end)
		draw->AddLine(ImVec2(end, centre.y), ImVec2(edge, centre.y), ImGui::GetColorU32(ImGuiCol_Separator));
	ImGui::Spacing();
}

// a toggle drawn as a pill: a swatch of the colour of what it shows and its name, lit while on
inline bool pill_toggle(char const *Label, ImU32 const Swatch, bool &Value)
{
	auto const text{ImGui::CalcTextSize(Label)};
	auto const height{ImGui::GetFrameHeight()};
	auto const swatch{std::round(height * 0.7f)};
	auto const pad{std::round(height * 0.35f)};
	ImVec2 const size{pad + swatch + pad * 0.6f + text.x + pad, height};
	auto const at{ImGui::GetCursorScreenPos()};
	ImGui::PushID(Label);
	auto const clicked{ImGui::InvisibleButton("##pill", size)};
	ImGui::PopID();
	if (clicked)
		Value = !Value;
	auto const hovered{ImGui::IsItemHovered()};
	auto *draw{ImGui::GetWindowDrawList()};
	ImVec2 const end{at.x + size.x, at.y + size.y};
	draw->AddRectFilled(at, end, ImGui::GetColorU32(Value ? (hovered ? ImGuiCol_ButtonHovered : ImGuiCol_Header) : (hovered ? ImGuiCol_FrameBgHovered : ImGuiCol_FrameBg)), height * 0.5f);
	auto const middle{at.y + height * 0.5f};
	auto const colour{Value ? Swatch : (Swatch & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 70)};
	draw->AddLine(ImVec2(at.x + pad, middle), ImVec2(at.x + pad + swatch, middle), colour, std::max(2.0f, height * 0.16f));
	draw->AddText(ImVec2(at.x + pad + swatch + pad * 0.6f, std::round(middle - text.y * 0.5f)), ImGui::GetColorU32(Value ? ImGuiCol_Text : ImGuiCol_TextDisabled), Label);
	return clicked;
}

// a choice of a few as one bar of segments across the panel, the chosen one lit; true when the choice changed
struct segment_option
{
	int value;
	char const *label;
	char const *tooltip{nullptr};
};
inline bool segmented(char const *Id, int &Value, std::initializer_list<segment_option> Options)
{
	auto const count{static_cast<float>(Options.size())};
	auto const width{ImGui::GetContentRegionAvail().x};
	auto const height{ImGui::GetFrameHeight()};
	auto const at{ImGui::GetCursorScreenPos()};
	auto *draw{ImGui::GetWindowDrawList()};
	auto const rounding{height * 0.5f};
	draw->AddRectFilled(at, ImVec2(at.x + width, at.y + height), ImGui::GetColorU32(ImGuiCol_FrameBg), rounding);
	ImGui::PushID(Id);
	bool changed{false};
	float index{0.0f};
	for (auto const &option : Options)
	{
		auto const left{std::round(at.x + width * index / count)};
		auto const right{std::round(at.x + width * (index + 1.0f) / count)};
		ImGui::SetCursorScreenPos(ImVec2(left, at.y));
		ImGui::PushID(option.value);
		if (ImGui::InvisibleButton("##segment", ImVec2(std::max(1.0f, right - left), height)) && Value != option.value)
		{
			Value = option.value;
			changed = true;
		}
		auto const hovered{ImGui::IsItemHovered()};
		if (option.tooltip != nullptr && hovered)
			ImGui::SetTooltip("%s", STR_C(option.tooltip));
		ImGui::PopID();
		auto const chosen{Value == option.value};
		if (chosen || hovered)
			draw->AddRectFilled(ImVec2(left + 2.0f, at.y + 2.0f), ImVec2(right - 2.0f, at.y + height - 2.0f), chosen ? family_colour(0, 0.8f) : ImGui::GetColorU32(ImGuiCol_FrameBgHovered), rounding - 2.0f);
		auto const text{ImGui::CalcTextSize(option.label)};
		draw->PushClipRect(ImVec2(left + 4.0f, at.y), ImVec2(right - 4.0f, at.y + height), true);
		draw->AddText(ImVec2(std::round(std::max(left + 6.0f, (left + right - text.x) * 0.5f)), std::round(at.y + (height - text.y) * 0.5f)), ImGui::GetColorU32(chosen ? ImGuiCol_Text : ImGuiCol_TextDisabled), option.label);
		draw->PopClipRect();
		index += 1.0f;
	}
	ImGui::PopID();
	ImGui::SetCursorScreenPos(at);
	ImGui::Dummy(ImVec2(width, height));
	return changed;
}

// values picked with a click, as round chips which go on to the next line at the edge of the panel; true when one was picked
inline bool value_chips(char const *Id, double &Value, std::initializer_list<double> Values, char const *Format)
{
	auto const edge{ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x};
	auto const gap{std::round(ImGui::GetStyle().ItemSpacing.x * 0.6f)};
	ImGui::PushID(Id);
	ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, ImGui::GetFrameHeight() * 0.5f);
	bool changed{false};
	bool first{true};
	for (auto const value : Values)
	{
		char text[32];
		std::snprintf(text, sizeof(text), Format, value);
		auto const width{ImGui::CalcTextSize(text).x + ImGui::GetStyle().FramePadding.x * 2.0f};
		if (false == first)
		{
			ImGui::SameLine(0.0f, gap);
			if (ImGui::GetCursorScreenPos().x + width > edge)
				ImGui::NewLine();
		}
		first = false;
		auto const chosen{std::abs(Value - value) < 1e-6};
		ImGui::PushStyleColor(ImGuiCol_Button, chosen ? family_colour(0, 0.8f) : ImGui::GetColorU32(ImGuiCol_FrameBg));
		ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetColorU32(chosen ? ImGuiCol_Text : ImGuiCol_TextDisabled));
		if (ImGui::Button(text))
		{
			Value = value;
			changed = true;
		}
		ImGui::PopStyleColor(2);
	}
	ImGui::PopStyleVar();
	ImGui::PopID();
	return changed;
}

// one of a few named choices as round chips which go on to the next line at the edge of the panel, where a tab bar
// would cut the names; true when one was picked
inline bool choice_chips(char const *Id, int &Value, std::vector<segment_option> const &Options)
{
	auto const edge{ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x};
	auto const gap{std::round(ImGui::GetStyle().ItemSpacing.x * 0.6f)};
	ImGui::PushID(Id);
	ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, ImGui::GetFrameHeight() * 0.5f);
	bool changed{false};
	bool first{true};
	for (auto const &option : Options)
	{
		auto const width{ImGui::CalcTextSize(option.label).x + ImGui::GetStyle().FramePadding.x * 2.0f};
		if (false == first)
		{
			ImGui::SameLine(0.0f, gap);
			if (ImGui::GetCursorScreenPos().x + width > edge)
				ImGui::NewLine();
		}
		first = false;
		auto const chosen{Value == option.value};
		ImGui::PushID(option.value);
		ImGui::PushStyleColor(ImGuiCol_Button, chosen ? family_colour(0, 0.8f) : ImGui::GetColorU32(ImGuiCol_FrameBg));
		if (ImGui::Button(option.label) && false == chosen)
		{
			Value = option.value;
			changed = true;
		}
		if (option.tooltip != nullptr)
			item_tooltip(option.tooltip);
		ImGui::PopStyleColor();
		ImGui::PopID();
	}
	ImGui::PopStyleVar();
	ImGui::PopID();
	return changed;
}

// values at a glance as tiles: the value in its colour over its name, a bar of the colour at the left, on to the next row
// at the edge of the panel
struct metric_tile
{
	std::string value;
	char const *label;
	ImU32 colour;
};
inline void metric_tiles(std::initializer_list<metric_tile> Tiles)
{
	auto const font{ImGui::GetFontSize()};
	auto const height{std::round(font * 2.5f)};
	auto const edge{ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x};
	auto const gap{std::round(font * 0.3f)};
	auto *list{ImGui::GetWindowDrawList()};
	bool first{true};
	for (auto const &entry : Tiles)
	{
		auto const value{ImGui::CalcTextSize(entry.value.c_str())};
		auto const label{ImGui::CalcTextSize(entry.label)};
		auto const width{std::max(std::round(font * 5.0f), std::max(value.x, label.x) + font)};
		if (false == first)
		{
			ImGui::SameLine(0.0f, gap);
			if (ImGui::GetCursorScreenPos().x + width > edge)
				ImGui::NewLine();
		}
		first = false;
		auto const at{ImGui::GetCursorScreenPos()};
		ImGui::Dummy(ImVec2(width, height));
		list->AddRectFilled(at, ImVec2(at.x + width, at.y + height), ImGui::GetColorU32(ImGuiCol_FrameBg), std::round(font * 0.3f));
		list->AddRectFilled(at, ImVec2(at.x + 3.0f, at.y + height), entry.colour, 2.0f, ImDrawFlags_RoundCornersLeft);
		list->AddText(ImVec2(at.x + font * 0.5f, at.y + height * 0.12f), entry.colour, entry.value.c_str());
		list->AddText(ImVec2(at.x + font * 0.5f, at.y + height * 0.12f + value.y), ImGui::GetColorU32(ImGuiCol_TextDisabled), entry.label);
	}
}

// a choice as a tile: a sketch drawn by Sketch into the box on the left (in the colours of the main line and of what it makes),
// the name beside it; lit while chosen, true when clicked
inline bool sketch_tile(char const *Id, char const *Label, bool const Chosen, float const Width, std::function<void(ImDrawList &, ImVec2, ImVec2, ImU32, ImU32)> const &Sketch)
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
	auto const side{std::round(font * 2.2f)};
	ImVec2 const low{top.x + pad, top.y + pad};
	ImVec2 const high{top.x + pad + side, bottom.y - pad};
	Sketch(*list, low, high, Chosen ? accent : ImGui::GetColorU32(ImGuiCol_Text, 0.7f), Chosen ? IM_COL32(255, 210, 60, 255) : ImGui::GetColorU32(ImGuiCol_Text, 0.45f));
	auto const text_left{high.x + pad};
	auto const wrap{bottom.x - pad - text_left};
	auto const size{ImGui::CalcTextSize(Label, nullptr, false, wrap)};
	list->AddText(nullptr, 0.0f, ImVec2(text_left, std::round(top.y + (height - size.y) * 0.5f)), ImGui::GetColorU32(Chosen ? ImGuiCol_Text : ImGuiCol_TextDisabled), Label, nullptr, wrap);
	return clicked;
}

// the state of something as a rounded chip in its colour; true when clicked
inline bool state_chip(char const *Label, ImVec4 const &Colour)
{
	ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, ImGui::GetFrameHeight() * 0.5f);
	ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(Colour.x * 0.3f, Colour.y * 0.3f, Colour.z * 0.3f, 0.9f));
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(Colour.x * 0.45f, Colour.y * 0.45f, Colour.z * 0.45f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(Colour.x * 0.55f, Colour.y * 0.55f, Colour.z * 0.55f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_Text, Colour);
	auto const clicked{ImGui::Button(Label)};
	ImGui::PopStyleColor(4);
	ImGui::PopStyleVar();
	return clicked;
}

// fields as rows of a table: the name wrapped in a column on the left, the field as wide as the rest of the panel;
// field_row takes the untranslated "Name##id" (a literal: the translations are cached by the pointer, a name made at run
// time goes with Translate false) and leaves the cursor where the field goes
inline bool begin_fields(char const *Id, float const Names = 0.42f)
{
	if (false == ImGui::BeginTable(Id, 2, ImGuiTableFlags_SizingStretchProp))
		return false;
	ImGui::TableSetupColumn("##name", ImGuiTableColumnFlags_WidthStretch, Names);
	ImGui::TableSetupColumn("##field", ImGuiTableColumnFlags_WidthStretch, 1.0f - Names);
	return true;
}
inline void field_row(char const *Label, bool const Translate = true)
{
	ImGui::TableNextRow();
	ImGui::TableNextColumn();
	ImGui::AlignTextToFramePadding();
	auto const *text{Translate ? STR_C(Label) : Label};
	ImGui::PushTextWrapPos(0.0f);
	ImGui::TextUnformatted(text, std::strstr(text, "##"));
	ImGui::PopTextWrapPos();
	ImGui::TableNextColumn();
	ImGui::SetNextItemWidth(-FLT_MIN);
}

// switches of a few things as round chips which go on to the next line at the edge of the panel, lit while on;
// the index of the one clicked, or -1
struct toggle_option
{
	char const *label;
	bool *value;
	char const *tooltip{nullptr};
};
inline int toggle_chips(char const *Id, std::initializer_list<toggle_option> Options)
{
	auto const edge{ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x};
	auto const gap{std::round(ImGui::GetStyle().ItemSpacing.x * 0.6f)};
	auto const mark{std::round(ImGui::GetFontSize() * 0.45f)};
	ImGui::PushID(Id);
	ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, ImGui::GetFrameHeight() * 0.5f);
	int clicked{-1};
	int index{0};
	for (auto const &option : Options)
	{
		auto const on{*option.value};
		auto const width{ImGui::CalcTextSize(option.label).x + ImGui::GetStyle().FramePadding.x * 2.0f + mark * 1.6f};
		if (index > 0)
		{
			ImGui::SameLine(0.0f, gap);
			if (ImGui::GetCursorScreenPos().x + width > edge)
				ImGui::NewLine();
		}
		auto const at{ImGui::GetCursorScreenPos()};
		ImGui::PushID(index);
		ImGui::PushStyleColor(ImGuiCol_Button, on ? family_colour(0, 0.8f) : ImGui::GetColorU32(ImGuiCol_FrameBg));
		ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetColorU32(on ? ImGuiCol_Text : ImGuiCol_TextDisabled));
		ImGui::PushStyleVar(ImGuiStyleVar_ButtonTextAlign, ImVec2(1.0f, 0.5f));
		if (ImGui::Button(option.label, ImVec2(width, 0.0f)))
		{
			*option.value = !on;
			clicked = index;
		}
		ImGui::PopStyleVar();
		ImGui::PopStyleColor(2);
		if (option.tooltip != nullptr && ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C(option.tooltip));
		ImGui::PopID();
		auto const middle{ImVec2(at.x + ImGui::GetStyle().FramePadding.x + mark * 0.5f, at.y + ImGui::GetFrameHeight() * 0.5f)};
		auto *draw{ImGui::GetWindowDrawList()};
		if (*option.value)
			draw->AddCircleFilled(middle, mark * 0.5f, IM_COL32(255, 210, 60, 255));
		else
			draw->AddCircle(middle, mark * 0.5f, ImGui::GetColorU32(ImGuiCol_TextDisabled), 0, 1.5f);
		++index;
	}
	ImGui::PopStyleVar();
	ImGui::PopID();
	return clicked;
}
