/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the mouse and the keys drawn as pictures: the lessons of the tutorial, the gestures of the tools and of the vertical profile

#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "imgui/imgui.h"
#include "utilities/translation.h"

ImU32 const kGestureLit{IM_COL32(90, 175, 255, 255)};

// the mouse with the part used lit: LMB, RMB, MMB, wheel (with arrows), move (with arrows); the width it took
inline float gesture_mouse(ImDrawList &List, ImVec2 const At, float const Height, std::string const &Part)
{
	auto const width{std::round(Height * 0.68f)};
	auto const line{ImGui::GetColorU32(ImGuiCol_Text)};
	auto const thickness{std::max(1.0f, Height * 0.07f)};
	auto const rounding{width * 0.5f};
	auto const split{At.y + std::round(Height * 0.45f)};
	auto const middle{At.x + width * 0.5f};
	ImVec2 const high{At.x + width, At.y + Height};
	if (Part == "LMB")
		List.AddRectFilled(At, ImVec2(middle, split), kGestureLit, rounding, ImDrawFlags_RoundCornersTopLeft);
	if (Part == "RMB")
		List.AddRectFilled(ImVec2(middle, At.y), ImVec2(high.x, split), kGestureLit, rounding, ImDrawFlags_RoundCornersTopRight);
	List.AddRect(At, high, line, rounding, 0, thickness);
	List.AddLine(ImVec2(At.x, split), ImVec2(high.x, split), line, thickness);
	List.AddLine(ImVec2(middle, At.y), ImVec2(middle, split), line, thickness);
	auto const wheel{std::max(3.0f, std::round(width * 0.24f))};
	ImVec2 const wheel_low{middle - wheel * 0.5f, At.y + Height * 0.1f};
	ImVec2 const wheel_high{middle + wheel * 0.5f, At.y + Height * 0.36f};
	List.AddRectFilled(wheel_low, wheel_high, (Part == "MMB" || Part == "wheel") ? kGestureLit : ImGui::GetColorU32(ImGuiCol_WindowBg), wheel * 0.5f);
	List.AddRect(wheel_low, wheel_high, line, wheel * 0.5f, 0, thickness * 0.8f);
	auto right{high.x};
	auto const arrow{Height * 0.16f};
	if (Part == "wheel")
	{
		auto const x{right + arrow * 1.6f};
		List.AddTriangleFilled(ImVec2(x - arrow, At.y + arrow * 1.8f), ImVec2(x + arrow, At.y + arrow * 1.8f), ImVec2(x, At.y + arrow * 0.3f), kGestureLit);
		List.AddTriangleFilled(ImVec2(x - arrow, high.y - arrow * 1.8f), ImVec2(x + arrow, high.y - arrow * 1.8f), ImVec2(x, high.y - arrow * 0.3f), kGestureLit);
		right = x + arrow;
	}
	if (Part == "move")
	{
		auto const reach{Height * 0.42f};
		ImVec2 const centre{right + arrow * 0.8f + reach, At.y + Height * 0.5f};
		List.AddLine(ImVec2(centre.x - reach + arrow, centre.y), ImVec2(centre.x + reach - arrow, centre.y), kGestureLit, thickness);
		List.AddLine(ImVec2(centre.x, centre.y - reach + arrow), ImVec2(centre.x, centre.y + reach - arrow), kGestureLit, thickness);
		List.AddTriangleFilled(ImVec2(centre.x - reach, centre.y), ImVec2(centre.x - reach + arrow * 1.4f, centre.y - arrow), ImVec2(centre.x - reach + arrow * 1.4f, centre.y + arrow), kGestureLit);
		List.AddTriangleFilled(ImVec2(centre.x + reach, centre.y), ImVec2(centre.x + reach - arrow * 1.4f, centre.y + arrow), ImVec2(centre.x + reach - arrow * 1.4f, centre.y - arrow), kGestureLit);
		List.AddTriangleFilled(ImVec2(centre.x, centre.y - reach), ImVec2(centre.x + arrow, centre.y - reach + arrow * 1.4f), ImVec2(centre.x - arrow, centre.y - reach + arrow * 1.4f), kGestureLit);
		List.AddTriangleFilled(ImVec2(centre.x, centre.y + reach), ImVec2(centre.x - arrow, centre.y + reach - arrow * 1.4f), ImVec2(centre.x + arrow, centre.y + reach - arrow * 1.4f), kGestureLit);
		right = centre.x + reach;
	}
	return right - At.x;
}

// a key cap with its name; the width it took
inline float gesture_key(ImDrawList &List, ImVec2 const At, float const Height, std::string const &Text)
{
	auto const size{ImGui::CalcTextSize(Text.c_str())};
	auto const width{std::max(std::round(size.x + Height * 0.6f), Height)};
	auto const rounding{Height * 0.2f};
	ImVec2 const high{At.x + width, At.y + Height};
	List.AddRectFilled(At, high, ImGui::GetColorU32(ImGuiCol_FrameBg), rounding);
	List.AddRect(At, high, ImGui::GetColorU32(ImGuiCol_TextDisabled), rounding, 0, std::max(1.0f, Height * 0.06f));
	List.AddText(ImVec2(std::round(At.x + (width - size.x) * 0.5f), std::round(At.y + (Height - size.y) * 0.5f)), ImGui::GetColorU32(ImGuiCol_Text), Text.c_str());
	return width;
}

// the words of a gesture as they're drawn: the parts of the mouse, the keys, "+" between them, other words as text
struct gesture_word
{
	enum class kind
	{
		mouse,
		key,
		plus,
		text
	} what;
	std::string text;
};

// "Ctrl+wheel", "Shift+click x4", "Drag a point", "Q W E": the keys and the parts of the mouse it names, the rest as text
inline std::vector<gesture_word> gesture_words(std::string const &Keys)
{
	static char const *const keys[]{"Ctrl", "Shift", "Alt", "Esc", "Del", "Enter", "Backspace", "Tab", "Space", "Home", "End", "PrtScr"};
	auto const mouse = [](std::string const &Word) -> std::string {
		if (Word == "LMB" || Word == "click" || Word == "Click" || Word == "Press")
			return "LMB";
		if (Word == "RMB" || Word == "MMB" || Word == "wheel")
			return Word;
		if (Word == "Wheel")
			return "wheel";
		if (Word == "Drag" || Word == "drag" || Word == "move")
			return "move";
		return {};
	};
	auto const key = [&](std::string const &Word) {
		if (Word.size() == 1 && (std::isupper(static_cast<unsigned char>(Word[0])) || std::isdigit(static_cast<unsigned char>(Word[0]))))
			return true;
		if (Word.size() >= 2 && Word[0] == 'F' && std::isdigit(static_cast<unsigned char>(Word[1])))
			return true;
		return std::find_if(std::begin(keys), std::end(keys), [&](char const *Key) { return Word == Key; }) != std::end(keys);
	};
	std::vector<gesture_word> words;
	std::string text;
	auto const flush = [&]() {
		if (false == text.empty())
			words.push_back({gesture_word::kind::text, text});
		text.clear();
	};
	std::istringstream stream{Keys};
	for (std::string word; stream >> word;)
	{
		std::vector<std::string> parts;
		std::size_t from{0};
		for (std::size_t plus; (plus = word.find('+', from)) != std::string::npos && plus > from; from = plus + 1)
			parts.push_back(word.substr(from, plus - from));
		parts.push_back(word.substr(from));
		bool const all{std::all_of(parts.begin(), parts.end(), [&](std::string const &Part) { return false == mouse(Part).empty() || key(Part); })};
		if (false == all || (parts.size() == 1 && words.empty() == false && words.back().what == gesture_word::kind::text && false == key(parts[0]) && mouse(parts[0]).empty()))
		{
			text += (text.empty() ? "" : " ") + word;
			continue;
		}
		flush();
		for (std::size_t index = 0; index < parts.size(); ++index)
		{
			if (index > 0)
				words.push_back({gesture_word::kind::plus, "+"});
			if (auto const part{mouse(parts[index])}; false == part.empty())
				words.push_back({gesture_word::kind::mouse, part});
			else
				words.push_back({gesture_word::kind::key, parts[index]});
		}
	}
	flush();
	return words;
}

// the mouse and the keys of a gesture only, its words left to the text beside it
inline std::vector<gesture_word> gesture_picture(std::string const &Keys)
{
	std::vector<gesture_word> words;
	for (auto const &word : gesture_words(Keys))
		if (word.what != gesture_word::kind::text && (word.what != gesture_word::kind::plus || (false == words.empty() && words.back().what != gesture_word::kind::plus)))
			words.push_back(word);
	while (false == words.empty() && words.back().what == gesture_word::kind::plus)
		words.pop_back();
	return words;
}

// the picture says all the words of the gesture: keys alone, no mouse nor words
inline bool gesture_keys_only(std::string const &Keys)
{
	auto const words{gesture_words(Keys)};
	return false == words.empty() && std::all_of(words.begin(), words.end(), [](gesture_word const &Word) { return Word.what == gesture_word::kind::key || Word.what == gesture_word::kind::plus; });
}

// the gesture drawn in a row; the width it took
inline float gesture_draw(ImDrawList &List, ImVec2 const At, float const Height, std::vector<gesture_word> const &Words, ImU32 const Text)
{
	auto const gap{std::round(Height * 0.2f)};
	float x{0.0f};
	for (auto const &word : Words)
	{
		ImVec2 const place{At.x + x, At.y};
		switch (word.what)
		{
		case gesture_word::kind::mouse: x += gesture_mouse(List, place, Height, word.text) + gap; break;
		case gesture_word::kind::key: x += gesture_key(List, place, Height, word.text) + gap; break;
		case gesture_word::kind::plus:
		case gesture_word::kind::text:
		{
			auto const size{ImGui::CalcTextSize(word.text.c_str())};
			List.AddText(ImVec2(place.x, std::round(At.y + (Height - size.y) * 0.5f)), word.what == gesture_word::kind::plus ? ImGui::GetColorU32(ImGuiCol_TextDisabled) : Text, word.text.c_str());
			x += size.x + gap;
			break;
		}
		}
	}
	return std::max(0.0f, x - gap);
}

inline float gesture_width(float const Height, std::vector<gesture_word> const &Words)
{
	auto const gap{std::round(Height * 0.2f)};
	auto const arrow{Height * 0.16f};
	float x{0.0f};
	for (auto const &word : Words)
	{
		switch (word.what)
		{
		case gesture_word::kind::mouse:
			x += std::round(Height * 0.68f) + (word.text == "wheel" ? arrow * 2.6f : word.text == "move" ? arrow * 0.8f + Height * 0.84f : 0.0f);
			break;
		case gesture_word::kind::key: x += std::max(std::round(ImGui::CalcTextSize(word.text.c_str()).x + Height * 0.6f), Height); break;
		default: x += ImGui::CalcTextSize(word.text.c_str()).x; break;
		}
		x += gap;
	}
	return std::max(0.0f, x - gap);
}

// rows of gestures, each drawn with its key and what it does beside it; the keys and the texts are the English ones, translated here
inline void gesture_rows(std::vector<std::pair<char const *, char const *>> const &Rows)
{
	auto *list{ImGui::GetWindowDrawList()};
	auto const font{ImGui::GetFontSize()};
	auto const size{std::round(font * 1.25f)};
	auto const pad{std::round(font * 0.6f)};
	std::vector<std::vector<gesture_word>> pictures;
	float column{0.0f};
	for (auto const &row : Rows)
	{
		pictures.push_back(gesture_picture(row.first));
		column = std::max(column, gesture_width(size, pictures.back()));
	}
	for (std::size_t i = 0; i < Rows.size(); ++i)
	{
		auto const at{ImGui::GetCursorScreenPos()};
		auto const line{ImGui::GetTextLineHeight()};
		auto const height{std::max(size, line)};
		gesture_draw(*list, ImVec2(at.x, at.y + std::round((height - size) * 0.5f)), size, pictures[i], ImGui::GetColorU32(ImGuiCol_Text));
		ImGui::Dummy(ImVec2(column, height));
		ImGui::SameLine(0.0f, pad);
		ImGui::SetCursorPosY(ImGui::GetCursorPosY() + std::round((height - line) * 0.5f));
		if (false == gesture_keys_only(Rows[i].first))
		{
			ImGui::TextColored(ImVec4(1.0f, 0.83f, 0.3f, 1.0f), "%s", STR_C(Rows[i].first));
			ImGui::SameLine();
		}
		ImGui::TextWrapped("%s", STR_C(Rows[i].second));
	}
}
