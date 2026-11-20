/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string_view>

#include "imgui/imgui.h"
#include "imgui/imgui_internal.h"

namespace ui
{

// how long the cursor rests on an item of a menu or a tool before its description shows
inline constexpr float rest_description_seconds{5.f};

// whether the cursor has rested on the item just drawn for the given time without leaving it. one item is under the cursor at a
// time, so one record of it is enough: it starts anew with another item, or when the cursor came back after a frame away
inline bool rested_on_item(float const Seconds)
{
	struct rest
	{
		ImGuiID item{0};
		int frame{-1};
		double since{0.0};
	};
	static rest last; // NOSONAR: the state of the cursor between frames, touched by the ui thread only

	if (false == ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
		return false;
	auto const item{ImGui::GetItemID()};
	auto const frame{ImGui::GetFrameCount()};
	auto const now{ImGui::GetTime()};
	if (item != last.item || frame > last.frame + 1)
	{
		last.item = item;
		last.since = now;
	}
	last.frame = frame;
	return now - last.since >= Seconds;
}

// the longer description of how a menu entry or a tool works, shown once the cursor has rested on the item just drawn for the
// given time. called right after the item, as ImGui::IsItemHovered; it takes the place of a short tooltip the item shows at once
inline void rest_description(std::string_view const Text, float const Seconds = rest_description_seconds)
{
	if (Text.empty() || false == rested_on_item(Seconds))
		return;
	// over the short tooltip of the item, not added under it
	if (ImGui::BeginTooltipEx(ImGuiTooltipFlags_OverridePrevious, ImGuiWindowFlags_None))
	{
		ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.f);
		ImGui::TextUnformatted(Text.data(), Text.data() + Text.size());
		ImGui::PopTextWrapPos();
		ImGui::EndTooltip();
	}
}

} // namespace ui
