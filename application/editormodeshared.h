/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <deque>
#include <vector>

#include "imgui/imgui.h"
#include "utilities/translation.h"

float constexpr kDragPixels{5.0f};

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
