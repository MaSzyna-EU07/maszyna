#pragma once

#include <eu07/scene/match.hpp>

#include <array>
#include <string_view>

namespace eu07::scene::detail {

[[nodiscard]] inline bool isNodeSubtype(const std::string_view token) noexcept {
    static constexpr std::array<std::string_view, 14> kSubtypes{{
        "dynamic",
        "model",
        "track",
        "traction",
        "tractionpowersource",
        "triangles",
        "triangle_strip",
        "triangle_fan",
        "lines",
        "line_strip",
        "line_loop",
        "memcell",
        "eventlauncher",
        "sound",
    }};
    for (const std::string_view subtype : kSubtypes) {
        if (isKeyword(token, subtype)) {
            return true;
        }
    }
    return false;
}

} // namespace eu07::scene::detail
