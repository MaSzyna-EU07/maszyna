#pragma once

#include <eu07/scene/match.hpp>

#include <array>
#include <string_view>

namespace eu07::scene {

inline constexpr std::array<std::string_view, 30> kScenarioDirectiveKeywords{{
    "atmo",
    "sky",
    "time",
    "FirstInit",
    "trainset",
    "endtrainset",
    "assignment",
    "endassignment",
    "event",
    "include",
    "camera",
    "config",
    "lua",
    "node",
    "origin",
    "orgin",
    "endorigin",
    "scale",
    "endscale",
    "rotate",
    "group",
    "endgroup",
    "isolated",
    "area",
    "terrain",
    "description",
    "light",
    "test",
    "endtest",
}};

[[nodiscard]] inline bool isScenarioDirectiveKeyword(const std::string_view token) noexcept {
    for (const std::string_view keyword : kScenarioDirectiveKeywords) {
        if (keyword == "FirstInit") {
            if (isKeywordIgnoreCase(token, keyword)) {
                return true;
            }
            continue;
        }
        if (isKeyword(token, keyword)) {
            return true;
        }
    }
    return false;
}

} // namespace eu07::scene
