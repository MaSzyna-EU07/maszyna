#pragma once

#include <eu07/scene/match.hpp>

#include <string_view>

namespace eu07::scene::detail {

// Token w srodku wpisu node (np. isolated NazwaToru, velocity, endtrack).
[[nodiscard]] inline bool isEmbeddedInNode(const std::string_view token) noexcept {
    return isKeyword(token, "velocity") || isKeyword(token, "rail_screw_used1") ||
           isKeyword(token, "endtrack") || isKeyword(token, "endtraction") ||
           isKeyword(token, "endtri") || isKeyword(token, "endtriangles") ||
           isKeyword(token, "enddynamic") || isKeyword(token, "endmodel") ||
           isKeyword(token, "vis") || isKeyword(token, "flat");
}

[[nodiscard]] inline bool isTopLevelStarter(const std::string_view token) noexcept {
    return isKeyword(token, "node") || isKeyword(token, "include") ||
           isKeyword(token, "trainset") || isKeyword(token, "event") ||
           isKeyword(token, "rotate") || isKeyword(token, "origin") ||
           isKeyword(token, "scale") || isKeyword(token, "group");
}

} // namespace eu07::scene::detail
