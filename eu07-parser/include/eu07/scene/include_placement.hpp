#pragma once

// Mapowanie origin/rotate z .inc na indeksy parametrow INCL — bez heurystyki w runtime.

#include <eu07/scene/document.hpp>
#include <eu07/scene/include_resolve.hpp>

#include <cstdint>
#include <optional>
#include <string_view>

namespace eu07::scene {

struct IncludePlacement {
    std::uint8_t origin_x_param = 0;
    std::uint8_t origin_y_param = 0;
    std::uint8_t origin_z_param = 0;
    std::uint8_t rotation_y_param = 0;

    [[nodiscard]] bool empty() const noexcept {
        return origin_x_param == 0 && origin_y_param == 0 && origin_z_param == 0 &&
               rotation_y_param == 0;
    }
};

namespace detail {

[[nodiscard]] inline std::uint8_t placementParamFromToken(const std::string_view text) {
    std::uint8_t index = 0;
    if (parseIncludeParameterIndex(text, index)) {
        return index;
    }
    return 0;
}

} // namespace detail

[[nodiscard]] inline std::optional<IncludePlacement> extractIncludePlacement(const SceneDocument& document) {
    if (document.origin.empty()) {
        return std::nullopt;
    }

    const DirectiveBlock& originBlock = document.origin.front();
    if (originBlock.tokens.size() < 3) {
        return std::nullopt;
    }

    IncludePlacement placement;
    placement.origin_x_param = detail::placementParamFromToken(originBlock.tokens[0].value);
    placement.origin_y_param = detail::placementParamFromToken(originBlock.tokens[1].value);
    placement.origin_z_param = detail::placementParamFromToken(originBlock.tokens[2].value);

    if (!document.rotate.empty()) {
        const DirectiveBlock& rotateBlock = document.rotate.front();
        if (rotateBlock.tokens.size() >= 2) {
            placement.rotation_y_param = detail::placementParamFromToken(rotateBlock.tokens[1].value);
        }
    }

    if (placement.empty()) {
        return std::nullopt;
    }
    return placement;
}

} // namespace eu07::scene
