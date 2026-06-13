#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>

#include <eu07/scene/match.hpp>
#include <eu07/scene/node/types.hpp>

namespace eu07::scene {

enum class NodeVisibilityMode {
    Visible,
    Hidden,
};

enum class TractionMaterialKind {
    Copper,
    Aluminum,
    None,
    Raw,
};

struct TrainsetContext {
    std::string name;
    std::string track;
    double consistOffset = 0.0;
    double velocity = 0.0;
};

struct CouplingSpec {
    std::string raw = "3";
    int type = 3;
    std::string params;
};

[[nodiscard]] inline CouplingSpec parseCouplingSpec(const std::string& raw) {
    CouplingSpec out;
    out.raw = raw;
    const std::size_t dot = raw.find('.');
    const std::string typeToken = dot == std::string::npos ? raw : raw.substr(0, dot);
    try {
        out.type = std::stoi(typeToken);
    } catch (...) {
        out.type = 3;
    }
    if (dot != std::string::npos) {
        out.params = raw.substr(dot + 1);
    }
    return out;
}

[[nodiscard]] inline TractionMaterialKind parseTractionMaterial(const std::string_view token) {
    if (isKeyword(token, "none")) {
        return TractionMaterialKind::None;
    }
    if (isKeyword(token, "al")) {
        return TractionMaterialKind::Aluminum;
    }
    if (isKeyword(token, "cu") || isKeyword(token, "miedz") || isKeyword(token, "miedź")) {
        return TractionMaterialKind::Copper;
    }
    return TractionMaterialKind::Raw;
}

[[nodiscard]] inline NodeVisibilityMode parseNodeVisibility(const std::string_view token) noexcept {
    if (isKeyword(token, "vis")) {
        return NodeVisibilityMode::Visible;
    }
    return NodeVisibilityMode::Hidden;
}

} // namespace eu07::scene
