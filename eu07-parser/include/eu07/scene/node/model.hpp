#pragma once

// https://wiki.eu07.pl/index.php?title=Obiekt_node::model

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/header.hpp>
#include <eu07/scene/node/parse_util.hpp>
#include <eu07/scene/node/types.hpp>

#include <optional>
#include <string>
#include <vector>

namespace eu07::scene {

struct ParsedNodeModel {
    NodeHeader header;
    Vec3 location;
    double rotationY = 0.0;
    std::string modelPath;
    std::string replaceableSkin;
    std::optional<std::vector<double>> lightStates;
    std::optional<Vec3> angles;
    std::optional<std::vector<MaterialRgb>> lightColors;
    std::optional<Vec3> inlineScale;
    bool noTransition = false;
    std::vector<std::string> extra;
    std::vector<SourceToken> raw;
};

namespace node_model {

inline constexpr std::string_view kSubtype = "model";
inline constexpr std::string_view kEndMarker = "endmodel";

[[nodiscard]] inline bool takeLightColor(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    MaterialRgb& color) {
    SourceToken token;
    if (!node::io::takeToken(stream, raw, token)) {
        return false;
    }
    if (const std::optional<double> value = node::io::parseDouble(token.value)) {
        color.r = *value;
        SourceToken g;
        SourceToken b;
        if (!node::io::takeToken(stream, raw, g) || !node::io::takeToken(stream, raw, b)) {
            return false;
        }
        if (const std::optional<double> gv = node::io::parseDouble(g.value)) {
            color.g = *gv;
        }
        if (const std::optional<double> bv = node::io::parseDouble(b.value)) {
            color.b = *bv;
        }
        return true;
    }
    try {
        const int packed = std::stoi(token.value, nullptr, 16);
        color.r = static_cast<double>((packed >> 16) & 0xFF);
        color.g = static_cast<double>((packed >> 8) & 0xFF);
        color.b = static_cast<double>(packed & 0xFF);
        return true;
    } catch (...) {
        return false;
    }
}

[[nodiscard]] inline bool parseBody(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const NodeHeader& header,
    ParsedNodeModel& out) {
    out.header = header;
    out.raw.clear();

    if (!node::io::takeVec3(stream, raw, out.location) ||
        !node::io::takeDouble(stream, raw, out.rotationY) ||
        !node::io::takeString(stream, raw, out.modelPath) ||
        !node::io::takeString(stream, raw, out.replaceableSkin)) {
        return false;
    }

    while (!stream.empty() && !node::io::atEnd(stream, kSubtype, kEndMarker)) {
        const std::string_view value = stream.peek().value;

        if (isKeyword(value, "lights")) {
            SourceToken kw;
            if (!node::io::takeToken(stream, raw, kw)) {
                return false;
            }
            std::vector<double> states;
            while (!stream.empty() && !node::io::atEnd(stream, kSubtype, kEndMarker)) {
                const std::string_view next = stream.peek().value;
                if (isKeyword(next, "angles") || isKeyword(next, "lightcolors") ||
                    isKeyword(next, "notransition") || isKeyword(next, "scale")) {
                    break;
                }
                double state = 0.0;
                if (!node::io::takeDouble(stream, raw, state)) {
                    break;
                }
                states.push_back(state);
            }
            out.lightStates = std::move(states);
            continue;
        }
        if (isKeyword(value, "angles")) {
            SourceToken kw;
            if (!node::io::takeToken(stream, raw, kw)) {
                return false;
            }
            Vec3 angles;
            if (!node::io::takeVec3(stream, raw, angles)) {
                return false;
            }
            out.angles = angles;
            continue;
        }
        if (isKeyword(value, "lightcolors")) {
            SourceToken kw;
            if (!node::io::takeToken(stream, raw, kw)) {
                return false;
            }
            std::vector<MaterialRgb> colors;
            while (!stream.empty() && !node::io::atEnd(stream, kSubtype, kEndMarker)) {
                const std::string_view next = stream.peek().value;
                if (isKeyword(next, "notransition") || isKeyword(next, "scale")) {
                    break;
                }
                MaterialRgb color;
                if (!takeLightColor(stream, raw, color)) {
                    break;
                }
                colors.push_back(color);
            }
            out.lightColors = std::move(colors);
            continue;
        }
        if (isKeyword(value, "scale")) {
            SourceToken kw;
            if (!node::io::takeToken(stream, raw, kw)) {
                return false;
            }
            Vec3 scale;
            if (!node::io::takeVec3(stream, raw, scale)) {
                return false;
            }
            out.inlineScale = scale;
            continue;
        }
        if (isKeyword(value, "notransition")) {
            SourceToken kw;
            if (!node::io::takeToken(stream, raw, kw)) {
                return false;
            }
            out.noTransition = true;
            continue;
        }

        SourceToken token;
        if (!node::io::takeToken(stream, raw, token)) {
            break;
        }
        out.extra.push_back(token.value);
    }

    return node::io::consumeEnd(stream, raw, kSubtype, kEndMarker);
}

} // namespace node_model

} // namespace eu07::scene
