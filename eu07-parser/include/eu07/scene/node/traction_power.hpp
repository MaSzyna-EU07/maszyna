#pragma once

// https://wiki.eu07.pl — node tractionpowersource (world/TractionPower.cpp)

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/header.hpp>
#include <eu07/scene/node/parse_util.hpp>
#include <eu07/scene/node/types.hpp>

#include <string>
#include <vector>

namespace eu07::scene {

struct ParsedNodeTractionPowerSource {
    NodeHeader header;
    Vec3 position;
    double nominalVoltage = 0.0;
    double voltageFrequency = 0.0;
    double internalResistance = 0.2;
    double maxOutputCurrent = 0.0;
    double fastFuseTimeout = 0.0;
    double fastFuseRepetition = 0.0;
    double slowFuseTimeout = 0.0;
    bool recuperation = false;
    bool section = false;
    std::vector<SourceToken> raw;
};

namespace node_traction_power {

inline constexpr std::string_view kSubtype = "tractionpowersource";
inline constexpr std::string_view kEndMarker = "end";

[[nodiscard]] inline bool parseBody(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const NodeHeader& header,
    ParsedNodeTractionPowerSource& out) {
    out.header = header;
    out.raw.clear();
    out.recuperation = false;
    out.section = false;

    if (!node::io::takeVec3(stream, raw, out.position) ||
        !node::io::takeDouble(stream, raw, out.nominalVoltage) ||
        !node::io::takeDouble(stream, raw, out.voltageFrequency) ||
        !node::io::takeDouble(stream, raw, out.internalResistance) ||
        !node::io::takeDouble(stream, raw, out.maxOutputCurrent) ||
        !node::io::takeDouble(stream, raw, out.fastFuseTimeout) ||
        !node::io::takeDouble(stream, raw, out.fastFuseRepetition) ||
        !node::io::takeDouble(stream, raw, out.slowFuseTimeout)) {
        return false;
    }

    while (!stream.empty()) {
        std::string token;
        if (!node::io::takeString(stream, raw, token)) {
            return false;
        }
        if (isKeyword(token, kEndMarker)) {
            return true;
        }
        if (isKeywordIgnoreCase(token, "recuperation")) {
            out.recuperation = true;
            continue;
        }
        if (isKeywordIgnoreCase(token, "section")) {
            out.section = true;
            continue;
        }
    }

    return false;
}

} // namespace node_traction_power

} // namespace eu07::scene
