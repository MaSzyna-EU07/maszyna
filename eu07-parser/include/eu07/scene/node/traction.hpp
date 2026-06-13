#pragma once

// https://wiki.eu07.pl/index.php?title=Obiekt_node::traction

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/header.hpp>
#include <eu07/scene/node/parse_util.hpp>
#include <eu07/scene/node/shared.hpp>
#include <eu07/scene/node/types.hpp>

#include <optional>
#include <string>
#include <vector>

namespace eu07::scene {

struct ParsedNodeTraction {
    NodeHeader header;
    std::string powerSourceName;
    double nominalVoltage = 0.0;
    double maxCurrent = 0.0;
    double resistivity = 0.0;
    TractionMaterialKind materialKind = TractionMaterialKind::Copper;
    std::string materialRaw;
    double wireThickness = 0.0;
    int damageFlag = 0;
    Vec3 wireLowerStart;
    Vec3 wireLowerEnd;
    Vec3 wireUpperStart;
    Vec3 wireUpperEnd;
    double minHeight = 0.0;
    double segmentLength = 0.0;
    int wires = 0;
    double wireOffset = 0.0;
    NodeVisibilityMode visibilityMode = NodeVisibilityMode::Hidden;
    std::optional<std::string> parallel;
    std::vector<SourceToken> raw;
};

namespace node_traction {

inline constexpr std::string_view kSubtype = "traction";
inline constexpr std::string_view kEndMarker = "endtraction";

[[nodiscard]] inline bool parseBody(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const NodeHeader& header,
    ParsedNodeTraction& out) {
    out.header = header;
    out.raw.clear();

    double damageFlag = 0.0;
    double wires = 0.0;
    if (!node::io::takeString(stream, raw, out.powerSourceName) ||
        !node::io::takeDouble(stream, raw, out.nominalVoltage) ||
        !node::io::takeDouble(stream, raw, out.maxCurrent) ||
        !node::io::takeDouble(stream, raw, out.resistivity) ||
        !node::io::takeString(stream, raw, out.materialRaw) ||
        !node::io::takeDouble(stream, raw, out.wireThickness) ||
        !node::io::takeDouble(stream, raw, damageFlag) ||
        !node::io::takeVec3(stream, raw, out.wireLowerStart) ||
        !node::io::takeVec3(stream, raw, out.wireLowerEnd) ||
        !node::io::takeVec3(stream, raw, out.wireUpperStart) ||
        !node::io::takeVec3(stream, raw, out.wireUpperEnd) ||
        !node::io::takeDouble(stream, raw, out.minHeight) ||
        !node::io::takeDouble(stream, raw, out.segmentLength) ||
        !node::io::takeDouble(stream, raw, wires) ||
        !node::io::takeDouble(stream, raw, out.wireOffset)) {
        return false;
    }
    out.damageFlag = static_cast<int>(damageFlag);
    out.wires = static_cast<int>(wires);
    out.materialKind = parseTractionMaterial(out.materialRaw);

    std::string vis;
    if (!node::io::takeString(stream, raw, vis)) {
        return false;
    }
    out.visibilityMode = parseNodeVisibility(vis);

    if (!stream.empty() && isKeyword(stream.peek().value, "parallel")) {
        SourceToken kw;
        if (!node::io::takeToken(stream, raw, kw)) {
            return false;
        }
        std::string name;
        if (!node::io::takeString(stream, raw, name)) {
            return false;
        }
        out.parallel = std::move(name);
    }

    return node::io::consumeEnd(stream, raw, kSubtype, kEndMarker);
}

} // namespace node_traction

} // namespace eu07::scene
