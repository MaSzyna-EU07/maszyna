#pragma once

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/node/header.hpp>
#include <eu07/scene/node/parse_util.hpp>
#include <eu07/scene/node/types.hpp>

#include <vector>

namespace eu07::scene {

struct ParsedNodeLineLoop {
    NodeHeader header;
    MaterialRgb color;
    double thickness = 0.0;
    std::vector<Vec3> points;
    std::vector<SourceToken> raw;
};

namespace node_line_loop {

inline constexpr std::string_view kSubtype = "line_loop";
inline constexpr std::string_view kEndMarker = "endline_loop";

[[nodiscard]] inline bool parseBody(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const NodeHeader& header,
    ParsedNodeLineLoop& out) {
    out.header = header;
    out.raw.clear();

    if (!node::io::takeDouble(stream, raw, out.color.r) ||
        !node::io::takeDouble(stream, raw, out.color.g) ||
        !node::io::takeDouble(stream, raw, out.color.b) ||
        !node::io::takeDouble(stream, raw, out.thickness)) {
        return false;
    }

    while (!stream.empty() && !node::io::atEnd(stream, kSubtype, kEndMarker)) {
        Vec3 point;
        if (!node::io::takeVec3(stream, raw, point)) {
            return false;
        }
        out.points.push_back(point);
    }

    return node::io::consumeEnd(stream, raw, kSubtype, kEndMarker);
}

} // namespace node_line_loop

} // namespace eu07::scene
