#pragma once

// https://wiki.eu07.pl/index.php?title=Obiekt_node (lines)

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/node/header.hpp>
#include <eu07/scene/node/parse_util.hpp>
#include <eu07/scene/node/types.hpp>

#include <utility>
#include <vector>

namespace eu07::scene {

struct LineSegment {
    Vec3 a;
    Vec3 b;
};

struct ParsedNodeLines {
    NodeHeader header;
    MaterialRgb color;
    double thickness = 0.0;
    std::vector<LineSegment> segments;
    std::vector<SourceToken> raw;
};

namespace node_lines {

inline constexpr std::string_view kSubtype = "lines";
inline constexpr std::string_view kEndMarker = "endlines";

[[nodiscard]] inline bool parseBody(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const NodeHeader& header,
    ParsedNodeLines& out) {
    out.header = header;
    out.raw.clear();

    if (!node::io::takeDouble(stream, raw, out.color.r) ||
        !node::io::takeDouble(stream, raw, out.color.g) ||
        !node::io::takeDouble(stream, raw, out.color.b) ||
        !node::io::takeDouble(stream, raw, out.thickness)) {
        return false;
    }

    while (!stream.empty() && !node::io::atEnd(stream, kSubtype, kEndMarker)) {
        LineSegment segment;
        if (!node::io::takeVec3(stream, raw, segment.a) || !node::io::takeVec3(stream, raw, segment.b)) {
            return false;
        }
        out.segments.push_back(segment);
    }

    return node::io::consumeEnd(stream, raw, kSubtype, kEndMarker);
}

} // namespace node_lines

} // namespace eu07::scene
