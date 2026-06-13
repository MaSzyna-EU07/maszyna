#pragma once

// Wspolne parsowanie siatki: triangle_strip, triangle_fan

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/header.hpp>
#include <eu07/scene/node/parse_util.hpp>
#include <eu07/scene/node/triangles.hpp>
#include <eu07/scene/node/types.hpp>

#include <string>
#include <string_view>
#include <vector>

namespace eu07::scene::node::io {

template <typename Parsed>
[[nodiscard]] inline bool parseTexturedMeshBody(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const NodeHeader& header,
    const std::string_view subtype,
    const std::string_view endMarker,
    Parsed& out) {
    out.header = header;
    out.raw.clear();

    if (!takeString(stream, raw, out.texture)) {
        return false;
    }

    while (!stream.empty() && !atEnd(stream, subtype, endMarker)) {
        MeshVertex vertex;
        const bool hasMore = stream.remaining() > 1;
        if (!node_triangles::takeVertex(stream, raw, vertex, hasMore)) {
            return false;
        }
        out.vertices.push_back(vertex);
    }

    return consumeEnd(stream, raw, subtype, endMarker);
}

} // namespace eu07::scene::node::io
