#pragma once

// https://wiki.eu07.pl/index.php?title=Obiekt_node

#include <eu07/scene/node/header.hpp>
#include <eu07/scene/node/mesh.hpp>
#include <eu07/scene/node/types.hpp>

#include <string>
#include <vector>

namespace eu07::scene {

struct ParsedNodeTriangleStrip {
    NodeHeader header;
    std::string texture;
    std::vector<MeshVertex> vertices;
    std::vector<SourceToken> raw;
};

namespace node_triangle_strip {

inline constexpr std::string_view kSubtype = "triangle_strip";
inline constexpr std::string_view kEndMarker = "endtriangle_strip";

[[nodiscard]] inline bool parseBody(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const NodeHeader& header,
    ParsedNodeTriangleStrip& out) {
    return node::io::parseTexturedMeshBody(stream, raw, header, kSubtype, kEndMarker, out);
}

} // namespace node_triangle_strip

} // namespace eu07::scene
