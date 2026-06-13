#pragma once

// https://wiki.eu07.pl/index.php?title=Obiekt_node

#include <eu07/scene/node/header.hpp>
#include <eu07/scene/node/mesh.hpp>
#include <eu07/scene/node/types.hpp>

#include <string>
#include <vector>

namespace eu07::scene {

struct ParsedNodeTriangleFan {
    NodeHeader header;
    std::string texture;
    std::vector<MeshVertex> vertices;
    std::vector<SourceToken> raw;
};

namespace node_triangle_fan {

inline constexpr std::string_view kSubtype = "triangle_fan";
inline constexpr std::string_view kEndMarker = "endtriangle_fan";

[[nodiscard]] inline bool parseBody(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const NodeHeader& header,
    ParsedNodeTriangleFan& out) {
    return node::io::parseTexturedMeshBody(stream, raw, header, kSubtype, kEndMarker, out);
}

} // namespace node_triangle_fan

} // namespace eu07::scene
