#pragma once

// https://wiki.eu07.pl/index.php?title=Obiekt_node::triangles

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/header.hpp>
#include <eu07/scene/node/parse_util.hpp>
#include <eu07/scene/node/types.hpp>

#include <string>
#include <vector>

namespace eu07::scene {

struct ParsedNodeTriangles {
    NodeHeader header;
    NodeMaterial material;
    std::string texture;
    std::vector<MeshVertex> vertices;
    std::vector<SourceToken> raw;
};

namespace node_triangles {

inline constexpr std::string_view kSubtype = "triangles";
inline constexpr std::string_view kEndMarker = "endtriangles";

[[nodiscard]] inline bool parseMaterial(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    NodeMaterial& material) {
    if (stream.empty() || !isKeyword(stream.peek().value, "material")) {
        return false;
    }
    SourceToken kw;
    if (!node::io::takeToken(stream, raw, kw)) {
        return false;
    }

    auto matchesMaterialLabel = [](const std::string_view token, const std::string_view label) noexcept {
        if (isKeyword(token, label)) {
            return true;
        }
        // MaSzyna tokenizuje "ambient:" jako jeden token (scenenode.cpp).
        return token.size() == label.size() + 1 && token.back() == ':' &&
               isKeyword(token.substr(0, label.size()), label);
    };

    auto takeComponent = [&](const std::string_view label, MaterialRgb& rgb) -> bool {
        if (stream.empty() || !matchesMaterialLabel(stream.peek().value, label)) {
            return false;
        }
        if (!node::io::takeToken(stream, raw, kw)) {
            return false;
        }
        return node::io::takeDouble(stream, raw, rgb.r) && node::io::takeDouble(stream, raw, rgb.g) &&
               node::io::takeDouble(stream, raw, rgb.b);
    };

    if (!takeComponent("ambient", material.ambient) || !takeComponent("diffuse", material.diffuse) ||
        !takeComponent("specular", material.specular)) {
        return false;
    }
    if (stream.empty() || !isKeyword(stream.peek().value, "endmaterial")) {
        return false;
    }
    return node::io::takeToken(stream, raw, kw);
}

[[nodiscard]] inline bool takeVertex(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    MeshVertex& vertex,
    const bool allowEndSuffix) {
    if (!node::io::takeDouble(stream, raw, vertex.x) || !node::io::takeDouble(stream, raw, vertex.y) ||
        !node::io::takeDouble(stream, raw, vertex.z) || !node::io::takeDouble(stream, raw, vertex.nx) ||
        !node::io::takeDouble(stream, raw, vertex.ny) || !node::io::takeDouble(stream, raw, vertex.nz) ||
        !node::io::takeDouble(stream, raw, vertex.u) || !node::io::takeDouble(stream, raw, vertex.v)) {
        return false;
    }
    if (allowEndSuffix && !stream.empty() && isKeyword(stream.peek().value, "end")) {
        SourceToken endToken;
        if (!node::io::takeToken(stream, raw, endToken)) {
            return false;
        }
    }
    return true;
}

[[nodiscard]] inline bool parseBody(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const NodeHeader& header,
    ParsedNodeTriangles& out) {
    out.header = header;
    out.raw.clear();

    if (!stream.empty() && isKeyword(stream.peek().value, "material")) {
        if (!parseMaterial(stream, raw, out.material) ||
            !node::io::takeString(stream, raw, out.texture)) {
            return false;
        }
    } else if (!node::io::takeString(stream, raw, out.texture)) {
        return false;
    }

    while (!stream.empty() && !node::io::atEnd(stream, kSubtype, kEndMarker)) {
        MeshVertex vertex;
        const bool hasMore = stream.remaining() > 1;
        if (!takeVertex(stream, raw, vertex, hasMore)) {
            return false;
        }
        out.vertices.push_back(vertex);
    }

    return node::io::consumeEnd(stream, raw, kSubtype, kEndMarker);
}

} // namespace node_triangles

} // namespace eu07::scene
