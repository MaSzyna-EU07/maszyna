#pragma once

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/detail/subtype.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/header.hpp>
#include <eu07/scene/node/parse_util.hpp>

#include <string_view>
#include <vector>

namespace eu07::scene {

struct SceneDocument;

} // namespace eu07::scene

namespace eu07::scene::node::io {

struct NodeKindDesc {
    std::string_view subtype;
    std::string_view endMarker;
    bool (*parseAndStore)(
        TokenStream& stream,
        SceneDocument& document,
        const NodeHeader& header,
        std::vector<SourceToken>* embedRaw);
};

[[nodiscard]] inline bool consumeHeader(
    TokenStream& stream,
    NodeHeader& header,
    std::string& subtypeOut,
    std::vector<SourceToken>& raw) {
    if (stream.empty() || !isKeyword(stream.peek().value, "node")) {
        return false;
    }

    SourceToken head;
    if (!takeToken(stream, raw, head)) {
        return false;
    }
    header.line = head.sourceLine;

    if (!takeDouble(stream, raw, header.rangeMax) || !takeDouble(stream, raw, header.rangeMin) ||
        !takeString(stream, raw, header.name)) {
        return false;
    }

    if (stream.empty() || !eu07::scene::detail::isNodeSubtype(stream.peek().value)) {
        return false;
    }

    if (!takeString(stream, raw, subtypeOut)) {
        return false;
    }
    return true;
}

} // namespace eu07::scene::node::io
