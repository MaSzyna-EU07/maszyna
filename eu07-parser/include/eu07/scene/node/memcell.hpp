#pragma once

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/header.hpp>
#include <eu07/scene/node/parse_util.hpp>
#include <eu07/scene/node/types.hpp>

#include <optional>
#include <string>
#include <vector>

namespace eu07::scene {

struct ParsedNodeMemcell {
    NodeHeader header;
    Vec3 position;
    std::string command;
    double value1 = 0.0;
    double value2 = 0.0;
    std::optional<std::string> trackName;
    std::vector<SourceToken> raw;
};

namespace node_memcell {

inline constexpr std::string_view kSubtype = "memcell";
inline constexpr std::string_view kEndMarker = "endmemcell";

[[nodiscard]] inline bool parseBody(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const NodeHeader& header,
    ParsedNodeMemcell& out) {
    out.header = header;
    out.raw.clear();

    std::string trackRaw;
    if (!node::io::takeVec3(stream, raw, out.position) ||
        !node::io::takeString(stream, raw, out.command) ||
        !node::io::takeDouble(stream, raw, out.value1) ||
        !node::io::takeDouble(stream, raw, out.value2) ||
        !node::io::takeString(stream, raw, trackRaw)) {
        return false;
    }

    if (!isKeyword(trackRaw, "none")) {
        out.trackName = std::move(trackRaw);
    }

    return node::io::consumeEnd(stream, raw, kSubtype, kEndMarker);
}

} // namespace node_memcell

} // namespace eu07::scene
