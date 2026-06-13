#pragma once

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/node/header.hpp>
#include <eu07/scene/node/parse_util.hpp>
#include <eu07/scene/node/types.hpp>

#include <string>
#include <vector>

namespace eu07::scene {

struct ParsedNodeSound {
    NodeHeader header;
    Vec3 position;
    std::string wavFile;
    std::vector<SourceToken> raw;
};

namespace node_sound {

inline constexpr std::string_view kSubtype = "sound";
inline constexpr std::string_view kEndMarker = "endsound";

[[nodiscard]] inline bool parseBody(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const NodeHeader& header,
    ParsedNodeSound& out) {
    out.header = header;
    out.raw.clear();

    return node::io::takeVec3(stream, raw, out.position) &&
           node::io::takeString(stream, raw, out.wavFile) &&
           node::io::consumeEnd(stream, raw, kSubtype, kEndMarker);
}

} // namespace node_sound

} // namespace eu07::scene
