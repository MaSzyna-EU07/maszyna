#pragma once

// https://wiki.eu07.pl/index.php?title=Dyrektywa_rotate
// rotate x y z — nadpisuje rotację, bez endrotate i bez połykania kolejnych tokenów.

#include <eu07/scene/context.hpp>
#include <eu07/scene/cursor.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/parse_util.hpp>
#include <eu07/scene/node/types.hpp>

namespace eu07::scene::rotate {

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& ctx) {
    if (stream.empty() || !isKeyword(stream.peek().value, "rotate")) {
        return false;
    }

    DirectiveBlock block;
    block.line = stream.consume().sourceLine;

    Vec3 rotation;
    std::vector<SourceToken> raw;
    if (!node::io::takeDoubleOrPlaceholder(stream, raw, rotation.x) ||
        !node::io::takeDoubleOrPlaceholder(stream, raw, rotation.y) ||
        !node::io::takeDoubleOrPlaceholder(stream, raw, rotation.z)) {
        return false;
    }

    block.tokens = std::move(raw);
    ctx.scratch.setRotation(rotation);
    ctx.document.rotate.push_back(std::move(block));
    return true;
}

} // namespace eu07::scene::rotate
