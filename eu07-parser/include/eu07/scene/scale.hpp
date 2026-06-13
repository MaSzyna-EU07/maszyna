#pragma once

// https://wiki.eu07.pl/index.php?title=Dyrektywa_scale
// scale x y z … endscale — push skali na stos.

#include <eu07/scene/context.hpp>
#include <eu07/scene/cursor.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/parse_util.hpp>
#include <eu07/scene/node/types.hpp>

namespace eu07::scene::scale {

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& ctx) {
    if (stream.empty() || !isKeyword(stream.peek().value, "scale")) {
        return false;
    }

    DirectiveBlock block;
    block.line = stream.consume().sourceLine;

    Vec3 factor;
    std::vector<SourceToken> raw;
    if (!node::io::takeDouble(stream, raw, factor.x) ||
        !node::io::takeDouble(stream, raw, factor.y) ||
        !node::io::takeDouble(stream, raw, factor.z)) {
        return false;
    }

    block.tokens = std::move(raw);
    ctx.scratch.pushScale(factor);
    ctx.document.scale.push_back(std::move(block));
    return true;
}

} // namespace eu07::scene::scale
