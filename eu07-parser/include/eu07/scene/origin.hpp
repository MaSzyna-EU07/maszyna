#pragma once

// https://wiki.eu07.pl/index.php?title=Dyrektywa_origin
// origin x y z … endorigin — push offsetu na stos, reszta idzie główną pętlą.

#include <eu07/scene/context.hpp>
#include <eu07/scene/cursor.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/parse_util.hpp>
#include <eu07/scene/node/types.hpp>

namespace eu07::scene::origin {

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& ctx) {
    if (stream.empty()) {
        return false;
    }
    const std::string_view kw = stream.peek().value;
    if (!isKeyword(kw, "origin") && !isKeyword(kw, "orgin")) {
        return false;
    }

    DirectiveBlock block;
    block.line = stream.consume().sourceLine;

    Vec3 offset;
    std::vector<SourceToken> raw;
    if (!node::io::takeDoubleOrPlaceholder(stream, raw, offset.x) ||
        !node::io::takeDoubleOrPlaceholder(stream, raw, offset.y) ||
        !node::io::takeDoubleOrPlaceholder(stream, raw, offset.z)) {
        return false;
    }

    block.tokens = std::move(raw);
    ctx.scratch.pushOrigin(offset);
    ctx.document.origin.push_back(std::move(block));
    return true;
}

} // namespace eu07::scene::origin
