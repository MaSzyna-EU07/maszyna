#pragma once

// https://wiki.eu07.pl/index.php?title=Dyrektywa_test
// legacy — skip_until(endtest), bez parsowania zawartości.

#include <eu07/scene/context.hpp>
#include <eu07/scene/cursor.hpp>
#include <eu07/scene/match.hpp>

namespace eu07::scene::test {

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& ctx) {
    if (stream.empty() || !isKeyword(stream.peek().value, "test")) {
        return false;
    }

    DirectiveBlock block;
    block.line = stream.consume().sourceLine;

    while (!stream.empty() && !isKeyword(stream.peek().value, "endtest")) {
        block.tokens.push_back(stream.consume());
    }

    if (!stream.empty() && isKeyword(stream.peek().value, "endtest")) {
        block.tokens.push_back(stream.consume());
    }

    ctx.document.test.push_back(std::move(block));
    return true;
}

} // namespace eu07::scene::test
