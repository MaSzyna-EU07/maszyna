#pragma once

// https://wiki.eu07.pl/index.php?title=Dyrektywa_FirstInit
// FirstInit — samo slowo kluczowe, bez end*

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/context.hpp>
#include <eu07/scene/match.hpp>

namespace eu07::scene::first_init {

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& ctx) {
    if (stream.empty() || !isKeywordIgnoreCase(stream.peek().value, "FirstInit")) {
        return false;
    }

    DirectiveBlock block;
    block.line = stream.consume().sourceLine;
    ctx.document.firstInit.push_back(std::move(block));
    return true;
}

} // namespace eu07::scene::first_init
