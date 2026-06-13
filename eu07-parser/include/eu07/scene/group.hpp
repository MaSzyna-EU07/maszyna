#pragma once

// https://wiki.eu07.pl/index.php?title=Dyrektywa_group
// group … endgroup — jak oryginal: tylko push/pop grupy, bez połykania tokenów.

#include <eu07/scene/context.hpp>
#include <eu07/scene/cursor.hpp>
#include <eu07/scene/match.hpp>

namespace eu07::scene::group {

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& ctx) {
    if (stream.empty() || !isKeyword(stream.peek().value, "group")) {
        return false;
    }

    DirectiveBlock block;
    block.line = stream.consume().sourceLine;
    ctx.scratch.openGroup();
    ctx.document.group.push_back(std::move(block));
    return true;
}

} // namespace eu07::scene::group
