#pragma once

// https://wiki.eu07.pl/index.php?title=Dyrektywa_sky
// sky sky endsky

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/detail/parse.hpp>
#include <eu07/scene/context.hpp>
#include <eu07/scene/match.hpp>

namespace eu07::scene::sky {

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& ctx) {
    if (stream.empty() || !isKeyword(stream.peek().value, "sky")) {
        return false;
    }

    DirectiveBlock block;
    block.line = stream.consume().sourceLine;
    const std::size_t anchor = stream.checkpoint();
    detail::ParseSession session(stream, block, anchor);

    if (!session.empty()) {
        session.take();
    }
    if (session.at("endsky")) {
        session.take();
    }

    ctx.document.sky.push_back(std::move(block));
    return true;
}

} // namespace eu07::scene::sky
