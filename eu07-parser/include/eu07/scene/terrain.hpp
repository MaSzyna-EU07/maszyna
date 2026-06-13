#pragma once

// https://wiki.eu07.pl/index.php?title=Dyrektywa_terrain
// terrain terrain endterrain

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/detail/parse.hpp>
#include <eu07/scene/context.hpp>
#include <eu07/scene/match.hpp>

namespace eu07::scene::terrain {

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& ctx) {
    if (stream.empty() || !isKeyword(stream.peek().value, "terrain")) {
        return false;
    }

    DirectiveBlock block;
    block.line = stream.consume().sourceLine;
    const std::size_t anchor = stream.checkpoint();
    detail::ParseSession session(stream, block, anchor);

    if (!session.empty()) {
        session.take();
    }
    if (session.at("endterrain")) {
        session.take();
    }

    ctx.document.terrain.push_back(std::move(block));
    return true;
}

} // namespace eu07::scene::terrain
