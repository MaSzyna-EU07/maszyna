#pragma once

// https://wiki.eu07.pl/index.php?title=Dyrektywa_area
// area name tracks endarea

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/detail/boundary.hpp>
#include <eu07/scene/detail/parse.hpp>
#include <eu07/scene/context.hpp>
#include <eu07/scene/match.hpp>

namespace eu07::scene::area {

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& ctx) {
    if (stream.empty() || !isKeyword(stream.peek().value, "area")) {
        return false;
    }

    const std::size_t anchor = stream.checkpoint();
    DirectiveBlock block;
    block.line = stream.consume().sourceLine;
    detail::ParseSession session(stream, block, anchor);

    while (!session.empty()) {
        if (session.at("endarea")) {
            session.take();
            ctx.document.area.push_back(std::move(block));
            return true;
        }

        const std::string_view value = session.peek().value;
        if (detail::isTopLevelStarter(value) || detail::isEmbeddedInNode(value)) {
            session.fail();
            stream.rewind(anchor);
            return false;
        }

        session.take();
    }

    session.fail();
    stream.rewind(anchor);
    return false;
}

} // namespace eu07::scene::area
