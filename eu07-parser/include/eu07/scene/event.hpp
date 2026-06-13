#pragma once

// https://wiki.eu07.pl/index.php?title=Obiekt_event
// event name type delay … endevent

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/detail/parse.hpp>
#include <eu07/scene/context.hpp>
#include <eu07/scene/match.hpp>

namespace eu07::scene::event {

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& ctx) {
    if (stream.empty() || !isKeyword(stream.peek().value, "event")) {
        return false;
    }

    DirectiveBlock block;
    SourceToken head = stream.consume();
    block.line = head.sourceLine;
    block.tokens.push_back(std::move(head));
    const std::size_t anchor = stream.checkpoint();
    detail::ParseSession session(stream, block, anchor);

    while (!session.empty()) {
        if (session.at("endevent")) {
            session.take();
            break;
        }
        session.take();
    }

    ctx.document.event.push_back(std::move(block));
    return true;
}

} // namespace eu07::scene::event
