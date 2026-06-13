#pragma once

// https://wiki.eu07.pl/index.php?title=Dyrektywa_isolated
// isolated name tracks endisolated
// (nie mylic z isolated Nazwa wewnatrz node::track)

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/detail/boundary.hpp>
#include <eu07/scene/detail/parse.hpp>
#include <eu07/scene/context.hpp>
#include <eu07/scene/match.hpp>

namespace eu07::scene::isolated {

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& ctx) {
    if (stream.empty() || !isKeyword(stream.peek().value, "isolated")) {
        return false;
    }

    const std::size_t anchor = stream.checkpoint();
    DirectiveBlock block;
    block.line = stream.consume().sourceLine;
    detail::ParseSession session(stream, block, anchor);

    while (!session.empty()) {
        if (session.at("endisolated")) {
            session.take();
            ctx.document.isolated.push_back(std::move(block));
            return true;
        }

        const std::string_view value = session.peek().value;
        if (detail::isTopLevelStarter(value) || detail::isEmbeddedInNode(value) ||
            isKeyword(value, "isolated")) {
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

} // namespace eu07::scene::isolated
