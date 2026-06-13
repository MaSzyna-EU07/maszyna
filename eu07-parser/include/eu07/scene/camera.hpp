#pragma once

// https://wiki.eu07.pl/index.php?title=Dyrektywa_camera
// camera position rotation index endcamera

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/detail/parse.hpp>
#include <eu07/scene/context.hpp>
#include <eu07/scene/match.hpp>

namespace eu07::scene::camera {

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& ctx) {
    if (stream.empty() || !isKeyword(stream.peek().value, "camera")) {
        return false;
    }

    DirectiveBlock block;
    block.line = stream.consume().sourceLine;
    const std::size_t anchor = stream.checkpoint();
    detail::ParseSession session(stream, block, anchor);

    while (!session.empty()) {
        if (session.at("endcamera")) {
            session.take();
            break;
        }
        session.take();
    }

    ctx.document.camera.push_back(std::move(block));
    return true;
}

} // namespace eu07::scene::camera
