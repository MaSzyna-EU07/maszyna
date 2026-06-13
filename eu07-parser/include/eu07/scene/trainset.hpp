#pragma once

// https://wiki.eu07.pl/index.php?title=Dyrektywa_trainset
// trainset name track offset velocity — tryb scratchpadu do endtrainset.

#include <eu07/scene/context.hpp>
#include <eu07/scene/cursor.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/parse_util.hpp>

namespace eu07::scene::trainset {

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& ctx) {
    if (stream.empty() || !isKeyword(stream.peek().value, "trainset")) {
        return false;
    }

    if (ctx.scratch.trainset.isOpen) {
        ctx.scratch.closeTrainset();
    }

    DirectiveBlock block;
    SourceToken head = stream.consume();
    block.line = head.sourceLine;

    std::vector<SourceToken> raw;
    raw.push_back(std::move(head));
    std::string name;
    std::string track;
    double offset = 0.0;
    double velocity = 0.0;
    if (!node::io::takeString(stream, raw, name) || !node::io::takeString(stream, raw, track) ||
        !node::io::takeDouble(stream, raw, offset) ||
        !node::io::takeDouble(stream, raw, velocity)) {
        return false;
    }

    block.tokens = std::move(raw);
    ctx.scratch.openTrainset();
    ctx.scratch.trainset.name = std::move(name);
    ctx.scratch.trainset.track = std::move(track);
    ctx.scratch.trainset.offset = offset;
    ctx.scratch.trainset.velocity = velocity;
    ctx.scratch.trainset.docIndex = ctx.document.trainset.size();

    ctx.document.trainset.push_back(std::move(block));
    return true;
}

} // namespace eu07::scene::trainset
