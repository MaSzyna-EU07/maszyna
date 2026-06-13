#pragma once

#include <eu07/parser.hpp>
#include <eu07/scene/context.hpp>
#include <eu07/scene/cursor.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/parse_util.hpp>

#include <string>
#include <vector>

namespace eu07::scene::endgroup {

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& ctx) {
    if (stream.empty() || !isKeyword(stream.peek().value, "endgroup")) {
        return false;
    }

    const SourceToken endToken = stream.consume();
    DirectiveBlock block;
    block.line = endToken.sourceLine;
    block.tokens.push_back(endToken);
    ctx.scratch.closeGroup();
    ctx.document.group.push_back(std::move(block));
    return true;
}

} // namespace eu07::scene::endgroup

namespace eu07::scene::endorigin {

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& ctx) {
    if (stream.empty() || !isKeyword(stream.peek().value, "endorigin")) {
        return false;
    }

    const SourceToken endToken = stream.consume();
    DirectiveBlock block;
    block.line = endToken.sourceLine;
    block.tokens.push_back(endToken);
    ctx.scratch.popOrigin();
    ctx.document.origin.push_back(std::move(block));
    return true;
}

} // namespace eu07::scene::endorigin

namespace eu07::scene::endscale {

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& ctx) {
    if (stream.empty() || !isKeyword(stream.peek().value, "endscale")) {
        return false;
    }

    const SourceToken endToken = stream.consume();
    DirectiveBlock block;
    block.line = endToken.sourceLine;
    block.tokens.push_back(endToken);
    ctx.scratch.popScale();
    ctx.document.scale.push_back(std::move(block));
    return true;
}

} // namespace eu07::scene::endscale

namespace eu07::scene::assignment {

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& ctx) {
    if (stream.empty() || !isKeyword(stream.peek().value, "assignment")) {
        return false;
    }

    stream.consume();

    std::vector<SourceToken> raw;
    while (!stream.empty() && !isKeyword(stream.peek().value, "endassignment")) {
        std::string key;
        std::string value;
        if (!node::io::takeString(stream, raw, key) || !node::io::takeString(stream, raw, value)) {
            return false;
        }
        if (ctx.scratch.trainset.isOpen) {
            ctx.scratch.trainset.assignment.emplace_back(std::move(key), std::move(value));
        }
    }

    if (!stream.empty() && isKeyword(stream.peek().value, "endassignment")) {
        stream.consume();
    }
    return true;
}

} // namespace eu07::scene::assignment

namespace eu07::scene::endtrainset {

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& ctx) {
    if (stream.empty() || !isKeyword(stream.peek().value, "endtrainset")) {
        return false;
    }

    const SourceToken endToken = stream.consume();

    if (ctx.scratch.trainset.isOpen && ctx.scratch.trainset.docIndex) {
        ctx.document.trainset[*ctx.scratch.trainset.docIndex].tokens.push_back(endToken);
    }

    ctx.scratch.closeTrainset();
    return true;
}

} // namespace eu07::scene::endtrainset
