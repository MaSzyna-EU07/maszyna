#pragma once

// https://wiki.eu07.pl/index.php?title=Dyrektywa_include
// include file parameters end — ekspansja rekurencyjna (wariant 2)

#include <eu07/scene/context.hpp>
#include <eu07/scene/cursor.hpp>
#include <eu07/scene/include_resolve.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/parse_util.hpp>

namespace eu07::scene::scn_include {

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& ctx) {
    if (stream.empty() || !isKeyword(stream.peek().value, "include")) {
        return false;
    }

    ParsedInclude entry;
    SourceToken head = stream.consume();
    entry.line = head.sourceLine;
    entry.raw.push_back(head);

    if (stream.empty()) {
        entry.error = "brak sciezki pliku";
        ctx.document.include.push_back(std::move(entry));
        return true;
    }

    SourceToken fileToken;
    if (!node::io::takeToken(stream, entry.raw, fileToken)) {
        entry.error = "brak sciezki pliku";
        ctx.document.include.push_back(std::move(entry));
        return true;
    }
    entry.file = fileToken.value;

    while (!stream.empty()) {
        if (isKeyword(stream.peek().value, "end")) {
            entry.raw.push_back(stream.consume());
            break;
        }
        SourceToken param;
        if (!node::io::takeToken(stream, entry.raw, param)) {
            break;
        }
        std::string value = param.value;
        if (!ctx.activeIncludeParameters.empty()) {
            value = detail::applyIncludeParameters(value, ctx.activeIncludeParameters.back());
        }
        entry.parameters.push_back(std::move(value));
    }

    entry.siteTransform.originStack = ctx.scratch.location.originStack;
    entry.siteTransform.scaleStack = ctx.scratch.location.scaleStack;
    entry.siteTransform.rotation = ctx.scratch.location.rotation;
    entry.siteTransform.groupStackDepth = ctx.scratch.group.activeStack.size();

    const std::size_t includeIndex = ctx.document.include.size();
    ctx.document.include.push_back(std::move(entry));

    if (ctx.expandIncludes) {
        detail::IncludeExpansionRequest request;
        request.entryIndex = includeIndex;
        request.file = ctx.document.include[includeIndex].file;
        request.parameters = ctx.document.include[includeIndex].parameters;
        detail::expandInclude(ctx, request);
    }

    return true;
}

} // namespace eu07::scene::scn_include
