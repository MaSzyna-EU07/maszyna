#pragma once



// https://wiki.eu07.pl/index.php?title=Plik_scenerii



#include <eu07/parser.hpp>

#include <eu07/scene/context.hpp>

#include <eu07/scene/cursor.hpp>

#include <eu07/scene/dispatch_table.hpp>

#include <eu07/scene/document.hpp>

#include <eu07/scene/include_resolve.hpp>

#include <eu07/scene/parallel_models.hpp>

#include <filesystem>

#include <system_error>



namespace eu07::scene {



namespace detail {



inline ParsedInclude* includeEntryAt(ParseContext& context, const std::size_t entryIndex) noexcept {

    if (entryIndex >= context.document.include.size()) {

        return nullptr;

    }

    return &context.document.include[entryIndex];

}



inline void processStream(TokenStream& stream, ParseContext& context) {

    while (!stream.empty()) {

        if (dispatchDirective(stream, context)) {

            continue;

        }



        UnknownEntry unknown;

        unknown.line = stream.peek().sourceLine;

        unknown.token = stream.consume().value;

        context.document.unknown.push_back(std::move(unknown));

    }

}



inline void expandInclude(ParseContext& context, const IncludeExpansionRequest& request) {

    ParsedInclude* const entry = includeEntryAt(context, request.entryIndex);

    if (entry == nullptr) {

        return;

    }



    if (request.file.empty()) {

        entry->error = "pusta sciezka include";

        return;

    }



    const std::filesystem::path resolved = detail::resolveExpandedIncludePath(

        context.baseDirectory, context.includeStack, request.file);



    if (detail::isIncludeCycle(resolved, context.includeStack)) {

        entry->error = "cykl include: " + resolved.string();

        return;

    }



    std::error_code existsEc;

    if (!std::filesystem::exists(resolved, existsEc)) {

        entry->error = "nie znaleziono pliku: " + resolved.string();

        return;

    }



    context.includeStack.push_back(resolved.lexically_normal());



    const bool wrapIncGroup = detail::isIncFile(request.file);

    if (wrapIncGroup) {

        context.scratch.openGroup();

    }



    context.activeIncludeParameters.push_back(request.parameters);

    try {

        const eu07::ParseResult included = detail::parseIncludedFile(resolved, request.parameters);

        TokenStream childStream(included.tokens);

        processStream(childStream, context);



        if (ParsedInclude* const done = includeEntryAt(context, request.entryIndex)) {

            done->expanded = true;

        }

    } catch (const std::exception& ex) {

        if (ParsedInclude* const failed = includeEntryAt(context, request.entryIndex)) {

            failed->error = ex.what();

        }

    }

    context.activeIncludeParameters.pop_back();



    if (wrapIncGroup) {

        context.scratch.closeGroup();

    }



    if (!context.includeStack.empty()) {

        context.includeStack.pop_back();

    }

}



} // namespace detail



struct SceneProcessOptions {
    bool expandIncludes = true;
    bool packComposeLightweight = false;
};

[[nodiscard]] inline SceneProcessResult processScene(

    const ParseResult& parsed,

    const std::filesystem::path& baseDirectory = {},

    const SceneProcessOptions& options = {}) {

    if (!options.expandIncludes) {
        if (std::optional<SceneDocument> fast =
                detail::tryProcessFlatModels(parsed, baseDirectory)) {
            SceneProcessResult result;
            result.document = std::move(*fast);
            return result;
        }
    }

    TokenStream stream(parsed.tokens);

    SceneDocument document;



    std::filesystem::path includeRoot = baseDirectory;

    if (includeRoot.empty()) {

        includeRoot = std::filesystem::current_path();

    }



    ParseContext context{document, {}, includeRoot, {}};

    context.expandIncludes = options.expandIncludes;
    context.packComposeLightweight = options.packComposeLightweight;

    detail::processStream(stream, context);



    SceneProcessResult result;

    result.document = std::move(document);

    return result;

}



} // namespace eu07::scene


