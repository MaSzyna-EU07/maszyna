#pragma once

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/document.hpp>
#include <eu07/scene/scratch.hpp>

#include <filesystem>
#include <vector>

namespace eu07::scene {

struct ParseContext {
    SceneDocument& document;
    SceneScratchpad scratch;
    std::filesystem::path baseDirectory;
    std::vector<std::filesystem::path> includeStack;
    std::vector<std::vector<std::string>> activeIncludeParameters;
    bool expandIncludes = true;
    // PACK compose needs includes + models only; skip heavy geometry nodes (CityGML teren).
    bool packComposeLightweight = false;
};

using DirectiveParser = bool (*)(TokenStream& stream, ParseContext& context);

namespace detail {

struct IncludeExpansionRequest {
    std::size_t entryIndex = 0;
    std::string file;
    std::vector<std::string> parameters;
};

void expandInclude(ParseContext& context, const IncludeExpansionRequest& request);

} // namespace detail

} // namespace eu07::scene
