#pragma once

#include <eu07/parser.hpp>
#include <eu07/scene/runtime/basic_node.hpp>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace eu07::scene {

struct ParsedInclude {
    std::size_t line = 0;
    std::string file;
    std::vector<std::string> parameters;
    std::vector<SourceToken> raw;
    runtime::TransformContext siteTransform;
    bool expanded = false;
    std::optional<std::string> error;
};

} // namespace eu07::scene
