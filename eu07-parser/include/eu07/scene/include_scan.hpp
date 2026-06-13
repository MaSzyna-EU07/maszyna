#pragma once

// Szybkie wyciagniecie sciezek z dyrektyw include — bez processScene / bake.

#include <eu07/parser.hpp>
#include <eu07/scene/match.hpp>

#include <filesystem>
#include <span>
#include <string>
#include <vector>

namespace eu07::scene {

[[nodiscard]] inline std::vector<std::string> scanIncludeFilePaths(
    const std::span<const SourceToken> tokens) {
    std::vector<std::string> files;
    files.reserve(8);

    for (std::size_t i = 0; i < tokens.size(); ++i) {
        if (!isKeyword(tokens[i].value, "include")) {
            continue;
        }
        if (i + 1 >= tokens.size()) {
            break;
        }

        files.push_back(tokens[i + 1].value);

        for (std::size_t j = i + 2; j < tokens.size(); ++j) {
            if (isKeyword(tokens[j].value, "end")) {
                i = j;
                break;
            }
        }
    }

    return files;
}

[[nodiscard]] inline std::vector<std::string> scanIncludeFilePaths(
    const std::filesystem::path& sourcePath) {
    const ParseResult parsed = parseFile(sourcePath);
    return scanIncludeFilePaths(parsed.tokens);
}

} // namespace eu07::scene
