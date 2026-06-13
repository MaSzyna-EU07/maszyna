#pragma once



#include <eu07/parser.hpp>



#include <algorithm>

#include <cctype>

#include <filesystem>

#include <optional>

#include <span>

#include <string>

#include <string_view>

#include <vector>



namespace eu07::scene::detail {



enum class IncludeScanMode { Code, LineComment, BlockComment, QuotedString };



[[nodiscard]] inline std::string normalizeIncludePathToken(std::string path) {

    std::replace(path.begin(), path.end(), '\\', '/');

    return path;

}



[[nodiscard]] inline std::filesystem::path resolveIncludePath(

    const std::filesystem::path& baseDirectory,

    const std::string& fileToken) {

    const std::filesystem::path relative(normalizeIncludePathToken(fileToken));

    if (relative.is_absolute()) {

        return relative;

    }

    return baseDirectory / relative;

}



[[nodiscard]] inline bool isIncFile(const std::string& fileToken) {

    if (fileToken.size() < 4) {

        return false;

    }

    const std::string_view tail(fileToken.data() + fileToken.size() - 4, 4);

    std::string lower(tail);

    for (char& ch : lower) {

        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));

    }

    return lower == ".inc";

}



[[nodiscard]] inline bool tryParseIncludeParameterRef(

    const std::string_view text,

    const std::size_t pos,

    std::size_t& endOut) {

    if (pos + 3 >= text.size() || text[pos] != '(' || text[pos + 1] != 'p') {

        return false;

    }



    std::size_t i = pos + 2;

    if (i >= text.size() || !std::isdigit(static_cast<unsigned char>(text[i]))) {

        return false;

    }



    while (i < text.size() && std::isdigit(static_cast<unsigned char>(text[i]))) {

        ++i;

    }



    if (i >= text.size() || text[i] != ')') {

        return false;

    }



    endOut = i;

    return true;

}

[[nodiscard]] inline bool parseIncludeParameterIndex(
    const std::string_view text,
    std::uint8_t& indexOut) {
    std::size_t end = 0;
    if (!tryParseIncludeParameterRef(text, 0, end) || end + 1 != text.size()) {
        return false;
    }

    std::size_t value = 0;
    for (std::size_t i = 2; i < text.size() && std::isdigit(static_cast<unsigned char>(text[i])); ++i) {
        value = value * 10 + static_cast<std::size_t>(text[i] - '0');
    }
    if (value == 0 || value > 255) {
        return false;
    }

    indexOut = static_cast<std::uint8_t>(value);
    return true;
}

[[nodiscard]] inline std::string resolveIncludeParameter(

    const std::string_view indexText,

    const std::span<const std::string> parameters) {

    std::size_t index = 0;

    try {

        index = std::stoul(std::string(indexText));

    } catch (...) {

        index = 0;

    }



    if (index >= 1 && index <= parameters.size()) {

        return parameters[index - 1];

    }

    return "none";

}



[[nodiscard]] inline std::string applyIncludeParameters(

    std::string text,

    const std::span<const std::string> parameters) {

    if (parameters.empty()) {

        return text;

    }



    std::string out;

    out.reserve(text.size());



    IncludeScanMode mode = IncludeScanMode::Code;



    for (std::size_t i = 0; i < text.size(); ) {

        const char ch = text[i];



        if (mode == IncludeScanMode::QuotedString) {

            out.push_back(ch);

            if (ch == '\\' && i + 1 < text.size()) {

                out.push_back(text[i + 1]);

                i += 2;

                continue;

            }

            if (ch == '"') {

                mode = IncludeScanMode::Code;

            }

            ++i;

            continue;

        }



        if (mode == IncludeScanMode::Code) {

            if (ch == '"') {

                out.push_back(ch);

                mode = IncludeScanMode::QuotedString;

                ++i;

                continue;

            }

            if (ch == '/' && i + 1 < text.size()) {

                if (text[i + 1] == '/') {

                    out.push_back(ch);

                    out.push_back(text[i + 1]);

                    i += 2;

                    mode = IncludeScanMode::LineComment;

                    continue;

                }

                if (text[i + 1] == '*') {

                    out.push_back(ch);

                    out.push_back(text[i + 1]);

                    i += 2;

                    mode = IncludeScanMode::BlockComment;

                    continue;

                }

            }

        } else if (mode == IncludeScanMode::BlockComment) {

            if (ch == '*' && i + 1 < text.size() && text[i + 1] == '/') {

                out.push_back(ch);

                out.push_back(text[i + 1]);

                i += 2;

                mode = IncludeScanMode::Code;

                continue;

            }

        }



        std::size_t refEnd = 0;

        if (tryParseIncludeParameterRef(text, i, refEnd)) {

            const std::string replacement = resolveIncludeParameter(

                text.substr(i + 2, refEnd - (i + 2)),

                parameters);

            out.append(replacement);

            i = refEnd + 1;

            continue;

        }



        out.push_back(ch);

        if (ch == '\n' && mode == IncludeScanMode::LineComment) {

            mode = IncludeScanMode::Code;

        }

        ++i;

    }



    return out;

}



[[nodiscard]] inline eu07::ParseResult parseIncludedFile(

    const std::filesystem::path& path,

    const std::vector<std::string>& parameters) {

    const eu07::RawFile raw = eu07::readRawFile(path);

    const std::string expanded = applyIncludeParameters(raw.data, parameters);

    return eu07::parseText(expanded);

}



[[nodiscard]] inline bool isIncludeCycle(

    const std::filesystem::path& resolved,

    const std::vector<std::filesystem::path>& includeStack) {

    const std::filesystem::path normalized = resolved.lexically_normal();

    for (const std::filesystem::path& entry : includeStack) {

        if (entry.lexically_normal() == normalized) {

            return true;

        }

    }

    return false;

}



// Jak cParser / eu7_loader::resolve_parser_include_path — wzgledem scenery/, nie parent pliku.

[[nodiscard]] inline std::optional<std::filesystem::path> findSceneryRootInPath(

    std::filesystem::path inputPath) {

    inputPath = inputPath.lexically_normal();

    while (!inputPath.empty()) {

        if (inputPath.filename() == "scenery") {

            return inputPath;

        }

        const std::filesystem::path parent = inputPath.parent_path();

        if (parent == inputPath) {

            break;

        }

        inputPath = parent;

    }

    return std::nullopt;

}



[[nodiscard]] inline std::string relativeSceneryFile(

    const std::filesystem::path& sceneryRoot,

    const std::filesystem::path& absolutePath) {

    std::error_code ec;

    const std::filesystem::path relative =

        std::filesystem::relative(absolutePath.lexically_normal(), sceneryRoot.lexically_normal(), ec);

    if (ec) {

        return absolutePath.filename().generic_string();

    }

    return relative.generic_string();

}



[[nodiscard]] inline std::filesystem::path resolveParserIncludePath(

    const std::filesystem::path& sceneryRoot,

    const std::string& currentRelativeFile,

    const std::string& fileToken) {

    std::string reference = normalizeIncludePathToken(fileToken);

    while (!reference.empty() && reference[0] == '$') {

        reference.erase(0, 1);

    }



    const std::filesystem::path root = sceneryRoot.lexically_normal();



    if (reference.find('/') != std::string::npos) {

        return root / std::filesystem::path(reference);

    }

    if (!currentRelativeFile.empty()) {

        const std::size_t slash = currentRelativeFile.find_last_of('/');

        if (slash != std::string::npos) {

            return root / std::filesystem::path(currentRelativeFile.substr(0, slash + 1) + reference);

        }

    }

    return root / std::filesystem::path(reference);

}



[[nodiscard]] inline std::filesystem::path resolveIncludeSourcePath(

    const std::filesystem::path& sceneryRoot,

    const std::string& currentRelativeFile,

    const std::string& fileToken) {

    const std::filesystem::path primary =

        resolveParserIncludePath(sceneryRoot, currentRelativeFile, fileToken);

    if (std::filesystem::exists(primary)) {

        return primary;

    }



    std::filesystem::path stem = primary;

    stem.replace_extension();

    for (const std::string_view ext : {".scm", ".inc", ".scn"}) {

        std::filesystem::path candidate = stem;

        candidate.replace_extension(ext);

        if (std::filesystem::exists(candidate)) {

            return candidate;

        }

    }



    // Include bez katalogu (tree.inc, grass.inc) — plik w korzeniu scenery/, nie w podfolderze parenta.

    std::string reference = normalizeIncludePathToken(fileToken);

    while (!reference.empty() && reference[0] == '$') {

        reference.erase(0, 1);

    }

    if (reference.find('/') == std::string::npos && primary != sceneryRoot / reference) {

        const std::filesystem::path rootCandidate =

            sceneryRoot.lexically_normal() / std::filesystem::path(reference);

        if (std::filesystem::exists(rootCandidate)) {

            return rootCandidate;

        }

        std::filesystem::path rootStem = rootCandidate;

        rootStem.replace_extension();

        for (const std::string_view ext : {".scm", ".inc", ".scn"}) {

            std::filesystem::path candidate = rootStem;

            candidate.replace_extension(ext);

            if (std::filesystem::exists(candidate)) {

                return candidate;

            }

        }

    }

    return primary;

}



[[nodiscard]] inline std::filesystem::path resolveExpandedIncludePath(

    const std::filesystem::path& baseDirectory,

    const std::vector<std::filesystem::path>& includeStack,

    const std::string& fileToken) {

    const std::filesystem::path anchor =

        baseDirectory.empty() ? std::filesystem::current_path() : baseDirectory;

    if (const std::optional<std::filesystem::path> sceneryRoot = findSceneryRootInPath(anchor)) {

        std::string currentRelative;

        if (!includeStack.empty()) {

            currentRelative = relativeSceneryFile(*sceneryRoot, includeStack.back());

        }

        return resolveIncludeSourcePath(*sceneryRoot, currentRelative, fileToken);

    }

    return resolveIncludePath(baseDirectory, fileToken);

}



} // namespace eu07::scene::detail


