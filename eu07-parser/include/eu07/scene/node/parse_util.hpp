#pragma once

#include <eu07/parser.hpp>
#include <eu07/scene/cursor.hpp>
#include <eu07/scene/include_resolve.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/types.hpp>

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace eu07::scene::node::io {

[[nodiscard]] inline bool isEndMarkerFor(
    const std::string_view token,
    const std::string_view subtype,
    const std::string_view endMarker) noexcept {
    if (isKeyword(token, endMarker)) {
        return true;
    }
    if (isKeyword(subtype, "triangle_strip") && isKeyword(token, "endtri")) {
        return true;
    }
    if (isKeyword(subtype, "triangles") &&
        (isKeyword(token, "endtri") || isKeyword(token, "endtriangles"))) {
        return true;
    }
    if (isKeyword(subtype, "triangle_fan") && isKeyword(token, "endtri")) {
        return true;
    }
    if (isKeyword(subtype, "lines") &&
        (isKeyword(token, "endlines") || isKeyword(token, "endline"))) {
        return true;
    }
    if (isKeyword(subtype, "line_strip") &&
        (isKeyword(token, "endline_strip") || isKeyword(token, "endline"))) {
        return true;
    }
    if (isKeyword(subtype, "line_loop") &&
        (isKeyword(token, "endline_loop") || isKeyword(token, "endline"))) {
        return true;
    }
    if (isKeyword(subtype, "eventlauncher") &&
        (isKeyword(token, "end") || isKeyword(token, endMarker))) {
        return true;
    }
    return false;
}

inline void appendRaw(std::vector<SourceToken>& raw, const SourceToken& token) {
    raw.push_back(token);
}

[[nodiscard]] inline bool isMissingNumericToken(const std::string_view text) noexcept {
    return text.empty() || isKeywordIgnoreCase(text, "none") ||
           isKeywordIgnoreCase(text, "undefined");
}

[[nodiscard]] inline std::optional<double> parseDouble(const std::string_view text) {
    if (isMissingNumericToken(text)) {
        return 0.0;
    }

    try {
        std::size_t used = 0;
        const double value = std::stod(std::string(text), &used);
        if (used != text.size()) {
            return std::nullopt;
        }
        return value;
    } catch (...) {
        return std::nullopt;
    }
}

[[nodiscard]] inline bool takeToken(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    SourceToken& out) {
    if (stream.empty()) {
        return false;
    }
    out = stream.consume();
    appendRaw(raw, out);
    return true;
}

[[nodiscard]] inline bool takeString(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    std::string& out) {
    SourceToken token;
    if (!takeToken(stream, raw, token)) {
        return false;
    }
    out = token.value;
    return true;
}

[[nodiscard]] inline bool takeDouble(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    double& out) {
    SourceToken token;
    if (!takeToken(stream, raw, token)) {
        return false;
    }
    const std::optional<double> value = parseDouble(token.value);
    if (!value) {
        return false;
    }
    out = *value;
    return true;
}

[[nodiscard]] inline bool takeDoubleOrPlaceholder(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    double& out) {
    SourceToken token;
    if (!takeToken(stream, raw, token)) {
        return false;
    }
    if (const std::optional<double> value = parseDouble(token.value)) {
        out = *value;
        return true;
    }
    std::uint8_t paramIndex = 0;
    if (detail::parseIncludeParameterIndex(token.value, paramIndex)) {
        out = 0.0;
        return true;
    }
    return false;
}

[[nodiscard]] inline bool takeVec3(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    Vec3& out) {
    return takeDouble(stream, raw, out.x) && takeDouble(stream, raw, out.y) &&
           takeDouble(stream, raw, out.z);
}

[[nodiscard]] inline bool atEnd(
    TokenStream& stream,
    const std::string_view subtype,
    const std::string_view endMarker) {
    return !stream.empty() && isEndMarkerFor(stream.peek().value, subtype, endMarker);
}

[[nodiscard]] inline bool consumeEnd(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const std::string_view subtype,
    const std::string_view endMarker) {
    if (!atEnd(stream, subtype, endMarker)) {
        return false;
    }
    SourceToken token;
    return takeToken(stream, raw, token);
}

} // namespace eu07::scene::node::io
