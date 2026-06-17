#pragma once

// https://wiki.eu07.pl/index.php?title=Plik_tekstowy
// + dyrektywy startera (//$…)

#include <array>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace eu07 {

struct SourceToken {
    std::string value;
    std::size_t sourceLine = 0; // indeks linii w pliku (0 = pierwsza)
};

// =============================================================================
// Tokeny (wiki)
// =============================================================================

[[nodiscard]] inline bool isFieldSeparator(const unsigned char ch) noexcept {
    return ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n' || ch == ';' || ch == ',';
}

inline void skipFieldSeparators(std::string_view& text) noexcept {
    while (!text.empty() && isFieldSeparator(static_cast<unsigned char>(text.front()))) {
        text.remove_prefix(1);
    }
}

inline void tokenizeInto(
    std::string_view buffer,
    std::vector<SourceToken>& out,
    const std::size_t sourceLine = 0) {
    const auto lineFor = [&](const char* /*tokenStart*/) -> std::size_t { return sourceLine; };

    std::string_view text = buffer;
    skipFieldSeparators(text);

    while (!text.empty()) {
        const char* const tokenStart = text.data();

        if (text.front() == '"') {
            text.remove_prefix(1);
            std::string quoted;
            while (!text.empty()) {
                if (text.front() == '"') {
                    text.remove_prefix(1);
                    break;
                }
                if (text.front() == '\\') {
                    text.remove_prefix(1);
                    if (text.empty()) {
                        break;
                    }
                    if (text.front() == '\\') {
                        quoted.push_back('\\');
                        text.remove_prefix(1);
                        continue;
                    }
                    quoted.push_back(text.front());
                    text.remove_prefix(1);
                    continue;
                }
                quoted.push_back(text.front());
                text.remove_prefix(1);
            }
            out.push_back(SourceToken{std::move(quoted), lineFor(tokenStart)});
            skipFieldSeparators(text);
            continue;
        }

        if (text.front() == '[') {
            const std::size_t close = text.find(']', 1);
            if (close != std::string_view::npos) {
                out.push_back(SourceToken{
                    std::string(text.data(), close + 1),
                    lineFor(tokenStart),
                });
                text.remove_prefix(close + 1);
                skipFieldSeparators(text);
                continue;
            }
            // Unterminated '[' (no matching ']' in the remainder): it is an
            // ordinary character, not a bracket token. The engine's default
            // break set is "\n\r\t ;", so '[' belongs inside identifiers such
            // as event names ("Kon_It2[Sch2"). Fall through to the general scan
            // below, which does NOT stop at '[' and therefore consumes it,
            // guaranteeing the outer loop makes forward progress (previously it
            // spun forever: the '[' was neither a bracket nor a separator nor a
            // token character, so text never shrank).
        }

        const char* const start = text.data();
        while (!text.empty() &&
               !isFieldSeparator(static_cast<unsigned char>(text.front())) &&
               text.front() != '"') {
            text.remove_prefix(1);
        }
        const std::size_t len = static_cast<std::size_t>(text.data() - start);
        if (len > 0) {
            out.push_back(SourceToken{std::string(start, len), lineFor(tokenStart)});
        }
        skipFieldSeparators(text);
    }
}

[[nodiscard]] inline std::vector<std::string> tokenize(std::string_view text) {
    std::vector<SourceToken> located;
    located.reserve(8);
    tokenizeInto(text, located);
    std::vector<std::string> tokens;
    tokens.reserve(located.size());
    for (SourceToken& token : located) {
        tokens.push_back(std::move(token.value));
    }
    return tokens;
}

// =============================================================================
// Dyrektywy startera (//$…)
// =============================================================================

enum class StarterId {
    Name,
    Description,
    Link,
    ConsistDesc,
    Image,
    ImageTrain,
    DecorSkip,
    Archive,
    Category,
    Error,
    GeoMap,
    Reference,
    TerrainRegen,
    ConsistParams,
    ExeVersion,
};

struct StarterDirective {
    StarterId id{};
    std::size_t line = 0;
    std::string value;
    bool hiddenConsist = false;
};

namespace detail {

inline constexpr std::array<std::pair<std::string_view, StarterId>, 15> kStarterTags{{
    {"//$decor", StarterId::DecorSkip},
    {"//$it", StarterId::ImageTrain},
    {"//$n", StarterId::Name},
    {"//$d", StarterId::Description},
    {"//$f", StarterId::Link},
    {"//$o", StarterId::ConsistDesc},
    {"//$i", StarterId::Image},
    {"//$a", StarterId::Archive},
    {"//$l", StarterId::Category},
    {"//$e", StarterId::Error},
    {"//$g", StarterId::GeoMap},
    {"//$r", StarterId::Reference},
    {"//$t", StarterId::TerrainRegen},
    {"//$w", StarterId::ConsistParams},
    {"//$x", StarterId::ExeVersion},
}};

[[nodiscard]] inline bool tagOk(const std::string_view line, const std::string_view tag) {
    return line.size() == tag.size() ||
           isFieldSeparator(static_cast<unsigned char>(line[tag.size()]));
}

[[nodiscard]] inline std::optional<StarterId> matchStarter(std::string_view line) {
    skipFieldSeparators(line);
    for (const auto& [tag, id] : kStarterTags) {
        if (line.starts_with(tag) && tagOk(line, tag)) {
            return id;
        }
    }
    return std::nullopt;
}

[[nodiscard]] inline bool isStarter(std::string_view line) {
    return matchStarter(line).has_value();
}

[[nodiscard]] inline bool isLineComment(std::string_view line) {
    skipFieldSeparators(line);
    return line.starts_with("//") && !isStarter(line);
}

[[nodiscard]] inline std::string_view stripInlineComment(std::string_view line) {
    skipFieldSeparators(line);
    if (line.starts_with("//$")) {
        return {};
    }
    const std::size_t pos = line.find("//");
    return pos == std::string_view::npos ? line : line.substr(0, pos);
}

} // namespace detail

[[nodiscard]] inline std::string_view starterTag(const StarterId id) {
    for (const auto& [tag, sid] : detail::kStarterTags) {
        if (sid == id) {
            return tag;
        }
    }
    return "//$?";
}

[[nodiscard]] inline std::array<std::string_view, 15> supportedStarterTags() {
    std::array<std::string_view, 15> tags{};
    for (std::size_t i = 0; i < detail::kStarterTags.size(); ++i) {
        tags[i] = detail::kStarterTags[i].first;
    }
    return tags;
}

inline void writeStartersReport(
    const std::filesystem::path& outPath,
    const std::vector<StarterDirective>& starters) {
    std::ofstream out(outPath);
    if (!out) {
        throw std::runtime_error("Nie mozna zapisac: " + outPath.string());
    }

    out << "# dyrektywy startera (//$…)\n";
    out << "# obslugiwane:\n";
    for (const std::string_view tag : supportedStarterTags()) {
        out << "#   " << tag << '\n';
    }
    out << "# wykryte: " << starters.size() << "\n\n";

    for (const StarterDirective& s : starters) {
        out << starterTag(s.id) << " L" << (s.line + 1) << ": \"" << s.value << '"';
        if (s.hiddenConsist) {
            out << " [ukryty]";
        }
        out << '\n';
    }
}

[[nodiscard]] inline std::optional<StarterDirective> parseStarter(
    std::string_view line,
    const std::size_t lineNo) {
    skipFieldSeparators(line);
    const std::optional<StarterId> id = detail::matchStarter(line);
    if (!id) {
        return std::nullopt;
    }

    for (const auto& [tag, sid] : detail::kStarterTags) {
        if (sid != *id) {
            continue;
        }

        StarterDirective d;
        d.id = *id;
        d.line = lineNo;

        std::string_view rest = line.substr(tag.size());
        skipFieldSeparators(rest);
        d.value.assign(rest);

        if (*id == StarterId::ConsistDesc && !d.value.empty() && d.value.front() == '-') {
            d.hiddenConsist = true;
            std::string_view tail = d.value;
            tail.remove_prefix(1);
            skipFieldSeparators(tail);
            d.value.assign(tail);
        }
        return d;
    }

    return std::nullopt;
}

// =============================================================================
// Komentarze: // … oraz /* … */ (wiki)
// =============================================================================

struct RawFile {
    std::string data;
    std::vector<std::string_view> lines;
};

[[nodiscard]] inline RawFile readRawFile(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Nie mozna otworzyc: " + path.string());
    }

    RawFile f;
    in.seekg(0, std::ios::end);
    const std::streamoff fileSize = in.tellg();
    in.seekg(0, std::ios::beg);
    if (fileSize > 0) {
        f.data.resize(static_cast<std::size_t>(fileSize));
        in.read(f.data.data(), fileSize);
    }
    f.lines.reserve(f.data.empty() ? 0 : f.data.size() / 24 + 4);

    std::size_t start = 0;
    for (std::size_t i = 0; i <= f.data.size(); ++i) {
        if (i == f.data.size() || f.data[i] == '\n') {
            std::size_t end = i;
            if (end > start && f.data[end - 1] == '\r') {
                --end;
            }
            f.lines.emplace_back(f.data.data() + start, end - start);
            start = i + 1;
        }
    }
    return f;
}

struct LogicalSegment {
    std::size_t sourceLine = 0;
    std::string_view text;
};

struct LogicalPass {
    std::vector<StarterDirective> starters;
    std::vector<LogicalSegment> segments;
    std::vector<std::string> spill;
};

[[nodiscard]] inline LogicalPass toLogicalLines(const std::vector<std::string_view>& raw) {
    LogicalPass out;
    out.segments.reserve(raw.size());
    out.starters.reserve(8);
    out.spill.reserve(4);
    bool inBlock = false;

    const auto emit = [&](std::string_view piece, const std::size_t lineNo) {
        if (piece.empty()) {
            return;
        }
        skipFieldSeparators(piece);

        if (detail::isStarter(piece)) {
            return;
        }
        if (detail::isLineComment(piece)) {
            return;
        }

        const std::string_view code = detail::stripInlineComment(piece);
        if (code.empty()) {
            return;
        }

        LogicalSegment segment;
        segment.sourceLine = lineNo;
        if (code.data() >= piece.data() && code.data() + code.size() <= piece.data() + piece.size()) {
            segment.text = code;
        } else {
            out.spill.emplace_back(code);
            segment.text = out.spill.back();
        }
        out.segments.push_back(segment);
    };

    for (std::size_t n = 0; n < raw.size(); ++n) {
        std::string_view line = raw[n];

        {
            std::string_view whole = line;
            skipFieldSeparators(whole);
            if (const std::optional<StarterDirective> starter = parseStarter(whole, n)) {
                out.starters.push_back(*starter);
                continue;
            }
        }

        if (!inBlock) {
            std::size_t at = 0;
            while (at < line.size()) {
                const std::size_t slash = line.find('/', at);
                if (slash == std::string_view::npos) {
                    emit(line.substr(at), n);
                    break;
                }
                if (slash + 1 < line.size() && line[slash + 1] == '/') {
                    if (slash > at) {
                        emit(line.substr(at, slash - at), n);
                    }
                    break;
                }
                if (slash + 1 < line.size() && line[slash + 1] == '*') {
                    if (slash > at) {
                        emit(line.substr(at, slash - at), n);
                    }
                    inBlock = true;
                    line = line.substr(slash + 2);
                    at = 0;
                    continue;
                }
                // Pojedynczy '/' to separator sciezki (np. 6-9-9/l33230.inc), nie komentarz.
                emit(line.substr(at), n);
                break;
            }
            continue;
        }

        while (inBlock && !line.empty()) {
            const std::size_t close = line.find("*/");
            if (close == std::string_view::npos) {
                line = {};
                break;
            }
            line.remove_prefix(close + 2);
            inBlock = false;
        }
        if (!inBlock && !line.empty()) {
            emit(line, n);
        }
    }

    return out;
}

[[nodiscard]] inline std::vector<StarterDirective> scanStarters(
    const std::vector<std::string_view>& rawLines) {
    return toLogicalLines(rawLines).starters;
}

[[nodiscard]] inline bool isBracketToken(std::string_view token) {
    skipFieldSeparators(token);
    return token.size() >= 2 && token.front() == '[' && token.back() == ']';
}

struct ParseOptions {
    bool tokenize = true;
};

struct ParseResult {
    std::vector<StarterDirective> starters;
    std::vector<SourceToken> tokens;
};

[[nodiscard]] inline std::vector<SourceToken> tokenizeSegments(
    const std::vector<LogicalSegment>& segments) {
    std::vector<SourceToken> tokens;
    if (segments.empty()) {
        return tokens;
    }

    std::size_t chars = 0;
    for (const LogicalSegment& segment : segments) {
        chars += segment.text.size();
    }
    tokens.reserve(chars / 10 + 16);

    for (const LogicalSegment& segment : segments) {
        tokenizeInto(segment.text, tokens, segment.sourceLine);
    }
    return tokens;
}

[[nodiscard]] inline ParseResult parseRawFile(RawFile raw, const ParseOptions& options = {}) {
    LogicalPass logical = toLogicalLines(raw.lines);

    ParseResult result;
    result.starters = std::move(logical.starters);

    if (options.tokenize) {
        result.tokens = tokenizeSegments(logical.segments);
    }
    return result;
}

[[nodiscard]] inline ParseResult parseFile(
    const std::filesystem::path& path,
    const ParseOptions& options = {}) {
    return parseRawFile(readRawFile(path), options);
}

[[nodiscard]] inline ParseResult parseText(const std::string& text) {
    RawFile raw;
    raw.data = text;
    std::size_t start = 0;
    for (std::size_t i = 0; i <= raw.data.size(); ++i) {
        if (i == raw.data.size() || raw.data[i] == '\n') {
            std::size_t end = i;
            if (end > start && raw.data[end - 1] == '\r') {
                --end;
            }
            raw.lines.emplace_back(raw.data.data() + start, end - start);
            start = i + 1;
        }
    }

    LogicalPass logical = toLogicalLines(raw.lines);

    ParseResult result;
    result.starters = std::move(logical.starters);
    result.tokens = tokenizeSegments(logical.segments);
    return result;
}

} // namespace eu07
