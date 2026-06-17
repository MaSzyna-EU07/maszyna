#pragma once

// Fast path: plaskie pliki .scm — kazdy wpis w jednej linii logicznej.
// (1) same "node model" (flora) albo (2) same "include;...;end" (placementy).
// Pomija tokenizacje calego pliku; parse linii rownolegle (per-line tokenize).

#include <eu07/nmt/parallel.hpp>
#include <eu07/parser.hpp>
#include <eu07/scene/context.hpp>
#include <eu07/scene/cursor.hpp>
#include <eu07/scene/dispatch_table.hpp>
#include <eu07/scene/document.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node.hpp>
#include <eu07/scene/node/model.hpp>

#include <atomic>
#include <array>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace eu07::scene::detail {

enum class FlatFileKind {
    None,
    Models,
    Includes,
};

inline constexpr std::size_t kParallelFlatLineThreshold = 2048;

[[nodiscard]] inline bool segmentStartsWithKeyword(
    std::string_view text,
    const std::string_view keyword) {
    skipFieldSeparators(text);
    if (text.size() < keyword.size()) {
        return false;
    }
    if (text.compare(0, keyword.size(), keyword) != 0) {
        return false;
    }
    if (text.size() == keyword.size()) {
        return true;
    }
    return isFieldSeparator(static_cast<unsigned char>(text[keyword.size()]));
}

[[nodiscard]] inline FlatFileKind classifyFlatFile(const std::vector<LogicalSegment>& segments) {
    FlatFileKind kind = FlatFileKind::None;
    for (const LogicalSegment& segment : segments) {
        std::string_view text = segment.text;
        skipFieldSeparators(text);
        if (text.empty()) {
            continue;
        }

        if (segmentStartsWithKeyword(text, "node")) {
            if (text.find("endmodel") == std::string_view::npos) {
                return FlatFileKind::None;
            }
            if (kind == FlatFileKind::Includes) {
                return FlatFileKind::None;
            }
            kind = FlatFileKind::Models;
            continue;
        }

        if (segmentStartsWithKeyword(text, "include")) {
            if (text.find("end") == std::string_view::npos) {
                return FlatFileKind::None;
            }
            if (kind == FlatFileKind::Models) {
                return FlatFileKind::None;
            }
            kind = FlatFileKind::Includes;
            continue;
        }

        return FlatFileKind::None;
    }
    return kind;
}

struct ModelParseJob {
    std::size_t segment_index = 0;
    NodeHeader header;
};

[[nodiscard]] inline bool skipModelBody(TokenStream& stream) {
    std::vector<SourceToken> raw;
    while (!stream.empty() && !node::io::atEnd(stream, node_model::kSubtype, node_model::kEndMarker)) {
        stream.consume();
    }
    return node::io::consumeEnd(stream, raw, node_model::kSubtype, node_model::kEndMarker);
}

[[nodiscard]] inline bool parseModelSegment(
    const LogicalSegment& segment,
    ParsedNodeModel& out) {
    const std::vector<SourceToken> tokens = tokenizeSegments({segment});
    TokenStream stream(tokens);
    NodeHeader header;
    std::string subtype;
    std::vector<SourceToken> raw;
    if (!node::io::consumeHeader(stream, header, subtype, raw) ||
        !isKeyword(subtype, node_model::kSubtype)) {
        return false;
    }
    return node_model::parseBody(stream, raw, header, out);
}

[[nodiscard]] inline std::string_view trimFieldView(std::string_view text) {
    skipFieldSeparators(text);
    while (!text.empty() && isFieldSeparator(static_cast<unsigned char>(text.back()))) {
        text.remove_suffix(1);
    }
    return text;
}

[[nodiscard]] inline std::vector<std::string_view> splitSemicolonFields(std::string_view text) {
    std::vector<std::string_view> fields;
    while (!text.empty()) {
        skipFieldSeparators(text);
        if (text.empty()) {
            break;
        }
        const std::size_t separator = text.find(';');
        if (separator == std::string_view::npos) {
            fields.push_back(trimFieldView(text));
            break;
        }
        fields.push_back(trimFieldView(text.substr(0, separator)));
        text.remove_prefix(separator + 1);
    }
    return fields;
}

[[nodiscard]] inline bool parseIncludeSegmentSemicolon(
    const LogicalSegment& segment,
    ParsedInclude& out) {
    std::string_view text = segment.text;
    if (const std::size_t comment = text.find("//"); comment != std::string_view::npos) {
        text = text.substr(0, comment);
    }
    skipFieldSeparators(text);
    if (!segmentStartsWithKeyword(text, "include")) {
        return false;
    }

    const std::vector<std::string_view> fields = splitSemicolonFields(text);
    if (fields.empty() || !isKeyword(fields.front(), "include")) {
        return false;
    }

    out = ParsedInclude {};
    out.line = segment.sourceLine;
    if (fields.size() < 2) {
        out.error = "brak sciezki pliku";
        return true;
    }

    out.file = std::string(fields[1]);
    for (std::size_t index = 2; index < fields.size(); ++index) {
        if (isKeyword(fields[index], "end")) {
            break;
        }
        out.parameters.emplace_back(fields[index]);
    }
    return true;
}

[[nodiscard]] inline bool parseIncludeSegment(
    const LogicalSegment& segment,
    ParsedInclude& out) {
    if (parseIncludeSegmentSemicolon(segment, out)) {
        return true;
    }
    const std::vector<SourceToken> tokens = tokenizeSegments({segment});
    TokenStream stream(tokens);
    if (stream.empty() || !isKeyword(stream.peek().value, "include")) {
        return false;
    }

    out = ParsedInclude {};
    out.line = segment.sourceLine;
    stream.consume();

    if (stream.empty()) {
        out.error = "brak sciezki pliku";
        return true;
    }

    SourceToken file_token;
    if (!node::io::takeToken(stream, out.raw, file_token)) {
        out.error = "brak sciezki pliku";
        return true;
    }
    out.file = file_token.value;

    while (!stream.empty()) {
        if (isKeyword(stream.peek().value, "end")) {
            stream.consume();
            break;
        }
        SourceToken param;
        if (!node::io::takeToken(stream, out.raw, param)) {
            break;
        }
        out.parameters.push_back(param.value);
    }

    out.raw.clear();
    return true;
}

[[nodiscard]] inline std::optional<SceneDocument> parseFlatModels(
    const std::vector<LogicalSegment>& segments) {
    SceneDocument document;
    if (segments.empty()) {
        return document;
    }

    if (segments.size() < kParallelFlatLineThreshold) {
        document.nodeModel.reserve(segments.size());
        for (const LogicalSegment& segment : segments) {
            ParsedNodeModel model;
            if (!parseModelSegment(segment, model)) {
                return std::nullopt;
            }
            document.nodeModel.push_back(std::move(model));
        }
        return document;
    }

    document.nodeModel.resize(segments.size());
    const unsigned worker_count =
        std::min(static_cast<unsigned>(segments.size()), eu07::nmt::workerThreadCount());
    std::atomic<std::size_t> next_job { 0 };
    std::atomic<bool> failed { false };

    auto worker = [&]() {
        while (true) {
            if (failed.load(std::memory_order_relaxed)) {
                return;
            }
            const std::size_t index = next_job.fetch_add(1, std::memory_order_relaxed);
            if (index >= segments.size()) {
                return;
            }
            ParsedNodeModel model;
            if (!parseModelSegment(segments[index], model)) {
                failed.store(true, std::memory_order_relaxed);
                return;
            }
            document.nodeModel[index] = std::move(model);
        }
    };

    std::vector<std::thread> threads;
    const unsigned launch = std::max(1u, worker_count);
    threads.reserve(launch);
    for (unsigned i = 0; i < launch; ++i) {
        threads.emplace_back(worker);
    }
    for (std::thread& thread : threads) {
        thread.join();
    }

    if (failed.load(std::memory_order_relaxed)) {
        return std::nullopt;
    }
    return document;
}

[[nodiscard]] inline std::optional<SceneDocument> parseFlatIncludes(
    const std::vector<LogicalSegment>& segments) {
    SceneDocument document;
    document.include.resize(segments.size());

    if (segments.size() < kParallelFlatLineThreshold) {
        for (std::size_t index = 0; index < segments.size(); ++index) {
            if (!parseIncludeSegment(segments[index], document.include[index])) {
                return std::nullopt;
            }
        }
        return document;
    }

    std::atomic<std::size_t> next_job { 0 };
    std::atomic<bool> failed { false };

    auto worker = [&]() {
        while (true) {
            if (failed.load(std::memory_order_relaxed)) {
                return;
            }
            const std::size_t index = next_job.fetch_add(1, std::memory_order_relaxed);
            if (index >= segments.size()) {
                return;
            }
            ParsedInclude entry;
            if (!parseIncludeSegment(segments[index], entry)) {
                failed.store(true, std::memory_order_relaxed);
                return;
            }
            document.include[index] = std::move(entry);
        }
    };

    const unsigned worker_count =
        std::min(static_cast<unsigned>(segments.size()), eu07::nmt::workerThreadCount());
    std::vector<std::thread> threads;
    const unsigned launch = std::max(1u, worker_count);
    threads.reserve(launch);
    for (unsigned i = 0; i < launch; ++i) {
        threads.emplace_back(worker);
    }
    for (std::thread& thread : threads) {
        thread.join();
    }

    if (failed.load(std::memory_order_relaxed)) {
        return std::nullopt;
    }
    return document;
}

[[nodiscard]] inline std::optional<SceneDocument> tryProcessFlatLogical(
    const LogicalPass& logical,
    const std::filesystem::path& /*baseDirectory*/) {
    const FlatFileKind kind = classifyFlatFile(logical.segments);
    if (kind == FlatFileKind::None) {
        return std::nullopt;
    }
    if (kind == FlatFileKind::Models) {
        return parseFlatModels(logical.segments);
    }
    return parseFlatIncludes(logical.segments);
}

inline constexpr std::size_t kParallelModelParseThreshold = 4096;

[[nodiscard]] inline std::optional<SceneDocument> tryProcessFlatModels(
    const ParseResult& parsed,
    const std::filesystem::path& baseDirectory) {
    if (parsed.tokens.empty()) {
        return SceneDocument {};
    }

    SceneDocument document;
    std::filesystem::path includeRoot = baseDirectory;
    if (includeRoot.empty()) {
        includeRoot = std::filesystem::current_path();
    }

    ParseContext context { document, {}, includeRoot, {} };
    context.expandIncludes = false;

    TokenStream stream(parsed.tokens);
    std::vector<ModelParseJob> jobs;
    jobs.reserve(parsed.tokens.size() / 16);

    while (!stream.empty()) {
        if (detail::dispatchDirective(stream, context)) {
            continue;
        }

        if (!stream.empty() && isKeyword(stream.peek().value, "node")) {
            const std::size_t anchor = stream.checkpoint();
            NodeHeader header;
            std::string subtype;
            std::vector<SourceToken> raw;
            if (!node::io::consumeHeader(stream, header, subtype, raw)) {
                return std::nullopt;
            }
            if (!isKeyword(subtype, node_model::kSubtype)) {
                return std::nullopt;
            }
            node::applyScratchContext(header, context.scratch);
            jobs.push_back(ModelParseJob { anchor, header });
            if (!skipModelBody(stream)) {
                return std::nullopt;
            }
            continue;
        }

        UnknownEntry unknown;
        unknown.line = stream.peek().sourceLine;
        unknown.token = stream.consume().value;
        document.unknown.push_back(std::move(unknown));
    }

    if (jobs.size() < kParallelModelParseThreshold) {
        document.nodeModel.reserve(jobs.size());
        for (const ModelParseJob& job : jobs) {
            ParsedNodeModel model;
            TokenStream job_stream(parsed.tokens);
            job_stream.rewind(job.segment_index);
            NodeHeader parsed_header;
            std::string subtype;
            std::vector<SourceToken> raw;
            if (!node::io::consumeHeader(job_stream, parsed_header, subtype, raw) ||
                !isKeyword(subtype, node_model::kSubtype)) {
                return std::nullopt;
            }
            model.header = job.header;
            if (!node_model::parseBody(job_stream, raw, job.header, model)) {
                return std::nullopt;
            }
            document.nodeModel.push_back(std::move(model));
        }
        return document;
    }

    document.nodeModel.resize(jobs.size());
    const unsigned worker_count =
        std::min(static_cast<unsigned>(jobs.size()), eu07::nmt::workerThreadCount());
    std::atomic<std::size_t> next_job { 0 };
    std::atomic<bool> failed { false };

    auto worker = [&]() {
        while (true) {
            if (failed.load(std::memory_order_relaxed)) {
                return;
            }
            const std::size_t index = next_job.fetch_add(1, std::memory_order_relaxed);
            if (index >= jobs.size()) {
                return;
            }
            ParsedNodeModel model;
            TokenStream job_stream(parsed.tokens);
            job_stream.rewind(jobs[index].segment_index);
            NodeHeader parsed_header;
            std::string subtype;
            std::vector<SourceToken> raw;
            if (!node::io::consumeHeader(job_stream, parsed_header, subtype, raw) ||
                !isKeyword(subtype, node_model::kSubtype)) {
                failed.store(true, std::memory_order_relaxed);
                return;
            }
            model.header = jobs[index].header;
            if (!node_model::parseBody(job_stream, raw, jobs[index].header, model)) {
                failed.store(true, std::memory_order_relaxed);
                return;
            }
            document.nodeModel[index] = std::move(model);
        }
    };

    std::vector<std::thread> threads;
    const unsigned launch = std::max(1u, worker_count);
    threads.reserve(launch);
    for (unsigned i = 0; i < launch; ++i) {
        threads.emplace_back(worker);
    }
    for (std::thread& thread : threads) {
        thread.join();
    }

    if (failed.load(std::memory_order_relaxed)) {
        return std::nullopt;
    }
    return document;
}

[[nodiscard]] inline std::optional<SceneDocument> tryProcessFlatSceneFile(
    eu07::RawFile raw,
    const std::filesystem::path& baseDirectory) {
    const LogicalPass logical = toLogicalLines(raw.lines);
    return tryProcessFlatLogical(logical, baseDirectory);
}

inline constexpr std::size_t kStreamFlatFileThresholdBytes = 4u * 1024u * 1024u;

[[nodiscard]] inline std::size_t streamSourceFileSizeBytes(const std::filesystem::path& path) {
    std::error_code ec;
    const auto size = std::filesystem::file_size(path, ec);
    return ec ? 0u : static_cast<std::size_t>(size);
}

[[nodiscard]] inline bool shouldStreamFlatSourceFile(
    const std::filesystem::path& path,
    const std::size_t threshold_bytes = kStreamFlatFileThresholdBytes) {
    if (threshold_bytes == 0) {
        return false;
    }
    return streamSourceFileSizeBytes(path) > threshold_bytes;
}

// Linia po linii, z ta sama logika komentarzy co toLogicalLines — bez readRawFile.
template <typename Fn>
void forEachStreamingLogicalSegment(const std::filesystem::path& path, Fn&& on_segment) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Nie mozna otworzyc: " + path.string());
    }

    std::vector<char> read_buffer(1024u * 1024u);
    in.rdbuf()->pubsetbuf(read_buffer.data(), read_buffer.size());

    bool in_block = false;
    std::string line;
    line.reserve(512);
    std::size_t source_line = 0;
    std::string spill;

    const auto emit = [&](std::string_view piece, const std::size_t line_no) -> bool {
        if (piece.empty()) {
            return true;
        }
        skipFieldSeparators(piece);
        if (eu07::detail::isStarter(piece)) {
            return true;
        }
        if (eu07::detail::isLineComment(piece)) {
            return true;
        }
        const std::string_view code = eu07::detail::stripInlineComment(piece);
        if (code.empty()) {
            return true;
        }
        LogicalSegment segment;
        segment.sourceLine = line_no;
        if (code.data() >= piece.data() && code.data() + code.size() <= piece.data() + piece.size()) {
            segment.text = code;
        } else {
            spill.assign(code.begin(), code.end());
            segment.text = spill;
        }
        return on_segment(segment);
    };

    while (std::getline(in, line)) {
        ++source_line;
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        {
            std::string_view whole = line;
            skipFieldSeparators(whole);
            if (parseStarter(whole, source_line - 1)) {
                continue;
            }
        }

        if (!in_block) {
            std::size_t at = 0;
            std::string_view line_view = line;
            while (at < line_view.size()) {
                const std::size_t slash = line_view.find('/', at);
                if (slash == std::string_view::npos) {
                    if (!emit(line_view.substr(at), source_line)) {
                        return;
                    }
                    break;
                }
                if (slash + 1 < line_view.size() && line_view[slash + 1] == '/') {
                    if (slash > at) {
                        if (!emit(line_view.substr(at, slash - at), source_line)) {
                            return;
                        }
                    }
                    break;
                }
                if (slash + 1 < line_view.size() && line_view[slash + 1] == '*') {
                    if (slash > at) {
                        if (!emit(line_view.substr(at, slash - at), source_line)) {
                            return;
                        }
                    }
                    in_block = true;
                    line_view = line_view.substr(slash + 2);
                    at = 0;
                    continue;
                }
                if (!emit(line_view.substr(at), source_line)) {
                    return;
                }
                break;
            }
            continue;
        }

        while (in_block && !line.empty()) {
            const std::size_t close = line.find("*/");
            if (close == std::string_view::npos) {
                line.clear();
                break;
            }
            line.erase(0, close + 2);
            in_block = false;
        }
        if (!in_block && !line.empty()) {
            if (!emit(line, source_line)) {
                return;
            }
        }
    }
}

[[nodiscard]] inline FlatFileKind scanFlatFileKindStreaming(const std::filesystem::path& path) {
    FlatFileKind kind = FlatFileKind::None;
    bool ok = true;
    forEachStreamingLogicalSegment(path, [&](const LogicalSegment& segment) {
        std::string_view text = segment.text;
        skipFieldSeparators(text);
        if (text.empty()) {
            return true;
        }
        if (segmentStartsWithKeyword(text, "node")) {
            if (text.find("endmodel") == std::string_view::npos) {
                ok = false;
                return false;
            }
            if (kind == FlatFileKind::Includes) {
                ok = false;
                return false;
            }
            kind = FlatFileKind::Models;
            return true;
        }
        if (segmentStartsWithKeyword(text, "include")) {
            if (text.find("end") == std::string_view::npos) {
                ok = false;
                return false;
            }
            if (kind == FlatFileKind::Models) {
                ok = false;
                return false;
            }
            kind = FlatFileKind::Includes;
            return true;
        }
        ok = false;
        return false;
    });
    return ok ? kind : FlatFileKind::None;
}

[[nodiscard]] inline std::optional<SceneDocument> buildFlatIncludesDocumentStreaming(
    const std::filesystem::path& path) {
    SceneDocument document;
    struct OwnedIncludeLine {
        std::size_t source_line = 0;
        std::string text;
    };
    std::vector<OwnedIncludeLine> batch;
    batch.reserve(8192);
    bool ok = true;

    const auto flush_batch = [&]() -> bool {
        if (batch.empty()) {
            return true;
        }
        const std::size_t base = document.include.size();
        document.include.resize(base + batch.size());

        const auto parse_one = [&](const std::size_t index) -> bool {
            LogicalSegment segment;
            segment.sourceLine = batch[index].source_line;
            segment.text = batch[index].text;
            ParsedInclude entry;
            if (!parseIncludeSegment(segment, entry)) {
                return false;
            }
            document.include[base + index] = std::move(entry);
            return true;
        };

        if (batch.size() >= kParallelFlatLineThreshold) {
            std::atomic<std::size_t> next_job { 0 };
            std::atomic<bool> failed { false };
            const unsigned worker_count = std::min(
                static_cast<unsigned>(batch.size()), eu07::nmt::workerThreadCount());
            const auto worker = [&]() {
                while (!failed.load(std::memory_order_relaxed)) {
                    const std::size_t index =
                        next_job.fetch_add(1, std::memory_order_relaxed);
                    if (index >= batch.size()) {
                        return;
                    }
                    if (!parse_one(index)) {
                        failed.store(true, std::memory_order_relaxed);
                        return;
                    }
                }
            };
            std::vector<std::thread> threads;
            threads.reserve(worker_count);
            for (unsigned i = 0; i < worker_count; ++i) {
                threads.emplace_back(worker);
            }
            for (std::thread& thread : threads) {
                thread.join();
            }
            if (failed.load(std::memory_order_relaxed)) {
                return false;
            }
        } else {
            for (std::size_t index = 0; index < batch.size(); ++index) {
                if (!parse_one(index)) {
                    return false;
                }
            }
        }

        batch.clear();
        return true;
    };

    forEachStreamingLogicalSegment(path, [&](const LogicalSegment& segment) {
        std::string_view text = segment.text;
        skipFieldSeparators(text);
        if (text.empty() || !segmentStartsWithKeyword(text, "include")) {
            return true;
        }
        batch.push_back(OwnedIncludeLine { segment.sourceLine, std::string(segment.text) });
        if (batch.size() >= 8192) {
            if (!flush_batch()) {
                ok = false;
                return false;
            }
        }
        return true;
    });

    if (ok) {
        ok = flush_batch();
    }
    return ok ? std::optional<SceneDocument>(std::move(document)) : std::nullopt;
}

template <typename Fn>
void streamFlatFileModels(const std::filesystem::path& path, Fn&& on_model) {
    forEachStreamingLogicalSegment(path, [&](const LogicalSegment& segment) {
        std::string_view text = segment.text;
        skipFieldSeparators(text);
        if (text.empty() || !segmentStartsWithKeyword(text, "node")) {
            return true;
        }
        if (text.find("endmodel") == std::string_view::npos) {
            throw std::runtime_error(
                "Plaski plik modeli: oczekiwano endmodel w linii " +
                std::to_string(segment.sourceLine));
        }
        ParsedNodeModel parsed;
        if (!parseModelSegment(segment, parsed)) {
            throw std::runtime_error(
                "Plaski plik modeli: blad parse linii " + std::to_string(segment.sourceLine));
        }
        on_model(parsed);
        return true;
    });
}

} // namespace eu07::scene::detail
