#pragma once

// Duze pliki terenu (same node triangles, np. nmt100_warmaz_ter.scm):
// linia po linii, parse per-wezel, bez readRawFile / tokenizacji calego pliku.

#include <eu07/parser.hpp>
#include <eu07/scene/bake/mesh.hpp>
#include <eu07/scene/bake/module.hpp>
#include <eu07/scene/bake/pack_model_spool.hpp>
#include <eu07/scene/context.hpp>
#include <eu07/scene/cursor.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/common.hpp>
#include <eu07/scene/node/triangles.hpp>
#include <eu07/scene/parallel_models.hpp>

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace eu07::scene::bake::detail {

enum class TriangleTerrainLineKind {
    Skip,
    Header,
    Vertex,
    EndTri,
    Invalid,
};

[[nodiscard]] inline TriangleTerrainLineKind classifyTriangleTerrainLine(std::string_view text) {
    skipFieldSeparators(text);
    if (text.empty()) {
        return TriangleTerrainLineKind::Skip;
    }
    if (eu07::detail::isLineComment(text)) {
        return TriangleTerrainLineKind::Skip;
    }

    if (eu07::scene::detail::segmentStartsWithKeyword(text, "node")) {
        if (text.find("triangles") == std::string_view::npos) {
            return TriangleTerrainLineKind::Invalid;
        }
        return TriangleTerrainLineKind::Header;
    }

    if (isKeyword(text, "endtri") || isKeyword(text, "endtriangles")) {
        return TriangleTerrainLineKind::EndTri;
    }

    if (!text.empty() &&
        (text.front() == '-' || text.front() == '+' ||
         (text.front() >= '0' && text.front() <= '9'))) {
        return TriangleTerrainLineKind::Vertex;
    }

    return TriangleTerrainLineKind::Invalid;
}

[[nodiscard]] inline bool parseTriangleTerrainBlock(
    const std::vector<std::string>& lines,
    const std::size_t header_line,
    ParsedNodeTriangles& out) {
    if (lines.size() < 2) {
        return false;
    }

    std::vector<SourceToken> header_tokens;
    tokenizeInto(lines.front(), header_tokens, header_line);

    TokenStream stream(header_tokens);
    NodeHeader header;
    std::string subtype;
    std::vector<SourceToken> raw;
    if (!node::io::consumeHeader(stream, header, subtype, raw) ||
        !isKeyword(subtype, node_triangles::kSubtype) ||
        !node::io::takeString(stream, raw, out.texture)) {
        return false;
    }

    out.header = header;
    out.raw.clear();
    out.vertices.clear();
    out.vertices.reserve(lines.size() > 2 ? lines.size() - 2 : 0);

    for (std::size_t i = 1; i + 1 < lines.size(); ++i) {
        std::vector<SourceToken> vertex_tokens;
        tokenizeInto(lines[i], vertex_tokens, header_line + i);

        TokenStream vertex_stream(vertex_tokens);
        MeshVertex vertex;
        if (!node_triangles::takeVertex(vertex_stream, raw, vertex, !vertex_stream.empty())) {
            return false;
        }
        out.vertices.push_back(vertex);
    }

    return !out.vertices.empty();
}

struct TriangleTerrainBlock {
    std::size_t header_line = 0;
    std::vector<std::string> lines;
};

constexpr std::size_t kTerrainParallelBatchSize = 8192;
constexpr std::size_t kTerrainParallelMinBlocks = 8;
constexpr std::size_t kTerrainPipelineMaxQueuedBatches = 2;

[[nodiscard]] inline unsigned resolveTerrainThreadCount(const unsigned max_threads) {
    const unsigned hw =
        std::thread::hardware_concurrency() == 0 ? 4u : std::thread::hardware_concurrency();
    if (max_threads == 0) {
        return std::max(1u, hw);
    }
    return std::max(1u, max_threads);
}

[[nodiscard]] inline bool bakeTriangleTerrainBatch(
    const std::vector<TriangleTerrainBlock>& batch,
    const unsigned max_threads,
    std::vector<runtime::RuntimeShapeNode>& baked_out) {
    baked_out.resize(batch.size());
    if (batch.empty()) {
        return true;
    }
    if (batch.size() < kTerrainParallelMinBlocks ||
        resolveTerrainThreadCount(max_threads) <= 1) {
        for (std::size_t i = 0; i < batch.size(); ++i) {
            ParsedNodeTriangles parsed;
            if (!parseTriangleTerrainBlock(batch[i].lines, batch[i].header_line, parsed)) {
                return false;
            }
            baked_out[i] = bakeTriangles(parsed);
        }
        return true;
    }

    const unsigned worker_count = resolveTerrainThreadCount(max_threads);
    std::atomic<std::size_t> next_index { 0 };
    std::atomic<bool> ok { true };

    const auto worker = [&]() {
        while (ok.load(std::memory_order_relaxed)) {
            const std::size_t index = next_index.fetch_add(1, std::memory_order_relaxed);
            if (index >= batch.size()) {
                return;
            }
            try {
                ParsedNodeTriangles parsed;
                if (!parseTriangleTerrainBlock(
                        batch[index].lines, batch[index].header_line, parsed)) {
                    ok.store(false, std::memory_order_relaxed);
                    return;
                }
                baked_out[index] = bakeTriangles(parsed);
            } catch (...) {
                ok.store(false, std::memory_order_relaxed);
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
    return ok.load(std::memory_order_relaxed);
}

[[nodiscard]] inline bool scanTriangleTerrainOnlyStreaming(const std::filesystem::path& path) {
    bool saw_header = false;
    bool in_block = false;
    bool ok = true;

    eu07::scene::detail::forEachStreamingLogicalSegment(
        path, [&](const LogicalSegment& segment) {
            const TriangleTerrainLineKind kind = classifyTriangleTerrainLine(segment.text);
            switch (kind) {
            case TriangleTerrainLineKind::Skip:
                return true;
            case TriangleTerrainLineKind::Header:
                saw_header = true;
                in_block = true;
                return true;
            case TriangleTerrainLineKind::Vertex:
                if (!in_block) {
                    ok = false;
                    return false;
                }
                return true;
            case TriangleTerrainLineKind::EndTri:
                if (!in_block) {
                    ok = false;
                    return false;
                }
                in_block = false;
                return true;
            case TriangleTerrainLineKind::Invalid:
            default:
                ok = false;
                return false;
            }
        });

    return ok && saw_header && !in_block;
}

[[nodiscard]] inline bool scanTriangleTerrainBlocksStreaming(
    const std::filesystem::path& path,
    const std::function<bool(std::vector<TriangleTerrainBlock>&&)>& on_batch) {
    std::vector<TriangleTerrainBlock> batch;
    batch.reserve(kTerrainParallelBatchSize);

    TriangleTerrainBlock current_block;
    bool in_block = false;
    bool saw_header = false;

    const auto finish_block = [&]() -> bool {
        if (current_block.lines.empty()) {
            return true;
        }
        batch.push_back(std::move(current_block));
        current_block = {};
        in_block = false;
        if (batch.size() >= kTerrainParallelBatchSize) {
            if (!on_batch(std::move(batch))) {
                return false;
            }
            batch = {};
            batch.reserve(kTerrainParallelBatchSize);
        }
        return true;
    };

    bool ok = true;
    eu07::scene::detail::forEachStreamingLogicalSegment(
        path, [&](const LogicalSegment& segment) {
            const TriangleTerrainLineKind kind = classifyTriangleTerrainLine(segment.text);
            switch (kind) {
            case TriangleTerrainLineKind::Skip:
                return true;
            case TriangleTerrainLineKind::Header:
                saw_header = true;
                if (!finish_block()) {
                    ok = false;
                    return false;
                }
                current_block.header_line = segment.sourceLine;
                current_block.lines.emplace_back(segment.text);
                in_block = true;
                return true;
            case TriangleTerrainLineKind::Vertex:
                if (!in_block) {
                    ok = false;
                    return false;
                }
                current_block.lines.emplace_back(segment.text);
                return true;
            case TriangleTerrainLineKind::EndTri:
                if (!in_block) {
                    ok = false;
                    return false;
                }
                current_block.lines.emplace_back(segment.text);
                if (!finish_block()) {
                    ok = false;
                    return false;
                }
                return true;
            case TriangleTerrainLineKind::Invalid:
            default:
                ok = false;
                return false;
            }
        });

    if (ok && in_block) {
        ok = finish_block();
    }
    if (ok && !batch.empty()) {
        ok = on_batch(std::move(batch));
    }

    return ok && saw_header;
}

// Zwraca false gdy plik nie jest czystym terenem triangles (wtedy uzyj documentFor).
// Gdy shape_spool != nullptr, ksztalty trafiaja na dysk (module.scene.shapes puste).
[[nodiscard]] inline bool streamBakeTriangleTerrain(
    const std::filesystem::path& path,
    RuntimeModule& module,
    ShapeSpoolFile* shape_spool = nullptr,
    const unsigned max_threads = 0) {
    module.scene.shapes.clear();

    const unsigned bake_threads = resolveTerrainThreadCount(max_threads);
    bool parsed_any = false;

    const auto emit_baked = [&](std::vector<runtime::RuntimeShapeNode>& baked) -> bool {
        if (baked.empty()) {
            return true;
        }
        parsed_any = true;
        if (shape_spool != nullptr) {
            shape_spool->append_batch(std::move(baked));
        } else {
            module.scene.shapes.insert(
                module.scene.shapes.end(),
                std::make_move_iterator(baked.begin()),
                std::make_move_iterator(baked.end()));
            baked.clear();
        }
        return true;
    };

    const auto bake_and_emit = [&](std::vector<TriangleTerrainBlock>&& blocks) -> bool {
        if (blocks.empty()) {
            return true;
        }
        std::vector<runtime::RuntimeShapeNode> baked;
        if (!bakeTriangleTerrainBatch(blocks, max_threads, baked)) {
            return false;
        }
        return emit_baked(baked);
    };

    if (bake_threads <= 1) {
        const bool ok = scanTriangleTerrainBlocksStreaming(path, bake_and_emit);
        if (!ok || !parsed_any) {
            module.scene.shapes.clear();
            return false;
        }
        return true;
    }

    std::mutex queue_mutex;
    std::condition_variable queue_cv;
    std::deque<std::vector<TriangleTerrainBlock>> ready_batches;
    std::atomic<bool> scan_ok { true };
    std::atomic<bool> scan_done { false };
    std::atomic<bool> bake_failed { false };

    std::thread scanner([&]() {
        try {
            const bool ok = scanTriangleTerrainBlocksStreaming(
                path, [&](std::vector<TriangleTerrainBlock>&& blocks) -> bool {
                    if (bake_failed.load(std::memory_order_relaxed)) {
                        return false;
                    }
                    std::unique_lock lock(queue_mutex);
                    queue_cv.wait(lock, [&]() {
                        return bake_failed.load(std::memory_order_relaxed) ||
                               ready_batches.size() < kTerrainPipelineMaxQueuedBatches;
                    });
                    if (bake_failed.load(std::memory_order_relaxed)) {
                        return false;
                    }
                    ready_batches.push_back(std::move(blocks));
                    lock.unlock();
                    queue_cv.notify_one();
                    return true;
                });
            scan_ok.store(ok, std::memory_order_relaxed);
        } catch (...) {
            scan_ok.store(false, std::memory_order_relaxed);
            bake_failed.store(true, std::memory_order_relaxed);
            queue_cv.notify_all();
        }
        scan_done.store(true, std::memory_order_release);
        queue_cv.notify_all();
    });

    while (true) {
        std::vector<TriangleTerrainBlock> batch;
        {
            std::unique_lock lock(queue_mutex);
            queue_cv.wait(lock, [&]() {
                return bake_failed.load(std::memory_order_relaxed) ||
                       !ready_batches.empty() ||
                       scan_done.load(std::memory_order_acquire);
            });
            if (bake_failed.load(std::memory_order_relaxed)) {
                break;
            }
            if (ready_batches.empty() && scan_done.load(std::memory_order_acquire)) {
                break;
            }
            if (ready_batches.empty()) {
                continue;
            }
            batch = std::move(ready_batches.front());
            ready_batches.pop_front();
        }
        queue_cv.notify_one();

        std::vector<runtime::RuntimeShapeNode> baked;
        if (!bakeTriangleTerrainBatch(batch, max_threads, baked)) {
            bake_failed.store(true, std::memory_order_relaxed);
            queue_cv.notify_all();
            break;
        }
        if (!emit_baked(baked)) {
            bake_failed.store(true, std::memory_order_relaxed);
            queue_cv.notify_all();
            break;
        }
    }

    scanner.join();

    if (bake_failed.load(std::memory_order_relaxed) ||
        !scan_ok.load(std::memory_order_relaxed) || !parsed_any) {
        module.scene.shapes.clear();
        return false;
    }
    return true;
}

} // namespace eu07::scene::bake::detail
