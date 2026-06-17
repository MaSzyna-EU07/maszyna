#pragma once

// Compose MODL z calego drzewa INCL → world-space do chunku PACK (v7).
// Cache parsowania per plik + kolejka zadan z pulą watkow.

#include <eu07/parser.hpp>
#include <eu07/scene/binary/runtime_codec.hpp>
#include <eu07/scene/bake/model.hpp>
#include <eu07/scene/include_placement.hpp>
#include <eu07/scene/include_resolve.hpp>
#include <eu07/scene/dispatch_table.hpp>
#include <eu07/scene/node/model.hpp>
#include <eu07/scene/processor.hpp>
#include <eu07/scene/parallel_models.hpp>
#include <eu07/scene/runtime/basic_node.hpp>
#include <eu07/scene/runtime/nodes.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <functional>
#include <iostream>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace eu07::scene::bake {

using PackComposeProgressCallback =
    std::function<void(const std::filesystem::path& File, std::size_t FilesVisited, std::size_t ModelsTotal)>;

struct PackComposeProgress {
    PackComposeProgressCallback on_progress;
    std::size_t files_visited = 0;
    std::size_t models_total = 0;
    bool reported_once = false;
    std::chrono::steady_clock::time_point last_report {};
};

struct PackFileCacheEntry {
    std::vector<ParsedInclude> includes;
    IncludePlacement placement;
    std::vector<runtime::RuntimeModelInstance> base_models;
};

struct PackComposeSeed {
    std::filesystem::path path;
    PackFileCacheEntry entry;
};

// Liczniki diagnostyczne compose PACK (watki aktualizuja atomowo).
struct PackComposeStats {
    std::atomic<std::size_t> files_visited { 0 };
    std::atomic<std::size_t> models_emitted { 0 };
    std::atomic<std::size_t> cache_hits { 0 };
    std::atomic<std::size_t> cache_misses { 0 };
    std::atomic<std::size_t> cache_seeded { 0 };
    std::atomic<std::size_t> cycles_skipped { 0 };
    std::atomic<std::size_t> missing_skipped { 0 };
    std::atomic<std::size_t> includes_submitted { 0 };
    std::atomic<std::size_t> sink_batches { 0 };
    std::atomic<std::size_t> instantiate_fast { 0 };
    std::atomic<std::size_t> instantiate_full { 0 };
    std::atomic<std::size_t> queue_peak { 0 };
    std::atomic<std::size_t> unique_files_cached { 0 };
    std::atomic<std::uint64_t> parse_us { 0 };
    std::atomic<std::uint64_t> tokenize_us { 0 };   // parseFile (read + tokenize)
    std::atomic<std::uint64_t> process_us { 0 };    // processScene (build SceneDocument)
    std::atomic<std::uint64_t> makeentry_us { 0 };  // makePackFileCacheEntry (bakeModel loop)
    std::atomic<std::uint64_t> instantiate_us { 0 };
    std::atomic<std::uint64_t> sink_us { 0 };       // PackSectionAccumulator::add (sharding)
    std::atomic<std::uint64_t> finalize_us { 0 };
    std::atomic<std::uint64_t> compose_us { 0 };
    std::atomic<unsigned> threads { 0 };
    std::atomic<std::size_t> sections { 0 };
    std::atomic<std::size_t> chain_fold_steps { 0 };
    std::atomic<std::size_t> chain_fold_hops { 0 };
    std::atomic<std::size_t> inc_includes_inlined { 0 };
    // --- live deadlock diagnostics (read by external watchdog) ---
    std::atomic<std::size_t> diag_pending { 0 };       // outstanding queued/running tasks
    std::atomic<unsigned> diag_gate_active { 0 };      // gate slots in use
    std::atomic<unsigned> diag_gate_max { 0 };         // gate slot limit
    std::atomic<unsigned> diag_gate_waiting { 0 };     // workers blocked in gate acquire
    std::atomic<unsigned> diag_workers_alive { 0 };    // worker threads currently running
    std::atomic<unsigned> diag_workers_idle { 0 };     // workers blocked on queue_cv
    std::atomic<unsigned> diag_workers_parse { 0 };    // workers inside buildCacheEntry/parse
    std::atomic<unsigned> diag_workers_inst { 0 };     // workers inside instantiate/process
    std::atomic<unsigned> diag_main_wait { 0 };        // 1 while main blocks on done_cv
};

struct PackComposeStatsSnapshot {
    std::size_t files_visited = 0;
    std::size_t models_emitted = 0;
    std::size_t cache_hits = 0;
    std::size_t cache_misses = 0;
    std::size_t cache_seeded = 0;
    std::size_t cycles_skipped = 0;
    std::size_t missing_skipped = 0;
    std::size_t includes_submitted = 0;
    std::size_t sink_batches = 0;
    std::size_t instantiate_fast = 0;
    std::size_t instantiate_full = 0;
    std::size_t queue_peak = 0;
    std::size_t unique_files_cached = 0;
    std::uint64_t parse_us = 0;
    std::uint64_t tokenize_us = 0;
    std::uint64_t process_us = 0;
    std::uint64_t makeentry_us = 0;
    std::uint64_t instantiate_us = 0;
    std::uint64_t sink_us = 0;
    std::uint64_t finalize_us = 0;
    std::uint64_t compose_us = 0;
    unsigned threads = 0;
    std::size_t sections = 0;
    std::size_t chain_fold_steps = 0;
    std::size_t chain_fold_hops = 0;
    std::size_t inc_includes_inlined = 0;
};

[[nodiscard]] inline PackComposeStatsSnapshot snapshotPackComposeStats(
    const PackComposeStats& stats) {
    PackComposeStatsSnapshot out;
    out.files_visited = stats.files_visited.load(std::memory_order_relaxed);
    out.models_emitted = stats.models_emitted.load(std::memory_order_relaxed);
    out.cache_hits = stats.cache_hits.load(std::memory_order_relaxed);
    out.cache_misses = stats.cache_misses.load(std::memory_order_relaxed);
    out.cache_seeded = stats.cache_seeded.load(std::memory_order_relaxed);
    out.cycles_skipped = stats.cycles_skipped.load(std::memory_order_relaxed);
    out.missing_skipped = stats.missing_skipped.load(std::memory_order_relaxed);
    out.includes_submitted = stats.includes_submitted.load(std::memory_order_relaxed);
    out.sink_batches = stats.sink_batches.load(std::memory_order_relaxed);
    out.instantiate_fast = stats.instantiate_fast.load(std::memory_order_relaxed);
    out.instantiate_full = stats.instantiate_full.load(std::memory_order_relaxed);
    out.queue_peak = stats.queue_peak.load(std::memory_order_relaxed);
    out.unique_files_cached = stats.unique_files_cached.load(std::memory_order_relaxed);
    out.parse_us = stats.parse_us.load(std::memory_order_relaxed);
    out.tokenize_us = stats.tokenize_us.load(std::memory_order_relaxed);
    out.process_us = stats.process_us.load(std::memory_order_relaxed);
    out.makeentry_us = stats.makeentry_us.load(std::memory_order_relaxed);
    out.instantiate_us = stats.instantiate_us.load(std::memory_order_relaxed);
    out.sink_us = stats.sink_us.load(std::memory_order_relaxed);
    out.finalize_us = stats.finalize_us.load(std::memory_order_relaxed);
    out.compose_us = stats.compose_us.load(std::memory_order_relaxed);
    out.threads = stats.threads.load(std::memory_order_relaxed);
    out.sections = stats.sections.load(std::memory_order_relaxed);
    out.chain_fold_steps = stats.chain_fold_steps.load(std::memory_order_relaxed);
    out.chain_fold_hops = stats.chain_fold_hops.load(std::memory_order_relaxed);
    out.inc_includes_inlined = stats.inc_includes_inlined.load(std::memory_order_relaxed);
    return out;
}

inline void printPackComposeStats(const PackComposeStatsSnapshot& stats, std::ostream& out) {
    const std::size_t files = stats.files_visited;
    const std::size_t models = stats.models_emitted;
    const std::size_t hits = stats.cache_hits;
    const std::size_t misses = stats.cache_misses;
    const std::size_t visits = hits + misses;
    const double hit_pct = visits == 0 ? 0.0 : (100.0 * static_cast<double>(hits) / static_cast<double>(visits));
    const std::uint64_t compose_ms = stats.compose_us / 1000;
    const std::uint64_t parse_ms = stats.parse_us / 1000;
    const std::uint64_t inst_ms = stats.instantiate_us / 1000;
    const std::uint64_t fin_ms = stats.finalize_us / 1000;

    out << "[EU7]   PACK liczniki:\n"
        << "    pliki=" << files << " modele=" << models << " sekcje=" << stats.sections << '\n'
        << "    cache: trafienia=" << hits << " miss=" << misses << " (" << hit_pct
        << "%) unikalne=" << stats.unique_files_cached << " seed=" << stats.cache_seeded << '\n'
        << "    includy=" << stats.includes_submitted << " cykle=" << stats.cycles_skipped
        << " brak_pliku=" << stats.missing_skipped << '\n'
        << "    inst: fast=" << stats.instantiate_fast << " full=" << stats.instantiate_full
        << " sink_batchy=" << stats.sink_batches << '\n'
        << "    kolejka_max=" << stats.queue_peak << " watki=" << stats.threads << '\n'
        << "    lancuch: skladania=" << stats.chain_fold_hops << " kroki=" << stats.chain_fold_steps
        << " inc_inline=" << stats.inc_includes_inlined << '\n'
        << "    czas_ms: compose=" << compose_ms << " parse=" << parse_ms << " inst=" << inst_ms
        << " finalize=" << fin_ms << '\n';

    const unsigned threads = stats.threads;
    const std::size_t queue_peak = stats.queue_peak;
    if (compose_ms > 0) {
        if (parse_ms * 100 / compose_ms > 60) {
            out << "    (!) parse dominuje - dysk lub processScene na miss\n";
        }
        if (inst_ms * 100 / compose_ms > 50) {
            out << "    (!) instantiate dominuje - transformy/kopie modeli\n";
        }
        if (fin_ms * 100 / compose_ms > 20) {
            out << "    (!) finalize wolny - merge sekcji 1km\n";
        }
    }
    if (threads > 1 && queue_peak < threads / 2) {
        if (stats.chain_fold_steps > 100) {
            out << "    (!) plytka kolejka - lancuch liniowy (malo rownolegle), skladanie "
                   "zredukowalo narzut\n";
        } else {
            out << "    (!) plytka kolejka - drzewo include malo rownolegle\n";
        }
    }
    if (visits > 1000 && hit_pct < 40.0) {
        out << "    (!) niski cache hit - duzo unikalnych plikow .scm\n";
    }
    if (stats.instantiate_full > stats.instantiate_fast * 4) {
        out << "    (!) malo fast_inst - wiekszosc includow ma transform/param\n";
    }
    out << std::flush;
}

inline void printPackComposeStats(const PackComposeStats& stats, std::ostream& out) {
    printPackComposeStats(snapshotPackComposeStats(stats), out);
}

namespace detail {
class BakeParseSessionCache;
}

struct PackComposeOptions {
    PackComposeProgress* progress = nullptr;
    PackComposeStats* stats = nullptr;
    unsigned max_threads = 0;
    // Root juz sparsowany w bake_tree — bez drugiego parseFile.
    const PackComposeSeed* root_seed = nullptr;
    // Wspolny cache z bake_tree (eliminuje drugi parse tego samego pliku).
    detail::BakeParseSessionCache* session_cache = nullptr;
    // Wynik: sekcje 1 km (serializacja PACK w writeRuntimeModule).
    std::vector<binary::codec::ModelSectionBatch>* pack_batches = nullptr;
    // Strumieniuj modele do pack_batches (bez duzego akumulatora na koncu).
    bool low_memory = false;
    // low_memory: gdy w RAM jest wiecej niz threshold, przenies sekcje do on_flush.
    std::size_t pack_flush_threshold = 0;
    // low_memory: po kazdym pliku w compose — spłucz caly PACK z RAM (ignoruje threshold).
    bool pack_flush_per_file = false;
    std::function<void(std::vector<runtime::RuntimeModelInstance>&&)> on_pack_models_flush;
};

inline constexpr std::size_t kDefaultPackFlushThreshold = 65536u;

namespace detail {

// Po tylu hopach skladania wysylamy kontynuacje do kolejki (rownolegle segmenty lancucha).
constexpr std::size_t kMaxChainFoldPerTask = 96;

[[nodiscard]] inline unsigned resolveComposeThreadCount(const unsigned maxThreads) {
    const unsigned hw =
        std::thread::hardware_concurrency() == 0 ? 4u : std::thread::hardware_concurrency();
    if (maxThreads == 0) {
        return std::max(1u, hw);
    }
    return std::max(1u, maxThreads);
}

class ComposeConcurrencyGate {
public:
    explicit ComposeConcurrencyGate(const unsigned max_slots)
        : max_slots_(std::max(1u, max_slots)) {}

    void acquire() {
        std::unique_lock lock(mutex_);
        waiting_.fetch_add(1, std::memory_order_relaxed);
        cv_.wait(lock, [this]() { return active_slots_ < max_slots_; });
        waiting_.fetch_sub(1, std::memory_order_relaxed);
        ++active_slots_;
        active_snapshot_.store(active_slots_, std::memory_order_relaxed);
    }

    void release() {
        {
            std::lock_guard lock(mutex_);
            --active_slots_;
            active_snapshot_.store(active_slots_, std::memory_order_relaxed);
        }
        cv_.notify_one();
    }

    struct Scope {
        ComposeConcurrencyGate& gate;
        ~Scope() {
            gate.release();
        }
    };

    [[nodiscard]] Scope scoped() {
        acquire();
        return Scope { *this };
    }

    [[nodiscard]] unsigned maxSlots() const noexcept {
        return max_slots_;
    }
    [[nodiscard]] unsigned activeSlots() const noexcept {
        return active_snapshot_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] unsigned waiting() const noexcept {
        return waiting_.load(std::memory_order_relaxed);
    }

private:
    unsigned max_slots_;
    unsigned active_slots_ = 0;
    std::atomic<unsigned> active_snapshot_ { 0 };
    std::atomic<unsigned> waiting_ { 0 };
    std::mutex mutex_;
    std::condition_variable cv_;
};

[[nodiscard]] inline bool transformIsEmpty(const runtime::TransformContext& transform) noexcept {
    return (
        transform.originStack.empty() && transform.scaleStack.empty() &&
        transform.rotation.x == 0.0 && transform.rotation.y == 0.0 &&
        transform.rotation.z == 0.0 && transform.groupStackDepth == 0);
}

[[nodiscard]] inline runtime::Vec3 transformPoint(
    runtime::Vec3 location,
    const runtime::TransformContext& transform) {
    if (transform.rotation.y != 0.0) {
        const double rad = transform.rotation.y * (3.14159265358979323846 / 180.0);
        const double c = std::cos(rad);
        const double s = std::sin(rad);
        const double x = location.x * c + location.z * s;
        const double z = -location.x * s + location.z * c;
        location.x = x;
        location.z = z;
    }

    if (!transform.scaleStack.empty()) {
        const runtime::Vec3& scale = transform.scaleStack.back();
        if (scale.x != 0.0) {
            location.x *= scale.x;
        }
        if (scale.y != 0.0) {
            location.y *= scale.y;
        }
        if (scale.z != 0.0) {
            location.z *= scale.z;
        }
    }

    if (!transform.originStack.empty()) {
        const runtime::Vec3& origin = transform.originStack.back();
        location.x += origin.x;
        location.y += origin.y;
        location.z += origin.z;
    }

    return location;
}

inline void composeNodeTransform(
    runtime::TransformContext& node,
    const runtime::TransformContext& prefix) {
    if (transformIsEmpty(prefix)) {
        return;
    }

    const runtime::Vec3 parentOrigin =
        prefix.originStack.empty() ? runtime::Vec3{} : prefix.originStack.back();
    for (runtime::Vec3& offset : node.originStack) {
        offset.x += parentOrigin.x;
        offset.y += parentOrigin.y;
        offset.z += parentOrigin.z;
    }

    const runtime::Vec3 parentScale =
        prefix.scaleStack.empty() ? runtime::Vec3{1.0, 1.0, 1.0} : prefix.scaleStack.back();
    for (runtime::Vec3& scale : node.scaleStack) {
        scale.x *= parentScale.x;
        scale.y *= parentScale.y;
        scale.z *= parentScale.z;
    }

    node.rotation.x += prefix.rotation.x;
    node.rotation.y += prefix.rotation.y;
    node.rotation.z += prefix.rotation.z;
    node.groupStackDepth += prefix.groupStackDepth;
}

inline void composeModelsWithPrefix(
    std::vector<runtime::RuntimeModelInstance>& models,
    const runtime::TransformContext& prefix) {
    if (transformIsEmpty(prefix)) {
        return;
    }

    for (runtime::RuntimeModelInstance& model : models) {
        model.location = transformPoint(model.location, prefix);
        model.angles.x += prefix.rotation.x;
        model.angles.y += prefix.rotation.y;
        model.angles.z += prefix.rotation.z;
        composeNodeTransform(model.node.transform, prefix);
        model.node.area.center = model.location;
    }
}

inline void bindIncludeName(std::string& text, const std::span<const std::string> parameters) {
    std::string bound = eu07::scene::detail::applyIncludeParameters(text, parameters);
    for (char& ch : bound) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    text = std::move(bound);
}

inline void bindIncludePath(std::string& text, const std::span<const std::string> parameters) {
    text = eu07::scene::detail::applyIncludeParameters(text, parameters);
}

inline void applyIncludeParametersToModels(
    std::vector<runtime::RuntimeModelInstance>& models,
    const std::span<const std::string> parameters) {
    if (parameters.empty()) {
        return;
    }

    for (runtime::RuntimeModelInstance& model : models) {
        bindIncludeName(model.node.name, parameters);
        bindIncludeName(model.node.nodeType, parameters);
        bindIncludePath(model.modelFile, parameters);
        bindIncludePath(model.textureFile, parameters);
    }
}

[[nodiscard]] inline double resolvePlacementParam(
    const std::uint8_t paramIndex,
    const std::span<const std::string> parameters) {
    if (paramIndex == 0) {
        return 0.0;
    }

    const std::size_t index = static_cast<std::size_t>(paramIndex);
    if (index < 1 || index > parameters.size()) {
        throw std::runtime_error("EU7 PACK: brak parametru p" + std::to_string(paramIndex));
    }

    return std::stod(parameters[index - 1]);
}

[[nodiscard]] inline runtime::TransformContext placementTransformFromParameters(
    const IncludePlacement& binding,
    const std::span<const std::string> parameters) {
    runtime::TransformContext placement;
    if (binding.empty()) {
        return placement;
    }

    try {
        const double originX = resolvePlacementParam(binding.origin_x_param, parameters);
        const double originY = resolvePlacementParam(binding.origin_y_param, parameters);
        const double originZ = resolvePlacementParam(binding.origin_z_param, parameters);
        placement.originStack.push_back({originX, originY, originZ});
        placement.rotation.y = resolvePlacementParam(binding.rotation_y_param, parameters);
    } catch (const std::exception&) {
        placement = {};
    }

    return placement;
}

inline void applyIncludePlacementToModels(
    std::vector<runtime::RuntimeModelInstance>& models,
    const IncludePlacement& binding,
    const std::span<const std::string> parameters) {
    const runtime::TransformContext placement = placementTransformFromParameters(binding, parameters);
    composeModelsWithPrefix(models, placement);
}

using PackModelBatchSink = std::function<void(std::vector<runtime::RuntimeModelInstance>&&)>;

inline constexpr std::size_t kParallelBakeModelThreshold = 8192;

[[nodiscard]] inline PackFileCacheEntry makePackFileCacheEntry(
    const SceneDocument& document,
    const unsigned max_threads = 0) {
    PackFileCacheEntry entry;
    entry.includes = document.include;
    if (const std::optional<IncludePlacement> placement = extractIncludePlacement(document)) {
        entry.placement = *placement;
    }
    const std::size_t model_count = document.nodeModel.size();
    entry.base_models.resize(model_count);

    if (model_count < kParallelBakeModelThreshold) {
        for (std::size_t i = 0; i < model_count; ++i) {
            entry.base_models[i] = bakeModel(document.nodeModel[i]);
        }
        return entry;
    }

    const unsigned worker_count = resolveComposeThreadCount(max_threads);
    std::atomic<std::size_t> next_index { 0 };
    auto worker = [&]() {
        while (true) {
            const std::size_t index = next_index.fetch_add(1, std::memory_order_relaxed);
            if (index >= model_count) {
                return;
            }
            entry.base_models[index] = bakeModel(document.nodeModel[index]);
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
    return entry;
}

// Buduje PACK zwalniajac nodeModel w trakcie (mniejszy peak niz pelna kopia + bake).
[[nodiscard]] inline PackFileCacheEntry makePackFileCacheEntryConsuming(SceneDocument& document) {
    PackFileCacheEntry entry;
    entry.includes = std::move(document.include);
    if (const std::optional<IncludePlacement> placement = extractIncludePlacement(document)) {
        entry.placement = *placement;
    }
    const std::size_t model_count = document.nodeModel.size();
    entry.base_models.reserve(model_count);
    while (!document.nodeModel.empty()) {
        const std::size_t index = document.nodeModel.size() - 1;
        entry.base_models.push_back(bakeModel(document.nodeModel[index]));
        document.nodeModel.pop_back();
    }
    std::reverse(entry.base_models.begin(), entry.base_models.end());
    document.nodeModel.shrink_to_fit();
    return entry;
}

// low_memory .inc: parse jednego modelu → bake → base_models (bez wektora ParsedNodeModel).
[[nodiscard]] inline bool streamFlatSegmentsToPackModels(
    const std::vector<LogicalSegment>& segments,
    std::vector<runtime::RuntimeModelInstance>& base_models_out) {
    for (const LogicalSegment& segment : segments) {
        std::string_view text = segment.text;
        eu07::skipFieldSeparators(text);
        if (text.empty()) {
            continue;
        }
        ParsedNodeModel parsed;
        if (!eu07::scene::detail::parseModelSegment(segment, parsed)) {
            return false;
        }
        base_models_out.push_back(bakeModel(parsed));
    }
    return true;
}

[[nodiscard]] inline bool tryStreamTokenModelOnlyToPack(
    const ParseResult& parsed,
    const std::filesystem::path& base_directory,
    PackFileCacheEntry& entry) {
    SceneDocument scratch;
    std::filesystem::path include_root = base_directory;
    if (include_root.empty()) {
        include_root = std::filesystem::current_path();
    }

    ParseContext context { scratch, {}, include_root, {} };
    context.expandIncludes = false;

    TokenStream stream(parsed.tokens);
    std::vector<eu07::scene::detail::ModelParseJob> jobs;
    jobs.reserve(parsed.tokens.size() / 16);

    while (!stream.empty()) {
        if (eu07::scene::detail::dispatchDirective(stream, context)) {
            continue;
        }

        if (!stream.empty() && isKeyword(stream.peek().value, "node")) {
            const std::size_t anchor = stream.checkpoint();
            NodeHeader header;
            std::string subtype;
            std::vector<SourceToken> raw;
            if (!node::io::consumeHeader(stream, header, subtype, raw)) {
                return false;
            }
            if (!isKeyword(subtype, node_model::kSubtype)) {
                return false;
            }
            node::applyScratchContext(header, context.scratch);
            jobs.push_back(eu07::scene::detail::ModelParseJob { anchor, header });
            if (!eu07::scene::detail::skipModelBody(stream)) {
                return false;
            }
            continue;
        }

        UnknownEntry unknown;
        unknown.line = stream.peek().sourceLine;
        unknown.token = stream.consume().value;
        scratch.unknown.push_back(std::move(unknown));
    }

    if (jobs.empty()) {
        return false;
    }

    entry.includes = std::move(scratch.include);
    if (const std::optional<IncludePlacement> placement = extractIncludePlacement(scratch)) {
        entry.placement = *placement;
    }

    entry.base_models.clear();
    entry.base_models.reserve(jobs.size());
    for (const eu07::scene::detail::ModelParseJob& job : jobs) {
        ParsedNodeModel model;
        TokenStream job_stream(parsed.tokens);
        job_stream.rewind(job.segment_index);
        NodeHeader parsed_header;
        std::string subtype;
        std::vector<SourceToken> raw;
        if (!node::io::consumeHeader(job_stream, parsed_header, subtype, raw) ||
            !isKeyword(subtype, node_model::kSubtype)) {
            return false;
        }
        model.header = job.header;
        if (!node_model::parseBody(job_stream, raw, job.header, model)) {
            return false;
        }
        entry.base_models.push_back(bakeModel(model));
    }
    return true;
}

// PACK compose (lightweight): tylko include z plaskich linii — bez modeli i tokenizacji.
[[nodiscard]] inline std::optional<SceneDocument>
extractFlatIncludesOnly(const std::vector<LogicalSegment>& segments) {
    SceneDocument document;
    for (const LogicalSegment& segment : segments) {
        std::string_view text = segment.text;
        eu07::skipFieldSeparators(text);
        if (text.empty()) {
            continue;
        }
        if (!eu07::scene::detail::segmentStartsWithKeyword(text, "include")) {
            continue;
        }
        ParsedInclude entry;
        if (!eu07::scene::detail::parseIncludeSegment(segment, entry)) {
            return std::nullopt;
        }
        document.include.push_back(std::move(entry));
    }
    return document;
}

// Duze plaskie .scm: linia po linii → batch → spool (bez readRawFile / base_models w RAM).
inline void streamFlatModelsToPackFlush(
    const std::filesystem::path& path,
    const std::function<void(std::vector<runtime::RuntimeModelInstance>&&)>& on_flush,
    const std::size_t batch_size = 4096) {
    if (!on_flush) {
        return;
    }
    std::vector<runtime::RuntimeModelInstance> batch;
    batch.reserve(batch_size);
    eu07::scene::detail::streamFlatFileModels(path, [&](const ParsedNodeModel& parsed) {
        batch.push_back(bakeModel(parsed));
        if (batch.size() >= batch_size) {
            on_flush(std::move(batch));
            batch.clear();
            batch.reserve(batch_size);
        }
    });
    if (!batch.empty()) {
        on_flush(std::move(batch));
    }
}

// Wspolny cache parse na cala sesje bakeModuleTree — bake_tree i PACK compose
// czytaja ten sam SceneDocument + PackFileCacheEntry (bez drugiego parseFile).
class ParseConcurrencyGate {
  public:
    explicit ParseConcurrencyGate(const unsigned limit)
        : limit_(std::max(1u, limit)) {}

    void acquire() {
        std::unique_lock lock(mutex_);
        cv_.wait(lock, [&]() { return in_flight_ < limit_; });
        ++in_flight_;
    }

    void release() {
        {
            std::lock_guard lock(mutex_);
            if (in_flight_ > 0) {
                --in_flight_;
            }
        }
        cv_.notify_one();
    }

    struct Guard {
        ParseConcurrencyGate& gate;
        explicit Guard(ParseConcurrencyGate& g) : gate(g) { gate.acquire(); }
        Guard(const Guard&) = delete;
        Guard& operator=(const Guard&) = delete;
        ~Guard() { gate.release(); }
    };

  private:
    unsigned const limit_;
    unsigned in_flight_ = 0;
    std::mutex mutex_;
    std::condition_variable cv_;
};

// Pliki > heavy_threshold_bytes: parse/bake wyłącznie pojedynczo (zero innych parse).
// Mniejsze pliki: do normal_limit rownolegle.
class PathAwareParseGate {
  public:
    PathAwareParseGate(
        const unsigned normal_limit,
        const std::size_t heavy_threshold_bytes,
        const bool allow_parallel_heavy = false)
        : normal_limit_(std::max(1u, normal_limit)),
          heavy_threshold_bytes_(heavy_threshold_bytes),
          allow_parallel_heavy_(allow_parallel_heavy) {}

    [[nodiscard]] static std::size_t sourceFileSizeBytes(const std::filesystem::path& path) {
        std::error_code ec;
        const auto size = std::filesystem::file_size(path, ec);
        return ec ? 0u : static_cast<std::size_t>(size);
    }

    struct Guard {
        PathAwareParseGate& gate;
        explicit Guard(PathAwareParseGate& g, const std::filesystem::path& path) : gate(g) {
            gate.acquire(path);
        }
        Guard(const Guard&) = delete;
        Guard& operator=(const Guard&) = delete;
        ~Guard() { gate.release(); }
    };

  private:
    friend struct Guard;

    void acquire(const std::filesystem::path& path) {
        const bool heavy = !allow_parallel_heavy_ && heavy_threshold_bytes_ > 0 &&
                           sourceFileSizeBytes(path) > heavy_threshold_bytes_;
        std::unique_lock lock(mutex_);
        if (heavy) {
            cv_.wait(lock, [&]() { return in_flight_ == 0; });
            heavy_active_ = true;
            in_flight_ = 1;
            return;
        }
        cv_.wait(lock, [&]() { return !heavy_active_ && in_flight_ < normal_limit_; });
        ++in_flight_;
    }

    void release() {
        {
            std::lock_guard lock(mutex_);
            if (heavy_active_) {
                heavy_active_ = false;
                in_flight_ = 0;
            } else if (in_flight_ > 0) {
                --in_flight_;
            }
        }
        cv_.notify_all();
    }

    unsigned const normal_limit_;
    std::size_t const heavy_threshold_bytes_;
    bool const allow_parallel_heavy_;
    unsigned in_flight_ = 0;
    bool heavy_active_ = false;
    std::mutex mutex_;
    std::condition_variable cv_;
};

class BakeParseSessionCache {
public:
    explicit BakeParseSessionCache(
        const unsigned max_concurrent_parses = 1,
        const bool low_memory = false,
        const std::size_t heavy_parse_threshold_bytes = 0)
        : parse_gate_(max_concurrent_parses, heavy_parse_threshold_bytes, low_memory),
          low_memory_mode_(low_memory) {}

    struct Entry {
        std::filesystem::file_time_type mtime {};
        bool has_document = false;
        SceneDocument document;
        bool pack_lightweight_only = false;
        bool has_pack = false;
        PackFileCacheEntry pack;
    };

    std::atomic<std::size_t> document_hits { 0 };
    std::atomic<std::size_t> document_misses { 0 };
    std::atomic<std::size_t> pack_hits { 0 };
    std::atomic<std::size_t> pack_builds { 0 };
    std::atomic<std::size_t> full_parses { 0 };

    [[nodiscard]] std::size_t fileCount() const {
        std::shared_lock lock(mutex_);
        return entries_.size();
    }

    [[nodiscard]] const SceneDocument& documentFor(
        const std::filesystem::path& path,
        const std::filesystem::path& scenery_root,
        PackComposeStats* stats = nullptr) {
        const std::string key = path.lexically_normal().generic_string();
        const std::filesystem::file_time_type mtime = readMtime(path);

        {
            std::shared_lock read_lock(mutex_);
            if (const auto found = entries_.find(key); found != entries_.end() &&
                                                     found->second.has_document &&
                                                     !found->second.pack_lightweight_only &&
                                                     found->second.mtime == mtime) {
                document_hits.fetch_add(1, std::memory_order_relaxed);
                return found->second.document;
            }
        }

        std::unique_lock write_lock(mutex_);
        Entry& slot = entries_[key];
        if (slot.has_document && !slot.pack_lightweight_only && slot.mtime == mtime) {
            document_hits.fetch_add(1, std::memory_order_relaxed);
            return slot.document;
        }

        document_misses.fetch_add(1, std::memory_order_relaxed);
        full_parses.fetch_add(1, std::memory_order_relaxed);

        // Nie trzymaj globalnego mutexa cache podczas parse (wolne I/O + tokenize).
        write_lock.unlock();
        PathAwareParseGate::Guard const parse_slot(parse_gate_, path);
        write_lock.lock();
        Entry& fresh = entries_[key];
        if (fresh.has_document && !fresh.pack_lightweight_only && fresh.mtime == mtime) {
            document_hits.fetch_add(1, std::memory_order_relaxed);
            return fresh.document;
        }

        parseIntoEntry(fresh, path, scenery_root, mtime, stats, false);
        return fresh.document;
    }

    [[nodiscard]] const SceneDocument& documentForPackCompose(
        const std::filesystem::path& path,
        const std::filesystem::path& scenery_root,
        PackComposeStats* stats = nullptr,
        const bool pack_lightweight = true) {
        const std::string key = path.lexically_normal().generic_string();
        const std::filesystem::file_time_type mtime = readMtime(path);

        {
            std::shared_lock read_lock(mutex_);
            if (const auto found = entries_.find(key); found != entries_.end() &&
                                                     found->second.has_document &&
                                                     found->second.mtime == mtime) {
                document_hits.fetch_add(1, std::memory_order_relaxed);
                return found->second.document;
            }
        }

        std::unique_lock write_lock(mutex_);
        Entry& slot = entries_[key];
        if (slot.has_document && slot.mtime == mtime) {
            document_hits.fetch_add(1, std::memory_order_relaxed);
            return slot.document;
        }

        document_misses.fetch_add(1, std::memory_order_relaxed);
        full_parses.fetch_add(1, std::memory_order_relaxed);

        write_lock.unlock();
        PathAwareParseGate::Guard const parse_slot(parse_gate_, path);
        write_lock.lock();
        Entry& fresh = entries_[key];
        if (fresh.has_document && fresh.mtime == mtime) {
            document_hits.fetch_add(1, std::memory_order_relaxed);
            return fresh.document;
        }

        parseIntoEntry(fresh, path, scenery_root, mtime, stats, pack_lightweight);
        return fresh.document;
    }

    [[nodiscard]] const PackFileCacheEntry* tryPackEntry(const std::filesystem::path& path) const {
        const std::string key = path.lexically_normal().generic_string();
        std::shared_lock read_lock(mutex_);
        if (const auto found = entries_.find(key); found != entries_.end() && found->second.has_pack) {
            return &found->second.pack;
        }
        return nullptr;
    }

    [[nodiscard]] const PackFileCacheEntry& packEntryFor(
        const std::filesystem::path& path,
        const std::filesystem::path& scenery_root,
        PackComposeStats* stats = nullptr) {
        if (const PackFileCacheEntry* cached = tryPackEntry(path)) {
            pack_hits.fetch_add(1, std::memory_order_relaxed);
            return *cached;
        }

        const std::string key = path.lexically_normal().generic_string();
        const std::filesystem::file_time_type mtime = readMtime(path);

        if (low_memory_mode_ &&
            (eu07::scene::detail::isIncFile(path.filename().string()) ||
             eu07::scene::detail::shouldStreamFlatSourceFile(path))) {
            std::unique_lock write_lock(mutex_);
            Entry& slot = entries_[key];
            if (slot.has_pack && slot.mtime == mtime) {
                pack_hits.fetch_add(1, std::memory_order_relaxed);
                return slot.pack;
            }

            write_lock.unlock();
            PathAwareParseGate::Guard const parse_slot(parse_gate_, path);
            write_lock.lock();
            Entry& fresh = entries_[key];
            if (fresh.has_pack && fresh.mtime == mtime) {
                pack_hits.fetch_add(1, std::memory_order_relaxed);
                return fresh.pack;
            }

            if (tryStreamIncPackEntry(path, scenery_root, fresh, mtime, stats)) {
                pack_builds.fetch_add(1, std::memory_order_relaxed);
                full_parses.fetch_add(1, std::memory_order_relaxed);
                return fresh.pack;
            }
            write_lock.unlock();
        }

        const bool pack_lightweight =
            !eu07::scene::detail::isIncFile(path.filename().string());
        documentForPackCompose(path, scenery_root, stats, pack_lightweight);

        std::unique_lock write_lock(mutex_);
        Entry& slot = entries_[key];
        if (!slot.has_pack) {
            buildPackFromDocument(slot, stats);
        } else {
            pack_hits.fetch_add(1, std::memory_order_relaxed);
        }
        return slot.pack;
    }

    // After pack exists, parsed nodeModel is redundant (see buildPackFromDocument).
    void releaseHeavyParseStorageAfterModuleBake(
        const std::filesystem::path& path,
        const bool models_pending_pack) {
        const std::string key = path.lexically_normal().generic_string();
        std::unique_lock write_lock(mutex_);
        if (const auto found = entries_.find(key); found != entries_.end()) {
            if (!found->second.has_document) {
                return;
            }
            const bool release_models = !models_pending_pack || found->second.has_pack;
            releaseHeavySceneParseStorage(found->second.document, release_models);
        }
    }

    // Drop parsed document after module bake+emit; retain PACK cache for root compose.
    void evictEntryIfNotNeeded(const std::filesystem::path& path) {
        const std::string key = path.lexically_normal().generic_string();
        std::unique_lock write_lock(mutex_);
        const auto found = entries_.find(key);
        if (found == entries_.end()) {
            return;
        }
        if (found->second.has_pack) {
            found->second.document = {};
            found->second.has_document = false;
            found->second.pack_lightweight_only = false;
            return;
        }
        (void)entries_.erase(key);
    }

    // PACK compose skopiowal modele — zwolnij cache (w low_memory: cale wpisy).
    void releasePackCacheAfterComposeUse(const std::filesystem::path& path) {
        if (low_memory_mode_) {
            dropPackCacheEntry(path);
            return;
        }
        releasePackBaseModelsUnlessInc(path);
    }

    void dropPackCacheEntry(const std::filesystem::path& path) {
        const std::string key = path.lexically_normal().generic_string();
        std::unique_lock write_lock(mutex_);
        (void)entries_.erase(key);
    }

private:
    void releasePackBaseModelsUnlessInc(const std::filesystem::path& path) {
        if (eu07::scene::detail::isIncFile(path.filename().string())) {
            return;
        }
        const std::string key = path.lexically_normal().generic_string();
        std::unique_lock write_lock(mutex_);
        if (const auto found = entries_.find(key); found != entries_.end() && found->second.has_pack) {
            found->second.pack.base_models.clear();
            found->second.pack.base_models.shrink_to_fit();
        }
    }

    [[nodiscard]] static std::filesystem::file_time_type readMtime(
        const std::filesystem::path& path) {
        std::error_code ec;
        const auto mtime = std::filesystem::last_write_time(path, ec);
        return ec ? std::filesystem::file_time_type {} : mtime;
    }

    static void addTimingUs(
        PackComposeStats* stats,
        std::atomic<std::uint64_t>& dst,
        const std::chrono::steady_clock::time_point begin,
        const std::chrono::steady_clock::time_point end) {
        if (stats == nullptr) {
            return;
        }
        dst.fetch_add(
            static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()),
            std::memory_order_relaxed);
    }

    void parseIntoEntry(
        Entry& slot,
        const std::filesystem::path& path,
        const std::filesystem::path& scenery_root,
        const std::filesystem::file_time_type mtime,
        PackComposeStats* stats,
        const bool pack_lightweight) {
        const auto parse_begin = std::chrono::steady_clock::now();

        const bool stream_source = pack_lightweight;
        const eu07::scene::detail::FlatFileKind flat_kind =
            (stream_source ||
             eu07::scene::detail::shouldStreamFlatSourceFile(path))
                ? eu07::scene::detail::scanFlatFileKindStreaming(path)
                : eu07::scene::detail::FlatFileKind::None;
        if (stream_source ||
            flat_kind != eu07::scene::detail::FlatFileKind::None) {
            if (flat_kind == eu07::scene::detail::FlatFileKind::Includes ||
                flat_kind == eu07::scene::detail::FlatFileKind::Models) {
                const std::optional<SceneDocument> flat =
                    eu07::scene::detail::buildFlatIncludesDocumentStreaming(path);
                if (flat) {
                    slot.document = *flat;
                    const auto process_end = std::chrono::steady_clock::now();
                    slot.mtime = mtime;
                    slot.has_document = true;
                    slot.pack_lightweight_only = true;
                    if (stats != nullptr) {
                        addTimingUs(stats, stats->process_us, parse_begin, process_end);
                        stats->parse_us.fetch_add(
                            static_cast<std::uint64_t>(
                                std::chrono::duration_cast<std::chrono::microseconds>(
                                    process_end - parse_begin)
                                    .count()),
                            std::memory_order_relaxed);
                    }
                    return;
                }
            }
            if (stream_source &&
                eu07::scene::detail::shouldStreamFlatSourceFile(path) &&
                flat_kind == eu07::scene::detail::FlatFileKind::None) {
                throw std::runtime_error(
                    "EU7: duzy plik nie-plaski — nie mozna zaladowac calego do RAM: " +
                    path.string());
            }
        }

        eu07::RawFile raw = readRawFile(path);
        SceneProcessOptions options;
        options.expandIncludes = false;
        options.packComposeLightweight = pack_lightweight;

        if (pack_lightweight) {
            // PACK compose: nie ladowac plaskiej flory (miliony modeli) — tylko include'y.
            const LogicalPass logical = toLogicalLines(raw.lines);
            const eu07::scene::detail::FlatFileKind kind =
                eu07::scene::detail::classifyFlatFile(logical.segments);
            if (kind == eu07::scene::detail::FlatFileKind::Includes) {
                if (const std::optional<SceneDocument> flat =
                        eu07::scene::detail::parseFlatIncludes(logical.segments)) {
                    slot.document = std::move(*flat);
                    const auto process_end = std::chrono::steady_clock::now();
                    slot.mtime = mtime;
                    slot.has_document = true;
                    slot.pack_lightweight_only = true;
                    if (stats != nullptr) {
                        addTimingUs(stats, stats->process_us, parse_begin, process_end);
                        stats->parse_us.fetch_add(
                            static_cast<std::uint64_t>(
                                std::chrono::duration_cast<std::chrono::microseconds>(
                                    process_end - parse_begin)
                                    .count()),
                            std::memory_order_relaxed);
                    }
                    return;
                }
            }
            if (kind == eu07::scene::detail::FlatFileKind::Models ||
                kind == eu07::scene::detail::FlatFileKind::None) {
                if (const std::optional<SceneDocument> flat =
                        extractFlatIncludesOnly(logical.segments)) {
                    slot.document = std::move(*flat);
                    const auto process_end = std::chrono::steady_clock::now();
                    slot.mtime = mtime;
                    slot.has_document = true;
                    slot.pack_lightweight_only = true;
                    if (stats != nullptr) {
                        addTimingUs(stats, stats->process_us, parse_begin, process_end);
                        stats->parse_us.fetch_add(
                            static_cast<std::uint64_t>(
                                std::chrono::duration_cast<std::chrono::microseconds>(
                                    process_end - parse_begin)
                                    .count()),
                            std::memory_order_relaxed);
                    }
                    return;
                }
            }
        } else if (std::optional<SceneDocument> flat =
                       eu07::scene::detail::tryProcessFlatSceneFile(raw, scenery_root)) {
            slot.document = std::move(*flat);
            const auto process_end = std::chrono::steady_clock::now();
            slot.mtime = mtime;
            slot.has_document = true;
            slot.pack_lightweight_only = pack_lightweight;
            if (stats != nullptr) {
                addTimingUs(stats, stats->process_us, parse_begin, process_end);
                stats->parse_us.fetch_add(
                    static_cast<std::uint64_t>(
                        std::chrono::duration_cast<std::chrono::microseconds>(
                            process_end - parse_begin)
                            .count()),
                    std::memory_order_relaxed);
            }
            return;
        }

        const ParseResult parsed = parseRawFile(std::move(raw));
        const auto tokenize_end = std::chrono::steady_clock::now();
        slot.document = processScene(parsed, scenery_root, options).document;
        const auto process_end = std::chrono::steady_clock::now();

        slot.mtime = mtime;
        slot.has_document = true;
        slot.pack_lightweight_only = pack_lightweight;

        if (stats != nullptr) {
            addTimingUs(stats, stats->tokenize_us, parse_begin, tokenize_end);
            addTimingUs(stats, stats->process_us, tokenize_end, process_end);
            stats->parse_us.fetch_add(
                static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(process_end - parse_begin)
                        .count()),
                std::memory_order_relaxed);
        }
    }

    void buildPackFromDocument(Entry& slot, PackComposeStats* stats) {
        const auto begin = std::chrono::steady_clock::now();
        slot.pack = low_memory_mode_ ? makePackFileCacheEntryConsuming(slot.document)
                                     : makePackFileCacheEntry(slot.document);
        const auto end = std::chrono::steady_clock::now();
        slot.has_pack = true;
        slot.document = {};
        slot.has_document = false;
        slot.pack_lightweight_only = false;
        pack_builds.fetch_add(1, std::memory_order_relaxed);
        addTimingUs(stats, stats->makeentry_us, begin, end);
    }

    [[nodiscard]] bool tryStreamIncPackEntry(
        const std::filesystem::path& path,
        const std::filesystem::path& scenery_root,
        Entry& slot,
        const std::filesystem::file_time_type mtime,
        PackComposeStats* stats) {
        (void)scenery_root;
        const auto parse_begin = std::chrono::steady_clock::now();

        const eu07::scene::detail::FlatFileKind kind =
            eu07::scene::detail::scanFlatFileKindStreaming(path);
        if (kind == eu07::scene::detail::FlatFileKind::None) {
            return false;
        }

        PackFileCacheEntry built;

        if (kind == eu07::scene::detail::FlatFileKind::Models) {
            eu07::scene::detail::streamFlatFileModels(
                path, [&](const ParsedNodeModel& parsed) {
                    built.base_models.push_back(bakeModel(parsed));
                });
        } else if (kind == eu07::scene::detail::FlatFileKind::Includes) {
            const std::optional<SceneDocument> doc =
                eu07::scene::detail::buildFlatIncludesDocumentStreaming(path);
            if (!doc) {
                return false;
            }
            built.includes = std::move(doc->include);
            if (const std::optional<IncludePlacement> placement = extractIncludePlacement(*doc)) {
                built.placement = *placement;
            }
        } else {
            return false;
        }

        const auto end = std::chrono::steady_clock::now();
        if (stats != nullptr) {
            addTimingUs(stats, stats->process_us, parse_begin, end);
            addTimingUs(stats, stats->makeentry_us, parse_begin, end);
            stats->parse_us.fetch_add(
                static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(end - parse_begin)
                        .count()),
                std::memory_order_relaxed);
        }
        slot.pack = std::move(built);
        slot.mtime = mtime;
        slot.has_pack = true;
        slot.has_document = false;
        slot.pack_lightweight_only = true;
        return true;
    }

    mutable std::shared_mutex mutex_;
    std::unordered_map<std::string, Entry> entries_;
    PathAwareParseGate parse_gate_;
    bool const low_memory_mode_;
};

inline void printBakeParseSessionStats(const BakeParseSessionCache& cache, std::ostream& out) {
    out << "[EU7]   session cache: pliki=" << cache.fileCount()
        << " doc_hit=" << cache.document_hits.load(std::memory_order_relaxed)
        << " doc_miss=" << cache.document_misses.load(std::memory_order_relaxed)
        << " pack_hit=" << cache.pack_hits.load(std::memory_order_relaxed)
        << " pack_build=" << cache.pack_builds.load(std::memory_order_relaxed)
        << " parse=" << cache.full_parses.load(std::memory_order_relaxed) << '\n'
        << std::flush;
}

class PackSectionAccumulator {
public:
    static constexpr unsigned kShardCount = 64;

    void add(std::vector<runtime::RuntimeModelInstance>&& batch, PackComposeStats* stats = nullptr) {
        if (stats != nullptr) {
            stats->sink_batches.fetch_add(1, std::memory_order_relaxed);
        }
        models_total_ += batch.size();
        std::array<
            std::unordered_map<
                binary::codec::TerrSectionKey,
                std::vector<runtime::RuntimeModelInstance>,
                binary::codec::TerrSectionKeyHash>,
            kShardCount>
            staged {};
        for (runtime::RuntimeModelInstance& model : batch) {
            const binary::codec::TerrSectionKey sectionKey =
                binary::codec::clampTerrSectionKey(binary::codec::terrSectionKey(model.location));
            const std::size_t shard =
                binary::codec::TerrSectionKeyHash {}(sectionKey) % kShardCount;
            staged[shard][sectionKey].push_back(std::move(model));
        }
        for (unsigned shard = 0; shard < kShardCount; ++shard) {
            if (staged[shard].empty()) {
                continue;
            }
            std::lock_guard lock(shard_mutex_[shard]);
            for (auto& [sectionKey, models] : staged[shard]) {
                std::vector<runtime::RuntimeModelInstance>& target = shards_[shard][sectionKey];
                target.insert(
                    target.end(),
                    std::make_move_iterator(models.begin()),
                    std::make_move_iterator(models.end()));
            }
        }
    }

    void mergeFrom(PackSectionAccumulator&& other) {
        models_total_ += other.models_total_;
        other.models_total_ = 0;
        for (unsigned shard = 0; shard < kShardCount; ++shard) {
            if (other.shards_[shard].empty()) {
                continue;
            }
            std::lock_guard lock(shard_mutex_[shard]);
            for (auto& [sectionKey, models] : other.shards_[shard]) {
                std::vector<runtime::RuntimeModelInstance>& target = shards_[shard][sectionKey];
                target.insert(
                    target.end(),
                    std::make_move_iterator(models.begin()),
                    std::make_move_iterator(models.end()));
            }
            other.shards_[shard].clear();
        }
    }

    [[nodiscard]] std::size_t modelsTotal() const {
        return models_total_;
    }

    [[nodiscard]] std::vector<binary::codec::ModelSectionBatch> finalize() {
        std::unordered_map<
            binary::codec::TerrSectionKey,
            std::vector<runtime::RuntimeModelInstance>,
            binary::codec::TerrSectionKeyHash>
            merged;
        merged.reserve(512);

        for (unsigned shard = 0; shard < kShardCount; ++shard) {
            std::lock_guard lock(shard_mutex_[shard]);
            for (auto& [key, models] : shards_[shard]) {
                std::vector<runtime::RuntimeModelInstance>& target = merged[key];
                target.insert(
                    target.end(),
                    std::make_move_iterator(models.begin()),
                    std::make_move_iterator(models.end()));
            }
            shards_[shard].clear();
        }

        std::vector<binary::codec::ModelSectionBatch> batches;
        batches.reserve(merged.size());
        for (auto& [key, models] : merged) {
            binary::codec::ModelSectionBatch batch;
            batch.section = key;
            batch.models = std::move(models);
            batches.push_back(std::move(batch));
        }

        std::sort(
            batches.begin(),
            batches.end(),
            [](const binary::codec::ModelSectionBatch& a,
               const binary::codec::ModelSectionBatch& b) noexcept {
                if (a.section.z != b.section.z) {
                    return a.section.z < b.section.z;
                }
                return a.section.x < b.section.x;
            });
        return batches;
    }

private:
    std::array<
        std::unordered_map<
            binary::codec::TerrSectionKey,
            std::vector<runtime::RuntimeModelInstance>,
            binary::codec::TerrSectionKeyHash>,
        kShardCount>
        shards_ {};
    std::array<std::mutex, kShardCount> shard_mutex_ {};
    std::size_t models_total_ = 0;
};

// low_memory: od razu do pack_batches (bez osobnego duzego akumulatora + finalize merge).
class PackBatchDirectBuilder {
public:
    PackBatchDirectBuilder(
        std::vector<binary::codec::ModelSectionBatch>* batches,
        const bool thread_safe)
        : batches_(batches), thread_safe_(thread_safe) {
        batches_->clear();
    }

    void set_flush(
        std::size_t threshold,
        std::function<void(std::vector<runtime::RuntimeModelInstance>&&)> on_flush) {
        flush_threshold_ = threshold;
        on_flush_ = std::move(on_flush);
    }

    void add(std::vector<runtime::RuntimeModelInstance>&& batch, PackComposeStats* stats = nullptr) {
        if (batch.empty()) {
            return;
        }
        if (stats != nullptr) {
            stats->sink_batches.fetch_add(1, std::memory_order_relaxed);
        }
        models_total_ += batch.size();
        in_ram_models_ += batch.size();

        std::array<
            std::unordered_map<
                binary::codec::TerrSectionKey,
                std::vector<runtime::RuntimeModelInstance>,
                binary::codec::TerrSectionKeyHash>,
            PackSectionAccumulator::kShardCount>
            staged {};
        for (runtime::RuntimeModelInstance& model : batch) {
            const binary::codec::TerrSectionKey section_key =
                binary::codec::clampTerrSectionKey(binary::codec::terrSectionKey(model.location));
            const std::size_t shard =
                binary::codec::TerrSectionKeyHash {}(section_key) %
                PackSectionAccumulator::kShardCount;
            staged[shard][section_key].push_back(std::move(model));
        }
        batch.clear();

        if (thread_safe_) {
            std::lock_guard lock(mutex_);
            mergeStaged(staged);
            maybe_flush();
        } else {
            mergeStaged(staged);
            maybe_flush();
        }
    }

    void finalizeSort() {
        if (batches_ == nullptr || batches_->empty()) {
            return;
        }
        if (thread_safe_) {
            std::lock_guard lock(mutex_);
            sortBatches();
        } else {
            sortBatches();
        }
    }

    [[nodiscard]] std::size_t modelsTotal() const noexcept {
        return models_total_;
    }

    [[nodiscard]] std::size_t flushedTotal() const noexcept {
        return flushed_total_;
    }

    // Spłucz wszystkie sekcje PACK z RAM (np. po kazdym pliku w compose).
    void flushAll() {
        if (!on_flush_ || batches_ == nullptr || batches_->empty()) {
            return;
        }
        if (thread_safe_) {
            std::lock_guard lock(mutex_);
            flushAllUnlocked();
        } else {
            flushAllUnlocked();
        }
    }

  private:
    void flushAllUnlocked() {
        if (!on_flush_ || batches_->empty()) {
            return;
        }
        std::vector<runtime::RuntimeModelInstance> chunk;
        chunk.reserve(in_ram_models_);
        for (binary::codec::ModelSectionBatch& batch : *batches_) {
            chunk.insert(
                chunk.end(),
                std::make_move_iterator(batch.models.begin()),
                std::make_move_iterator(batch.models.end()));
            batch.models.clear();
        }
        batches_->clear();
        section_index_.clear();
        flushed_total_ += in_ram_models_;
        in_ram_models_ = 0;
        on_flush_(std::move(chunk));
    }

    void rebuildSectionIndex() {
        section_index_.clear();
        for (std::size_t index = 0; index < batches_->size(); ++index) {
            section_index_.emplace(batches_->at(index).section, index);
        }
    }

    void maybe_flush() {
        if (flush_threshold_ == 0 || !on_flush_ || in_ram_models_ <= flush_threshold_) {
            return;
        }
        while (in_ram_models_ > flush_threshold_ && !batches_->empty()) {
            std::size_t best = 0;
            for (std::size_t index = 1; index < batches_->size(); ++index) {
                if (batches_->at(index).models.size() > batches_->at(best).models.size()) {
                    best = index;
                }
            }
            binary::codec::ModelSectionBatch& batch = batches_->at(best);
            const std::size_t flushed_count = batch.models.size();
            in_ram_models_ -= flushed_count;
            flushed_total_ += flushed_count;
            std::vector<runtime::RuntimeModelInstance> chunk = std::move(batch.models);
            batches_->erase(batches_->begin() + static_cast<std::ptrdiff_t>(best));
            rebuildSectionIndex();
            on_flush_(std::move(chunk));
        }
    }

    void sortBatches() {
        std::sort(
            batches_->begin(),
            batches_->end(),
            [](const binary::codec::ModelSectionBatch& a,
               const binary::codec::ModelSectionBatch& b) noexcept {
                if (a.section.z != b.section.z) {
                    return a.section.z < b.section.z;
                }
                return a.section.x < b.section.x;
            });
    }

    void mergeStaged(
        std::array<
            std::unordered_map<
                binary::codec::TerrSectionKey,
                std::vector<runtime::RuntimeModelInstance>,
                binary::codec::TerrSectionKeyHash>,
            PackSectionAccumulator::kShardCount>& staged) {
        for (unsigned shard = 0; shard < PackSectionAccumulator::kShardCount; ++shard) {
            for (auto& [section_key, models] : staged[shard]) {
                const auto found = section_index_.find(section_key);
                if (found == section_index_.end()) {
                    binary::codec::ModelSectionBatch batch;
                    batch.section = section_key;
                    batch.models = std::move(models);
                    batches_->push_back(std::move(batch));
                    section_index_.emplace(section_key, batches_->size() - 1);
                } else {
                    std::vector<runtime::RuntimeModelInstance>& target =
                        batches_->at(found->second).models;
                    target.insert(
                        target.end(),
                        std::make_move_iterator(models.begin()),
                        std::make_move_iterator(models.end()));
                }
            }
        }
    }

    std::vector<binary::codec::ModelSectionBatch>* batches_ = nullptr;
    std::unordered_map<
        binary::codec::TerrSectionKey,
        std::size_t,
        binary::codec::TerrSectionKeyHash>
        section_index_;
    std::mutex mutex_;
    bool const thread_safe_ = false;
    std::size_t models_total_ = 0;
    std::size_t in_ram_models_ = 0;
    std::size_t flushed_total_ = 0;
    std::size_t flush_threshold_ = 0;
    std::function<void(std::vector<runtime::RuntimeModelInstance>&&)> on_flush_;
};

struct PackComposeWorkItem {
    std::filesystem::path path;
    std::string relativeFile;
    std::vector<std::filesystem::path> includeStack;
    runtime::TransformContext includePrefix;
    std::vector<std::string> includeParameters;
    IncludePlacement pending_placement;
    std::vector<std::string> pending_placement_parameters;
};

struct PackIncludeFoldPlan {
    const ParsedInclude* include = nullptr;
    std::string relative_file;
    std::vector<std::filesystem::path> stack_suffix;
};

[[nodiscard]] inline std::vector<runtime::RuntimeModelInstance> instantiateModels(
    const PackFileCacheEntry& entry,
    const runtime::TransformContext& includePrefix,
    const std::span<const std::string> includeParameters,
    PackComposeStats* stats = nullptr) {
    const bool needs_parameters = !includeParameters.empty();
    const bool needs_prefix = !transformIsEmpty(includePrefix);
    const bool needs_placement = !entry.placement.empty();
    if (!needs_parameters && !needs_prefix && !needs_placement) {
        if (stats != nullptr) {
            stats->instantiate_fast.fetch_add(1, std::memory_order_relaxed);
        }
        // Copy is unavoidable when callers mutate/sink owns the batch, but avoid
        // doubling peak RAM on empty templates (common for .inc placement chains).
        if (entry.base_models.empty()) {
            return {};
        }
        return entry.base_models;
    }
    if (stats != nullptr) {
        stats->instantiate_full.fetch_add(1, std::memory_order_relaxed);
    }

    std::vector<runtime::RuntimeModelInstance> models = entry.base_models;
    if (needs_parameters) {
        applyIncludeParametersToModels(models, includeParameters);
    }
    if (needs_prefix) {
        composeModelsWithPrefix(models, includePrefix);
    }
    if (needs_placement) {
        applyIncludePlacementToModels(models, entry.placement, includeParameters);
    }
    return models;
}

class PackComposeSession {
public:
    PackComposeSession(
        const unsigned max_threads,
        PackComposeProgress* progress,
        const std::filesystem::path& scenery_root,
        PackComposeStats* stats = nullptr,
        BakeParseSessionCache* session_cache = nullptr)
        : max_threads_(resolveComposeThreadCount(max_threads))
        , gate_(max_threads_)
        , progress_(progress)
        , scenery_root_(scenery_root)
        , stats_(stats)
        , session_cache_(session_cache) {}

    void seedCache(const std::filesystem::path& resolved, PackFileCacheEntry entry) {
        const std::string key = resolved.lexically_normal().generic_string();
        std::unique_lock write_lock(cache_mutex_);
        cache_.emplace(key, std::move(entry));
        if (stats_ != nullptr) {
            stats_->cache_seeded.fetch_add(1, std::memory_order_relaxed);
            stats_->unique_files_cached.store(cache_.size(), std::memory_order_relaxed);
        }
    }

    void runCompose(
        const std::filesystem::path& path,
        const std::string& relativeFile,
        const std::vector<std::filesystem::path>& includeStack,
        const runtime::TransformContext& includePrefix,
        const std::span<const std::string> includeParameters,
        PackModelBatchSink& sink,
        std::function<void()>* after_file_done = nullptr) {
        std::mutex queue_mutex;
        std::deque<PackComposeWorkItem> queue;
        std::condition_variable queue_cv;
        std::condition_variable done_cv;
        std::atomic<std::size_t> pending_tasks { 0 };
        std::atomic<bool> stop_workers { false };
        std::atomic<bool> failed { false };
        std::exception_ptr first_error;
        std::mutex error_mutex;

        auto submit = [&](PackComposeWorkItem item) {
            std::size_t const now_pending {
                pending_tasks.fetch_add(1, std::memory_order_acq_rel) + 1 };
            if (stats_ != nullptr) {
                stats_->diag_pending.store(now_pending, std::memory_order_relaxed);
            }
            std::size_t queue_size = 0;
            {
                std::lock_guard lock(queue_mutex);
                queue.push_back(std::move(item));
                queue_size = queue.size();
            }
            if (stats_ != nullptr) {
                std::size_t peak = stats_->queue_peak.load(std::memory_order_relaxed);
                while (queue_size > peak &&
                       !stats_->queue_peak.compare_exchange_weak(
                           peak, queue_size, std::memory_order_relaxed, std::memory_order_relaxed)) {
                }
            }
            queue_cv.notify_one();
        };

        auto finish_task = [&]() {
            // The completion notification must be issued under queue_mutex: the
            // main thread waits on done_cv with that mutex and re-checks the
            // atomic predicate, so notifying without the lock can drop the final
            // wakeup (deadlock with all threads idle). Callers must NOT already
            // hold queue_mutex.
            std::size_t const prev { pending_tasks.fetch_sub(1, std::memory_order_acq_rel) };
            if (stats_ != nullptr) {
                stats_->diag_pending.store(prev - 1, std::memory_order_relaxed);
            }
            if (prev == 1) {
                std::lock_guard lock(queue_mutex);
                done_cv.notify_all();
            }
        };

        auto process_one = [&](PackComposeWorkItem work) {
            bool folded_hop = false;
            std::size_t task_fold_steps = 0;
            auto release_visited_pack = [&](const std::filesystem::path& file) {
                if (session_cache_ != nullptr) {
                    session_cache_->releasePackCacheAfterComposeUse(file);
                    return;
                }
                if (eu07::scene::detail::isIncFile(file.filename().string())) {
                    return;
                }
                const std::string key = file.lexically_normal().generic_string();
                std::unique_lock lock(cache_mutex_);
                if (const auto found = cache_.find(key); found != cache_.end()) {
                    found->second.base_models.clear();
                    found->second.base_models.shrink_to_fit();
                }
            };
            while (true) {
                const std::filesystem::path resolved = work.path.lexically_normal();
                // Surface the file we are about to process so a long (or stuck)
                // compose shows which scenery file it is currently working on.
                reportActiveFile(resolved);
                if (eu07::scene::detail::isIncludeCycle(resolved, work.includeStack)) {
                    if (stats_ != nullptr) {
                        stats_->cycles_skipped.fetch_add(1, std::memory_order_relaxed);
                    }
                    return;
                }

                if (const PackFileCacheEntry* known = tryGetCached(resolved)) {
                    (void)known;
                } else if (!std::filesystem::exists(resolved)) {
                    if (work.includeStack.empty()) {
                        throw std::runtime_error("EU7 PACK: brak pliku: " + resolved.string());
                    }
                    if (stats_ != nullptr) {
                        stats_->missing_skipped.fetch_add(1, std::memory_order_relaxed);
                    }
                    return;
                }

                const PackFileCacheEntry& cached = loadCached(resolved);
                const auto inst_begin = std::chrono::steady_clock::now();
                std::vector<runtime::RuntimeModelInstance> models = instantiateModels(
                    cached, work.includePrefix, work.includeParameters, stats_);
                if (!work.pending_placement.empty()) {
                    applyIncludePlacementToModels(
                        models, work.pending_placement, work.pending_placement_parameters);
                    work.pending_placement = {};
                    work.pending_placement_parameters.clear();
                }
                if (stats_ != nullptr) {
                    const auto inst_end = std::chrono::steady_clock::now();
                    stats_->instantiate_us.fetch_add(
                        static_cast<std::uint64_t>(
                            std::chrono::duration_cast<std::chrono::microseconds>(
                                inst_end - inst_begin)
                                .count()),
                        std::memory_order_relaxed);
                }
                reportProgress(resolved, models.size());

                std::optional<PackIncludeFoldPlan> fold_plan;
                std::vector<PackIncludeFoldPlan> fanout_plans;

                // .inc to szablon placementu — ten sam plik na kazdym segmencie lancucha jest OK.
                // Cykl sprawdzamy tylko w obecnym przejsciu inc->inc, nie w historii .scm.
                auto is_inc_walk_cycle = [&](const std::filesystem::path& target,
                                             const std::vector<std::filesystem::path>& suffix) {
                    const std::filesystem::path normalized = target.lexically_normal();
                    for (const std::filesystem::path& entry : suffix) {
                        if (entry.lexically_normal() == normalized) {
                            return true;
                        }
                    }
                    return false;
                };

                auto report_compose_throttle = [&](const std::filesystem::path& hint) {
                    if (progress_ == nullptr || progress_->on_progress == nullptr) {
                        return;
                    }
                    const auto now = std::chrono::steady_clock::now();
                    const bool timed =
                        !progress_->reported_once ||
                        std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - progress_->last_report)
                                .count() >= 100;
                    if (!timed) {
                        return;
                    }
                    std::lock_guard lock(progress_mutex_);
                    progress_->reported_once = true;
                    progress_->last_report = now;
                    progress_->on_progress(
                        hint,
                        stats_ != nullptr ? stats_->files_visited.load(std::memory_order_relaxed)
                                          : files_visited_.load(std::memory_order_relaxed),
                        stats_ != nullptr ? stats_->models_emitted.load(std::memory_order_relaxed)
                                          : models_total_.load(std::memory_order_relaxed));
                };

                auto note_inc_inline = [&](const std::filesystem::path& inc_path) {
                    if (stats_ != nullptr) {
                        const std::size_t steps =
                            stats_->inc_includes_inlined.fetch_add(1, std::memory_order_relaxed) +
                            1;
                        if ((steps % 50) == 0) {
                            report_compose_throttle(inc_path);
                        }
                    }
                };

                auto absorb_continuations = [&](const std::vector<const ParsedInclude*>& locals,
                                                const std::string& relative_file,
                                                const std::vector<std::filesystem::path>& suffix) {
                    if (locals.empty()) {
                        return;
                    }
                    if (locals.size() == 1) {
                        if (!fold_plan.has_value()) {
                            fold_plan = PackIncludeFoldPlan{locals.front(), relative_file, suffix};
                        }
                        return;
                    }
                    for (const ParsedInclude* include : locals) {
                        fanout_plans.push_back(PackIncludeFoldPlan{include, relative_file, suffix});
                    }
                };

                auto walk_inc_chain = [&](const ParsedInclude& start_inc,
                                          std::string relative_file,
                                          std::vector<std::filesystem::path> suffix) {
                    const ParsedInclude* step = &start_inc;
                    std::vector<std::string> placement_params = start_inc.parameters;

                    while (true) {
                        // Single memoized lookup yields the resolved path AND the
                        // parsed entry (or a missing marker) behind one shared lock,
                        // avoiding the per-placement resolve/stat/probe storm.
                        const ResolvedInclude& resolved_inc =
                            resolveIncludeCached(relative_file, step->file);
                        const std::filesystem::path& inc_path = resolved_inc.path;
                        if (is_inc_walk_cycle(inc_path, suffix)) {
                            if (stats_ != nullptr) {
                                stats_->cycles_skipped.fetch_add(1, std::memory_order_relaxed);
                            }
                            return;
                        }
                        if (resolved_inc.missing || resolved_inc.entry == nullptr) {
                            if (stats_ != nullptr) {
                                stats_->missing_skipped.fetch_add(1, std::memory_order_relaxed);
                            }
                            return;
                        }
                        noteCacheHit();

                        const PackFileCacheEntry& inc_entry = *resolved_inc.entry;
                        if (!inc_entry.placement.empty()) {
                            work.pending_placement = inc_entry.placement;
                            work.pending_placement_parameters = placement_params;
                        }
                        if (!inc_entry.base_models.empty()) {
                            std::vector<runtime::RuntimeModelInstance> inc_models =
                                instantiateModels(
                                    inc_entry, step->siteTransform, placement_params, stats_);
                            if (!inc_models.empty()) {
                                sink(std::move(inc_models));
                            }
                        }
                        note_inc_inline(inc_path);

                        const std::string& inc_relative = resolved_inc.relative;
                        std::vector<const ParsedInclude*> non_inc;
                        non_inc.reserve(inc_entry.includes.size());
                        const ParsedInclude* next_inc = nullptr;
                        for (const ParsedInclude& inner : inc_entry.includes) {
                            if (eu07::scene::detail::isIncFile(inner.file)) {
                                next_inc = &inner;
                                continue;
                            }
                            non_inc.push_back(&inner);
                        }

                        if (!non_inc.empty()) {
                            absorb_continuations(non_inc, inc_relative, suffix);
                            return;
                        }

                        if (next_inc != nullptr) {
                            suffix.push_back(inc_path);
                            relative_file = inc_relative;
                            placement_params = next_inc->parameters;
                            step = next_inc;
                            continue;
                        }

                        return;
                    }
                };

                auto submit_fold_plan = [&](const PackIncludeFoldPlan& plan) {
                    PackComposeWorkItem child;
                    child.path = eu07::scene::detail::resolveIncludeSourcePath(
                        scenery_root_, plan.relative_file, plan.include->file);
                    child.relativeFile =
                        eu07::scene::detail::relativeSceneryFile(scenery_root_, child.path);
                    child.includeStack = work.includeStack;
                    child.includeStack.push_back(resolved);
                    child.includePrefix = plan.include->siteTransform;
                    child.includeParameters = plan.include->parameters;
                    child.pending_placement = work.pending_placement;
                    child.pending_placement_parameters = work.pending_placement_parameters;
                    submit(std::move(child));
                    if (stats_ != nullptr) {
                        stats_->includes_submitted.fetch_add(1, std::memory_order_relaxed);
                    }
                };

                const bool parent_has_branching = cached.includes.size() > 1;
                std::vector<const ParsedInclude*> local_continuations;
                local_continuations.reserve(cached.includes.size());
                for (const ParsedInclude& include : cached.includes) {
                    if (eu07::scene::detail::isIncFile(include.file)) {
                        walk_inc_chain(include, work.relativeFile, {});
                        if (fold_plan.has_value() && parent_has_branching) {
                            submit_fold_plan(*fold_plan);
                            fold_plan.reset();
                        }
                        continue;
                    }
                    local_continuations.push_back(&include);
                }
                absorb_continuations(local_continuations, work.relativeFile, {});

                auto resolve_plan_path = [&](const PackIncludeFoldPlan& plan) {
                    return eu07::scene::detail::resolveIncludeSourcePath(
                               scenery_root_, plan.relative_file, plan.include->file)
                        .lexically_normal();
                };

                if (fold_plan.has_value() && !fanout_plans.empty()) {
                    const std::filesystem::path fold_path = resolve_plan_path(*fold_plan);
                    fanout_plans.erase(
                        std::remove_if(
                            fanout_plans.begin(),
                            fanout_plans.end(),
                            [&](const PackIncludeFoldPlan& plan) {
                                return resolve_plan_path(plan) == fold_path;
                            }),
                        fanout_plans.end());
                }

                if (fold_plan.has_value() && fanout_plans.empty()) {
                    if (!models.empty()) {
                        sink(std::move(models));
                    }
                    const PackIncludeFoldPlan& plan = *fold_plan;
                    const std::filesystem::path next_path =
                        eu07::scene::detail::resolveIncludeSourcePath(
                            scenery_root_, plan.relative_file, plan.include->file)
                            .lexically_normal();
                    if (next_path == resolved) {
                        if (stats_ != nullptr) {
                            stats_->cycles_skipped.fetch_add(1, std::memory_order_relaxed);
                        }
                        if (!models.empty()) {
                            sink(std::move(models));
                        }
                        release_visited_pack(resolved);
                        return;
                    }
                    release_visited_pack(resolved);
                    work.includeStack.push_back(resolved);
                    work.path = next_path;
                    work.relativeFile =
                        eu07::scene::detail::relativeSceneryFile(scenery_root_, work.path);
                    work.includePrefix = plan.include->siteTransform;
                    work.includeParameters = plan.include->parameters;
                    ++task_fold_steps;
                    if (stats_ != nullptr) {
                        stats_->chain_fold_steps.fetch_add(1, std::memory_order_relaxed);
                        if (!folded_hop) {
                            folded_hop = true;
                            stats_->chain_fold_hops.fetch_add(1, std::memory_order_relaxed);
                        }
                    }
                    if (task_fold_steps >= kMaxChainFoldPerTask) {
                        submit(std::move(work));
                        if (stats_ != nullptr) {
                            stats_->includes_submitted.fetch_add(1, std::memory_order_relaxed);
                        }
                        if (!models.empty()) {
                            sink(std::move(models));
                        }
                        release_visited_pack(resolved);
                        return;
                    }
                    continue;
                }

                if (!fanout_plans.empty()) {
                    for (const PackIncludeFoldPlan& plan : fanout_plans) {
                        submit_fold_plan(plan);
                    }
                }

                if (!models.empty()) {
                    sink(std::move(models));
                }
                release_visited_pack(resolved);
                return;
            }
        };

        auto worker_loop = [&]() {
            if (stats_ != nullptr) {
                stats_->diag_workers_alive.fetch_add(1, std::memory_order_relaxed);
            }
            while (true) {
                PackComposeWorkItem work;
                {
                    std::unique_lock lock(queue_mutex);
                    if (stats_ != nullptr) {
                        stats_->diag_workers_idle.fetch_add(1, std::memory_order_relaxed);
                    }
                    queue_cv.wait(lock, [&]() {
                        return stop_workers.load(std::memory_order_acquire) || !queue.empty();
                    });
                    if (stats_ != nullptr) {
                        stats_->diag_workers_idle.fetch_sub(1, std::memory_order_relaxed);
                    }
                    if (stop_workers.load(std::memory_order_acquire) && queue.empty()) {
                        if (stats_ != nullptr) {
                            stats_->diag_workers_alive.fetch_sub(1, std::memory_order_relaxed);
                        }
                        return;
                    }
                    if (queue.empty()) {
                        continue;
                    }
                    work = std::move(queue.front());
                    queue.pop_front();
                }

                if (stats_ != nullptr) {
                    stats_->diag_workers_inst.fetch_add(1, std::memory_order_relaxed);
                }
                try {
                    process_one(std::move(work));
                    if (after_file_done != nullptr && *after_file_done) {
                        (*after_file_done)();
                    }
                    if (stats_ != nullptr) {
                        stats_->diag_workers_inst.fetch_sub(1, std::memory_order_relaxed);
                    }
                } catch (...) {
                    if (stats_ != nullptr) {
                        stats_->diag_workers_inst.fetch_sub(1, std::memory_order_relaxed);
                    }
                    {
                        std::lock_guard lock(error_mutex);
                        if (!first_error) {
                            first_error = std::current_exception();
                        }
                    }
                    failed.store(true, std::memory_order_release);
                    stop_workers.store(true, std::memory_order_release);
                    queue_cv.notify_all();
                    {
                        std::lock_guard lock(queue_mutex);
                        done_cv.notify_all();
                    }
                }
                finish_task();
            }
        };

        if (stats_ != nullptr) {
            stats_->diag_gate_max.store(gate_.maxSlots(), std::memory_order_relaxed);
        }
        std::vector<std::thread> workers;
        workers.reserve(max_threads_);
        for (unsigned worker = 0; worker < max_threads_; ++worker) {
            workers.emplace_back(worker_loop);
        }

        PackComposeWorkItem root;
        root.path = path;
        root.relativeFile = relativeFile;
        root.includeStack = includeStack;
        root.includePrefix = includePrefix;
        root.includeParameters.assign(includeParameters.begin(), includeParameters.end());
        submit(std::move(root));

        {
            std::unique_lock lock(queue_mutex);
            if (stats_ != nullptr) {
                stats_->diag_main_wait.store(1, std::memory_order_relaxed);
            }
            done_cv.wait(lock, [&]() {
                return pending_tasks.load(std::memory_order_acquire) == 0 ||
                       failed.load(std::memory_order_acquire);
            });
            if (stats_ != nullptr) {
                stats_->diag_main_wait.store(0, std::memory_order_relaxed);
            }
        }

        stop_workers.store(true, std::memory_order_release);
        {
            // Already holding queue_mutex here, so decrement directly instead of
            // finish_task() (which would re-lock queue_mutex and deadlock).
            std::lock_guard lock(queue_mutex);
            while (!queue.empty()) {
                queue.pop_front();
                pending_tasks.fetch_sub(1, std::memory_order_acq_rel);
            }
        }
        queue_cv.notify_all();
        for (std::thread& worker : workers) {
            if (worker.joinable()) {
                worker.join();
            }
        }

        if (first_error) {
            std::rethrow_exception(first_error);
        }
    }

    [[nodiscard]] std::size_t cacheHits() const {
        return cache_hits_.load(std::memory_order_relaxed);
    }

    [[nodiscard]] std::size_t cacheMisses() const {
        return cache_misses_.load(std::memory_order_relaxed);
    }

private:
    [[nodiscard]] PackFileCacheEntry buildCacheEntry(const std::filesystem::path& resolved) {
        const auto add_us = [&](std::atomic<std::uint64_t>& dst,
                                const std::chrono::steady_clock::time_point a,
                                const std::chrono::steady_clock::time_point b) {
            if (stats_ != nullptr) {
                dst.fetch_add(
                    static_cast<std::uint64_t>(
                        std::chrono::duration_cast<std::chrono::microseconds>(b - a).count()),
                    std::memory_order_relaxed);
            }
        };
        const auto parse_begin = std::chrono::steady_clock::now();

        if (eu07::scene::detail::shouldStreamFlatSourceFile(resolved)) {
            const eu07::scene::detail::FlatFileKind kind =
                eu07::scene::detail::scanFlatFileKindStreaming(resolved);
            if (kind == eu07::scene::detail::FlatFileKind::Models) {
                PackFileCacheEntry entry;
                eu07::scene::detail::streamFlatFileModels(
                    resolved, [&](const ParsedNodeModel& parsed) {
                        entry.base_models.push_back(bakeModel(parsed));
                    });
                const auto parse_end = std::chrono::steady_clock::now();
                if (stats_ != nullptr) {
                    add_us(stats_->process_us, parse_begin, parse_end);
                    stats_->parse_us.fetch_add(
                        static_cast<std::uint64_t>(
                            std::chrono::duration_cast<std::chrono::microseconds>(
                                parse_end - parse_begin)
                                .count()),
                        std::memory_order_relaxed);
                }
                return entry;
            }
            if (kind == eu07::scene::detail::FlatFileKind::Includes) {
                if (const std::optional<SceneDocument> doc =
                        eu07::scene::detail::buildFlatIncludesDocumentStreaming(resolved)) {
                    const auto t_proc = std::chrono::steady_clock::now();
                    PackFileCacheEntry entry = makePackFileCacheEntry(*doc, max_threads_);
                    const auto parse_end = std::chrono::steady_clock::now();
                    if (stats_ != nullptr) {
                        add_us(stats_->process_us, parse_begin, t_proc);
                        add_us(stats_->makeentry_us, t_proc, parse_end);
                        stats_->parse_us.fetch_add(
                            static_cast<std::uint64_t>(
                                std::chrono::duration_cast<std::chrono::microseconds>(
                                    parse_end - parse_begin)
                                    .count()),
                            std::memory_order_relaxed);
                    }
                    return entry;
                }
            }
            throw std::runtime_error(
                "EU7: duzy plik nie-plaski — nie mozna zaladowac calego do RAM: " +
                resolved.string());
        }

        eu07::RawFile raw = readRawFile(resolved);
        SceneProcessOptions options;
        options.expandIncludes = false;
        SceneDocument document;
        if (std::optional<SceneDocument> flat =
                eu07::scene::detail::tryProcessFlatSceneFile(raw, scenery_root_)) {
            document = std::move(*flat);
            const auto t_proc = std::chrono::steady_clock::now();
            if (stats_ != nullptr) {
                add_us(stats_->process_us, parse_begin, t_proc);
            }
        } else {
            const ParseResult parsed = parseRawFile(std::move(raw));
            const auto t_tok = std::chrono::steady_clock::now();
            document = processScene(parsed, scenery_root_, options).document;
            const auto t_proc = std::chrono::steady_clock::now();
            if (stats_ != nullptr) {
                add_us(stats_->tokenize_us, parse_begin, t_tok);
                add_us(stats_->process_us, t_tok, t_proc);
            }
        }
        const auto t_proc = std::chrono::steady_clock::now();
        PackFileCacheEntry entry = makePackFileCacheEntry(document, max_threads_);
        const auto parse_end = std::chrono::steady_clock::now();
        if (stats_ != nullptr) {
            add_us(stats_->makeentry_us, t_proc, parse_end);
            stats_->parse_us.fetch_add(
                static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(parse_end - parse_begin)
                        .count()),
                std::memory_order_relaxed);
        }
        return entry;
    }

    // Lock-light cache probe used by the placement hot path. A hit proves the
    // file exists and is parsed, so the caller can skip the per-placement
    // filesystem::exists() syscall and the parse gate entirely. References into
    // cache_ stay valid because entries are never erased (node-stable map).
    // Fully resolved include reference: the normalized path plus the parsed cache
    // entry (or a "missing" marker). Caching this per (currentRelativeFile,
    // fileToken) collapses the flora placement hot path (~243k repeats of the same
    // few .inc templates) into a single map lookup behind one shared lock — instead
    // of re-running resolveIncludeSourcePath (which stats the disk), a separate
    // exists() syscall, and the parse-cache probe on every placement.
    struct ResolvedInclude {
        std::filesystem::path path;
        std::string relative;  // relativeSceneryFile(scenery_root_, path), memoized
        const PackFileCacheEntry* entry = nullptr;
        bool missing = false;
    };

    // Returns a stable reference into resolve_cache_ (node-stable map, never erased),
    // so callers may hold it freely. Resolution and parsing are deterministic for a
    // fixed scenery on disk, so memoizing is safe.
    [[nodiscard]] const ResolvedInclude& resolveIncludeCached(
        const std::string& currentRelativeFile, const std::string& fileToken) {
        std::string key;
        key.reserve(currentRelativeFile.size() + fileToken.size() + 1);
        key.append(currentRelativeFile);
        key.push_back('\x1f');
        key.append(fileToken);
        {
            std::shared_lock read_lock(resolve_cache_mutex_);
            if (const auto found = resolve_cache_.find(key); found != resolve_cache_.end()) {
                return found->second;
            }
        }
        ResolvedInclude resolved;
        resolved.path = eu07::scene::detail::resolveIncludeSourcePath(
                            scenery_root_, currentRelativeFile, fileToken)
                            .lexically_normal();
        if (const PackFileCacheEntry* cached = tryGetCached(resolved.path)) {
            resolved.entry = cached;
        } else if (!std::filesystem::exists(resolved.path)) {
            resolved.missing = true;
        } else {
            resolved.entry = &loadCached(resolved.path);
        }
        if (!resolved.missing) {
            resolved.relative =
                eu07::scene::detail::relativeSceneryFile(scenery_root_, resolved.path);
        }
        std::unique_lock write_lock(resolve_cache_mutex_);
        const auto [it, inserted] = resolve_cache_.emplace(std::move(key), std::move(resolved));
        return it->second;
    }

    [[nodiscard]] const PackFileCacheEntry* tryGetCached(const std::filesystem::path& resolved) {
        if (session_cache_ != nullptr) {
            return session_cache_->tryPackEntry(resolved);
        }
        const std::string key = resolved.generic_string();
        std::shared_lock read_lock(cache_mutex_);
        if (const auto found = cache_.find(key); found != cache_.end()) {
            return &found->second;
        }
        return nullptr;
    }

    [[nodiscard]] const PackFileCacheEntry& loadCached(const std::filesystem::path& resolved) {
        if (session_cache_ != nullptr) {
            if (const PackFileCacheEntry* cached = session_cache_->tryPackEntry(resolved)) {
                noteCacheHit();
                return *cached;
            }
            const ComposeConcurrencyGate::Scope gate_scope = gate_.scoped();
            if (stats_ != nullptr) {
                stats_->diag_gate_active.store(gate_.activeSlots(), std::memory_order_relaxed);
                stats_->diag_gate_waiting.store(gate_.waiting(), std::memory_order_relaxed);
            }
            if (const PackFileCacheEntry* cached = session_cache_->tryPackEntry(resolved)) {
                noteCacheHit();
                return *cached;
            }
            if (stats_ != nullptr) {
                stats_->diag_workers_parse.fetch_add(1, std::memory_order_relaxed);
            }
            const PackFileCacheEntry& built =
                session_cache_->packEntryFor(resolved, scenery_root_, stats_);
            if (stats_ != nullptr) {
                stats_->diag_workers_parse.fetch_sub(1, std::memory_order_relaxed);
            }
            noteCacheHit();
            return built;
        }

        const std::string key = resolved.generic_string();
        {
            std::shared_lock read_lock(cache_mutex_);
            if (const auto found = cache_.find(key); found != cache_.end()) {
                noteCacheHit();
                return found->second;
            }
        }

        const ComposeConcurrencyGate::Scope gate_scope = gate_.scoped();
        if (stats_ != nullptr) {
            stats_->diag_gate_active.store(gate_.activeSlots(), std::memory_order_relaxed);
            stats_->diag_gate_waiting.store(gate_.waiting(), std::memory_order_relaxed);
        }
        {
            std::shared_lock read_lock(cache_mutex_);
            if (const auto found = cache_.find(key); found != cache_.end()) {
                noteCacheHit();
                return found->second;
            }
        }

        if (stats_ != nullptr) {
            stats_->diag_workers_parse.fetch_add(1, std::memory_order_relaxed);
        }
        PackFileCacheEntry entry = buildCacheEntry(resolved);
        if (stats_ != nullptr) {
            stats_->diag_workers_parse.fetch_sub(1, std::memory_order_relaxed);
        }

        std::unique_lock write_lock(cache_mutex_);
        const auto [it, inserted] = cache_.emplace(key, std::move(entry));
        if (!inserted) {
            noteCacheHit();
        } else {
            noteCacheMiss();
        }
        if (stats_ != nullptr) {
            stats_->unique_files_cached.store(cache_.size(), std::memory_order_relaxed);
        }
        return it->second;
    }

    void noteCacheHit() {
        cache_hits_.fetch_add(1, std::memory_order_relaxed);
        if (stats_ != nullptr) {
            stats_->cache_hits.fetch_add(1, std::memory_order_relaxed);
        }
    }

    void noteCacheMiss() {
        cache_misses_.fetch_add(1, std::memory_order_relaxed);
        if (stats_ != nullptr) {
            stats_->cache_misses.fetch_add(1, std::memory_order_relaxed);
        }
    }

    // Reports the file currently entering compose (no counter mutation), so the
    // active scenery file is visible even before its models are instantiated.
    // Lightly throttled to avoid flooding on dense include trees.
    void reportActiveFile(const std::filesystem::path& file) {
        if (progress_ == nullptr || progress_->on_progress == nullptr) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard lock(progress_mutex_);
        const bool timed =
            !progress_->reported_once ||
            std::chrono::duration_cast<std::chrono::milliseconds>(now - progress_->last_report)
                    .count() >= 250;
        if (!timed) {
            return;
        }
        progress_->reported_once = true;
        progress_->last_report = now;
        progress_->on_progress(
            file,
            stats_ != nullptr ? stats_->files_visited.load(std::memory_order_relaxed)
                              : files_visited_.load(std::memory_order_relaxed),
            stats_ != nullptr ? stats_->models_emitted.load(std::memory_order_relaxed)
                              : models_total_.load(std::memory_order_relaxed));
    }

    void reportProgress(const std::filesystem::path& file, const std::size_t local_models) {
        const std::size_t files_visited = files_visited_.fetch_add(1, std::memory_order_relaxed) + 1;
        const std::size_t models_total =
            models_total_.fetch_add(local_models, std::memory_order_relaxed) + local_models;

        if (stats_ != nullptr) {
            stats_->files_visited.store(files_visited, std::memory_order_relaxed);
            stats_->models_emitted.store(models_total, std::memory_order_relaxed);
        }

        if (progress_ == nullptr || progress_->on_progress == nullptr) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        const bool periodic = (files_visited % 10) == 0;
        const bool timed =
            !progress_->reported_once ||
            std::chrono::duration_cast<std::chrono::milliseconds>(now - progress_->last_report)
                    .count() >= 100;
        if (!periodic && !timed) {
            return;
        }

        std::lock_guard lock(progress_mutex_);
        progress_->files_visited = files_visited;
        progress_->models_total = models_total;
        progress_->reported_once = true;
        progress_->last_report = now;
        progress_->on_progress(file, files_visited, models_total);
    }

    unsigned max_threads_;
    ComposeConcurrencyGate gate_;
    PackComposeProgress* progress_;
    std::filesystem::path scenery_root_;
    PackComposeStats* stats_ = nullptr;
    BakeParseSessionCache* session_cache_ = nullptr;
    std::shared_mutex cache_mutex_;
    std::unordered_map<std::string, PackFileCacheEntry> cache_;
    std::shared_mutex resolve_cache_mutex_;
    // Memoized include-path resolution keyed on (currentRelativeFile, fileToken).
    // The flora placement hot path re-resolves the same .inc template hundreds of
    // thousands of times; resolveIncludeSourcePath stats the filesystem and builds
    // paths, so caching avoids ~243k redundant syscalls + allocations. Resolution
    // is deterministic for a fixed scenery on disk, so memoizing is safe.
    std::unordered_map<std::string, ResolvedInclude> resolve_cache_;
    std::mutex progress_mutex_;
    std::atomic<std::size_t> files_visited_ { 0 };
    std::atomic<std::size_t> models_total_ { 0 };
    std::atomic<std::size_t> cache_hits_ { 0 };
    std::atomic<std::size_t> cache_misses_ { 0 };
};

} // namespace detail

using detail::BakeParseSessionCache;
using detail::makePackFileCacheEntry;
using detail::printBakeParseSessionStats;

[[nodiscard]] inline std::size_t composePackModels(
    const std::filesystem::path& path,
    const std::filesystem::path& sceneryRoot,
    const std::string& relativeFile,
    const PackComposeOptions& options,
    const std::vector<std::filesystem::path>& includeStack = {},
    const runtime::TransformContext& includePrefix = {},
    const std::span<const std::string> includeParameters = {}) {
    const auto compose_begin = std::chrono::steady_clock::now();
    detail::PackComposeSession session(
        options.max_threads,
        options.progress,
        sceneryRoot,
        options.stats,
        options.session_cache);
    if (options.session_cache == nullptr && options.root_seed != nullptr) {
        session.seedCache(options.root_seed->path, options.root_seed->entry);
    }

    if (options.pack_batches == nullptr) {
        throw std::invalid_argument("EU7 PACK: pack_batches jest wymagane");
    }

    const unsigned thread_count = detail::resolveComposeThreadCount(options.max_threads);
    if (options.stats != nullptr) {
        options.stats->threads.store(thread_count, std::memory_order_relaxed);
    }

    const auto finalize_begin = std::chrono::steady_clock::now();
    std::size_t models_total = 0;

    if (options.low_memory) {
        detail::PackBatchDirectBuilder direct_builder(
            options.pack_batches, thread_count > 1);
        std::function<void()> flush_after_file;
        if (options.on_pack_models_flush) {
            if (options.pack_flush_per_file) {
                direct_builder.set_flush(0, options.on_pack_models_flush);
                flush_after_file = [&direct_builder]() { direct_builder.flushAll(); };
            } else if (options.pack_flush_threshold != 0) {
                direct_builder.set_flush(
                    options.pack_flush_threshold, options.on_pack_models_flush);
            }
        }
        detail::PackModelBatchSink sink {
            [&direct_builder, stats = options.stats](
                std::vector<runtime::RuntimeModelInstance>&& batch) {
                const auto sink_begin = std::chrono::steady_clock::now();
                direct_builder.add(std::move(batch), stats);
                if (stats != nullptr) {
                    stats->sink_us.fetch_add(
                        static_cast<std::uint64_t>(
                            std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::steady_clock::now() - sink_begin)
                                .count()),
                        std::memory_order_relaxed);
                }
            } };
        session.runCompose(
            path,
            relativeFile,
            includeStack,
            includePrefix,
            includeParameters,
            sink,
            options.pack_flush_per_file && flush_after_file ? &flush_after_file : nullptr);
        direct_builder.finalizeSort();
        models_total = direct_builder.modelsTotal();
    } else {
        std::vector<detail::PackSectionAccumulator> thread_accumulators(thread_count);
        std::atomic<unsigned> next_worker_id { 0 };
        detail::PackModelBatchSink sink {
            [&thread_accumulators, &next_worker_id, stats = options.stats](
                std::vector<runtime::RuntimeModelInstance>&& batch) {
                thread_local unsigned worker_index = static_cast<unsigned>(-1);
                if (worker_index == static_cast<unsigned>(-1)) {
                    worker_index = next_worker_id.fetch_add(1, std::memory_order_relaxed);
                }
                const auto sink_begin = std::chrono::steady_clock::now();
                if (worker_index < thread_accumulators.size()) {
                    thread_accumulators[worker_index].add(std::move(batch), stats);
                } else {
                    thread_accumulators[worker_index % thread_accumulators.size()].add(
                        std::move(batch), stats);
                }
                if (stats != nullptr) {
                    stats->sink_us.fetch_add(
                        static_cast<std::uint64_t>(
                            std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::steady_clock::now() - sink_begin)
                                .count()),
                        std::memory_order_relaxed);
                }
            } };
        session.runCompose(
            path, relativeFile, includeStack, includePrefix, includeParameters, sink);

        detail::PackSectionAccumulator merged;
        for (detail::PackSectionAccumulator& accumulator : thread_accumulators) {
            merged.mergeFrom(std::move(accumulator));
        }
        *options.pack_batches = merged.finalize();
        models_total = merged.modelsTotal();
    }

    const auto compose_end = std::chrono::steady_clock::now();

    if (options.stats != nullptr) {
        options.stats->sections.store(options.pack_batches->size(), std::memory_order_relaxed);
        options.stats->finalize_us.fetch_add(
            static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                           compose_end - finalize_begin)
                                           .count()),
            std::memory_order_relaxed);
        options.stats->compose_us.store(
            static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(compose_end - compose_begin)
                    .count()),
            std::memory_order_relaxed);
        if (options.stats->models_emitted.load(std::memory_order_relaxed) == 0) {
            options.stats->models_emitted.store(models_total, std::memory_order_relaxed);
        }
    }
    return models_total;
}

} // namespace eu07::scene::bake
