#pragma once

// Compose MODL z calego drzewa INCL → world-space do chunku PACK (v7).
// Cache parsowania per plik + kolejka zadan z pulą watkow.

#include <eu07/parser.hpp>
#include <eu07/scene/binary/runtime_codec.hpp>
#include <eu07/scene/bake/model.hpp>
#include <eu07/scene/include_placement.hpp>
#include <eu07/scene/include_resolve.hpp>
#include <eu07/scene/processor.hpp>
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
    std::atomic<std::uint64_t> instantiate_us { 0 };
    std::atomic<std::uint64_t> finalize_us { 0 };
    std::atomic<std::uint64_t> compose_us { 0 };
    std::atomic<unsigned> threads { 0 };
    std::atomic<std::size_t> sections { 0 };
    std::atomic<std::size_t> chain_fold_steps { 0 };
    std::atomic<std::size_t> chain_fold_hops { 0 };
    std::atomic<std::size_t> inc_includes_inlined { 0 };
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
    std::uint64_t instantiate_us = 0;
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
    out.instantiate_us = stats.instantiate_us.load(std::memory_order_relaxed);
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

struct PackComposeOptions {
    PackComposeProgress* progress = nullptr;
    PackComposeStats* stats = nullptr;
    unsigned max_threads = 0;
    // Root juz sparsowany w bake_tree — bez drugiego parseFile.
    const PackComposeSeed* root_seed = nullptr;
    // Wynik: sekcje 1 km (serializacja PACK w writeRuntimeModule).
    std::vector<binary::codec::ModelSectionBatch>* pack_batches = nullptr;
};

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
        cv_.wait(lock, [this]() { return active_slots_ < max_slots_; });
        ++active_slots_;
    }

    void release() {
        {
            std::lock_guard lock(mutex_);
            --active_slots_;
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

private:
    unsigned max_slots_;
    unsigned active_slots_ = 0;
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

[[nodiscard]] inline PackFileCacheEntry makePackFileCacheEntry(const SceneDocument& document) {
    PackFileCacheEntry entry;
    entry.includes = document.include;
    if (const std::optional<IncludePlacement> placement = extractIncludePlacement(document)) {
        entry.placement = *placement;
    }
    entry.base_models.reserve(document.nodeModel.size());
    for (const ParsedNodeModel& parsedModel : document.nodeModel) {
        entry.base_models.push_back(bakeModel(parsedModel));
    }
    return entry;
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
        PackComposeStats* stats = nullptr)
        : max_threads_(resolveComposeThreadCount(max_threads))
        , gate_(max_threads_)
        , progress_(progress)
        , scenery_root_(scenery_root)
        , stats_(stats) {}

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
        PackModelBatchSink& sink) {
        std::mutex queue_mutex;
        std::deque<PackComposeWorkItem> queue;
        std::condition_variable queue_cv;
        std::condition_variable done_cv;
        std::atomic<std::size_t> pending_tasks { 0 };
        std::atomic<bool> stop_workers { false };
        std::exception_ptr first_error;
        std::mutex error_mutex;

        auto submit = [&](PackComposeWorkItem item) {
            pending_tasks.fetch_add(1, std::memory_order_acq_rel);
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
            if (pending_tasks.fetch_sub(1, std::memory_order_acq_rel) == 1) {
                done_cv.notify_all();
            }
        };

        auto process_one = [&](PackComposeWorkItem work) {
            bool folded_hop = false;
            std::size_t task_fold_steps = 0;
            while (true) {
                const std::filesystem::path resolved = work.path.lexically_normal();
                if (eu07::scene::detail::isIncludeCycle(resolved, work.includeStack)) {
                    if (stats_ != nullptr) {
                        stats_->cycles_skipped.fetch_add(1, std::memory_order_relaxed);
                    }
                    return;
                }

                if (!std::filesystem::exists(resolved)) {
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
                        const std::filesystem::path inc_path =
                            eu07::scene::detail::resolveIncludeSourcePath(
                                scenery_root_, relative_file, step->file)
                                .lexically_normal();
                        if (is_inc_walk_cycle(inc_path, suffix)) {
                            if (stats_ != nullptr) {
                                stats_->cycles_skipped.fetch_add(1, std::memory_order_relaxed);
                            }
                            return;
                        }
                        if (!std::filesystem::exists(inc_path)) {
                            if (stats_ != nullptr) {
                                stats_->missing_skipped.fetch_add(1, std::memory_order_relaxed);
                            }
                            return;
                        }

                        const PackFileCacheEntry& inc_entry = loadCached(inc_path);
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

                        const std::string inc_relative =
                            eu07::scene::detail::relativeSceneryFile(scenery_root_, inc_path);
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
                        return;
                    }
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
                return;
            }
        };

        auto worker_loop = [&]() {
            while (true) {
                PackComposeWorkItem work;
                {
                    std::unique_lock lock(queue_mutex);
                    queue_cv.wait(lock, [&]() {
                        return stop_workers.load(std::memory_order_acquire) || !queue.empty();
                    });
                    if (stop_workers.load(std::memory_order_acquire) && queue.empty()) {
                        return;
                    }
                    if (queue.empty()) {
                        continue;
                    }
                    work = std::move(queue.front());
                    queue.pop_front();
                }

                try {
                    process_one(std::move(work));
                } catch (...) {
                    {
                        std::lock_guard lock(error_mutex);
                        if (!first_error) {
                            first_error = std::current_exception();
                        }
                    }
                    stop_workers.store(true, std::memory_order_release);
                    queue_cv.notify_all();
                }
                finish_task();
            }
        };

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
            done_cv.wait(lock, [&]() {
                return pending_tasks.load(std::memory_order_acquire) == 0 || first_error != nullptr;
            });
        }

        stop_workers.store(true, std::memory_order_release);
        {
            std::lock_guard lock(queue_mutex);
            while (!queue.empty()) {
                queue.pop_front();
                finish_task();
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
        const auto parse_begin = std::chrono::steady_clock::now();
        const ParseResult parsed = parseFile(resolved);
        SceneProcessOptions options;
        options.expandIncludes = false;
        const SceneDocument document =
            processScene(parsed, scenery_root_, options).document;
        PackFileCacheEntry entry = makePackFileCacheEntry(document);
        if (stats_ != nullptr) {
            const auto parse_end = std::chrono::steady_clock::now();
            stats_->parse_us.fetch_add(
                static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(parse_end - parse_begin)
                        .count()),
                std::memory_order_relaxed);
        }
        return entry;
    }

    [[nodiscard]] const PackFileCacheEntry& loadCached(const std::filesystem::path& resolved) {
        const std::string key = resolved.generic_string();
        {
            std::shared_lock read_lock(cache_mutex_);
            if (const auto found = cache_.find(key); found != cache_.end()) {
                noteCacheHit();
                return found->second;
            }
        }

        const ComposeConcurrencyGate::Scope gate_scope = gate_.scoped();
        {
            std::shared_lock read_lock(cache_mutex_);
            if (const auto found = cache_.find(key); found != cache_.end()) {
                noteCacheHit();
                return found->second;
            }
        }

        PackFileCacheEntry entry = buildCacheEntry(resolved);

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
    std::shared_mutex cache_mutex_;
    std::unordered_map<std::string, PackFileCacheEntry> cache_;
    std::mutex progress_mutex_;
    std::atomic<std::size_t> files_visited_ { 0 };
    std::atomic<std::size_t> models_total_ { 0 };
    std::atomic<std::size_t> cache_hits_ { 0 };
    std::atomic<std::size_t> cache_misses_ { 0 };
};

} // namespace detail

using detail::makePackFileCacheEntry;

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
        options.max_threads, options.progress, sceneryRoot, options.stats);
    if (options.root_seed != nullptr) {
        session.seedCache(options.root_seed->path, options.root_seed->entry);
    }

    if (options.pack_batches == nullptr) {
        throw std::invalid_argument("EU7 PACK: pack_batches jest wymagane");
    }

    const unsigned thread_count = detail::resolveComposeThreadCount(options.max_threads);
    if (options.stats != nullptr) {
        options.stats->threads.store(thread_count, std::memory_order_relaxed);
    }
    std::vector<detail::PackSectionAccumulator> thread_accumulators(thread_count);
    std::atomic<unsigned> next_worker_id { 0 };

    detail::PackModelBatchSink sink {
        [&thread_accumulators, &next_worker_id, stats = options.stats](
            std::vector<runtime::RuntimeModelInstance>&& batch) {
            thread_local unsigned worker_index = static_cast<unsigned>(-1);
            if (worker_index == static_cast<unsigned>(-1)) {
                worker_index = next_worker_id.fetch_add(1, std::memory_order_relaxed);
            }
            if (worker_index < thread_accumulators.size()) {
                thread_accumulators[worker_index].add(std::move(batch), stats);
            } else {
                thread_accumulators[worker_index % thread_accumulators.size()].add(
                    std::move(batch), stats);
            }
        } };
    session.runCompose(path, relativeFile, includeStack, includePrefix, includeParameters, sink);

    const auto finalize_begin = std::chrono::steady_clock::now();
    detail::PackSectionAccumulator merged;
    for (detail::PackSectionAccumulator& accumulator : thread_accumulators) {
        merged.mergeFrom(std::move(accumulator));
    }
    *options.pack_batches = merged.finalize();
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
            options.stats->models_emitted.store(merged.modelsTotal(), std::memory_order_relaxed);
        }
    }
    return merged.modelsTotal();
}

} // namespace eu07::scene::bake
