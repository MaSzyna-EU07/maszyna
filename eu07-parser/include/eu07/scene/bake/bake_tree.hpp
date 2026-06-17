#pragma once



// Rekurencyjny bake: kazdy plik tekstowy → wlasny .eu7, dzieci z INCL tez.

// Znaleziony include → od razu na kolejke i wolny watek (bez osobnej fazy odkrywania).



#include <eu07/parser.hpp>

#include <eu07/scene/bake/compose_pack_models.hpp>
#include <eu07/scene/bake/module.hpp>
#include <eu07/scene/bake/pack_model_spool.hpp>
#include <eu07/scene/bake/streaming_terrain.hpp>

#include <eu07/scene/binary/runtime_module.hpp>

#include <eu07/scene/include_resolve.hpp>

#include <eu07/scene/processor.hpp>



#include <atomic>

#include <condition_variable>

#include <cstdint>

#include <deque>

#include <exception>

#include <filesystem>

#include <functional>

#include <iostream>

#include <memory>

#include <mutex>

#include <optional>

#include <stdexcept>

#include <string>

#include <thread>

#include <unordered_set>

#include <utility>

#include <vector>



namespace eu07::scene::bake {



struct BakeTreeStats {

    std::size_t modulesBaked = 0;

    std::vector<std::filesystem::path> writtenPaths;

    bool has_pack_diagnostics = false;
    PackComposeStatsSnapshot pack_compose;
    binary::PackWriteStats pack_write;
};



enum class BakeProgressPhase : std::uint8_t {

    Starting,

    Start,

    PackModels,

    PackCompose,

    PackComposeDone,

    Bake,

    Done,

};



struct BakeProgress {

    BakeProgressPhase phase = BakeProgressPhase::Bake;

    std::size_t current = 0;

    std::size_t total = 0;

    std::filesystem::path path;

    unsigned threads = 1;

    const PackComposeStats* pack_stats = nullptr;

};



using BakeProgressCallback = std::function<void(const BakeProgress&)>;



struct BakeTreeOptions {

    unsigned maxThreads = 0;

    BakeProgressCallback onProgress;

    // Hook wywolywany po zbakowaniu kazdego modulu (przed/zamiast zapisu .eu7).
    // Dla roota packBatches != nullptr (sploszczone modele PACK). isRoot == true
    // tylko dla pliku-korzenia drzewa bake. Moze byc wolany z wielu watkow.
    std::function<void(
        const RuntimeModule& module,
        const std::filesystem::path& textPath,
        bool isRoot,
        const std::vector<binary::codec::ModelSectionBatch>* packBatches,
        ShapeSpoolFile* shapeSpool)>
        onModuleBaked;

    // Gdy true: pomijamy zapis legacy .eu7 (writeRuntimeModule).
    bool skipLegacyWrite = false;

    // eu7v2 PACK: modele dzieci sa juz w root PACK — nie duplikuj INST w .eu7v2 dziecka.
    bool omitChildModuleModels = false;

    // Max rownoleglych pelnych parseFile/processScene w session cache (0 = auto: 1).
    // Ogranicza peak RAM gdy wiele ciezkich .scm startuje naraz (Braniewo / linie).
    unsigned maxConcurrentParses = 0;

    // Mniej RAM, wolniej: 1 watek bake/compose, agresywne zwalnianie cache PACK.
    bool lowMemoryMode = false;

    // Pliki wieksze niz X MB: parse/bake tylko pojedynczo (0 = wylaczone).
    unsigned heavyParseThresholdMb = 0;

    // PACK compose: spłucz sekcje modeli poza RAM gdy przekroczony prog (lowMemoryMode).
    std::size_t packFlushThreshold = 0;
    bool packFlushPerFile = false;
    std::function<void(std::vector<runtime::RuntimeModelInstance>&&)> onPackModelsFlush;

};

[[nodiscard]] inline bool useIncrementalRootPack(const BakeTreeOptions& options) {
    return options.lowMemoryMode;
}



struct BakeTreeContext {

    std::filesystem::path sceneryRoot;

    std::string currentRelativeFile;

};



namespace detail {



struct BakeWorkItem {

    std::filesystem::path path;

    std::string relativeFile;

    std::vector<std::filesystem::path> includeStack;

};



struct BakePoolState {

    std::mutex queueMutex;

    std::condition_variable queueCv;

    std::deque<BakeWorkItem> queue;



    std::mutex seenMutex;

    std::unordered_set<std::string> seen;



    std::atomic<std::size_t> inFlight{0};

    std::atomic<std::size_t> completed{0};

    std::atomic<bool> stop{false};



    std::mutex statsMutex;

    std::mutex rootMutex;

    std::mutex progressMutex;

    std::mutex errorMutex;

    std::exception_ptr firstError;

};



[[nodiscard]] inline BakeTreeContext makeBakeTreeContext(const std::filesystem::path& inputPath) {

    const std::filesystem::path resolved = inputPath.lexically_normal();

    BakeTreeContext context;



    if (const std::optional<std::filesystem::path> sceneryRoot =

            eu07::scene::detail::findSceneryRootInPath(resolved)) {

        context.sceneryRoot = *sceneryRoot;

        context.currentRelativeFile =

            eu07::scene::detail::relativeSceneryFile(context.sceneryRoot, resolved);

    } else {

        context.sceneryRoot = resolved.parent_path();

        context.currentRelativeFile = resolved.filename().generic_string();

    }



    return context;

}



[[nodiscard]] inline std::string canonicalPathKey(const std::filesystem::path& path) {

    return path.lexically_normal().generic_string();

}



[[nodiscard]] inline unsigned defaultPoolThreadCount() {

    const unsigned hw =

        std::thread::hardware_concurrency() == 0 ? 4u : std::thread::hardware_concurrency();

    return std::max(1u, hw);

}



[[nodiscard]] inline unsigned resolvePoolThreadCount(const unsigned maxThreads) {

    if (maxThreads == 0) {

        return defaultPoolThreadCount();

    }

    return std::max(1u, maxThreads);

}

[[nodiscard]] inline unsigned resolveParseConcurrencyLimit(const unsigned maxConcurrentParses) {
    if (maxConcurrentParses != 0) {
        return std::max(1u, maxConcurrentParses);
    }
    return 1u;
}

[[nodiscard]] inline std::size_t heavyParseThresholdBytes(const unsigned threshold_mb) {
    return threshold_mb == 0 ? 0u : static_cast<std::size_t>(threshold_mb) * 1024u * 1024u;
}



inline void recordError(BakePoolState& pool) {

    std::lock_guard lock(pool.errorMutex);

    if (!pool.firstError) {

        pool.firstError = std::current_exception();

    }

    pool.stop.store(true, std::memory_order_release);

    pool.queueCv.notify_all();

}



inline void enqueueWork(BakePoolState& pool, BakeWorkItem item) {

    const std::string key = canonicalPathKey(item.path);

    {

        std::lock_guard lock(pool.seenMutex);

        if (!pool.seen.insert(key).second) {

            return;

        }

    }



    {

        std::lock_guard lock(pool.queueMutex);

        pool.queue.push_back(std::move(item));

    }

    pool.queueCv.notify_one();

}



inline void bakeWorkItem(

    const BakeWorkItem& item,

    const BakeTreeContext& context,

    const std::string& rootKey,

    BakePoolState& pool,

    BakeTreeStats* stats,

    RuntimeModule& rootModuleOut,

    const BakeTreeOptions& options,

    const unsigned threadCount,

    BakeParseSessionCache& sessionCache) {

    if (pool.stop.load(std::memory_order_acquire)) {

        return;

    }



    if (eu07::scene::detail::isIncludeCycle(item.path, item.includeStack)) {

        throw std::runtime_error("EU7 bake: cykliczny include: " + item.path.string());

    }



    if (!std::filesystem::exists(item.path)) {

        if (item.includeStack.empty()) {

            throw std::runtime_error("EU7 bake: brak pliku: " + item.path.string());

        }

        return;

    }



    pool.inFlight.fetch_add(1, std::memory_order_relaxed);

    bool models_pending_pack = false;

    try {

        RuntimeModule module;
        const bool emitPackModels = canonicalPathKey(item.path) == rootKey;
        std::vector<ParsedInclude> deferred_child_includes;
        eu07::scene::detail::FlatFileKind flat_kind =
            eu07::scene::detail::FlatFileKind::None;
        std::unique_ptr<ShapeSpoolFile> triangle_shape_spool;
        {
            if (options.onProgress) {
                std::lock_guard lock(pool.progressMutex);
                options.onProgress(
                    BakeProgress{
                        BakeProgressPhase::Start,
                        pool.completed.load(std::memory_order_relaxed),
                        0,
                        item.path,
                        threadCount,
                    });
            }

            const bool prefer_lightweight_document =
                useIncrementalRootPack(options) ||
                eu07::scene::detail::shouldStreamFlatSourceFile(item.path);
            if (prefer_lightweight_document) {
                flat_kind = eu07::scene::detail::scanFlatFileKindStreaming(item.path);
            }

            const bool try_triangle_terrain_stream =
                prefer_lightweight_document &&
                flat_kind == eu07::scene::detail::FlatFileKind::None &&
                eu07::scene::detail::shouldStreamFlatSourceFile(item.path);

            bool used_triangle_terrain_stream = false;
            SceneDocument empty_document;
            const SceneDocument* document_ptr = nullptr;

            if (try_triangle_terrain_stream) {
                if (options.lowMemoryMode) {
                    const std::size_t path_hash = std::hash<std::string> {}(
                        item.path.lexically_normal().generic_string());
                    triangle_shape_spool = std::make_unique<ShapeSpoolFile>(
                        std::filesystem::temp_directory_path() /
                        ("eu7v2_shape_" + std::to_string(path_hash) + ".bin"));
                }
                if (detail::streamBakeTriangleTerrain(
                        item.path, module, triangle_shape_spool.get(), threadCount)) {
                    used_triangle_terrain_stream = true;
                    document_ptr = &empty_document;
                }
            }

            if (!used_triangle_terrain_stream) {
                const SceneDocument& document =
                    (prefer_lightweight_document &&
                     (flat_kind == eu07::scene::detail::FlatFileKind::Models ||
                      flat_kind == eu07::scene::detail::FlatFileKind::Includes))
                        ? sessionCache.documentForPackCompose(
                              item.path, context.sceneryRoot, nullptr, true)
                        : sessionCache.documentFor(item.path, context.sceneryRoot, nullptr);
                document_ptr = &document;
            }

            const SceneDocument& document = *document_ptr;

            const auto enqueue_unique_inc_modules = [&](const SceneDocument& doc) {
                if (!options.onModuleBaked) {
                    return;
                }
                BakeWorkItem child;
                child.includeStack = item.includeStack;
                child.includeStack.push_back(item.path);
                for (const ParsedInclude& include : doc.include) {
                    if (!eu07::scene::detail::isIncFile(include.file)) {
                        continue;
                    }
                    child.path = eu07::scene::detail::resolveIncludeSourcePath(
                        context.sceneryRoot, item.relativeFile, include.file);
                    child.relativeFile = eu07::scene::detail::relativeSceneryFile(
                        context.sceneryRoot, child.path);
                    enqueueWork(pool, child);
                }
            };

            const auto enqueue_discovered_includes = [&](const SceneDocument& doc) {
                BakeWorkItem child;
                child.includeStack = item.includeStack;
                child.includeStack.push_back(item.path);
                for (const ParsedInclude& include : doc.include) {
                    // Placementy .inc (flora, szablony) — tylko do PACK compose,
                    // nie osobne moduly bake (3_rosl1-leo: 17k+ linii).
                    if (eu07::scene::detail::isIncFile(include.file)) {
                        continue;
                    }
                    child.path = eu07::scene::detail::resolveIncludeSourcePath(
                        context.sceneryRoot, item.relativeFile, include.file);
                    child.relativeFile = eu07::scene::detail::relativeSceneryFile(
                        context.sceneryRoot, child.path);
                    enqueueWork(pool, child);
                }
            };

            // Root PACK compose jest bardzo pamieciochlonny — nie startuj rownolegle
            // pelnych parse linii (l204/l220/…) az compose roota sie nie skonczy.
            // lowMemoryMode + spool: jeden modul = jeden plik .scm, PACK skladany per modul.
            if (emitPackModels) {
                if (useIncrementalRootPack(options)) {
                    enqueue_discovered_includes(document);
                } else {
                    deferred_child_includes = document.include;
                }
            } else {
                enqueue_discovered_includes(document);
            }
            enqueue_unique_inc_modules(document);

            const bool skipLocalModels =
                emitPackModels ||
                (options.omitChildModuleModels && isModelOnlyDocument(document));
            if (!used_triangle_terrain_stream) {
                module = bakeModule(document, {.skipLocalModels = skipLocalModels});
            }

            models_pending_pack =
                !used_triangle_terrain_stream &&
                skipLocalModels &&
                (isModelOnlyDocument(document) ||
                 flat_kind == eu07::scene::detail::FlatFileKind::Models);
            if (!used_triangle_terrain_stream) {
                sessionCache.releaseHeavyParseStorageAfterModuleBake(
                    item.path, models_pending_pack);
            }
        }

        const std::filesystem::path eu7Path =

            item.path.parent_path() / (item.path.stem().string() + ".eu7");

        const std::string key = canonicalPathKey(item.path);

        binary::WriteRuntimeModuleOptions writeOpts;
        writeOpts.emitPackModels = emitPackModels;
        std::vector<binary::codec::ModelSectionBatch> packBatches;
        if (writeOpts.emitPackModels && !useIncrementalRootPack(options)) {
            if (options.onProgress) {
                std::lock_guard lock(pool.progressMutex);
                options.onProgress(
                    BakeProgress{
                        BakeProgressPhase::PackModels,
                        pool.completed.load(std::memory_order_relaxed),
                        0,
                        item.path,
                        threadCount,
                    });
            }
            PackComposeStats pack_stats;
            PackComposeProgress pack_progress;
            if (options.onProgress) {
                pack_progress.on_progress =
                    [&](const std::filesystem::path& file,
                        const std::size_t files_visited,
                        const std::size_t models_total) {
                        std::lock_guard lock(pool.progressMutex);
                        BakeProgress progress{
                            BakeProgressPhase::PackCompose,
                            files_visited,
                            models_total,
                            file,
                            threadCount,
                        };
                        progress.pack_stats = &pack_stats;
                        options.onProgress(progress);
                    };
            }
            PackComposeOptions pack_options;
            pack_options.progress = &pack_progress;
            pack_options.stats = &pack_stats;
            pack_options.max_threads = threadCount;
            pack_options.low_memory = options.lowMemoryMode;
            pack_options.pack_flush_threshold = options.packFlushThreshold;
            pack_options.pack_flush_per_file = options.packFlushPerFile;
            pack_options.on_pack_models_flush = options.onPackModelsFlush;
            pack_options.pack_batches = &packBatches;
            pack_options.session_cache = &sessionCache;
            if (options.onProgress) {
                std::cerr << "[EU7]   PACK compose: " << threadCount << " watkow\n" << std::flush;
            }
            const std::size_t pack_model_count = composePackModels(
                item.path, context.sceneryRoot, item.relativeFile, pack_options);
            writeOpts.pack_batches = &packBatches;
            if (options.onProgress) {
                std::lock_guard lock(pool.progressMutex);
                options.onProgress(
                    BakeProgress{
                        BakeProgressPhase::PackComposeDone,
                        pack_progress.files_visited,
                        pack_model_count,
                        item.path,
                        threadCount,
                    });
                std::cerr << "[EU7]   PACK gotowe: modele=" << pack_model_count
                          << " sekcji=" << packBatches.size() << '\n'
                          << std::flush;
            }
            printPackComposeStats(pack_stats, std::cerr);
            printBakeParseSessionStats(sessionCache, std::cerr);
            if (stats != nullptr) {
                stats->has_pack_diagnostics = true;
                stats->pack_compose = snapshotPackComposeStats(pack_stats);
            }
        }

        // lowMemoryMode: jeden plik = jeden modul — PACK tylko z tego .scm, potem spool/RAM zwolnione.
        if (useIncrementalRootPack(options) && models_pending_pack && !emitPackModels) {
            if (flat_kind == eu07::scene::detail::FlatFileKind::Models &&
                options.onPackModelsFlush) {
                detail::streamFlatModelsToPackFlush(item.path, options.onPackModelsFlush);
            } else {
                std::vector<binary::codec::ModelSectionBatch> module_pack_batches;
                PackComposeOptions pack_options;
                pack_options.low_memory = true;
                pack_options.pack_flush_per_file = options.packFlushPerFile;
                pack_options.on_pack_models_flush = options.onPackModelsFlush;
                pack_options.pack_batches = &module_pack_batches;
                pack_options.session_cache = &sessionCache;
                pack_options.max_threads = 1;
                (void)composePackModels(
                    item.path, context.sceneryRoot, item.relativeFile, pack_options);
                if (options.onPackModelsFlush) {
                    std::vector<runtime::RuntimeModelInstance> tail;
                    for (binary::codec::ModelSectionBatch& batch : module_pack_batches) {
                        tail.insert(
                            tail.end(),
                            std::make_move_iterator(batch.models.begin()),
                            std::make_move_iterator(batch.models.end()));
                    }
                    if (!tail.empty()) {
                        options.onPackModelsFlush(std::move(tail));
                    }
                }
            }
            sessionCache.dropPackCacheEntry(item.path);
        }

        // Hook: pozwala silnikowi wyemitowac wlasny format (eu7v2) bezposrednio
        // z RuntimeModule, z dostepem do sploszczonych modeli PACK dla roota.
        if (options.onModuleBaked) {
            const bool root_with_pack =
                emitPackModels && !useIncrementalRootPack(options);
            const std::vector<binary::codec::ModelSectionBatch>* batchesPtr =
                root_with_pack ? &packBatches : nullptr;
            options.onModuleBaked(
                module, item.path, emitPackModels, batchesPtr, triangle_shape_spool.get());
        }

        if (emitPackModels && !deferred_child_includes.empty() &&
            !useIncrementalRootPack(options)) {
            SceneDocument stub;
            stub.include = std::move(deferred_child_includes);
            BakeWorkItem child;
            child.includeStack = item.includeStack;
            child.includeStack.push_back(item.path);
            for (const ParsedInclude& include : stub.include) {
                if (eu07::scene::detail::isIncFile(include.file)) {
                    continue;
                }
                child.path = eu07::scene::detail::resolveIncludeSourcePath(
                    context.sceneryRoot, item.relativeFile, include.file);
                child.relativeFile =
                    eu07::scene::detail::relativeSceneryFile(context.sceneryRoot, child.path);
                enqueueWork(pool, child);
            }
        }

        if (!models_pending_pack) {
            sessionCache.evictEntryIfNotNeeded(item.path);
        }

        if (!options.skipLegacyWrite) {
            binary::PackWriteStats pack_write_stats;
            if (writeOpts.emitPackModels) {
                writeOpts.pack_write_stats = &pack_write_stats;
                std::cerr << "[EU7]   zapis .eu7 (PACK)...\n" << std::flush;
            }
            binary::writeRuntimeModule(eu7Path, module, writeOpts);
            if (writeOpts.emitPackModels) {
                printPackWriteStats(pack_write_stats, std::cerr);
                if (stats != nullptr) {
                    stats->pack_write = pack_write_stats;
                }
            }
        }



        const std::size_t done = pool.completed.fetch_add(1, std::memory_order_relaxed) + 1;



        if (stats != nullptr) {

            std::lock_guard lock(pool.statsMutex);

            ++stats->modulesBaked;

            stats->writtenPaths.push_back(eu7Path);

        }



        if (key == rootKey) {

            std::lock_guard lock(pool.rootMutex);

            rootModuleOut = std::move(module);

        }



        if (options.onProgress) {

            std::lock_guard lock(pool.progressMutex);

            options.onProgress(

                BakeProgress{

                    BakeProgressPhase::Bake,

                    done,

                    0,

                    item.path,

                    threadCount,

                });

        }

    } catch (...) {

        recordError(pool);

        pool.inFlight.fetch_sub(1, std::memory_order_relaxed);

        pool.queueCv.notify_all();

        return;

    }



    pool.inFlight.fetch_sub(1, std::memory_order_relaxed);

    pool.queueCv.notify_all();

}



inline void bakePoolWorker(

    BakePoolState& pool,

    const BakeTreeContext& context,

    const std::string& rootKey,

    BakeTreeStats* stats,

    RuntimeModule& rootModuleOut,

    const BakeTreeOptions& options,

    const unsigned threadCount,

    BakeParseSessionCache& sessionCache) {

    while (true) {

        BakeWorkItem item;

        {

            std::unique_lock lock(pool.queueMutex);

            pool.queueCv.wait(lock, [&]() {

                return pool.stop.load(std::memory_order_acquire) || !pool.queue.empty() ||

                       pool.inFlight.load(std::memory_order_relaxed) == 0;

            });



            if (pool.stop.load(std::memory_order_acquire)) {

                return;

            }



            if (pool.queue.empty()) {

                if (pool.inFlight.load(std::memory_order_relaxed) == 0) {

                    return;

                }

                continue;

            }



            item = std::move(pool.queue.front());

            pool.queue.pop_front();

        }



        try {

            bakeWorkItem(

                item, context, rootKey, pool, stats, rootModuleOut, options, threadCount, sessionCache);

        } catch (...) {

            recordError(pool);

        }



        if (pool.stop.load(std::memory_order_acquire)) {

            return;

        }

    }

}



inline void runBakePool(

    const std::filesystem::path& rootSourcePath,

    const BakeTreeContext& context,

    BakeTreeStats* stats,

    RuntimeModule& rootModuleOut,

    const BakeTreeOptions& options) {

    const std::string rootKey = canonicalPathKey(rootSourcePath.lexically_normal());

    const unsigned threadCount = resolvePoolThreadCount(options.maxThreads);

    if (options.onProgress) {
        options.onProgress(
            BakeProgress{
                BakeProgressPhase::Starting,
                0,
                0,
                rootSourcePath,
                threadCount,
            });
    }

    BakePoolState pool;

    BakeParseSessionCache sessionCache {
        detail::resolveParseConcurrencyLimit(options.maxConcurrentParses),
        options.lowMemoryMode,
        detail::heavyParseThresholdBytes(options.heavyParseThresholdMb) };

    enqueueWork(

        pool,

        BakeWorkItem{

            rootSourcePath.lexically_normal(),

            context.currentRelativeFile,

            {},

        });



    std::vector<std::thread> workers;

    workers.reserve(threadCount);

    for (unsigned i = 0; i < threadCount; ++i) {

        workers.emplace_back(

            bakePoolWorker,

            std::ref(pool),

            std::cref(context),

            std::cref(rootKey),

            stats,

            std::ref(rootModuleOut),

            std::cref(options),

            threadCount,

            std::ref(sessionCache));

    }



    for (std::thread& worker : workers) {

        worker.join();
    
    }



    if (pool.firstError) {

        std::rethrow_exception(pool.firstError);

    }



    if (options.onProgress) {

        const std::size_t done = pool.completed.load(std::memory_order_relaxed);

        options.onProgress(

            BakeProgress{

                BakeProgressPhase::Done,

                done,

                done,

                rootSourcePath,

                threadCount,

            });

    }

}



} // namespace detail



[[nodiscard]] inline RuntimeModule bakeModuleTree(

    const std::filesystem::path& inputPath,

    BakeTreeStats* stats = nullptr,

    const BakeTreeOptions& options = {}) {

    const BakeTreeContext context = detail::makeBakeTreeContext(inputPath);



    RuntimeModule rootModule;

    detail::runBakePool(inputPath, context, stats, rootModule, options);

    return rootModule;

}



} // namespace eu07::scene::bake


