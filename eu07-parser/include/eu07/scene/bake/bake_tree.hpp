#pragma once



// Rekurencyjny bake: kazdy plik tekstowy → wlasny .eu7, dzieci z INCL tez.

// Znaleziony include → od razu na kolejke i wolny watek (bez osobnej fazy odkrywania).



#include <eu07/parser.hpp>

#include <eu07/scene/bake/compose_pack_models.hpp>
#include <eu07/scene/bake/module.hpp>

#include <eu07/scene/binary/runtime_module.hpp>

#include <eu07/scene/include_resolve.hpp>

#include <eu07/scene/include_scan.hpp>

#include <eu07/scene/processor.hpp>



#include <atomic>

#include <condition_variable>

#include <cstdint>

#include <deque>

#include <exception>

#include <filesystem>

#include <functional>

#include <iostream>

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

};



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

    const unsigned threadCount) {

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

    try {

        const ParseResult parsed = parseFile(item.path);

        SceneProcessOptions shallowOpts;

        shallowOpts.expandIncludes = false;

        const SceneDocument document =

            processScene(parsed, context.sceneryRoot, shallowOpts).document;



        const bool emitPackModels = canonicalPathKey(item.path) == rootKey;
        RuntimeModule module = bakeModule(document, {.skipLocalModels = emitPackModels});

        const std::filesystem::path eu7Path =

            item.path.parent_path() / (item.path.stem().string() + ".eu7");

        const std::string key = canonicalPathKey(item.path);

        binary::WriteRuntimeModuleOptions writeOpts;
        writeOpts.emitPackModels = emitPackModels;
        std::vector<binary::codec::ModelSectionBatch> packBatches;
        if (writeOpts.emitPackModels) {
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
            pack_options.pack_batches = &packBatches;
            PackComposeSeed root_seed;
            root_seed.path = item.path.lexically_normal();
            root_seed.entry = makePackFileCacheEntry(document);
            pack_options.root_seed = &root_seed;
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
            if (stats != nullptr) {
                stats->has_pack_diagnostics = true;
                stats->pack_compose = snapshotPackComposeStats(pack_stats);
            }
        }
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



        BakeWorkItem child;

        child.includeStack = item.includeStack;

        child.includeStack.push_back(item.path);



        for (const std::string& includeFile : scanIncludeFilePaths(parsed.tokens)) {

            child.path = eu07::scene::detail::resolveIncludeSourcePath(

                context.sceneryRoot, item.relativeFile, includeFile);

            child.relativeFile = eu07::scene::detail::relativeSceneryFile(

                context.sceneryRoot, child.path);

            enqueueWork(pool, child);

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

    const unsigned threadCount) {

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

                item, context, rootKey, pool, stats, rootModuleOut, options, threadCount);

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

            threadCount);

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


