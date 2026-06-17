/*

This Source Code Form is subject to the

terms of the Mozilla Public License, v.

2.0. If a copy of the MPL was not

distributed with this file, You can

obtain one at

http://mozilla.org/MPL/2.0/.

*/



#include <eu07/scene/bake/bake_tree.hpp>
#include <eu07/scene/bake/pack_model_spool.hpp>
#include <eu07/scene/include_resolve.hpp>

#include "scene/eu7/eu7_bake_parser.h"

#include "scene/eu7/eu7_bake_mem_guard.h"
#include "scene/eu7/v2/eu7v2_emit_runtime.h"

#include "utilities/Logs.h"



#include <atomic>

#include <chrono>

#include <deque>

#include <filesystem>

#include <iomanip>

#include <iostream>

#include <memory>

#include <mutex>

#include <fstream>
#include <sstream>

#include <string>

#include <thread>

#include <unordered_map>



namespace scene::eu7::bake_parser {

namespace {



namespace fs = std::filesystem;

using clock_type = std::chrono::steady_clock;



// Bake logs go to stdout (headless console) and WriteLog (-> log.txt when the
// game's LogService is running). One mutex keeps interleaved lines readable
// across the bake worker threads.
std::mutex g_log_mutex;

void

bake_log( std::string const &line ) {

    std::lock_guard<std::mutex> lock { g_log_mutex };

    std::cout << line << '\n' << std::flush;

    WriteLog( line );

    // Also tee to a dedicated file so headless runs (where stdout lands in a
    // detached console and the game LogService is not flushing) still leave a
    // readable record of timings/summaries.
    static std::ofstream bake_file { "eu7v2_bake.log", std::ios::app };
    if( bake_file ) {
        bake_file << line << '\n';
        bake_file.flush();
    }

}



[[nodiscard]] std::string

fmt_ms( double const ms ) {

    std::ostringstream out;

    out << std::fixed << std::setprecision( 1 ) << ms;

    return out.str();

}



[[nodiscard]] std::string

fmt_s( double const s ) {

    std::ostringstream out;

    out << std::fixed << std::setprecision( 2 ) << s;

    return out.str();

}



// Shared progress/timing state for one bake run. The bake_tree onProgress
// callback and the onModuleBaked hook both fire from worker threads, so all
// mutable access is guarded.
struct bake_progress_state {

    clock_type::time_point start { clock_type::now() };

    std::mutex mutex;

    std::unordered_map<std::string, clock_type::time_point> module_start;

    clock_type::time_point compose_last_log {};

    bool compose_started { false };

    std::atomic<unsigned> threads { 0 };

    std::atomic<std::size_t> started { 0 };

    std::atomic<std::size_t> module_index { 0 };

    std::atomic<std::uint64_t> emit_us { 0 };

    // Live PACK-compose diagnostics, read by the watchdog thread so progress is

    // visible (and a stall is pinpointed) even if the worker callbacks stop.

    std::atomic<eu07::scene::bake::PackComposeStats const *> compose_stats { nullptr };

    std::atomic<bool> compose_active { false };

    std::string compose_file;

};



void

on_bake_progress( bake_progress_state &state, eu07::scene::bake::BakeProgress const &progress ) {

    using Phase = eu07::scene::bake::BakeProgressPhase;

    if( progress.threads > 0 ) {

        state.threads.store( progress.threads, std::memory_order_relaxed );

    }

    auto const name { progress.path.filename().generic_string() };

    switch( progress.phase ) {

    case Phase::Start: {

        std::size_t const i { state.started.fetch_add( 1, std::memory_order_relaxed ) + 1 };

        {

            std::lock_guard<std::mutex> lock { state.mutex };

            state.module_start[ progress.path.generic_string() ] = clock_type::now();

        }

        bake_log( "[EU7v2] (" + std::to_string( i ) + ") BAKING: " + progress.path.generic_string() );

        break;

    }

    case Phase::PackModels:

        state.compose_active.store( true, std::memory_order_relaxed );

        bake_log(

            "[EU7v2] ROOT " + name + ": PACK compose start (watki=" +

            std::to_string( progress.threads ) + ")" );

        break;

    case Phase::PackCompose: {

        if( progress.pack_stats != nullptr ) {

            state.compose_stats.store( progress.pack_stats, std::memory_order_relaxed );

        }

        // Throttle to ~2 s so a long compose visibly advances (and the last

        // file pinpoints a stall) without flooding the log.

        auto const now { clock_type::now() };

        std::lock_guard<std::mutex> lock { state.mutex };

        state.compose_file = name;

        bool const due {

            ( false == state.compose_started ) ||

            std::chrono::duration_cast<std::chrono::milliseconds>(

                now - state.compose_last_log )

                    .count() >= 2000 };

        if( due ) {

            state.compose_started = true;

            state.compose_last_log = now;

            bake_log(

                "[EU7v2]   PACK compose: pliki=" + std::to_string( progress.current ) +

                " modele=" + std::to_string( progress.total ) + " (plik: " + name + ")" );

        }

        break;

    }

    case Phase::PackComposeDone:

        state.compose_active.store( false, std::memory_order_relaxed );

        state.compose_stats.store( nullptr, std::memory_order_relaxed );

        bake_log(

            "[EU7v2]   PACK compose gotowe: pliki=" + std::to_string( progress.current ) +

            " modele=" + std::to_string( progress.total ) );

        break;

    case Phase::Starting:

    case Phase::Bake:

    case Phase::Done:

        break;

    }

}



[[nodiscard]] bool

is_text_scenery_path( fs::path const &path ) {

    auto const ext { path.extension().string() };

    return ext == ".scn" || ext == ".scm" || ext == ".inc" || ext == ".sbt";

}



} // namespace



bool

bake_scenario_tree(

    std::string const &text_scenario_path,

    unsigned const max_threads,

    std::string &error_out ) {

    error_out.clear();



    fs::path const input { text_scenario_path };

    if( false == fs::exists( input ) ) {

        error_out = "brak pliku: " + text_scenario_path;

        return false;

    }

    if( false == is_text_scenery_path( input ) ) {

        error_out = "nie jest plikiem SCM/SCN/INC: " + text_scenario_path;

        return false;

    }



    try {

        eu07::scene::bake::BakeTreeOptions options;

        options.maxThreads = max_threads;



        eu07::scene::bake::BakeTreeStats stats;

        (void)eu07::scene::bake::bakeModuleTree( input, &stats, options );

        return true;

    }

    catch( std::exception const &ex ) {

        error_out = ex.what();

        return false;

    }

    catch( ... ) {

        error_out = "nieznany wyjatek bake";

        return false;

    }

}



Eu7v2BakeReport
bake_scenario_tree_eu7v2(
    std::string const &text_scenario_path,
    unsigned const max_threads,
    bool const verify,
    unsigned const mem_limit_gb,
    unsigned const max_concurrent_parses,
    unsigned const heavy_parse_threshold_mb ) {

    Eu7v2BakeReport report;
    report.verify_requested = verify;

    std::uint64_t const mem_limit_bytes {
        mem_limit_gb == 0 ? 0u
                          : static_cast<std::uint64_t>( mem_limit_gb ) * 1024u * 1024u * 1024u };
    bake_mem_guard mem_guard { mem_limit_bytes };
    if( mem_limit_gb != 0 ) {
        bake_log(
            "[EU7v2] limit pamieci: " + std::to_string( mem_limit_gb ) +
            " GB (wylacz: --eu7v2-mem-limit-gb 0)" );
    }
    mem_guard.start();

    fs::path const input { text_scenario_path };
    if( false == fs::exists( input ) ) {
        report.error = "brak pliku: " + text_scenario_path;
        return report;
    }
    if( false == is_text_scenery_path( input ) ) {
        report.error = "nie jest plikiem SCM/SCN/INC: " + text_scenario_path;
        return report;
    }

    std::mutex emit_mutex;
    std::atomic<std::size_t> module_count { 0 };
    std::atomic<std::size_t> model_count { 0 };
    std::atomic<bool> verify_ok { true };
    std::atomic<bool> emit_failed { false };
    std::string root_binary_path;

    struct pending_verify_job {
        fs::path path;
        bool is_root { false };
        eu7v2::module_verify_spec spec {};
        std::size_t pack_models { 0 };
    };
    std::vector<pending_verify_job> verify_jobs;
    std::mutex verify_jobs_mutex;

    bake_progress_state pstate;

    bake_log(
        "[EU7v2] BAKE START: " + input.generic_string() +
        ( verify ? " (+verify)" : "" ) );

    try {
        eu07::scene::bake::BakeTreeOptions options;
        bool const spool_flush { mem_limit_gb != 0 };
        if( spool_flush ) {
            // PACK/shape spool + per-module flush bound RAM; pool can bake modules in parallel.
            options.lowMemoryMode = true;
            options.maxThreads = max_threads; // 0 => hardware default in runBakePool
            if( max_concurrent_parses != 0 ) {
                options.maxConcurrentParses = max_concurrent_parses;
            } else {
                options.maxConcurrentParses =
                    eu07::scene::bake::detail::defaultPoolThreadCount();
            }
        } else {
            options.maxThreads = max_threads != 0 ? max_threads : 1u;
            options.maxConcurrentParses =
                max_concurrent_parses != 0 ? max_concurrent_parses : 1u;
        }
        if( heavy_parse_threshold_mb != 0 ) {
            options.heavyParseThresholdMb = heavy_parse_threshold_mb;
        } else if( spool_flush ) {
            // Streaming/spool bounds RAM — duze pliki na dysku nie wymagaja serial parse.
            options.heavyParseThresholdMb = 0u;
        }
        std::unique_ptr<eu07::scene::bake::PackModelSpoolFile> root_pack_spool;
        if( spool_flush ) {
            fs::path const spool_path {
                fs::temp_directory_path() / "eu7v2_pack_flush.bin" };
            root_pack_spool =
                std::make_unique<eu07::scene::bake::PackModelSpoolFile>( spool_path );
            options.packFlushPerFile = true;
            options.onPackModelsFlush =
                [&root_pack_spool](
                    std::vector<eu07::scene::runtime::RuntimeModelInstance> &&models ) {
                    root_pack_spool->append( std::move( models ) );
                };
            bake_log(
                "[EU7v2] tryb: jeden plik = jeden modul, PACK per .scm -> " +
                spool_path.generic_string() +
                " (rownolegly bake modulow)" );
        }
        {
            unsigned const pool_threads {
                eu07::scene::bake::detail::resolvePoolThreadCount( options.maxThreads ) };
            unsigned const parse_limit {
                eu07::scene::bake::detail::resolveParseConcurrencyLimit(
                    options.maxConcurrentParses ) };
            bake_log(
                "[EU7v2] parse/bake throttle: watki=" +
                std::to_string( pool_threads ) +
                ( options.maxThreads == 0 ? " (auto)" : "" ) +
                ", rownolegle parse=" + std::to_string( parse_limit ) +
                ( options.heavyParseThresholdMb != 0
                      ? ", heavy serial >" + std::to_string( options.heavyParseThresholdMb ) + " MB"
                      : "" ) );
        }
        options.onProgress =
            [&pstate]( eu07::scene::bake::BakeProgress const &progress ) {
                on_bake_progress( pstate, progress );
            };
        options.skipLegacyWrite = true;
        options.omitChildModuleModels = true;

        struct deferred_root_emit {
            bool pending { false };
            eu07::scene::bake::RuntimeModule module;
            fs::path path;
        } deferred_root;
        bool const incremental_pack { options.lowMemoryMode };

        auto finish_module_emit =
            [&]( eu07::scene::bake::RuntimeModule const &module,
                 fs::path const &text_path,
                 bool const is_root,
                 std::vector<eu07::scene::binary::codec::ModelSectionBatch> const *pack_batches,
                 eu07::scene::bake::ShapeSpoolFile const *shape_spool ) {
                eu7v2::emit_outcome const outcome { eu7v2::emit_runtime_module(
                    module,
                    text_path,
                    is_root,
                    pack_batches,
                    false,
                    is_root && root_pack_spool != nullptr ? root_pack_spool.get() : nullptr,
                    shape_spool ) };

                double parse_bake_ms { 0.0 };
                {
                    std::lock_guard<std::mutex> lock { pstate.mutex };
                    auto const it { pstate.module_start.find( text_path.generic_string() ) };
                    if( it != pstate.module_start.end() ) {
                        parse_bake_ms = std::chrono::duration<double, std::milli>(
                                            clock_type::now() - it->second )
                                            .count();
                        pstate.module_start.erase( it );
                    }
                }

                std::size_t const idx {
                    module_count.fetch_add( 1, std::memory_order_relaxed ) + 1 };
                model_count.fetch_add( outcome.model_total, std::memory_order_relaxed );
                pstate.emit_us.fetch_add(
                    static_cast<std::uint64_t>(
                        ( outcome.build_ms + outcome.write_ms ) * 1000.0 ),
                    std::memory_order_relaxed );
                if( false == outcome.ok ) {
                    emit_failed.store( true, std::memory_order_relaxed );
                }

                if( verify && outcome.ok ) {
                    pending_verify_job job;
                    job.path = fs::path { outcome.written_path };
                    job.is_root = is_root;
                    for( auto const &inc : module.includes ) {
                        if( eu07::scene::detail::isIncFile( inc.sourcePath ) &&
                            inc.parameters.size() >= 5 ) {
                            try {
                                (void)std::stod( inc.parameters[ 1 ] );
                                (void)std::stod( inc.parameters[ 2 ] );
                                (void)std::stod( inc.parameters[ 3 ] );
                                (void)std::stod( inc.parameters[ 4 ] );
                                ++job.spec.placements;
                                continue;
                            } catch( ... ) {
                            }
                        }
                        ++job.spec.includes;
                    }
                    job.spec.models = module.scene.models.size();
                    job.spec.shapes = module.scene.shapes.size();
                    if( shape_spool != nullptr ) {
                        job.spec.shapes += shape_spool->shape_count();
                    }
                    job.spec.lines = module.scene.lines.size();
                    job.spec.tracks = module.scene.tracks.size();
                    job.spec.traction = module.scene.traction.size();
                    job.spec.power = module.scene.powerSources.size();
                    job.spec.memcells = module.scene.memcells.size();
                    job.spec.launchers = module.scene.eventLaunchers.size();
                    job.spec.events = module.scene.events.size();
                    job.spec.sounds = module.scene.sounds.size();
                    job.spec.dynamics = module.scene.dynamics.size();
                    job.spec.trainsets = module.scene.trainsets.size();
                    if( is_root && pack_batches != nullptr ) {
                        for( auto const &batch : *pack_batches ) {
                            job.pack_models += batch.models.size();
                        }
                    }
                    if( is_root && root_pack_spool != nullptr ) {
                        job.pack_models += root_pack_spool->model_count();
                    }
                    std::lock_guard<std::mutex> lock { verify_jobs_mutex };
                    verify_jobs.push_back( std::move( job ) );
                }

                bool const verbose_module {
                    is_root || false == outcome.ok || parse_bake_ms >= 300.0 ||
                    outcome.build_ms >= 100.0 };
                if( !verbose_module ) {
                    {
                        std::lock_guard<std::mutex> lock { emit_mutex };
                        if( is_root ) {
                            root_binary_path = outcome.written_path;
                        }
                    }
                    return;
                }

                std::string line {
                    "[EU7v2] (" + std::to_string( idx ) + ") " +
                    ( is_root ? "ROOT " : "" ) + text_path.filename().generic_string() +
                    " - parse+bake " + fmt_ms( parse_bake_ms ) + "ms, build " +
                    fmt_ms( outcome.build_ms ) + "ms, zapis " + fmt_ms( outcome.write_ms ) + "ms" };
                line += " (models=" + std::to_string( outcome.model_total ) + ", " +
                        std::to_string( ( outcome.byte_size + 1023 ) / 1024 ) + " KB)";
                if( false == outcome.ok ) {
                    line += " ZAPIS-BLAD: " + outcome.message;
                }

                {
                    std::lock_guard<std::mutex> lock { emit_mutex };
                    if( is_root ) {
                        root_binary_path = outcome.written_path;
                    }
                }
                bake_log( line );

                if( is_root && ( pack_batches != nullptr || root_pack_spool != nullptr ) ) {
                    std::size_t pack_models { 0 };
                    if( root_pack_spool != nullptr ) {
                        pack_models += root_pack_spool->model_count();
                    }
                    if( pack_batches != nullptr ) {
                        for( auto const &batch : *pack_batches ) {
                            pack_models += batch.models.size();
                        }
                    }
                    bake_log(
                        "[EU7v2]   ROOT packModels=" + std::to_string( pack_models ) +
                        ( root_pack_spool != nullptr
                              ? " (spool=" + std::to_string( root_pack_spool->model_count() ) +
                                    ")"
                              : "" ) );
                }
            };

        options.onModuleBaked =
            [&]( eu07::scene::bake::RuntimeModule const &module,
                 fs::path const &text_path,
                 bool const is_root,
                 std::vector<eu07::scene::binary::codec::ModelSectionBatch> const *pack_batches,
                 eu07::scene::bake::ShapeSpoolFile *shape_spool ) {
                if( is_root && incremental_pack ) {
                    deferred_root.pending = true;
                    deferred_root.module = module;
                    deferred_root.path = text_path;
                    return;
                }
                finish_module_emit( module, text_path, is_root, pack_batches, shape_spool );
            };

        // Watchdog: prints a heartbeat every ~2 s so a long (or stuck) bake is
        // visibly alive. During PACK compose it reads the live counters
        // directly, so a stall shows which counter is frozen and on which file.
        std::atomic<bool> watchdog_done { false };
        std::thread watchdog( [&]() {
            while( false == watchdog_done.load( std::memory_order_relaxed ) ) {
                for( int slice { 0 };
                     slice < 20 &&
                     false == watchdog_done.load( std::memory_order_relaxed );
                     ++slice ) {
                    std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
                }
                if( watchdog_done.load( std::memory_order_relaxed ) ) {
                    break;
                }
                double const elapsed {
                    std::chrono::duration<double>( clock_type::now() - pstate.start ).count() };
                std::string msg {
                    "[EU7v2] ... pracuje: elapsed=" + fmt_s( elapsed ) +
                    "s, moduly start=" + std::to_string( pstate.started.load( std::memory_order_relaxed ) ) +
                    " done=" + std::to_string( module_count.load( std::memory_order_relaxed ) ) };
                auto const *cs { pstate.compose_stats.load( std::memory_order_relaxed ) };
                if( pstate.compose_active.load( std::memory_order_relaxed ) && cs != nullptr ) {
                    std::string file;
                    {
                        std::lock_guard<std::mutex> lock { pstate.mutex };
                        file = pstate.compose_file;
                    }
                    msg +=
                        " | PACK compose: pliki=" +
                        std::to_string( cs->files_visited.load( std::memory_order_relaxed ) ) +
                        " submit=" +
                        std::to_string( cs->includes_submitted.load( std::memory_order_relaxed ) ) +
                        " inst=" +
                        std::to_string(
                            cs->instantiate_fast.load( std::memory_order_relaxed ) +
                            cs->instantiate_full.load( std::memory_order_relaxed ) ) +
                        " miss=" +
                        std::to_string( cs->cache_misses.load( std::memory_order_relaxed ) ) +
                        " kolejka_max=" +
                        std::to_string( cs->queue_peak.load( std::memory_order_relaxed ) ) +
                        " (plik: " + file + ")";
                    // Live deadlock diagnostics: what every worker/gate is doing.
                    msg +=
                        "\n[EU7v2]      DIAG: pending=" +
                        std::to_string( cs->diag_pending.load( std::memory_order_relaxed ) ) +
                        " workers_alive=" +
                        std::to_string( cs->diag_workers_alive.load( std::memory_order_relaxed ) ) +
                        " idle=" +
                        std::to_string( cs->diag_workers_idle.load( std::memory_order_relaxed ) ) +
                        " parse=" +
                        std::to_string( cs->diag_workers_parse.load( std::memory_order_relaxed ) ) +
                        " proc=" +
                        std::to_string( cs->diag_workers_inst.load( std::memory_order_relaxed ) ) +
                        " gate=" +
                        std::to_string( cs->diag_gate_active.load( std::memory_order_relaxed ) ) +
                        "/" +
                        std::to_string( cs->diag_gate_max.load( std::memory_order_relaxed ) ) +
                        " gate_wait=" +
                        std::to_string( cs->diag_gate_waiting.load( std::memory_order_relaxed ) ) +
                        " main_wait=" +
                        std::to_string( cs->diag_main_wait.load( std::memory_order_relaxed ) );
                }
                bake_log( msg );
            }
        } );

        eu07::scene::bake::BakeTreeStats stats;
        try {
            (void)eu07::scene::bake::bakeModuleTree( input, &stats, options );
        }
        catch( ... ) {
            watchdog_done.store( true, std::memory_order_relaxed );
            watchdog.join();
            throw;
        }
        watchdog_done.store( true, std::memory_order_relaxed );
        watchdog.join();

        if( deferred_root.pending ) {
            bake_log( "[EU7v2] ROOT emit (po wszystkich modulach)..." );
            finish_module_emit( deferred_root.module, deferred_root.path, true, nullptr, nullptr );
        }

        if( verify && false == verify_jobs.empty() ) {
            auto const verify_begin { clock_type::now() };
            std::atomic<std::size_t> next_job { 0 };
            std::mutex verify_error_mutex;
            std::string verify_error;
            const unsigned worker_count { std::max(
                1u,
                std::min(
                    static_cast<unsigned>( verify_jobs.size() ),
                    std::thread::hardware_concurrency() == 0 ? 4u
                                                             : std::thread::hardware_concurrency() ) ) };
            std::vector<std::thread> verify_workers;
            verify_workers.reserve( worker_count );
            for( unsigned worker_index { 0 }; worker_index < worker_count; ++worker_index ) {
                verify_workers.emplace_back( [&]() {
                    while( true ) {
                        std::size_t const job_index {
                            next_job.fetch_add( 1, std::memory_order_relaxed ) };
                        if( job_index >= verify_jobs.size() ) {
                            return;
                        }
                        pending_verify_job const &job { verify_jobs[ job_index ] };
                        std::string message;
                        if( false ==
                            eu7v2::verify_written_module(
                                job.path, job.spec, job.is_root, job.pack_models, &message ) ) {
                            verify_ok.store( false, std::memory_order_relaxed );
                            std::lock_guard<std::mutex> lock { verify_error_mutex };
                            if( verify_error.empty() ) {
                                verify_error = job.path.filename().generic_string() + ":\n" + message;
                            }
                        }
                    }
                } );
            }
            for( std::thread &worker : verify_workers ) {
                worker.join();
            }
            double const verify_s {
                std::chrono::duration<double>( clock_type::now() - verify_begin ).count() };
            pstate.emit_us.fetch_add(
                static_cast<std::uint64_t>( verify_s * 1.0e6 ),
                std::memory_order_relaxed );
            bake_log(
                "[EU7v2] VERIFY batch: " + std::to_string( verify_jobs.size() ) + " plikow, " +
                fmt_s( verify_s ) + "s" );
            if( false == verify_ok.load() && false == verify_error.empty() ) {
                bake_log( verify_error );
            }
        }

        report.baked = true;
        report.module_count = module_count.load();
        report.model_count = model_count.load();
        report.root_binary_path = root_binary_path;
        report.verify_ok = verify_ok.load() && ( false == emit_failed.load() );
        if( emit_failed.load() ) {
            report.error = "blad zapisu eu7v2 (patrz log)";
        }

        double const total_s {
            std::chrono::duration<double>( clock_type::now() - pstate.start ).count() };
        double const emit_s {
            static_cast<double>( pstate.emit_us.load( std::memory_order_relaxed ) ) / 1.0e6 };
        std::string summary {
            "[EU7v2] BAKE DONE: modules=" + std::to_string( report.module_count ) +
            ", total=" + fmt_s( total_s ) + "s, watki=" +
            std::to_string( pstate.threads.load( std::memory_order_relaxed ) ) +
            ", emit=" + fmt_s( emit_s ) + "s" };
        if( stats.has_pack_diagnostics ) {
            auto const &pc { stats.pack_compose };
            summary += ", compose=" + fmt_s( static_cast<double>( pc.compose_us ) / 1.0e6 ) +
                       "s, parse=" + fmt_s( static_cast<double>( pc.parse_us ) / 1.0e6 ) +
                       "s [tok=" + fmt_s( static_cast<double>( pc.tokenize_us ) / 1.0e6 ) +
                       " proc=" + fmt_s( static_cast<double>( pc.process_us ) / 1.0e6 ) +
                       " make=" + fmt_s( static_cast<double>( pc.makeentry_us ) / 1.0e6 ) +
                       "], inst=" + fmt_s( static_cast<double>( pc.instantiate_us ) / 1.0e6 ) +
                       "s, sink=" + fmt_s( static_cast<double>( pc.sink_us ) / 1.0e6 ) +
                       "s, finalize=" + fmt_s( static_cast<double>( pc.finalize_us ) / 1.0e6 ) +
                       "s (pliki=" + std::to_string( pc.files_visited ) +
                       ", modele=" + std::to_string( pc.models_emitted ) +
                       ", sekcje=" + std::to_string( pc.sections ) +
                       ", inc_inline=" + std::to_string( pc.inc_includes_inlined ) +
                       ", miss=" + std::to_string( pc.cache_misses ) +
                       ", hit=" + std::to_string( pc.cache_hits ) + ")";
        }
        bake_log( summary );
        if( verify ) {
            bake_log( report.verify_ok ? "[EU7v2] VERIFY: PASS" : "[EU7v2] VERIFY: FAIL" );
        }
        return report;
    }
    catch( std::exception const &ex ) {
        report.error = ex.what();
        return report;
    }
    catch( ... ) {
        report.error = "nieznany wyjatek bake eu7v2";
        return report;
    }
}

} // namespace scene::eu7::bake_parser

