/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/eu7/eu7_section_stream.h"

#include "scene/eu7/eu7_chunks.h"
#include "scene/eu7/eu7_load_stats.h"
#include "scene/eu7/eu7_pack_bench.h"
#include "model/AnimModel.h"
#include "model/MdlMngr.h"
#include "scene/eu7/eu7_model_prefetch.h"
#include "scene/eu7/eu7_reader.h"
#include "scene/eu7/eu7_section.h"
#include "simulation/simulation.h"
#include "simulation/simulationstateserializer.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/Timer.h"
#include "utilities/utilities.h"
#include "vehicle/DynObj.h"
#include "world/Track.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

namespace scene::eu7 {
namespace {

constexpr double kDrainBudgetMs { 12.0 };
constexpr double kLoaderDrainBudgetMs { 48.0 };
constexpr std::size_t kLoaderSectionsPerDrain { 8 };

constexpr double kGameplayApplyBudgetMs { 4.0 };
constexpr double kCatchupApplyBudgetMs { 6.0 };
constexpr std::size_t kGameplaySliceInstances { 96 };
constexpr std::size_t kCatchupSliceInstances { 160 };
constexpr std::size_t kLoaderSliceInstances { 768 };
constexpr std::size_t kGameplaySliceColdMeshes { 1 };
constexpr std::size_t kCatchupSliceColdMeshes { 2 };
constexpr std::size_t kLoaderSliceColdMeshes { 32 };
constexpr double kGameplayColdBudgetMs { 3.0 };
constexpr double kCatchupColdBudgetMs { 5.0 };

constexpr int kInitialBootstrapRadius { 4 };
constexpr int kStreamRadius { 11 };
constexpr int kMovementLookahead { 7 };
constexpr std::size_t kMaxPackStreamWorkers { 4 };
constexpr std::size_t kMaxInFlightSections { 8 };
constexpr std::size_t kMaxReadySections { 3 };
constexpr double kStreamStatusLogIntervalSec { 5.0 };
constexpr double kReenqueueDistanceM { 500.0 };
constexpr double kCatchupReenqueueDistanceM { 40.0 };
constexpr double kTeleportReenqueueDistanceM { 2000.0 };
constexpr float kStationaryCatchupRingThreshold { 0.70f };
constexpr double kStationaryCatchupSpeedMps { 5.0 };
constexpr std::size_t kCatchupMaxInFlightSections { 20 };
constexpr std::size_t kCatchupMaxReadySections { 8 };
constexpr std::size_t kPackWorkerSubChunkModels { 512 };
constexpr std::size_t kBootstrapDrainMs { 32 };
constexpr std::size_t kBootstrapTimeoutMs { 120000 };
constexpr std::chrono::milliseconds kPresentableHoldMs { 200 };
constexpr std::chrono::seconds kLoadingScreenMaxBlockSec { 90 };

std::optional<std::chrono::steady_clock::time_point> g_ring_ready_since;
std::optional<std::chrono::steady_clock::time_point> g_loading_block_started;
bool g_loading_screen_dismissed { false };
bool g_stream_catchup { false };
std::size_t g_stream_max_in_flight { kMaxInFlightSections };
std::size_t g_stream_max_ready { kMaxReadySections };

struct PackSectionJob {
    int row { 0 };
    int column { 0 };
    std::size_t section_idx { 0 };
    int priority { 0 };
    std::uint64_t resume_byte_offset { 0 };
    std::uint32_t subchunk_index { 0 };
    Eu7PackSectionCursor header_cursor {};
    Eu7PackSectionCursor resume_cursor {};
};

struct PackSectionReady {
    int row { 0 };
    int column { 0 };
    std::size_t section_idx { 0 };
    std::unique_ptr<std::vector<Eu7Model>> models;
    bool failed { false };
    bool section_final { true };
    std::uint32_t subchunk_index { 0 };
};

struct SectionStreamState {
    Eu7Module const *module { nullptr };
    std::string path;
    simulation::state_serializer *serializer { nullptr };
    std::unordered_set<std::size_t> loaded_sections;
    std::unordered_set<std::size_t> in_flight_sections;
    int center_row { -1 };
    int center_column { -1 };
    int radius { kStreamRadius };
    bool active { false };
    bool bootstrap_active { false };
    bool bootstrap_pending { false };

    std::optional<PackSectionReady> pending_apply;
    std::size_t pending_apply_offset { 0 };
    glm::dvec3 last_enqueue_position {};
    bool has_last_enqueue_position { false };

    std::unordered_map<std::string, TModel3d *> mesh_cache;
    std::unordered_map<std::string, scene::node_data> nodedata_cache;

    std::vector<std::jthread> workers;
    std::atomic<bool> worker_exit { false };
    threading::lockable<std::deque<PackSectionJob>> jobs;
    threading::lockable<std::deque<PackSectionReady>> ready;
    threading::condition_variable work_cv;
};

SectionStreamState g_stream;

[[nodiscard]] int
section_manhattan_sections(
    int const row,
    int const column,
    int const center_row,
    int const center_column );

void
push_prioritized_job( PackSectionJob job );

void
reprioritize_job_queue();

[[nodiscard]] bool
enqueue_section_if_needed(
    int const row,
    int const column,
    int const priority );

void
sync_stream_limits( glm::dvec3 const &world_position );

[[nodiscard]] bool
try_dequeue_ready_batch();

void
finalize_section( PackSectionReady const &batch );

void
fail_section( std::size_t const section_idx );

void
release_pending_buffer();

[[nodiscard]] double
camera_stream_speed_mps();

[[nodiscard]] glm::dvec3
rotate_velocity_y( glm::dvec3 const &velocity, double const yaw ) {
    auto const s { std::sin( yaw ) };
    auto const c { std::cos( yaw ) };
    return { c * velocity.x + s * velocity.z, velocity.y, c * velocity.z - s * velocity.x };
}

[[nodiscard]] std::size_t
stream_worker_count() {
    auto const hardware { std::thread::hardware_concurrency() };
    auto const threads { hardware > 0 ? hardware : 1u };
    auto const percent {
        static_cast<unsigned>(
            std::clamp( Global.eu7_pack_stream_workers_percent, 1, 100 ) ) };
    auto const requested {
        std::max(
            std::size_t { 1 },
            static_cast<std::size_t>( threads ) * percent / 100 ) };
    return std::min( requested, kMaxPackStreamWorkers );
}

void
reset_stream_fields() {
    g_stream.module = nullptr;
    g_stream.path.clear();
    g_stream.serializer = nullptr;
    g_stream.loaded_sections.clear();
    g_stream.in_flight_sections.clear();
    g_stream.center_row = -1;
    g_stream.center_column = -1;
    g_stream.radius = kStreamRadius;
    g_stream.active = false;
    g_stream.bootstrap_active = false;
    g_stream.bootstrap_pending = false;
    g_stream.pending_apply.reset();
    g_stream.pending_apply_offset = 0;
    g_stream.has_last_enqueue_position = false;
    g_stream.mesh_cache.clear();
    g_stream.nodedata_cache.clear();
    reset_pack_texture_warm_cache();
}

[[nodiscard]] Eu7Module const &
stream_module() {
    if( g_stream.module == nullptr ) {
        throw std::runtime_error( "EU7 PACK: stream module nie jest ustawiony" );
    }
    return *g_stream.module;
}

[[nodiscard]] std::string const &
pack_stream_file_path() {
    if( false == g_stream.path.empty() ) {
        return g_stream.path;
    }
    return stream_module().source_path;
}

void
release_pending_buffer() {
    if( false == g_stream.pending_apply.has_value() ) {
        g_stream.pending_apply_offset = 0;
        return;
    }

    if( g_stream.pending_apply->models != nullptr ) {
        g_stream.pending_apply->models->clear();
        g_stream.pending_apply->models->shrink_to_fit();
        g_stream.pending_apply->models.reset();
    }
    g_stream.pending_apply.reset();
    g_stream.pending_apply_offset = 0;
}

[[nodiscard]] std::size_t
ready_queue_size() {
    std::lock_guard<std::mutex> lock { g_stream.ready.mutex };
    return g_stream.ready.data.size();
}

[[nodiscard]] bool
stream_backpressure() {
    if( g_stream.bootstrap_active ) {
        return false;
    }
    return (
        g_stream.in_flight_sections.size() >= g_stream_max_in_flight ||
        ready_queue_size() >= g_stream_max_ready );
}

[[nodiscard]] double
camera_stream_speed_mps() {
    auto const rotated {
        rotate_velocity_y( Global.pCamera.Velocity, Global.pCamera.Angle.y ) };
    return glm::length( rotated ) * 5.0;
}

[[nodiscard]] glm::dvec3
camera_travel_forward() {
    if( glm::length2( Global.pCamera.Velocity ) > 0.01 ) {
        auto const rotated {
            rotate_velocity_y( Global.pCamera.Velocity, Global.pCamera.Angle.y ) };
        auto horizontal { glm::dvec3 { rotated.x, 0.0, rotated.z } };
        auto const length { glm::length( horizontal ) };
        if( length > 0.01 ) {
            return horizontal / length;
        }
    }

    auto const yaw { Global.pCamera.Angle.y };
    return glm::dvec3 { std::sin( yaw ), 0.0, std::cos( yaw ) };
}

[[nodiscard]] double
gameplay_apply_budget_ms() {
    return g_stream_catchup ? kCatchupApplyBudgetMs : kGameplayApplyBudgetMs;
}

[[nodiscard]] std::size_t
gameplay_slice_instances() {
    return g_stream_catchup ? kCatchupSliceInstances : kGameplaySliceInstances;
}

[[nodiscard]] std::size_t
gameplay_slice_cold_meshes() {
    return g_stream_catchup ? kCatchupSliceColdMeshes : kGameplaySliceColdMeshes;
}

[[nodiscard]] double
gameplay_cold_budget_ms() {
    return g_stream_catchup ? kCatchupColdBudgetMs : kGameplayColdBudgetMs;
}

[[nodiscard]] std::size_t
adaptive_slice_instances( std::size_t const section_total ) {
    auto limit { gameplay_slice_instances() };
    if( g_stream_catchup ) {
        auto const speed { camera_stream_speed_mps() };
        if( speed > 1200.0 ) {
            limit = std::min( limit, kCatchupSliceInstances );
        }
        else if( speed > 600.0 ) {
            limit = std::min( limit, std::size_t { 96 } );
        }
        else if( speed > 300.0 ) {
            limit = std::min( limit, std::size_t { 80 } );
        }
        else {
            limit = std::min( limit, std::size_t { 96 } );
        }
    }

    if( section_total > 4000 ) {
        limit = std::min( limit, std::size_t { 48 } );
    }
    else if( section_total > 2000 ) {
        limit = std::min( limit, std::size_t { 64 } );
    }
    else if( section_total > 800 ) {
        limit = std::min( limit, std::size_t { 80 } );
    }
    else if( section_total <= kPackWorkerSubChunkModels ) {
        limit = std::min( limit, std::size_t { 64 } );
        if( g_stream_catchup && camera_stream_speed_mps() > 300.0 ) {
            limit = std::min( limit, std::size_t { 48 } );
        }
    }

    auto const last_ms { pack_bench_stream().last_chunk_ms };
    if( last_ms >= 24.0 ) {
        limit = std::min( limit, std::size_t { 32 } );
    }
    else if( last_ms >= 12.0 ) {
        limit = std::min( limit, std::size_t { 48 } );
    }

    return std::max( limit, std::size_t { 16 } );
}

[[nodiscard]] std::size_t
adaptive_cold_meshes() {
    auto const &stream { pack_bench_stream() };
    if( stream.last_chunk_ms >= 40.0 || stream.peak_chunk_ms >= 80.0 ) {
        return 0;
    }
    auto limit { gameplay_slice_cold_meshes() };
    if( stream.last_chunk_ms >= 12.0 ) {
        limit = 1;
    }
    return limit;
}

[[nodiscard]] std::size_t
pending_section_total() {
    if(
        false == g_stream.pending_apply.has_value() ||
        g_stream.pending_apply->models == nullptr ) {
        return 0;
    }
    return g_stream.pending_apply->models->size();
}

[[nodiscard]] std::size_t
preload_slice_cold_meshes(
    scene::eu7::Eu7Model const *const models,
    std::size_t const count,
    std::size_t const max_cold_meshes,
    double const cold_budget_ms,
    std::size_t &slice_count ) {
    slice_count = count;
    if( models == nullptr || count == 0 ) {
        slice_count = 0;
        return 0;
    }

    std::size_t cold_loaded { 0 };
    auto const cold_started { std::chrono::steady_clock::now() };
    for( std::size_t i { 0 }; i < slice_count; ++i ) {
        auto model_file { models[ i ].model_file };
        if( model_file.empty() || model_file == "notload" ) {
            continue;
        }
        replace_slashes( model_file );
        if( g_stream.mesh_cache.contains( model_file ) ) {
            continue;
        }
        if( cold_loaded >= max_cold_meshes ) {
            slice_count = i;
            break;
        }
        if( cold_budget_ms > 0.0 && cold_loaded > 0 ) {
            auto const elapsed_ms {
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - cold_started ).count() };
            if( elapsed_ms >= cold_budget_ms ) {
                slice_count = i;
                break;
            }
        }
        TModel3d *mesh { nullptr };
        {
            PackBenchTimer const load_timer { &Eu7PackBench::main_cold_getmodel_ms };
            mesh = TModelsManager::GetModel( model_file, false, false );
            pack_bench_inc( &Eu7PackBench::main_cold_getmodel_calls );
        }
        if( mesh != nullptr ) {
            TAnimModel::warm_instanceable_cache( mesh );
        }
        g_stream.mesh_cache.emplace( model_file, mesh );
        ++cold_loaded;
    }

    if( slice_count == 0 && count > 0 ) {
        auto model_file { models[ 0 ].model_file };
        if( model_file.empty() || model_file == "notload" ) {
            slice_count = 1;
            return 0;
        }
        replace_slashes( model_file );
        if( g_stream.mesh_cache.contains( model_file ) ) {
            slice_count = 1;
            return 0;
        }
        if( max_cold_meshes == 0 ) {
            return 0;
        }
        TModel3d *mesh { nullptr };
        {
            PackBenchTimer const load_timer { &Eu7PackBench::main_cold_getmodel_ms };
            mesh = TModelsManager::GetModel( model_file, false, false );
            pack_bench_inc( &Eu7PackBench::main_cold_getmodel_calls );
        }
        if( mesh != nullptr ) {
            TAnimModel::warm_instanceable_cache( mesh );
        }
        g_stream.mesh_cache.emplace( model_file, mesh );
        slice_count = 1;
        cold_loaded = 1;
    }
    return cold_loaded;
}

[[nodiscard]] bool
apply_pending_chunk(
    double const budget_ms,
    std::size_t const max_instances,
    std::size_t const max_cold_meshes,
    double const cold_budget_ms,
    std::size_t const max_chunks ) {
    if( g_stream.serializer == nullptr ) {
        return false;
    }
    if( false == g_stream.pending_apply.has_value() && false == try_dequeue_ready_batch() ) {
        return false;
    }

    auto const started { std::chrono::steady_clock::now() };
    bool applied_work { false };
    std::size_t chunks_done { 0 };

    while( chunks_done < max_chunks ) {
        if( false == g_stream.pending_apply.has_value() ) {
            if( false == try_dequeue_ready_batch() ) {
                return applied_work;
            }
        }

        auto &batch { *g_stream.pending_apply };
        if( batch.failed || batch.models == nullptr || batch.models->empty() ) {
            if( batch.failed ) {
                fail_section( batch.section_idx );
            }
            else {
                finalize_section( batch );
            }
            release_pending_buffer();
            applied_work = true;
            continue;
        }

        scene::eu7::Eu7Model const * const models_ptr { batch.models->data() };
        auto const total { batch.models->size() };
        auto const offset { g_stream.pending_apply_offset };
        if( offset >= total ) {
            if( batch.section_final ) {
                finalize_section( batch );
            }
            release_pending_buffer();
            applied_work = true;
            continue;
        }

        auto const effective_instances {
            std::min( max_instances, adaptive_slice_instances( total ) ) };
        auto const effective_cold {
            std::min( max_cold_meshes, adaptive_cold_meshes() ) };
        auto const remaining { total - offset };
        auto const chunk_cap { std::min( effective_instances, remaining ) };
        std::size_t chunk_count { chunk_cap };

        auto const chunk_started { std::chrono::steady_clock::now() };
        double cold_ms { 0.0 };
        double warm_ms { 0.0 };
        double apply_ms { 0.0 };
        std::size_t cold_loads { 0 };
        std::size_t tex_fetches { 0 };

        {
            auto const phase_started { std::chrono::steady_clock::now() };
            cold_loads = preload_slice_cold_meshes(
                models_ptr + offset,
                chunk_cap,
                effective_cold,
                cold_budget_ms,
                chunk_count );
            cold_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - phase_started ).count();
        }
        if( chunk_count == 0 ) {
            return applied_work;
        }

        {
            auto const phase_started { std::chrono::steady_clock::now() };
            tex_fetches = warm_pack_textures_main( models_ptr + offset, chunk_count );
            warm_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - phase_started ).count();
        }
        pack_bench_inc( &Eu7PackBench::main_chunk_tex_fetches, tex_fetches );

        scene::scratch_data scratch;
        simulation::eu7_pack_apply_session const session {
            &g_stream.mesh_cache,
            &g_stream.nodedata_cache };
        {
            auto const phase_started { std::chrono::steady_clock::now() };
            g_stream.serializer->apply_eu7_pack_models(
                models_ptr,
                offset,
                chunk_count,
                scratch,
                &session );
            apply_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - phase_started ).count();
        }

        auto const chunk_wall_ms {
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - chunk_started ).count() };
        pack_bench_note_chunk(
            chunk_wall_ms,
            chunk_count,
            cold_loads,
            cold_ms,
            warm_ms,
            apply_ms );

        if( chunk_wall_ms >= 12.0 && pack_bench_stream_phase_active() ) {
            WriteLog(
                "EU7 PACK [hitch]: chunk_ms=" + std::to_string( static_cast<int>( chunk_wall_ms ) ) +
                " inst=" + std::to_string( chunk_count ) +
                " cold=" + std::to_string( cold_loads ) + "(" + std::to_string( static_cast<int>( cold_ms ) ) + "ms)" +
                " warm=" + std::to_string( tex_fetches ) + "(" + std::to_string( static_cast<int>( warm_ms ) ) + "ms)" +
                " apply=" + std::to_string( static_cast<int>( apply_ms ) ) + "ms" +
                " pending=" + std::to_string( offset + chunk_count ) + "/" + std::to_string( total ) +
                " sec=" + std::to_string( batch.row ) + "," + std::to_string( batch.column ) );
        }
        load_stats().pack_models += chunk_count;
        g_stream.pending_apply_offset = offset + chunk_count;
        applied_work = true;
        ++chunks_done;

        if( g_stream.pending_apply_offset >= total ) {
            if( batch.section_final ) {
                finalize_section( batch );
            }
            release_pending_buffer();
            continue;
        }

        if( budget_ms <= 0.0 ) {
            return applied_work;
        }

        auto const elapsed_ms {
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started ).count() };
        if( elapsed_ms >= budget_ms ) {
            pack_bench_inc( &Eu7PackBench::drain_budget_stops );
            return applied_work;
        }
    }

    return applied_work;
}

void
maybe_log_stream_status( glm::dvec3 const &world_position ) {
    if( false == pack_bench_stream_phase_active() ) {
        return;
    }

    static double s_last_status_log { 0.0 };
    auto const now { Timer::GetTime() };
    if( now - s_last_status_log < kStreamStatusLogIntervalSec ) {
        return;
    }
    s_last_status_log = now;

    auto const speed { camera_stream_speed_mps() };
    auto const inner_ring {
        section_stream_ring_progress( world_position, kSectionStreamGameplayRadiusKm ) };
    auto const outer_ring {
        section_stream_ring_progress( world_position, g_stream.radius ) };

    std::size_t pending_total { 0 };
    std::size_t pending_offset { g_stream.pending_apply_offset };
    if( g_stream.pending_apply.has_value() && g_stream.pending_apply->models != nullptr ) {
        pending_total = g_stream.pending_apply->models->size();
    }

    log_pack_stream_status(
        world_position,
        speed,
        inner_ring,
        outer_ring,
        ready_queue_size(),
        g_stream.in_flight_sections.size(),
        pending_offset,
        pending_total,
        gameplay_apply_budget_ms(),
        adaptive_slice_instances( pending_total ) );
}

[[nodiscard]] bool
section_has_pack_models( int const row, int const column ) {
    if( g_stream.module == nullptr ) {
        return false;
    }
    auto const entry { find_pack_entry( *g_stream.module, row, column ) };
    return entry.has_value() && entry->model_count > 0;
}

[[nodiscard]] bool
is_bootstrap_ring_loaded(
    int const center_row,
    int const center_column,
    int const radius ) {
    for( int row { center_row - radius }; row <= center_row + radius; ++row ) {
        if( row < 0 || row >= kRegionSideSectionCount ) {
            continue;
        }
        for( int column { center_column - radius }; column <= center_column + radius; ++column ) {
            if( column < 0 || column >= kRegionSideSectionCount ) {
                continue;
            }
            if( false == section_has_pack_models( row, column ) ) {
                continue;
            }
            if( false == g_stream.loaded_sections.contains( section_index( row, column ) ) ) {
                return false;
            }
        }
    }
    return true;
}

void
worker_loop( std::stop_token const stop_token ) {
    while( false == stop_token.stop_requested() && false == g_stream.worker_exit.load() ) {
        PackSectionJob job;
        bool has_job { false };
        {
            std::lock_guard<std::mutex> lock { g_stream.jobs.mutex };
            if( false == g_stream.jobs.data.empty() ) {
                job = g_stream.jobs.data.front();
                g_stream.jobs.data.pop_front();
                has_job = true;
            }
        }

        if( false == has_job ) {
            g_stream.work_cv.wait_for( std::chrono::milliseconds( 50 ) );
            continue;
        }

        if( ready_queue_size() >= g_stream_max_ready ) {
            {
                std::lock_guard<std::mutex> lock { g_stream.jobs.mutex };
                g_stream.jobs.data.push_front( job );
            }
            g_stream.work_cv.wait_for( std::chrono::milliseconds( 8 ) );
            continue;
        }

        try {
            auto const &module { stream_module() };
            auto const entry { find_pack_entry( module, job.row, job.column ) };
            if( false == entry.has_value() || entry->model_count == 0 ) {
                PackSectionReady result;
                result.row = job.row;
                result.column = job.column;
                result.section_idx = job.section_idx;
                result.failed = true;
                result.section_final = true;
                {
                    std::lock_guard<std::mutex> lock { g_stream.ready.mutex };
                    g_stream.ready.data.push_back( std::move( result ) );
                }
                pack_bench_inc( &Eu7PackBench::worker_failures );
                continue;
            }

            auto const &pack_path { pack_stream_file_path() };
            if( pack_path.empty() ) {
                throw std::runtime_error( "EU7 PACK: brak sciezki pliku PACK" );
            }

            std::ifstream input { pack_path, std::ios::binary };
            if( !input ) {
                throw std::runtime_error(
                    "EU7 PACK: nie mozna otworzyc \"" + pack_path + "\"" );
            }

            Eu7PackSectionCursor header_cursor {};
            Eu7PackSectionCursor cursor {};
            if( job.resume_byte_offset == 0 ) {
                seek_pack_section( module, input, *entry, header_cursor );
                cursor = header_cursor;
            }
            else {
                header_cursor = job.header_cursor;
                resume_pack_section(
                    module,
                    input,
                    *entry,
                    job.resume_byte_offset,
                    job.resume_cursor,
                    cursor );
            }

            auto batch_index { job.subchunk_index };
            bool section_completed { false };
            for( ;; ) {
                if( cursor.solo_remaining == 0 && cursor.inst_remaining == 0 ) {
                    break;
                }

                if( ready_queue_size() >= g_stream_max_ready ) {
                    PackSectionJob resume_job { job };
                    resume_job.header_cursor = header_cursor;
                    resume_job.resume_byte_offset = static_cast<std::uint64_t>(
                        input.tellg() - static_cast<std::streamoff>(
                            module.pack_payload_offset + entry->pack_offset ) );
                    resume_job.subchunk_index = batch_index;
                    resume_job.resume_cursor = cursor;
                    {
                        std::lock_guard<std::mutex> lock { g_stream.jobs.mutex };
                        g_stream.jobs.data.push_front( std::move( resume_job ) );
                    }
                    g_stream.work_cv.notify_all();
                    break;
                }

                std::unique_ptr<std::vector<Eu7Model>> models;
                {
                    PackBenchTimer const read_timer { &Eu7PackBench::worker_read_pack_ms };
                    auto chunk {
                        read_pack_models_chunk(
                            module,
                            input,
                            cursor,
                            kPackWorkerSubChunkModels ) };
                    if( chunk.empty() ) {
                        break;
                    }
                    models = std::make_unique<std::vector<Eu7Model>>( std::move( chunk ) );
                }

                section_completed =
                    cursor.solo_remaining == 0 && cursor.inst_remaining == 0;

                PackSectionReady result;
                result.row = job.row;
                result.column = job.column;
                result.section_idx = job.section_idx;
                result.models = std::move( models );
                result.subchunk_index = batch_index;
                result.section_final = section_completed;

                if( result.models != nullptr ) {
                    pack_bench_inc(
                        &Eu7PackBench::worker_models_decoded, result.models->size() );
                }

                {
                    std::lock_guard<std::mutex> lock { g_stream.ready.mutex };
                    g_stream.ready.data.push_back( std::move( result ) );
                }

                ++batch_index;

                if( section_completed ) {
                    break;
                }
            }

            if( section_completed ) {
                pack_bench_inc( &Eu7PackBench::worker_sections_done );
            }
        }
        catch( std::exception const &ex ) {
            ErrorLog(
                std::string{ "EU7 PACK: sekcja " + std::to_string( job.row ) + "," +
                    std::to_string( job.column ) + ": " } + ex.what() );
            fail_section( job.section_idx );
            PackSectionReady result;
            result.row = job.row;
            result.column = job.column;
            result.section_idx = job.section_idx;
            result.failed = true;
            result.section_final = true;
            {
                std::lock_guard<std::mutex> lock { g_stream.ready.mutex };
                g_stream.ready.data.push_back( std::move( result ) );
            }
            pack_bench_inc( &Eu7PackBench::worker_failures );
        }
    }
}

void
stop_workers() {
    g_stream.worker_exit = true;
    g_stream.work_cv.notify_all();
    g_stream.workers.clear();
    g_stream.worker_exit = false;

    {
        std::lock_guard<std::mutex> lock { g_stream.jobs.mutex };
        g_stream.jobs.data.clear();
    }
    {
        std::lock_guard<std::mutex> lock { g_stream.ready.mutex };
        for( auto &batch : g_stream.ready.data ) {
            if( batch.models != nullptr ) {
                batch.models->clear();
                batch.models->shrink_to_fit();
                batch.models.reset();
            }
        }
        g_stream.ready.data.clear();
    }
    g_stream.in_flight_sections.clear();
    release_pending_buffer();
    g_stream.bootstrap_active = false;
    g_ring_ready_since.reset();
}

void
start_workers() {
    stop_workers();
    if( false == g_stream.active ) {
        return;
    }

    auto const worker_count { stream_worker_count() };
    g_stream.workers.reserve( worker_count );
    for( std::size_t worker_idx { 0 }; worker_idx < worker_count; ++worker_idx ) {
        g_stream.workers.emplace_back( worker_loop );
    }

    WriteLog(
        "EU7 PACK: async loader started, workers=" + std::to_string( worker_count ) +
        ", radius=" + std::to_string( kStreamRadius ) +
        ", bootstrap=" + std::to_string( kInitialBootstrapRadius ) + "km" +
        ", lookahead=" + std::to_string( kMovementLookahead ) );
}

void
finalize_section( PackSectionReady const &batch ) {
    if( false == batch.section_final ) {
        return;
    }
    g_stream.loaded_sections.insert( batch.section_idx );
    g_stream.in_flight_sections.erase( batch.section_idx );
    ++load_stats().pack_sections_loaded;
    pack_bench_inc( &Eu7PackBench::sections_finalized );
}

void
fail_section( std::size_t const section_idx ) {
    g_stream.loaded_sections.insert( section_idx );
    g_stream.in_flight_sections.erase( section_idx );
}

[[nodiscard]] int
section_manhattan_sections(
    int const row,
    int const column,
    int const center_row,
    int const center_column ) {
    return std::abs( row - center_row ) + std::abs( column - center_column );
}

void
push_prioritized_job( PackSectionJob job ) {
    std::lock_guard<std::mutex> lock { g_stream.jobs.mutex };
    auto const insert_before {
        std::find_if(
            g_stream.jobs.data.begin(),
            g_stream.jobs.data.end(),
            [&]( PackSectionJob const &existing ) {
                return existing.priority > job.priority;
            } ) };
    g_stream.jobs.data.insert( insert_before, std::move( job ) );
}

void
reprioritize_job_queue() {
    std::lock_guard<std::mutex> lock { g_stream.jobs.mutex };
    if( g_stream.jobs.data.empty() ) {
        return;
    }

    for( auto &job : g_stream.jobs.data ) {
        job.priority = section_manhattan_sections(
            job.row,
            job.column,
            g_stream.center_row,
            g_stream.center_column );
    }

    std::stable_sort(
        g_stream.jobs.data.begin(),
        g_stream.jobs.data.end(),
        []( PackSectionJob const &lhs, PackSectionJob const &rhs ) {
            return lhs.priority < rhs.priority;
        } );
}

[[nodiscard]] bool
enqueue_section_if_needed(
    int const row,
    int const column,
    int const priority ) {
    if( row < 0 || row >= kRegionSideSectionCount || column < 0 || column >= kRegionSideSectionCount ) {
        return false;
    }
    if( stream_backpressure() ) {
        return false;
    }

    auto const section_idx { section_index( row, column ) };
    if( g_stream.loaded_sections.contains( section_idx ) ) {
        return false;
    }
    if( g_stream.in_flight_sections.contains( section_idx ) ) {
        {
            std::lock_guard<std::mutex> lock { g_stream.jobs.mutex };
            for( auto &queued : g_stream.jobs.data ) {
                if( queued.section_idx != section_idx ) {
                    continue;
                }
                if( priority < queued.priority ) {
                    queued.priority = priority;
                    std::stable_sort(
                        g_stream.jobs.data.begin(),
                        g_stream.jobs.data.end(),
                        []( PackSectionJob const &lhs, PackSectionJob const &rhs ) {
                            return lhs.priority < rhs.priority;
                        } );
                }
                return false;
            }
        }
        return false;
    }
    if(
        g_stream.pending_apply.has_value() &&
        g_stream.pending_apply->section_idx == section_idx ) {
        return false;
    }

    g_stream.in_flight_sections.insert( section_idx );

    PackSectionJob job;
    job.row = row;
    job.column = column;
    job.section_idx = section_idx;
    job.priority = priority;

    push_prioritized_job( std::move( job ) );
    g_stream.work_cv.notify_all();
    return true;
}

void
enqueue_sections_around(
    int const center_row,
    int const center_column,
    int const ring_radius,
    glm::dvec3 const &world_position ) {
    struct SectionCandidate {
        int row { 0 };
        int column { 0 };
        int distance { 0 };
    };

    std::vector<SectionCandidate> candidates;
    candidates.reserve( static_cast<std::size_t>( ( ring_radius * 2 + 1 ) * ( ring_radius * 2 + 1 ) ) );

    for( int row { center_row - ring_radius }; row <= center_row + ring_radius; ++row ) {
        if( row < 0 || row >= kRegionSideSectionCount ) {
            continue;
        }
        for( int column { center_column - ring_radius }; column <= center_column + ring_radius; ++column ) {
            if( column < 0 || column >= kRegionSideSectionCount ) {
                continue;
            }

            auto const section_center_x {
                ( static_cast<double>( column ) - kRegionSideSectionCount / 2 ) * kSectionSizeM +
                kSectionSizeM * 0.5 };
            auto const section_center_z {
                ( static_cast<double>( row ) - kRegionSideSectionCount / 2 ) * kSectionSizeM +
                kSectionSizeM * 0.5 };
            auto const dx { world_position.x - section_center_x };
            auto const dz { world_position.z - section_center_z };
            auto const dist {
                static_cast<int>(
                    ( std::abs( dx ) + std::abs( dz ) ) / kSectionSizeM ) };

            candidates.push_back( { row, column, dist } );
        }
    }

    std::sort(
        candidates.begin(),
        candidates.end(),
        []( SectionCandidate const &lhs, SectionCandidate const &rhs ) {
            return lhs.distance < rhs.distance;
        } );

    for( auto const &candidate : candidates ) {
        auto priority { candidate.distance };
        if( candidate.row == center_row && candidate.column == center_column ) {
            priority = 0;
        }
        enqueue_section_if_needed( candidate.row, candidate.column, priority );
        if( stream_backpressure() ) {
            break;
        }
    }
}

void
replenish_bootstrap_ring( glm::dvec3 const &world_position ) {
    if( world_position.x == 0.0 && world_position.y == 0.0 && world_position.z == 0.0 ) {
        return;
    }

    auto const [row, column] { section_row_column( world_position ) };
    g_stream.center_row = row;
    g_stream.center_column = column;
    enqueue_sections_around( row, column, kInitialBootstrapRadius, world_position );
}

[[nodiscard]] std::pair<int, int>
movement_section_delta( glm::dvec3 const &forward_hint ) {
    auto const horizontal { glm::dvec3 { forward_hint.x, 0.0, forward_hint.z } };
    auto const length { glm::length( horizontal ) };
    if( length < 0.01 ) {
        return { 0, 0 };
    }

    auto const fwd { horizontal / length };
    if( std::abs( fwd.x ) >= std::abs( fwd.z ) ) {
        return { 0, fwd.x > 0.0 ? 1 : -1 };
    }
    return { fwd.z > 0.0 ? 1 : -1, 0 };
}

[[nodiscard]] glm::dvec3
guess_travel_forward() {
    if(
        simulation::Train != nullptr &&
        simulation::Train->Dynamic() != nullptr ) {
        auto *const vehicle { simulation::Train->Dynamic() };
        if( std::abs( vehicle->GetVelocity() ) > 0.5 ) {
            auto forward { vehicle->VectorFront() };
            if( vehicle->DirectionGet() < 0 ) {
                forward = -forward;
            }
            return forward;
        }
    }

    if( Global.pCamera.m_owner == nullptr || FreeFlyModeFlag ) {
        return camera_travel_forward();
    }
    return {};
}

void
enqueue_movement_lookahead(
    int const center_row,
    int const center_column,
    int const ring_radius,
    glm::dvec3 const &forward_hint,
    int const max_steps ) {
    auto const [drow, dcolumn] { movement_section_delta( forward_hint ) };
    if( drow == 0 && dcolumn == 0 ) {
        return;
    }

    auto lookahead_steps { std::min( max_steps, kMovementLookahead ) };
    if(
        simulation::Train != nullptr &&
        simulation::Train->Dynamic() != nullptr ) {
        auto const speed { std::abs( simulation::Train->Dynamic()->GetVelocity() ) };
        if( speed > 25.0 ) {
            lookahead_steps = std::min( max_steps, 10 );
        }
        else if( speed > 15.0 ) {
            lookahead_steps = std::min( max_steps, 8 );
        }
        else if( speed > 8.0 ) {
            lookahead_steps = std::min( max_steps, 6 );
        }
    }
    if( max_steps > kMovementLookahead ) {
        auto const cam_speed { camera_stream_speed_mps() };
        if( cam_speed > 200.0 ) {
            lookahead_steps = max_steps;
        }
        else if( cam_speed > 80.0 ) {
            lookahead_steps = std::max( lookahead_steps, std::min( max_steps, 8 ) );
        }
    }

    for( int step { 1 }; step <= lookahead_steps; ++step ) {
        auto const row { center_row + drow * ( ring_radius + step ) };
        auto const column { center_column + dcolumn * ( ring_radius + step ) };
        auto const priority {
            section_manhattan_sections( row, column, center_row, center_column ) };
        if( enqueue_section_if_needed( row, column, priority ) ) {
            pack_bench_inc( &Eu7PackBench::stream_lookahead_enqueue );
        }
        if( stream_backpressure() ) {
            break;
        }
    }
}

[[nodiscard]] glm::dvec3
guess_initial_stream_position( Eu7Module const &root_module ) {
    auto const &saved_camera { Global.FreeCameraInit[ 0 ] };
    if( saved_camera.x != 0.0 || saved_camera.y != 0.0 || saved_camera.z != 0.0 ) {
        return saved_camera;
    }

    if(
        false == Global.local_start_vehicle.empty() &&
        Global.local_start_vehicle != "ghostview" ) {
        if( auto *vehicle { simulation::Vehicles.find( Global.local_start_vehicle ) };
            vehicle != nullptr ) {
            return vehicle->GetPosition();
        }
    }

    for( auto const &trainset : root_module.scene.trainsets ) {
        if( trainset.track.empty() ) {
            continue;
        }
        if( auto *track { simulation::Paths.find( trainset.track ) }; track != nullptr ) {
            return glm::dvec3 { track->location() };
        }
    }

    for( auto *vehicle : simulation::Vehicles.sequence() ) {
        if( vehicle != nullptr ) {
            return vehicle->GetPosition();
        }
    }

    return {};
}

[[nodiscard]] bool
try_dequeue_ready_batch() {
    if( g_stream.pending_apply.has_value() ) {
        return true;
    }

    PackSectionReady batch;
    bool has_batch { false };
    {
        std::lock_guard<std::mutex> lock { g_stream.ready.mutex };
        if( false == g_stream.ready.data.empty() ) {
            auto const best_it {
                std::min_element(
                    g_stream.ready.data.begin(),
                    g_stream.ready.data.end(),
                    []( PackSectionReady const &lhs, PackSectionReady const &rhs ) {
                        auto const lhs_dist {
                            section_manhattan_sections(
                                lhs.row,
                                lhs.column,
                                g_stream.center_row,
                                g_stream.center_column ) };
                        auto const rhs_dist {
                            section_manhattan_sections(
                                rhs.row,
                                rhs.column,
                                g_stream.center_row,
                                g_stream.center_column ) };
                        if( lhs_dist != rhs_dist ) {
                            return lhs_dist < rhs_dist;
                        }
                        if( lhs.section_idx != rhs.section_idx ) {
                            return lhs.section_idx < rhs.section_idx;
                        }
                        return lhs.subchunk_index < rhs.subchunk_index;
                    } ) };
            batch = std::move( *best_it );
            g_stream.ready.data.erase( best_it );
            has_batch = true;
        }
    }

    if( false == has_batch ) {
        return false;
    }

    if( batch.failed || batch.models == nullptr || batch.models->empty() ) {
        if( batch.failed ) {
            fail_section( batch.section_idx );
        }
        else {
            finalize_section( batch );
        }
        return false;
    }

    g_stream.pending_apply = std::move( batch );
    g_stream.pending_apply_offset = 0;
    return true;
}

[[nodiscard]] bool
gameplay_stream_mode() {
    return g_loading_screen_dismissed
        && false == g_stream.bootstrap_active
        && false == g_stream.bootstrap_pending;
}

void
replenish_bootstrap_ring( glm::dvec3 const &world_position );

void
note_drain_idle() {
    if( ready_queue_size() > 0 || false == g_stream.in_flight_sections.empty() ) {
        pack_bench_inc( &Eu7PackBench::main_drain_wait_worker );
    }
    else {
        pack_bench_inc( &Eu7PackBench::main_drain_idle );
    }
}

void
drain_apply_budget(
    double const budget_ms,
    std::size_t const max_instances,
    std::size_t const max_cold_meshes,
    double const cold_budget_ms,
    std::size_t const max_chunks ) {
    if( false == g_stream.active || g_stream.serializer == nullptr || budget_ms <= 0.0 ) {
        return;
    }

    pack_bench_inc( &Eu7PackBench::main_drain_calls );
    pack_bench_note_ready_queue( ready_queue_size() );
    pack_bench_note_in_flight( g_stream.in_flight_sections.size() );

    if( false == apply_pending_chunk(
            budget_ms,
            max_instances,
            max_cold_meshes,
            cold_budget_ms,
            max_chunks ) ) {
        note_drain_idle();
    }
}

void
drain_sections( std::size_t const max_sections ) {
    if( false == g_stream.active || g_stream.serializer == nullptr || max_sections == 0 ) {
        return;
    }

    pack_bench_inc( &Eu7PackBench::main_drain_calls );
    pack_bench_note_ready_queue( ready_queue_size() );
    pack_bench_note_in_flight( g_stream.in_flight_sections.size() );

    bool applied_work { false };
    for( std::size_t i { 0 }; i < max_sections; ++i ) {
        if( false == apply_pending_chunk(
                0.0,
                kLoaderSliceInstances,
                kLoaderSliceColdMeshes,
                0.0,
                std::numeric_limits<std::size_t>::max() ) ) {
            break;
        }
        applied_work = true;
    }

    if( false == applied_work ) {
        note_drain_idle();
    }
}

void
drain_until_budget( double const budget_ms ) {
    if( budget_ms <= 0.0 ) {
        return;
    }

    auto const started { std::chrono::steady_clock::now() };
    while( true ) {
        auto const elapsed_ms {
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started ).count() };
        auto const remaining_ms { budget_ms - elapsed_ms };
        if( remaining_ms <= 0.0 ) {
            break;
        }

        if( false == apply_pending_chunk(
                remaining_ms,
                kLoaderSliceInstances,
                kLoaderSliceColdMeshes,
                0.0,
                std::numeric_limits<std::size_t>::max() ) ) {
            break;
        }

        if(
            false == g_stream.pending_apply.has_value() &&
            ready_queue_size() == 0 ) {
            break;
        }
    }
}

void
sync_stream_limits( glm::dvec3 const &world_position ) {
    if( false == g_loading_screen_dismissed ) {
        g_stream_catchup = false;
        g_stream_max_in_flight = 6;
        g_stream_max_ready = 2;
        return;
    }

    auto const inner_progress {
        section_stream_ring_progress( world_position, kSectionStreamGameplayRadiusKm ) };
    g_stream_catchup =
        inner_progress < 0.99f
        || false == section_stream_ready_around(
            world_position,
            kSectionStreamGameplayRadiusKm );
    g_stream_max_in_flight = g_stream_catchup ?
        kCatchupMaxInFlightSections :
        kMaxInFlightSections;
    g_stream_max_ready = g_stream_catchup ?
        kCatchupMaxReadySections :
        kMaxReadySections;
}

} // namespace

void
init_section_stream(
    Eu7Module const &root_module,
    std::string const &resolved_path,
    simulation::state_serializer &serializer ) {
    stop_workers();
    reset_stream_fields();

    g_stream.module = &root_module;
    g_stream.path = resolved_path;
    g_stream.serializer = &serializer;
    g_stream.active = root_module.has_pack_chunk;

    g_stream.mesh_cache.reserve( 1024 );
    g_stream.nodedata_cache.reserve( 1024 );

    if( g_stream.active ) {
        g_loading_block_started = std::chrono::steady_clock::now();
        g_stream.bootstrap_pending = true;
        WriteLog(
            "EU7 PACK: streaming wlaczony, plik=\"" + resolved_path + "\", sekcji w indeksie=" +
            std::to_string( root_module.pack_catalog.entries.size() ) +
            ", payload=" + std::to_string( root_module.pack_catalog.pack_payload_size ) + " B" );
        start_workers();
    }
}

void
prime_section_stream( Eu7Module const &root_module ) {
    if( false == g_stream.active ) {
        return;
    }

    auto const initial { guess_initial_stream_position( root_module ) };
    if( initial.x == 0.0 && initial.y == 0.0 && initial.z == 0.0 ) {
        WriteLog( "EU7 PACK: brak pozycji startowej — bootstrap po zaladowaniu scenariusza" );
        return;
    }

    auto const [row, column] { section_row_column( initial ) };
    g_stream.center_row = row;
    g_stream.center_column = column;

    WriteLog(
        "EU7 PACK: pre-warm sekcji " + std::to_string( row ) + "," + std::to_string( column ) +
        " (" + std::to_string( initial.x ) + "," + std::to_string( initial.y ) + "," +
        std::to_string( initial.z ) + ")" );

    g_stream.bootstrap_active = true;
    enqueue_sections_around( row, column, kInitialBootstrapRadius, initial );
    g_stream.bootstrap_active = false;
    g_stream.bootstrap_pending = false;
}

[[nodiscard]] glm::dvec3
resolve_stream_position() {
    if(
        simulation::Train != nullptr &&
        simulation::Train->Dynamic() != nullptr ) {
        return simulation::Train->Dynamic()->GetPosition();
    }

    if(
        false == Global.local_start_vehicle.empty() &&
        Global.local_start_vehicle != "ghostview" ) {
        if( auto *vehicle { simulation::Vehicles.find( Global.local_start_vehicle ) };
            vehicle != nullptr ) {
            return vehicle->GetPosition();
        }
    }

    auto const &saved_camera { Global.FreeCameraInit[ 0 ] };
    if( saved_camera.x != 0.0 || saved_camera.y != 0.0 || saved_camera.z != 0.0 ) {
        return saved_camera;
    }

    for( auto *vehicle : simulation::Vehicles.sequence() ) {
        if( vehicle != nullptr ) {
            return vehicle->GetPosition();
        }
    }

    return {};
}

void
bootstrap_section_stream( glm::dvec3 const &world_position ) {
    if( false == g_stream.active ) {
        return;
    }

    auto const [row, column] { section_row_column( world_position ) };
    g_stream.center_row = row;
    g_stream.center_column = column;

    WriteLog(
        "EU7 PACK: bootstrap " + std::to_string( kInitialBootstrapRadius ) + "km, sekcja " +
        std::to_string( row ) + "," + std::to_string( column ) );

    auto const started { std::chrono::steady_clock::now() };
    g_stream.bootstrap_active = true;
    enqueue_sections_around( row, column, kInitialBootstrapRadius, world_position );

    while( false == is_bootstrap_ring_loaded( row, column, kInitialBootstrapRadius ) ) {
        drain_until_budget( kBootstrapDrainMs );

        if( false == is_bootstrap_ring_loaded( row, column, kInitialBootstrapRadius ) ) {
            enqueue_sections_around( row, column, kInitialBootstrapRadius, world_position );
        }

        auto const elapsed_ms {
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - started ).count() };
        if( static_cast<std::size_t>( elapsed_ms ) >= kBootstrapTimeoutMs ) {
            ErrorLog( "EU7 PACK: bootstrap timeout po " + std::to_string( elapsed_ms ) + " ms" );
            break;
        }

        std::this_thread::sleep_for( std::chrono::milliseconds( 8 ) );
    }

    g_stream.bootstrap_active = false;
    g_stream.bootstrap_pending = false;

    auto const elapsed_ms {
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - started ).count() };
    WriteLog(
        "EU7 PACK: bootstrap zakonczony, sekcji=" +
        std::to_string( g_stream.loaded_sections.size() ) + ", czas=" +
        std::to_string( elapsed_ms ) + " ms" );
}

void
update_section_stream( glm::dvec3 const &world_position ) {
    if( false == g_stream.active ) {
        return;
    }

    auto const [center_row, center_column] { section_row_column( world_position ) };
    auto const center_moved {
        center_row != g_stream.center_row || center_column != g_stream.center_column };
    auto const moved_since_enqueue {
        g_stream.has_last_enqueue_position ?
            glm::length( world_position - g_stream.last_enqueue_position ) :
            std::numeric_limits<double>::infinity() };

    if( moved_since_enqueue > kTeleportReenqueueDistanceM ) {
        g_stream.has_last_enqueue_position = false;
    }

    g_stream.center_row = center_row;
    g_stream.center_column = center_column;

    if( center_moved ) {
        reprioritize_job_queue();
    }

    sync_stream_limits( world_position );

    auto const ring_radius { g_stream.radius };
    auto const reenqueue_distance {
        g_stream_catchup ? kCatchupReenqueueDistanceM : kReenqueueDistanceM };
    auto max_lookahead { g_stream_catchup ? 15 : 5 };
    if( g_stream_catchup ) {
        auto const speed { camera_stream_speed_mps() };
        if( speed > 1200.0 ) {
            max_lookahead = 25;
        }
        else if( speed > 600.0 ) {
            max_lookahead = 20;
        }
    }
    auto const travel_forward { guess_travel_forward() };

    auto const current_section_unloaded {
        section_has_pack_models( center_row, center_column )
        && false == g_stream.loaded_sections.contains( section_index( center_row, center_column ) ) };
    auto const inner_ring_incomplete {
        current_section_unloaded
        || false == section_stream_ready_around( world_position, kSectionStreamGameplayRadiusKm ) };
    auto const should_reenqueue {
        center_moved
        || false == g_stream.has_last_enqueue_position
        || moved_since_enqueue >= reenqueue_distance };

    if( should_reenqueue || inner_ring_incomplete || g_stream_catchup ) {
        enqueue_sections_around( center_row, center_column, ring_radius, world_position );
        g_stream.last_enqueue_position = world_position;
        g_stream.has_last_enqueue_position = true;
        if( should_reenqueue || inner_ring_incomplete ) {
            pack_bench_inc( &Eu7PackBench::stream_reenqueue );
        }
        enqueue_movement_lookahead(
            center_row,
            center_column,
            ring_radius,
            travel_forward,
            max_lookahead );
    }
}

[[nodiscard]] bool
section_stream_drain_idle( glm::dvec3 const &world_position ) {
    if( g_stream.bootstrap_pending || g_stream.bootstrap_active ) {
        return false;
    }
    if( g_stream.pending_apply.has_value() || ready_queue_size() > 0 ) {
        return false;
    }
    if( false == g_stream.in_flight_sections.empty() || g_stream_catchup ) {
        return false;
    }
    if( false == section_stream_ready_around( world_position, kSectionStreamGameplayRadiusKm ) ) {
        return false;
    }
    return load_stats().pack_models >= 500;
}

void
drain_section_stream(
    std::size_t const max_cold_meshes,
    std::size_t const max_instances ) {
    (void)max_cold_meshes;
    (void)max_instances;

    if( false == g_stream.active ) {
        return;
    }
    if( section_stream_drain_idle( Global.pCamera.Pos ) ) {
        return;
    }

    if( gameplay_stream_mode() ) {
        auto apply_budget { gameplay_apply_budget_ms() };
        std::size_t max_chunks { 1 };
        if( g_stream_catchup ) {
            auto const inner_ring {
                section_stream_ring_progress(
                    Global.pCamera.Pos,
                    kSectionStreamGameplayRadiusKm ) };
            auto const speed { camera_stream_speed_mps() };
            if(
                inner_ring < kStationaryCatchupRingThreshold
                && speed < kStationaryCatchupSpeedMps ) {
                max_chunks = 2;
                apply_budget = std::max( apply_budget, kCatchupApplyBudgetMs * 1.25 );
            }
        }
        drain_apply_budget(
            apply_budget,
            gameplay_slice_instances(),
            gameplay_slice_cold_meshes(),
            gameplay_cold_budget_ms(),
            max_chunks );
        maybe_log_stream_status( Global.pCamera.Pos );
    }
    else if( false == g_loading_screen_dismissed ) {
        auto position { stream_loading_position() };
        if( section_stream_needs_bootstrap() ) {
            kick_section_stream_bootstrap();
        }
        else if( position.x != 0.0 || position.y != 0.0 || position.z != 0.0 ) {
            replenish_bootstrap_ring( position );
        }
        drain_until_budget( kLoaderDrainBudgetMs );
    }
    else {
        drain_until_budget( kDrainBudgetMs );
    }
}

bool
section_stream_active() {
    return g_stream.active;
}

bool
section_stream_needs_bootstrap() {
    return g_stream.active && g_stream.bootstrap_pending;
}

void
kick_section_stream_bootstrap() {
    if( false == section_stream_needs_bootstrap() ) {
        return;
    }

    auto position { resolve_stream_position() };
    if( position.x == 0.0 && position.y == 0.0 && position.z == 0.0 ) {
        position = guess_initial_stream_position( stream_module() );
    }
    if( position.x == 0.0 && position.y == 0.0 && position.z == 0.0 ) {
        WriteLog( "EU7 PACK: bootstrap async — brak pozycji startowej" );
        g_stream.bootstrap_pending = false;
        return;
    }

    auto const [row, column] { section_row_column( position ) };
    g_stream.center_row = row;
    g_stream.center_column = column;

    WriteLog(
        "EU7 PACK: bootstrap async " + std::to_string( kInitialBootstrapRadius ) + "km, sekcja " +
        std::to_string( row ) + "," + std::to_string( column ) );

    enqueue_sections_around( row, column, kInitialBootstrapRadius, position );
    g_stream.bootstrap_pending = false;
}

void
try_bootstrap_section_stream() {
    if( false == section_stream_needs_bootstrap() ) {
        return;
    }

    auto position { resolve_stream_position() };
    if( position.x == 0.0 && position.y == 0.0 && position.z == 0.0 ) {
        position = guess_initial_stream_position( stream_module() );
    }
    if( position.x == 0.0 && position.y == 0.0 && position.z == 0.0 ) {
        return;
    }

    bootstrap_section_stream( position );
}

void
preload_section_stream( double const max_drain_ms ) {
    if( false == g_stream.active ) {
        return;
    }

    simulation::State.drain_deferred_eu7_trainsets( 16.0 );

    auto position { resolve_stream_position() };
    if( position.x == 0.0 && position.y == 0.0 && position.z == 0.0 ) {
        position = guess_initial_stream_position( stream_module() );
    }

    if( section_stream_needs_bootstrap() ) {
        if( position.x != 0.0 || position.y != 0.0 || position.z != 0.0 ) {
            bootstrap_section_stream( position );
        }
    }
    else if( position.x != 0.0 || position.y != 0.0 || position.z != 0.0 ) {
        update_section_stream( position );
    }

    auto const started { std::chrono::steady_clock::now() };
    while( true ) {
        drain_until_budget( kDrainBudgetMs );
        if( position.x != 0.0 || position.y != 0.0 || position.z != 0.0 ) {
            update_section_stream( position );
        }

        auto const elapsed_ms {
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - started ).count() };
        if( static_cast<double>( elapsed_ms ) >= max_drain_ms ) {
            break;
        }
        if(
            g_stream.pending_apply.has_value() || ready_queue_size() > 0 ||
            false == g_stream.in_flight_sections.empty() ) {
            continue;
        }
        if( position.x != 0.0 || position.y != 0.0 || position.z != 0.0 ) {
            if( section_stream_ready_around( position, kInitialBootstrapRadius ) ) {
                break;
            }
        }
        if( load_stats().pack_models >= 1500 ) {
            break;
        }
        std::this_thread::sleep_for( std::chrono::milliseconds( 8 ) );
    }

    WriteLog(
        "EU7 PACK: preload sekcji=" + std::to_string( load_stats().pack_sections_loaded ) +
        " modele=" + std::to_string( load_stats().pack_models ) );
}

[[nodiscard]] bool
ring_section_in_radius(
    int const row,
    int const column,
    int const center_row,
    int const center_column,
    int const radius_km ) {
    return std::abs( row - center_row ) <= radius_km
        && std::abs( column - center_column ) <= radius_km;
}

[[nodiscard]] float
section_ring_apply_fraction( int const row, int const column ) {
    auto const section_idx { section_index( row, column ) };

    if( g_stream.loaded_sections.contains( section_idx ) ) {
        return 1.0f;
    }

    if(
        g_stream.pending_apply.has_value() &&
        g_stream.pending_apply->section_idx == section_idx &&
        g_stream.pending_apply->models != nullptr &&
        false == g_stream.pending_apply->models->empty() ) {
        auto const total { g_stream.pending_apply->models->size() };
        return static_cast<float>( g_stream.pending_apply_offset ) /
            static_cast<float>( total );
    }

    {
        std::lock_guard<std::mutex> lock { g_stream.ready.mutex };
        for( auto const &batch : g_stream.ready.data ) {
            if( batch.section_idx == section_idx ) {
                return 0.85f;
            }
        }
    }

    if( g_stream.in_flight_sections.contains( section_idx ) ) {
        return 0.15f;
    }

    return 0.f;
}

[[nodiscard]] bool
ring_has_pending_pack_work(
    int const center_row,
    int const center_column,
    int const radius_km ) {
    for( int row { center_row - radius_km }; row <= center_row + radius_km; ++row ) {
        if( row < 0 || row >= kRegionSideSectionCount ) {
            continue;
        }
        for( int column { center_column - radius_km }; column <= center_column + radius_km; ++column ) {
            if( column < 0 || column >= kRegionSideSectionCount ) {
                continue;
            }
            if( false == section_has_pack_models( row, column ) ) {
                continue;
            }
            auto const section_idx { section_index( row, column ) };
            if( g_stream.in_flight_sections.contains( section_idx ) ) {
                return true;
            }
            if(
                g_stream.pending_apply.has_value() &&
                g_stream.pending_apply->section_idx == section_idx ) {
                return true;
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock { g_stream.ready.mutex };
        for( auto const &batch : g_stream.ready.data ) {
            if( ring_section_in_radius(
                    batch.row,
                    batch.column,
                    center_row,
                    center_column,
                    radius_km )
                && section_has_pack_models( batch.row, batch.column ) ) {
                return true;
            }
        }
    }

    return false;
}

bool
section_stream_ready_around(
    glm::dvec3 const &world_position,
    int const radius_km ) {
    if( false == g_stream.active ) {
        return true;
    }

    auto const [center_row, center_column] { section_row_column( world_position ) };
    if( false == is_bootstrap_ring_loaded( center_row, center_column, radius_km ) ) {
        return false;
    }
    if( g_stream.pending_apply.has_value() ) {
        auto const &batch { *g_stream.pending_apply };
        if( ring_section_in_radius(
                batch.row,
                batch.column,
                center_row,
                center_column,
                radius_km )
            && section_has_pack_models( batch.row, batch.column )
            && batch.models != nullptr ) {
            return false;
        }
    }
    if( ring_has_pending_pack_work( center_row, center_column, radius_km ) ) {
        return false;
    }
    if( section_stream_needs_bootstrap() ) {
        return false;
    }

    return true;
}

bool
section_stream_presentable_around(
    glm::dvec3 const &world_position,
    int const radius_km ) {
    if( false == section_stream_ready_around( world_position, radius_km ) ) {
        g_ring_ready_since.reset();
        return false;
    }

    auto const now { std::chrono::steady_clock::now() };
    if( false == g_ring_ready_since.has_value() ) {
        g_ring_ready_since = now;
        return false;
    }

    return ( now - *g_ring_ready_since ) >= kPresentableHoldMs;
}

bool
loading_screen_dismissed() {
    return g_loading_screen_dismissed;
}

void
dismiss_loading_screen() {
    pack_bench_begin_stream_phase();
    g_loading_screen_dismissed = true;
    g_ring_ready_since.reset();
}

bool
loading_screen_blocks_world(
    glm::dvec3 const &world_position,
    int const radius_km ) {
    if( false == g_stream.active || g_loading_screen_dismissed ) {
        return false;
    }

    if( g_loading_block_started.has_value() ) {
        auto const blocked_for {
            std::chrono::steady_clock::now() - *g_loading_block_started };
        if( blocked_for >= kLoadingScreenMaxBlockSec ) {
            auto const ring {
                section_stream_ring_progress( world_position, radius_km ) };
            ErrorLog(
                "EU7 PACK: loading screen timeout — wchodzę w świat (ring=" +
                std::to_string( static_cast<int>( ring * 100.f ) ) + "%, ready=" +
                std::to_string( section_stream_ready_around( world_position, radius_km ) ? 1 : 0 ) +
                ", pending_apply=" +
                std::to_string( g_stream.pending_apply.has_value() ? 1 : 0 ) +
                ", ready_q=" + std::to_string( ready_queue_size() ) +
                ", in_flight=" + std::to_string( g_stream.in_flight_sections.size() ) + ")" );
            dismiss_loading_screen();
            return false;
        }
    }

    if(
        section_stream_ring_progress( world_position, radius_km ) >= 1.0f
        && section_stream_ready_around( world_position, radius_km ) ) {
        dismiss_loading_screen();
        return false;
    }

    if( section_stream_presentable_around( world_position, radius_km ) ) {
        dismiss_loading_screen();
        return false;
    }

    return true;
}

glm::dvec3
stream_loading_position() {
    if( Global.pCamera.Pos.x != 0.0 || Global.pCamera.Pos.y != 0.0 || Global.pCamera.Pos.z != 0.0 ) {
        return Global.pCamera.Pos;
    }

    auto position { resolve_stream_position() };
    if( position.x != 0.0 || position.y != 0.0 || position.z != 0.0 ) {
        return position;
    }

    return Global.FreeCameraInit[ 0 ];
}

float
section_stream_ring_progress(
    glm::dvec3 const &world_position,
    int const radius_km ) {
    if( false == g_stream.active ) {
        return 1.f;
    }

    auto const [center_row, center_column] { section_row_column( world_position ) };
    std::size_t pack_sections { 0 };
    float accumulated { 0.f };

    for( int row { center_row - radius_km }; row <= center_row + radius_km; ++row ) {
        if( row < 0 || row >= kRegionSideSectionCount ) {
            continue;
        }
        for( int column { center_column - radius_km }; column <= center_column + radius_km; ++column ) {
            if( column < 0 || column >= kRegionSideSectionCount ) {
                continue;
            }
            if( false == section_has_pack_models( row, column ) ) {
                continue;
            }
            ++pack_sections;
            accumulated += section_ring_apply_fraction( row, column );
        }
    }

    if( pack_sections == 0 ) {
        return 1.f;
    }

    return std::clamp( accumulated / static_cast<float>( pack_sections ), 0.f, 1.f );
}

void
reset_section_stream() {
    flush_pack_stream_bench();
    stop_workers();
    reset_stream_fields();
    g_ring_ready_since.reset();
    g_loading_block_started.reset();
    g_loading_screen_dismissed = false;
    g_stream_catchup = false;
    g_stream_max_in_flight = kMaxInFlightSections;
    g_stream_max_ready = kMaxReadySections;
    reset_pack_bench();
}

} // namespace scene::eu7
