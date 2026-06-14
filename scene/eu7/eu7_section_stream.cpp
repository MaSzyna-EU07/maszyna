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
#include <list>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

namespace scene::eu7 {

[[nodiscard]] bool
ring_section_in_radius(
    int row,
    int column,
    int center_row,
    int center_column,
    int radius_km );

namespace {

constexpr double kDrainBudgetMs { 12.0 };
constexpr double kLoaderDrainBudgetMs { 96.0 };
constexpr std::size_t kLoaderSectionsPerDrain { 8 };

constexpr double kGameplayApplyBudgetMs { 4.0 };
constexpr double kCatchupApplyBudgetMs { 6.0 };
constexpr std::size_t kGameplaySliceInstances { 96 };
constexpr std::size_t kCatchupSliceInstances { 64 };
constexpr std::size_t kLoaderSliceInstances { 768 };
constexpr std::size_t kGameplaySliceColdMeshes { 1 };
constexpr std::size_t kCatchupSliceColdMeshes { 2 };
constexpr std::size_t kLoaderSliceColdMeshes { 32 };
constexpr double kGameplayColdBudgetMs { 3.0 };
constexpr double kCatchupColdBudgetMs { 5.0 };

constexpr int kInitialBootstrapRadius { 5 };
constexpr int kStreamRadius { 14 };
constexpr int kSectionUnloadMarginKm { 2 };
constexpr std::size_t kMeshCacheLruCap { 2000 };
constexpr std::size_t kNodedataCacheLruCap { 2000 };
constexpr int kMovementLookahead { 10 };
constexpr std::size_t kMaxPackStreamWorkers { 8 };
constexpr std::size_t kMaxInFlightSections { 12 };
constexpr std::size_t kMaxReadySections { 6 };
constexpr double kStreamStatusLogIntervalSec { 5.0 };
constexpr double kReenqueueDistanceM { 500.0 };
constexpr double kCatchupReenqueueDistanceM { 40.0 };
constexpr std::size_t kHeavySectionModelThreshold { 32 };
constexpr std::size_t kPackTextureWarmSlice { 8 };
constexpr double kPackTextureWarmBudgetMs { 4.0 };
constexpr double kUrgentApplyBudgetMs { 32.0 };
constexpr std::size_t kUrgentSliceInstances { 96 };
constexpr std::size_t kUrgentSliceColdMeshes { 2 };
constexpr double kUrgentColdBudgetMs { 24.0 };
constexpr std::size_t kUrgentMaxChunksPerDrain { 8 };
constexpr double kReadyTexturePrefetchBudgetMs { 14.0 };
constexpr std::size_t kReadyTexturePrefetchSlice { 256 };
constexpr int kUmesPrefetchMaxDistanceKm { 2 };
constexpr double kReadyUmesPrefetchBudgetMs { 8.0 };
constexpr std::size_t kReadyUmesPrefetchMaxMeshes { 6 };
constexpr std::size_t kUmesPrefetchJobsBacklogThreshold { 4 };
constexpr std::size_t kCatchupMaxInFlightSections { 28 };
constexpr std::size_t kCatchupMaxReadySections { 14 };
constexpr int kUrgentSectionDistanceKm { 1 };
constexpr int kFastFlightApplyMaxDistanceKm { 2 };
constexpr int kFarApplySlowDistanceKm { 4 };
constexpr int kPreemptPendingDistanceKm { 2 };
constexpr double kTeleportReenqueueDistanceM { 2000.0 };
constexpr float kStationaryCatchupRingThreshold { 0.70f };
constexpr double kStationaryCatchupSpeedMps { 5.0 };
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
    std::uint32_t next_chunk { 0 };
};

struct PackSectionReady {
    int row { 0 };
    int column { 0 };
    std::size_t section_idx { 0 };
    std::unique_ptr<std::vector<Eu7Model>> models;
    std::vector<std::string> unique_meshes;
    std::vector<std::string> unique_textures;
    bool failed { false };
    bool section_final { true };
    std::size_t apply_offset { 0 };
    std::size_t texture_warm_offset { 0 };
    std::size_t umes_cold_offset { 0 };
    std::size_t umes_prefetch_offset { 0 };
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
    glm::dvec3 anchor_position {};
    bool has_anchor_position { false };

    std::unordered_map<std::string, TModel3d *> mesh_cache;
    std::unordered_map<std::string, scene::node_data> nodedata_cache;
    std::list<std::string> mesh_lru;
    std::unordered_map<std::string, std::list<std::string>::iterator> mesh_lru_iters;
    std::unordered_map<std::string, std::size_t> mesh_ref_counts;
    std::unordered_map<std::size_t, std::unordered_set<std::string>> section_mesh_keys;
    std::list<std::string> nodedata_lru;
    std::unordered_map<std::string, std::list<std::string>::iterator> nodedata_lru_iters;
    std::unordered_map<std::string, std::size_t> nodedata_ref_counts;
    std::unordered_map<std::size_t, std::unordered_set<std::string>> section_nodedata_keys;

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

void
maybe_unload_distant_sections( int const center_row, int const center_column );

void
evict_unreferenced_stream_caches();

[[nodiscard]] bool
try_dequeue_ready_batch();

void
finalize_section( PackSectionReady const &batch );

void
fail_section( std::size_t const section_idx, int const row = -1, int const column = -1 );

void
enqueue_failed_section( PackSectionJob const &job );

void
release_pending_buffer();

void
prefetch_ready_queue_umes_worker( double budget_ms, std::size_t max_meshes );

[[nodiscard]] std::size_t
warm_pack_section_textures(
    PackSectionReady &batch,
    std::size_t const model_offset,
    std::size_t const model_count,
    std::size_t const max_textures = kPackTextureWarmSlice ) {
    if( false == batch.unique_textures.empty() ) {
        if( batch.texture_warm_offset >= batch.unique_textures.size() ) {
            return 0;
        }
        auto const remaining { batch.unique_textures.size() - batch.texture_warm_offset };
        auto const slice { std::min( remaining, max_textures ) };
        std::size_t processed { 0 };
        auto const warmed {
            warm_pack_texture_paths_main(
                batch.unique_textures.data() + batch.texture_warm_offset,
                slice,
                kPackTextureWarmBudgetMs,
                &processed ) };
        batch.texture_warm_offset += processed;
        return warmed;
    }

    if( batch.models == nullptr || model_count == 0 ) {
        return 0;
    }
    auto const begin { model_offset };
    if( begin >= batch.models->size() ) {
        return 0;
    }
    auto const remaining { std::min( model_count, batch.models->size() - begin ) };
    auto const warmed {
        warm_pack_textures_main( batch.models->data() + begin, remaining ) };
    batch.texture_warm_offset = begin + remaining;
    return warmed;
}

[[nodiscard]] bool
section_has_pack_models( int const row, int const column );

[[nodiscard]] std::size_t
pending_section_total();

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
touch_string_lru(
    std::string const &key,
    std::list<std::string> &order,
    std::unordered_map<std::string, std::list<std::string>::iterator> &iters ) {
    auto const found { iters.find( key ) };
    if( found == iters.end() ) {
        order.push_back( key );
        iters.emplace( key, std::prev( order.end() ) );
        return;
    }
    order.splice( order.end(), order, found->second );
}

void
retain_cache_key(
    std::size_t const section_idx,
    std::string key,
    std::unordered_map<std::size_t, std::unordered_set<std::string>> &section_keys,
    std::unordered_map<std::string, std::size_t> &ref_counts,
    std::list<std::string> &lru_order,
    std::unordered_map<std::string, std::list<std::string>::iterator> &lru_iters ) {
    if( key.empty() ) {
        return;
    }
    auto &keys { section_keys[ section_idx ] };
    if( false == keys.insert( key ).second ) {
        touch_string_lru( key, lru_order, lru_iters );
        return;
    }
    ++ref_counts[ key ];
    touch_string_lru( key, lru_order, lru_iters );
}

void
release_section_key_refs(
    std::size_t const section_idx,
    std::unordered_map<std::size_t, std::unordered_set<std::string>> &section_keys,
    std::unordered_map<std::string, std::size_t> &ref_counts ) {
    auto const found { section_keys.find( section_idx ) };
    if( found == section_keys.end() ) {
        return;
    }

    for( auto const &key : found->second ) {
        auto ref_it { ref_counts.find( key ) };
        if( ref_it == ref_counts.end() ) {
            continue;
        }
        if( ref_it->second <= 1 ) {
            ref_counts.erase( ref_it );
        }
        else {
            --ref_it->second;
        }
    }
    section_keys.erase( found );
}

void
evict_cache_lru_if_needed(
    std::unordered_map<std::string, std::size_t> &ref_counts,
    std::unordered_map<std::string, TModel3d *> *const mesh_cache,
    std::unordered_map<std::string, scene::node_data> *const nodedata_cache,
    std::list<std::string> &lru_order,
    std::unordered_map<std::string, std::list<std::string>::iterator> &lru_iters,
    std::size_t const cap,
    std::uint64_t Eu7PackBench::*const eviction_counter ) {
    while( lru_order.size() > cap ) {
        bool evicted { false };
        for( auto it { lru_order.begin() }; it != lru_order.end(); ++it ) {
            auto const &key { *it };
            auto const ref_it { ref_counts.find( key ) };
            if( ref_it != ref_counts.end() && ref_it->second > 0 ) {
                continue;
            }
            lru_iters.erase( key );
            lru_order.erase( it );
            if( mesh_cache != nullptr ) {
                mesh_cache->erase( key );
            }
            if( nodedata_cache != nullptr ) {
                nodedata_cache->erase( key );
            }
            pack_bench_inc( eviction_counter );
            evicted = true;
            break;
        }
        if( false == evicted ) {
            break;
        }
    }
}

void
retain_section_mesh_keys(
    std::size_t const section_idx,
    Eu7Model const *const models,
    std::size_t const offset,
    std::size_t const count ) {
    if( models == nullptr || count == 0 ) {
        return;
    }
    for( std::size_t i { 0 }; i < count; ++i ) {
        auto model_file { models[ offset + i ].model_file };
        if( model_file.empty() || model_file == "notload" ) {
            continue;
        }
        replace_slashes( model_file );
        retain_cache_key(
            section_idx,
            model_file,
            g_stream.section_mesh_keys,
            g_stream.mesh_ref_counts,
            g_stream.mesh_lru,
            g_stream.mesh_lru_iters );
    }
}

void
retain_section_nodedata_keys(
    std::size_t const section_idx,
    Eu7Model const *const models,
    std::size_t const offset,
    std::size_t const count ) {
    if( models == nullptr || count == 0 ) {
        return;
    }
    for( std::size_t i { 0 }; i < count; ++i ) {
        retain_cache_key(
            section_idx,
            pack_nodedata_cache_key( models[ offset + i ] ),
            g_stream.section_nodedata_keys,
            g_stream.nodedata_ref_counts,
            g_stream.nodedata_lru,
            g_stream.nodedata_lru_iters );
    }
}

void
release_section_cache_refs( std::size_t const section_idx ) {
    release_section_key_refs(
        section_idx,
        g_stream.section_mesh_keys,
        g_stream.mesh_ref_counts );
    release_section_key_refs(
        section_idx,
        g_stream.section_nodedata_keys,
        g_stream.nodedata_ref_counts );
    evict_unreferenced_stream_caches();
}

void
evict_unreferenced_stream_caches() {
    evict_cache_lru_if_needed(
        g_stream.mesh_ref_counts,
        &g_stream.mesh_cache,
        nullptr,
        g_stream.mesh_lru,
        g_stream.mesh_lru_iters,
        kMeshCacheLruCap,
        &Eu7PackBench::stream_mesh_cache_evictions );
    evict_cache_lru_if_needed(
        g_stream.nodedata_ref_counts,
        nullptr,
        &g_stream.nodedata_cache,
        g_stream.nodedata_lru,
        g_stream.nodedata_lru_iters,
        kNodedataCacheLruCap,
        &Eu7PackBench::stream_nodedata_cache_evictions );
}

[[nodiscard]] bool
ensure_stream_pack_mesh( std::string model_file ) {
    if( model_file.empty() || model_file == "notload" ) {
        return false;
    }
    replace_slashes( model_file );
    (void)ensure_pack_mesh_in_session_cache( model_file, g_stream.mesh_cache );
    touch_string_lru( model_file, g_stream.mesh_lru, g_stream.mesh_lru_iters );
    return true;
}

void
cancel_section_stream_work( std::size_t const section_idx ) {
    {
        std::lock_guard<std::mutex> lock { g_stream.jobs.mutex };
        auto &jobs { g_stream.jobs.data };
        jobs.erase(
            std::remove_if(
                jobs.begin(),
                jobs.end(),
                [&]( PackSectionJob const &job ) {
                    return job.section_idx == section_idx;
                } ),
            jobs.end() );
    }
    {
        std::lock_guard<std::mutex> lock { g_stream.ready.mutex };
        for( auto it { g_stream.ready.data.begin() }; it != g_stream.ready.data.end(); ) {
            if( it->section_idx != section_idx ) {
                ++it;
                continue;
            }
            if( it->models != nullptr ) {
                it->models->clear();
                it->models->shrink_to_fit();
                it->models.reset();
            }
            it = g_stream.ready.data.erase( it );
        }
    }
    if(
        g_stream.pending_apply.has_value() &&
        g_stream.pending_apply->section_idx == section_idx ) {
        release_pending_buffer();
    }
    g_stream.in_flight_sections.erase( section_idx );
    g_stream.loaded_sections.erase( section_idx );
}

[[nodiscard]] std::size_t
unload_pack_section( int const row, int const column ) {
    auto const section_idx { section_index( row, column ) };
    if( false == g_stream.loaded_sections.contains( section_idx ) ) {
        return 0;
    }

    std::size_t removed { 0 };
    if( g_stream.serializer != nullptr ) {
        removed = g_stream.serializer->unload_eu7_pack_section( row, column );
    }
    release_section_cache_refs( section_idx );
    cancel_section_stream_work( section_idx );

    if( removed > 0 ) {
        if( load_stats().pack_models >= removed ) {
            load_stats().pack_models -= removed;
        }
        else {
            load_stats().pack_models = 0;
        }
        pack_bench_inc( &Eu7PackBench::stream_unload_instances, removed );
    }
    pack_bench_inc( &Eu7PackBench::stream_sections_unloaded );
    return removed;
}

void
maybe_unload_distant_sections( int const center_row, int const center_column ) {
    if( false == g_loading_screen_dismissed || g_stream.bootstrap_active ) {
        return;
    }

    auto const keep_radius { g_stream.radius + kSectionUnloadMarginKm };
    std::vector<std::size_t> to_unload;
    to_unload.reserve( g_stream.loaded_sections.size() );

    for( auto const section_idx : g_stream.loaded_sections ) {
        auto const row {
            static_cast<int>( section_idx / static_cast<std::size_t>( kRegionSideSectionCount ) ) };
        auto const column {
            static_cast<int>( section_idx % static_cast<std::size_t>( kRegionSideSectionCount ) ) };
        if( ring_section_in_radius( row, column, center_row, center_column, keep_radius ) ) {
            continue;
        }
        to_unload.push_back( section_idx );
    }

    if( to_unload.empty() ) {
        return;
    }

    static double s_last_unload_log { 0.0 };
    auto const now { Timer::GetTime() };
    auto const should_log {
        now - s_last_unload_log >= kStreamStatusLogIntervalSec };

    std::size_t total_removed { 0 };
    for( auto const section_idx : to_unload ) {
        auto const row {
            static_cast<int>( section_idx / static_cast<std::size_t>( kRegionSideSectionCount ) ) };
        auto const column {
            static_cast<int>( section_idx % static_cast<std::size_t>( kRegionSideSectionCount ) ) };
        total_removed += unload_pack_section( row, column );
    }

    if( should_log && total_removed > 0 ) {
        s_last_unload_log = now;
        WriteLog(
            "EU7 PACK: unload " + std::to_string( to_unload.size() ) + " sekcji, inst=" +
            std::to_string( total_removed ) + ", loaded=" +
            std::to_string( g_stream.loaded_sections.size() ) + ", mesh_cache=" +
            std::to_string( g_stream.mesh_cache.size() ) );
    }
}

void
reset_stream_fields();

void
note_apply_progress();

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
    g_stream.anchor_position = {};
    g_stream.has_anchor_position = false;
    g_stream.mesh_cache.clear();
    g_stream.nodedata_cache.clear();
    g_stream.mesh_lru.clear();
    g_stream.mesh_lru_iters.clear();
    g_stream.mesh_ref_counts.clear();
    g_stream.section_mesh_keys.clear();
    g_stream.nodedata_lru.clear();
    g_stream.nodedata_lru_iters.clear();
    g_stream.nodedata_ref_counts.clear();
    g_stream.section_nodedata_keys.clear();
    reset_pack_texture_warm_cache();
    note_apply_progress();
}

[[nodiscard]] Eu7Module const &
stream_module() {
    if( g_stream.module == nullptr ) {
        throw std::runtime_error( "EU7 PACK: stream module nie jest ustawiony" );
    }
    return *g_stream.module;
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
camera_section_needs_priority() {
    if( g_stream.center_row < 0 || g_stream.center_column < 0 ) {
        return false;
    }
    if( camera_stream_speed_mps() <= 200.0 ) {
        return false;
    }
    auto const section_idx { section_index( g_stream.center_row, g_stream.center_column ) };
    if( g_stream.loaded_sections.contains( section_idx ) ) {
        return false;
    }
    return section_has_pack_models( g_stream.center_row, g_stream.center_column );
}

[[nodiscard]] bool
should_throttle_far_stream_jobs( int const row, int const column ) {
    if( false == camera_section_needs_priority() ) {
        return false;
    }
    if( row == g_stream.center_row && column == g_stream.center_column ) {
        return false;
    }
    return section_manhattan_sections(
               row,
               column,
               g_stream.center_row,
               g_stream.center_column ) > kFastFlightApplyMaxDistanceKm;
}

[[nodiscard]] bool
stream_backpressure() {
    if( g_stream.bootstrap_active ) {
        return false;
    }
    if( camera_section_needs_priority() ) {
        if( g_stream.in_flight_sections.size() >= 6 ) {
            return true;
        }
        if( ready_queue_size() >= 4 ) {
            return true;
        }
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

[[nodiscard]] bool pending_apply_is_urgent();
[[nodiscard]] double gameplay_apply_budget_ms();

[[nodiscard]] double
ready_texture_prefetch_budget_ms() {
    auto const speed { camera_stream_speed_mps() };
    if(
        speed > 500.0 &&
        pending_apply_is_urgent() ) {
        return 0.0;
    }

    double budget { kReadyTexturePrefetchBudgetMs * 0.5 };
    if( g_stream_catchup || speed > 80.0 ) {
        budget = kReadyTexturePrefetchBudgetMs;
    }

    if( g_stream_catchup ) {
        auto const apply_budget {
            pending_apply_is_urgent() ?
                kUrgentApplyBudgetMs :
                gameplay_apply_budget_ms() };
        return std::min( budget, apply_budget * 0.25 );
    }
    return std::min( budget, kDrainBudgetMs * 0.35 );
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

[[nodiscard]] bool
pending_section_near_camera( int const max_distance_km = kUrgentSectionDistanceKm ) {
    if( false == g_stream.pending_apply.has_value() ) {
        return false;
    }
    auto const &batch { *g_stream.pending_apply };
    return section_manhattan_sections(
               batch.row,
               batch.column,
               g_stream.center_row,
               g_stream.center_column ) <= max_distance_km;
}

[[nodiscard]] int
pending_section_distance_km() {
    if( false == g_stream.pending_apply.has_value() ) {
        return std::numeric_limits<int>::max();
    }
    auto const &batch { *g_stream.pending_apply };
    return section_manhattan_sections(
        batch.row,
        batch.column,
        g_stream.center_row,
        g_stream.center_column );
}

[[nodiscard]] bool
pending_apply_is_urgent() {
    auto const speed { camera_stream_speed_mps() };
    if( speed <= 80.0 && false == g_stream_catchup ) {
        return false;
    }
    auto const max_dist {
        speed > 600.0 ? kFastFlightApplyMaxDistanceKm : kUrgentSectionDistanceKm };
    return pending_section_near_camera( max_dist );
}

[[nodiscard]] std::size_t
scene_apply_pressure_slice_cap() {
    auto const applied { load_stats().pack_models };
    if( applied >= 200000 ) {
        return 16;
    }
    if( applied >= 120000 ) {
        return 24;
    }
    if( applied >= 80000 ) {
        return 32;
    }
    if( applied >= 40000 ) {
        return 48;
    }
    return std::numeric_limits<std::size_t>::max();
}

[[nodiscard]] std::size_t
adaptive_slice_instances( std::size_t const section_total, bool const urgent = false ) {
    if( urgent ) {
        auto limit { kUrgentSliceInstances };
        auto const speed { camera_stream_speed_mps() };
        if( speed > 1200.0 ) {
            limit = std::min( limit, std::size_t { 64 } );
        }
        else if( speed > 600.0 ) {
            limit = std::min( limit, std::size_t { 96 } );
        }
        if( section_total > 4000 ) {
            limit = std::min( limit, std::size_t { 64 } );
        }
        else if( section_total > 2000 ) {
            limit = std::min( limit, std::size_t { 96 } );
        }
        auto const last_ms { pack_bench_stream().last_chunk_ms };
        if( last_ms >= 24.0 ) {
            limit = std::min( limit, std::size_t { 32 } );
        }
        else if( last_ms >= 12.0 ) {
            limit = std::min( limit, std::size_t { 48 } );
        }
        limit = std::min( limit, scene_apply_pressure_slice_cap() );
        auto const last_apply { pack_bench_stream().last_apply_ms };
        if( last_apply >= 20.0 ) {
            limit = std::min( limit, std::size_t { 16 } );
        }
        else if( last_apply >= 12.0 ) {
            limit = std::min( limit, std::size_t { 24 } );
        }
        return std::max( limit, std::size_t { 16 } );
    }

    auto limit { gameplay_slice_instances() };
    if( g_stream_catchup ) {
        auto const speed { camera_stream_speed_mps() };
        if( speed > 1200.0 ) {
            limit = std::min( limit, kCatchupSliceInstances );
        }
        else if( speed > 600.0 ) {
            limit = std::min( limit, std::size_t { 48 } );
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

    auto const last_ms { pack_bench_stream().last_chunk_ms };
    if( last_ms >= 24.0 ) {
        limit = std::min( limit, std::size_t { 32 } );
    }
    else if( last_ms >= 12.0 ) {
        limit = std::min( limit, std::size_t { 64 } );
    }

    auto const last_apply { pack_bench_stream().last_apply_ms };
    if( last_apply >= 24.0 ) {
        limit = std::min( limit, std::size_t { 16 } );
    }
    else if( last_apply >= 16.0 ) {
        limit = std::min( limit, std::size_t { 24 } );
    }
    else if( last_apply >= 10.0 ) {
        limit = std::min( limit, std::size_t { 32 } );
    }

    limit = std::min( limit, scene_apply_pressure_slice_cap() );

    return std::max( limit, std::size_t { 16 } );
}

[[nodiscard]] std::size_t
adaptive_cold_meshes( bool const urgent = false ) {
    if( urgent ) {
        return kUrgentSliceColdMeshes;
    }
    auto limit { gameplay_slice_cold_meshes() };
    if( pack_bench_stream().last_chunk_ms >= 12.0 ) {
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
            pack_bench_inc( &Eu7PackBench::stream_mesh_session_hit );
            continue;
        }
        if( cold_loaded >= max_cold_meshes ) {
            pack_bench_inc( &Eu7PackBench::stream_cold_slice_truncated );
            slice_count = i;
            break;
        }
        if( cold_budget_ms > 0.0 && cold_loaded > 0 ) {
            auto const elapsed_ms {
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - cold_started ).count() };
            if( elapsed_ms >= cold_budget_ms ) {
                pack_bench_inc( &Eu7PackBench::stream_cold_slice_truncated );
                slice_count = i;
                break;
            }
        }
        {
            PackBenchTimer const load_timer { &Eu7PackBench::main_cold_getmodel_ms };
            (void)ensure_stream_pack_mesh( model_file );
            pack_bench_inc( &Eu7PackBench::main_cold_getmodel_calls );
        }
        ++cold_loaded;
    }

    if( slice_count == 0 && count > 0 ) {
        auto model_file { models[ 0 ].model_file };
        if( model_file.empty() || model_file == "notload" ) {
            slice_count = 1;
            return 0;
        }
        replace_slashes( model_file );
        if( false == g_stream.mesh_cache.contains( model_file ) ) {
            {
                PackBenchTimer const load_timer { &Eu7PackBench::main_cold_getmodel_ms };
                (void)ensure_stream_pack_mesh( model_file );
                pack_bench_inc( &Eu7PackBench::main_cold_getmodel_calls );
            }
            cold_loaded = 1;
        }
        slice_count = 1;
    }
    return cold_loaded;
}

[[nodiscard]] std::size_t
preload_umes_cold_meshes(
    std::vector<std::string> const &unique_meshes,
    std::size_t const offset,
    std::size_t const max_cold_meshes,
    double const cold_budget_ms,
    std::size_t &offset_out ) {
    offset_out = offset;
    if( unique_meshes.empty() || offset >= unique_meshes.size() ) {
        return 0;
    }

    std::size_t cold_loaded { 0 };
    auto const cold_started { std::chrono::steady_clock::now() };
    for( std::size_t i { offset }; i < unique_meshes.size(); ++i ) {
        auto model_file { unique_meshes[ i ] };
        if( model_file.empty() || model_file == "notload" ) {
            offset_out = i + 1;
            continue;
        }
        replace_slashes( model_file );
        if( g_stream.mesh_cache.contains( model_file ) ) {
            pack_bench_inc( &Eu7PackBench::stream_mesh_session_hit );
            offset_out = i + 1;
            continue;
        }
        if( cold_loaded >= max_cold_meshes ) {
            pack_bench_inc( &Eu7PackBench::stream_cold_slice_truncated );
            break;
        }
        if( cold_budget_ms > 0.0 && cold_loaded > 0 ) {
            auto const elapsed_ms {
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - cold_started ).count() };
            if( elapsed_ms >= cold_budget_ms ) {
                pack_bench_inc( &Eu7PackBench::stream_cold_slice_truncated );
                break;
            }
        }
        {
            PackBenchTimer const load_timer { &Eu7PackBench::main_cold_getmodel_ms };
            (void)ensure_stream_pack_mesh( model_file );
            pack_bench_inc( &Eu7PackBench::main_cold_getmodel_calls );
        }
        ++cold_loaded;
        offset_out = i + 1;
    }

    return cold_loaded;
}

constexpr int kApplyStuckSkipFrames { 20 };

std::size_t g_apply_stuck_offset { std::numeric_limits<std::size_t>::max() };
int g_apply_stuck_frames { 0 };

void
note_apply_progress() {
    g_apply_stuck_offset = std::numeric_limits<std::size_t>::max();
    g_apply_stuck_frames = 0;
}

[[nodiscard]] bool
maybe_skip_stuck_apply_offset() {
    if( false == g_stream.pending_apply.has_value() ) {
        return false;
    }

    auto const offset { g_stream.pending_apply_offset };
    if( offset == g_apply_stuck_offset ) {
        ++g_apply_stuck_frames;
    }
    else {
        g_apply_stuck_offset = offset;
        g_apply_stuck_frames = 0;
    }

    if( g_apply_stuck_frames < kApplyStuckSkipFrames ) {
        return false;
    }

    auto const total {
        g_stream.pending_apply->models != nullptr ?
            g_stream.pending_apply->models->size() :
            0 };
    if( offset >= total ) {
        return false;
    }

    auto model_file { ( *g_stream.pending_apply->models )[ offset ].model_file };
    WriteLog(
        "EU7 PACK: skip stuck model offset=" + std::to_string( offset ) + "/" +
        std::to_string( total ) + " sec=" + std::to_string( g_stream.pending_apply->row ) + "," +
        std::to_string( g_stream.pending_apply->column ) +
        ( model_file.empty() ? "" : " file=\"" + model_file + "\"" ) );
    if( false == model_file.empty() && model_file != "notload" ) {
        replace_slashes( model_file );
        touch_string_lru( model_file, g_stream.mesh_lru, g_stream.mesh_lru_iters );
        g_stream.mesh_cache.emplace( std::move( model_file ), nullptr );
    }
    g_stream.pending_apply_offset = offset + 1;
    note_apply_progress();
    pack_bench_inc( &Eu7PackBench::stream_apply_stuck_skip );
    return true;
}

void
maybe_preempt_distant_pending() {
    if(
        false == g_loading_screen_dismissed ||
        g_stream.bootstrap_active ||
        g_stream.bootstrap_pending ||
        false == g_stream.pending_apply.has_value() ) {
        return;
    }

    auto &batch { *g_stream.pending_apply };
    if( batch.models == nullptr || batch.models->empty() ) {
        return;
    }
    if( g_stream.pending_apply_offset >= batch.models->size() ) {
        return;
    }

    auto const pending_dist {
        section_manhattan_sections(
            batch.row,
            batch.column,
            g_stream.center_row,
            g_stream.center_column ) };
    if( pending_dist <= kUrgentSectionDistanceKm ) {
        return;
    }
    if( pending_dist < 5 ) {
        return;
    }

    auto const total { batch.models->size() };
    if(
        total > 0 &&
        g_stream.pending_apply_offset > total / 2 ) {
        return;
    }

    auto const speed { camera_stream_speed_mps() };
    auto const near_apply_km {
        speed > 600.0 ? kFastFlightApplyMaxDistanceKm : kUrgentSectionDistanceKm };

    bool nearer_ready { false };
    {
        std::lock_guard<std::mutex> lock { g_stream.ready.mutex };
        for( auto const &ready : g_stream.ready.data ) {
            if( ready.failed || ready.models == nullptr || ready.models->empty() ) {
                continue;
            }
            auto const ready_dist {
                section_manhattan_sections(
                    ready.row,
                    ready.column,
                    g_stream.center_row,
                    g_stream.center_column ) };
            if(
                ready.row == g_stream.center_row &&
                ready.column == g_stream.center_column ) {
                nearer_ready = true;
                break;
            }
            if(
                ready_dist + 3 < pending_dist &&
                ready_dist <= near_apply_km ) {
                nearer_ready = true;
                break;
            }
        }
    }

    if( false == nearer_ready ) {
        if(
            pending_dist <= kPreemptPendingDistanceKm ||
            camera_stream_speed_mps() <= 80.0 ) {
            return;
        }

        auto const [cam_row, cam_col] {
            section_row_column( resolve_section_stream_position( Global.pCamera.Pos ) ) };
        if( section_has_pack_models( cam_row, cam_col ) ) {
            nearer_ready = true;
        }
    }

    if( false == nearer_ready ) {
        return;
    }

    batch.apply_offset = g_stream.pending_apply_offset;
    auto suspended { std::move( batch ) };
    g_stream.pending_apply.reset();
    g_stream.pending_apply_offset = 0;

    {
        std::lock_guard<std::mutex> lock { g_stream.ready.mutex };
        g_stream.ready.data.push_front( std::move( suspended ) );
    }
    pack_bench_inc( &Eu7PackBench::stream_preempt_pending );
    pack_bench_inc( &Eu7PackBench::stream_reenqueue );
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
    if( maybe_skip_stuck_apply_offset() ) {
        return true;
    }
    maybe_preempt_distant_pending();
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
                fail_section( batch.section_idx, batch.row, batch.column );
            }
            else if( batch.section_final ) {
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

        auto const remaining { total - offset };
        auto const pending_dist { pending_section_distance_km() };
        auto const cam_speed { camera_stream_speed_mps() };
        auto const is_urgent { pending_apply_is_urgent() };
        auto const is_heavy {
            false == batch.unique_meshes.empty() ||
            false == batch.unique_textures.empty() ||
            total >= kHeavySectionModelThreshold ||
            remaining >= kHeavySectionModelThreshold };
        auto const far_slow_drain {
            cam_speed > 300.0 &&
            pending_dist > kFarApplySlowDistanceKm &&
            false == is_urgent };

        std::size_t chunk_count { remaining };
        std::size_t cold_mesh_limit { remaining };
        double effective_cold_budget_ms { 0.0 };

        if( is_heavy ) {
            chunk_count = std::min( remaining, adaptive_slice_instances( total, is_urgent ) );
            cold_mesh_limit = adaptive_cold_meshes( is_urgent );
            effective_cold_budget_ms =
                is_urgent ? kUrgentColdBudgetMs : gameplay_cold_budget_ms();
            if( far_slow_drain ) {
                chunk_count = std::min( chunk_count, std::size_t { 32 } );
                cold_mesh_limit = std::min( cold_mesh_limit, std::size_t { 1 } );
                effective_cold_budget_ms = std::min( effective_cold_budget_ms, 8.0 );
            }
            if( max_instances > 0 ) {
                chunk_count = std::min( chunk_count, max_instances );
            }
            if( max_cold_meshes > 0 ) {
                cold_mesh_limit = std::min( cold_mesh_limit, max_cold_meshes );
            }
            if( cold_budget_ms > 0.0 ) {
                effective_cold_budget_ms = cold_budget_ms;
            }
        }

        auto const planned_inst { chunk_count };
        pack_bench_inc( &Eu7PackBench::stream_inst_planned, planned_inst );
        if( is_urgent ) {
            pack_bench_inc( &Eu7PackBench::stream_chunks_urgent );
        }
        else if( is_heavy ) {
            pack_bench_inc( &Eu7PackBench::stream_chunks_heavy );
        }
        else {
            pack_bench_inc( &Eu7PackBench::stream_chunks_light );
        }
        pack_bench_note_pending_total( total );

        auto const chunk_started { std::chrono::steady_clock::now() };
        double cold_ms { 0.0 };
        double warm_ms { 0.0 };
        double apply_ms { 0.0 };
        std::size_t cold_loads { 0 };
        std::size_t tex_fetches { 0 };

        {
            auto const phase_started { std::chrono::steady_clock::now() };
            if( false == batch.unique_meshes.empty() ) {
                cold_loads = preload_umes_cold_meshes(
                    batch.unique_meshes,
                    batch.umes_cold_offset,
                    cold_mesh_limit,
                    effective_cold_budget_ms,
                    batch.umes_cold_offset );
            }
            else {
                cold_loads = preload_slice_cold_meshes(
                    models_ptr + offset,
                    chunk_count,
                    cold_mesh_limit,
                    effective_cold_budget_ms,
                    chunk_count );
            }
            cold_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - phase_started ).count();
        }
        pack_bench_inc( &Eu7PackBench::stream_inst_after_cold, chunk_count );
        if( batch.unique_meshes.empty() && chunk_count < planned_inst ) {
            pack_bench_inc( &Eu7PackBench::stream_cold_slice_truncated );
        }
        if( chunk_count == 0 ) {
            g_stream.pending_apply_offset = offset + 1;
            note_apply_progress();
            applied_work = true;
            continue;
        }

        {
            auto const phase_started { std::chrono::steady_clock::now() };
            tex_fetches = warm_pack_section_textures( batch, offset, chunk_count );
            warm_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - phase_started ).count();
        }
        pack_bench_inc( &Eu7PackBench::main_chunk_tex_fetches, tex_fetches );

        scene::scratch_data scratch;
        simulation::eu7_pack_apply_session const session {
            &g_stream.mesh_cache,
            &g_stream.nodedata_cache,
            batch.section_idx };
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
        retain_section_mesh_keys( batch.section_idx, models_ptr, offset, chunk_count );
        retain_section_nodedata_keys( batch.section_idx, models_ptr, offset, chunk_count );
        evict_unreferenced_stream_caches();

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
                ( planned_inst != chunk_count ?
                    ( "/" + std::to_string( planned_inst ) ) :
                    std::string{} ) +
                " urg=" + std::to_string( is_urgent ? 1 : 0 ) +
                " heavy=" + std::to_string( is_heavy ? 1 : 0 ) +
                " cold=" + std::to_string( cold_loads ) + "(" + std::to_string( static_cast<int>( cold_ms ) ) + "ms)" +
                " warm=" + std::to_string( tex_fetches ) + "(" + std::to_string( static_cast<int>( warm_ms ) ) + "ms)" +
                " apply=" + std::to_string( static_cast<int>( apply_ms ) ) + "ms" +
                " pending=" + std::to_string( offset + chunk_count ) + "/" + std::to_string( total ) +
                " sec=" + std::to_string( batch.row ) + "," + std::to_string( batch.column ) );
        }
        load_stats().pack_models += chunk_count;
        g_stream.pending_apply_offset = offset + chunk_count;
        note_apply_progress();
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
        g_stream.pending_apply.has_value() && g_stream.pending_apply->models != nullptr ?
            g_stream.pending_apply->models->size() :
            std::size_t { 0 } );
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
                enqueue_failed_section( job );
                continue;
            }

            std::unique_ptr<std::vector<Eu7Model>> models;
            std::vector<std::string> unique_meshes;
            std::vector<std::string> unique_textures;
            std::uint32_t chunk_count { 1 };
            {
                PackBenchTimer const read_timer { &Eu7PackBench::worker_read_pack_ms };
                auto const chunk {
                    read_pack_section_chunk_load(
                        module, job.row, job.column, job.next_chunk ) };
                chunk_count = chunk.chunk_count;
                unique_meshes = std::move( chunk.unique_meshes );
                unique_textures = std::move( chunk.unique_textures );
                if( false == chunk.models.empty() ) {
                    models = std::make_unique<std::vector<Eu7Model>>(
                        std::move( chunk.models ) );
                }
            }

            PackSectionReady result;
            result.row = job.row;
            result.column = job.column;
            result.section_idx = job.section_idx;
            result.models = std::move( models );
            result.unique_meshes = std::move( unique_meshes );
            result.unique_textures = std::move( unique_textures );
            result.failed = result.models == nullptr;
            result.section_final = job.next_chunk + 1 >= chunk_count;

            if( result.failed ) {
                ErrorLog(
                    "EU7 PACK: pusty odczyt sekcji " + std::to_string( job.row ) + "," +
                    std::to_string( job.column ) + " chunk=" +
                    std::to_string( job.next_chunk ) + " (PIDX model_count=" +
                    std::to_string( entry->model_count ) + ")" );
                enqueue_failed_section( job );
                continue;
            }

            pack_bench_inc(
                &Eu7PackBench::worker_models_decoded, result.models->size() );

            preload_pack_models( *result.models, result.unique_meshes );

            {
                std::lock_guard<std::mutex> lock { g_stream.ready.mutex };
                g_stream.ready.data.push_back( std::move( result ) );
            }

            pack_bench_inc( &Eu7PackBench::worker_chunks_decoded );
            if( result.section_final ) {
                pack_bench_inc( &Eu7PackBench::worker_sections_done );
            }

            if( job.next_chunk == 0 ) {
                std::size_t pending_jobs { 0 };
                {
                    std::lock_guard<std::mutex> lock { g_stream.jobs.mutex };
                    pending_jobs = g_stream.jobs.data.size();
                }
                if( pending_jobs < kUmesPrefetchJobsBacklogThreshold ) {
                    prefetch_ready_queue_umes_worker(
                        kReadyUmesPrefetchBudgetMs, kReadyUmesPrefetchMaxMeshes );
                }
            }

            if( job.next_chunk + 1 < chunk_count ) {
                PackSectionJob continue_job { job };
                continue_job.next_chunk = job.next_chunk + 1;
                {
                    std::lock_guard<std::mutex> lock { g_stream.jobs.mutex };
                    g_stream.jobs.data.push_front( continue_job );
                }
                g_stream.work_cv.notify_one();
            }
        }
        catch( std::exception const &ex ) {
            ErrorLog(
                std::string{ "EU7 PACK: sekcja " + std::to_string( job.row ) + "," +
                    std::to_string( job.column ) + ": " } + ex.what() );
            enqueue_failed_section( job );
        }
    }
}

void
stop_workers() {
    g_stream.worker_exit = true;
    g_stream.work_cv.notify_all();
    g_stream.workers.clear();
    g_stream.worker_exit = false;
    reset_pack_section_read_cache();

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
    g_stream.loaded_sections.insert( batch.section_idx );
    g_stream.in_flight_sections.erase( batch.section_idx );
    ++load_stats().pack_sections_loaded;
    pack_bench_inc( &Eu7PackBench::sections_finalized );
}

void
fail_section(
    std::size_t const section_idx,
    int const row,
    int const column ) {
    if( g_stream.loaded_sections.contains( section_idx ) ) {
        return;
    }

    if( row >= 0 && column >= 0 ) {
        WriteLog(
            "EU7 PACK: failed section " + std::to_string( row ) + "," +
            std::to_string( column ) );
    }
    else {
        WriteLog( "EU7 PACK: failed section idx=" + std::to_string( section_idx ) );
    }

    g_stream.loaded_sections.insert( section_idx );
    g_stream.in_flight_sections.erase( section_idx );
}

void
enqueue_failed_section( PackSectionJob const &job ) {
    PackSectionReady result;
    result.row = job.row;
    result.column = job.column;
    result.section_idx = job.section_idx;
    result.failed = true;

    {
        std::lock_guard<std::mutex> lock { g_stream.ready.mutex };
        g_stream.ready.data.push_back( std::move( result ) );
    }

    pack_bench_inc( &Eu7PackBench::worker_failures );
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
    if( should_throttle_far_stream_jobs( row, column ) ) {
        pack_bench_inc( &Eu7PackBench::stream_jobs_blocked_far );
        return false;
    }

    auto const section_idx { section_index( row, column ) };
    if( g_stream.loaded_sections.contains( section_idx ) ) {
        return false;
    }

    // Pusta komorka siatki (brak wpisu PACK) — nie kolejkuj, oznacz jako zaladowana.
    if( false == section_has_pack_models( row, column ) ) {
        g_stream.loaded_sections.insert( section_idx );
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

    if( row == g_stream.center_row && column == g_stream.center_column && priority == 0 ) {
        std::lock_guard<std::mutex> lock { g_stream.jobs.mutex };
        g_stream.jobs.data.push_front( std::move( job ) );
    }
    else {
        push_prioritized_job( std::move( job ) );
    }
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
    {
        auto const cam_speed { camera_stream_speed_mps() };
        if( cam_speed > 200.0 ) {
            lookahead_steps = max_steps;
        }
        else if( cam_speed > 80.0 ) {
            lookahead_steps = std::max( lookahead_steps, std::min( max_steps, 10 ) );
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
        for( auto it { g_stream.ready.data.begin() }; it != g_stream.ready.data.end(); ) {
            if( it->failed || it->models == nullptr || it->models->empty() ) {
                if( it->failed ) {
                    fail_section( it->section_idx, it->row, it->column );
                }
                else if( it->section_final ) {
                    finalize_section( *it );
                }
                it = g_stream.ready.data.erase( it );
                continue;
            }
            ++it;
        }

        if( false == g_stream.ready.data.empty() ) {
            auto const pick_best {
                [&]( int const max_distance ) -> std::deque<PackSectionReady>::iterator {
                    auto best_it { g_stream.ready.data.end() };
                    auto best_dist { std::numeric_limits<int>::max() };
                    for( auto it { g_stream.ready.data.begin() }; it != g_stream.ready.data.end(); ++it ) {
                        if( it->failed || it->models == nullptr || it->models->empty() ) {
                            continue;
                        }
                        auto const dist {
                            section_manhattan_sections(
                                it->row,
                                it->column,
                                g_stream.center_row,
                                g_stream.center_column ) };
                        if( max_distance >= 0 && dist > max_distance ) {
                            continue;
                        }
                        if( dist < best_dist || ( dist == best_dist && best_it == g_stream.ready.data.end() ) ) {
                            best_dist = dist;
                            best_it = it;
                        }
                        else if(
                            dist == best_dist && best_it != g_stream.ready.data.end() &&
                            it->section_idx < best_it->section_idx ) {
                            best_it = it;
                        }
                    }
                    return best_it;
                } };

            auto const pick_camera_section {
                [&]() -> std::deque<PackSectionReady>::iterator {
                    for( auto it { g_stream.ready.data.begin() }; it != g_stream.ready.data.end(); ++it ) {
                        if( it->failed || it->models == nullptr || it->models->empty() ) {
                            continue;
                        }
                        if(
                            it->row == g_stream.center_row &&
                            it->column == g_stream.center_column ) {
                            return it;
                        }
                    }
                    return g_stream.ready.data.end();
                } };

            auto const cam_speed { camera_stream_speed_mps() };
            auto const max_apply_dist {
                cam_speed > 600.0 ?
                    kFastFlightApplyMaxDistanceKm :
                    ( cam_speed > 80.0 ? 3 : kSectionStreamGameplayRadiusKm ) };

            auto best_it { pick_camera_section() };
            if( best_it != g_stream.ready.data.end() ) {
                pack_bench_inc( &Eu7PackBench::stream_dequeue_camera );
            }
            else {
                best_it = pick_best( max_apply_dist );
            }
            if( best_it == g_stream.ready.data.end() ) {
                best_it = pick_best( kSectionStreamGameplayRadiusKm );
            }
            if( best_it == g_stream.ready.data.end() && false == g_stream.ready.data.empty() ) {
                best_it = pick_best( -1 );
            }
            if( best_it == g_stream.ready.data.end() ) {
                return false;
            }
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
            fail_section( batch.section_idx, batch.row, batch.column );
        }
        else if( batch.section_final ) {
            finalize_section( batch );
        }
        return false;
    }

    g_stream.pending_apply = std::move( batch );
    g_stream.pending_apply_offset = g_stream.pending_apply->apply_offset;
    g_stream.pending_apply->apply_offset = 0;

    auto const dequeue_dist {
        section_manhattan_sections(
            g_stream.pending_apply->row,
            g_stream.pending_apply->column,
            g_stream.center_row,
            g_stream.center_column ) };
    if( dequeue_dist <= kFastFlightApplyMaxDistanceKm ) {
        pack_bench_inc( &Eu7PackBench::stream_dequeue_near );
    }
    else {
        pack_bench_inc( &Eu7PackBench::stream_dequeue_far );
    }

    return true;
}

void
prefetch_ready_queue_textures( double const budget_ms ) {
    if( budget_ms <= 0.0 || GfxRenderer == nullptr ) {
        return;
    }

    PackBenchTimer const prefetch_timer { &Eu7PackBench::stream_prefetch_ready_tex_ms };
    auto const started { std::chrono::steady_clock::now() };
    std::vector<std::deque<PackSectionReady>::iterator> candidates;
    candidates.reserve( ready_queue_size() );

    {
        std::lock_guard<std::mutex> lock { g_stream.ready.mutex };
        for( auto it { g_stream.ready.data.begin() }; it != g_stream.ready.data.end(); ++it ) {
            if( it->failed ) {
                continue;
            }
            if( false == it->unique_textures.empty() ) {
                if( it->texture_warm_offset >= it->unique_textures.size() ) {
                    continue;
                }
                candidates.push_back( it );
                continue;
            }
            if( it->models == nullptr || it->models->empty() ) {
                continue;
            }
            if( it->texture_warm_offset >= it->models->size() ) {
                continue;
            }
            candidates.push_back( it );
        }
    }

    if( candidates.empty() ) {
        return;
    }

    std::sort(
        candidates.begin(),
        candidates.end(),
        []( std::deque<PackSectionReady>::iterator const &lhs,
            std::deque<PackSectionReady>::iterator const &rhs ) {
            auto const lhs_dist {
                section_manhattan_sections(
                    lhs->row,
                    lhs->column,
                    g_stream.center_row,
                    g_stream.center_column ) };
            auto const rhs_dist {
                section_manhattan_sections(
                    rhs->row,
                    rhs->column,
                    g_stream.center_row,
                    g_stream.center_column ) };
            return lhs_dist < rhs_dist;
        } );

    for( auto const it : candidates ) {
        auto &batch { *it };
        if( false == batch.unique_textures.empty() ) {
            while( batch.texture_warm_offset < batch.unique_textures.size() ) {
                auto const remaining {
                    batch.unique_textures.size() - batch.texture_warm_offset };
            auto const slice { std::min( remaining, kReadyTexturePrefetchSlice ) };
            auto const tex_slice { std::min( slice, kPackTextureWarmSlice ) };
            std::size_t processed { 0 };
            (void)warm_pack_texture_paths_main(
                batch.unique_textures.data() + batch.texture_warm_offset,
                tex_slice,
                kPackTextureWarmBudgetMs,
                &processed );
            batch.texture_warm_offset += processed;

                auto const elapsed_ms {
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - started ).count() };
                if( elapsed_ms >= budget_ms ) {
                    return;
                }
            }
            continue;
        }

        while( batch.texture_warm_offset < batch.models->size() ) {
            auto const remaining { batch.models->size() - batch.texture_warm_offset };
            auto const slice { std::min( remaining, kReadyTexturePrefetchSlice ) };
            (void)warm_pack_textures_main(
                batch.models->data() + batch.texture_warm_offset,
                slice );
            batch.texture_warm_offset += slice;

            auto const elapsed_ms {
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - started ).count() };
            if( elapsed_ms >= budget_ms ) {
                return;
            }
        }
    }
}

void
prefetch_ready_queue_umes_worker( double const budget_ms, std::size_t const max_meshes ) {
    if( budget_ms <= 0.0 || max_meshes == 0 ) {
        return;
    }

    PackBenchTimer const prefetch_timer { &Eu7PackBench::stream_prefetch_ready_umes_ms };
    auto const started { std::chrono::steady_clock::now() };
    std::size_t loads { 0 };

    std::lock_guard<std::mutex> lock { g_stream.ready.mutex };

    std::vector<std::deque<PackSectionReady>::iterator> candidates;
    candidates.reserve( g_stream.ready.data.size() );
    for( auto it { g_stream.ready.data.begin() }; it != g_stream.ready.data.end(); ++it ) {
        if( it->failed || it->unique_meshes.empty() ) {
            continue;
        }
        if( it->umes_prefetch_offset >= it->unique_meshes.size() ) {
            continue;
        }
        if( g_stream.center_row >= 0 && g_stream.center_column >= 0 ) {
            auto const dist {
                section_manhattan_sections(
                    it->row,
                    it->column,
                    g_stream.center_row,
                    g_stream.center_column ) };
            if( dist > kUmesPrefetchMaxDistanceKm ) {
                continue;
            }
        }
        candidates.push_back( it );
    }

    if( candidates.empty() ) {
        return;
    }

    std::sort(
        candidates.begin(),
        candidates.end(),
        []( std::deque<PackSectionReady>::iterator const &lhs,
            std::deque<PackSectionReady>::iterator const &rhs ) {
            auto const lhs_dist {
                section_manhattan_sections(
                    lhs->row,
                    lhs->column,
                    g_stream.center_row,
                    g_stream.center_column ) };
            auto const rhs_dist {
                section_manhattan_sections(
                    rhs->row,
                    rhs->column,
                    g_stream.center_row,
                    g_stream.center_column ) };
            return lhs_dist < rhs_dist;
        } );

    for( auto const it : candidates ) {
        auto &batch { *it };
        while( batch.umes_prefetch_offset < batch.unique_meshes.size() ) {
            auto model_file { batch.unique_meshes[ batch.umes_prefetch_offset ] };

            if( model_file.empty() || model_file == "notload" ) {
                ++batch.umes_prefetch_offset;
                continue;
            }
            replace_slashes( model_file );
            if( g_stream.mesh_cache.contains( model_file ) ) {
                pack_bench_inc( &Eu7PackBench::stream_mesh_session_hit );
                ++batch.umes_prefetch_offset;
                continue;
            }
            if( loads >= max_meshes ) {
                return;
            }
            if( loads > 0 ) {
                auto const elapsed_ms {
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - started ).count() };
                if( elapsed_ms >= budget_ms ) {
                    return;
                }
            }

            (void)ensure_stream_pack_mesh( model_file );
            ++loads;
            pack_bench_inc( &Eu7PackBench::stream_prefetch_ready_umes_loads );
            ++batch.umes_prefetch_offset;
        }
    }
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

    auto const stream_position { resolve_section_stream_position( world_position ) };
    auto const cam_speed { camera_stream_speed_mps() };
    auto const outer_ring {
        section_stream_ring_progress( stream_position, g_stream.radius ) };

    g_stream_catchup = (
        cam_speed > 80.0 ||
        outer_ring < 0.98f );

    if( g_stream_catchup ) {
        g_stream_max_in_flight = kCatchupMaxInFlightSections;
        g_stream_max_ready = kCatchupMaxReadySections;
    }
    else {
        g_stream_max_in_flight = kMaxInFlightSections;
        g_stream_max_ready = kMaxReadySections;
    }
}

} // namespace

void
remember_stream_anchor( glm::dvec3 const &position );

[[nodiscard]] bool
camera_far_from_stream_anchor( glm::dvec3 const &camera_position );

[[nodiscard]] glm::dvec3
resolve_section_stream_position( glm::dvec3 const &hint );

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

    remember_stream_anchor( initial );

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

    auto const stream_position { resolve_section_stream_position( world_position ) };

    auto const [center_row, center_column] { section_row_column( stream_position ) };
    auto const center_moved {
        center_row != g_stream.center_row || center_column != g_stream.center_column };
    auto const moved_since_enqueue {
        g_stream.has_last_enqueue_position ?
            glm::length( stream_position - g_stream.last_enqueue_position ) :
            std::numeric_limits<double>::infinity() };

    if( moved_since_enqueue > kTeleportReenqueueDistanceM ) {
        g_stream.has_last_enqueue_position = false;
    }

    g_stream.center_row = center_row;
    g_stream.center_column = center_column;

    if( center_moved ) {
        reprioritize_job_queue();
        maybe_unload_distant_sections( center_row, center_column );
    }

    sync_stream_limits( stream_position );

    auto const inner_ring_ready {
        section_stream_ready_around( stream_position, kSectionStreamGameplayRadiusKm ) };
    auto const ring_radius {
        inner_ring_ready ? g_stream.radius : kSectionStreamGameplayRadiusKm };

    auto const cam_speed { camera_stream_speed_mps() };
    auto const reenqueue_distance {
        ( g_stream_catchup || cam_speed > 80.0 ) ?
            std::clamp( cam_speed * 0.25, kCatchupReenqueueDistanceM, kReenqueueDistanceM ) :
            kReenqueueDistanceM };

    auto const current_section_unloaded {
        section_has_pack_models( center_row, center_column )
        && false == g_stream.loaded_sections.contains( section_index( center_row, center_column ) ) };

    auto max_lookahead { 0 };
    if( inner_ring_ready ) {
        max_lookahead = ( g_stream_catchup || cam_speed > 200.0 ) ?
            kMovementLookahead :
            ( cam_speed > 80.0 ? 10 : 7 );
        if( current_section_unloaded && cam_speed > 200.0 ) {
            max_lookahead = std::min( max_lookahead, 4 );
        }
    }

    auto const travel_forward { guess_travel_forward() };

    auto const inner_ring_incomplete { current_section_unloaded || false == inner_ring_ready };
    auto const should_reenqueue {
        center_moved
        || false == g_stream.has_last_enqueue_position
        || moved_since_enqueue >= reenqueue_distance };

    if( current_section_unloaded ) {
        enqueue_section_if_needed( center_row, center_column, 0 );
    }

    if( should_reenqueue || inner_ring_incomplete ) {
        auto const enqueue_radius {
            ( current_section_unloaded && cam_speed > 400.0 ) ?
                std::min( ring_radius, 3 ) :
                ring_radius };
        enqueue_sections_around( center_row, center_column, enqueue_radius, stream_position );
        g_stream.last_enqueue_position = stream_position;
        g_stream.has_last_enqueue_position = true;
        if( should_reenqueue || inner_ring_incomplete ) {
            pack_bench_inc( &Eu7PackBench::stream_reenqueue );
        }
    }

    if(
        inner_ring_ready &&
        max_lookahead > 0 &&
        cam_speed > 50.0 &&
        false == ( current_section_unloaded && cam_speed > 200.0 ) ) {
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
    (void)world_position;
    return false;
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
    auto const stream_position {
        resolve_section_stream_position(
            g_loading_screen_dismissed ?
                Global.pCamera.Pos :
                stream_loading_position() ) };
    if( gameplay_stream_mode() && section_stream_drain_idle( stream_position ) ) {
        return;
    }

    auto const inner_ring_ready {
        section_stream_ready_around(
            stream_position,
            kSectionStreamGameplayRadiusKm ) };

    if( gameplay_stream_mode() && inner_ring_ready ) {
        maybe_unload_distant_sections(
            g_stream.center_row,
            g_stream.center_column );
        prefetch_ready_queue_textures( ready_texture_prefetch_budget_ms() );

        if( g_stream_catchup ) {
            auto const urgent {
                pending_apply_is_urgent() ||
                ( pending_section_distance_km() <= kFastFlightApplyMaxDistanceKm &&
                  camera_stream_speed_mps() > 80.0 ) };
            if( urgent ) {
                pack_bench_inc( &Eu7PackBench::stream_drain_catchup_urgent );
            }
            else {
                pack_bench_inc( &Eu7PackBench::stream_drain_catchup );
            }
            drain_apply_budget(
                urgent ? kUrgentApplyBudgetMs : gameplay_apply_budget_ms(),
                adaptive_slice_instances( pending_section_total(), urgent ),
                adaptive_cold_meshes( urgent ),
                urgent ? kUrgentColdBudgetMs : gameplay_cold_budget_ms(),
                urgent ? kUrgentMaxChunksPerDrain : 1 );
        }
        else {
            pack_bench_inc( &Eu7PackBench::stream_drain_gameplay );
            drain_until_budget( kDrainBudgetMs );
        }
        maybe_log_stream_status( stream_position );
    }
    else if( gameplay_stream_mode() ) {
        pack_bench_inc( &Eu7PackBench::stream_drain_loader );
        drain_until_budget( kLoaderDrainBudgetMs );
        maybe_log_stream_status( stream_position );
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
        pack_bench_inc( &Eu7PackBench::stream_drain_gameplay );
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

    remember_stream_anchor( position );

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
    position = resolve_section_stream_position( position );

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

    if( g_stream.has_anchor_position ) {
        auto const &saved_camera { Global.FreeCameraInit[ 0 ] };
        auto const camera_unset {
            saved_camera.x == 0.0 && saved_camera.y == 0.0 && saved_camera.z == 0.0 };
        auto const camera_off_scenery {
            false == camera_unset && camera_far_from_stream_anchor( saved_camera ) };
        if( camera_unset || camera_off_scenery ) {
            Global.FreeCameraInit[ 0 ] = g_stream.anchor_position;
            Global.pCamera.Pos = g_stream.anchor_position;
            WriteLog(
                "EU7 PACK: ustawiam kamere startowa na " +
                std::to_string( g_stream.anchor_position.x ) + "," +
                std::to_string( g_stream.anchor_position.y ) + "," +
                std::to_string( g_stream.anchor_position.z ) );
        }
    }
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
    if( g_loading_screen_dismissed ) {
        return Global.pCamera.Pos;
    }

    auto const &camera { Global.pCamera.Pos };
    if( camera.x != 0.0 || camera.y != 0.0 || camera.z != 0.0 ) {
        return camera;
    }

    auto position { resolve_stream_position() };
    if( position.x != 0.0 || position.y != 0.0 || position.z != 0.0 ) {
        return position;
    }

    if( g_stream.has_anchor_position ) {
        return g_stream.anchor_position;
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
remember_stream_anchor( glm::dvec3 const &position ) {
    if( position.x == 0.0 && position.y == 0.0 && position.z == 0.0 ) {
        return;
    }
    g_stream.anchor_position = position;
    g_stream.has_anchor_position = true;
}

[[nodiscard]] glm::dvec3
resolve_section_stream_position( glm::dvec3 const &hint ) {
    if(
        hint.x == 0.0 && hint.y == 0.0 && hint.z == 0.0
        && g_stream.has_anchor_position ) {
        return g_stream.anchor_position;
    }
    if( camera_far_from_stream_anchor( hint ) && g_stream.has_anchor_position ) {
        return g_stream.anchor_position;
    }
    return hint;
}

[[nodiscard]] bool
camera_far_from_stream_anchor( glm::dvec3 const &camera_position ) {
    if( false == g_stream.has_anchor_position ) {
        return false;
    }
    auto const anchor_dist {
        glm::length(
            glm::dvec2 {
                camera_position.x - g_stream.anchor_position.x,
                camera_position.z - g_stream.anchor_position.z } ) };
    auto const anchor_magnitude {
        glm::length(
            glm::dvec2 { g_stream.anchor_position.x, g_stream.anchor_position.z } ) };
    auto const camera_magnitude {
        glm::length( glm::dvec2 { camera_position.x, camera_position.z } ) };
    return anchor_dist > 30000.0
        && camera_magnitude < 30000.0
        && anchor_magnitude > 50000.0;
}

void
reset_section_stream() {
    flush_pack_stream_bench();
    stop_workers();
    if( g_stream.serializer != nullptr ) {
        g_stream.serializer->reset_eu7_pack_section_instances();
    }
    reset_stream_fields();
    g_ring_ready_since.reset();
    g_loading_block_started.reset();
    g_loading_screen_dismissed = false;
    g_stream_catchup = false;
    g_stream_max_in_flight = kMaxInFlightSections;
    g_stream_max_ready = kMaxReadySections;
    reset_pack_bench();
}

bool
section_stream_frame_budget_pressure() {
    return g_stream_catchup || pending_apply_is_urgent();
}

} // namespace scene::eu7
