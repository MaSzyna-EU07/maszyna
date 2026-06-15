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
#include "scene/eu7/eu7_pack_mesh_loader.h"
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
constexpr double kLoaderDrainBudgetMs { 256.0 };
constexpr double kFrameDrainBudgetMs { 10.0 };
constexpr std::size_t kLoaderSectionsPerDrain { 8 };

constexpr double kGameplayApplyBudgetMs { 6.0 };
constexpr double kCatchupApplyBudgetMs { 8.0 };
constexpr std::size_t kGameplaySliceInstances { 48 };
constexpr std::size_t kCatchupSliceInstances { 48 };
constexpr std::size_t kFrameSliceInstances { 24 };
constexpr std::size_t kLoaderSliceInstances { 768 };
constexpr std::size_t kGameplaySliceColdMeshes { 4 };
constexpr std::size_t kCatchupSliceColdMeshes { 6 };
constexpr std::size_t kFrameSliceColdMeshes { 2 };
constexpr std::size_t kLoaderSliceColdMeshes { 32 };
constexpr double kGameplayColdBudgetMs { 6.0 };
constexpr double kCatchupColdBudgetMs { 8.0 };
constexpr double kFrameColdBudgetMs { 4.0 };

constexpr int kInitialBootstrapEnqueueRadius { 2 };
constexpr int kInitialBootstrapRadius { 3 };
constexpr float kEarlyDismissRingProgress { 0.65f };
constexpr int kStreamRadius { kSectionStreamTargetRadiusKm };
constexpr int kMaintenanceLookaheadMax { 6 };
constexpr double kRadiusFillApplyBudgetMs { 8.0 };
constexpr std::size_t kRadiusFillSliceInstances { 16 };
constexpr std::size_t kRadiusFillMaxChunksPerDrain { 2 };
constexpr std::size_t kRadiusFillMaxInFlight { 24 };
constexpr std::size_t kRadiusFillMaxReady { 14 };
constexpr float kRadiusFillOuterTarget { 0.98f };
constexpr int kSectionUnloadMarginKm { 3 };
constexpr int kSectionUnloadCameraGuardKm { 5 };
constexpr int kSectionUnloadMaxSpeedExtraKm { 10 };
constexpr double kSectionUnloadKeepRadiusDecayKmPerSec { 2.0 };
constexpr int kSectionUnloadTeleportJumpKm { 4 };
constexpr std::size_t kMeshCacheLruCap { 2000 };
constexpr std::size_t kNodedataCacheLruCap { 2000 };
constexpr int kMovementLookahead { 10 };
constexpr std::size_t kMaxPackStreamWorkers { 16 };
constexpr std::size_t kMaxInFlightSections { 20 };
constexpr std::size_t kMaxReadySections { 10 };
constexpr double kStreamStatusLogIntervalSec { 5.0 };
constexpr double kReenqueueDistanceM { 500.0 };
constexpr double kCatchupReenqueueDistanceM { 40.0 };
constexpr std::size_t kHeavySectionModelThreshold { 32 };
constexpr std::size_t kPackTextureWarmSlice { 8 };
constexpr double kPackTextureWarmBudgetMs { 4.0 };
constexpr double kUrgentApplyBudgetMs { 10.0 };
constexpr double kRingDeficitDrainBudgetMs { 8.0 };
constexpr double kSmoothApplyTargetMs { 7.0 };
constexpr double kSmoothApplyDeficitTargetMs { 8.0 };
constexpr std::size_t kUrgentSliceInstances { 48 };
constexpr std::size_t kUrgentSliceColdMeshes { 4 };
constexpr double kUrgentColdBudgetMs { 6.0 };
constexpr std::size_t kUrgentMaxChunksPerDrain { 2 };
constexpr std::size_t kFrameMaxChunksPerDrain { 1 };
constexpr std::size_t kRingDeficitMaxChunksPerDrain { 2 };
constexpr double kReadyTexturePrefetchBudgetMs { 14.0 };
constexpr std::size_t kReadyTexturePrefetchSlice { 256 };
constexpr int kUmesPrefetchMaxDistanceKm { 6 };
constexpr double kReadyUmesPrefetchBudgetMs { 16.0 };
constexpr std::size_t kReadyUmesPrefetchMaxMeshes { 64 };
constexpr std::size_t kCatchupMaxInFlightSections { 32 };
constexpr std::size_t kCatchupMaxReadySections { 16 };
constexpr int kUrgentSectionDistanceKm { 1 };
constexpr int kFastFlightApplyMaxDistanceKm { 3 };
constexpr int kFarApplySlowDistanceKm { 4 };
constexpr int kPreemptPendingDistanceKm { 2 };
constexpr float kRingGatedOuterThreshold { 0.35f };
constexpr float kRingGatedInnerTarget { 0.70f };
constexpr float kRingBoostOuterThreshold { 0.55f };
constexpr float kRingStrongBoostOuterThreshold { 0.75f };
constexpr std::size_t kPreemptReadyQueueThreshold { 8 };
constexpr int kWorkerContinueMaxDistanceKm { 8 };
constexpr std::size_t kFastFlightMaxFarInFlight { 3 };
constexpr double kFastFlightRingGateSpeedMps { 600.0 };
constexpr double kRingBoostInFlightSpeedMps { 600.0 };
constexpr std::size_t kRingBoostMaxInFlight { 12 };
constexpr std::size_t kRingStrongBoostMaxInFlight { 16 };
constexpr std::size_t kRingBoostMaxReady { 8 };
constexpr std::size_t kRingStrongBoostMaxReady { 10 };
constexpr double kPreemptDisableSpeedMps { 400.0 };
constexpr double kTeleportReenqueueDistanceM { 2000.0 };
constexpr float kStationaryCatchupRingThreshold { 0.70f };
constexpr double kStationaryCatchupSpeedMps { 5.0 };
constexpr std::size_t kBootstrapDrainMs { 32 };
constexpr std::size_t kBootstrapTimeoutMs { 120000 };
constexpr std::size_t kBootstrapApplyModelThreshold { 400 };
constexpr std::chrono::milliseconds kPresentableHoldMs { 0 };
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
    std::vector<std::string> unique_textures;
    std::shared_ptr<Eu7PackSectionPathTables const> path_tables;
    bool failed { false };
    bool section_final { true };
    std::size_t apply_offset { 0 };
    std::size_t texture_warm_offset { 0 };
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
    bool radius_fill_active { false };

    std::optional<PackSectionReady> pending_apply;
    std::size_t pending_apply_offset { 0 };
    glm::dvec3 last_enqueue_position {};
    bool has_last_enqueue_position { false };
    glm::dvec3 anchor_position {};
    bool has_anchor_position { false };
    float last_inner_ring { 1.f };
    float last_outer_ring { 1.f };
    int unload_keep_radius { 0 };
    double unload_keep_radius_floor { 0.0 };
    double unload_keep_radius_last_update { 0.0 };

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

[[nodiscard]] std::size_t
count_far_in_flight_sections( int const max_distance_km );

[[nodiscard]] bool
should_block_far_enqueue(
    int const row,
    int const column,
    bool const lookahead_enqueue );

void
push_prioritized_job( PackSectionJob job );

void
reprioritize_job_queue();

[[nodiscard]] bool
enqueue_section_if_needed(
    int const row,
    int const column,
    int const priority,
    bool const lookahead_enqueue = false );

void
sync_stream_limits( glm::dvec3 const &world_position );

void
maybe_unload_distant_sections(
    int const prev_center_row,
    int const prev_center_column );

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

[[nodiscard]] bool
gameplay_stream_mode();

[[nodiscard]] std::size_t
warm_pack_section_textures(
    PackSectionReady &batch,
    std::size_t const model_offset,
    std::size_t const model_count,
    double const budget_ms = 0.0 ) {
    if( batch.models == nullptr || model_count == 0 ) {
        return 0;
    }
    auto const begin { model_offset };
    if( begin >= batch.models->size() ) {
        return 0;
    }
    auto const remaining { std::min( model_count, batch.models->size() - begin ) };
    auto const warmed {
        warm_pack_textures_main( batch.models->data() + begin, remaining, budget_ms ) };
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

[[nodiscard]] PackMeshLoadWait
stream_mesh_load_wait_policy() {
    return gameplay_stream_mode() ?
        PackMeshLoadWait::NonBlocking :
        PackMeshLoadWait::BlockUntilReady;
}

[[nodiscard]] bool
pack_mesh_ready_for_slice( std::string const &model_file ) {
    if( false == try_adopt_pack_mesh_for_slice( model_file, g_stream.mesh_cache ) ) {
        return false;
    }
    auto const found { g_stream.mesh_cache.find( model_file ) };
    return found != g_stream.mesh_cache.end() &&
        found->second != nullptr &&
        TModelsManager::IsModelCached( model_file );
}

[[nodiscard]] bool
ensure_stream_pack_mesh(
    std::string model_file,
    double const block_budget_ms = 0.0 ) {
    if( model_file.empty() || model_file == "notload" ) {
        return false;
    }
    replace_slashes( model_file );
    auto *const mesh {
        ensure_pack_mesh_in_session_cache(
            model_file,
            g_stream.mesh_cache,
            stream_mesh_load_wait_policy(),
            block_budget_ms ) };
    if( mesh != nullptr ) {
        touch_string_lru( model_file, g_stream.mesh_lru, g_stream.mesh_lru_iters );
    }
    return mesh != nullptr;
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
update_unload_keep_radius_hysteresis() {
    if( g_stream.center_row < 0 || g_stream.center_column < 0 ) {
        return;
    }

    auto const speed { camera_stream_speed_mps() };
    auto const speed_extra {
        std::min(
            static_cast<int>( speed / 150.0 ),
            kSectionUnloadMaxSpeedExtraKm ) };
    auto const desired_keep_radius {
        static_cast<double>( g_stream.radius + kSectionUnloadMarginKm + speed_extra ) };
    auto const now { Timer::GetTime() };

    if( g_stream.unload_keep_radius_floor <= 0.0 ) {
        g_stream.unload_keep_radius_floor = desired_keep_radius;
        g_stream.unload_keep_radius_last_update = now;
        g_stream.unload_keep_radius = static_cast<int>( std::ceil( desired_keep_radius ) );
        return;
    }

    auto const dt { std::max( 0.0, now - g_stream.unload_keep_radius_last_update ) };
    g_stream.unload_keep_radius_last_update = now;

    if( desired_keep_radius > g_stream.unload_keep_radius_floor ) {
        g_stream.unload_keep_radius_floor = desired_keep_radius;
    }
    else if( desired_keep_radius < g_stream.unload_keep_radius_floor && dt > 0.0 ) {
        g_stream.unload_keep_radius_floor = std::max(
            desired_keep_radius,
            g_stream.unload_keep_radius_floor - dt * kSectionUnloadKeepRadiusDecayKmPerSec );
    }
    g_stream.unload_keep_radius = static_cast<int>( std::ceil( g_stream.unload_keep_radius_floor ) );
}

void
maybe_unload_distant_sections(
    int const prev_center_row,
    int const prev_center_column ) {
    if( false == g_loading_screen_dismissed || g_stream.bootstrap_active ) {
        return;
    }
    if( g_stream.center_row < 0 || g_stream.center_column < 0 ) {
        return;
    }

    auto const [cam_row, cam_col] { section_row_column( Global.pCamera.Pos ) };
    auto const keep_radius { g_stream.unload_keep_radius };

    auto const center_jump {
        prev_center_row >= 0 && prev_center_column >= 0 ?
            section_manhattan_sections(
                g_stream.center_row,
                g_stream.center_column,
                prev_center_row,
                prev_center_column ) :
            0 };
    if( center_jump >= kSectionUnloadTeleportJumpKm ) {
        return;
    }

    std::vector<std::size_t> to_unload;
    to_unload.reserve( g_stream.loaded_sections.size() );

    for( auto const section_idx : g_stream.loaded_sections ) {
        auto const row {
            static_cast<int>( section_idx / static_cast<std::size_t>( kRegionSideSectionCount ) ) };
        auto const column {
            static_cast<int>( section_idx % static_cast<std::size_t>( kRegionSideSectionCount ) ) };

        if(
            ring_section_in_radius(
                row,
                column,
                cam_row,
                cam_col,
                kSectionUnloadCameraGuardKm ) ) {
            continue;
        }
        if(
            ring_section_in_radius(
                row,
                column,
                g_stream.center_row,
                g_stream.center_column,
                keep_radius ) ) {
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
    g_stream.radius_fill_active = false;
    g_stream.pending_apply.reset();
    g_stream.pending_apply_offset = 0;
    g_stream.has_last_enqueue_position = false;
    g_stream.anchor_position = {};
    g_stream.has_anchor_position = false;
    g_stream.last_inner_ring = 1.f;
    g_stream.last_outer_ring = 1.f;
    g_stream.unload_keep_radius = 0;
    g_stream.unload_keep_radius_floor = 0.0;
    g_stream.unload_keep_radius_last_update = 0.0;
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
    if(
        false == g_stream.bootstrap_active &&
        g_stream.in_flight_sections.size() >= g_stream_max_in_flight ) {
        return true;
    }
    return ready_queue_size() >= g_stream_max_ready;
}

[[nodiscard]] double
camera_stream_speed_mps() {
    auto const rotated {
        rotate_velocity_y( Global.pCamera.Velocity, Global.pCamera.Angle.y ) };
    return glm::length( rotated ) * 5.0;
}

[[nodiscard]] bool stream_ring_deficit();
[[nodiscard]] bool pending_apply_is_urgent();
[[nodiscard]] bool in_radius_fill_phase();

[[nodiscard]] bool
in_radius_fill_phase() {
    return g_stream.radius_fill_active;
}

[[nodiscard]] double
frame_drain_budget_ms() {
    if( in_radius_fill_phase() ) {
        return kRadiusFillApplyBudgetMs;
    }
    if( stream_ring_deficit() ) {
        return kRingDeficitDrainBudgetMs;
    }
    auto const last_chunk { pack_bench_stream().last_chunk_ms };
    if( last_chunk >= 32.0 ) {
        return 3.0;
    }
    if( last_chunk >= 16.0 ) {
        return 5.0;
    }
    if( last_chunk >= 12.0 ) {
        return 7.0;
    }
    return kFrameDrainBudgetMs;
}
[[nodiscard]] double gameplay_apply_budget_ms();

[[nodiscard]] bool
stream_ring_deficit() {
    if( in_radius_fill_phase() ) {
        return g_stream.last_outer_ring < kRadiusFillOuterTarget;
    }
    return g_stream.last_inner_ring < kRingGatedInnerTarget;
}

[[nodiscard]] int
umes_prefetch_max_distance_km() {
    auto const speed { camera_stream_speed_mps() };
    if( g_stream_catchup || speed > 80.0 ) {
        return std::min( g_stream.radius, kWorkerContinueMaxDistanceKm );
    }
    return kUmesPrefetchMaxDistanceKm;
}

[[nodiscard]] double
ready_texture_prefetch_budget_ms() {
    auto const speed { camera_stream_speed_mps() };
    auto const ring_deficit { stream_ring_deficit() };

    if( ring_deficit && speed > 200.0 ) {
        return kPackTextureWarmBudgetMs;
    }
    if(
        speed > 500.0 &&
        pending_apply_is_urgent() ) {
        return kPackTextureWarmBudgetMs * 0.5;
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
    if( in_radius_fill_phase() ) {
        return kRadiusFillApplyBudgetMs;
    }
    double budget { kGameplayApplyBudgetMs };
    if( g_stream_catchup ) {
        if( stream_ring_deficit() ) {
            budget = pending_apply_is_urgent() ? kUrgentApplyBudgetMs : 8.0;
        }
        else if(
            g_stream.last_outer_ring < kRingBoostOuterThreshold &&
            camera_stream_speed_mps() > kPreemptDisableSpeedMps ) {
            budget = 8.0;
        }
        else {
            budget = kCatchupApplyBudgetMs;
        }
    }
    if( g_stream.last_inner_ring < 0.5f ) {
        budget = std::max( budget, 12.0 );
    }
    return budget;
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
    if( in_radius_fill_phase() ) {
        return pending_section_near_camera( kSectionStreamGameplayRadiusKm );
    }
    if( stream_ring_deficit() ) {
        return true;
    }
    auto const speed { camera_stream_speed_mps() };
    if( speed <= 80.0 && false == g_stream_catchup ) {
        return false;
    }
    auto const max_dist {
        speed > 600.0 ? kFastFlightApplyMaxDistanceKm : kUrgentSectionDistanceKm };
    return pending_section_near_camera( max_dist );
}

[[nodiscard]] std::size_t
apply_time_target_slice( double const target_ms, std::size_t const ceiling ) {
    auto const &bench { pack_bench_stream() };
    auto effective_apply_ms { bench.last_apply_ms };
    if( bench.last_chunk_ms > effective_apply_ms && bench.last_chunk_instances > 0 ) {
        effective_apply_ms = bench.last_chunk_ms;
    }
    if( effective_apply_ms >= 1.0 && bench.last_chunk_instances > 0 ) {
        auto const scaled { static_cast<std::size_t>(
            static_cast<double>( bench.last_chunk_instances ) *
            ( target_ms / effective_apply_ms ) ) };
        return std::clamp( scaled, std::size_t { 8 }, ceiling );
    }
    return std::min( ceiling, std::size_t { 24 } );
}

[[nodiscard]] std::size_t
gameplay_max_chunks_per_drain() {
    if( in_radius_fill_phase() ) {
        return kRadiusFillMaxChunksPerDrain;
    }
    auto const &bench { pack_bench_stream() };
    if( stream_ring_deficit() ) {
        if( bench.last_chunk_ms >= 8.0 ) {
            return 1;
        }
        return kRingDeficitMaxChunksPerDrain;
    }
    if( pending_apply_is_urgent() ) {
        if( bench.last_chunk_ms >= 8.0 ) {
            return 1;
        }
        return kUrgentMaxChunksPerDrain;
    }
    if( bench.last_chunk_ms >= 12.0 ) {
        return 1;
    }
    return kFrameMaxChunksPerDrain;
}

[[nodiscard]] std::size_t
scene_apply_pressure_slice_cap() {
    auto const applied { load_stats().pack_models };
    if( applied >= 300000 ) {
        return 24;
    }
    if( applied >= 200000 ) {
        return 32;
    }
    if( applied >= 120000 ) {
        return 48;
    }
    if( applied >= 80000 ) {
        return 64;
    }
    if( applied >= 40000 ) {
        return 80;
    }
    return std::numeric_limits<std::size_t>::max();
}

[[nodiscard]] std::size_t
adaptive_slice_instances( std::size_t const section_total, bool const urgent = false ) {
    auto const ring_deficit { stream_ring_deficit() };

    if( urgent ) {
        auto limit { kUrgentSliceInstances };
        auto const speed { camera_stream_speed_mps() };
        if( speed > 1200.0 ) {
            limit = std::min( limit, std::size_t { 64 } );
        }
        else if( speed > 600.0 ) {
            limit = std::min( limit, std::size_t { 48 } );
        }
        if( section_total > 4000 ) {
            limit = std::min( limit, std::size_t { 48 } );
        }
        else if( section_total > 2000 ) {
            limit = std::min( limit, std::size_t { 48 } );
        }
        auto const last_ms { pack_bench_stream().last_chunk_ms };
        if( false == ring_deficit ) {
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
        }
        if( ring_deficit ) {
            auto const target_ms {
                g_stream.last_inner_ring < 0.5f ?
                    kSmoothApplyDeficitTargetMs :
                    kSmoothApplyTargetMs };
            auto const ceiling { std::size_t { 28 } };
            limit = std::min( limit, apply_time_target_slice( target_ms, ceiling ) );
            auto const last_apply { pack_bench_stream().last_apply_ms };
            if( last_apply >= 10.0 ) {
                limit = std::min(
                    limit,
                    apply_time_target_slice( kSmoothApplyTargetMs * 0.6, std::size_t { 20 } ) );
            }
            if( pack_bench_stream().last_chunk_ms >= 12.0 ) {
                limit = std::min( limit, std::size_t { 20 } );
            }
            return std::max( limit, std::size_t { 8 } );
        }
        limit = std::min(
            limit,
            apply_time_target_slice( kSmoothApplyTargetMs, std::size_t { 48 } ) );
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
        if( pack_bench_stream().last_chunk_ms >= 12.0 ) {
            limit = std::min( limit, std::size_t { 32 } );
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
    if( false == ring_deficit ) {
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
    }

    if( ring_deficit ) {
        auto const target_ms {
            g_stream.last_inner_ring < 0.5f ?
                kSmoothApplyDeficitTargetMs :
                kSmoothApplyTargetMs };
        limit = std::min( limit, apply_time_target_slice( target_ms, std::size_t { 28 } ) );
        if( pack_bench_stream().last_chunk_ms >= 12.0 ) {
            limit = std::min( limit, std::size_t { 20 } );
        }
        return std::max( limit, std::size_t { 8 } );
    }

    return std::max( limit, std::size_t { 16 } );
}

[[nodiscard]] std::size_t
adaptive_cold_meshes( bool const urgent = false ) {
    auto limit {
        urgent ? kUrgentSliceColdMeshes : gameplay_slice_cold_meshes() };
    if( stream_ring_deficit() ) {
        limit = std::max( limit, urgent ? std::size_t { 12 } : std::size_t { 8 } );
    }
    if( pack_bench_stream().main_chunk_cold_ms >= 12.0 ) {
        limit = std::min( limit, std::size_t { 2 } );
    }
    else if( pack_bench_stream().main_chunk_cold_ms >= 6.0 ) {
        limit = std::min( limit, std::max( limit / 2, std::size_t { 3 } ) );
    }
    return std::max( limit, std::size_t { 2 } );
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
    std::size_t &slice_count,
    Eu7PackSectionPathTables const *path_tables = nullptr ) {
    slice_count = 0;
    if( models == nullptr || count == 0 ) {
        return 0;
    }

    (void)drain_pack_mesh_loader_ready( g_stream.mesh_cache );

    auto mesh_ready { [&]( std::size_t const index ) -> bool {
        if( eu7_pack_model_needs_full_load( models[ index ] ) ) {
            return true;
        }
        return pack_mesh_ready_for_slice(
            std::string( pack_model_mesh_path( models[ index ], path_tables ) ) );
    } };

    std::size_t cold_loaded { 0 };
    auto const cold_started { std::chrono::steady_clock::now() };
    for( std::size_t i { 0 }; i < count; ++i ) {
        if( eu7_pack_model_needs_full_load( models[ i ] ) ) {
            auto model_file {
                std::string( pack_model_mesh_path( models[ i ], path_tables ) ) };
            if( false == model_file.empty() && model_file != "notload" ) {
                replace_slashes( model_file );
                if( false == pack_mesh_ready_for_slice( model_file ) ) {
                    request_pack_mesh_load( model_file );
                }
            }
            continue;
        }
        if( mesh_ready( i ) ) {
            continue;
        }
        if( cold_loaded >= max_cold_meshes ) {
            break;
        }
        auto remaining_budget_ms { cold_budget_ms };
        if( cold_budget_ms > 0.0 && cold_loaded > 0 ) {
            auto const elapsed_ms {
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - cold_started ).count() };
            if( elapsed_ms >= cold_budget_ms ) {
                break;
            }
            remaining_budget_ms = cold_budget_ms - elapsed_ms;
        }
        auto model_file { std::string( pack_model_mesh_path( models[ i ], path_tables ) ) };
        replace_slashes( model_file );
        if( ensure_stream_pack_mesh( model_file, remaining_budget_ms ) ) {
            ++cold_loaded;
            pack_bench_inc( &Eu7PackBench::main_cold_getmodel_calls );
            continue;
        }
        request_pack_mesh_load( model_file );
        if( gameplay_stream_mode() ) {
            break;
        }
    }

    for( std::size_t i { 0 }; i < count; ++i ) {
        if( false == mesh_ready( i ) ) {
            slice_count = i;
            break;
        }
        slice_count = i + 1;
    }

    if( slice_count == 0 && count > 0 && pending_apply_is_urgent() ) {
        auto model_file {
            std::string( pack_model_mesh_path( models[ 0 ], path_tables ) ) };
        if( model_file.empty() || model_file == "notload" ) {
            slice_count = 1;
        }
        else {
            replace_slashes( model_file );
            auto const wait_budget { std::min( cold_budget_ms, 8.0 ) };
            if( ensure_stream_pack_mesh( model_file, wait_budget ) ) {
                ++cold_loaded;
            }
            if( mesh_ready( 0 ) ) {
                slice_count = 1;
            }
        }
    }
    return cold_loaded;
}

template<typename Fn>
void
for_each_chunk_mesh_path(
    PackSectionReady const &batch,
    Fn const &fn ) {
    if( batch.models == nullptr ) {
        return;
    }

    std::unordered_set<std::string> seen;
    seen.reserve( batch.models->size() );
    for( auto const &model : *batch.models ) {
        if( eu7_pack_model_needs_full_load( model ) ) {
            continue;
        }
        auto const model_file {
            pack_model_mesh_path( model, batch.path_tables.get() ) };
        if( model_file.empty() || model_file == "notload" ) {
            continue;
        }
        auto normalized { std::string( model_file ) };
        replace_slashes( normalized );
        if( false == seen.insert( normalized ).second ) {
            continue;
        }
        fn( normalized );
    }
}

[[nodiscard]] bool
file_chunk_meshes_ready(
    PackSectionReady const &batch,
    std::size_t const model_offset = 0,
    std::size_t const model_count = std::numeric_limits<std::size_t>::max() );

void
request_missing_file_chunk_meshes(
    PackSectionReady const &batch,
    std::size_t const model_offset = 0,
    std::size_t const model_count = std::numeric_limits<std::size_t>::max() );

[[nodiscard]] bool
has_nearer_mesh_ready_batch( int const pending_dist ) {
    std::lock_guard<std::mutex> lock { g_stream.ready.mutex };
    for( auto const &ready : g_stream.ready.data ) {
        if( ready.failed || ready.models == nullptr || ready.models->empty() ) {
            continue;
        }
        if( false == file_chunk_meshes_ready( ready ) ) {
            continue;
        }
        if( ready.row == g_stream.center_row && ready.column == g_stream.center_column ) {
            return true;
        }
        auto const dist {
            section_manhattan_sections(
                ready.row,
                ready.column,
                g_stream.center_row,
                g_stream.center_column ) };
        if( dist + 1 < pending_dist ) {
            return true;
        }
    }
    return false;
}

void
suspend_pending_apply_to_ready() {
    if( false == g_stream.pending_apply.has_value() ) {
        return;
    }

    g_stream.pending_apply->apply_offset = g_stream.pending_apply_offset;
    PackSectionReady suspended { std::move( *g_stream.pending_apply ) };
    g_stream.pending_apply.reset();
    g_stream.pending_apply_offset = 0;
    note_apply_progress();

    {
        std::lock_guard<std::mutex> lock { g_stream.ready.mutex };
        g_stream.ready.data.push_back( std::move( suspended ) );
    }
    pack_bench_inc( &Eu7PackBench::stream_reenqueue );
}

[[nodiscard]] bool
file_chunk_meshes_ready(
    PackSectionReady const &batch,
    std::size_t const model_offset,
    std::size_t const model_count ) {
    if( batch.models == nullptr || batch.models->empty() ) {
        return false;
    }

    auto const end {
        std::min( batch.models->size(), model_offset + model_count ) };
    if( model_offset >= end ) {
        return true;
    }

    bool needs_mesh { false };
    bool all_ready { true };
    std::unordered_set<std::string> seen;
    seen.reserve( std::min( end - model_offset, std::size_t { 64 } ) );
    for( std::size_t i { model_offset }; i < end; ++i ) {
        auto const &model { ( *batch.models )[ i ] };
        if( eu7_pack_model_needs_full_load( model ) ) {
            continue;
        }
        auto const model_file {
            pack_model_mesh_path( model, batch.path_tables.get() ) };
        if( model_file.empty() || model_file == "notload" ) {
            continue;
        }
        auto normalized { std::string( model_file ) };
        replace_slashes( normalized );
        if( false == seen.insert( normalized ).second ) {
            continue;
        }
        needs_mesh = true;
        if( false == pack_mesh_ready_for_slice( normalized ) ) {
            all_ready = false;
        }
    }
    return needs_mesh ? all_ready : true;
}

void
request_missing_file_chunk_meshes(
    PackSectionReady const &batch,
    std::size_t const model_offset,
    std::size_t const model_count ) {
    auto const dist {
        section_manhattan_sections(
            batch.row,
            batch.column,
            g_stream.center_row,
            g_stream.center_column ) };

    auto const end {
        std::min( batch.models->size(), model_offset + model_count ) };
    if( batch.models == nullptr || model_offset >= end ) {
        return;
    }

    std::unordered_set<std::string> seen;
    seen.reserve( std::min( end - model_offset, std::size_t { 64 } ) );
    for( std::size_t i { model_offset }; i < end; ++i ) {
        auto const &model { ( *batch.models )[ i ] };
        if( eu7_pack_model_needs_full_load( model ) ) {
            continue;
        }
        auto const model_file {
            pack_model_mesh_path( model, batch.path_tables.get() ) };
        if( model_file.empty() || model_file == "notload" ) {
            continue;
        }
        auto normalized { std::string( model_file ) };
        replace_slashes( normalized );
        if( false == seen.insert( normalized ).second ) {
            continue;
        }
        if( false == pack_mesh_ready_for_slice( normalized ) ) {
            request_pack_mesh_load( normalized, dist );
        }
    }
}

void
block_until_file_chunk_meshes( PackSectionReady const &batch, double const block_budget_ms ) {
    for_each_chunk_mesh_path( batch, [&]( std::string const &path ) {
        if( pack_mesh_ready_for_slice( path ) ) {
            return;
        }
        (void)ensure_stream_pack_mesh( path, block_budget_ms );
    } );
}

[[nodiscard]] bool
batch_apply_meshes_ready(
    PackSectionReady const &batch,
    std::size_t const offset ) {
    (void)offset;
    return file_chunk_meshes_ready( batch );
}

void
request_missing_batch_meshes(
    PackSectionReady const &batch,
    std::size_t const offset ) {
    (void)offset;
    request_missing_file_chunk_meshes( batch );
}

void
prefetch_pack_section_meshes(
    Eu7Module const &module,
    int const row,
    int const column ) {
    auto const dist {
        section_manhattan_sections(
            row,
            column,
            g_stream.center_row,
            g_stream.center_column ) };
    for_each_pack_section_unique_mesh(
        module,
        row,
        column,
        [&]( std::string const &path ) {
            request_pack_mesh_load( path, dist );
        } );
}

void
note_apply_progress() {
}

[[nodiscard]] bool
apply_pending_chunk(
    double const budget_ms,
    std::size_t const max_instances,
    std::size_t const max_cold_meshes,
    double const cold_budget_ms,
    std::size_t const max_chunks ) {
    (void)max_cold_meshes;
    (void)cold_budget_ms;
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
        if( offset >= total ) {
            if( batch.section_final ) {
                finalize_section( batch );
            }
            release_pending_buffer();
            applied_work = true;
            continue;
        }

        auto const remaining { total - offset };
        auto const is_urgent { pending_apply_is_urgent() };
        auto const loading { false == g_loading_screen_dismissed };

        std::size_t slice_limit { remaining };
        if( loading ) {
            slice_limit = std::min(
                remaining,
                max_instances > 0 ? max_instances : kLoaderSliceInstances );
        }
        else if( max_instances > 0 ) {
            slice_limit = std::min( remaining, max_instances );
        }
        else {
            slice_limit = std::min(
                remaining,
                adaptive_slice_instances( total, is_urgent ) );
        }

        std::size_t const chunk_count { slice_limit };
        auto const planned_inst { chunk_count };
        pack_bench_inc( &Eu7PackBench::stream_inst_planned, planned_inst );
        if( is_urgent ) {
            pack_bench_inc( &Eu7PackBench::stream_chunks_urgent );
        }
        else if( chunk_count >= kHeavySectionModelThreshold ) {
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
        auto const pending_dist {
            section_manhattan_sections(
                batch.row,
                batch.column,
                g_stream.center_row,
                g_stream.center_column ) };

        resolve_pack_model_paths(
            batch.models->data() + offset,
            chunk_count,
            batch.path_tables.get() );

        {
            auto const phase_started { std::chrono::steady_clock::now() };
            auto const loading_phase { false == g_loading_screen_dismissed };
            auto const gameplay { gameplay_stream_mode() };
            (void)drain_pack_mesh_loader_ready( g_stream.mesh_cache );

            if( false == file_chunk_meshes_ready( batch, offset, chunk_count ) ) {
                pack_bench_inc( &Eu7PackBench::stream_apply_deferred );
                request_missing_file_chunk_meshes( batch, offset, chunk_count );

                if( gameplay ) {
                    suspend_pending_apply_to_ready();
                    return applied_work;
                }

                if( has_nearer_mesh_ready_batch( pending_dist ) ) {
                    suspend_pending_apply_to_ready();
                    return applied_work;
                }

                block_until_file_chunk_meshes( batch, std::max( budget_ms, 16.0 ) );

                if( false == file_chunk_meshes_ready( batch, offset, chunk_count ) ) {
                    if( has_nearer_mesh_ready_batch( pending_dist ) ) {
                        suspend_pending_apply_to_ready();
                    }
                    return applied_work;
                }
            }
            else if( loading_phase ) {
                (void)pump_pack_mesh_loader( 8.0, 4 );
            }

            cold_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - phase_started ).count();
        }
        pack_bench_inc( &Eu7PackBench::stream_inst_after_cold, chunk_count );

        {
            auto const phase_started { std::chrono::steady_clock::now() };
            if( false == gameplay_stream_mode() ) {
                tex_fetches = warm_pack_section_textures( batch, offset, chunk_count );
            }
            else {
                tex_fetches = warm_pack_section_textures(
                    batch, offset, chunk_count, kPackTextureWarmBudgetMs );
            }
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
        if(
            g_stream.mesh_lru.size() > kMeshCacheLruCap ||
            g_stream.nodedata_lru.size() > kNodedataCacheLruCap ) {
            evict_unreferenced_stream_caches();
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
                ( planned_inst != chunk_count ?
                    ( "/" + std::to_string( planned_inst ) ) :
                    std::string{} ) +
                " urg=" + std::to_string( is_urgent ? 1 : 0 ) +
                " heavy=" + std::to_string( chunk_count >= kHeavySectionModelThreshold ? 1 : 0 ) +
                " cold=" + std::to_string( cold_loads ) + "(" + std::to_string( static_cast<int>( cold_ms ) ) + "ms)" +
                " warm=" + std::to_string( tex_fetches ) + "(" + std::to_string( static_cast<int>( warm_ms ) ) + "ms)" +
                " apply=" + std::to_string( static_cast<int>( apply_ms ) ) + "ms" +
                " ring=" + std::to_string( static_cast<int>( g_stream.last_inner_ring * 100.f ) ) + "/" +
                std::to_string( static_cast<int>( g_stream.last_outer_ring * 100.f ) ) +
                " pending=" + std::to_string( offset + chunk_count ) + "/" + std::to_string( total ) +
                " sec=" + std::to_string( batch.row ) + "," + std::to_string( batch.column ) );
        }
        load_stats().pack_models += chunk_count;
        if( load_stats().pack_models == chunk_count && offset == 0 ) {
            WriteLog(
                "EU7 PACK: pierwszy apply CHNK inst=" + std::to_string( chunk_count ) +
                " sec=" + std::to_string( batch.row ) + "," + std::to_string( batch.column ) );
        }
        g_stream.pending_apply_offset = offset + chunk_count;
        batch.apply_offset = g_stream.pending_apply_offset;
        note_apply_progress();
        applied_work = true;
        ++chunks_done;

        if( g_stream.pending_apply_offset >= total ) {
            if( batch.section_final ) {
                finalize_section( batch );
            }
            release_pending_buffer();
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
        continue;
    }

    return applied_work;
}

void
maybe_log_loading_stream_status() {
    if( false == g_stream.active || g_loading_screen_dismissed ) {
        return;
    }

    static double s_last_log { 0.0 };
    auto const now { Timer::GetTime() };
    if( now - s_last_log < kStreamStatusLogIntervalSec ) {
        return;
    }
    s_last_log = now;

    std::size_t pending_total { 0 };
    if( g_stream.pending_apply.has_value() && g_stream.pending_apply->models != nullptr ) {
        pending_total = g_stream.pending_apply->models->size();
    }
    std::size_t job_count { 0 };
    {
        std::lock_guard<std::mutex> lock { g_stream.jobs.mutex };
        job_count = g_stream.jobs.data.size();
    }

    WriteLog(
        "EU7 PACK [loading]: pack_models=" + std::to_string( load_stats().pack_models ) +
        " ready_q=" + std::to_string( ready_queue_size() ) +
        " in_flight=" + std::to_string( g_stream.in_flight_sections.size() ) +
        " pending=" + std::to_string( g_stream.pending_apply_offset ) + "/" +
        std::to_string( pending_total ) +
        " jobs=" + std::to_string( job_count ) );
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
        section_stream_ring_progress( world_position, kSectionStreamTargetRadiusKm ) };

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

            if( job.next_chunk == 0 ) {
                prefetch_pack_section_meshes( module, job.row, job.column );
            }

            auto const entry { find_pack_entry( module, job.row, job.column ) };
            if( false == entry.has_value() || entry->model_count == 0 ) {
                enqueue_failed_section( job );
                continue;
            }

            std::unique_ptr<std::vector<Eu7Model>> models;
            std::vector<std::string> unique_textures;
            std::shared_ptr<Eu7PackSectionPathTables const> path_tables;
            std::uint32_t chunk_count { 1 };
            {
                PackBenchTimer const read_timer { &Eu7PackBench::worker_read_pack_ms };
                auto chunk {
                    read_pack_section_chunk_load(
                        module, job.row, job.column, job.next_chunk ) };
                chunk_count = chunk.chunk_count;
                unique_textures = std::move( chunk.unique_textures );
                path_tables = std::move( chunk.path_tables );
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
            result.unique_textures = std::move( unique_textures );
            result.path_tables = std::move( path_tables );
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

            auto const section_final { result.section_final };
            {
                std::lock_guard<std::mutex> lock { g_stream.ready.mutex };
                g_stream.ready.data.push_back( std::move( result ) );
            }

            pack_bench_inc( &Eu7PackBench::worker_chunks_decoded );
            if( section_final ) {
                pack_bench_inc( &Eu7PackBench::worker_sections_done );
            }

            if( job.next_chunk + 1 < chunk_count ) {
                PackSectionJob continue_job { job };
                continue_job.next_chunk = job.next_chunk + 1;
                auto const dist {
                    section_manhattan_sections(
                        continue_job.row,
                        continue_job.column,
                        g_stream.center_row,
                        g_stream.center_column ) };
                if( dist > kWorkerContinueMaxDistanceKm ) {
                    push_prioritized_job( continue_job );
                    pack_bench_inc( &Eu7PackBench::stream_worker_deferred_distant );
                }
                else {
                    std::lock_guard<std::mutex> lock { g_stream.jobs.mutex };
                    g_stream.jobs.data.push_front( std::move( continue_job ) );
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
    stop_pack_mesh_loader();
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
    start_pack_mesh_loader();

    WriteLog(
        "EU7 PACK: mesh queue loader started (main-thread), stream_workers=" +
        std::to_string( worker_count ) +
        ", radius=" + std::to_string( kSectionStreamTargetRadiusKm ) +
        "km, bootstrap_enqueue=" + std::to_string( kInitialBootstrapEnqueueRadius ) + "km" +
        ", bootstrap_replenish=" + std::to_string( kInitialBootstrapRadius ) + "km" +
        ", maintenance_inner=" + std::to_string( kSectionStreamGameplayRadiusKm ) + "km" +
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

[[nodiscard]] std::size_t
count_far_in_flight_sections( int const max_distance_km ) {
    if( g_stream.center_row < 0 || g_stream.center_column < 0 ) {
        return 0;
    }

    std::size_t count { 0 };
    for( auto const section_idx : g_stream.in_flight_sections ) {
        auto const row {
            static_cast<int>( section_idx / static_cast<std::size_t>( kRegionSideSectionCount ) ) };
        auto const column {
            static_cast<int>( section_idx % static_cast<std::size_t>( kRegionSideSectionCount ) ) };
        if(
            section_manhattan_sections(
                row,
                column,
                g_stream.center_row,
                g_stream.center_column ) > max_distance_km ) {
            ++count;
        }
    }
    return count;
}

[[nodiscard]] bool
should_block_far_enqueue(
    int const row,
    int const column,
    bool const lookahead_enqueue ) {
    if( g_stream.bootstrap_active || false == g_loading_screen_dismissed ) {
        return false;
    }
    if( g_stream.center_row < 0 || g_stream.center_column < 0 ) {
        return false;
    }

    auto const dist {
        section_manhattan_sections(
            row,
            column,
            g_stream.center_row,
            g_stream.center_column ) };
    if( dist <= kSectionStreamGameplayRadiusKm ) {
        return false;
    }

    if( in_radius_fill_phase() ) {
        return false;
    }

    if(
        lookahead_enqueue &&
        g_stream.last_inner_ring < kRingGatedInnerTarget ) {
        pack_bench_inc( &Eu7PackBench::stream_jobs_blocked_ring_inner );
        return true;
    }

    auto const speed { camera_stream_speed_mps() };
    if(
        speed > kFastFlightRingGateSpeedMps &&
        dist > kFastFlightApplyMaxDistanceKm &&
        count_far_in_flight_sections( kFastFlightApplyMaxDistanceKm ) >= kFastFlightMaxFarInFlight ) {
        pack_bench_inc( &Eu7PackBench::stream_jobs_blocked_far );
        return true;
    }

    return false;
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
    int const priority,
    bool const lookahead_enqueue ) {
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
    if( should_block_far_enqueue( row, column, lookahead_enqueue ) ) {
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

    if( g_stream.module != nullptr ) {
        prefetch_pack_section_meshes( *g_stream.module, row, column );
    }

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

    if( false == g_loading_screen_dismissed ) {
        enqueue_sections_around( row, column, kInitialBootstrapEnqueueRadius, world_position );
    }
    else if( in_radius_fill_phase() ) {
        enqueue_sections_around(
            row, column, kSectionStreamTargetRadiusKm, world_position );
    }
    else {
        enqueue_sections_around(
            row, column, kSectionStreamGameplayRadiusKm, world_position );
    }
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
        if( g_stream.pending_apply_offset >= total ) {
            return 1.0f;
        }
        return static_cast<float>( g_stream.pending_apply_offset ) /
            static_cast<float>( total );
    }

    {
        std::lock_guard<std::mutex> lock { g_stream.ready.mutex };
        for( auto const &batch : g_stream.ready.data ) {
            if( batch.section_idx == section_idx ) {
                return 0.25f;
            }
        }
    }

    if( g_stream.in_flight_sections.contains( section_idx ) ) {
        return 0.1f;
    }

    return 0.f;
}

[[nodiscard]] bool
center_section_stream_satisfied( glm::dvec3 const &world_position ) {
    auto const [center_row, center_column] { section_row_column( world_position ) };
    if( false == section_has_pack_models( center_row, center_column ) ) {
        return true;
    }

    auto const section_idx { section_index( center_row, center_column ) };
    if( g_stream.loaded_sections.contains( section_idx ) ) {
        return true;
    }

    if(
        g_stream.pending_apply.has_value() &&
        g_stream.pending_apply->section_idx == section_idx &&
        g_stream.pending_apply->models != nullptr &&
        false == g_stream.pending_apply->models->empty() ) {
        auto const total { g_stream.pending_apply->models->size() };
        if( g_stream.pending_apply_offset >= total ) {
            return true;
        }
    }

    return false;
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
        if( enqueue_section_if_needed( row, column, priority, true ) ) {
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

        if( g_stream.ready.data.empty() ) {
            return false;
        }

        auto const gameplay { gameplay_stream_mode() };

        auto const pick_best {
            [&]( int const max_distance, bool const require_mesh_ready )
                -> std::deque<PackSectionReady>::iterator {
                auto best_it { g_stream.ready.data.end() };
                auto best_dist { std::numeric_limits<int>::max() };
                auto best_mesh_ready { false };
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
                    auto const mesh_ready { file_chunk_meshes_ready( *it ) };
                    if( gameplay && require_mesh_ready && false == mesh_ready ) {
                        continue;
                    }
                    if(
                        dist < best_dist ||
                        ( dist == best_dist && mesh_ready && false == best_mesh_ready ) ||
                        ( dist == best_dist && best_it == g_stream.ready.data.end() ) ) {
                        best_dist = dist;
                        best_it = it;
                        best_mesh_ready = mesh_ready;
                    }
                    else if(
                        dist == best_dist && best_it != g_stream.ready.data.end() &&
                        it->section_idx < best_it->section_idx &&
                        mesh_ready == best_mesh_ready ) {
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
                        if( gameplay && false == file_chunk_meshes_ready( *it ) ) {
                            return g_stream.ready.data.end();
                        }
                        return it;
                    }
                }
                return g_stream.ready.data.end();
            } };

        auto const cam_speed { camera_stream_speed_mps() };
        auto const ring_deficit { stream_ring_deficit() };
        auto const radius_fill { in_radius_fill_phase() };
        auto max_apply_dist {
            cam_speed > 600.0 ?
                kFastFlightApplyMaxDistanceKm :
                ( cam_speed > 80.0 ? 3 : kSectionStreamGameplayRadiusKm ) };
        if( ring_deficit && false == radius_fill ) {
            max_apply_dist = cam_speed > 200.0 ?
                kFastFlightApplyMaxDistanceKm :
                kUrgentSectionDistanceKm;
        }

        auto best_it { pick_camera_section() };
        if( best_it != g_stream.ready.data.end() ) {
            pack_bench_inc( &Eu7PackBench::stream_dequeue_camera );
        }
        else if( radius_fill ) {
            best_it = pick_best( -1, true );
        }
        else {
            best_it = pick_best( max_apply_dist, true );
        }
        if( best_it == g_stream.ready.data.end() && false == radius_fill ) {
            auto const wider_dist {
                ring_deficit ?
                    std::max( max_apply_dist, kFastFlightApplyMaxDistanceKm ) :
                    kSectionStreamGameplayRadiusKm };
            best_it = pick_best( wider_dist, true );
        }
        if( best_it == g_stream.ready.data.end() && false == g_stream.ready.data.empty() ) {
            best_it = pick_best( -1, true );
        }
        if(
            best_it == g_stream.ready.data.end() &&
            gameplay &&
            false == g_stream.ready.data.empty() ) {
            best_it = pick_best( -1, false );
        }
        if( best_it == g_stream.ready.data.end() ) {
            return false;
        }

        g_stream.pending_apply.emplace( std::move( *best_it ) );
        g_stream.ready.data.erase( best_it );
    }

    g_stream.pending_apply_offset = g_stream.pending_apply->apply_offset;

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
            if( it->failed || it->models == nullptr || it->models->empty() ) {
                continue;
            }
            auto const tex_total {
                false == it->unique_textures.empty() ?
                    it->unique_textures.size() :
                    it->models->size() };
            if( it->texture_warm_offset >= tex_total ) {
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
        auto const use_utex { false == batch.unique_textures.empty() };

        while( true ) {
            auto const tex_total {
                use_utex ? batch.unique_textures.size() : batch.models->size() };
            if( batch.texture_warm_offset >= tex_total ) {
                break;
            }

            auto const remaining { tex_total - batch.texture_warm_offset };
            auto const slice { std::min( remaining, kReadyTexturePrefetchSlice ) };

            if( use_utex ) {
                (void)warm_pack_texture_paths_main(
                    batch.unique_textures.data() + batch.texture_warm_offset,
                    slice );
            }
            else {
                (void)warm_pack_textures_main(
                    batch.models->data() + batch.texture_warm_offset,
                    slice );
            }
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
        if( it->failed || it->models == nullptr || it->models->empty() ) {
            continue;
        }
        if( it->umes_prefetch_offset >= it->models->size() ) {
            continue;
        }
        if( g_stream.center_row >= 0 && g_stream.center_column >= 0 ) {
            auto const dist {
                section_manhattan_sections(
                    it->row,
                    it->column,
                    g_stream.center_row,
                    g_stream.center_column ) };
            if( dist > umes_prefetch_max_distance_km() ) {
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
        while( batch.umes_prefetch_offset < batch.models->size() ) {
            auto const &model { ( *batch.models )[ batch.umes_prefetch_offset ] };
            if( eu7_pack_model_needs_full_load( model ) ) {
                ++batch.umes_prefetch_offset;
                continue;
            }
            auto const &model_file { model.model_file };
            if( model_file.empty() || model_file == "notload" ) {
                ++batch.umes_prefetch_offset;
                continue;
            }
            if( pack_mesh_ready_for_slice( model_file ) ) {
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

            request_pack_mesh_load(
                model_file,
                section_manhattan_sections(
                    batch.row,
                    batch.column,
                    g_stream.center_row,
                    g_stream.center_column ) );
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
drain_frame_budget( double const budget_ms ) {
    if( budget_ms <= 0.0 ) {
        return;
    }

    auto const started { std::chrono::steady_clock::now() };
    std::size_t chunks_done { 0 };
    while( chunks_done < kFrameMaxChunksPerDrain ) {
        auto const elapsed_ms {
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started ).count() };
        auto const remaining_ms { budget_ms - elapsed_ms };
        if( remaining_ms <= 0.0 ) {
            break;
        }

        if( false == apply_pending_chunk(
                remaining_ms,
                kFrameSliceInstances,
                kFrameSliceColdMeshes,
                kFrameColdBudgetMs,
                1 ) ) {
            break;
        }
        ++chunks_done;

        if(
            false == g_stream.pending_apply.has_value() &&
            ready_queue_size() == 0 ) {
            break;
        }
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
        g_stream.radius_fill_active = false;
        g_stream_max_in_flight = 16;
        g_stream_max_ready = 8;
        return;
    }

    auto const stream_position { resolve_section_stream_position( world_position ) };
    auto const cam_speed { camera_stream_speed_mps() };
    g_stream.last_inner_ring =
        section_stream_ring_progress( stream_position, kSectionStreamGameplayRadiusKm );
    g_stream.last_outer_ring =
        section_stream_ring_progress( stream_position, kSectionStreamTargetRadiusKm );

    auto const was_radius_fill { g_stream.radius_fill_active };
    g_stream.radius_fill_active = false == section_stream_ready_around(
        stream_position, kSectionStreamTargetRadiusKm );
    if( false == was_radius_fill && g_stream.radius_fill_active ) {
        WriteLog(
            "EU7 PACK: faza radius_fill — docelowy promien " +
            std::to_string( kSectionStreamTargetRadiusKm ) + "km, ring=" +
            std::to_string( static_cast<int>( g_stream.last_outer_ring * 100.f ) ) + "%" );
    }
    else if( was_radius_fill && false == g_stream.radius_fill_active ) {
        WriteLog(
            "EU7 PACK: faza maintenance — promien " +
            std::to_string( kSectionStreamTargetRadiusKm ) + "km zapelniony, ring=" +
            std::to_string( static_cast<int>( g_stream.last_outer_ring * 100.f ) ) + "%" );
    }

    if( g_stream.radius_fill_active ) {
        g_stream_catchup = true;
        g_stream_max_in_flight = kRadiusFillMaxInFlight;
        g_stream_max_ready = kRadiusFillMaxReady;
        return;
    }

    g_stream_catchup = (
        cam_speed > 80.0 ||
        g_stream.last_inner_ring < 0.98f );

    if( g_stream_catchup ) {
        g_stream_max_in_flight = kCatchupMaxInFlightSections;
        g_stream_max_ready = kCatchupMaxReadySections;

        if(
            cam_speed > kRingBoostInFlightSpeedMps &&
            g_stream.last_inner_ring < kRingBoostOuterThreshold ) {
            g_stream_max_in_flight = std::max(
                g_stream_max_in_flight, kRingStrongBoostMaxInFlight );
            g_stream_max_ready = std::max(
                g_stream_max_ready, kRingStrongBoostMaxReady );
        }
        else if(
            cam_speed > kRingBoostInFlightSpeedMps &&
            g_stream.last_inner_ring < kRingStrongBoostOuterThreshold ) {
            g_stream_max_in_flight = std::max(
                g_stream_max_in_flight, kRingBoostMaxInFlight );
            g_stream_max_ready = std::max(
                g_stream_max_ready, kRingBoostMaxReady );
        }
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
    enqueue_sections_around( row, column, kInitialBootstrapEnqueueRadius, initial );
    auto const prime_started { std::chrono::steady_clock::now() };
    while( true ) {
        drain_until_budget( kBootstrapDrainMs );
        if( load_stats().pack_models > 0 ) {
            break;
        }
        auto const elapsed_ms {
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - prime_started ).count() };
        if( elapsed_ms >= 4000 ) {
            break;
        }
        if(
            false == g_stream.pending_apply.has_value() &&
            ready_queue_size() == 0 &&
            g_stream.in_flight_sections.empty() ) {
            break;
        }
    }
    WriteLog(
        "EU7 PACK: prime drain modele=" + std::to_string( load_stats().pack_models ) +
        " ready_q=" + std::to_string( ready_queue_size() ) +
        " in_flight=" + std::to_string( g_stream.in_flight_sections.size() ) );
    g_stream.bootstrap_active = false;
    g_stream.bootstrap_pending = false;
    g_stream.radius_fill_active = false;
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
    g_stream.radius_fill_active = false;

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

    auto const prev_center_row { g_stream.center_row };
    auto const prev_center_column { g_stream.center_column };
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

    update_unload_keep_radius_hysteresis();

    if( center_moved ) {
        reprioritize_job_queue();
        maybe_unload_distant_sections( prev_center_row, prev_center_column );
    }

    sync_stream_limits( stream_position );

    auto const radius_fill { in_radius_fill_phase() };
    auto const inner_ring_ready {
        section_stream_ready_around( stream_position, kSectionStreamGameplayRadiusKm ) };
    auto const ring_radius {
        radius_fill ?
            kSectionStreamTargetRadiusKm :
            kSectionStreamGameplayRadiusKm };

    auto const cam_speed { camera_stream_speed_mps() };
    auto const reenqueue_distance {
        radius_fill ?
            kCatchupReenqueueDistanceM :
            ( ( g_stream_catchup || cam_speed > 80.0 ) ?
                std::clamp( cam_speed * 0.25, kCatchupReenqueueDistanceM, kReenqueueDistanceM ) :
                kReenqueueDistanceM ) };

    auto const current_section_unloaded {
        section_has_pack_models( center_row, center_column )
        && false == g_stream.loaded_sections.contains( section_index( center_row, center_column ) ) };

    auto const ring_deficit { stream_ring_deficit() };
    auto const desperate { ring_deficit || current_section_unloaded };

    auto max_lookahead { 0 };
    if( radius_fill ) {
        max_lookahead = 0;
    }
    else if( inner_ring_ready || desperate ) {
        if( desperate && cam_speed > 200.0 ) {
            max_lookahead = cam_speed > 600.0 ? 10 : 8;
        }
        else {
            max_lookahead = ( g_stream_catchup || cam_speed > 200.0 ) ?
                kMaintenanceLookaheadMax :
                ( cam_speed > 80.0 ? 6 : 4 );
        }
        max_lookahead = std::min( max_lookahead, kMaintenanceLookaheadMax );
    }

    auto const travel_forward { guess_travel_forward() };

    auto const ring_incomplete {
        radius_fill ?
            false == section_stream_ready_around(
                stream_position, kSectionStreamTargetRadiusKm ) :
            ( current_section_unloaded || false == inner_ring_ready ) };
    auto const should_reenqueue {
        center_moved
        || false == g_stream.has_last_enqueue_position
        || moved_since_enqueue >= reenqueue_distance };

    if( current_section_unloaded ) {
        enqueue_section_if_needed( center_row, center_column, 0 );
    }

    if( should_reenqueue || ring_incomplete ) {
        enqueue_sections_around( center_row, center_column, ring_radius, stream_position );
        g_stream.last_enqueue_position = stream_position;
        g_stream.has_last_enqueue_position = true;
        if( should_reenqueue || ring_incomplete ) {
            pack_bench_inc( &Eu7PackBench::stream_reenqueue );
        }
    }

    if(
        false == radius_fill &&
        ( inner_ring_ready || desperate ) &&
        max_lookahead > 0 &&
        cam_speed > 50.0 ) {
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

    (void)drain_pack_mesh_loader_ready( g_stream.mesh_cache );

    auto const stream_position {
        resolve_section_stream_position(
            g_loading_screen_dismissed ?
                Global.pCamera.Pos :
                stream_loading_position() ) };
    auto const loading { false == g_loading_screen_dismissed };

    if( false == loading ) {
        sync_stream_limits( stream_position );
    }

    if( loading ) {
        if( section_stream_needs_bootstrap() ) {
            kick_section_stream_bootstrap();
        }
        else if(
            stream_position.x != 0.0 || stream_position.y != 0.0 || stream_position.z != 0.0 ) {
            replenish_bootstrap_ring( stream_position );
        }
        (void)pump_pack_mesh_loader( 8.0, 4 );
    }
    else {
        if(
            stream_position.x != 0.0 || stream_position.y != 0.0 || stream_position.z != 0.0 ) {
            replenish_bootstrap_ring( stream_position );
        }

        auto const radius_fill { in_radius_fill_phase() };
        auto const gpu_init_budget { radius_fill ? 12.0 : ( g_stream_catchup ? 14.0 : 10.0 ) };
        auto const gpu_init_loads {
            radius_fill ?
                std::size_t { 12 } :
                ( g_stream_catchup ? std::size_t { 14 } : std::size_t { 10 } ) };
        (void)pump_pack_mesh_loader( gpu_init_budget, gpu_init_loads );
        if( false == radius_fill ) {
            prefetch_ready_queue_umes_worker(
                kReadyUmesPrefetchBudgetMs,
                g_stream_catchup ?
                    kReadyUmesPrefetchMaxMeshes + kReadyUmesPrefetchMaxMeshes / 2 :
                    kReadyUmesPrefetchMaxMeshes );
            prefetch_ready_queue_textures( ready_texture_prefetch_budget_ms() );
        }
    }

    auto const radius_fill { false == loading && in_radius_fill_phase() };
    drain_apply_budget(
        loading ? kLoaderDrainBudgetMs : frame_drain_budget_ms(),
        loading ? kLoaderSliceInstances :
            ( radius_fill ? kRadiusFillSliceInstances : 0 ),
        0,
        0,
        loading ? kLoaderSectionsPerDrain : gameplay_max_chunks_per_drain() );

    if( loading ) {
        maybe_log_loading_stream_status();
    }
    else {
        maybe_log_stream_status( stream_position );
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
        "EU7 PACK: bootstrap async " + std::to_string( kInitialBootstrapEnqueueRadius ) + "km, sekcja " +
        std::to_string( row ) + "," + std::to_string( column ) );

    enqueue_sections_around( row, column, kInitialBootstrapEnqueueRadius, position );
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

    simulation::State.drain_deferred_eu7_trainsets( max_drain_ms > 0.0 ? 8.0 : 0.0 );

    auto position { resolve_stream_position() };
    if( position.x == 0.0 && position.y == 0.0 && position.z == 0.0 ) {
        position = guess_initial_stream_position( stream_module() );
    }
    position = resolve_section_stream_position( position );

    if( section_stream_needs_bootstrap() ) {
        if( position.x != 0.0 || position.y != 0.0 || position.z != 0.0 ) {
            kick_section_stream_bootstrap();
        }
    }
    else if( position.x != 0.0 || position.y != 0.0 || position.z != 0.0 ) {
        update_section_stream( position );
    }

    if( max_drain_ms <= 0.0 ) {
        WriteLog(
            "EU7 PACK: preload async kick, sekcji=" +
            std::to_string( load_stats().pack_sections_loaded ) +
            " modele=" + std::to_string( load_stats().pack_models ) );
        return;
    }

    auto const started { std::chrono::steady_clock::now() };
    while( true ) {
        drain_until_budget( kLoaderDrainBudgetMs );
        if( position.x != 0.0 || position.y != 0.0 || position.z != 0.0 ) {
            update_section_stream( position );
        }

        auto const elapsed_ms {
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - started ).count() };
        if( static_cast<double>( elapsed_ms ) >= max_drain_ms ) {
            break;
        }
        if( position.x != 0.0 || position.y != 0.0 || position.z != 0.0 ) {
            if( section_stream_ready_around(
                    position, kSectionStreamLoadingDismissRadiusKm ) ) {
                break;
            }
            if(
                center_section_stream_satisfied( position ) &&
                section_stream_ring_progress(
                    position, kSectionStreamLoadingDismissRadiusKm ) >= kEarlyDismissRingProgress ) {
                break;
            }
        }
        if(
            g_stream.pending_apply.has_value() || ready_queue_size() > 0 ||
            false == g_stream.in_flight_sections.empty() ) {
            continue;
        }
        if( load_stats().pack_models >= 400 ) {
            break;
        }
        std::this_thread::sleep_for( std::chrono::milliseconds( 4 ) );
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
    g_stream.radius_fill_active = true;
    g_ring_ready_since.reset();
    simulation::is_ready = true;

    WriteLog(
        "EU7 PACK: loading screen dismissed — faza radius_fill do " +
        std::to_string( kSectionStreamTargetRadiusKm ) + "km" );

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
    (void)radius_km;
    if( false == g_stream.active || g_loading_screen_dismissed ) {
        return false;
    }

    if( load_stats().pack_models >= kBootstrapApplyModelThreshold ) {
        WriteLog(
            "EU7 PACK: loading screen off, pack_models=" +
            std::to_string( load_stats().pack_models ) );
        dismiss_loading_screen();
        return false;
    }

    auto const [center_row, center_column] { section_row_column( world_position ) };
    if( section_has_pack_models( center_row, center_column ) ) {
        auto const section_idx { section_index( center_row, center_column ) };
        if( g_stream.loaded_sections.contains( section_idx ) ) {
            WriteLog( "EU7 PACK: loading screen off, centrum zaladowane" );
            dismiss_loading_screen();
            return false;
        }
    }

    if( g_loading_block_started.has_value() ) {
        auto const blocked_for {
            std::chrono::steady_clock::now() - *g_loading_block_started };
        if( blocked_for >= kLoadingScreenMaxBlockSec ) {
            ErrorLog(
                "EU7 PACK: loading screen timeout — wchodzę w świat (pack_models=" +
                std::to_string( load_stats().pack_models ) + ", ready_q=" +
                std::to_string( ready_queue_size() ) + ", in_flight=" +
                std::to_string( g_stream.in_flight_sections.size() ) + ")" );
            dismiss_loading_screen();
            return false;
        }
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
    return g_stream_catchup || pending_apply_is_urgent() || in_radius_fill_phase();
}

} // namespace scene::eu7
