/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <chrono>
#include <cstdint>

#include <glm/glm.hpp>

namespace scene::eu7 {

// Liczniki i czasy PACK — osobno od Eu7LoadStats (deserialize / SCM).
struct Eu7PackBench {
    // --- worker (async read PACK) ---
    double worker_read_pack_ms { 0.0 };
    std::uint64_t worker_chunks_decoded { 0 };
    std::uint64_t worker_sections_done { 0 };
    std::uint64_t sections_finalized { 0 };
    std::uint64_t worker_models_decoded { 0 };
    std::uint64_t worker_failures { 0 };

    // --- main thread: apply ---
    double main_read_pack_ms { 0.0 };
    double main_cold_getmodel_ms { 0.0 };
    double main_load_eu7_pack_ms { 0.0 };
    double main_load_eu7_full_ms { 0.0 };
    double main_region_insert_ms { 0.0 };

    std::uint64_t main_drain_calls { 0 };
    std::uint64_t main_cold_getmodel_calls { 0 };
    std::uint64_t main_instances_applied { 0 };
    std::uint64_t main_pack_fast_loads { 0 };
    std::uint64_t main_pack_full_loads { 0 };
    std::uint64_t main_region_inserts { 0 };

    std::uint64_t main_drain_wait_cold_mesh { 0 };
    std::uint64_t main_drain_wait_worker { 0 };
    std::uint64_t main_drain_idle { 0 };

    std::uint64_t peak_ready_queue { 0 };
    std::uint64_t peak_in_flight { 0 };
    std::uint64_t peak_pending_cold_meshes { 0 };

    std::uint64_t main_texture_assign_fail { 0 };
    std::uint64_t main_texture_warm_miss { 0 };
    std::uint64_t stream_reenqueue { 0 };
    std::uint64_t stream_lookahead_enqueue { 0 };

    // --- stream diagnostics (gdzie ginie czas / skad przyciecie) ---
    std::uint64_t stream_drain_catchup_urgent { 0 };
    std::uint64_t stream_drain_catchup { 0 };
    std::uint64_t stream_drain_gameplay { 0 };
    std::uint64_t stream_drain_loader { 0 };
    std::uint64_t stream_preempt_pending { 0 };
    std::uint64_t stream_cold_slice_truncated { 0 };
    std::uint64_t stream_mesh_session_hit { 0 };
    std::uint64_t stream_mesh_global_hit { 0 };
    std::uint64_t stream_mesh_disk_load { 0 };
    std::uint64_t stream_mesh_async_queued { 0 };
    std::uint64_t stream_mesh_async_ready { 0 };
    std::uint64_t stream_mesh_async_drained { 0 };
    std::uint64_t stream_mesh_async_wait_timeout { 0 };
    std::uint64_t loader_thread_disk_loads { 0 };
    double loader_thread_getmodel_ms { 0.0 };
    std::uint64_t stream_chunks_urgent { 0 };
    std::uint64_t stream_chunks_heavy { 0 };
    std::uint64_t stream_chunks_light { 0 };
    std::uint64_t stream_dequeue_near { 0 };
    std::uint64_t stream_dequeue_far { 0 };
    std::uint64_t stream_dequeue_camera { 0 };
    std::uint64_t stream_dequeue_wait_near { 0 };
    std::uint64_t stream_jobs_blocked_far { 0 };
    std::uint64_t stream_jobs_blocked_ring_inner { 0 };
    std::uint64_t stream_jobs_blocked_ring_outer { 0 };
    std::uint64_t stream_worker_deferred_distant { 0 };
    std::uint64_t stream_apply_stuck_skip { 0 };
    std::uint64_t stream_apply_deferred { 0 };
    std::uint64_t stream_apply_defer_bypass { 0 };
    std::uint64_t stream_sparse_apply_skip { 0 };
    std::uint64_t stream_sections_unloaded { 0 };
    std::uint64_t stream_unload_instances { 0 };
    std::uint64_t stream_mesh_cache_evictions { 0 };
    std::uint64_t stream_nodedata_cache_evictions { 0 };
    std::uint64_t stream_texture_cache_evictions { 0 };
    double stream_prefetch_ready_tex_ms { 0.0 };
    double stream_prefetch_ready_umes_ms { 0.0 };
    std::uint64_t stream_prefetch_ready_umes_loads { 0 };
    std::uint64_t peak_pending_total { 0 };
    std::uint64_t stream_inst_planned { 0 };
    std::uint64_t stream_inst_after_cold { 0 };

    // --- main: pointer chunk apply (diag zwiechy) ---
    double main_chunk_cold_ms { 0.0 };
    double main_chunk_warm_tex_ms { 0.0 };
    double main_chunk_pointer_apply_ms { 0.0 };
    std::uint64_t main_chunks { 0 };
    std::uint64_t main_chunk_instances { 0 };
    std::uint64_t main_chunk_cold_loads { 0 };
    std::uint64_t main_chunk_tex_fetches { 0 };
    std::uint64_t chunk_slow_8ms { 0 };
    std::uint64_t chunk_slow_16ms { 0 };
    std::uint64_t drain_budget_stops { 0 };
    double peak_chunk_ms { 0.0 };
    std::uint64_t peak_chunk_instances { 0 };
    std::uint64_t peak_chunk_cold { 0 };
    double last_chunk_ms { 0.0 };
    double last_apply_ms { 0.0 };
    std::uint64_t last_chunk_instances { 0 };
    std::uint64_t last_chunk_cold { 0 };

    [[nodiscard]] double accounted_ms() const;
    [[nodiscard]] double chunk_accounted_ms() const;
};

class PackBenchTimer {
public:
    explicit PackBenchTimer( double Eu7PackBench::*Field );
    ~PackBenchTimer();

    PackBenchTimer( PackBenchTimer const & ) = delete;
    PackBenchTimer &operator=( PackBenchTimer const & ) = delete;

private:
    double Eu7PackBench::*const m_field;
    std::chrono::steady_clock::time_point m_start;
};

[[nodiscard]] Eu7PackBench &
pack_bench();

// Po dismiss overlay — liczniki runtime streamingu (latanie kamera / jazda).
[[nodiscard]] Eu7PackBench &
pack_bench_stream();

// Snapshot z fazy bootstrap (ekran ladowania).
[[nodiscard]] Eu7PackBench const &
pack_bench_bootstrap();

[[nodiscard]] bool
pack_bench_stream_phase_active();

void
reset_pack_bench();

// Wywolac raz przy dismiss overlay — oddziela bootstrap od streamu.
void
pack_bench_begin_stream_phase();

void
pack_bench_note_ready_queue( std::size_t Size );

void
pack_bench_note_in_flight( std::size_t Size );

void
pack_bench_note_pending_cold_meshes( std::size_t Size );

void
pack_bench_note_pending_total( std::size_t Total );

void
pack_bench_inc( std::uint64_t Eu7PackBench::*Field, std::uint64_t Amount = 1 );

void
pack_bench_note_chunk(
    double WallMs,
    std::size_t Instances,
    std::size_t ColdMeshes,
    double ColdMs,
    double WarmTexMs,
    double PointerApplyMs );

void
log_pack_texture_fail( std::string const &TexturePath );

void
log_pack_bench();

void
log_pack_stream_bench();

// Na wyjsciu z jazdy / reset sceny — jednorazowy dump bez czekania na timer.
void
flush_pack_stream_bench();

// Co kilka sekund / przy szybkiej kamerze — stan kolejek i ringu.
void
log_pack_stream_status(
    glm::dvec3 const &CameraPosition,
    double CameraSpeedMps,
    float InnerRingProgress,
    float OuterRingProgress,
    std::size_t ReadyQueue,
    std::size_t InFlight,
    std::size_t PendingOffset,
    std::size_t PendingTotal,
    double DrainBudgetMs,
    std::size_t ChunkLimit );

} // namespace scene::eu7
