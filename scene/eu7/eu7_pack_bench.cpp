/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/eu7/eu7_pack_mesh_loader.h"
#include "scene/eu7/eu7_pack_bench.h"

#include "utilities/Logs.h"
#include "utilities/utilities.h"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace scene::eu7 {
namespace {

Eu7PackBench g_pack_bench;
Eu7PackBench g_pack_bench_stream;
Eu7PackBench g_pack_bench_bootstrap;
bool g_stream_phase { false };
std::unordered_set<std::string> g_logged_texture_fails;

struct BenchLine {
    std::string label;
    double ms { 0.0 };
    std::string detail;
};

[[nodiscard]] std::string
format_seconds( double const ms ) {
    std::ostringstream out;
    out << std::fixed << std::setprecision( 1 ) << ( ms / 1000.0 ) << 's';
    return out.str();
}

[[nodiscard]] std::string
format_ms_per( double const ms, std::uint64_t const count ) {
    if( count == 0 ) {
        return "n/a";
    }
    std::ostringstream out;
    out << std::fixed << std::setprecision( 2 ) << ( ms / static_cast<double>( count ) ) << " ms/item";
    return out.str();
}

void
note_peak(
    std::uint64_t Eu7PackBench::*const field,
    std::size_t const size ) {
    auto &total_peak { g_pack_bench.*field };
    total_peak = std::max( total_peak, size );
    if( g_stream_phase ) {
        auto &stream_peak { g_pack_bench_stream.*field };
        stream_peak = std::max( stream_peak, size );
    }
}

void
add_bench_ms( double Eu7PackBench::*const field, double const delta ) {
    g_pack_bench.*field += delta;
    if( g_stream_phase ) {
        g_pack_bench_stream.*field += delta;
    }
}

void
log_pack_bench_impl( Eu7PackBench const &bench, char const *const title ) {
    auto const total { bench.accounted_ms() };

    if( total <= 0.0 && bench.main_drain_calls == 0 && bench.worker_sections_done == 0 ) {
        return;
    }

    auto const main_apply_sum {
        bench.main_load_eu7_pack_ms + bench.main_load_eu7_full_ms + bench.main_region_insert_ms };
    auto const chunk_sum {
        bench.main_chunk_cold_ms + bench.main_chunk_warm_tex_ms + bench.main_chunk_pointer_apply_ms };

    std::vector<BenchLine> lines {
        { "worker read_pack", bench.worker_read_pack_ms,
            "chunks=" + std::to_string( bench.worker_chunks_decoded ) +
            " sections=" + std::to_string( bench.worker_sections_done ) +
            " finalized=" + std::to_string( bench.sections_finalized ) +
            " models=" + std::to_string( bench.worker_models_decoded ) +
            " fail=" + std::to_string( bench.worker_failures ) },
        { "main read_pack", bench.main_read_pack_ms,
            "models=" + std::to_string( bench.worker_models_decoded ) },
        { "main GetModel (cold)", bench.main_cold_getmodel_ms,
            "calls=" + std::to_string( bench.main_cold_getmodel_calls ) + " " +
            format_ms_per( bench.main_cold_getmodel_ms, bench.main_cold_getmodel_calls ) },
        { "main LoadEu7Pack", bench.main_load_eu7_pack_ms,
            "loads=" + std::to_string( bench.main_pack_fast_loads ) + " " +
            format_ms_per( bench.main_load_eu7_pack_ms, bench.main_pack_fast_loads ) },
        { "main LoadEu7 (full)", bench.main_load_eu7_full_ms,
            "loads=" + std::to_string( bench.main_pack_full_loads ) + " " +
            format_ms_per( bench.main_load_eu7_full_ms, bench.main_pack_full_loads ) },
        { "main Region::insert", bench.main_region_insert_ms,
            "inserts=" + std::to_string( bench.main_region_inserts ) + " " +
            format_ms_per( bench.main_region_insert_ms, bench.main_region_inserts ) },
        { "main apply (load+region)", main_apply_sum,
            "instances=" + std::to_string( bench.main_instances_applied ) + " " +
            format_ms_per( main_apply_sum, bench.main_instances_applied ) },
        { "chunk cold preload", bench.main_chunk_cold_ms,
            "loads=" + std::to_string( bench.main_chunk_cold_loads ) + " " +
            format_ms_per( bench.main_chunk_cold_ms, bench.main_chunk_cold_loads ) },
        { "chunk warm textures", bench.main_chunk_warm_tex_ms,
            "fetches=" + std::to_string( bench.main_chunk_tex_fetches ) + " " +
            format_ms_per( bench.main_chunk_warm_tex_ms, bench.main_chunk_tex_fetches ) },
        { "chunk pointer apply", bench.main_chunk_pointer_apply_ms,
            "chunks=" + std::to_string( bench.main_chunks ) + " inst=" +
            std::to_string( bench.main_chunk_instances ) + " " +
            format_ms_per( bench.main_chunk_pointer_apply_ms, bench.main_chunk_instances ) },
        { "chunk wall (phases)", chunk_sum,
            "chunks=" + std::to_string( bench.main_chunks ) + " slow8=" +
            std::to_string( bench.chunk_slow_8ms ) + " slow16=" +
            std::to_string( bench.chunk_slow_16ms ) },
    };

    std::sort(
        lines.begin(),
        lines.end(),
        []( BenchLine const &lhs, BenchLine const &rhs ) {
            return lhs.ms > rhs.ms;
        } );

    WriteLog( std::string{ title } + " (accounted " + format_seconds( total ) + "):" );
    for( auto const &line : lines ) {
        if( line.ms <= 0.0 ) {
            continue;
        }
        auto const pct {
            total > 0.0 ?
                ( 100.0 * line.ms / total ) :
                0.0 };
        std::ostringstream row;
        row << std::fixed << std::setprecision( 1 );
        row << "  " << line.label << ": " << format_seconds( line.ms );
        if( total > 0.0 ) {
            row << " (" << pct << "%)";
        }
        if( false == line.detail.empty() ) {
            row << "  " << line.detail;
        }
        WriteLog( row.str() );
    }

    WriteLog(
        "  drain: calls=" + std::to_string( bench.main_drain_calls ) +
        " wait_cold_frames=" + std::to_string( bench.main_drain_wait_cold_mesh ) +
        " wait_worker_frames=" + std::to_string( bench.main_drain_wait_worker ) +
        " idle_frames=" + std::to_string( bench.main_drain_idle ) );
    WriteLog(
        "  stream: reenqueue=" + std::to_string( bench.stream_reenqueue ) +
        " lookahead=" + std::to_string( bench.stream_lookahead_enqueue ) +
        " tex_fail=" + std::to_string( bench.main_texture_assign_fail ) +
        " warm_miss=" + std::to_string( bench.main_texture_warm_miss ) );
    WriteLog(
        "  diag drain: urg=" + std::to_string( bench.stream_drain_catchup_urgent ) +
        " catchup=" + std::to_string( bench.stream_drain_catchup ) +
        " gameplay=" + std::to_string( bench.stream_drain_gameplay ) +
        " loader=" + std::to_string( bench.stream_drain_loader ) +
        " preempt=" + std::to_string( bench.stream_preempt_pending ) +
        " budget_stops=" + std::to_string( bench.drain_budget_stops ) );
    WriteLog(
        "  diag chunk: urg=" + std::to_string( bench.stream_chunks_urgent ) +
        " heavy=" + std::to_string( bench.stream_chunks_heavy ) +
        " light=" + std::to_string( bench.stream_chunks_light ) +
        " cold_trunc=" + std::to_string( bench.stream_cold_slice_truncated ) +
        " inst_plan=" + std::to_string( bench.stream_inst_planned ) +
        " inst_cold=" + std::to_string( bench.stream_inst_after_cold ) );
    WriteLog(
        "  diag mesh: sess_hit=" + std::to_string( bench.stream_mesh_session_hit ) +
        " glob_hit=" + std::to_string( bench.stream_mesh_global_hit ) +
        " disk=" + std::to_string( bench.stream_mesh_disk_load ) +
        " async_q=" + std::to_string( bench.stream_mesh_async_queued ) +
        " async_r=" + std::to_string( bench.stream_mesh_async_ready ) +
        " loader=" + std::to_string( bench.loader_thread_disk_loads ) +
        " dequeue_near=" + std::to_string( bench.stream_dequeue_near ) +
        " far=" + std::to_string( bench.stream_dequeue_far ) +
        " cam=" + std::to_string( bench.stream_dequeue_camera ) +
        " wait_near=" + std::to_string( bench.stream_dequeue_wait_near ) +
        " block_far=" + std::to_string( bench.stream_jobs_blocked_far ) +
        " block_ring_in=" + std::to_string( bench.stream_jobs_blocked_ring_inner ) +
        " block_ring_out=" + std::to_string( bench.stream_jobs_blocked_ring_outer ) +
        " defer_dist=" + std::to_string( bench.stream_worker_deferred_distant ) +
        " stuck_skip=" + std::to_string( bench.stream_apply_stuck_skip ) +
        " apply_defer=" + std::to_string( bench.stream_apply_deferred ) +
        " defer_bypass=" + std::to_string( bench.stream_apply_defer_bypass ) +
        " sparse_skip=" + std::to_string( bench.stream_sparse_apply_skip ) );
    WriteLog(
        "  diag ready_tex_ms=" + std::to_string( static_cast<int>( bench.stream_prefetch_ready_tex_ms ) ) +
        " ready_umes_ms=" + std::to_string( static_cast<int>( bench.stream_prefetch_ready_umes_ms ) ) +
        " umes_loads=" + std::to_string( bench.stream_prefetch_ready_umes_loads ) +
        " peak_pending=" + std::to_string( bench.peak_pending_total ) );
    WriteLog(
        "  peaks: ready_q=" + std::to_string( bench.peak_ready_queue ) +
        " in_flight=" + std::to_string( bench.peak_in_flight ) +
        " cold_q=" + std::to_string( bench.peak_pending_cold_meshes ) );
    if( bench.main_chunks > 0 ) {
        WriteLog(
            "  chunk peaks: wall_ms=" + std::to_string( static_cast<int>( bench.peak_chunk_ms ) ) +
            " inst=" + std::to_string( bench.peak_chunk_instances ) +
            " cold=" + std::to_string( bench.peak_chunk_cold ) +
            " last_ms=" + std::to_string( static_cast<int>( bench.last_chunk_ms ) ) +
            " last_inst=" + std::to_string( bench.last_chunk_instances ) +
            " budget_stops=" + std::to_string( bench.drain_budget_stops ) );
    }

    if( false == lines.empty() && lines.front().ms > 0.0 ) {
        WriteLog( "  >> bottleneck: " + lines.front().label );
    }
}

} // namespace

double
Eu7PackBench::accounted_ms() const {
    return (
        worker_read_pack_ms +
        main_read_pack_ms +
        main_cold_getmodel_ms +
        main_load_eu7_pack_ms +
        main_load_eu7_full_ms +
        main_region_insert_ms );
}

double
Eu7PackBench::chunk_accounted_ms() const {
    return (
        main_chunk_cold_ms +
        main_chunk_warm_tex_ms +
        main_chunk_pointer_apply_ms );
}

PackBenchTimer::PackBenchTimer( double Eu7PackBench::*const field )
    : m_field { field }
    , m_start { std::chrono::steady_clock::now() } {}

PackBenchTimer::~PackBenchTimer() {
    auto const end { std::chrono::steady_clock::now() };
    auto const delta {
        std::chrono::duration<double, std::milli>( end - m_start ).count() };
    g_pack_bench.*m_field += delta;
    if( g_stream_phase ) {
        g_pack_bench_stream.*m_field += delta;
    }
}

Eu7PackBench &
pack_bench() {
    return g_pack_bench;
}

Eu7PackBench &
pack_bench_stream() {
    return g_pack_bench_stream;
}

Eu7PackBench const &
pack_bench_bootstrap() {
    return g_pack_bench_bootstrap;
}

bool
pack_bench_stream_phase_active() {
    return g_stream_phase;
}

void
reset_pack_bench() {
    g_pack_bench = {};
    g_pack_bench_stream = {};
    g_pack_bench_bootstrap = {};
    g_stream_phase = false;
    g_logged_texture_fails.clear();
}

void
pack_bench_begin_stream_phase() {
    g_pack_bench_bootstrap = g_pack_bench;
    g_pack_bench_stream = {};
    g_stream_phase = true;
}

void
pack_bench_note_ready_queue( std::size_t const size ) {
    note_peak( &Eu7PackBench::peak_ready_queue, size );
}

void
pack_bench_note_in_flight( std::size_t const size ) {
    note_peak( &Eu7PackBench::peak_in_flight, size );
}

void
pack_bench_note_pending_cold_meshes( std::size_t const size ) {
    note_peak( &Eu7PackBench::peak_pending_cold_meshes, size );
}

void
pack_bench_note_pending_total( std::size_t const total ) {
    note_peak( &Eu7PackBench::peak_pending_total, total );
}

void
pack_bench_inc( std::uint64_t Eu7PackBench::*const field, std::uint64_t const amount ) {
    g_pack_bench.*field += amount;
    if( g_stream_phase ) {
        g_pack_bench_stream.*field += amount;
    }
}

void
pack_bench_note_chunk(
    double const wall_ms,
    std::size_t const instances,
    std::size_t const cold_meshes,
    double const cold_ms,
    double const warm_tex_ms,
    double const pointer_apply_ms ) {
    add_bench_ms( &Eu7PackBench::main_chunk_cold_ms, cold_ms );
    add_bench_ms( &Eu7PackBench::main_chunk_warm_tex_ms, warm_tex_ms );
    add_bench_ms( &Eu7PackBench::main_chunk_pointer_apply_ms, pointer_apply_ms );
    pack_bench_inc( &Eu7PackBench::main_chunks );
    pack_bench_inc( &Eu7PackBench::main_chunk_instances, instances );
    pack_bench_inc( &Eu7PackBench::main_chunk_cold_loads, cold_meshes );

    g_pack_bench.last_chunk_ms = wall_ms;
    g_pack_bench.last_apply_ms = pointer_apply_ms;
    g_pack_bench.last_chunk_instances = instances;
    g_pack_bench.last_chunk_cold = cold_meshes;
    if( wall_ms > g_pack_bench.peak_chunk_ms ) {
        g_pack_bench.peak_chunk_ms = wall_ms;
        g_pack_bench.peak_chunk_instances = instances;
        g_pack_bench.peak_chunk_cold = cold_meshes;
    }
    if( wall_ms >= 8.0 ) {
        pack_bench_inc( &Eu7PackBench::chunk_slow_8ms );
    }
    if( wall_ms >= 16.0 ) {
        pack_bench_inc( &Eu7PackBench::chunk_slow_16ms );
    }

    if( false == g_stream_phase ) {
        return;
    }

    auto &stream { g_pack_bench_stream };
    stream.last_chunk_ms = wall_ms;
    stream.last_apply_ms = pointer_apply_ms;
    stream.last_chunk_instances = instances;
    stream.last_chunk_cold = cold_meshes;
    if( wall_ms > stream.peak_chunk_ms ) {
        stream.peak_chunk_ms = wall_ms;
        stream.peak_chunk_instances = instances;
        stream.peak_chunk_cold = cold_meshes;
    }
}

void
log_pack_texture_fail( std::string const &texture_path ) {
    if( texture_path.empty() ) {
        return;
    }

    auto key { texture_path };
    replace_slashes( key );
    key = ToLower( key );
    if( false == g_logged_texture_fails.insert( key ).second ) {
        return;
    }

    WriteLog( "EU7 PACK tex_fail: \"" + texture_path + "\"" );
}

void
log_pack_bench() {
    if( g_pack_bench_bootstrap.worker_sections_done > 0
        || g_pack_bench_bootstrap.accounted_ms() > 0.0 ) {
        log_pack_bench_impl( g_pack_bench_bootstrap, "EU7 PACK bench [bootstrap]" );
        return;
    }
    log_pack_bench_impl( g_pack_bench, "EU7 PACK bench" );
}

void
log_pack_stream_bench() {
    log_pack_bench_impl( g_pack_bench_stream, "EU7 PACK bench [stream]" );
}

void
flush_pack_stream_bench() {
    if( false == g_stream_phase ) {
        return;
    }
    if(
        g_pack_bench_stream.sections_finalized == 0 &&
        g_pack_bench_stream.main_drain_calls == 0 &&
        g_pack_bench_stream.accounted_ms() <= 0.0 ) {
        return;
    }
    log_pack_stream_bench();
}

void
log_pack_stream_status(
    glm::dvec3 const &camera_position,
    double const camera_speed_mps,
    float const inner_ring_progress,
    float const outer_ring_progress,
    std::size_t const ready_queue,
    std::size_t const in_flight,
    std::size_t const pending_offset,
    std::size_t const pending_total,
    double const drain_budget_ms,
    std::size_t const chunk_limit ) {
    auto const &bench { g_pack_bench_stream };
    WriteLog(
        "EU7 PACK [live]: cam=" + std::to_string( camera_position.x ) + "," +
        std::to_string( camera_position.z ) + " m/s=" + std::to_string( camera_speed_mps ) +
        " ring4=" + std::to_string( static_cast<int>( inner_ring_progress * 100.f ) ) + "%" +
        " ring11=" + std::to_string( static_cast<int>( outer_ring_progress * 100.f ) ) + "%" +
        " ready=" + std::to_string( ready_queue ) +
        " in_flight=" + std::to_string( in_flight ) +
        " pending=" + std::to_string( pending_offset ) + "/" + std::to_string( pending_total ) +
        " slice=" + std::to_string( chunk_limit ) +
        " budget_ms=" + std::to_string( static_cast<int>( drain_budget_ms ) ) +
        " applied=" + std::to_string( bench.main_instances_applied ) +
        " tex_fail=" + std::to_string( bench.main_texture_assign_fail ) +
        " warm_miss=" + std::to_string( bench.main_texture_warm_miss ) +
        " chunks=" + std::to_string( bench.main_chunks ) +
        " last_chunk_ms=" + std::to_string( static_cast<int>( bench.last_chunk_ms ) ) +
        " peak_chunk_ms=" + std::to_string( static_cast<int>( bench.peak_chunk_ms ) ) +
        " slow8=" + std::to_string( bench.chunk_slow_8ms ) +
        " cold_ms=" + std::to_string( static_cast<int>( bench.main_chunk_cold_ms ) ) +
        " warm_ms=" + std::to_string( static_cast<int>( bench.main_chunk_warm_tex_ms ) ) +
        " apply_ms=" + std::to_string( static_cast<int>( bench.main_chunk_pointer_apply_ms ) ) +
        " reenq=" + std::to_string( bench.stream_reenqueue ) +
        " ahead=" + std::to_string( bench.stream_lookahead_enqueue ) +
        " urg_chunks=" + std::to_string( bench.stream_chunks_urgent ) +
        " cold_trunc=" + std::to_string( bench.stream_cold_slice_truncated ) +
        " mesh_q=" + std::to_string( pack_mesh_loader_queue_depth() ) +
        " mesh_r=" + std::to_string( pack_mesh_loader_ready_count() ) +
        " mesh_d=" + std::to_string( bench.stream_mesh_async_drained ) +
        " preempt=" + std::to_string( bench.stream_preempt_pending ) );
}

} // namespace scene::eu7
