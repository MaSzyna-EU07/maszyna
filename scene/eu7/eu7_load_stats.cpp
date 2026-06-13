/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/eu7/eu7_load_stats.h"

#include "utilities/Logs.h"

#include <iomanip>
#include <sstream>

namespace scene::eu7 {
namespace {

Eu7LoadStats g_load_stats;

[[nodiscard]] std::string
format_seconds( double const ms ) {
    std::ostringstream out;
    out << std::fixed << std::setprecision( 1 ) << ( ms / 1000.0 ) << 's';
    return out.str();
}

} // namespace

double
Eu7LoadStats::total_ms() const {
    return (
        read_ms +
        scm_fallback_ms +
        place_fast_ms +
        place_full_ms +
        terr_ms +
        mesh_ms +
        line_ms +
        trak_ms +
        trac_ms +
        power_ms +
        model_ms +
        memcell_ms +
        launcher_ms +
        dynamic_ms +
        sound_ms +
        event_ms +
        trainset_ms +
        first_init_ms );
}

ScopedTimer::ScopedTimer( double &target )
    : m_target { target }
    , m_start { std::chrono::steady_clock::now() } {}

ScopedTimer::~ScopedTimer() {
    auto const end { std::chrono::steady_clock::now() };
    m_target += std::chrono::duration<double, std::milli>( end - m_start ).count();
}

Eu7LoadStats &
load_stats() {
    return g_load_stats;
}

void
reset_load_stats() {
    g_load_stats = {};
}

void
log_load_stats() {
    auto const &s { g_load_stats };
    auto const total { s.total_ms() };

    WriteLog( "EU7 load stats (accounted " + format_seconds( total ) + "):" );
    WriteLog(
        "  read/cache: " + format_seconds( s.read_ms ) +
        " reads=" + std::to_string( s.module_read ) +
        " cache_hit=" + std::to_string( s.module_cache_hit ) );
    WriteLog(
        "  scm fallback: " + format_seconds( s.scm_fallback_ms ) +
        " count=" + std::to_string( s.scm_fallback ) );
    WriteLog(
        "  place fast (MODL-only): " + format_seconds( s.place_fast_ms ) +
        " paths=" + std::to_string( s.model_fast_path ) );
    WriteLog(
        "  place full: " + format_seconds( s.place_full_ms ) +
        " paths=" + std::to_string( s.module_full_path ) );
    WriteLog(
        "  terr: " + format_seconds( s.terr_ms ) );
    WriteLog(
        "  mesh: " + format_seconds( s.mesh_ms ) );
    WriteLog(
        "  line: " + format_seconds( s.line_ms ) );
    WriteLog(
        "  trak: " + format_seconds( s.trak_ms ) +
        " count=" + std::to_string( s.tracks ) );
    WriteLog(
        "  trac: " + format_seconds( s.trac_ms ) +
        " count=" + std::to_string( s.traction ) );
    WriteLog(
        "  power: " + format_seconds( s.power_ms ) +
        " count=" + std::to_string( s.power_sources ) );
    WriteLog(
        "  model insert: " + format_seconds( s.model_ms ) +
        " count=" + std::to_string( s.models ) );
    WriteLog(
        "  memcell: " + format_seconds( s.memcell_ms ) +
        " count=" + std::to_string( s.memcells ) );
    WriteLog(
        "  launcher: " + format_seconds( s.launcher_ms ) +
        " count=" + std::to_string( s.launchers ) );
    WriteLog(
        "  dynamic: " + format_seconds( s.dynamic_ms ) +
        " count=" + std::to_string( s.dynamics ) );
    WriteLog(
        "  sound: " + format_seconds( s.sound_ms ) +
        " count=" + std::to_string( s.sounds ) );
    WriteLog(
        "  event: " + format_seconds( s.event_ms ) +
        " count=" + std::to_string( s.events ) );
    WriteLog(
        "  trainset: " + format_seconds( s.trainset_ms ) +
        " count=" + std::to_string( s.trainsets ) );
    WriteLog(
        "  first_init: " + format_seconds( s.first_init_ms ) );
    if(
        s.pack_sections_loaded > 0 || s.pack_skipped_includes > 0 ||
        s.pack_skipped_models > 0 ) {
        WriteLog(
            "  pack: sections=" + std::to_string( s.pack_sections_loaded ) +
            " models=" + std::to_string( s.pack_models ) +
            " skipped_incl=" + std::to_string( s.pack_skipped_includes ) +
            " skipped_modl=" + std::to_string( s.pack_skipped_models ) );
    }
    WriteLog(
        "  visits=" + std::to_string( s.module_visits ) +
        " applied=" + std::to_string( s.module_applied ) +
        " deduped=" + std::to_string( s.module_deduped ) );
    WriteLog(
        "  (roznica vs Scenario loading time = deserialize_continue, map geometry, itd.)" );
}

} // namespace scene::eu7
