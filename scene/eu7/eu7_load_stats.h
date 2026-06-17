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

namespace scene::eu7 {

struct Eu7LoadStats {
    double read_ms { 0.0 };
    double scm_fallback_ms { 0.0 };
    double place_fast_ms { 0.0 };
    double place_full_ms { 0.0 };
    double terr_ms { 0.0 };
    double mesh_ms { 0.0 };
    double line_ms { 0.0 };
    double trak_ms { 0.0 };
    double trac_ms { 0.0 };
    double power_ms { 0.0 };
    double model_ms { 0.0 };
    double memcell_ms { 0.0 };
    double launcher_ms { 0.0 };
    double dynamic_ms { 0.0 };
    double sound_ms { 0.0 };
    double event_ms { 0.0 };
    double trainset_ms { 0.0 };
    double first_init_ms { 0.0 };

    std::uint64_t module_visits { 0 };
    std::uint64_t module_applied { 0 };
    std::uint64_t module_deduped { 0 };
    std::uint64_t module_read { 0 };
    std::uint64_t module_cache_hit { 0 };
    std::uint64_t model_fast_path { 0 };
    std::uint64_t module_full_path { 0 };
    std::uint64_t scm_fallback { 0 };
    std::uint64_t models { 0 };
    std::uint64_t tracks { 0 };
    std::uint64_t traction { 0 };
    std::uint64_t power_sources { 0 };
    std::uint64_t memcells { 0 };
    std::uint64_t launchers { 0 };
    std::uint64_t dynamics { 0 };
    std::uint64_t sounds { 0 };
    std::uint64_t events { 0 };
    std::uint64_t trainsets { 0 };
    std::uint64_t module_v2_loaded { 0 };
    std::uint64_t module_v2_baked { 0 };
    std::uint64_t pack_skipped_includes { 0 };
    std::uint64_t pack_sections_loaded { 0 };
    std::uint64_t pack_models { 0 };
    std::uint64_t pack_skipped_models { 0 };

    [[nodiscard]] double total_ms() const;
};

class ScopedTimer {
public:
    explicit ScopedTimer( double &Target );
    ~ScopedTimer();

    ScopedTimer( ScopedTimer const & ) = delete;
    ScopedTimer &operator=( ScopedTimer const & ) = delete;

private:
    double &m_target;
    std::chrono::steady_clock::time_point m_start;
};

[[nodiscard]] Eu7LoadStats &
load_stats();

void
reset_load_stats();

void
log_load_stats();

} // namespace scene::eu7
