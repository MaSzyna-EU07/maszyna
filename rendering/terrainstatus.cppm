/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

export module eu07.rendering.terrainstatus;

export {

// distance out to which a terrain tile is drawn at its finest level; each further level
// doubles it
inline constexpr double terrain_finest_range { 512.0 };

// the distance up to which a level is used
inline double
terrain_level_distance( std::uint32_t const Level ) {
    return terrain_finest_range * std::pow( 2.0, Level );
}

// what one terrain field's streamer is doing, kept by the field and read by the debug panel
struct terrain_statistics {
    std::size_t resident { 0 };     // tiles holding gl textures
    std::size_t inview { 0 };       // resident tiles within range in the last update
    std::size_t wanted { 0 };       // tiles in range missing, or at the wrong level
    std::size_t uploaded { 0 };     // uploads since opening
    std::size_t dropped { 0 };      // arrivals discarded as stale since opening
    std::size_t texturebytes { 0 };
    std::array<std::size_t, 8> perlevel {}; // tiles in view at each level
};

// one field as the debug panel shows it. the renderer writes it once a frame from the main
// view, the panel reads it; neither needs to know the other's types
struct terrain_field_status {
    std::string path;
    float gridstep { 0.f };
    float tilesize { 0.f };
    float range { 0.f };
    std::uint32_t levels { 0 };
    std::size_t backlog { 0 };      // tiles the loader has queued or finished and not handed over
    terrain_statistics stats;
};

inline std::vector<terrain_field_status> TerrainStatus;

}
