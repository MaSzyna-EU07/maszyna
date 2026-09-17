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

// distance out to which a terrain tile is drawn at its finest level, in tile sides; each
// further level doubles it. four sides is 512 m for the usual 128 m tile. the ranges are
// measured in tiles rather than metres because the morphing below only closes the seams
// when a level's range is several tiles long, whatever the tile size
inline constexpr double terrain_finest_tiles { 4.0 };

// how far the camera may move before the tiles in range and their levels are worked out
// again, in tile sides
inline constexpr double terrain_rescan_tiles { 0.25 };

// the distance up to which a level is used, for tiles of the given side
inline double
terrain_level_distance( std::uint32_t const Level, double const Tilesize ) {
    return terrain_finest_tiles * Tilesize * std::pow( 2.0, Level );
}

// A tile morphs into the next level over a band at the far end of its level's range. Two
// neighbours at levels L and L+1 then meet exactly when, all along their shared edge, the
// finer tile has finished morphing and the coarser one has not started. The band ends a
// rescan's worth of travel short of the range, because levels are chosen where the camera
// was at the last rescan while the morph follows the camera every frame; and it is a fifth
// of the range wide, which keeps the coarser tile's band, a whole range further out, clear of
// any edge a finer tile can reach. Both were settled by a geometric test over random camera
// positions and three tile sizes, which finds no gap with these values and finds them at
// every level boundary with a band ending at the range itself.
inline constexpr double terrain_morph_band { 0.2 };

// the distances, from the camera, over which a level L tile morphs into level L+1
inline double
terrain_morph_end( std::uint32_t const Level, double const Tilesize ) {
    return terrain_level_distance( Level, Tilesize ) - terrain_rescan_tiles * Tilesize;
}

inline double
terrain_morph_begin( std::uint32_t const Level, double const Tilesize ) {
    return terrain_morph_end( Level, Tilesize ) - terrain_morph_band * terrain_level_distance( Level, Tilesize );
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
