/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

export module eu07.rendering.terrainstatus;

export {

// Which level a tile is drawn at follows from what its samples look like on screen: a level is
// used out to the distance where one of its sample spacings covers the detail setting in
// pixels. Each level doubles the spacing, so each doubles that distance too. Pixelsperunit is
// the viewport height over twice the tangent of half the vertical field of view - what a
// metre at a metre's distance spans - so zooming in or a taller window pushes detail further.
//
// The finest range never drops below four tile sides, whatever the screen: the morphing below
// only closes the seams when a level's range is several tiles long.
inline constexpr double terrain_finest_tiles { 4.0 };

inline double
terrain_finest_range( double const Gridstep, double const Tilesize, double const Pixelsperunit, double const Detail ) {
    return std::max( terrain_finest_tiles * Tilesize, Gridstep * Pixelsperunit / std::max( 0.1, Detail ) );
}

// how far the camera may move before the tiles in range and their levels are worked out
// again, in tile sides
inline constexpr double terrain_rescan_tiles { 0.25 };

// the distance up to which a level is used, given the finest level's range
inline double
terrain_level_distance( std::uint32_t const Level, double const Finest ) {
    return Finest * std::pow( 2.0, Level );
}

// A tile morphs into the next level over a band at the far end of its level's range. Two
// neighbours at levels L and L+1 then meet exactly when, all along their shared edge, the
// finer tile has finished morphing and the coarser one has not started. The band ends a
// rescan's worth of travel short of the range, because levels are chosen where the camera
// was at the last rescan while the morph follows the camera every frame; and it is a fifth
// of the range wide, which keeps the coarser tile's band, a whole range further out, clear of
// any edge a finer tile can reach. Both were settled by a geometric test over random camera
// positions, three tile sizes and several finest ranges (tools/terraincook/seamcheck.py),
// which finds no gap with these values and finds them at every level boundary with a band
// ending at the range itself.
inline constexpr double terrain_morph_band { 0.2 };

// the distances, from the camera, over which a level L tile morphs into level L+1
inline double
terrain_morph_end( std::uint32_t const Level, double const Finest, double const Tilesize ) {
    return terrain_level_distance( Level, Finest ) - terrain_rescan_tiles * Tilesize;
}

inline double
terrain_morph_begin( std::uint32_t const Level, double const Finest, double const Tilesize ) {
    return terrain_morph_end( Level, Finest, Tilesize ) - terrain_morph_band * terrain_level_distance( Level, Finest );
}

// what one terrain field's streamer is doing, kept by the field and read by the debug panel
struct terrain_statistics {
    std::size_t resident { 0 };     // tiles holding gl textures
    std::size_t inview { 0 };       // resident tiles within range in the last update
    std::size_t drawn { 0 };        // of those, the ones inside the frustum in the last draw
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
    float finest { 0.f };           // range of the finest level, from the screen
    std::uint32_t levels { 0 };
    std::size_t backlog { 0 };      // tiles the loader has queued or finished and not handed over
    terrain_statistics stats;
};

inline std::vector<terrain_field_status> TerrainStatus;

}
