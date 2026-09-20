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

// Which level of the quadtree a tile is drawn at follows from what the coarser mesh would cost in
// pixels. Every tile says how far its ground departs from the finest mesh, in metres; that error,
// at the tile's distance, covers a number of pixels, and while it covers fewer than the detail
// setting asks for there is nothing to gain by going finer. Pixelsperunit is the viewport height
// over twice the tangent of half the vertical field of view - what a metre at a metre's distance
// spans - so zooming in or a taller window pushes detail further out on its own.
//
// This is the format's own rule, and the reason a tile carries its error at all.
inline constexpr std::size_t terrain_maxlevels { 16 };

inline bool
terrain_level_enough(
    double const Error, double const Distance, double const Pixelsperunit, double const Detail ) {
    return Error * Pixelsperunit <= std::max( 0.1, Detail ) * std::max( 1.0, Distance );
}

// how far the camera may move before which tiles are wanted is worked out again, as a share of the
// finest tile's side
inline constexpr double terrain_rescan_tiles { 0.25 };

// what one terrain's streamer is doing, kept by it and read by the debug panel
struct terrain_statistics {
    std::size_t resident { 0 };     // tiles held in the vertex and index buffers
    std::size_t chosen { 0 };       // tiles the last walk of the quadtree settled on drawing
    std::size_t drawn { 0 };        // of those, the ones inside the frustum in the last draw
    std::size_t triangles { 0 };    // triangles in the last draw
    std::size_t wanted { 0 };       // tiles the walk asked for and did not have
    std::size_t uploaded { 0 };     // tiles put in the buffers since opening
    std::size_t dropped { 0 };      // arrivals discarded as no longer wanted
    std::size_t gpubytes { 0 };     // what the two buffers hold
    std::array<std::size_t, terrain_maxlevels> perlevel {}; // tiles drawn at each level
};

// one terrain as the debug panel shows it. the renderer writes it once a frame from the main view,
// the panel reads it; neither needs to know the other's types
struct terrain_field_status {
    std::string path;
    float tilesize { 0.f };         // side of a tile at the finest level, in metres
    float range { 0.f };
    float detail { 0.f };           // pixels an error may cover before a finer level is used
    std::uint32_t levels { 0 };
    std::size_t backlog { 0 };      // tiles the loader has queued or finished and not handed over
    std::array<float, terrain_maxlevels> leveltile {};   // tile side at each level
    std::array<float, terrain_maxlevels> levelerror {};  // metres a level departs from the finest
    terrain_statistics stats;
};

inline std::vector<terrain_field_status> TerrainStatus;

}
