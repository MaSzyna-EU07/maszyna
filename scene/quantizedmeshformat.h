/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// Cesium Quantized Mesh, as the terrain tiles are written.
// https://github.com/CesiumGS/quantized-mesh
//
// The ground in a MaSzyna scenery is already an irregular mesh: fine along the track where it
// came from a survey, coarse in the open where it was drawn by hand. The cook keeps that mesh
// rather than resampling it - it cuts it into tiles, welds the vertices and builds coarser
// levels of it - so there is nothing to interpolate, nothing to fill in where the ground has
// holes, and no grid step to pick.
//
// Tiles form a quadtree, as the format intends: one tile at the root covers the whole terrain,
// and each level splits every tile into four. A distant tile is therefore a large one, and the
// horizon costs tens of tiles rather than thousands. Every vertex on a tile's border survives
// into all of its coarser levels, so two neighbours drawn at different levels share their
// border exactly, and nothing has to be stitched or morphed at draw time.
//
// Where this departs from the specification, and why:
//
//  - the header's fields are geodetic (ECEF centre, bounding sphere, horizon occlusion point).
//    A scenery has a local metric frame and no geographic reference to convert from, so those
//    fields carry scenery metres. The geometry reads back correctly anywhere; a Cesium client
//    would place the tile wrongly on the globe, which is a trade for something we do not want.
//  - the format has no texture coordinates and no materials: its u and v are the vertex's
//    horizontal position within the tile. A vertex of ours carries which ground material it
//    belongs to, in extension ext_ground below, which is what the extension mechanism is for.
//    Texture coordinates are not stored at all: ground textures tile in world space, so the
//    coordinate follows from where the vertex is and how many metres one repeat of its material
//    covers, which is measured from the scenery once and kept in the terrain's table.
//  - tiles are stored inside the scenery's archive, one zstd-compressed entry each, in place of
//    the gzip the format expects from an HTTP layer. The bytes of a tile are the format's.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace quantizedmesh {

// u, v and height are quantized to this, not to 65535: the format says so
inline constexpr std::uint16_t quantum_max { 32767 };

// what the cook counted as ground, and how it cut and simplified it. tiles cooked under other
// rules are baked again rather than read
inline constexpr std::uint32_t cook_rules { 3 };

// What one tile's ground asks for, measured when it was cooked: how far it departs from the finest
// mesh, and how long its triangles are. Which level to draw follows from these two, and they belong
// to the tile rather than to its level. The level's figure is the worst tile in it, so a single
// steep corner made the whole pyramid ask to be drawn from as far away as a mountain - and flat
// ground twenty kilometres out was split as finely as a valley wall underfoot.
struct tile_measure {
    float error { 0.f };
    float edge { 0.f };
};

#pragma pack( push, 1 )

// the format's header, unchanged. 88 bytes
struct file_header {
    double centre_x, centre_y, centre_z;
    float lowest, highest;                  // the tile's height range, in metres
    double sphere_x, sphere_y, sphere_z;     // bounding sphere, for culling
    double sphere_radius;
    double horizon_x, horizon_y, horizon_z;  // horizon occlusion point; unused here
};

static_assert( sizeof( file_header ) == 88 );

#pragma pack( pop )

// extension ids. 1, 2 and 4 are taken by the format for normals, the water mask and metadata
enum extension_id : std::uint8_t {
    // the format's own: one normal per vertex, octahedron-encoded into two bytes
    ext_normals = 1,
    ext_watermask = 2,
    ext_metadata = 4,
    // ours: one uint16 per vertex, its material's place in the terrain's table
    ext_ground = 64,
};

// zigzag, as the format uses it on the deltas between consecutive vertices
inline std::uint16_t
zigzag_encode( std::int32_t const Value ) {
    return static_cast<std::uint16_t>( ( Value << 1 ) ^ ( Value >> 31 ) );
}

inline std::int32_t
zigzag_decode( std::uint16_t const Stored ) {
    return ( Stored >> 1 ) ^ -static_cast<std::int32_t>( Stored & 1 );
}

// A unit vector into two bytes, as the format's normals extension has it: the sphere folded onto an
// octahedron, unfolded into a square, and the square's coordinates stored as unsigned bytes. Two bytes
// hold a normal to well under a degree, which is what the lighting needs and no more.
//
// Normals are stored rather than worked out while drawing. A normal taken from how the position
// changes across a triangle is that triangle's own, so the ground reads as facets, and - worse - a
// coarser level's facets face differently from the ones they stand for, so the light over a hillside
// changes as the level does. A normal carried on the vertex comes from the ground as the scenery drew
// it and stays the same at every level.
inline void
encode_normal( double const X, double const Y, double const Z, std::uint8_t &Out0, std::uint8_t &Out1 ) {

    auto const length { std::abs( X ) + std::abs( Y ) + std::abs( Z ) };
    if( length < 1e-12 ) { Out0 = Out1 = 128; return; }
    auto first { X / length };
    auto second { Z / length };
    if( Y < 0.0 ) {
        auto const was { first };
        first = ( 1.0 - std::abs( second ) ) * ( was >= 0.0 ? 1.0 : -1.0 );
        second = ( 1.0 - std::abs( was ) ) * ( second >= 0.0 ? 1.0 : -1.0 );
    }
    auto const tobyte = []( double const Value ) {
        return static_cast<std::uint8_t>(
            std::clamp( std::round( ( Value + 1.0 ) * 0.5 * 255.0 ), 0.0, 255.0 ) ); };
    Out0 = tobyte( first );
    Out1 = tobyte( second );
}

// a quantized coordinate back to metres, against the range it was quantized in
inline double
dequantize( std::uint16_t const Stored, double const Low, double const Span ) {
    return Low + ( static_cast<double>( Stored ) / quantum_max ) * Span;
}

inline std::uint16_t
quantize( double const Value, double const Low, double const Span ) {
    if( Span <= 0.0 ) { return 0; }
    auto const share { std::clamp( ( Value - Low ) / Span, 0.0, 1.0 ) };
    return static_cast<std::uint16_t>( share * quantum_max + 0.5 );
}

// Indices are stored high-water-mark encoded: a code of zero means the next index never used
// before, and anything else counts back from the highest index used so far. The run of codes is
// small numbers whatever the mesh looks like, which is what makes the tile compress.
template <typename Index_>
void
highwater_encode( std::vector<Index_> const &Indices, std::vector<Index_> &Out ) {

    Out.clear();
    Out.reserve( Indices.size() );
    std::int64_t highest { 0 };
    for( auto const index : Indices ) {
        Out.push_back( static_cast<Index_>( highest - static_cast<std::int64_t>( index ) ) );
        if( static_cast<std::int64_t>( index ) == highest ) { ++highest; }
    }
}

// returns false when a code points past what has been used, which a corrupted tile can do
template <typename Index_>
bool
highwater_decode( std::vector<Index_> const &Codes, std::vector<std::uint32_t> &Out ) {

    Out.clear();
    Out.reserve( Codes.size() );
    std::int64_t highest { 0 };
    for( auto const code : Codes ) {
        auto const index { highest - static_cast<std::int64_t>( code ) };
        if( ( index < 0 ) || ( index > highest ) ) { return false; }
        Out.push_back( static_cast<std::uint32_t>( index ) );
        if( index == highest ) { ++highest; }
    }
    return true;
}

// A tile's place in the quadtree. Level 0 is the root, covering the whole terrain; each level
// splits every tile into four. X grows east, Z grows with the scenery's z, so a tile's square
// follows from the terrain's extent and the level alone.
struct tile_address {
    std::uint32_t level { 0 };
    std::int32_t x { 0 }, z { 0 };

    bool operator==( tile_address const & ) const = default;
};

constexpr std::int64_t
tile_key( tile_address const &Tile ) {
    // a level fits in six bits for any terrain worth cooking, and the coordinates cannot exceed
    // the level, so this is unique
    return ( static_cast<std::int64_t>( Tile.level ) << 58 )
         ^ ( static_cast<std::int64_t>( Tile.x ) << 29 )
         ^ static_cast<std::int64_t>( Tile.z );
}

// where a tile's file sits in the archive, following the format's own layout of level, column
// and row
inline std::string
tile_name( tile_address const &Tile ) {
    return std::to_string( Tile.level ) + "/" + std::to_string( Tile.x ) + "/"
         + std::to_string( Tile.z ) + ".terrain";
}

// the terrain's extent, height range, depth and material table are the same for every tile, so
// they are written once beside them, under this name. the format keeps the same thing in
// layer.json
inline constexpr char const *tablename { "terrain.table" };

// what that file says
struct terrain_table {
    std::uint32_t rules { 0 };
    // the square the root tile covers. always square, so that a tile at any level is too
    double west { 0.0 }, north { 0.0 }, side { 0.0 };
    double lowest { 0.0 }, highest { 0.0 };
    // levels cooked, root included: addresses run from 0 to levels - 1
    std::uint32_t levels { 1 };
    // metres the ground of a tile at each level departs from the finest one, and how long a triangle
    // of it typically is. Between them they decide which level to draw. The error alone cannot: it is
    // the worst departure anywhere in the tile and is inherited by every level above, so one steep
    // corner sets it for the whole pyramid and the levels become indistinguishable. What actually
    // tells a level of sixty triangles from one of fifteen thousand is how large the triangles are.
    std::vector<float> errors;
    std::vector<float> edges;
    // one ground texture: its name, and how many metres of ground one repeat of it covers, as
    // measured from the texture coordinates the scenery drew it with
    struct material {
        std::string name;
        float repeat { 4.f };
    };
    // in the order the vertices' material indices count
    std::vector<material> materials;

    // metres along the side of a tile at this level
    double tilesize( std::uint32_t const Level ) const {
        return side / static_cast<double>( 1u << Level );
    }
    float error( std::uint32_t const Level ) const {
        return Level < errors.size() ? errors[ Level ] : 0.f;
    }
    float edge( std::uint32_t const Level ) const {
        return Level < edges.size() ? edges[ Level ] : 0.f;
    }
};

} // namespace quantizedmesh
