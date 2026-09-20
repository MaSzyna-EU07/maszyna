/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "scene/quantizedmeshcooker.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <map>
#include <queue>

namespace quantizedmesh {

namespace {

// a triangle with less surface than this is nothing at all, whichever way it faces, and is the
// only thing the cook refuses to lay. Walls - the sides of an embankment, a cutting, an abutment -
// are ground as much as anything else: nothing about a mesh says it has to be a height field, and
// leaving them out is what opens holes along every bank
constexpr double nullarea { 1e-6 };
// vertices nearer than this to each other, to a tile border or to a plane are the same thing.
// a tenth of a millimetre is below anything the game can show and far above what the arithmetic
// of cutting a triangle can be out by
constexpr double weldepsilon { 1e-4 };
// the deepest quadtree worth cooking: 4096 tiles along a side of the finest level
constexpr std::uint32_t maxlevels { 13 };
// The finest tile is this many typical triangle edges across, and never outside these bounds. A tile
// has to be large against the triangles in it: a triangle crossing a border is cut, and the pieces
// cost vertices neither side had. It also has to stay small enough to be worth loading and culling on
// its own, which is what the bounds are for.
//
// The typical edge is averaged over the ground weighted by how much of it each triangle covers, not
// over the triangles counted one each. A scenery holds both kinds of ground - a dense patch from a
// survey by the track, and big triangles drawn by hand for the rest - and counting triangles one each
// lets a patch of a few hundred metres decide the tile for forty kilometres. What the cut costs
// depends on how much ground the large triangles cover, which is what this measures.
constexpr double tileedges { 16.0 };
constexpr double smallesttileside { 128.0 };
constexpr double largesttileside { 1024.0 };
// a tile is never collapsed below this, however far away it is drawn: past a point the triangles
// saved are not worth the ground losing its shape
constexpr std::size_t smallesttile { 64 };
// A level may move the ground by this share of its tile's side, and no further.
//
// The number decides how far away a level has to be before it may be used, and so how long the level
// below it has to be drawn. Too generous and the finest mesh - which is the whole of the scenery's
// ground - is drawn out to several kilometres, and only three or four levels are ever in view at once.
// A third of what it was buys coarse levels faithful enough to be used three times closer, which is
// both fewer triangles in the end and more of the ladder visible at any moment.
constexpr double errorshare { 384.0 };
// How far the finest level may move the ground, in metres. Not a share of the tile like the levels
// above: this is the one that is stood on, so what it may lose is set by what the eye can catch from
// there rather than by how large the tile happens to be. Three centimetres of ground twenty metres
// ahead covers about a pixel and a half.
//
// What it takes off is redundancy rather than detail - a field drawn as a grid a metre across is a
// grid a metre across even where it is flat, and survey data is mostly flat. Nothing else in the
// pyramid can remove it, because every level above is built from this one.
constexpr double finesterror { 0.03 };
// How wide the ground may change over where two materials meet, as a share of the tile's side.
//
// The fragment stage blends the three materials of a triangle by how near the fragment is to each
// corner, so a transition is one triangle wide - and a triangle is exactly what a coarser level makes
// larger. Left alone the ground would fade over four metres near to and over sixty far away, and the
// change would be seen as the level changed.
//
// Tying it to the tile rather than to a fixed number of metres holds it steady where it counts, which
// is on screen: a level is drawn from about as far away as its tiles are wide, so a share of the tile
// is a roughly constant number of pixels. It also lets material detail finer than this merge away at
// the levels where it could not be seen anyway - without that, ground drawn in stripes a few metres
// across can never be simplified at all, because every edge in it crosses a boundary.
constexpr double blendshare { 64.0 };

// the area of a triangle in space, whichever way it faces
double
surface_area( cooker::corner const &A, cooker::corner const &B, cooker::corner const &C ) {
    auto const nx { ( B.y - A.y ) * ( C.z - A.z ) - ( B.z - A.z ) * ( C.y - A.y ) };
    auto const ny { ( B.z - A.z ) * ( C.x - A.x ) - ( B.x - A.x ) * ( C.z - A.z ) };
    auto const nz { ( B.x - A.x ) * ( C.y - A.y ) - ( B.y - A.y ) * ( C.x - A.x ) };
    return 0.5 * std::sqrt( nx * nx + ny * ny + nz * nz );
}

// a triangle standing upright has no extent in the plan at all. it still belongs to the ground,
// but it has to be told apart: the tile it goes to cannot be decided by cutting it in two
double
planar_extent( cooker::corner const &A, cooker::corner const &B, cooker::corner const &C ) {
    return std::max(
        std::max( { A.x, B.x, C.x } ) - std::min( { A.x, B.x, C.x } ),
        std::max( { A.z, B.z, C.z } ) - std::min( { A.z, B.z, C.z } ) );
}

// The error of collapsing a vertex away, as Garland and Heckbert measure it: the sum of the
// squared distances to the planes of the triangles that met there. Kept as the ten distinct
// entries of the symmetric matrix.
struct quadric {
    double m[ 10 ] {};

    void add_plane( double const A, double const B, double const C, double const D, double const Weight ) {
        m[ 0 ] += Weight * A * A; m[ 1 ] += Weight * A * B; m[ 2 ] += Weight * A * C; m[ 3 ] += Weight * A * D;
        m[ 4 ] += Weight * B * B; m[ 5 ] += Weight * B * C; m[ 6 ] += Weight * B * D;
        m[ 7 ] += Weight * C * C; m[ 8 ] += Weight * C * D;
        m[ 9 ] += Weight * D * D;
    }

    void operator+=( quadric const &Other ) {
        for( std::size_t index = 0; index < 10; ++index ) { m[ index ] += Other.m[ index ]; }
    }

    double at( double const X, double const Y, double const Z ) const {
        return m[ 0 ] * X * X + 2.0 * m[ 1 ] * X * Y + 2.0 * m[ 2 ] * X * Z + 2.0 * m[ 3 ] * X
             + m[ 4 ] * Y * Y + 2.0 * m[ 5 ] * Y * Z + 2.0 * m[ 6 ] * Y
             + m[ 7 ] * Z * Z + 2.0 * m[ 8 ] * Z
             + m[ 9 ];
    }
};

// which sides of a tile's square a point lies on, one bit each
std::uint8_t
sides_of( double const X, double const Z, double const West, double const North, double const Side ) {
    std::uint8_t sides { 0 };
    if( std::abs( X - West ) < weldepsilon )            { sides |= 1; }
    if( std::abs( X - ( West + Side ) ) < weldepsilon ) { sides |= 2; }
    if( std::abs( Z - North ) < weldepsilon )           { sides |= 4; }
    if( std::abs( Z - ( North + Side ) ) < weldepsilon ){ sides |= 8; }
    return sides;
}

// a vertex of a polygon being cut along the tile grid
struct clipvertex {
    double x, y, z;
    double nx, ny, nz;
};

clipvertex
between( clipvertex const &From, clipvertex const &To, double const Share ) {
    return {
        From.x + ( To.x - From.x ) * Share,
        From.y + ( To.y - From.y ) * Share,
        From.z + ( To.z - From.z ) * Share,
        From.nx + ( To.nx - From.nx ) * Share,
        From.ny + ( To.ny - From.ny ) * Share,
        From.nz + ( To.nz - From.nz ) * Share };
}

// Sutherland and Hodgman against one side of the tile's square. Axis 0 is x and 1 is z; the
// coordinate cut against is set to the bound itself rather than to what the division gives, so
// that the two tiles sharing that border end up with a vertex in exactly the same place.
void
clip_side( std::vector<clipvertex> &Polygon, int const Axis, double const Bound, bool const Keepabove ) {

    if( Polygon.empty() ) { return; }

    auto const coordinate = [ Axis ]( clipvertex const &Vertex ) {
        return ( Axis == 0 ) ? Vertex.x : Vertex.z; };
    auto const inside = [ & ]( clipvertex const &Vertex ) {
        return Keepabove ? ( coordinate( Vertex ) >= Bound - weldepsilon )
                         : ( coordinate( Vertex ) <= Bound + weldepsilon ); };

    std::vector<clipvertex> kept;
    kept.reserve( Polygon.size() + 2 );
    for( std::size_t index = 0; index < Polygon.size(); ++index ) {
        auto const &here { Polygon[ index ] };
        auto const &next { Polygon[ ( index + 1 ) % Polygon.size() ] };
        auto const hereinside { inside( here ) };
        if( true == hereinside ) { kept.push_back( here ); }
        if( hereinside == inside( next ) ) { continue; }
        auto const span { coordinate( next ) - coordinate( here ) };
        if( std::abs( span ) < weldepsilon ) { continue; }
        auto crossing { between( here, next, ( Bound - coordinate( here ) ) / span ) };
        if( Axis == 0 ) { crossing.x = Bound; } else { crossing.z = Bound; }
        kept.push_back( crossing );
    }
    Polygon.swap( kept );
}

} // anonymous namespace

cooker::~cooker() {

    if( true == m_scratch.is_open() ) { m_scratch.close(); }
    if( false == m_scratchpath.empty() ) {
        std::error_code error;
        std::filesystem::remove( m_scratchpath, error );
    }
}

bool
cooker::begin( std::string const &Scratchpath, std::size_t const Tilebudget ) {

    m_scratchpath = Scratchpath;
    m_tilebudget = std::max<std::size_t>( 256, Tilebudget );
    m_scratch.open( m_scratchpath, std::ios::binary | std::ios::in | std::ios::out | std::ios::trunc );
    return m_scratch.good();
}

std::uint16_t
cooker::material_index( std::string_view const Material ) {

    auto const found { m_materiallookup.find( std::string { Material } ) };
    if( found != m_materiallookup.end() ) { return found->second; }
    auto const index { static_cast<std::uint16_t>( m_materials.size() ) };
    m_materials.emplace_back( Material );
    m_materiallookup.emplace( std::string { Material }, index );
    return index;
}

// The scenery's texture coordinates are not kept: ground textures tile in world space, so a
// coordinate follows from where a vertex is. What has to be kept is the scale the scenery drew the
// material at, and that is measured here - the ratio of world distance to texture distance along
// a triangle's edges. Edges too short, or with no texture movement along them, say nothing.
void
cooker::measure_repeat( std::uint16_t const Material, corner const ( &Corners )[ 3 ] ) {

    if( Material >= m_repeatcounts.size() ) {
        m_repeatsums.resize( Material + 1, 0.0 );
        m_repeatsquares.resize( Material + 1, 0.0 );
        m_repeatcounts.resize( Material + 1, 0 );
    }
    for( std::size_t index = 0; index < 3; ++index ) {
        auto const &from { Corners[ index ] };
        auto const &to { Corners[ ( index + 1 ) % 3 ] };
        auto const world {
            std::sqrt( ( to.x - from.x ) * ( to.x - from.x ) + ( to.z - from.z ) * ( to.z - from.z ) ) };
        auto const texture {
            std::sqrt( ( to.u - from.u ) * ( to.u - from.u ) + ( to.v - from.v ) * ( to.v - from.v ) ) };
        if( ( world < 0.5 ) || ( texture < 1e-4 ) ) { continue; }
        auto const ratio { world / texture };
        m_repeatsums[ Material ] += ratio;
        m_repeatsquares[ Material ] += ratio * ratio;
        ++m_repeatcounts[ Material ];
    }
}

void
cooker::add( corner const &A, corner const &B, corner const &C, std::string_view const Material ) {

    auto const id { static_cast<std::uint32_t>( m_added ) };
    ++m_added;

    if( ( surface_area( A, B, C ) < nullarea ) || ( planar_extent( A, B, C ) < weldepsilon ) ) {
        // no surface, or standing on a single point in the plan: nothing to lay, and the scenery
        // goes on drawing it itself
        m_refusals.push_back( id );
        return;
    }

    corner const corners[ 3 ] { A, B, C };
    {
        auto edges { 0.0 };
        for( std::size_t index = 0; index < 3; ++index ) {
            auto const &from { corners[ index ] };
            auto const &to { corners[ ( index + 1 ) % 3 ] };
            edges += std::sqrt(
                ( to.x - from.x ) * ( to.x - from.x ) + ( to.z - from.z ) * ( to.z - from.z ) );
        }
        auto const plan {
            0.5 * std::abs(
                ( B.x - A.x ) * ( C.z - A.z ) - ( C.x - A.x ) * ( B.z - A.z ) ) };
        m_edgesum += plan * edges / 3.0;
        m_areasum += plan;
    }
    spilled record {};
    record.material = material_index( Material );
    measure_repeat( record.material, corners );
    for( std::size_t index = 0; index < 3; ++index ) {
        record.corners[ index * 3 + 0 ] = corners[ index ].x;
        record.corners[ index * 3 + 1 ] = corners[ index ].y;
        record.corners[ index * 3 + 2 ] = corners[ index ].z;
    }
    m_scratch.write( reinterpret_cast<char const *>( &record ), sizeof( record ) );

    for( auto const &vertex : corners ) {
        if( false == m_anything ) {
            m_anything = true;
            m_minx = m_maxx = vertex.x;
            m_minz = m_maxz = vertex.z;
            m_lowest = m_highest = vertex.y;
            continue;
        }
        m_minx = std::min( m_minx, vertex.x ); m_maxx = std::max( m_maxx, vertex.x );
        m_minz = std::min( m_minz, vertex.z ); m_maxz = std::max( m_maxz, vertex.z );
        m_lowest = std::min( m_lowest, vertex.y ); m_highest = std::max( m_highest, vertex.y );
    }
}

// The root square starts on a multiple of the finest tile side and doubles until it holds
// everything, so that where a tile's borders fall follows from the terrain alone and a second
// cook of the same scenery puts them in the same place.
void
cooker::measure() {

    // the typical edge and the tile that suits it, rounded to a power of two so that the same scenery
    // always comes out the same way
    if( m_areasum > 0.0 ) {
        auto const typical { m_edgesum / m_areasum };
        auto const wanted { std::clamp( tileedges * typical, smallesttileside, largesttileside ) };
        m_finesttile = std::clamp(
            std::exp2( std::round( std::log2( wanted ) ) ), smallesttileside, largesttileside );
    }

    m_west = std::floor( m_minx / m_finesttile ) * m_finesttile;
    m_north = std::floor( m_minz / m_finesttile ) * m_finesttile;
    auto const needed { std::max( m_maxx - m_west, m_maxz - m_north ) };
    m_side = m_finesttile;
    m_levels = 1;
    while( ( m_side < needed ) && ( m_levels < maxlevels ) ) {
        m_side *= 2.0;
        ++m_levels;
    }
    // a height range of zero would make every height quantize to nothing
    if( m_highest - m_lowest < 1.0 ) { m_highest = m_lowest + 1.0; }
}

void
cooker::square( tile_address const &Tile, double &West, double &North, double &Side ) const {

    Side = m_side / static_cast<double>( 1u << Tile.level );
    West = m_west + Tile.x * Side;
    North = m_north + Tile.z * Side;
}

bool
cooker::finish( sink const &Sink, terrain_table &Table ) {

    std::sort( m_refusals.begin(), m_refusals.end() );
    m_report.refused = m_refusals.size();

    if( ( false == m_anything ) || ( false == m_scratch.good() ) ) {
        return false;
    }
    m_scratch.flush();
    measure();

    // which of the spilled triangles touch which tile of the finest level. a triangle can span
    // several, and each of them cuts its own piece out of it
    auto const finest { m_levels - 1 };
    auto const acrossfinest { static_cast<std::int32_t>( 1u << finest ) };
    auto const records { m_scratch.tellp() / static_cast<std::streamoff>( sizeof( spilled ) ) };
    m_scratch.seekg( 0 );
    for( std::streamoff index = 0; index < records; ++index ) {
        spilled record {};
        m_scratch.read( reinterpret_cast<char *>( &record ), sizeof( record ) );
        if( false == m_scratch.good() ) { return false; }
        auto minx { record.corners[ 0 ] }, maxx { minx };
        auto minz { record.corners[ 2 ] }, maxz { minz };
        for( std::size_t at = 0; at < 3; ++at ) {
            minx = std::min( minx, record.corners[ at * 3 + 0 ] ); maxx = std::max( maxx, record.corners[ at * 3 + 0 ] );
            minz = std::min( minz, record.corners[ at * 3 + 2 ] ); maxz = std::max( maxz, record.corners[ at * 3 + 2 ] );
        }
        auto const first = [ this ]( double const Value, double const Origin ) {
            return static_cast<std::int32_t>( std::floor( ( Value - Origin ) / m_finesttile ) ); };
        // a hair past the triangle either way: one that merely touches a border belongs to the tile
        // beyond it as well, and that is what lets two tiles agree on the normal of a vertex they
        // share. it contributes nothing to that tile's own triangles - there is nothing of it inside
        auto const fromx { std::clamp( first( minx - weldepsilon, m_west ), 0, acrossfinest - 1 ) };
        auto const tox { std::clamp( first( maxx + weldepsilon, m_west ), 0, acrossfinest - 1 ) };
        auto const fromz { std::clamp( first( minz - weldepsilon, m_north ), 0, acrossfinest - 1 ) };
        auto const toz { std::clamp( first( maxz + weldepsilon, m_north ), 0, acrossfinest - 1 ) };
        for( auto x = fromx; x <= tox; ++x ) {
            for( auto z = fromz; z <= toz; ++z ) {
                m_placement.emplace_back(
                    tile_key( { finest, x, z } ), static_cast<std::uint32_t>( index ) );
            }
        }
    }
    std::sort( m_placement.begin(), m_placement.end() );

    m_errors.assign( m_levels, 0.0 );
    m_levelarea.assign( m_levels, 0.0 );
    m_leveltriangles.assign( m_levels, 0 );
    cook( { 0, 0, 0 }, Sink );
    m_report.levels = m_levels;

    Table.rules = cook_rules;
    Table.west = m_west;
    Table.north = m_north;
    Table.side = m_side;
    Table.lowest = m_lowest;
    Table.highest = m_highest;
    Table.levels = m_levels;
    Table.errors.assign( m_errors.begin(), m_errors.end() );
    // the side of the square a triangle of this level covers on average: what it is worth on screen
    Table.edges.clear();
    for( std::uint32_t level = 0; level < m_levels; ++level ) {
        auto const triangles { m_leveltriangles[ level ] };
        auto const typical {
            triangles > 0 ? std::sqrt( 2.0 * m_levelarea[ level ] / static_cast<double>( triangles ) ) : 0.0 };
        Table.edges.push_back( static_cast<float>( typical ) );
    }
    Table.materials.clear();
    for( std::size_t index = 0; index < m_materials.size(); ++index ) {
        // a material the measurement never caught falls back to something plausible rather than
        // to zero, which would divide by nothing in the shader
        auto const caught { ( index < m_repeatcounts.size() ) && ( m_repeatcounts[ index ] > 0 ) };
        auto const repeat {
            caught ? m_repeatsums[ index ] / static_cast<double>( m_repeatcounts[ index ] ) : 4.0 };
        Table.materials.push_back( { m_materials[ index ], static_cast<float>( repeat ) } );
    }

    m_scratch.close();
    std::error_code error;
    std::filesystem::remove( m_scratchpath, error );
    m_scratchpath.clear();
    return true;
}

cooker::mesh
cooker::cook( tile_address const &Tile, sink const &Sink ) {

    auto result {
        ( Tile.level + 1 == m_levels ) ? lay( Tile ) : merge( Tile, Sink ) };
    if( true == result.empty() ) { return result; }

    {
        double west { 0.0 }, north { 0.0 }, side { 0.0 };
        square( Tile, west, north, side );
        auto const merged { result.indices.size() / 3 };
        if( Tile.level + 1 < m_levels ) {
            // A level holds what four tiles below it held, and is collapsed back to what one of them
            // held: the cost of a level is then a quarter of the level below, and the whole pyramid
            // costs a third more than the finest alone. A count fixed in advance cannot do that - a
            // tile that never reaches it would be copied rather than simplified - so it only serves
            // as a ceiling.
            simplify(
                result, std::clamp( merged / 4, smallesttile, m_tilebudget ),
                side / errorshare, side / blendshare, side / blendshare );
        }
        else {
            // the finest level keeps its shape to within a fraction of nothing, and is let go of
            // only where it says the same thing twice
            // Its outline is left exactly where the scenery put it. Ground drawn in separate patches
            // meets at T-junctions - a vertex of one lying partway along an edge of the other - and the
            // two outlines there are not the same set of vertices, so moving one and not the other opens
            // a crack. Far away that is a fraction of a pixel and worth the triangles; on the ground the
            // camera stands on it is a line of sky through the field.
            simplify( result, smallesttile, finesterror, side / blendshare, 0.0 );
        }
    }

    m_errors[ Tile.level ] = std::max( m_errors[ Tile.level ], result.error );
    // the ground this tile covers and how many triangles it spends on it, from which the side of the
    // square one of them covers on average - what a triangle of this tile is worth on screen
    double area { 0.0 };
    std::size_t triangles { 0 };
    for( std::size_t index = 0; index + 2 < result.indices.size(); index += 3 ) {
        auto const &a { result.vertices[ result.indices[ index + 0 ] ] };
        auto const &b { result.vertices[ result.indices[ index + 1 ] ] };
        auto const &c { result.vertices[ result.indices[ index + 2 ] ] };
        area += 0.5 * std::abs( ( b.x - a.x ) * ( c.z - a.z ) - ( c.x - a.x ) * ( b.z - a.z ) );
        ++triangles;
    }
    m_levelarea[ Tile.level ] += area;
    m_leveltriangles[ Tile.level ] += triangles;

    tile_measure measure {};
    measure.error = static_cast<float>( result.error );
    measure.edge = static_cast<float>(
        triangles > 0 ? std::sqrt( 2.0 * area / static_cast<double>( triangles ) ) : 0.0 );

    std::vector<std::uint8_t> bytes;
    encode( Tile, result, bytes );
    Sink( Tile, bytes, measure );

    ++m_report.tiles;
    m_report.triangles += result.indices.size() / 3;
    m_report.vertices += result.vertices.size();
    m_report.bytes += bytes.size();
    if( ( false == m_quiet ) && ( m_report.tiles % 200 == 0 ) ) {
        std::printf( "\rCooking terrain: %zu tiles", m_report.tiles );
        std::fflush( stdout );
    }
    return result;
}

namespace {

// Welds vertices that land in the same place, whatever material the triangle bringing them there is
// drawn with. That is what lets one triangle have a different material at each corner, which is what
// the fragment stage blends across: keeping a vertex per material instead would put a hard edge along
// every boundary, on the triangle's own edges. The vertex keeps the material of whichever triangle
// reached it first, and a boundary therefore moves by at most half a triangle.
//
// Everything the cut produces is either a vertex of the source or a point on a tile border worked out
// the same way from both sides, so the tolerance only has to cover the last bits of the arithmetic.
class welder {

public:
    explicit welder( std::vector<cooker::meshvertex> &Vertices ) : m_vertices( Vertices ) {}

    std::uint32_t operator()( cooker::meshvertex const &Vertex ) {
        auto const key { make_key( Vertex ) };
        auto const found { m_known.find( key ) };
        if( found != m_known.end() ) { return found->second; }
        auto const index { static_cast<std::uint32_t>( m_vertices.size() ) };
        m_vertices.push_back( Vertex );
        m_known.emplace( key, index );
        return index;
    }

private:
    struct key {
        std::int64_t x, y, z;
        bool operator==( key const & ) const = default;
    };
    struct keyhash {
        std::size_t operator()( key const &Key ) const noexcept {
            std::size_t hash { 1469598103934665603ull };
            auto const mix = [ &hash ]( std::int64_t const Value ) {
                hash ^= static_cast<std::size_t>( Value );
                hash *= 1099511628211ull; };
            mix( Key.x ); mix( Key.y ); mix( Key.z );
            return hash;
        }
    };

    static key make_key( cooker::meshvertex const &Vertex ) {
        auto const grid = []( double const Value ) {
            return static_cast<std::int64_t>( std::llround( Value / weldepsilon ) ); };
        return { grid( Vertex.x ), grid( Vertex.y ), grid( Vertex.z ) };
    }

    std::vector<cooker::meshvertex> &m_vertices;
    std::unordered_map<key, std::uint32_t, keyhash> m_known;
};

} // anonymous namespace

cooker::mesh
cooker::lay( tile_address const &Tile ) {

    mesh result;
    double west { 0.0 }, north { 0.0 }, side { 0.0 };
    square( Tile, west, north, side );

    auto const wanted { tile_key( Tile ) };
    auto const from {
        std::lower_bound( m_placement.begin(), m_placement.end(),
            std::pair<std::int64_t, std::uint32_t> { wanted, 0 } ) };
    auto const to {
        std::upper_bound( m_placement.begin(), m_placement.end(),
            std::pair<std::int64_t, std::uint32_t> { wanted, std::numeric_limits<std::uint32_t>::max() } ) };
    if( from == to ) { return result; }

    // read once into memory: the triangles are walked twice, first for the normals and then for the
    // cutting, and a tile holds few enough of them for that to be cheaper than seeking again
    std::vector<spilled> records;
    records.reserve( static_cast<std::size_t>( to - from ) );
    for( auto entry = from; entry != to; ++entry ) {
        m_scratch.seekg( static_cast<std::streamoff>( entry->second ) * sizeof( spilled ) );
        spilled record {};
        m_scratch.read( reinterpret_cast<char *>( &record ), sizeof( record ) );
        if( false == m_scratch.good() ) { m_scratch.clear(); continue; }
        records.push_back( record );
    }
    if( true == records.empty() ) { return result; }

    // The normal at a vertex is the area-weighted sum of the faces meeting there, taken from the
    // scenery's triangles as they were written rather than from the pieces the cut leaves. Both tiles
    // sharing a border read the same triangles around a vertex on it, so both work out the same
    // normal and the light runs across the border without a line.
    std::unordered_map<std::int64_t, std::array<double, 3>> normals;
    auto const place_key = []( double const X, double const Y, double const Z ) {
        auto const grid = []( double const Value ) {
            return static_cast<std::int64_t>( std::llround( Value / weldepsilon ) ); };
        std::int64_t hash { 1469598103934665603ll };
        for( auto const value : { grid( X ), grid( Y ), grid( Z ) } ) {
            hash ^= value;
            hash *= 1099511628211ll;
        }
        return hash; };
    for( auto const &record : records ) {
        auto const &corners { record.corners };
        double const ax { corners[ 0 ] }, ay { corners[ 1 ] }, az { corners[ 2 ] };
        double const bx { corners[ 3 ] }, by { corners[ 4 ] }, bz { corners[ 5 ] };
        double const cx { corners[ 6 ] }, cy { corners[ 7 ] }, cz { corners[ 8 ] };
        // twice the area times the unit normal, which is the weighting wanted
        std::array<double, 3> const face {
            ( by - ay ) * ( cz - az ) - ( bz - az ) * ( cy - ay ),
            ( bz - az ) * ( cx - ax ) - ( bx - ax ) * ( cz - az ),
            ( bx - ax ) * ( cy - ay ) - ( by - ay ) * ( cx - ax ) };
        for( std::size_t corner = 0; corner < 3; ++corner ) {
            auto &sum {
                normals[ place_key(
                    corners[ corner * 3 + 0 ], corners[ corner * 3 + 1 ], corners[ corner * 3 + 2 ] ) ] };
            sum[ 0 ] += face[ 0 ]; sum[ 1 ] += face[ 1 ]; sum[ 2 ] += face[ 2 ];
        }
    }
    // the sum as it stands, not a unit vector: its length is the surface behind it, and that is what
    // makes adding two of them an average rather than a guess
    auto const normal_at = [ & ]( double const X, double const Y, double const Z ) {
        auto const found { normals.find( place_key( X, Y, Z ) ) };
        if( found == normals.end() ) { return std::array<double, 3> { 0.0, 1.0, 0.0 }; }
        return found->second; };

    welder weld { result.vertices };
    std::vector<clipvertex> polygon;
    for( auto const &record : records ) {

        polygon.clear();
        for( std::size_t at = 0; at < 3; ++at ) {
            auto const x { record.corners[ at * 3 + 0 ] };
            auto const y { record.corners[ at * 3 + 1 ] };
            auto const z { record.corners[ at * 3 + 2 ] };
            auto const normal { normal_at( x, y, z ) };
            polygon.push_back( { x, y, z, normal[ 0 ], normal[ 1 ], normal[ 2 ] } );
        }
        clip_side( polygon, 0, west, true );
        clip_side( polygon, 0, west + side, false );
        clip_side( polygon, 1, north, true );
        clip_side( polygon, 1, north + side, false );
        if( polygon.size() < 3 ) { continue; }

        // A wall lying along a tile border is inside both tiles: the cut keeps it whole on either
        // side, since it has nothing to cut. Whichever border it sits on, the tile past it is the one
        // that keeps it, so that it is laid exactly once.
        {
            auto onborderonly { false };
            for( int axis = 0; axis < 2; ++axis ) {
                auto const at = [ axis ]( clipvertex const &Vertex ) { return axis == 0 ? Vertex.x : Vertex.z; };
                auto low { at( polygon.front() ) }, high { low };
                for( auto const &vertex : polygon ) {
                    low = std::min( low, at( vertex ) );
                    high = std::max( high, at( vertex ) );
                }
                auto const upper { ( axis == 0 ? west + side : north + side ) };
                if( ( high - low < weldepsilon ) && ( std::abs( low - upper ) < weldepsilon ) ) {
                    onborderonly = true;
                }
            }
            if( true == onborderonly ) { continue; }
        }

        auto const place = [ & ]( clipvertex const &Vertex ) {
            return weld( { Vertex.x, Vertex.y, Vertex.z, Vertex.nx, Vertex.ny, Vertex.nz,
                           record.material, sides_of( Vertex.x, Vertex.z, west, north, side ) } ); };

        auto const first { place( polygon[ 0 ] ) };
        for( std::size_t corner = 1; corner + 1 < polygon.size(); ++corner ) {
            // the cut keeps the source winding, so fanning the polygon keeps it too
            auto const second { place( polygon[ corner ] ) };
            auto const third { place( polygon[ corner + 1 ] ) };
            if( ( first == second ) || ( second == third ) || ( third == first ) ) { continue; }
            // a triangle that only touched this tile leaves nothing of itself inside it
            auto const &a { result.vertices[ first ] };
            auto const &b { result.vertices[ second ] };
            auto const &c { result.vertices[ third ] };
            auto const twicearea {
                std::sqrt(
                    std::pow( ( b.y - a.y ) * ( c.z - a.z ) - ( b.z - a.z ) * ( c.y - a.y ), 2.0 )
                  + std::pow( ( b.z - a.z ) * ( c.x - a.x ) - ( b.x - a.x ) * ( c.z - a.z ), 2.0 )
                  + std::pow( ( b.x - a.x ) * ( c.y - a.y ) - ( b.y - a.y ) * ( c.x - a.x ), 2.0 ) ) };
            if( 0.5 * twicearea < nullarea ) { continue; }
            result.indices.push_back( first );
            result.indices.push_back( second );
            result.indices.push_back( third );
        }
    }
    return result;
}

cooker::mesh
cooker::merge( tile_address const &Tile, sink const &Sink ) {

    mesh result;
    double west { 0.0 }, north { 0.0 }, side { 0.0 };
    square( Tile, west, north, side );

    welder weld { result.vertices };
    for( std::int32_t stepz = 0; stepz < 2; ++stepz ) {
        for( std::int32_t stepx = 0; stepx < 2; ++stepx ) {

            auto const child {
                cook( { Tile.level + 1, Tile.x * 2 + stepx, Tile.z * 2 + stepz }, Sink ) };
            if( true == child.empty() ) { continue; }
            result.error = std::max( result.error, child.error );

            // the borders between the four are no longer borders: only the outer edge of this
            // tile has to stay where it is
            std::vector<std::uint32_t> here;
            here.reserve( child.vertices.size() );
            for( auto vertex : child.vertices ) {
                vertex.sides = sides_of( vertex.x, vertex.z, west, north, side );
                here.push_back( weld( vertex ) );
            }
            for( auto const index : child.indices ) {
                result.indices.push_back( here[ index ] );
            }
        }
    }
    return result;
}

// Edge collapse, cheapest first. A vertex is only ever collapsed onto one of its neighbours -
// never onto a new point between them - so a vertex that has to stay put, because a neighbouring
// tile shares it, simply is never the one that goes. That is the whole of the crack handling:
// levels meet because the border is literally the same set of vertices in both.
void
cooker::simplify(
    mesh &Mesh, std::size_t const Budget, double const Allowed,
    double const Boundaryreach, double const Outlinereach ) {

    auto triangles { Mesh.indices.size() / 3 };
    if( triangles <= Budget ) { return; }

    auto &vertices { Mesh.vertices };

    // The quadrics are built and evaluated about the middle of the mesh rather than about the
    // scenery's origin. A plane's own constant is minus the normal times a point on it, so in world
    // coordinates it runs to tens or hundreds of thousands, and the quadric's value - a small
    // distance squared - comes out as the difference of two such numbers. There is no precision left
    // in that. Measured from the middle of the tile the constants are the size of the tile.
    std::vector<std::array<double, 3>> local( vertices.size() );
    {
        std::array<double, 3> middle { 0.0, 0.0, 0.0 };
        for( auto const &vertex : vertices ) {
            middle[ 0 ] += vertex.x; middle[ 1 ] += vertex.y; middle[ 2 ] += vertex.z;
        }
        for( auto &value : middle ) { value /= static_cast<double>( vertices.size() ); }
        for( std::size_t index = 0; index < vertices.size(); ++index ) {
            local[ index ] = {
                vertices[ index ].x - middle[ 0 ],
                vertices[ index ].y - middle[ 1 ],
                vertices[ index ].z - middle[ 2 ] };
        }
    }

    std::vector<quadric> quadrics( vertices.size() );
    // the worst the ground has been moved at each vertex by a collapse into it, in metres. what a
    // level is off by against the level below it; merging four tiles already carried their own
    // figures up, so the levels are not counted twice
    std::vector<double> moved( vertices.size(), 0.0 );
    std::vector<std::vector<std::uint32_t>> incident( vertices.size() );
    std::vector<bool> gone( triangles, false );
    std::vector<std::uint32_t> version( vertices.size(), 0 );
    std::vector<bool> removed( vertices.size(), false );

    auto const cornerof = [ &Mesh ]( std::size_t const Triangle, std::size_t const Corner ) -> std::uint32_t & {
        return Mesh.indices[ Triangle * 3 + Corner ]; };

    for( std::size_t triangle = 0; triangle < triangles; ++triangle ) {
        auto const &a { local[ cornerof( triangle, 0 ) ] };
        auto const &b { local[ cornerof( triangle, 1 ) ] };
        auto const &c { local[ cornerof( triangle, 2 ) ] };
        double nx { ( b[ 1 ] - a[ 1 ] ) * ( c[ 2 ] - a[ 2 ] ) - ( b[ 2 ] - a[ 2 ] ) * ( c[ 1 ] - a[ 1 ] ) };
        double ny { ( b[ 2 ] - a[ 2 ] ) * ( c[ 0 ] - a[ 0 ] ) - ( b[ 0 ] - a[ 0 ] ) * ( c[ 2 ] - a[ 2 ] ) };
        double nz { ( b[ 0 ] - a[ 0 ] ) * ( c[ 1 ] - a[ 1 ] ) - ( b[ 1 ] - a[ 1 ] ) * ( c[ 0 ] - a[ 0 ] ) };
        auto const length { std::sqrt( nx * nx + ny * ny + nz * nz ) };
        if( length < 1e-12 ) { continue; }
        auto const area { 0.5 * length };
        nx /= length; ny /= length; nz /= length;
        auto const offset { -( nx * a[ 0 ] + ny * a[ 1 ] + nz * a[ 2 ] ) };
        for( std::size_t corner = 0; corner < 3; ++corner ) {
            auto const index { cornerof( triangle, corner ) };
            quadrics[ index ].add_plane( nx, ny, nz, offset, area );
            incident[ index ].push_back( static_cast<std::uint32_t>( triangle ) );
        }
    }

    // Where the ground ends - the outline of a hole the scenery left, the edge of the terrain, or the
    // seam between two patches drawn at different densities - only one triangle meets the edge. Those
    // edges are what the rules below protect, and a plane standing on each of them, weighted heavily,
    // is added to both its ends so that the outline is expensive to disturb as well as forbidden to
    // break.
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::size_t> uses;
    std::vector<bool> onmeshedge( vertices.size(), false );
    {
        for( std::size_t triangle = 0; triangle < triangles; ++triangle ) {
            for( std::size_t corner = 0; corner < 3; ++corner ) {
                auto const from { cornerof( triangle, corner ) };
                auto const to { cornerof( triangle, ( corner + 1 ) % 3 ) };
                ++uses[ { std::min( from, to ), std::max( from, to ) } ];
            }
        }
        for( std::size_t triangle = 0; triangle < triangles; ++triangle ) {
            auto const &a { local[ cornerof( triangle, 0 ) ] };
            auto const &b { local[ cornerof( triangle, 1 ) ] };
            auto const &c { local[ cornerof( triangle, 2 ) ] };
            double nx { ( b[ 1 ] - a[ 1 ] ) * ( c[ 2 ] - a[ 2 ] ) - ( b[ 2 ] - a[ 2 ] ) * ( c[ 1 ] - a[ 1 ] ) };
            double ny { ( b[ 2 ] - a[ 2 ] ) * ( c[ 0 ] - a[ 0 ] ) - ( b[ 0 ] - a[ 0 ] ) * ( c[ 2 ] - a[ 2 ] ) };
            double nz { ( b[ 0 ] - a[ 0 ] ) * ( c[ 1 ] - a[ 1 ] ) - ( b[ 1 ] - a[ 1 ] ) * ( c[ 0 ] - a[ 0 ] ) };
            auto const facelength { std::sqrt( nx * nx + ny * ny + nz * nz ) };
            if( facelength < 1e-12 ) { continue; }
            nx /= facelength; ny /= facelength; nz /= facelength;
            for( std::size_t corner = 0; corner < 3; ++corner ) {
                auto const from { cornerof( triangle, corner ) };
                auto const to { cornerof( triangle, ( corner + 1 ) % 3 ) };
                auto const use { uses.find( { std::min( from, to ), std::max( from, to ) } ) };
                if( ( use == uses.end() ) || ( use->second != 1 ) ) { continue; }
                auto const &head { local[ from ] };
                auto const &tail { local[ to ] };
                double ex { tail[ 0 ] - head[ 0 ] }, ey { tail[ 1 ] - head[ 1 ] }, ez { tail[ 2 ] - head[ 2 ] };
                auto const edgelength { std::sqrt( ex * ex + ey * ey + ez * ez ) };
                if( edgelength < 1e-9 ) { continue; }
                ex /= edgelength; ey /= edgelength; ez /= edgelength;
                // perpendicular to both the edge and the face it belongs to
                auto const px { ey * nz - ez * ny };
                auto const py { ez * nx - ex * nz };
                auto const pz { ex * ny - ey * nx };
                auto const offset { -( px * head[ 0 ] + py * head[ 1 ] + pz * head[ 2 ] ) };
                auto const weight { 100.0 * edgelength * edgelength };
                quadrics[ from ].add_plane( px, py, pz, offset, weight );
                quadrics[ to ].add_plane( px, py, pz, offset, weight );
            }
        }
        for( auto const &[ edge, count ] : uses ) {
            if( count != 1 ) { continue; }
            onmeshedge[ edge.first ] = true;
            onmeshedge[ edge.second ] = true;
        }
    }


    struct candidate {
        double cost;
        std::uint32_t from, to;
        std::uint32_t fromversion, toversion;
        bool operator<( candidate const &Other ) const { return cost > Other.cost; }
    };
    std::priority_queue<candidate> queue;

    // the cost of taking From away and letting To stand for both. only what the collapse leaves
    // behind is measured; where To sits does not change, so its own quadric is already paid for
    auto const cost_of = [ & ]( std::uint32_t const From, std::uint32_t const To ) {
        quadric together { quadrics[ From ] };
        together += quadrics[ To ];
        return std::max( 0.0, together.at( local[ To ][ 0 ], local[ To ][ 1 ], local[ To ][ 2 ] ) ); };

    // How far the ground moves where From used to be, if From were collapsed into To: the height of
    // whatever would then lie over that spot, against the height that is there now. Worked out before
    // the collapse rather than after it, so that a collapse which moves the ground too far is passed
    // over instead of being taken and regretted. The cost above orders the candidates; this is the
    // one that says whether a candidate may be taken at all, because it is a distance and the cost is
    // not.
    auto const drop_of = [ & ]( std::uint32_t const From, std::uint32_t const To ) {
        auto const &was { local[ From ] };
        for( auto const triangle : incident[ From ] ) {
            if( true == gone[ triangle ] ) { continue; }
            std::array<std::uint32_t, 3> corners {
                cornerof( triangle, 0 ), cornerof( triangle, 1 ), cornerof( triangle, 2 ) };
            // a triangle holding both ends of the edge disappears with it, and covers nothing after
            if( std::find( corners.begin(), corners.end(), To ) != corners.end() ) { continue; }
            std::array<std::array<double, 3> const *, 3> after {
                &local[ corners[ 0 ] ], &local[ corners[ 1 ] ], &local[ corners[ 2 ] ] };
            for( std::size_t corner = 0; corner < 3; ++corner ) {
                if( corners[ corner ] == From ) { after[ corner ] = &local[ To ]; }
            }
            auto const &a { *after[ 0 ] }; auto const &b { *after[ 1 ] }; auto const &c { *after[ 2 ] };
            auto const twice {
                ( b[ 0 ] - a[ 0 ] ) * ( c[ 2 ] - a[ 2 ] ) - ( c[ 0 ] - a[ 0 ] ) * ( b[ 2 ] - a[ 2 ] ) };
            if( std::abs( twice ) < 1e-12 ) { continue; }
            auto const first {
                ( ( b[ 0 ] - was[ 0 ] ) * ( c[ 2 ] - was[ 2 ] ) - ( c[ 0 ] - was[ 0 ] ) * ( b[ 2 ] - was[ 2 ] ) ) / twice };
            auto const second {
                ( ( c[ 0 ] - was[ 0 ] ) * ( a[ 2 ] - was[ 2 ] ) - ( a[ 0 ] - was[ 0 ] ) * ( c[ 2 ] - was[ 2 ] ) ) / twice };
            auto const third { 1.0 - first - second };
            if( ( first < -1e-9 ) || ( second < -1e-9 ) || ( third < -1e-9 ) ) { continue; }
            return std::abs( first * a[ 1 ] + second * b[ 1 ] + third * c[ 1 ] - was[ 1 ] );
        }
        // nothing would lie over that spot: the edge of the ground, or a wall, whose triangles cover
        // nothing in the plan at all. what changes there is the height where the vertex went
        return std::abs( was[ 1 ] - local[ To ][ 1 ] ); };

    // vertices with a neighbour of another material: the ground changes colour around them, and how
    // large the triangles there are is how wide that change looks
    std::vector<bool> onboundary( vertices.size(), false );
    for( std::size_t triangle = 0; triangle < triangles; ++triangle ) {
        for( std::size_t corner = 0; corner < 3; ++corner ) {
            auto const here { cornerof( triangle, corner ) };
            auto const next { cornerof( triangle, ( corner + 1 ) % 3 ) };
            if( vertices[ here ].material != vertices[ next ].material ) {
                onboundary[ here ] = true;
                onboundary[ next ] = true;
            }
        }
    }

    auto const meshedge = [ &uses ]( std::uint32_t const From, std::uint32_t const To ) {
        auto const found { uses.find( { std::min( From, To ), std::max( From, To ) } ) };
        return ( found != uses.end() ) && ( found->second == 1 ); };

    auto const allowed = [ & ]( std::uint32_t const From, std::uint32_t const To ) {
        if( ( true == removed[ From ] ) || ( true == removed[ To ] ) ) { return false; }
        // a vertex a neighbouring tile shares stays where it is
        if( true == vertices[ From ].locked() ) { return false; }
        // Where the ground ends, a vertex may only travel along that end, onto another vertex of it, and
        // no further than this level reaches. What is lost then is the thin wedge between the old
        // outline and the new one. Letting it collapse inward instead takes whole triangles with it and
        // leaves holes the size of them - and since the height barely changes there, nothing in the
        // error measure objects: forty per cent of the ground went that way before this rule. Forbidding
        // it outright is no good either, since a narrow strip of terrain is almost all outline, and then
        // no level above the finest can be thinned at all
        if( true == onmeshedge[ From ] ) {
            if( Outlinereach <= 0.0 ) { return false; }
            if( false == meshedge( From, To ) ) { return false; }
            auto const &from { local[ From ] };
            auto const &to { local[ To ] };
            auto const reach {
                std::sqrt( ( from[ 0 ] - to[ 0 ] ) * ( from[ 0 ] - to[ 0 ] )
                         + ( from[ 1 ] - to[ 1 ] ) * ( from[ 1 ] - to[ 1 ] )
                         + ( from[ 2 ] - to[ 2 ] ) * ( from[ 2 ] - to[ 2 ] ) ) };
            if( reach > Outlinereach ) { return false; }
        }

        // nothing that turns a triangle over or squashes it flat
        for( auto const triangle : incident[ From ] ) {
            if( true == gone[ triangle ] ) { continue; }
            std::array<std::uint32_t, 3> corners {
                cornerof( triangle, 0 ), cornerof( triangle, 1 ), cornerof( triangle, 2 ) };
            if( std::find( corners.begin(), corners.end(), To ) != corners.end() ) {
                // this triangle goes when the edge does. it may not be the only one holding a
                // piece of the tile's border up: From is not on a border, so the only edge of it
                // that can be one is between To and the third corner
                for( auto const third : corners ) {
                    if( ( third == From ) || ( third == To ) ) { continue; }
                    // a piece of a tile's border, or of the ground's own edge: either way this triangle
                    // is the only thing holding it, and it is about to go
                    if( ( vertices[ To ].sides & vertices[ third ].sides ) != 0 ) { return false; }
                    if( true == meshedge( To, third ) ) { return false; }
                }
                continue;
            }
            std::array<std::array<double, 3> const *, 3> before {
                &local[ corners[ 0 ] ], &local[ corners[ 1 ] ], &local[ corners[ 2 ] ] };
            std::array<std::array<double, 3> const *, 3> after { before };
            for( std::size_t corner = 0; corner < 3; ++corner ) {
                if( corners[ corner ] == From ) { after[ corner ] = &local[ To ]; }
            }
            // the normal in space, so that this holds for a wall as well as for open ground
            auto const normal = []( std::array<std::array<double, 3> const *, 3> const &Corners ) {
                auto const &a { *Corners[ 0 ] }; auto const &b { *Corners[ 1 ] }; auto const &c { *Corners[ 2 ] };
                return std::array<double, 3> {
                    ( b[ 1 ] - a[ 1 ] ) * ( c[ 2 ] - a[ 2 ] ) - ( b[ 2 ] - a[ 2 ] ) * ( c[ 1 ] - a[ 1 ] ),
                    ( b[ 2 ] - a[ 2 ] ) * ( c[ 0 ] - a[ 0 ] ) - ( b[ 0 ] - a[ 0 ] ) * ( c[ 2 ] - a[ 2 ] ),
                    ( b[ 0 ] - a[ 0 ] ) * ( c[ 1 ] - a[ 1 ] ) - ( b[ 1 ] - a[ 1 ] ) * ( c[ 0 ] - a[ 0 ] ) }; };
            auto const was { normal( before ) };
            auto const now { normal( after ) };
            auto const length {
                std::sqrt( now[ 0 ] * now[ 0 ] + now[ 1 ] * now[ 1 ] + now[ 2 ] * now[ 2 ] ) };
            // nothing that squashes a triangle flat, and nothing that turns one over
            if( 0.5 * length < nullarea ) { return false; }
            if( was[ 0 ] * now[ 0 ] + was[ 1 ] * now[ 1 ] + was[ 2 ] * now[ 2 ] <= 0.0 ) { return false; }
        }
        // and nothing that moves the ground further than this level is allowed to
        if( drop_of( From, To ) > Allowed ) { return false; }
        // Anything touching a boundary between materials - crossing it, or standing next to it - may
        // only reach so far. Short of that the boundary is left where the scenery drew it and the
        // change stays narrow; past it there is nothing worth defending, since at the distance this
        // level is drawn from the whole thing is a pixel or two wide.
        if( ( true == onboundary[ From ] ) || ( true == onboundary[ To ] )
         || ( vertices[ From ].material != vertices[ To ].material ) ) {
            auto const &from { local[ From ] };
            auto const &to { local[ To ] };
            auto const reach {
                std::sqrt( ( from[ 0 ] - to[ 0 ] ) * ( from[ 0 ] - to[ 0 ] )
                         + ( from[ 1 ] - to[ 1 ] ) * ( from[ 1 ] - to[ 1 ] )
                         + ( from[ 2 ] - to[ 2 ] ) * ( from[ 2 ] - to[ 2 ] ) ) };
            if( reach > Boundaryreach ) { return false; }
        }
        return true; };

    auto const offer = [ & ]( std::uint32_t const Vertex ) {
        for( auto const triangle : incident[ Vertex ] ) {
            if( true == gone[ triangle ] ) { continue; }
            for( std::size_t corner = 0; corner < 3; ++corner ) {
                auto const other { cornerof( triangle, corner ) };
                if( other == Vertex ) { continue; }
                for( auto const [ from, to ] : { std::pair { Vertex, other }, std::pair { other, Vertex } } ) {
                    if( false == allowed( from, to ) ) { continue; }
                    queue.push( { cost_of( from, to ), from, to, version[ from ], version[ to ] } );
                }
            }
        } };

    for( std::uint32_t vertex = 0; vertex < vertices.size(); ++vertex ) { offer( vertex ); }

    while( ( triangles > Budget ) && ( false == queue.empty() ) ) {

        auto const best { queue.top() };
        queue.pop();
        if( ( version[ best.from ] != best.fromversion ) || ( version[ best.to ] != best.toversion ) ) {
            continue;   // one of the two has moved on since this was worked out
        }
        if( false == allowed( best.from, best.to ) ) { continue; }

        auto const drop { drop_of( best.from, best.to ) };
        moved[ best.to ] = std::max( { moved[ best.to ], moved[ best.from ], drop } );
        Mesh.error = std::max( Mesh.error, drop );

        removed[ best.from ] = true;
        quadrics[ best.to ] += quadrics[ best.from ];
        // the survivor now stands for both, and its normal has to say so, or a coarse level ends up
        // lit by whichever slope happened to keep its vertex
        vertices[ best.to ].nx += vertices[ best.from ].nx;
        vertices[ best.to ].ny += vertices[ best.from ].ny;
        vertices[ best.to ].nz += vertices[ best.from ].nz;

        for( auto const triangle : incident[ best.from ] ) {
            if( true == gone[ triangle ] ) { continue; }
            auto wasthere { false };
            for( std::size_t corner = 0; corner < 3; ++corner ) {
                if( cornerof( triangle, corner ) == best.to ) { wasthere = true; }
            }
            if( true == wasthere ) {
                // the triangle had both ends of the edge: it has no area left
                gone[ triangle ] = true;
                --triangles;
                continue;
            }
            for( std::size_t corner = 0; corner < 3; ++corner ) {
                if( cornerof( triangle, corner ) == best.from ) { cornerof( triangle, corner ) = best.to; }
            }
            incident[ best.to ].push_back( triangle );
        }
        incident[ best.from ].clear();
        ++version[ best.from ];
        ++version[ best.to ];

        offer( best.to );
    }

    // drop what went, and close up the numbering
    std::vector<std::uint32_t> indices;
    indices.reserve( triangles * 3 );
    for( std::size_t triangle = 0; triangle < gone.size(); ++triangle ) {
        if( true == gone[ triangle ] ) { continue; }
        for( std::size_t corner = 0; corner < 3; ++corner ) {
            indices.push_back( cornerof( triangle, corner ) );
        }
    }
    std::vector<std::uint32_t> renumber( vertices.size(), std::numeric_limits<std::uint32_t>::max() );
    std::vector<meshvertex> kept;
    kept.reserve( vertices.size() );
    for( auto &index : indices ) {
        if( renumber[ index ] == std::numeric_limits<std::uint32_t>::max() ) {
            renumber[ index ] = static_cast<std::uint32_t>( kept.size() );
            kept.push_back( vertices[ index ] );
        }
        index = renumber[ index ];
    }
    Mesh.vertices.swap( kept );
    Mesh.indices.swap( indices );
}

namespace {

template <typename Type_>
void
append( std::vector<std::uint8_t> &Out, Type_ const &Value ) {
    auto const *bytes { reinterpret_cast<std::uint8_t const *>( &Value ) };
    Out.insert( Out.end(), bytes, bytes + sizeof( Type_ ) );
}

template <typename Index_>
void
append_indices( std::vector<std::uint8_t> &Out, std::vector<std::uint32_t> const &Indices ) {
    std::vector<Index_> narrow( Indices.begin(), Indices.end() );
    std::vector<Index_> codes;
    highwater_encode( narrow, codes );
    for( auto const code : codes ) { append( Out, code ); }
}

template <typename Index_>
void
append_edge( std::vector<std::uint8_t> &Out, std::vector<std::uint32_t> const &Edge ) {
    append( Out, static_cast<std::uint32_t>( Edge.size() ) );
    for( auto const index : Edge ) { append( Out, static_cast<Index_>( index ) ); }
}

} // anonymous namespace

// The layout is the format's: the header, then the vertices as zigzag deltas of their quantized
// coordinates, then the triangles high-water-mark encoded, then the four lists of border
// vertices, then extensions. Texture coordinates and materials are ours, and ride in one.
void
cooker::encode( tile_address const &Tile, mesh const &Mesh, std::vector<std::uint8_t> &Out ) const {

    double west { 0.0 }, north { 0.0 }, side { 0.0 };
    square( Tile, west, north, side );

    // The format's indices are coded against the highest one used so far, which only works when a
    // vertex is numbered the first time a triangle reaches for it. So the vertices are put in the
    // order the triangles ask for them; anything no triangle asks for is left out.
    std::vector<std::uint32_t> order;
    std::vector<std::uint32_t> indices;
    {
        constexpr std::uint32_t unplaced { std::numeric_limits<std::uint32_t>::max() };
        std::vector<std::uint32_t> placed( Mesh.vertices.size(), unplaced );
        order.reserve( Mesh.vertices.size() );
        indices.reserve( Mesh.indices.size() );
        for( auto const index : Mesh.indices ) {
            if( placed[ index ] == unplaced ) {
                placed[ index ] = static_cast<std::uint32_t>( order.size() );
                order.push_back( index );
            }
            indices.push_back( placed[ index ] );
        }
    }

    auto lowest { Mesh.vertices[ order.front() ].y }, highest { lowest };
    for( auto const index : order ) {
        lowest = std::min( lowest, Mesh.vertices[ index ].y );
        highest = std::max( highest, Mesh.vertices[ index ].y );
    }
    // heights are quantized against this tile's own range, as the format says. two tiles sharing
    // a border therefore round a shared vertex a fraction of a millimetre differently, which is
    // what the format's skirts are for elsewhere and is below anything visible here
    auto const heightspan { std::max( 1e-3, highest - lowest ) };

    file_header header {};
    header.centre_x = west + side * 0.5;
    header.centre_y = ( lowest + highest ) * 0.5;
    header.centre_z = north + side * 0.5;
    header.lowest = static_cast<float>( lowest );
    header.highest = static_cast<float>( highest );
    header.sphere_x = header.centre_x;
    header.sphere_y = header.centre_y;
    header.sphere_z = header.centre_z;
    header.sphere_radius = std::sqrt( 0.5 * side * side + 0.25 * heightspan * heightspan );
    header.horizon_x = header.horizon_y = header.horizon_z = 0.0;

    Out.clear();
    Out.reserve( order.size() * 10 + indices.size() * 2 + sizeof( header ) + 64 );
    append( Out, header );
    append( Out, static_cast<std::uint32_t>( order.size() ) );

    std::vector<std::uint16_t> quantx( order.size() );
    std::vector<std::uint16_t> quanty( order.size() );
    std::vector<std::uint16_t> quantz( order.size() );
    for( std::size_t index = 0; index < order.size(); ++index ) {
        auto const &vertex { Mesh.vertices[ order[ index ] ] };
        quantx[ index ] = quantize( vertex.x, west, side );
        quanty[ index ] = quantize( vertex.y, lowest, heightspan );
        quantz[ index ] = quantize( vertex.z, north, side );
    }
    // the deltas between one vertex and the next, so the run is small numbers
    auto const deltas = [ &Out ]( std::vector<std::uint16_t> const &Values ) {
        std::int32_t previous { 0 };
        for( auto const value : Values ) {
            append( Out, zigzag_encode( static_cast<std::int32_t>( value ) - previous ) );
            previous = static_cast<std::int32_t>( value );
        } };
    deltas( quantx );
    deltas( quantz );
    deltas( quanty );

    auto const wide { order.size() > 65536 };
    auto const indexsize { wide ? 4u : 2u };
    // the format wants the indices to start on a boundary of their own size
    while( ( Out.size() % indexsize ) != 0 ) { Out.push_back( 0 ); }

    append( Out, static_cast<std::uint32_t>( indices.size() / 3 ) );
    if( true == wide ) { append_indices<std::uint32_t>( Out, indices ); }
    else               { append_indices<std::uint16_t>( Out, indices ); }

    // the border vertices, which the format expects to be told about by the quantized coordinate
    // they share
    std::vector<std::uint32_t> edges[ 4 ];
    for( std::size_t index = 0; index < order.size(); ++index ) {
        if( quantx[ index ] == 0 )            { edges[ 0 ].push_back( static_cast<std::uint32_t>( index ) ); }
        if( quantz[ index ] == 0 )            { edges[ 1 ].push_back( static_cast<std::uint32_t>( index ) ); }
        if( quantx[ index ] == quantum_max )  { edges[ 2 ].push_back( static_cast<std::uint32_t>( index ) ); }
        if( quantz[ index ] == quantum_max )  { edges[ 3 ].push_back( static_cast<std::uint32_t>( index ) ); }
    }
    for( auto const &edge : edges ) {
        if( true == wide ) { append_edge<std::uint32_t>( Out, edge ); }
        else               { append_edge<std::uint16_t>( Out, edge ); }
    }

    // the format's own: a normal per vertex, two bytes each
    append( Out, static_cast<std::uint8_t>( ext_normals ) );
    append( Out, static_cast<std::uint32_t>( order.size() * 2 ) );
    for( auto const index : order ) {
        auto const &vertex { Mesh.vertices[ index ] };
        std::uint8_t first { 128 }, second { 128 };
        encode_normal( vertex.nx, vertex.ny, vertex.nz, first, second );
        append( Out, first );
        append( Out, second );
    }

    // ours: which ground material each vertex belongs to. the format has nowhere for it
    append( Out, static_cast<std::uint8_t>( ext_ground ) );
    append( Out, static_cast<std::uint32_t>( order.size() * sizeof( std::uint16_t ) ) );
    for( auto const index : order ) {
        append( Out, Mesh.vertices[ index ].material );
    }
}

} // namespace quantizedmesh
