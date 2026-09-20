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
// cost vertices neither side had. It also has to stay small enough to be worth loading and culling
// on its own, which is what the bounds are for.
constexpr double tileedges { 16.0 };
constexpr double smallesttileside { 128.0 };
constexpr double largesttileside { 1024.0 };
// how many edge lengths are kept to find the typical one. the first hundred thousand triangles say
// as much about a scenery as all of them
constexpr std::size_t edgesample { 100000 };
// a tile is never collapsed below this, however far away it is drawn: past a point the triangles
// saved are not worth the ground losing its shape
constexpr std::size_t smallesttile { 64 };
// A level may move the ground by this share of its tile's side, and no further. A level is used from
// about twice its tile's side away, where a share this small is well under a pixel; letting it go
// further would buy triangles that no distance is far enough to use. Ground already so coarse that a
// level cannot be thinned within it simply comes out nearly a copy of the level below, which is the
// right answer: there was nothing there to save.
constexpr double errorshare { 128.0 };

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
};

clipvertex
between( clipvertex const &From, clipvertex const &To, double const Share ) {
    return {
        From.x + ( To.x - From.x ) * Share,
        From.y + ( To.y - From.y ) * Share,
        From.z + ( To.z - From.z ) * Share };
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
    if( m_edges.size() < edgesample ) {
        for( std::size_t index = 0; index < 3; ++index ) {
            auto const &from { corners[ index ] };
            auto const &to { corners[ ( index + 1 ) % 3 ] };
            auto const length {
                std::sqrt( ( to.x - from.x ) * ( to.x - from.x ) + ( to.z - from.z ) * ( to.z - from.z ) ) };
            if( length > 0.0 ) { m_edges.push_back( length ); }
        }
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

    // the middle edge length of the sample, and the tile that suits it, rounded to a power of two so
    // that the same scenery always comes out the same way
    if( false == m_edges.empty() ) {
        auto const middle { m_edges.begin() + m_edges.size() / 2 };
        std::nth_element( m_edges.begin(), middle, m_edges.end() );
        auto const wanted { std::clamp( tileedges * *middle, smallesttileside, largesttileside ) };
        m_finesttile = std::exp2( std::round( std::log2( wanted ) ) );
        m_finesttile = std::clamp( m_finesttile, smallesttileside, largesttileside );
        m_edges.clear();
        m_edges.shrink_to_fit();
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
        auto const fromx { std::clamp( first( minx, m_west ), 0, acrossfinest - 1 ) };
        auto const tox { std::clamp( first( maxx, m_west ), 0, acrossfinest - 1 ) };
        auto const fromz { std::clamp( first( minz, m_north ), 0, acrossfinest - 1 ) };
        auto const toz { std::clamp( first( maxz, m_north ), 0, acrossfinest - 1 ) };
        for( auto x = fromx; x <= tox; ++x ) {
            for( auto z = fromz; z <= toz; ++z ) {
                m_placement.emplace_back(
                    tile_key( { finest, x, z } ), static_cast<std::uint32_t>( index ) );
            }
        }
    }
    std::sort( m_placement.begin(), m_placement.end() );

    m_errors.assign( m_levels, 0.0 );
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

    if( Tile.level + 1 < m_levels ) {
        // The finest level is the ground as the scenery drew it and is left alone. A level above it
        // holds what four tiles below it held, and is collapsed back to what one of them held: the
        // cost of a level is then a quarter of the level below, and the whole pyramid costs a third
        // more than the finest alone. A count fixed in advance cannot do that - a tile that never
        // reaches it would be copied rather than simplified - so it only serves as a ceiling.
        double west { 0.0 }, north { 0.0 }, side { 0.0 };
        square( Tile, west, north, side );
        auto const merged { result.indices.size() / 3 };
        simplify( result, std::clamp( merged / 4, smallesttile, m_tilebudget ), side / errorshare );
    }

    m_errors[ Tile.level ] = std::max( m_errors[ Tile.level ], result.error );

    std::vector<std::uint8_t> bytes;
    encode( Tile, result, bytes );
    Sink( Tile, bytes );

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

// Welds vertices that land in the same place and belong to the same material. Everything the cut
// produces is either a vertex of the source or a point on a tile border worked out the same way
// from both sides, so the tolerance only has to cover the last bits of the arithmetic.
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
        std::uint16_t material;
        bool operator==( key const & ) const = default;
    };
    struct keyhash {
        std::size_t operator()( key const &Key ) const noexcept {
            std::size_t hash { 1469598103934665603ull };
            auto const mix = [ &hash ]( std::int64_t const Value ) {
                hash ^= static_cast<std::size_t>( Value );
                hash *= 1099511628211ull; };
            mix( Key.x ); mix( Key.y ); mix( Key.z );
            mix( Key.material );
            return hash;
        }
    };

    static key make_key( cooker::meshvertex const &Vertex ) {
        auto const grid = []( double const Value ) {
            return static_cast<std::int64_t>( std::llround( Value / weldepsilon ) ); };
        return { grid( Vertex.x ), grid( Vertex.y ), grid( Vertex.z ), Vertex.material };
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

    welder weld { result.vertices };
    std::vector<clipvertex> polygon;
    for( auto entry = from; entry != to; ++entry ) {

        m_scratch.seekg( static_cast<std::streamoff>( entry->second ) * sizeof( spilled ) );
        spilled record {};
        m_scratch.read( reinterpret_cast<char *>( &record ), sizeof( record ) );
        if( false == m_scratch.good() ) { m_scratch.clear(); continue; }

        polygon.clear();
        for( std::size_t at = 0; at < 3; ++at ) {
            polygon.push_back(
                { record.corners[ at * 3 + 0 ], record.corners[ at * 3 + 1 ], record.corners[ at * 3 + 2 ] } );
        }
        clip_side( polygon, 0, west, true );
        clip_side( polygon, 0, west + side, false );
        clip_side( polygon, 1, north, true );
        clip_side( polygon, 1, north + side, false );
        if( polygon.size() < 3 ) { continue; }

        // A wall lying along a tile border is inside both tiles: the cut keeps it whole on either
        // side, since it has nothing to cut. Whichever border it sits on, the tile past it is the
        // one that keeps it, so that it is laid exactly once.
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
            return weld( { Vertex.x, Vertex.y, Vertex.z, record.material,
                           sides_of( Vertex.x, Vertex.z, west, north, side ) } ); };

        auto const first { place( polygon[ 0 ] ) };
        for( std::size_t corner = 1; corner + 1 < polygon.size(); ++corner ) {
            // the cut keeps the source winding, so fanning the polygon keeps it too
            auto const second { place( polygon[ corner ] ) };
            auto const third { place( polygon[ corner + 1 ] ) };
            if( ( first == second ) || ( second == third ) || ( third == first ) ) { continue; }
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
cooker::simplify( mesh &Mesh, std::size_t const Budget, double const Allowed ) {

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

    // Where the ground ends - the outline of a hole the scenery left, or the edge of the terrain -
    // only one triangle meets the edge. Nothing in the quadrics above holds such an edge in
    // place, so a plane standing on it, weighted heavily, is added to both its ends: the outline
    // then survives into the coarse levels instead of being eaten away.
    {
        std::map<std::pair<std::uint32_t, std::uint32_t>, std::size_t> uses;
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

    auto const allowed = [ & ]( std::uint32_t const From, std::uint32_t const To ) {
        if( ( true == removed[ From ] ) || ( true == removed[ To ] ) ) { return false; }
        // a vertex a neighbouring tile shares stays where it is
        if( true == vertices[ From ].locked() ) { return false; }
        // and a material boundary stays where it is too, or the ground would change colour by
        // level
        if( vertices[ From ].material != vertices[ To ].material ) { return false; }
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
                    if( ( vertices[ To ].sides & vertices[ third ].sides ) != 0 ) { return false; }
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
    Out.reserve( order.size() * 8 + indices.size() * 2 + sizeof( header ) + 64 );
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

    // ours: which ground material each vertex belongs to. the format has nowhere for it
    append( Out, static_cast<std::uint8_t>( ext_ground ) );
    append( Out, static_cast<std::uint32_t>( order.size() * sizeof( std::uint16_t ) ) );
    for( auto const index : order ) {
        append( Out, Mesh.vertices[ index ].material );
    }
}

} // namespace quantizedmesh
