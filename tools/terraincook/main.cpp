/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// Terrain analysis and cooking outside the engine. Reads terrain .scm files, detects the base
// grid, reports how the triangle soup splits into a regular heightfield and an off-grid
// overlay - with greyscale previews, since whether the overlay is a narrow strip along the
// track has to be seen - and cooks, verifies or probes a tiled heightfield. The cooking
// itself is scene/terraincooker.h, shared with the engine's first-run bake.

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "scene/heightfieldformat.h"
#include "scene/heightfieldreader.h"
#include "scene/terraincooker.h"
#include "scene/quantizedmesharchive.h"
#include "scene/quantizedmeshcooker.h"
#include "scene/quantizedmeshreader.h"

namespace {

using terrain::conflictepsilon;
using terrain::on_grid;
using terrain::planar_length;
using terrain::vertex;
using terrain::writepreview;


struct triangle {
    vertex v[ 3 ];
};

struct bounds {
    double minx { 1e30 }, maxx { -1e30 };
    double minz { 1e30 }, maxz { -1e30 };
    void add( double const X, double const Z ) {
        minx = std::min( minx, X ); maxx = std::max( maxx, X );
        minz = std::min( minz, Z ); maxz = std::max( maxz, Z ); }
};

// cell sizes the covered area is measured at. vertices landing on a 1 m grid says
// nothing about how far apart neighbouring samples actually are, and the difference
// decides what a heightfield of this terrain would really cost
constexpr double coveragescales[] { 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 128.0, 256.0, 512.0 };
constexpr std::size_t coveragescalecount { sizeof( coveragescales ) / sizeof( coveragescales[ 0 ] ) };

struct statistics {
    std::size_t triangles { 0 };
    std::size_t gridtriangles { 0 };
    std::size_t vertices { 0 };
    std::size_t gridvertices { 0 };
    std::size_t conflicts { 0 };
    double maxconflict { 0.0 };
    bounds extent;
    bounds overlayextent;
    // triangle edge lengths, bucketed by powers of two: <1, <2, <4, <8, <16, <32, <64, rest
    std::size_t edgebuckets[ 8 ] {};
    double areasum { 0.0 };
};

using heightfield::tile_key;

bool
numeric( char const Character ) {
    return ( Character >= '0' && Character <= '9' ) || Character == '-' || Character == '+' || Character == '.';
}

std::string
readfile( std::filesystem::path const &Path ) {

    std::ifstream file( Path, std::ios::binary );
    if( false == file.good() ) { return {}; }
    file.seekg( 0, std::ios::end );
    auto const size { static_cast<std::size_t>( file.tellg() ) };
    file.seekg( 0, std::ios::beg );
    std::string content;
    content.resize( size );
    file.read( content.data(), static_cast<std::streamsize>( size ) );
    return content;
}

// walks the scenery text and hands every triangle of a triangle-family node to the sink.
// deliberately forgiving: anything that is not a node header or vertex data is skipped
template <typename Sink_>
void
parse( std::string const &Content, Sink_ &&Handler ) {

    auto const *cursor { Content.data() };
    auto const *end { cursor + Content.size() };

    auto skipspace = [ & ]() {
        while( cursor < end && ( static_cast<unsigned char>( *cursor ) <= ' ' ) ) { ++cursor; } };
    auto token = [ & ]() -> std::string_view {
        skipspace();
        auto const *start { cursor };
        while( cursor < end && ( static_cast<unsigned char>( *cursor ) > ' ' ) ) { ++cursor; }
        return { start, static_cast<std::size_t>( cursor - start ) }; };
    auto skipline = [ & ]() {
        while( cursor < end && *cursor != '\n' ) { ++cursor; } };

    std::vector<double> numbers;
    numbers.reserve( 32 );
    auto transformdepth { 0 };
    auto rotated { false };

    while( cursor < end ) {

        auto word { token() };
        if( word.empty() ) { break; }
        if( word.starts_with( "//" ) ) { skipline(); continue; }

        // the same selection the engine bakes with (heightfield::selection_rules): terrain
        // is written in world space, and whatever sits under origin, rotate or scale is an
        // object built from triangles rather than ground
        if( word == "origin" )    { ++transformdepth; continue; }
        if( word == "endorigin" ) { transformdepth = std::max( 0, transformdepth - 1 ); continue; }
        if( word == "scale" )     { ++transformdepth; continue; }
        if( word == "endscale" )  { transformdepth = std::max( 0, transformdepth - 1 ); continue; }
        if( word == "rotate" ) {
            auto const x { token() }, y { token() }, z { token() };
            auto const zero = []( std::string_view const Value ) {
                return std::strtod( std::string( Value ).c_str(), nullptr ) == 0.0; };
            rotated = !( zero( x ) && zero( y ) && zero( z ) );
            continue;
        }
        if( word != "node" ) { continue; }
        if( ( transformdepth > 0 ) || ( true == rotated ) ) {
            // an object, not terrain: step over its body without handing it on
            while( cursor < end ) {
                auto const body { token() };
                if( body.empty() || ( body.starts_with( "end" ) && ( body != "end" ) ) ) { break; }
            }
            continue;
        }

        // node header: range_max range_min name type
        token(); token(); token();
        auto const type { token() };
        if( ( type != "triangles" ) && ( type != "triangle_strip" ) && ( type != "triangle_fan" ) ) {
            continue;
        }
        auto const material { token() };

        numbers.clear();
        while( cursor < end ) {
            auto const value { token() };
            if( value.empty() ) { break; }
            if( value == "endtri" ) { break; }
            if( false == numeric( value.front() ) ) { continue; } // "end" and friends
            double parsed { 0.0 };
            auto const result {
                std::from_chars( value.data(), value.data() + value.size(), parsed ) };
            if( result.ec == std::errc() ) { numbers.push_back( parsed ); }
        }

        // vertex is position, normal and texture coordinates; only the position matters here
        auto const stride { 8u };
        auto const vertexcount { numbers.size() / stride };
        std::vector<vertex> vertices;
        vertices.reserve( vertexcount );
        for( auto index { 0u }; index < vertexcount; ++index ) {
            vertices.push_back( {
                numbers[ index * stride + 0 ],
                numbers[ index * stride + 1 ],
                numbers[ index * stride + 2 ],
                numbers[ index * stride + 6 ],
                numbers[ index * stride + 7 ] } );
        }

        if( type == "triangles" ) {
            for( auto index { 0u }; index + 2 < vertices.size(); index += 3 ) {
                Handler( vertices[ index ], vertices[ index + 1 ], vertices[ index + 2 ], material );
            }
        }
        else if( type == "triangle_strip" ) {
            for( auto index { 2u }; index < vertices.size(); ++index ) {
                Handler( vertices[ index - 2 ], vertices[ index - 1 ], vertices[ index ], material );
            }
        }
        else { // triangle_fan
            for( auto index { 2u }; index < vertices.size(); ++index ) {
                Handler( vertices[ 0 ], vertices[ index - 1 ], vertices[ index ], material );
            }
        }
    }
}






// ---------------------------------------------------------------------------------
// track proximity. the question is whether the fine sampling of the terrain follows the
// railway or is spread evenly: that decides whether a coarse heightfield plus a corridor
// overlay is enough, or whether the base grid has to be fine everywhere

// distance field over a coarse grid, in metres, built from the sampled track centrelines
class distance_field {

public:
    void
        build( std::vector<vertex> const &Samples, double const Cell ) {

            m_cell = Cell;
            if( true == Samples.empty() ) { return; }
            for( auto const &point : Samples ) { m_extent.add( point.x, point.z ); }
            // a margin, so that terrain lying off to the side still gets a real distance
            m_extent.minx -= margin; m_extent.maxx += margin;
            m_extent.minz -= margin; m_extent.maxz += margin;
            m_width = static_cast<std::size_t>( ( m_extent.maxx - m_extent.minx ) / m_cell ) + 1;
            m_height = static_cast<std::size_t>( ( m_extent.maxz - m_extent.minz ) / m_cell ) + 1;
            m_distance.assign( m_width * m_height, unreachable );

            for( auto const &point : Samples ) {
                auto const index { cell( point.x, point.z ) };
                if( index != npos ) { m_distance[ index ] = 0.0f; }
            }
            // two-pass chamfer transform; weights 3/4 approximate euclidean within ~2%
            constexpr float straight { 3.0f }, diagonal { 4.0f };
            auto relax = [ & ]( std::size_t const Target, std::size_t const Source, float const Weight ) {
                m_distance[ Target ] = std::min( m_distance[ Target ], m_distance[ Source ] + Weight ); };
            for( std::size_t y = 0; y < m_height; ++y ) {
                for( std::size_t x = 0; x < m_width; ++x ) {
                    auto const here { y * m_width + x };
                    if( y > 0 ) {
                        relax( here, here - m_width, straight );
                        if( x > 0 ) { relax( here, here - m_width - 1, diagonal ); }
                        if( x + 1 < m_width ) { relax( here, here - m_width + 1, diagonal ); }
                    }
                    if( x > 0 ) { relax( here, here - 1, straight ); }
                }
            }
            for( std::size_t y = m_height; y-- > 0; ) {
                for( std::size_t x = m_width; x-- > 0; ) {
                    auto const here { y * m_width + x };
                    if( y + 1 < m_height ) {
                        relax( here, here + m_width, straight );
                        if( x > 0 ) { relax( here, here + m_width - 1, diagonal ); }
                        if( x + 1 < m_width ) { relax( here, here + m_width + 1, diagonal ); }
                    }
                    if( x + 1 < m_width ) { relax( here, here + 1, straight ); }
                }
            }
        }

    // distance in metres, or a negative value when the point falls outside the field
    double
        at( double const X, double const Z ) const {
            auto const index { cell( X, Z ) };
            if( index == npos ) { return -1.0; }
            auto const value { m_distance[ index ] };
            if( value >= unreachable ) { return -1.0; }
            return value * m_cell / 3.0; }

    bool ready() const { return false == m_distance.empty(); }

private:
    static constexpr std::size_t npos { static_cast<std::size_t>( -1 ) };
    static constexpr float unreachable { 1e9f };
    static constexpr double margin { 4000.0 };

    std::size_t
        cell( double const X, double const Z ) const {
            if( ( X < m_extent.minx ) || ( X > m_extent.maxx )
             || ( Z < m_extent.minz ) || ( Z > m_extent.maxz ) ) { return npos; }
            auto const x { static_cast<std::size_t>( ( X - m_extent.minx ) / m_cell ) };
            auto const z { static_cast<std::size_t>( ( Z - m_extent.minz ) / m_cell ) };
            if( ( x >= m_width ) || ( z >= m_height ) ) { return npos; }
            return z * m_width + x; }

    bounds m_extent;
    double m_cell { 16.0 };
    std::size_t m_width { 0 }, m_height { 0 };
    std::vector<float> m_distance;
};


// reads track nodes and samples their centrelines. the scenery spells a track as a cubic
// bezier: two endpoints with roll, and two control vectors relative to them
std::vector<vertex>
sampletracks( std::vector<std::filesystem::path> const &Paths, double const Spacing ) {

    std::vector<vertex> samples;
    std::size_t nodes { 0 }, rejected { 0 };
    double tracklength { 0.0 };

    for( auto const &path : Paths ) {

        auto const content { readfile( path ) };
        auto const *cursor { content.data() };
        auto const *end { cursor + content.size() };

        auto token = [ & ]() -> std::string_view {
            while( cursor < end && ( static_cast<unsigned char>( *cursor ) <= ' ' ) ) { ++cursor; }
            auto const *start { cursor };
            while( cursor < end && ( static_cast<unsigned char>( *cursor ) > ' ' ) ) { ++cursor; }
            return { start, static_cast<std::size_t>( cursor - start ) }; };
        auto number = [ & ]() -> double {
            auto const word { token() };
            if( ( true == word.empty() ) || ( false == numeric( word.front() ) ) ) { return 0.0; }
            double parsed { 0.0 };
            std::from_chars( word.data(), word.data() + word.size(), parsed );
            return parsed; };

        while( cursor < end ) {

            auto const word { token() };
            if( true == word.empty() ) { break; }
            if( word != "track" ) { continue; }

            // the header has to be walked token by token, in the order TTrack::Load reads
            // it: skipping to "the first numbers after the keyword" lands in the middle of
            // the parameter line and yields nonsense geometry
            token();                                    // type
            for( auto field { 0u }; field < 4u; ++field ) { number(); } // length, gauge, friction, sound distance
            for( auto field { 0u }; field < 2u; ++field ) { number(); } // quality, damage
            token();                                    // environment
            auto const visibility { token() };          // vis / unvis
            if( visibility == "vis" ) {
                token();                                // rail texture
                number();                               // texture tile length
                token();                                // sub or rail texture
                for( auto field { 0u }; field < 3u; ++field ) { number(); } // tex height, width, slope
            }

            vertex p0 {}, c0 {}, c1 {}, p1 {};
            p0.x = number(); p0.y = number(); p0.z = number(); number(); // roll
            c0.x = number(); c0.y = number(); c0.z = number();
            c1.x = number(); c1.y = number(); c1.z = number();
            p1.x = number(); p1.y = number(); p1.z = number(); number(); // roll

            vertex const control0 { p0.x + c0.x, p0.y + c0.y, p0.z + c0.z };
            vertex const control1 { p1.x + c1.x, p1.y + c1.y, p1.z + c1.z };

            auto const chord {
                std::sqrt( ( p1.x - p0.x ) * ( p1.x - p0.x ) + ( p1.z - p0.z ) * ( p1.z - p0.z ) ) };
            ++nodes;
            if( ( chord <= 0.0 ) || ( chord > 5000.0 ) ) { ++rejected; continue; }
            tracklength += chord;
            auto const steps { std::max<std::size_t>( 2u, static_cast<std::size_t>( chord / Spacing ) + 1 ) };

            for( auto step { 0u }; step <= steps; ++step ) {
                auto const t { static_cast<double>( step ) / static_cast<double>( steps ) };
                auto const u { 1.0 - t };
                auto const w0 { u * u * u }, w1 { 3.0 * u * u * t }, w2 { 3.0 * u * t * t }, w3 { t * t * t };
                samples.push_back( {
                    w0 * p0.x + w1 * control0.x + w2 * control1.x + w3 * p1.x,
                    0.0,
                    w0 * p0.z + w1 * control0.z + w2 * control1.z + w3 * p1.z } );
            }
        }
    }
    std::printf( "   track nodes %zu, rejected %zu, total chord length %.1f km\n",
        nodes, rejected, tracklength / 1000.0 );
    return samples;
}



// prints a cross section straight out of a cooked file. a screenshot can show that the
// ground has a cliff in it; only the samples say how tall it is and how many of them it
// takes, which is the difference between a slope that is too steep and one that is right
int
probe( std::filesystem::path const &Path, double const X, double const Z,
       double const Dirx, double const Dirz, std::size_t const Count ) {

    heightfield::reader reader;
    if( false == reader.open( Path.string() ) ) {
        std::printf( "probe: cannot open %s\n", Path.string().c_str() );
        return 1;
    }

    auto const length { std::sqrt( Dirx * Dirx + Dirz * Dirz ) };
    if( length < 1e-9 ) { std::printf( "probe: direction has no length\n" ); return 1; }
    auto const stepx { Dirx / length * reader.gridstep() };
    auto const stepz { Dirz / length * reader.gridstep() };

    std::printf( "-- cross section from %.1f, %.1f along %.2f, %.2f, %zu samples of %.2f m\n",
        X, Z, Dirx / length, Dirz / length, Count, reader.gridstep() );

    std::vector<std::uint16_t> heights;
    std::vector<std::uint8_t> materials;
    auto previous { std::numeric_limits<double>::quiet_NaN() };

    for( std::size_t index = 0; index < Count; ++index ) {

        auto const x { X + stepx * static_cast<double>( index ) };
        auto const z { Z + stepz * static_cast<double>( index ) };
        auto const tilex { reader.tile_x( x ) };
        auto const tilez { reader.tile_z( z ) };
        if( false == reader.read_level( tilex, tilez, 0, heights, materials ) ) {
            std::printf( "   %8.1f %8.1f   no tile\n", x, z );
            continue;
        }
        auto const side { reader.samples_per_side( 0 ) };
        auto const local {
            static_cast<std::int64_t>( std::llround( x / reader.gridstep() ) )
          - static_cast<std::int64_t>( tilex ) * reader.tilesamples() };
        auto const localrow {
            static_cast<std::int64_t>( std::llround( z / reader.gridstep() ) )
          - static_cast<std::int64_t>( tilez ) * reader.tilesamples() };
        if( ( local < 0 ) || ( localrow < 0 ) || ( local >= side ) || ( localrow >= side ) ) {
            std::printf( "   %8.1f %8.1f   outside\n", x, z );
            continue;
        }
        auto const sample { heights[ localrow * side + local ] };
        if( sample == heightfield::nodata ) {
            std::printf( "   %8.1f %8.1f   hole\n", x, z );
            previous = std::numeric_limits<double>::quiet_NaN();
            continue;
        }
        auto const height { reader.height( sample ) };
        auto const material { materials[ localrow * side + local ] };
        std::printf( "   %8.1f %8.1f   %8.3f m   step %+7.3f   %s\n",
            x, z, height, std::isnan( previous ) ? 0.0 : height - previous,
            material < reader.materials().size() ? reader.materials()[ material ].c_str() : "?" );
        previous = height;
    }
    return 0;
}

// reads a cooked file back with the reader the engine uses, and checks the promises the
// format makes. the one that matters is the shared edge: neighbouring tiles must hold
// byte-identical samples along the boundary they share, at every mip level, or the
// terrain will show seams that no amount of skirting hides
int
verify( std::filesystem::path const &Path ) {

    heightfield::reader reader;
    if( false == reader.open( Path.string() ) ) {
        std::printf( "verify: cannot open %s as a heightfield\n", Path.string().c_str() );
        return 1;
    }

    std::printf( "-- verifying %s\n", Path.string().c_str() );
    std::printf( "   tiles %zu, levels %u, grid step %.2f m, tile side %.0f m, materials %zu\n",
        reader.tiles().size(), reader.levels(), reader.gridstep(), reader.tilesize(), reader.materials().size() );

    std::vector<std::uint16_t> heights, neighbourheights;
    std::vector<std::uint8_t> materials, neighbourmaterials;

    std::size_t unreadable { 0 }, seams { 0 }, checkededges { 0 }, holes { 0 }, samples { 0 };

    for( auto const &tile : reader.tiles() ) {
        for( std::uint32_t level = 0; level < reader.levels(); ++level ) {

            if( false == reader.read_level( tile.x, tile.z, level, heights, materials ) ) {
                ++unreadable;
                continue;
            }
            auto const side { reader.samples_per_side( level ) };
            if( heights.size() != static_cast<std::size_t>( side ) * side ) {
                ++unreadable;
                continue;
            }
            if( level == 0 ) {
                samples += heights.size();
                for( auto const value : heights ) {
                    if( value == heightfield::nodata ) { ++holes; }
                }
            }

            // the tile to the east shares this tile's last column with its first
            if( true == reader.contains( tile.x + 1, tile.z ) ) {
                if( true == reader.read_level( tile.x + 1, tile.z, level, neighbourheights, neighbourmaterials ) ) {
                    ++checkededges;
                    for( std::uint32_t row = 0; row < side; ++row ) {
                        if( heights[ row * side + ( side - 1 ) ] != neighbourheights[ row * side ] ) { ++seams; break; }
                    }
                }
                // re-reading this tile, since the reader keeps one decoded payload
                reader.read_level( tile.x, tile.z, level, heights, materials );
            }
            // and the tile to the south shares the last row
            if( true == reader.contains( tile.x, tile.z + 1 ) ) {
                if( true == reader.read_level( tile.x, tile.z + 1, level, neighbourheights, neighbourmaterials ) ) {
                    ++checkededges;
                    for( std::uint32_t column = 0; column < side; ++column ) {
                        if( heights[ ( side - 1 ) * side + column ] != neighbourheights[ column ] ) { ++seams; break; }
                    }
                }
                reader.read_level( tile.x, tile.z, level, heights, materials );
            }
        }
    }

    std::printf( "   unreadable tile levels %zu\n", unreadable );
    std::printf( "   shared edges checked   %zu\n", checkededges );
    std::printf( "   mismatched edges       %zu\n", seams );
    std::printf( "   holes at finest level  %zu of %zu samples (%.2f %%)\n",
        holes, samples, samples == 0 ? 0.0 : 100.0 * static_cast<double>( holes ) / static_cast<double>( samples ) );

    auto const failed { ( unreadable > 0 ) || ( seams > 0 ) };
    std::printf( "   %s\n", failed ? "FAILED" : "ok" );
    return failed ? 1 : 0;
}

} // namespace

int
main( int argc, char *argv[] ) {

    if( argc < 2 ) {
        std::cout
            << "usage: terraincook <terrain .scm files or directory>\n"
            << "         [-step <metres>] [-tracks <dir>] [-out <directory>]\n"
            << "         [-cook] [-tilesize <metres>] [-lodlevels <n>]\n"
            << "  analyses the terrain: grid detection, heightfield/overlay split, sampling\n"
            << "  density against distance from the nearest track, coverage by cell size\n"
            << "  -cook generates TIN tiles (terrain_*.qm) with adaptive triangulation\n"
            << "  -raw leaves the cooked tiles uncompressed\n"
            << "usage: terraincook -verify <terrain_dir>\n"
            << "  verifies TIN tiles in directory\n"
            << "usage: terraincook -probe <terrain_dir> <x,z,lod>\n"
            << "  dumps tile data for inspection\n";
        return 1;
    }

    for( auto index { 1 }; index < argc; ++index ) {
        if( std::string { argv[ index ] } != "-probe" ) { continue; }
        if( index + 2 >= argc ) {
            std::cerr << "-probe needs <terrain.ehf> <x,z,dirx,dirz,count>\n";
            return 1;
        }
        std::vector<double> fields;
        std::istringstream stream { std::string { argv[ index + 2 ] } };
        std::string field;
        while( std::getline( stream, field, ',' ) ) { fields.push_back( std::strtod( field.c_str(), nullptr ) ); }
        if( fields.size() < 5 ) {
            std::cerr << "-probe wants x,z,dirx,dirz,count\n";
            return 1;
        }
        return probe(
            argv[ index + 1 ], fields[ 0 ], fields[ 1 ], fields[ 2 ], fields[ 3 ],
            static_cast<std::size_t>( fields[ 4 ] ) );
    }

    // verification stands on its own: it reads a cooked file and needs no source data
    for( auto index { 1 }; index < argc; ++index ) {
        if( std::string { argv[ index ] } != "-verify" ) { continue; }
        if( index + 1 >= argc ) {
            std::cerr << "-verify needs a path to a cooked heightfield\n";
            return 1;
        }
        return verify( argv[ index + 1 ] );
    }

    std::vector<std::filesystem::path> inputs;
    std::vector<std::filesystem::path> trackinputs;
    std::filesystem::path outputdirectory { "." };
    double step { 0.0 };
    bool cooking { false };
    std::filesystem::path archive { "terrain.pak" };

    for( auto index { 1 }; index < argc; ++index ) {
        std::string const argument { argv[ index ] };
        if( argument == "-step" && index + 1 < argc ) { step = std::strtod( argv[ ++index ], nullptr ); continue; }
        if( argument == "-out" && index + 1 < argc ) { outputdirectory = argv[ ++index ]; continue; }
        if( argument == "-cook" ) { cooking = true; continue; }
        if( argument == "-archive" && index + 1 < argc ) { archive = argv[ ++index ]; continue; }
        if( argument == "-tracks" && index + 1 < argc ) {
            std::filesystem::path const trackpath { argv[ ++index ] };
            if( std::filesystem::is_directory( trackpath ) ) {
                for( auto const &entry : std::filesystem::directory_iterator( trackpath ) ) {
                    if( entry.path().extension() == ".scm" ) { trackinputs.push_back( entry.path() ); }
                }
            }
            else { trackinputs.push_back( trackpath ); }
            continue; }
        std::filesystem::path const path { argument };
        if( std::filesystem::is_directory( path ) ) {
            for( auto const &entry : std::filesystem::directory_iterator( path ) ) {
                if( entry.path().extension() == ".scm" ) { inputs.push_back( entry.path() ); }
            }
        }
        else {
            inputs.push_back( path );
        }
    }
    std::sort( inputs.begin(), inputs.end() );

    if( true == inputs.empty() ) {
        std::cerr << "no input files\n";
        return 1;
    }

    // first pass: sample vertices to pick the grid step, unless one was given
    if( step <= 0.0 ) {
        std::vector<vertex> sample;
        auto const content { readfile( inputs.front() ) };
        parse( content, [ & ]( vertex const &A, vertex const &B, vertex const &C, std::string_view ) {
            if( sample.size() >= 300000 ) { return; }
            sample.push_back( A ); sample.push_back( B ); sample.push_back( C ); } );

        std::cout << "grid step detection, on " << sample.size() << " sampled vertices:\n";
        step = terrain::detect_step(
            sample,
            []( double const Candidate, double const Share ) {
                std::printf( "   %6.2f m  %6.2f %%\n", Candidate, Share ); } );
        std::cout << "   chosen: " << step << " m\n\n";
    }

    // distance-to-track field, when track files were given
    distance_field trackdistance;
    if( false == trackinputs.empty() ) {
        std::sort( trackinputs.begin(), trackinputs.end() );
        auto const samples { sampletracks( trackinputs, 8.0 ) };
        std::cout << "track centreline samples: " << samples.size() << "\n";
        trackdistance.build( samples, 16.0 );
    }
    // triangles bucketed by distance from the nearest track, with their sampling scale
    constexpr double distancebands[] { 25.0, 50.0, 100.0, 200.0, 400.0, 800.0, 1600.0 };
    constexpr std::size_t bandcount { sizeof( distancebands ) / sizeof( distancebands[ 0 ] ) + 2 };
    std::array<std::size_t, bandcount> bandtriangles {};
    std::array<double, bandcount> bandedgesum {};
    std::array<std::size_t, bandcount> bandfine {};

    // the same cooker the engine's first-run cook uses, so the two cannot drift apart
    quantizedmesh::cooker cooker;
    if( cooking ) {
        auto const scratch { ( outputdirectory / "terrain.cook" ).string() };
        std::filesystem::create_directories( outputdirectory );
        if( false == cooker.begin( scratch ) ) {
            std::cerr << "cannot write " << scratch << "\n";
            return 1;
        }
        std::printf( "cooking terrain mesh tiles into %s\n", ( outputdirectory / archive ).string().c_str() );
    }

    statistics stats;
    std::vector<std::unordered_map<std::int64_t, char>> coverage( coveragescalecount );
    std::unordered_map<std::int64_t, float> heights;
    heights.reserve( 8u << 20 );
    std::vector<triangle> overlay;

    for( auto const &path : inputs ) {

        auto const content { readfile( path ) };
        if( true == content.empty() ) {
            std::cerr << "cannot read " << path << "\n";
            continue;
        }
        auto const before { stats.triangles };

        parse( content, [ & ]( vertex const &A, vertex const &B, vertex const &C, std::string_view Material ) {

            if( cooking ) {
                auto const corner = []( vertex const &Vertex ) {
                    return quantizedmesh::cooker::corner {
                        Vertex.x, Vertex.y, Vertex.z,
                        static_cast<float>( Vertex.u ), static_cast<float>( Vertex.v ) }; };
                cooker.add( corner( A ), corner( B ), corner( C ), Material );
            }
            ++stats.triangles;
            stats.vertices += 3;
            vertex const points[ 3 ] { A, B, C };

            auto gridpoints { 0 };
            for( auto const &point : points ) {
                stats.extent.add( point.x, point.z );
                if( on_grid( point.x, step ) && on_grid( point.z, step ) ) { ++gridpoints; }
                for( auto scaleindex { 0u }; scaleindex < coveragescalecount; ++scaleindex ) {
                    auto const cell { coveragescales[ scaleindex ] };
                    coverage[ scaleindex ].emplace(
                        tile_key(
                            static_cast<std::int32_t>( std::floor( point.x / cell ) ),
                            static_cast<std::int32_t>( std::floor( point.z / cell ) ) ),
                        char { 0 } );
                }
            }
            // horizontal edge lengths and footprint area, to see the real sampling density
            for( auto index { 0u }; index < 3u; ++index ) {
                auto const length { planar_length( points[ index ], points[ ( index + 1 ) % 3 ] ) };
                auto bucket { 0u };
                while( ( bucket < 7u ) && ( length >= ( 1.0 * ( 1u << bucket ) ) ) ) { ++bucket; }
                ++stats.edgebuckets[ bucket ];
            }
            if( true == trackdistance.ready() ) {
                auto const centroidx { ( points[ 0 ].x + points[ 1 ].x + points[ 2 ].x ) / 3.0 };
                auto const centroidz { ( points[ 0 ].z + points[ 1 ].z + points[ 2 ].z ) / 3.0 };
                auto edgesum { 0.0 };
                for( auto index { 0u }; index < 3u; ++index ) {
                    edgesum += planar_length( points[ index ], points[ ( index + 1 ) % 3 ] );
                }
                auto const scale { edgesum / 3.0 };
                auto const distance { trackdistance.at( centroidx, centroidz ) };
                std::size_t band { bandcount - 1 }; // outside the field
                if( distance >= 0.0 ) {
                    band = bandcount - 2; // beyond the last threshold
                    for( auto index { 0u }; index < bandcount - 2; ++index ) {
                        if( distance < distancebands[ index ] ) { band = index; break; }
                    }
                }
                ++bandtriangles[ band ];
                bandedgesum[ band ] += scale;
                if( scale < 4.0 ) { ++bandfine[ band ]; }
            }
            stats.areasum += 0.5 * std::abs(
                ( points[ 1 ].x - points[ 0 ].x ) * ( points[ 2 ].z - points[ 0 ].z )
              - ( points[ 2 ].x - points[ 0 ].x ) * ( points[ 1 ].z - points[ 0 ].z ) );
            stats.gridvertices += static_cast<std::size_t>( gridpoints );

            if( gridpoints == 3 ) {
                ++stats.gridtriangles;
                for( auto const &point : points ) {
                    auto const key {
                        tile_key(
                            static_cast<std::int32_t>( std::llround( point.x / step ) ),
                            static_cast<std::int32_t>( std::llround( point.z / step ) ) ) };
                    auto const existing { heights.find( key ) };
                    if( existing == heights.end() ) {
                        heights.emplace( key, static_cast<float>( point.y ) );
                    }
                    else {
                        auto const difference { std::abs( static_cast<double>( existing->second ) - point.y ) };
                        if( difference > conflictepsilon ) {
                            ++stats.conflicts;
                            stats.maxconflict = std::max( stats.maxconflict, difference );
                        }
                    }
                }
            }
            else {
                overlay.push_back( { { A, B, C } } );
                for( auto const &point : points ) { stats.overlayextent.add( point.x, point.z ); }
            }
        } );

        std::printf( "%-28s %10zu triangles\n",
            path.filename().string().c_str(), stats.triangles - before );
    }

    auto const share = []( std::size_t const Part, std::size_t const Whole ) {
        return Whole == 0 ? 0.0 : 100.0 * static_cast<double>( Part ) / static_cast<double>( Whole ); };

    // the full bounding box is set by outliers, so derive the working extent from the
    // occupied grid samples themselves, trimming the sparsest 0.1% at each end
    std::vector<std::int32_t> columns, rows;
    columns.reserve( heights.size() );
    rows.reserve( heights.size() );
    for( auto const & [ key, height ] : heights ) {
        columns.push_back( static_cast<std::int32_t>( key >> 32 ) );
        rows.push_back( static_cast<std::int32_t>( static_cast<std::uint32_t>( key & 0xffffffff ) ) );
    }
    auto trimmed = []( std::vector<std::int32_t> &Values ) -> std::pair<double, double> {
        if( true == Values.empty() ) { return { 0.0, 0.0 }; }
        auto const low { Values.size() / 1000 };
        auto const high { Values.size() - 1 - low };
        std::nth_element( Values.begin(), Values.begin() + static_cast<std::ptrdiff_t>( low ), Values.end() );
        auto const minimum { Values[ low ] };
        std::nth_element( Values.begin(), Values.begin() + static_cast<std::ptrdiff_t>( high ), Values.end() );
        auto const maximum { Values[ high ] };
        return { static_cast<double>( minimum ), static_cast<double>( maximum ) }; };
    auto const [ columnmin, columnmax ] { trimmed( columns ) };
    auto const [ rowmin, rowmax ] { trimmed( rows ) };

    auto const gridwidth { static_cast<std::size_t>( columnmax - columnmin ) + 1 };
    auto const gridheight { static_cast<std::size_t>( rowmax - rowmin ) + 1 };
    auto const overlaytriangles { stats.triangles - stats.gridtriangles };

    std::printf( "\n-- terrain decomposition, grid step %.2f m\n", step );
    std::printf( "   triangles total      %12zu\n", stats.triangles );
    std::printf( "   on grid (heightfield)%12zu  %6.2f %%\n", stats.gridtriangles, share( stats.gridtriangles, stats.triangles ) );
    std::printf( "   off grid (overlay)   %12zu  %6.2f %%\n", overlaytriangles, share( overlaytriangles, stats.triangles ) );
    std::printf( "   vertices on grid     %12zu  %6.2f %%\n", stats.gridvertices, share( stats.gridvertices, stats.vertices ) );
    std::printf( "   distinct grid samples%12zu\n", heights.size() );
    std::printf( "   stacked samples      %12zu  (max height difference %.2f m)\n", stats.conflicts, stats.maxconflict );
    std::printf( "   full bounding box    %.0f .. %.0f x  %.0f .. %.0f z\n",
        stats.extent.minx, stats.extent.maxx, stats.extent.minz, stats.extent.maxz );
    std::printf( "   working extent       %.0f .. %.0f x  %.0f .. %.0f z  (%zu x %zu grid)\n",
        columnmin * step, columnmax * step, rowmin * step, rowmax * step, gridwidth, gridheight );
    std::printf( "   covered area         %12.2f km2  (%.1f %% of the working extent)\n",
        heights.size() * step * step / 1.0e6,
        share( heights.size(), gridwidth * gridheight ) );
    if( overlaytriangles > 0 ) {
        std::printf( "   overlay extent       %.0f .. %.0f x  %.0f .. %.0f z\n",
            stats.overlayextent.minx, stats.overlayextent.maxx,
            stats.overlayextent.minz, stats.overlayextent.maxz );
    }

    std::printf( "\n-- real sampling density\n" );
    std::printf( "   summed triangle footprint %8.2f km2\n", stats.areasum / 1.0e6 );
    std::printf( "   mean triangle footprint   %8.2f m2\n",
        stats.triangles == 0 ? 0.0 : stats.areasum / static_cast<double>( stats.triangles ) );
    char const *edgelabels[ 8 ] { "<1 m", "<2 m", "<4 m", "<8 m", "<16 m", "<32 m", "<64 m", ">=64 m" };
    std::size_t edgetotal { 0 };
    for( auto const bucket : stats.edgebuckets ) { edgetotal += bucket; }
    for( auto index { 0u }; index < 8u; ++index ) {
        if( stats.edgebuckets[ index ] == 0 ) { continue; }
        std::printf( "   edges %-7s        %12zu  %6.2f %%\n",
            edgelabels[ index ], stats.edgebuckets[ index ], share( stats.edgebuckets[ index ], edgetotal ) );
    }

    if( true == trackdistance.ready() ) {
        std::printf( "\n-- sampling density against distance from the nearest track\n" );
        std::printf( "   %-14s %12s %12s %10s\n", "band", "triangles", "mean edge", "fine <4 m" );
        for( auto index { 0u }; index < bandcount; ++index ) {
            if( bandtriangles[ index ] == 0 ) { continue; }
            char label[ 32 ];
            if( index < bandcount - 2 ) {
                std::snprintf( label, sizeof( label ), "< %.0f m", distancebands[ index ] );
            }
            else if( index == bandcount - 2 ) {
                std::snprintf( label, sizeof( label ), ">= %.0f m", distancebands[ bandcount - 3 ] );
            }
            else {
                std::snprintf( label, sizeof( label ), "off the field" );
            }
            std::printf( "   %-14s %12zu %10.2f m %9.2f %%\n",
                label, bandtriangles[ index ],
                bandedgesum[ index ] / static_cast<double>( bandtriangles[ index ] ),
                share( bandfine[ index ], bandtriangles[ index ] ) );
        }
    }

    std::printf( "\n-- covered area by cell size, and what a heightfield at that resolution costs\n" );
    for( auto scaleindex { 0u }; scaleindex < coveragescalecount; ++scaleindex ) {
        auto const cell { coveragescales[ scaleindex ] };
        auto const cells { coverage[ scaleindex ].size() };
        std::printf( "   %5.0f m  %12zu cells  %9.2f km2  %9.1f MB R16\n",
            cell, cells, cells * cell * cell / 1.0e6, cells * 2u / 1048576.0 );
    }

    auto const heightfieldbytes { gridwidth * gridheight * 2u };
    auto const overlaybytes { overlaytriangles * 3u * 20u };
    std::printf( "\n-- storage estimate\n" );
    std::printf( "   dense heightfield R16%12.1f MB  (whole bounding box)\n", heightfieldbytes / 1048576.0 );
    std::printf( "   occupied samples R16 %12.1f MB  (sparse, what is actually covered)\n", heights.size() * 2u / 1048576.0 );
    std::printf( "   overlay at 20 B/vertex%11.1f MB\n", overlaybytes / 1048576.0 );

    // previews, downsampled so that the wider side is at most 2048 pixels
    std::filesystem::create_directories( outputdirectory );
    auto const preview { std::max<std::size_t>( 1u, ( std::max( gridwidth, gridheight ) + 2047u ) / 2048u ) };
    auto const previewwidth { gridwidth / preview + 1 };
    auto const previewheight { gridheight / preview + 1 };
    constexpr float sentinel { -1e30f };

    std::vector<float> heightpreview( previewwidth * previewheight, sentinel );
    for( auto const & [ key, height ] : heights ) {
        auto const x { static_cast<std::int32_t>( key >> 32 ) };
        auto const z { static_cast<std::int32_t>( static_cast<std::uint32_t>( key & 0xffffffff ) ) };
        if( ( x < columnmin ) || ( x > columnmax ) || ( z < rowmin ) || ( z > rowmax ) ) { continue; }
        auto const column { static_cast<std::size_t>( x - columnmin ) / preview };
        auto const row { static_cast<std::size_t>( z - rowmin ) / preview };
        if( column >= previewwidth || row >= previewheight ) { continue; }
        auto &target { heightpreview[ row * previewwidth + column ] };
        target = ( target == sentinel ? height : std::max( target, height ) );
    }
    writepreview( outputdirectory / "height.pgm", heightpreview, previewwidth, previewheight, sentinel );

    std::vector<float> overlaypreview( previewwidth * previewheight, 0.0f );
    for( auto const &piece : overlay ) {
        for( auto const &point : piece.v ) {
            auto const gridx { std::llround( point.x / step ) };
            auto const gridz { std::llround( point.z / step ) };
            if( ( gridx < columnmin ) || ( gridx > columnmax ) || ( gridz < rowmin ) || ( gridz > rowmax ) ) { continue; }
            auto const column { static_cast<std::size_t>( gridx - columnmin ) / preview };
            auto const row { static_cast<std::size_t>( gridz - rowmin ) / preview };
            if( column >= previewwidth || row >= previewheight ) { continue; }
            overlaypreview[ row * previewwidth + column ] += 1.0f;
        }
    }
    writepreview( outputdirectory / "overlay.pgm", overlaypreview, previewwidth, previewheight, -1.0f );

    if( cooking ) {
        std::printf( "\n-- cooking terrain mesh tiles\n" );

        quantizedmesh::archive_writer writer;
        auto const archivepath { ( outputdirectory / archive ).string() };
        if( false == writer.open( archivepath, quantizedmesh::cook_rules ) ) {
            std::cerr << "cannot write " << archivepath << "\n";
            return 1;
        }
        auto written { true };
        quantizedmesh::terrain_table table;
        auto const sink {
            [ &writer, &written ](
                quantizedmesh::tile_address const &Tile, std::vector<std::uint8_t> const &Bytes,
                quantizedmesh::tile_measure const &Measure ) {
                written = writer.add( Tile, Bytes, Measure ) && written; } };
        if( ( false == cooker.finish( sink, table ) ) || ( false == written ) ) {
            std::cerr << "cooking failed\n";
            return 1;
        }
        written = writer.table( quantizedmesh::write_table( table ) ) && written;
        if( ( false == writer.close() ) || ( false == written ) ) {
            std::cerr << "cannot finish " << archivepath << "\n";
            return 1;
        }

        auto const &report { cooker.result() };
        std::printf( "   %zu triangles laid in %zu tiles of %.0f m over %u levels\n",
            report.triangles, report.tiles, cooker.finesttile(), report.levels );
        std::printf( "   %zu vertices, %llu kB, %zu triangles refused\n",
            report.vertices, static_cast<unsigned long long>( writer.bytes() / 1024 ), report.refused );
        for( std::uint32_t level = 0; level < table.levels; ++level ) {
            std::printf( "   level %u: tile %.0f m, off by %.3f m\n",
                level, table.tilesize( level ), table.error( level ) );
        }
        for( auto const &material : table.materials ) {
            std::printf( "   material %-40s one repeat every %6.2f m\n", material.name.c_str(), material.repeat );
        }
    }

    std::printf( "\n   previews written to %s (%zux%zu, 1 px = %.0f m)\n",
        outputdirectory.string().c_str(), previewwidth, previewheight, step * preview );

    return 0;
}
