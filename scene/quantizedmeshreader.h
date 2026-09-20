/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// Reads cooked terrain tiles back, from a scenery's archive or from a directory of loose files.
// A directory is what a tile cooked by hand or by another tool arrives as; the archive is what
// the bake writes.
//
// Everything a tile says about itself is checked before it is believed: a truncated or mangled
// tile is refused rather than handed on, because what comes out of here goes straight into a
// vertex buffer and an index buffer.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "scene/pakformat.h"
#include "scene/quantizedmeshformat.h"

namespace quantizedmesh {

// one vertex as the renderer wants it: measured from the terrain's origin, so that the whole
// terrain fits in floats and every tile of it can go into one vertex buffer. there are no texture
// coordinates - the ground tiles in world space, so the shader works them out from the position
#pragma pack( push, 1 )
struct render_vertex {
    float x, y, z;
    std::uint16_t material;
    std::uint16_t padding;
};
#pragma pack( pop )

static_assert( sizeof( render_vertex ) == 16 );

class reader {

public:
    struct tile_data {
        tile_address tile;
        std::vector<render_vertex> vertices;
        std::vector<std::uint32_t> indices;
        // what the vertices are measured from, in scenery metres
        double originx { 0.0 }, originy { 0.0 }, originz { 0.0 };
        // for culling, in scenery metres
        double centrex { 0.0 }, centrey { 0.0 }, centrez { 0.0 };
        double radius { 0.0 };
        float lowest { 0.f }, highest { 0.f };
    };

    reader() = default;
    ~reader() { close(); }
    reader( reader const & ) = delete;
    reader &operator=( reader const & ) = delete;

    // an archive or a directory. reads the terrain's table, without which nothing can be placed
    bool open( std::string const &Path );
    void close();
    bool ready() const { return m_ready; }

    terrain_table const &table() const { return m_table; }
    std::string const &path() const { return m_path; }

    // Origin is what the vertices come back measured from: one point for the whole terrain, so
    // that every tile can go into the same vertex buffer and be drawn in one call
    bool read_tile( tile_address const &Tile, tile_data &Out,
        double Originx, double Originy, double Originz );
    // the tiles the source holds, by tile_key. a tile the cook found nothing for is not there
    std::unordered_set<std::int64_t> const &present() const { return m_present; }
    bool contains( tile_address const &Tile ) const { return m_present.contains( tile_key( Tile ) ); }

private:
    bool entry( std::string const &Name, std::vector<std::uint8_t> &Out );
    bool read_table();
    void find_tiles();

    std::string m_path;
    bool m_archive { false };
    pak::reader m_pak;
    terrain_table m_table;
    std::unordered_set<std::int64_t> m_present;
    bool m_ready { false };
    std::vector<std::uint8_t> m_buffer;
};

// Implementation.

inline bool
reader::open( std::string const &Path ) {

    close();
    m_path = Path;
    std::error_code error;
    if( true == std::filesystem::is_directory( Path, error ) ) {
        m_archive = false;
    }
    else if( true == std::filesystem::is_regular_file( Path, error ) ) {
        m_archive = true;
        if( false == m_pak.open( Path ) ) { close(); return false; }
    }
    else {
        close();
        return false;
    }
    if( false == read_table() ) { close(); return false; }
    find_tiles();
    m_ready = true;
    return true;
}

inline void
reader::close() {

    m_pak.close();
    m_table = {};
    m_present.clear();
    m_path.clear();
    m_archive = false;
    m_ready = false;
}

inline bool
reader::entry( std::string const &Name, std::vector<std::uint8_t> &Out ) {

    if( true == m_archive ) { return m_pak.read( Name, Out ); }

    std::ifstream file( std::filesystem::path( m_path ) / Name, std::ios::binary );
    if( false == file.good() ) { return false; }
    file.seekg( 0, std::ios::end );
    auto const size { static_cast<std::size_t>( file.tellg() ) };
    file.seekg( 0, std::ios::beg );
    Out.resize( size );
    if( size > 0 ) {
        file.read( reinterpret_cast<char *>( Out.data() ), static_cast<std::streamsize>( size ) );
    }
    return static_cast<std::size_t>( file.gcount() ) == size;
}

// the table is text: a terrain outlives the build that cooked it, and a line of it should be
// readable without a tool
inline bool
reader::read_table() {

    std::vector<std::uint8_t> bytes;
    if( false == entry( tablename, bytes ) ) { return false; }
    std::string const text( bytes.begin(), bytes.end() );

    auto found { false };
    std::size_t at { 0 };
    while( at < text.size() ) {
        auto const end { std::min( text.find( '\n', at ), text.size() ) };
        std::string const line { text.substr( at, end - at ) };
        at = end + 1;
        if( line.empty() ) { continue; }
        auto const space { line.find( ' ' ) };
        std::string const what { line.substr( 0, space ) };
        std::string const rest { space == std::string::npos ? std::string {} : line.substr( space + 1 ) };
        if( what == "eu07-terrain" ) { found = true; }
        else if( what == "rules" )  { m_table.rules = static_cast<std::uint32_t>( std::strtoul( rest.c_str(), nullptr, 10 ) ); }
        else if( what == "levels" ) { m_table.levels = static_cast<std::uint32_t>( std::strtoul( rest.c_str(), nullptr, 10 ) ); }
        else if( what == "square" ) {
            char const *cursor { rest.c_str() };
            m_table.west = std::strtod( cursor, const_cast<char **>( &cursor ) );
            m_table.north = std::strtod( cursor, const_cast<char **>( &cursor ) );
            m_table.side = std::strtod( cursor, const_cast<char **>( &cursor ) );
        }
        else if( what == "height" ) {
            char const *cursor { rest.c_str() };
            m_table.lowest = std::strtod( cursor, const_cast<char **>( &cursor ) );
            m_table.highest = std::strtod( cursor, const_cast<char **>( &cursor ) );
        }
        else if( what == "material" ) {
            // name first, then how many metres one repeat covers. a name can hold spaces, so the
            // number is taken off the end
            auto const gap { rest.rfind( ' ' ) };
            if( gap == std::string::npos ) { m_table.materials.push_back( { rest, 4.f } ); }
            else {
                m_table.materials.push_back(
                    { rest.substr( 0, gap ), static_cast<float>( std::strtod( rest.c_str() + gap + 1, nullptr ) ) } );
            }
        }
        else if( what == "error" ) {
            char const *cursor { rest.c_str() };
            auto const level { std::strtoul( cursor, const_cast<char **>( &cursor ), 10 ) };
            auto const metres { std::strtod( cursor, const_cast<char **>( &cursor ) ) };
            if( m_table.errors.size() <= level ) { m_table.errors.resize( level + 1, 0.f ); }
            m_table.errors[ level ] = static_cast<float>( metres );
        }
    }
    return found
        && ( m_table.rules == cook_rules )
        && ( m_table.side > 0.0 )
        && ( m_table.levels >= 1 );
}

// Which tiles there are is worked out once. Asking the filesystem, or even the archive's index,
// for a tile that is not there is the commonest thing the renderer does - most of a quadtree is
// empty - and it must not cost anything.
inline void
reader::find_tiles() {

    auto const remember = [ this ]( std::string const &Name ) {
        if( false == Name.ends_with( ".terrain" ) ) { return; }
        tile_address tile {};
        if( 3 != std::sscanf( Name.c_str(), "%u/%d/%d.terrain", &tile.level, &tile.x, &tile.z ) ) { return; }
        if( Name != tile_name( tile ) ) { return; }
        m_present.insert( tile_key( tile ) ); };

    if( true == m_archive ) {
        for( auto const &name : m_pak.names() ) { remember( name ); }
        return;
    }
    std::error_code error;
    auto const base { std::filesystem::path( m_path ) };
    for( auto const &found : std::filesystem::recursive_directory_iterator( base, error ) ) {
        if( false == found.is_regular_file( error ) ) { continue; }
        auto const relative { std::filesystem::relative( found.path(), base, error ) };
        if( error ) { continue; }
        remember( relative.generic_string() );
    }
}

inline bool
reader::read_tile( tile_address const &Tile, tile_data &Out,
    double const Originx, double const Originy, double const Originz ) {

    if( false == m_ready ) { return false; }
    if( Tile.level >= m_table.levels ) { return false; }
    if( false == entry( tile_name( Tile ), m_buffer ) ) { return false; }

    auto const *cursor { m_buffer.data() };
    auto const *end { cursor + m_buffer.size() };
    auto const take = [ &cursor, end ]( void *Into, std::size_t const Bytes ) {
        if( static_cast<std::size_t>( end - cursor ) < Bytes ) { return false; }
        std::memcpy( Into, cursor, Bytes );
        cursor += Bytes;
        return true; };

    file_header header {};
    if( false == take( &header, sizeof( header ) ) ) { return false; }

    std::uint32_t vertexcount { 0 };
    if( false == take( &vertexcount, sizeof( vertexcount ) ) ) { return false; }
    if( ( vertexcount < 3 ) || ( vertexcount > ( 1u << 24 ) ) ) { return false; }

    // the three runs of zigzag deltas: x, z, then height
    std::vector<std::uint16_t> quantized[ 3 ];
    for( auto &run : quantized ) {
        run.resize( vertexcount );
        std::int32_t value { 0 };
        for( auto &stored : run ) {
            std::uint16_t delta { 0 };
            if( false == take( &delta, sizeof( delta ) ) ) { return false; }
            value += zigzag_decode( delta );
            if( ( value < 0 ) || ( value > quantum_max ) ) { return false; }
            stored = static_cast<std::uint16_t>( value );
        }
    }

    auto const wide { vertexcount > 65536 };
    auto const indexsize { wide ? 4u : 2u };
    while( ( static_cast<std::size_t>( cursor - m_buffer.data() ) % indexsize ) != 0 ) {
        if( cursor >= end ) { return false; }
        ++cursor;
    }

    std::uint32_t trianglecount { 0 };
    if( false == take( &trianglecount, sizeof( trianglecount ) ) ) { return false; }
    if( ( trianglecount == 0 ) || ( trianglecount > ( 1u << 24 ) ) ) { return false; }

    auto const codes { static_cast<std::size_t>( trianglecount ) * 3 };
    auto decoded { false };
    if( true == wide ) {
        std::vector<std::uint32_t> raw( codes );
        for( auto &code : raw ) { if( false == take( &code, sizeof( code ) ) ) { return false; } }
        decoded = highwater_decode( raw, Out.indices );
    }
    else {
        std::vector<std::uint16_t> raw( codes );
        for( auto &code : raw ) { if( false == take( &code, sizeof( code ) ) ) { return false; } }
        decoded = highwater_decode( raw, Out.indices );
    }
    if( false == decoded ) { return false; }
    for( auto const index : Out.indices ) {
        if( index >= vertexcount ) { return false; }
    }

    // the four lists of border vertices. nothing here needs them - a tile's border survives into
    // every level of it, so neighbours meet whatever level they are drawn at - but they are part
    // of the format and are stepped over
    for( int edge = 0; edge < 4; ++edge ) {
        std::uint32_t count { 0 };
        if( false == take( &count, sizeof( count ) ) ) { return false; }
        if( count > vertexcount ) { return false; }
        if( static_cast<std::size_t>( end - cursor ) < count * indexsize ) { return false; }
        cursor += count * indexsize;
    }

    // which material each vertex belongs to, from the extension the cook writes
    std::vector<std::uint16_t> ground;
    while( cursor < end ) {
        std::uint8_t id { 0 };
        std::uint32_t length { 0 };
        if( false == take( &id, sizeof( id ) ) ) { return false; }
        if( false == take( &length, sizeof( length ) ) ) { return false; }
        if( static_cast<std::size_t>( end - cursor ) < length ) { return false; }
        if( ( id == ext_ground ) && ( length == vertexcount * sizeof( std::uint16_t ) ) ) {
            ground.resize( vertexcount );
            std::memcpy( ground.data(), cursor, length );
        }
        cursor += length;
    }
    if( ground.size() != vertexcount ) { return false; }

    auto const side { m_table.tilesize( Tile.level ) };
    auto const west { m_table.west + Tile.x * side };
    auto const north { m_table.north + Tile.z * side };
    auto const heightspan { std::max( 1e-3, static_cast<double>( header.highest ) - header.lowest ) };

    Out.tile = Tile;
    Out.originx = Originx;
    Out.originy = Originy;
    Out.originz = Originz;
    Out.centrex = header.sphere_x;
    Out.centrey = header.sphere_y;
    Out.centrez = header.sphere_z;
    Out.radius = header.sphere_radius;
    Out.lowest = header.lowest;
    Out.highest = header.highest;

    Out.vertices.resize( vertexcount );
    for( std::uint32_t index = 0; index < vertexcount; ++index ) {
        auto &vertex { Out.vertices[ index ] };
        vertex.x = static_cast<float>( dequantize( quantized[ 0 ][ index ], west, side ) - Out.originx );
        vertex.y = static_cast<float>( dequantize( quantized[ 2 ][ index ], header.lowest, heightspan ) - Out.originy );
        vertex.z = static_cast<float>( dequantize( quantized[ 1 ][ index ], north, side ) - Out.originz );
        vertex.material = ground[ index ];
        vertex.padding = 0;
    }
    return true;
}

// writes the table the reader expects, so that the cook and the reader cannot disagree about it
inline std::vector<std::uint8_t>
write_table( terrain_table const &Table ) {

    auto const number = []( double const Value ) {
        char text[ 64 ];
        std::snprintf( text, sizeof( text ), "%.6f", Value );
        return std::string { text }; };

    std::string text { "eu07-terrain 1\n" };
    text += "rules " + std::to_string( Table.rules ) + "\n";
    text += "square " + number( Table.west ) + " " + number( Table.north ) + " " + number( Table.side ) + "\n";
    text += "height " + number( Table.lowest ) + " " + number( Table.highest ) + "\n";
    text += "levels " + std::to_string( Table.levels ) + "\n";
    for( std::size_t level = 0; level < Table.errors.size(); ++level ) {
        text += "error " + std::to_string( level ) + " " + number( Table.errors[ level ] ) + "\n";
    }
    for( auto const &material : Table.materials ) {
        text += "material " + material.name + " " + number( material.repeat ) + "\n";
    }
    return { text.begin(), text.end() };
}

} // namespace quantizedmesh
