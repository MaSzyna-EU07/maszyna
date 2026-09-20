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
#include <vector>

#include "scene/quantizedmesharchive.h"
#include "scene/quantizedmeshformat.h"

namespace quantizedmesh {

// one vertex as the renderer wants it: measured from the terrain's origin, so that the whole
// terrain fits in floats and every tile of it can go into one vertex buffer. there are no texture
// coordinates - the ground tiles in world space, so the shader works them out from the position -
// and the normal stays octahedron-encoded, which the shader unfolds. sixteen bytes either way
#pragma pack( push, 1 )
struct render_vertex {
    float x, y, z;
    std::uint16_t material;
    std::uint8_t normal0, normal1;
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
    // whether the source holds this tile. a tile the cook found no ground for is not there, and most
    // of a quadtree is like that, so this is the commonest question the renderer asks
    bool contains( tile_address const &Tile ) const;
    // What this tile's ground asks for. An archive carries it per tile; a directory of loose tiles
    // has nowhere to keep it, so there the level's figure stands in, which is what the whole terrain
    // used before the index carried anything better
    tile_measure measure( tile_address const &Tile ) const;
    std::size_t tilecount() const;

private:
    bool loose_file( std::string const &Name, std::vector<std::uint8_t> &Out ) const;
    bool read_table();
    void find_loose_tiles();

    std::string m_path;
    bool m_isarchive { false };
    archive_reader m_archive;
    terrain_table m_table;
    // only for a directory of loose tiles; an archive is its own index
    std::vector<std::int64_t> m_loose;
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
        m_isarchive = false;
    }
    else if( true == std::filesystem::is_regular_file( Path, error ) ) {
        m_isarchive = true;
        if( false == m_archive.open( Path, cook_rules ) ) { close(); return false; }
    }
    else {
        close();
        return false;
    }
    if( false == read_table() ) { close(); return false; }
    if( false == m_isarchive ) { find_loose_tiles(); }
    m_ready = true;
    return true;
}

inline void
reader::close() {

    m_archive.close();
    m_table = {};
    m_loose.clear();
    m_loose.shrink_to_fit();
    m_path.clear();
    m_isarchive = false;
    m_ready = false;
}

inline bool
reader::contains( tile_address const &Tile ) const {

    if( true == m_isarchive ) { return m_archive.contains( Tile ); }
    return std::binary_search( m_loose.begin(), m_loose.end(), tile_key( Tile ) );
}

inline tile_measure
reader::measure( tile_address const &Tile ) const {

    tile_measure measure { m_table.error( Tile.level ), m_table.edge( Tile.level ) };
    if( true == m_isarchive ) { m_archive.measure( Tile, measure ); }
    return measure;
}

inline std::size_t
reader::tilecount() const {

    return m_isarchive ? m_archive.tilecount() : m_loose.size();
}

inline bool
reader::loose_file( std::string const &Name, std::vector<std::uint8_t> &Out ) const {

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

// Which tiles a directory holds is worked out once. Asking the filesystem for a tile that is not there
// is the commonest thing the renderer does - most of a quadtree is empty - and it must not cost a
// system call.
inline void
reader::find_loose_tiles() {

    std::error_code error;
    auto const base { std::filesystem::path( m_path ) };
    for( auto const &found : std::filesystem::recursive_directory_iterator( base, error ) ) {
        if( false == found.is_regular_file( error ) ) { continue; }
        auto const relative { std::filesystem::relative( found.path(), base, error ) };
        if( error ) { continue; }
        auto const name { relative.generic_string() };
        if( false == name.ends_with( ".terrain" ) ) { continue; }
        tile_address tile {};
        if( 3 != std::sscanf( name.c_str(), "%u/%d/%d.terrain", &tile.level, &tile.x, &tile.z ) ) { continue; }
        if( name != tile_name( tile ) ) { continue; }
        m_loose.push_back( tile_key( tile ) );
    }
    std::sort( m_loose.begin(), m_loose.end() );
}

// the table is text: a terrain outlives the build that cooked it, and a line of it should be
// readable without a tool
inline bool
reader::read_table() {

    std::vector<std::uint8_t> bytes;
    auto const got {
        m_isarchive ? m_archive.read_table( bytes ) : loose_file( tablename, bytes ) };
    if( false == got ) { return false; }
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
        else if( what == "edge" ) {
            char const *cursor { rest.c_str() };
            auto const level { std::strtoul( cursor, const_cast<char **>( &cursor ), 10 ) };
            auto const metres { std::strtod( cursor, const_cast<char **>( &cursor ) ) };
            if( m_table.edges.size() <= level ) { m_table.edges.resize( level + 1, 0.f ); }
            m_table.edges[ level ] = static_cast<float>( metres );
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

inline bool
reader::read_tile( tile_address const &Tile, tile_data &Out,
    double const Originx, double const Originy, double const Originz ) {

    if( false == m_ready ) { return false; }
    if( Tile.level >= m_table.levels ) { return false; }
    auto const got {
        m_isarchive ? m_archive.read_tile( Tile, m_buffer ) : loose_file( tile_name( Tile ), m_buffer ) };
    if( false == got ) { return false; }

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

    // the normals the format carries, and which material each vertex belongs to, which is ours
    std::vector<std::uint16_t> ground;
    std::vector<std::uint8_t> normals;
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
        else if( ( id == ext_normals ) && ( length == vertexcount * 2u ) ) {
            normals.resize( length );
            std::memcpy( normals.data(), cursor, length );
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
        // a tile without the normals extension - one made by another tool - is lit as if the ground
        // were level, which is wrong but is not a reason to refuse it
        vertex.normal0 = ( normals.size() == vertexcount * 2u ) ? normals[ index * 2 + 0 ] : 128;
        vertex.normal1 = ( normals.size() == vertexcount * 2u ) ? normals[ index * 2 + 1 ] : 128;
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
    for( std::size_t level = 0; level < Table.edges.size(); ++level ) {
        text += "edge " + std::to_string( level ) + " " + number( Table.edges[ level ] ) + "\n";
    }
    for( auto const &material : Table.materials ) {
        text += "material " + material.name + " " + number( material.repeat ) + "\n";
    }
    return { text.begin(), text.end() };
}

} // namespace quantizedmesh
