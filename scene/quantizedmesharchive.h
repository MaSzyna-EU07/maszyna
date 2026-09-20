/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// One file holding a scenery's cooked terrain tiles.
//
// A terrain is hundreds of thousands of small files, which is the one thing a filesystem is worst at:
// opening them costs more than reading them. They go into an archive instead, each entry compressed on
// its own so that a tile can be read without touching the rest, and the index written last so that
// cooking never has to hold the tiles in memory.
//
// A tile is found by where it sits in the quadtree, not by a name. Galicja cooks to three quarters of
// a million tiles: a name of its own for each would be seventy megabytes of index to read and a hash
// table of that many strings to build before the first tile can be drawn. The index here is a sorted
// run of thirty-two byte entries, read in one go and searched in place.
//
// An entry carries what the tile's ground asks for as well as where it lies, because which level to
// draw has to be settled for tiles that are not on the card yet - that is the whole of the question -
// and the index is the only thing about them that is read up front.

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <zstd.h>

#include "scene/quantizedmeshformat.h"

namespace quantizedmesh {

inline constexpr char archive_magic[ 8 ] { 'E', 'U', '0', '7', 'T', 'A', 'R', '\0' };
inline constexpr std::uint32_t archive_version { 2 };

#pragma pack( push, 1 )

struct archive_header {
    char magic[ 8 ];
    std::uint32_t version;
    // the cook's rules, so that an archive this build cannot make sense of is refused at the door
    std::uint32_t rules;
    std::uint64_t entrycount;
    std::uint64_t indexoffset;
    // the terrain's table, which every tile needs before it can be placed
    std::uint64_t tableoffset;
    std::uint32_t tablestored;
    std::uint32_t tableplain;
};

struct archive_entry {
    std::int64_t key;          // tile_key of the tile this holds
    std::uint64_t offset;
    std::uint32_t stored;      // bytes in the file
    std::uint32_t plain;       // bytes once decompressed; equal to stored when kept as it is
    // what this tile's own ground asks for, as the cook measured it
    float error;
    float edge;

    bool operator<( archive_entry const &Other ) const { return key < Other.key; }
};

#pragma pack( pop )

static_assert( sizeof( archive_entry ) == 32 );

namespace detail {

// zstd, or the bytes as they are when compressing them does not pay
inline void
pack( std::vector<std::uint8_t> const &Data, std::vector<std::uint8_t> &Room,
    std::uint8_t const *&Bytes, std::size_t &Size ) {

    Bytes = Data.data();
    Size = Data.size();
    if( true == Data.empty() ) { return; }
    Room.resize( ZSTD_compressBound( Data.size() ) );
    auto const packed { ZSTD_compress( Room.data(), Room.size(), Data.data(), Data.size(), 3 ) };
    if( ( 0 == ZSTD_isError( packed ) ) && ( packed < Data.size() ) ) {
        Bytes = Room.data();
        Size = packed;
    }
}

} // namespace detail

// Writes tiles straight through to disk as they are cooked.
class archive_writer {

public:
    bool open( std::string const &Path, std::uint32_t const Rules ) {
        m_rules = Rules;
        m_file.open( Path, std::ios::binary | std::ios::trunc );
        if( false == m_file.good() ) { return false; }
        archive_header header {};
        m_file.write( reinterpret_cast<char const *>( &header ), sizeof( header ) );
        m_at = sizeof( header );
        return m_file.good();
    }

    bool add( tile_address const &Tile, std::vector<std::uint8_t> const &Data,
        tile_measure const &Measure ) {
        if( false == m_file.good() ) { return false; }
        std::uint8_t const *bytes { nullptr };
        std::size_t size { 0 };
        detail::pack( Data, m_room, bytes, size );
        archive_entry entry {};
        entry.key = tile_key( Tile );
        entry.offset = m_at;
        entry.stored = static_cast<std::uint32_t>( size );
        entry.plain = static_cast<std::uint32_t>( Data.size() );
        entry.error = Measure.error;
        entry.edge = Measure.edge;
        m_file.write( reinterpret_cast<char const *>( bytes ), static_cast<std::streamsize>( size ) );
        m_at += size;
        m_index.push_back( entry );
        return m_file.good();
    }

    // the terrain's table, written once. it has a place of its own in the header rather than an entry
    bool table( std::vector<std::uint8_t> const &Data ) {
        if( false == m_file.good() ) { return false; }
        std::uint8_t const *bytes { nullptr };
        std::size_t size { 0 };
        detail::pack( Data, m_room, bytes, size );
        m_tableoffset = m_at;
        m_tablestored = static_cast<std::uint32_t>( size );
        m_tableplain = static_cast<std::uint32_t>( Data.size() );
        m_file.write( reinterpret_cast<char const *>( bytes ), static_cast<std::streamsize>( size ) );
        m_at += size;
        return m_file.good();
    }

    bool close() {
        if( false == m_file.good() ) { return false; }
        // in order, so that the index can be searched where it lies and so that cooking the same
        // scenery twice writes the same file
        std::sort( m_index.begin(), m_index.end() );

        archive_header header {};
        std::memcpy( header.magic, archive_magic, sizeof( archive_magic ) );
        header.version = archive_version;
        header.rules = m_rules;
        header.entrycount = m_index.size();
        header.indexoffset = m_at;
        header.tableoffset = m_tableoffset;
        header.tablestored = m_tablestored;
        header.tableplain = m_tableplain;
        for( auto const &entry : m_index ) {
            m_file.write( reinterpret_cast<char const *>( &entry ), sizeof( entry ) );
        }
        m_file.seekp( 0 );
        m_file.write( reinterpret_cast<char const *>( &header ), sizeof( header ) );
        auto const good { m_file.good() };
        m_file.close();
        return good;
    }

    std::size_t tilecount() const { return m_index.size(); }
    std::uint64_t bytes() const { return m_at + m_index.size() * sizeof( archive_entry ); }

private:
    std::ofstream m_file;
    std::uint64_t m_at { 0 };
    std::uint32_t m_rules { 0 };
    std::uint64_t m_tableoffset { 0 };
    std::uint32_t m_tablestored { 0 };
    std::uint32_t m_tableplain { 0 };
    std::vector<archive_entry> m_index;
    std::vector<std::uint8_t> m_room;
};

// Reads tiles back. One reader is used from one thread at a time: it keeps a handle and seeks.
class archive_reader {

public:
    bool open( std::string const &Path, std::uint32_t const Rules ) {
        close();
        m_file.open( Path, std::ios::binary );
        if( false == m_file.good() ) { return false; }
        archive_header header {};
        m_file.read( reinterpret_cast<char *>( &header ), sizeof( header ) );
        if( false == m_file.good() ) { return false; }
        if( 0 != std::memcmp( header.magic, archive_magic, sizeof( archive_magic ) ) ) { return false; }
        if( header.version != archive_version ) { return false; }
        if( header.rules != Rules ) { return false; }
        if( header.entrycount > ( 1ull << 32 ) ) { return false; }

        m_index.resize( static_cast<std::size_t>( header.entrycount ) );
        if( false == m_index.empty() ) {
            m_file.seekg( static_cast<std::streamoff>( header.indexoffset ) );
            m_file.read(
                reinterpret_cast<char *>( m_index.data() ),
                static_cast<std::streamsize>( m_index.size() * sizeof( archive_entry ) ) );
            if( false == m_file.good() ) { close(); return false; }
            // written in order, but a file from anywhere is not to be believed about that
            if( false == std::is_sorted( m_index.begin(), m_index.end() ) ) {
                std::sort( m_index.begin(), m_index.end() );
            }
        }
        m_tableoffset = header.tableoffset;
        m_tablestored = header.tablestored;
        m_tableplain = header.tableplain;
        m_open = true;
        return true;
    }

    void close() {
        m_index.clear();
        m_index.shrink_to_fit();
        if( true == m_file.is_open() ) { m_file.close(); }
        m_file.clear();
        m_open = false;
    }

    bool contains( tile_address const &Tile ) const { return nullptr != find( tile_key( Tile ) ); }
    // what the tile asks for, or nothing when the archive does not hold it
    bool measure( tile_address const &Tile, tile_measure &Out ) const {
        auto const *entry { find( tile_key( Tile ) ) };
        if( nullptr == entry ) { return false; }
        Out.error = entry->error;
        Out.edge = entry->edge;
        return true;
    }
    std::size_t tilecount() const { return m_index.size(); }
    // the tiles there are, in order, for anything that wants to walk them
    std::vector<archive_entry> const &index() const { return m_index; }

    bool read_table( std::vector<std::uint8_t> &Out ) {
        if( ( false == m_open ) || ( m_tableplain == 0 ) ) { return false; }
        return fetch( m_tableoffset, m_tablestored, m_tableplain, Out );
    }

    bool read_tile( tile_address const &Tile, std::vector<std::uint8_t> &Out ) {
        if( false == m_open ) { return false; }
        auto const *entry { find( tile_key( Tile ) ) };
        if( nullptr == entry ) { return false; }
        return fetch( entry->offset, entry->stored, entry->plain, Out );
    }

private:
    archive_entry const *find( std::int64_t const Key ) const {
        archive_entry wanted {};
        wanted.key = Key;
        auto const found { std::lower_bound( m_index.begin(), m_index.end(), wanted ) };
        if( ( found == m_index.end() ) || ( found->key != Key ) ) { return nullptr; }
        return &( *found );
    }

    bool fetch( std::uint64_t const Offset, std::uint32_t const Stored, std::uint32_t const Plain,
        std::vector<std::uint8_t> &Out ) {

        m_stored.resize( Stored );
        m_file.seekg( static_cast<std::streamoff>( Offset ) );
        m_file.read( reinterpret_cast<char *>( m_stored.data() ), static_cast<std::streamsize>( Stored ) );
        if( false == m_file.good() ) { m_file.clear(); return false; }
        if( Stored == Plain ) {
            Out = m_stored;
            return true;
        }
        Out.resize( Plain );
        auto const plain { ZSTD_decompress( Out.data(), Out.size(), m_stored.data(), m_stored.size() ) };
        return ( 0 == ZSTD_isError( plain ) ) && ( plain == Plain );
    }

    std::ifstream m_file;
    std::vector<archive_entry> m_index;
    std::vector<std::uint8_t> m_stored;
    std::uint64_t m_tableoffset { 0 };
    std::uint32_t m_tablestored { 0 };
    std::uint32_t m_tableplain { 0 };
    bool m_open { false };
};

} // namespace quantizedmesh
