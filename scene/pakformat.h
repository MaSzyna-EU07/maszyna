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
// A terrain is tens of thousands of small files, which is the one thing a filesystem is worst at:
// opening them costs more than reading them. They go into an archive instead, each entry
// compressed on its own so that a tile can be read without touching the rest, and the index is
// written last so that cooking never has to hold the tiles in memory.

#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <zstd.h>

namespace pak {

inline constexpr char magic[ 8 ] { 'E', 'U', '0', '7', 'P', 'A', 'K', '\0' };
inline constexpr std::uint32_t version { 2 };
inline constexpr std::size_t namelength { 64 };

#pragma pack( push, 1 )

struct file_header {
    char magic[ 8 ];
    std::uint32_t version;
    std::uint32_t entrycount;
    std::uint64_t indexoffset;
};

struct index_entry {
    char name[ namelength ];
    std::uint64_t offset;
    std::uint64_t stored;    // bytes in the file
    std::uint64_t plain;     // bytes once decompressed; equal to stored when kept as it is
};

#pragma pack( pop )

// Writes entries straight through to disk as they are added.
class writer {

public:
    bool open( std::string const &Path ) {
        m_file.open( Path, std::ios::binary | std::ios::trunc );
        if( false == m_file.good() ) { return false; }
        file_header header {};
        m_file.write( reinterpret_cast<char const *>( &header ), sizeof( header ) );
        m_at = sizeof( header );
        return m_file.good();
    }

    bool add( std::string const &Name, std::vector<std::uint8_t> const &Data ) {
        if( ( false == m_file.good() ) || ( Name.size() >= namelength ) ) { return false; }
        index_entry entry {};
        std::memcpy( entry.name, Name.c_str(), Name.size() );
        entry.offset = m_at;
        entry.plain = Data.size();

        m_packed.resize( ZSTD_compressBound( Data.size() ) );
        auto const packed {
            ZSTD_compress( m_packed.data(), m_packed.size(), Data.data(), Data.size(), 3 ) };
        auto const *bytes { reinterpret_cast<char const *>( Data.data() ) };
        auto size { Data.size() };
        if( ( 0 == ZSTD_isError( packed ) ) && ( packed < Data.size() ) ) {
            bytes = reinterpret_cast<char const *>( m_packed.data() );
            size = packed;
        }
        entry.stored = size;
        m_file.write( bytes, static_cast<std::streamsize>( size ) );
        m_at += size;
        m_index.push_back( entry );
        return m_file.good();
    }

    bool close() {
        if( false == m_file.good() ) { return false; }
        file_header header {};
        std::memcpy( header.magic, magic, sizeof( magic ) );
        header.version = version;
        header.entrycount = static_cast<std::uint32_t>( m_index.size() );
        header.indexoffset = m_at;
        for( auto const &entry : m_index ) {
            m_file.write( reinterpret_cast<char const *>( &entry ), sizeof( entry ) );
        }
        m_file.seekp( 0 );
        m_file.write( reinterpret_cast<char const *>( &header ), sizeof( header ) );
        auto const good { m_file.good() };
        m_file.close();
        return good;
    }

    std::size_t entrycount() const { return m_index.size(); }
    std::uint64_t bytes() const { return m_at + m_index.size() * sizeof( index_entry ); }

private:
    std::ofstream m_file;
    std::uint64_t m_at { 0 };
    std::vector<index_entry> m_index;
    std::vector<std::uint8_t> m_packed;
};

// Reads entries back. One reader is used from one thread at a time: it keeps a handle and seeks.
class reader {

public:
    bool open( std::string const &Path ) {
        close();
        m_file.open( Path, std::ios::binary );
        if( false == m_file.good() ) { return false; }
        file_header header {};
        m_file.read( reinterpret_cast<char *>( &header ), sizeof( header ) );
        if( false == m_file.good() ) { return false; }
        if( 0 != std::memcmp( header.magic, magic, sizeof( magic ) ) ) { return false; }
        if( header.version != version ) { return false; }
        m_file.seekg( static_cast<std::streamoff>( header.indexoffset ) );
        for( std::uint32_t entry = 0; entry < header.entrycount; ++entry ) {
            index_entry record {};
            m_file.read( reinterpret_cast<char *>( &record ), sizeof( record ) );
            if( false == m_file.good() ) { return false; }
            record.name[ namelength - 1 ] = '\0';
            m_index.emplace( record.name, record );
        }
        return true;
    }

    void close() {
        m_index.clear();
        if( true == m_file.is_open() ) { m_file.close(); }
        m_file.clear();
    }

    bool exists( std::string const &Name ) const { return m_index.contains( Name ); }
    std::size_t entrycount() const { return m_index.size(); }

    std::vector<std::string> names() const {
        std::vector<std::string> names;
        names.reserve( m_index.size() );
        for( auto const &[ name, entry ] : m_index ) { names.push_back( name ); }
        return names;
    }

    bool read( std::string const &Name, std::vector<std::uint8_t> &Out ) {
        auto const found { m_index.find( Name ) };
        if( found == m_index.end() ) { return false; }
        auto const &entry { found->second };
        m_stored.resize( entry.stored );
        m_file.seekg( static_cast<std::streamoff>( entry.offset ) );
        m_file.read( reinterpret_cast<char *>( m_stored.data() ), static_cast<std::streamsize>( entry.stored ) );
        if( false == m_file.good() ) { m_file.clear(); return false; }
        if( entry.stored == entry.plain ) {
            Out.swap( m_stored );
            return true;
        }
        Out.resize( entry.plain );
        auto const plain {
            ZSTD_decompress( Out.data(), Out.size(), m_stored.data(), m_stored.size() ) };
        return ( 0 == ZSTD_isError( plain ) ) && ( plain == entry.plain );
    }

private:
    std::ifstream m_file;
    std::unordered_map<std::string, index_entry> m_index;
    std::vector<std::uint8_t> m_stored;
};

} // namespace pak
