/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/scenerybinary.h"

#include "scene/sn_utils.h"
#include "utilities/utilities.h" // ToLower
#include "utilities/Logs.h"

#include <unordered_map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>
#include <fstream>

namespace scene {

namespace {

// LEB128 unsigned varint: 1 byte for values < 128 (covers most string-table indices),
// keeping keyword/index references compact.
void write_varint( std::ostream &Output, std::uint64_t Value ) {
    do {
        std::uint8_t byte = Value & 0x7F;
        Value >>= 7;
        if( Value != 0 ) { byte |= 0x80; }
        Output.put( static_cast<char>( byte ) );
    } while( Value != 0 );
}

std::uint64_t read_varint( std::istream &Input ) {
    std::uint64_t value = 0;
    int shift = 0;
    while( shift < 64 ) {
        int const c = Input.get();
        if( c == std::char_traits<char>::eof() ) { break; }
        value |= ( static_cast<std::uint64_t>( c & 0x7F ) << shift );
        if( ( c & 0x80 ) == 0 ) { break; }
        shift += 7;
    }
    return value;
}

// builds the per-file string table while interning, returning each string's index
class string_interner {
public:
    std::uint32_t intern( std::string const &Text ) {
        auto const it = m_lookup.find( Text );
        if( it != m_lookup.end() ) { return it->second; }
        auto const index = static_cast<std::uint32_t>( m_table.size() );
        m_lookup.emplace( Text, index );
        m_table.emplace_back( Text );
        return index;
    }
    std::vector<std::string> const &table() const { return m_table; }
private:
    std::unordered_map<std::string, std::uint32_t> m_lookup;
    std::vector<std::string> m_table;
};

} // anonymous namespace

void
scenery_binary_writer::add_token( std::string Token ) {
    scenery_entry entry;
    entry.type = scenery_entry_type::token;
    entry.text = std::move( Token );
    m_entries.emplace_back( std::move( entry ) );
}

void
scenery_binary_writer::add_number( double Value ) {
    scenery_entry entry;
    entry.type = scenery_entry_type::number;
    entry.number = Value;
    m_entries.emplace_back( std::move( entry ) );
}

void
scenery_binary_writer::add_include( std::vector<std::string> Fileexpr, std::vector<std::string> Params ) {
    scenery_entry entry;
    entry.type = scenery_entry_type::include;
    entry.fileexpr = std::move( Fileexpr );
    entry.params = std::move( Params );
    m_entries.emplace_back( std::move( entry ) );
}

bool
scenery_binary_writer::write( std::ostream &Output, scenery_file_kind Kind ) const {

    // first pass: intern every string the entries reference so keywords/paths that
    // repeat are stored once and referenced by index
    string_interner interner;
    std::vector<std::uint32_t> tokenindex;       // index per token entry, in order
    std::vector<std::vector<std::uint32_t>> fileexpridx, paramidx; // per include entry
    for( auto const &entry : m_entries ) {
        if( entry.type == scenery_entry_type::token ) {
            tokenindex.emplace_back( interner.intern( entry.text ) );
        }
        else if( entry.type == scenery_entry_type::include ) {
            std::vector<std::uint32_t> fe, pe;
            for( auto const &token : entry.fileexpr ) { fe.emplace_back( interner.intern( token ) ); }
            for( auto const &param : entry.params )   { pe.emplace_back( interner.intern( param ) ); }
            fileexpridx.emplace_back( std::move( fe ) );
            paramidx.emplace_back( std::move( pe ) );
        }
    }

    // header: magic + kind + flags
    sn_utils::ls_uint32( Output, SCENERYBINARY_MAGIC );
    sn_utils::s_uint8( Output, static_cast<std::uint8_t>( Kind ) );
    sn_utils::ls_uint32( Output, 0 ); // flags, reserved

    // string table
    auto const &table = interner.table();
    sn_utils::ls_uint32( Output, static_cast<std::uint32_t>( table.size() ) );
    for( auto const &text : table ) {
        sn_utils::s_str( Output, text );
    }

    // entries
    sn_utils::ls_uint32( Output, static_cast<std::uint32_t>( m_entries.size() ) );
    std::size_t tokencursor = 0, includecursor = 0;
    for( auto const &entry : m_entries ) {
        sn_utils::s_uint8( Output, static_cast<std::uint8_t>( entry.type ) );
        if( entry.type == scenery_entry_type::include ) {
            auto const &fe = fileexpridx[ includecursor ];
            auto const &pe = paramidx[ includecursor ];
            ++includecursor;
            write_varint( Output, fe.size() );
            for( auto idx : fe ) { write_varint( Output, idx ); }
            write_varint( Output, pe.size() );
            for( auto idx : pe ) { write_varint( Output, idx ); }
        }
        else if( entry.type == scenery_entry_type::number ) {
            sn_utils::ls_float64( Output, entry.number );
        }
        else {
            write_varint( Output, tokenindex[ tokencursor++ ] );
        }
    }

    return ( false == Output.fail() );
}

bool
scenery_binary_reader::open( std::istream &Input ) {

    m_entries.clear();

    auto const magic { sn_utils::ld_uint32( Input ) };
    if( magic != SCENERYBINARY_MAGIC ) {
        // unrecognized type or incompatible version
        return false;
    }
    m_kind = static_cast<scenery_file_kind>( sn_utils::d_uint8( Input ) );
    sn_utils::ld_uint32( Input ); // flags, reserved

    // string table
    std::vector<std::string> table;
    auto tablesize { sn_utils::ld_uint32( Input ) };
    table.reserve( tablesize );
    while( tablesize-- ) {
        table.emplace_back( sn_utils::d_str( Input ) );
    }
    // resolves an interned index back to its string, guarding against corruption
    auto const resolve = [ & ]( std::uint64_t index ) -> std::string {
        return ( index < table.size() ) ? table[ static_cast<std::size_t>( index ) ] : std::string();
    };

    auto count { sn_utils::ld_uint32( Input ) };
    m_entries.reserve( count );
    while( count-- ) {
        scenery_entry entry;
        entry.type = static_cast<scenery_entry_type>( sn_utils::d_uint8( Input ) );
        if( entry.type == scenery_entry_type::include ) {
            auto expr { read_varint( Input ) };
            entry.fileexpr.reserve( expr );
            while( expr-- ) {
                entry.fileexpr.emplace_back( resolve( read_varint( Input ) ) );
            }
            auto params { read_varint( Input ) };
            entry.params.reserve( params );
            while( params-- ) {
                entry.params.emplace_back( resolve( read_varint( Input ) ) );
            }
        }
        else if( entry.type == scenery_entry_type::number ) {
            entry.number = sn_utils::ld_float64( Input );
        }
        else {
            entry.text = resolve( read_varint( Input ) );
        }
        m_entries.emplace_back( std::move( entry ) );
    }

    return ( false == Input.fail() );
}

namespace {

// minimal bounded thread pool for offloading twin serialization/writing
class bake_pool {
public:
    bake_pool() {
        unsigned const workers = std::max( 2u, std::min( 8u, std::thread::hardware_concurrency() ) );
        for( unsigned i = 0; i < workers; ++i ) {
            m_threads.emplace_back( [ this ] { run(); } );
        }
    }
    ~bake_pool() {
        { std::unique_lock<std::mutex> lock( m_mutex ); m_stop = true; }
        m_wake.notify_all();
        for( auto &thread : m_threads ) { if( thread.joinable() ) { thread.join(); } }
    }
    void enqueue( std::function<void()> Task ) {
        { std::unique_lock<std::mutex> lock( m_mutex ); m_tasks.emplace( std::move( Task ) ); ++m_pending; }
        m_wake.notify_one();
    }
    void wait_idle() {
        std::unique_lock<std::mutex> lock( m_mutex );
        m_idle.wait( lock, [ this ] { return m_pending == 0; } );
    }
private:
    void run() {
        for( ;; ) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock( m_mutex );
                m_wake.wait( lock, [ this ] { return m_stop || ( false == m_tasks.empty() ); } );
                if( m_stop && m_tasks.empty() ) { return; }
                task = std::move( m_tasks.front() );
                m_tasks.pop();
            }
            task();
            {
                std::unique_lock<std::mutex> lock( m_mutex );
                if( --m_pending == 0 ) { m_idle.notify_all(); }
            }
        }
    }
    std::vector<std::thread> m_threads;
    std::queue<std::function<void()>> m_tasks;
    std::mutex m_mutex;
    std::condition_variable m_wake;
    std::condition_variable m_idle;
    bool m_stop { false };
    std::size_t m_pending { 0 };
};

bake_pool &pool() {
    static bake_pool instance;
    return instance;
}

// results recorded by worker threads, drained and logged on the main thread
std::mutex g_resultmutex;
std::vector<std::string> g_resultlog; // success lines, in completion order
std::vector<std::string> g_resultfail; // failure paths

} // anonymous namespace

void
scenerybinary_write_async( std::unique_ptr<scenery_binary_writer> Writer, std::string Path, scenery_file_kind Kind ) {
    // std::function requires a copyable target, so move the writer into a shared_ptr
    std::shared_ptr<scenery_binary_writer> writer { std::move( Writer ) };
    pool().enqueue( [ writer, path = std::move( Path ), Kind ] {
        std::ofstream output( path, std::ios::binary );
        bool const ok = output.good() && writer->write( output, Kind );
        output.flush();
        std::lock_guard<std::mutex> lock( g_resultmutex );
        if( ok ) {
            g_resultlog.emplace_back( "Compiled binary scenery: " + path + " (" + std::to_string( writer->entry_count() ) + " entries)" );
        }
        else {
            g_resultfail.emplace_back( path );
        }
    } );
}

void
scenerybinary_wait_all() {
    pool().wait_idle();
    std::lock_guard<std::mutex> lock( g_resultmutex );
    for( auto const &line : g_resultlog ) { WriteLog( line ); }
    for( auto const &path : g_resultfail ) { ErrorLog( "Failed to write binary scenery \"" + path + "\"" ); }
    g_resultlog.clear();
    g_resultfail.clear();
}

std::string
scenerybinary_extension_for( std::string const &Sourcefile ) {

    auto const lower { ToLower( Sourcefile ) };
    if( lower.size() >= 4 && lower.compare( lower.size() - 4, 4, ".inc" ) == 0 ) {
        return SCENERYBINARY_EXT_INC;
    }
    if( lower.size() >= 4 && lower.compare( lower.size() - 4, 4, ".scm" ) == 0 ) {
        return SCENERYBINARY_EXT_SCM;
    }
    if( lower.size() >= 4 && lower.compare( lower.size() - 4, 4, ".ctr" ) == 0 ) {
        return SCENERYBINARY_EXT_CTR;
    }
    // default: treat as a top-level scenario file (.scn)
    return SCENERYBINARY_EXT_SCN;
}

} // namespace scene
