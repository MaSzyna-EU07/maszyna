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
#include <string_view>
#include <cmath>
#include <cstring>

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

// zig-zag maps small-magnitude signed integers to small unsigned varints
std::uint64_t zigzag_encode( std::int64_t Value ) {
    return ( static_cast<std::uint64_t>( Value ) << 1 ) ^ static_cast<std::uint64_t>( Value >> 63 );
}
std::int64_t zigzag_decode( std::uint64_t Value ) {
    return static_cast<std::int64_t>( ( Value >> 1 ) ^ ( ~( Value & 1 ) + 1 ) );
}

// buffer cursor reads (little-endian, matching sn_utils), bounds-checked: on overrun the
// cursor is parked at the end and zero is returned, so a truncated twin decodes to empty
// rather than reading out of bounds.
std::uint64_t read_varint( char const *&Cursor, char const *End ) {
    std::uint64_t value = 0;
    int shift = 0;
    while( ( Cursor < End ) && ( shift < 64 ) ) {
        std::uint8_t const byte = static_cast<std::uint8_t>( *Cursor++ );
        value |= ( static_cast<std::uint64_t>( byte & 0x7F ) << shift );
        if( ( byte & 0x80 ) == 0 ) { break; }
        shift += 7;
    }
    return value;
}
std::uint32_t read_u32le( char const *&Cursor, char const *End ) {
    if( End - Cursor < 4 ) { Cursor = End; return 0; }
    auto const *b = reinterpret_cast<std::uint8_t const *>( Cursor );
    Cursor += 4;
    return ( std::uint32_t( b[ 0 ] ) ) | ( std::uint32_t( b[ 1 ] ) << 8 )
         | ( std::uint32_t( b[ 2 ] ) << 16 ) | ( std::uint32_t( b[ 3 ] ) << 24 );
}
float read_f32le( char const *&Cursor, char const *End ) {
    if( End - Cursor < 4 ) { Cursor = End; return 0.f; }
    std::uint32_t const v = read_u32le( Cursor, End );
    float f; std::memcpy( &f, &v, 4 ); return f;
}
double read_f64le( char const *&Cursor, char const *End ) {
    if( End - Cursor < 8 ) { Cursor = End; return 0.0; }
    auto const *b = reinterpret_cast<std::uint8_t const *>( Cursor );
    Cursor += 8;
    std::uint64_t v = 0;
    for( int i = 0; i < 8; ++i ) { v |= ( static_cast<std::uint64_t>( b[ i ] ) << ( 8 * i ) ); }
    double d; std::memcpy( &d, &v, 8 ); return d;
}

// on-disk entry tag, packed into the low 3 bits of the per-entry head varint
enum : std::uint64_t {
    TAG_TOKEN   = 0, // head >> 3 == interned string index
    TAG_INCLUDE = 1,
    TAG_INT     = 2, // followed by a zig-zag varint
    TAG_F32     = 3, // followed by 4 little-endian bytes
    TAG_F64     = 4, // followed by 8 little-endian bytes
    TAG_QTOKEN  = 5, // quoted token; head >> 3 == interned string index
    TAG_NODE    = 6, // node marker: followed by varint(class) + varint(byte span)
    TAG_BITS    = 3,
    TAG_MASK    = 0x7,
};

// node class stored in the TAG_NODE marker
enum : std::uint64_t {
    NODECLASS_INFRA      = 0,
    NODECLASS_VISUAL     = 1,
    NODECLASS_VISUAL_POS = 2, // visual node whose marker is followed by 3 f32 local position
};

// writes a numeric value in the most compact lossless-enough form: integral values as a
// zig-zag varint, otherwise f32 when it represents the value with negligible error,
// otherwise full f64.
void write_number( std::ostream &Output, double Value ) {
    double integral = 0.0;
    if( ( std::modf( Value, &integral ) == 0.0 )
     && ( Value >= -9.007199254740992e15 ) && ( Value <= 9.007199254740992e15 ) ) {
        write_varint( Output, TAG_INT );
        write_varint( Output, zigzag_encode( static_cast<std::int64_t>( Value ) ) );
        return;
    }
    float const f = static_cast<float>( Value );
    bool const f32ok = std::isfinite( f )
        && ( ( static_cast<double>( f ) == Value )
          || ( std::abs( static_cast<double>( f ) - Value ) <= 1e-6 * std::abs( Value ) ) );
    if( f32ok ) {
        write_varint( Output, TAG_F32 );
        sn_utils::ls_float32( Output, f );
    }
    else {
        write_varint( Output, TAG_F64 );
        sn_utils::ls_float64( Output, Value );
    }
}

} // anonymous namespace

std::uint32_t
scenery_binary_writer::intern( std::string const &Text ) {
    auto const it = m_lookup.find( Text );
    if( it != m_lookup.end() ) { return it->second; }
    auto const index = static_cast<std::uint32_t>( m_table.size() );
    m_lookup.emplace( Text, index );
    m_table.emplace_back( Text );
    return index;
}

std::ostream &
scenery_binary_writer::sink() {
    return ( m_innode ? m_nodebuf : m_entries );
}

void
scenery_binary_writer::add_token( std::string const &Token, bool Quoted ) {
    auto const tag = ( Quoted ? TAG_QTOKEN : TAG_TOKEN );
    write_varint( sink(), ( static_cast<std::uint64_t>( intern( Token ) ) << TAG_BITS ) | tag );
    ++m_count;
}

void
scenery_binary_writer::add_number( double Value ) {
    write_number( sink(), Value );
    ++m_count;
}

void
scenery_binary_writer::add_include( std::vector<std::string> const &Fileexpr, std::vector<std::string> const &Params ) {
    auto &out = sink();
    write_varint( out, TAG_INCLUDE );
    write_varint( out, Fileexpr.size() );
    for( auto const &token : Fileexpr ) { write_varint( out, intern( token ) ); }
    write_varint( out, Params.size() );
    for( auto const &param : Params ) { write_varint( out, intern( param ) ); }
    ++m_count;
}

void
scenery_binary_writer::begin_node() {
    // start buffering this node's entries; end_node() emits the marker + buffered bytes
    m_nodebuf.str( std::string() );
    m_nodebuf.clear();
    m_innode = true;
}

void
scenery_binary_writer::end_node( bool Visual, bool Haspos, double X, double Y, double Z, double Range ) {
    if( false == m_innode ) { return; }
    m_innode = false;
    auto const body = m_nodebuf.str();
    write_varint( m_entries, TAG_NODE );
    auto const cls =
        ( false == Visual ) ? NODECLASS_INFRA :
        ( Haspos          ) ? NODECLASS_VISUAL_POS :
                              NODECLASS_VISUAL;
    write_varint( m_entries, cls );
    write_varint( m_entries, body.size() );
    // a visual model node stores its local position + visibility range right after the span, so
    // the reader can hand them to the streaming load without the node body being decoded
    if( cls == NODECLASS_VISUAL_POS ) {
        sn_utils::ls_float32( m_entries, static_cast<float>( X ) );
        sn_utils::ls_float32( m_entries, static_cast<float>( Y ) );
        sn_utils::ls_float32( m_entries, static_cast<float>( Z ) );
        sn_utils::ls_float32( m_entries, static_cast<float>( Range ) );
    }
    m_entries.write( body.data(), static_cast<std::streamsize>( body.size() ) );
}

bool
scenery_binary_writer::write( std::ostream &Output, scenery_file_kind Kind ) const {

    // header: magic + kind + flags
    sn_utils::ls_uint32( Output, SCENERYBINARY_MAGIC );
    sn_utils::s_uint8( Output, static_cast<std::uint8_t>( Kind ) );
    sn_utils::ls_uint32( Output, 0 ); // flags, reserved

    // string table: count, then each string as varint(length) + raw bytes (so the
    // reader can take views into the buffer without scanning for terminators)
    write_varint( Output, m_table.size() );
    for( auto const &text : m_table ) {
        write_varint( Output, text.size() );
        Output.write( text.data(), static_cast<std::streamsize>( text.size() ) );
    }

    // entries: the pre-encoded entry bytes (heads + payloads + node markers) verbatim,
    // running to end-of-file (the reader streams until the buffer is exhausted)
    auto const encoded = m_entries.str();
    Output.write( encoded.data(), static_cast<std::streamsize>( encoded.size() ) );

    return ( false == Output.fail() );
}

bool
scenery_binary_reader::open( std::string_view Buffer ) {

    m_table.clear();
    m_begin = m_cursor = m_end = nullptr;
    m_size = 0;

    char const *cursor = Buffer.data();
    char const *const end = Buffer.data() + Buffer.size();

    if( end - cursor < 9 ) { return false; } // magic(4) + kind(1) + flags(4)
    if( read_u32le( cursor, end ) != SCENERYBINARY_MAGIC ) {
        // unrecognized type or incompatible version
        return false;
    }
    m_kind = static_cast<scenery_file_kind>( static_cast<std::uint8_t>( *cursor++ ) );
    read_u32le( cursor, end ); // flags, reserved

    // string table: views into the buffer (no copies)
    auto tablesize = read_varint( cursor, end );
    m_table.reserve( tablesize );
    while( ( tablesize-- != 0 ) && ( cursor < end ) ) {
        auto const len = read_varint( cursor, end );
        if( static_cast<std::uint64_t>( end - cursor ) < len ) { cursor = end; break; }
        m_table.emplace_back( cursor, static_cast<std::size_t>( len ) );
        cursor += len;
    }

    // entries run from here to end-of-buffer
    m_begin = m_cursor = cursor;
    m_end = end;
    m_size = end - cursor;
    return true;
}

bool
scenery_binary_reader::next( scenery_entry_view &Out ) {

    auto const resolve = [ this ]( std::uint64_t index ) -> std::string_view {
        return ( index < m_table.size() ) ? m_table[ static_cast<std::size_t>( index ) ] : std::string_view();
    };

    // skip node markers (and whole nodes not belonging to the current pass) until a
    // servable entry is reached
    std::uint64_t head = 0;
    std::uint64_t tag = 0;
    for( ;; ) {
        if( m_cursor >= m_end ) { return false; }
        char const *markerstart = m_cursor; // where this entry's marker begins (for seek_node)
        head = read_varint( m_cursor, m_end );
        tag = head & TAG_MASK;
        if( tag != TAG_NODE ) { break; }
        // record the served node's marker offset so the section-index streamer can seek back to
        // it and rebuild the node on demand without re-scanning the whole twin
        m_nodeoffset = static_cast<std::size_t>( markerstart - m_begin );
        auto const cls = read_varint( m_cursor, m_end );
        auto const span = read_varint( m_cursor, m_end );
        bool const isvisual = ( cls == NODECLASS_VISUAL ) || ( cls == NODECLASS_VISUAL_POS );
        // a visual model marker carries the node's local position right after the span
        m_nodehaspos = ( cls == NODECLASS_VISUAL_POS );
        if( true == m_nodehaspos ) {
            m_nodepos[ 0 ] = static_cast<double>( read_f32le( m_cursor, m_end ) );
            m_nodepos[ 1 ] = static_cast<double>( read_f32le( m_cursor, m_end ) );
            m_nodepos[ 2 ] = static_cast<double>( read_f32le( m_cursor, m_end ) );
            m_noderange    = static_cast<double>( read_f32le( m_cursor, m_end ) );
        }
        bool const process =
            ( m_pass == scenery_load_pass::all )
         || ( ( m_pass == scenery_load_pass::infrastructure ) && ( cls == NODECLASS_INFRA ) )
         || ( ( m_pass == scenery_load_pass::visual ) && ( true == isvisual ) );
        if( false == process ) {
            // skip the whole node body
            m_cursor += static_cast<std::ptrdiff_t>( span );
            if( m_cursor > m_end ) { m_cursor = m_end; }
            m_nodehaspos = false;
        }
        else {
            // remember where this node ends so the consumer can bail out of it in O(1)
            // (skip_to_node_end) after peeking just its first few entries -- used by the
            // camera-ring visual load to drop a node that's outside the current distance ring
            m_nodeend = m_cursor + static_cast<std::ptrdiff_t>( span );
            if( m_nodeend > m_end ) { m_nodeend = m_end; }
        }
        // when processing, fall through: the loop re-reads and decodes the node's first entry
    }

    Out.fileexpr.clear();
    Out.params.clear();

    switch( tag ) {
        case TAG_INCLUDE: {
            Out.type = scenery_entry_type::include;
            auto fe = read_varint( m_cursor, m_end );
            Out.fileexpr.reserve( fe );
            while( ( fe-- != 0 ) && ( m_cursor < m_end ) ) { Out.fileexpr.emplace_back( resolve( read_varint( m_cursor, m_end ) ) ); }
            auto pe = read_varint( m_cursor, m_end );
            Out.params.reserve( pe );
            while( ( pe-- != 0 ) && ( m_cursor < m_end ) ) { Out.params.emplace_back( resolve( read_varint( m_cursor, m_end ) ) ); }
            break;
        }
        case TAG_INT:
            Out.type = scenery_entry_type::number;
            Out.number = static_cast<double>( zigzag_decode( read_varint( m_cursor, m_end ) ) );
            break;
        case TAG_F32:
            Out.type = scenery_entry_type::number;
            Out.number = static_cast<double>( read_f32le( m_cursor, m_end ) );
            break;
        case TAG_F64:
            Out.type = scenery_entry_type::number;
            Out.number = read_f64le( m_cursor, m_end );
            break;
        case TAG_QTOKEN:
            Out.type = scenery_entry_type::qtoken;
            Out.text = resolve( head >> TAG_BITS );
            break;
        case TAG_TOKEN:
        default:
            Out.type = scenery_entry_type::token;
            Out.text = resolve( head >> TAG_BITS );
            break;
    }
    return true;
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
