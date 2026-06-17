/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// ---------------------------------------------------------------------------
// eu7v2 - clean-slate compiled scenery format core (no backward compatibility
// with the legacy EU7B v4..v13 chunks).
//
// Design goals:
//   * self-describing, skippable chunks: [u32 id][u64 size][payload]
//   * explicit little-endian on disk so files are deterministic & portable
//   * a central string table so payloads reference strings by u32 id
//   * a separation of file kinds: sim core / reusable module / streamable tile
//   * header-only and dependency-free so it can be unit tested in isolation
//
// This is the foundation layer only (iteration 1). Higher layers (baker,
// runtime loader, streaming) build on top of these primitives.
// ---------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace eu7v2 {

// FourCC packed little-endian: 'A' is the least significant byte so the bytes
// appear in source order when the file is viewed in a hex editor.
[[nodiscard]] constexpr std::uint32_t
fourcc( char const a, char const b, char const c, char const d ) noexcept {
    return ( static_cast<std::uint32_t>( static_cast<unsigned char>( a ) ) )
        | ( static_cast<std::uint32_t>( static_cast<unsigned char>( b ) ) << 8 )
        | ( static_cast<std::uint32_t>( static_cast<unsigned char>( c ) ) << 16 )
        | ( static_cast<std::uint32_t>( static_cast<unsigned char>( d ) ) << 24 );
}

// File magic + single current version. No legacy versions are recognised.
constexpr std::uint32_t kMagic { fourcc( 'E', 'U', '7', 'C' ) };
constexpr std::uint16_t kVersion { 1 };

enum class file_kind : std::uint16_t {
    sim = 1,      // global simulation core, loaded once
    module = 2,   // reusable module / prototype library (baked .inc)
    tile = 3,     // streamable 1km visual tile (terrain + instances)
    manifest = 4, // index of tiles & modules for a map
};

// Chunk identifiers (FourCC). Kept small & explicit; grows as layers land.
namespace chunk {
constexpr std::uint32_t strs { fourcc( 'S', 'T', 'R', 'S' ) }; // string table
constexpr std::uint32_t meta { fourcc( 'M', 'E', 'T', 'A' ) }; // key/value metadata
constexpr std::uint32_t prot { fourcc( 'P', 'R', 'O', 'T' ) }; // model prototypes (module)
constexpr std::uint32_t inst { fourcc( 'I', 'N', 'S', 'T' ) }; // lean instances (tile)
constexpr std::uint32_t mesh { fourcc( 'M', 'E', 'S', 'H' ) }; // baked terrain mesh (tile)
constexpr std::uint32_t shpe { fourcc( 'S', 'H', 'P', 'E' ) }; // non-terrain shapes (triangles)
constexpr std::uint32_t line { fourcc( 'L', 'I', 'N', 'E' ) }; // line geometry nodes
constexpr std::uint32_t incl { fourcc( 'I', 'N', 'C', 'L' ) }; // module includes (recursion refs)
constexpr std::uint32_t plce { fourcc( 'P', 'L', 'C', 'E' ) }; // lean .inc placements (binary coords)
constexpr std::uint32_t trst { fourcc( 'T', 'R', 'S', 'T' ) }; // trainsets
constexpr std::uint32_t trgr { fourcc( 'T', 'R', 'G', 'R' ) }; // precomputed track graph (sim)
constexpr std::uint32_t sidx { fourcc( 'S', 'I', 'D', 'X' ) }; // spatial section index (sim/manifest)
// simulation record chunks (sim)
constexpr std::uint32_t trak { fourcc( 'T', 'R', 'A', 'K' ) }; // tracks
constexpr std::uint32_t trac { fourcc( 'T', 'R', 'A', 'C' ) }; // traction wires
constexpr std::uint32_t pwrs { fourcc( 'P', 'W', 'R', 'S' ) }; // traction power sources
constexpr std::uint32_t memc { fourcc( 'M', 'E', 'M', 'C' ) }; // memory cells
constexpr std::uint32_t laun { fourcc( 'L', 'A', 'U', 'N' ) }; // event launchers
constexpr std::uint32_t evnt { fourcc( 'E', 'V', 'N', 'T' ) }; // events
constexpr std::uint32_t sond { fourcc( 'S', 'O', 'N', 'D' ) }; // sounds
constexpr std::uint32_t dynm { fourcc( 'D', 'Y', 'N', 'M' ) }; // dynamic vehicles
} // namespace chunk

// ---------------------------------------------------------------------------
// Writer side
// ---------------------------------------------------------------------------

// Little-endian byte sink. All multi-byte values are written explicitly so the
// output does not depend on host endianness.
class byte_writer {
  public:
    void put_u8( std::uint8_t const v ) { m_data.push_back( v ); }

    void put_u16( std::uint16_t const v ) {
        put_u8( static_cast<std::uint8_t>( v ) );
        put_u8( static_cast<std::uint8_t>( v >> 8 ) );
    }

    void put_u32( std::uint32_t const v ) {
        put_u16( static_cast<std::uint16_t>( v ) );
        put_u16( static_cast<std::uint16_t>( v >> 16 ) );
    }

    void put_u64( std::uint64_t const v ) {
        put_u32( static_cast<std::uint32_t>( v ) );
        put_u32( static_cast<std::uint32_t>( v >> 32 ) );
    }

    void put_i32( std::int32_t const v ) {
        put_u32( static_cast<std::uint32_t>( v ) );
    }

    void put_f32( float const v ) {
        std::uint32_t bits;
        std::memcpy( &bits, &v, sizeof( bits ) );
        put_u32( bits );
    }

    // doubles are stored as f32 on disk (positions excepted via put_f64 where
    // the extra precision actually matters)
    void put_f64( double const v ) {
        std::uint64_t bits;
        std::memcpy( &bits, &v, sizeof( bits ) );
        put_u64( bits );
    }

    void put_vec3f( double const x, double const y, double const z ) {
        put_f32( static_cast<float>( x ) );
        put_f32( static_cast<float>( y ) );
        put_f32( static_cast<float>( z ) );
    }

    void put_bytes( void const *data, std::size_t const size ) {
        auto const *bytes { static_cast<std::uint8_t const *>( data ) };
        m_data.insert( m_data.end(), bytes, bytes + size );
    }

    [[nodiscard]] std::vector<std::uint8_t> const &data() const noexcept { return m_data; }
    [[nodiscard]] std::vector<std::uint8_t> &data() noexcept { return m_data; }
    [[nodiscard]] std::size_t size() const noexcept { return m_data.size(); }

  private:
    std::vector<std::uint8_t> m_data;
};

// Deduplicating string table. Returns a stable u32 id per unique string.
class string_table {
  public:
    [[nodiscard]] std::uint32_t intern( std::string_view const s ) {
        auto const key { std::string( s ) };
        auto const it { m_lookup.find( key ) };
        if( it != m_lookup.end() ) {
            return it->second;
        }
        auto const id { static_cast<std::uint32_t>( m_strings.size() ) };
        m_strings.emplace_back( key );
        m_lookup.emplace( key, id );
        return id;
    }

    [[nodiscard]] std::size_t size() const noexcept { return m_strings.size(); }

    // Serialise as: [u32 count]( [u32 len][bytes] )*
    void serialize( byte_writer &out ) const {
        out.put_u32( static_cast<std::uint32_t>( m_strings.size() ) );
        for( auto const &s : m_strings ) {
            out.put_u32( static_cast<std::uint32_t>( s.size() ) );
            out.put_bytes( s.data(), s.size() );
        }
    }

  private:
    std::vector<std::string> m_strings;
    std::unordered_map<std::string, std::uint32_t> m_lookup;
};

// Top-level container writer: file header followed by [id][size][payload] chunks.
class container_writer {
  public:
    explicit container_writer( file_kind const kind ) : m_kind( kind ) {
        m_out.put_u32( kMagic );
        m_out.put_u16( kVersion );
        m_out.put_u16( static_cast<std::uint16_t>( kind ) );
        // reserved for flags / future use, keeps header 16-byte aligned
        m_out.put_u32( 0 );
        m_out.put_u32( 0 );
    }

    // Append a fully-built chunk payload under the given id.
    void add_chunk( std::uint32_t const id, std::vector<std::uint8_t> const &payload ) {
        m_out.put_u32( id );
        m_out.put_u64( static_cast<std::uint64_t>( payload.size() ) );
        m_out.put_bytes( payload.data(), payload.size() );
    }

    void add_chunk( std::uint32_t const id, byte_writer const &payload ) {
        add_chunk( id, payload.data() );
    }

    [[nodiscard]] file_kind kind() const noexcept { return m_kind; }
    [[nodiscard]] std::vector<std::uint8_t> const &data() const noexcept { return m_out.data(); }

  private:
    file_kind m_kind;
    byte_writer m_out;
};

// ---------------------------------------------------------------------------
// Reader side (bounds-checked)
// ---------------------------------------------------------------------------

class parse_error : public std::runtime_error {
  public:
    explicit parse_error( std::string const &what ) : std::runtime_error( what ) {}
};

// Little-endian cursor over a byte span. Every read is bounds-checked and
// throws parse_error on truncation so a corrupt file can never read OOB.
class byte_reader {
  public:
    byte_reader( std::uint8_t const *data, std::size_t const size ) noexcept
        : m_data( data ), m_size( size ) {}

    [[nodiscard]] std::size_t remaining() const noexcept { return m_size - m_pos; }
    [[nodiscard]] bool empty() const noexcept { return m_pos >= m_size; }
    [[nodiscard]] std::size_t position() const noexcept { return m_pos; }

    std::uint8_t get_u8() {
        require( 1 );
        return m_data[ m_pos++ ];
    }

    std::uint16_t get_u16() {
        std::uint16_t const lo { get_u8() };
        std::uint16_t const hi { get_u8() };
        return static_cast<std::uint16_t>( lo | ( hi << 8 ) );
    }

    std::uint32_t get_u32() {
        std::uint32_t const lo { get_u16() };
        std::uint32_t const hi { get_u16() };
        return lo | ( hi << 16 );
    }

    std::uint64_t get_u64() {
        std::uint64_t const lo { get_u32() };
        std::uint64_t const hi { get_u32() };
        return lo | ( hi << 32 );
    }

    std::int32_t get_i32() { return static_cast<std::int32_t>( get_u32() ); }

    float get_f32() {
        std::uint32_t const bits { get_u32() };
        float v;
        std::memcpy( &v, &bits, sizeof( v ) );
        return v;
    }

    double get_f64() {
        std::uint64_t const bits { get_u64() };
        double v;
        std::memcpy( &v, &bits, sizeof( v ) );
        return v;
    }

    void get_bytes( void *dst, std::size_t const size ) {
        require( size );
        std::memcpy( dst, m_data + m_pos, size );
        m_pos += size;
    }

    [[nodiscard]] std::uint8_t const *take( std::size_t const size ) {
        require( size );
        auto const *ptr { m_data + m_pos };
        m_pos += size;
        return ptr;
    }

  private:
    void require( std::size_t const n ) const {
        if( m_pos + n > m_size ) {
            throw parse_error( "eu7v2: unexpected end of data" );
        }
    }

    std::uint8_t const *m_data;
    std::size_t m_size;
    std::size_t m_pos { 0 };
};

// Decoded string table: id -> string view into the owned backing store.
class string_pool {
  public:
    void deserialize( byte_reader &in ) {
        auto const count { in.get_u32() };
        m_strings.clear();
        m_strings.reserve( count );
        for( std::uint32_t i { 0 }; i < count; ++i ) {
            auto const len { in.get_u32() };
            std::string s;
            s.resize( len );
            if( len != 0 ) {
                in.get_bytes( s.data(), len );
            }
            m_strings.emplace_back( std::move( s ) );
        }
    }

    [[nodiscard]] std::string const &get( std::uint32_t const id ) const {
        if( id >= m_strings.size() ) {
            throw parse_error( "eu7v2: string id out of range" );
        }
        return m_strings[ id ];
    }

    [[nodiscard]] std::size_t size() const noexcept { return m_strings.size(); }

  private:
    std::vector<std::string> m_strings;
};

// One chunk located inside a container: its id and a view of its payload.
struct chunk_view {
    std::uint32_t id { 0 };
    std::uint8_t const *data { nullptr };
    std::size_t size { 0 };

    [[nodiscard]] byte_reader reader() const noexcept { return byte_reader( data, size ); }
};

// Reads the file header and iterates chunks without copying payloads.
class container_reader {
  public:
    container_reader( std::uint8_t const *data, std::size_t const size ) : m_cursor( data, size ) {
        auto const magic { m_cursor.get_u32() };
        if( magic != kMagic ) {
            throw parse_error( "eu7v2: bad magic" );
        }
        m_version = m_cursor.get_u16();
        if( m_version != kVersion ) {
            throw parse_error( "eu7v2: unsupported version" );
        }
        m_kind = static_cast<file_kind>( m_cursor.get_u16() );
        (void)m_cursor.get_u32(); // reserved
        (void)m_cursor.get_u32(); // reserved
    }

    [[nodiscard]] file_kind kind() const noexcept { return m_kind; }
    [[nodiscard]] std::uint16_t version() const noexcept { return m_version; }

    // Returns false when there are no more chunks.
    [[nodiscard]] bool next( chunk_view &out ) {
        if( m_cursor.remaining() == 0 ) {
            return false;
        }
        out.id = m_cursor.get_u32();
        auto const size { m_cursor.get_u64() };
        out.size = static_cast<std::size_t>( size );
        out.data = m_cursor.take( out.size );
        return true;
    }

  private:
    byte_reader m_cursor;
    file_kind m_kind { file_kind::sim };
    std::uint16_t m_version { 0 };
};

// Maps a text scenery source to its compiled .eu7v2 path. Standard module
// extensions (.scm/.scn/.sbt/.inc) become "<stem>.eu7v2"; other extensions
// (.ctr, …) keep the source suffix so e.g. foo.scm and foo.ctr do not collide.
[[nodiscard]] inline std::filesystem::path
binary_path_from_text( std::filesystem::path const &text_path ) {
    auto const ext { text_path.extension().string() };
    if( ext == ".scm" || ext == ".scn" || ext == ".sbt" || ext == ".inc" ) {
        return text_path.parent_path() / ( text_path.stem().string() + ".eu7v2" );
    }
    return text_path.parent_path() / ( text_path.filename().string() + ".eu7v2" );
}

} // namespace eu7v2
