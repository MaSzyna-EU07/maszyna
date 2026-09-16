/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// Reader for the cooked terrain heightfield. Header-only and free of engine types, so
// that the cooker can verify its own output with the same code the engine runs - one
// reader, not two that drift.
//
// The class owns the file handle and the directory; sample data it decodes into buffers
// the caller provides and recycles. Nothing about the on-disk layout escapes past
// read_level(): callers see tile coordinates, mip levels and samples.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <zstd.h>

#include "scene/heightfieldformat.h"

namespace heightfield {

class reader {

public:
    struct tile_description {
        std::int32_t x { 0 }, z { 0 };
        float minheight { 0.f }, maxheight { 0.f };
        std::uint32_t coverage { 0 }; // samples carrying terrain at the finest level
    };

    reader() = default;
    ~reader() { ZSTD_freeDCtx( m_context ); }
    reader( reader const & ) = delete;
    reader &operator=( reader const & ) = delete;

    // returns false and leaves the reader closed when the file is missing, truncated,
    // or written by a version this build does not know
    bool
        open( std::string const &Path ) {

            close();
            m_file.open( Path, std::ios::binary );
            if( false == m_file.good() ) { return false; }

            if( false == read_at( 0, &m_header, sizeof( m_header ) ) ) { return close_failed(); }
            if( 0 != std::memcmp( m_header.magic, magic, sizeof( magic ) ) ) { return close_failed(); }
            if( m_header.version != version ) { return close_failed(); }
            if( ( m_header.tilesamples == 0 ) || ( m_header.miplevels == 0 ) ) { return close_failed(); }

            std::vector<tile_entry> directory( m_header.tilecount );
            if( false == read_at(
                    sizeof( file_header ), directory.data(),
                    directory.size() * sizeof( tile_entry ) ) ) { return close_failed(); }

            m_entries.reserve( directory.size() );
            m_index.reserve( directory.size() );
            m_descriptions.reserve( directory.size() );
            for( auto const &entry : directory ) {
                m_index.emplace( tile_key( entry.x, entry.z ), m_entries.size() );
                m_entries.push_back( entry );
                m_descriptions.push_back( { entry.x, entry.z, entry.minheight, entry.maxheight, entry.coverage } );
            }

            auto offset { sizeof( file_header ) + directory.size() * sizeof( tile_entry ) };
            m_materials.reserve( m_header.materialcount );
            for( std::uint32_t index = 0; index < m_header.materialcount; ++index ) {
                std::uint16_t length { 0 };
                if( false == read_at( offset, &length, sizeof( length ) ) ) { return close_failed(); }
                offset += sizeof( length );
                std::string name( length, '\0' );
                if( ( length > 0 ) && ( false == read_at( offset, name.data(), length ) ) ) { return close_failed(); }
                offset += length;
                float scale { 4.f };
                if( false == read_at( offset, &scale, sizeof( scale ) ) ) { return close_failed(); }
                offset += sizeof( scale );
                m_materials.push_back( std::move( name ) );
                m_materialscales.push_back( scale );
            }

            m_open = true;
            return true;
        }

    void
        close() {
            m_file.close();
            m_file.clear();
            m_entries.clear();
            m_descriptions.clear();
            m_index.clear();
            m_materials.clear();
            m_materialscales.clear();
            m_header = {};
            m_open = false; }

    bool ready() const { return m_open; }

    // metres between samples at the finest level
    float gridstep() const { return m_header.gridstep; }
    // samples a tile owns along a side, shared edge excluded
    std::uint32_t tilesamples() const { return m_header.tilesamples; }
    // side of a tile in metres
    float tilesize() const { return m_header.gridstep * static_cast<float>( m_header.tilesamples ); }
    std::uint32_t levels() const { return m_header.miplevels; }
    std::uint32_t samples_per_side( std::uint32_t const Level ) const {
        return samples_at_level( m_header.tilesamples, Level ); }

    std::vector<tile_description> const &tiles() const { return m_descriptions; }
    std::vector<std::string> const &materials() const { return m_materials; }
    // metres of ground covered by one repeat of a material's texture, measured by the
    // cooker from the texture coordinates the source geometry carried
    std::vector<float> const &material_scales() const { return m_materialscales; }

    // tile coordinates covering a world position
    std::int32_t tile_x( double const X ) const { return tile_of( X ); }
    std::int32_t tile_z( double const Z ) const { return tile_of( Z ); }

    // the terrain selection rules the file was cooked under
    std::uint32_t
        selection() const { return m_header.selection; }

    bool
        contains( std::int32_t const X, std::int32_t const Z ) const {
            return m_index.find( tile_key( X, Z ) ) != m_index.end(); }

    // description of one tile, or nullptr when the file does not hold it
    tile_description const *
        find( std::int32_t const X, std::int32_t const Z ) const {
            auto const lookup { m_index.find( tile_key( X, Z ) ) };
            return lookup == m_index.end() ? nullptr : &m_descriptions[ lookup->second ]; }

    // world height of a stored sample; Value must not be nodata
    float
        height( std::uint16_t const Value ) const {
            return m_header.heightbias + static_cast<float>( Value ) * m_header.heightscale; }

    // decodes one mip level of one tile into the caller's buffers, which are resized to
    // the level's sample count. returns false when the tile is absent or the payload is
    // damaged; the buffers are then left untouched
    bool
        read_level(
            std::int32_t const X, std::int32_t const Z, std::uint32_t const Level,
            std::vector<std::uint16_t> &Heights, std::vector<std::uint8_t> &Materials ) {

            if( false == m_open ) { return false; }
            if( Level >= m_header.miplevels ) { return false; }
            auto const lookup { m_index.find( tile_key( X, Z ) ) };
            if( lookup == m_index.end() ) { return false; }
            auto const &entry { m_entries[ lookup->second ] };

            if( false == decode( entry ) ) { return false; }

            auto const side { samples_at_level( m_header.tilesamples, Level ) };
            auto const samples { static_cast<std::size_t>( side ) * side };
            auto const offset { static_cast<std::size_t>( level_offset( m_header.tilesamples, Level ) ) };
            if( offset + samples * 3u > m_plain.size() ) { return false; }

            Heights.resize( samples );
            Materials.resize( samples );
            std::memcpy( Heights.data(), m_plain.data() + offset, samples * sizeof( std::uint16_t ) );
            std::memcpy( Materials.data(), m_plain.data() + offset + samples * sizeof( std::uint16_t ), samples );
            return true;
        }

private:
    std::int32_t
        tile_of( double const World ) const {
            return static_cast<std::int32_t>( std::floor( World / static_cast<double>( tilesize() ) ) ); }

    bool
        read_at( std::uint64_t const Offset, void *Target, std::size_t const Bytes ) {
            if( Bytes == 0 ) { return true; }
            m_file.seekg( static_cast<std::streamoff>( Offset ), std::ios::beg );
            m_file.read( static_cast<char *>( Target ), static_cast<std::streamsize>( Bytes ) );
            return m_file.good(); }

    bool
        close_failed() {
            close();
            return false; }

    // keeps the most recently decoded payload, because a caller walking the mip chain of
    // one tile would otherwise decompress it once per level
    bool
        decode( tile_entry const &Entry ) {

            if( ( true == m_decoded ) && ( m_decodedoffset == Entry.offset ) ) { return true; }
            m_decoded = false;

            m_stored.resize( Entry.storedsize );
            if( false == read_at( Entry.offset, m_stored.data(), Entry.storedsize ) ) { return false; }

            if( static_cast<codec>( Entry.tilecodec ) == codec::raw ) {
                m_plain.swap( m_stored );
            }
            else {
                m_plain.resize( Entry.plainsize );
                // one context for the life of the reader: creating one per tile costs more than
                // decoding a small tile does
                if( m_context == nullptr ) { m_context = ZSTD_createDCtx(); }
                auto const written {
                    ZSTD_decompressDCtx( m_context, m_plain.data(), m_plain.size(), m_stored.data(), m_stored.size() ) };
                if( ( 0 != ZSTD_isError( written ) ) || ( written != Entry.plainsize ) ) { return false; }
            }

            m_decoded = true;
            m_decodedoffset = Entry.offset;
            return true;
        }

    std::ifstream m_file;
    ZSTD_DCtx *m_context { nullptr };
    file_header m_header {};
    std::vector<tile_entry> m_entries;
    std::vector<tile_description> m_descriptions;
    std::unordered_map<std::int64_t, std::size_t> m_index;
    std::vector<std::string> m_materials;
    std::vector<float> m_materialscales;
    std::vector<std::uint8_t> m_stored, m_plain;
    std::uint64_t m_decodedoffset { 0 };
    bool m_decoded { false };
    bool m_open { false };
};

} // namespace heightfield
