/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// On-disk layout of the cooked terrain heightfield. Single source of truth: the cooker
// (tools/terraincook) writes it, the engine reads it, and neither owns a private copy.
// Plain structs and free functions only - no engine types, no allocation, no i/o - so
// that the tool keeps building without the engine and the format stays readable on its own.
//
// File layout:
//   header
//   tile directory, one entry per tile, ordered by morton code
//   material table, each entry a uint16 length, that many name bytes, and a float
//   giving how many metres of ground one repeat of the texture covers
//   tile payloads, at the offsets the directory gives
//
// A tile payload holds a mip chain, finest level first. Level l has
// samples_at_level( tilesamples, l ) values along a side, laid out as that many rows of
// uint16 heights, followed by the same grid of uint8 material indices.

#include <cstdint>

namespace heightfield {

// "EU07HFLD", spelled out so the header needs no string functions
inline constexpr char magic[ 8 ] { 'E', 'U', '0', '7', 'H', 'F', 'L', 'D' };
inline constexpr std::uint32_t version { 5 };
// terrain selection rules, stamped into every cooked file.
//   0 - every triangle node the scenery holds
//   1 - world space triangle nodes only: anything under origin, rotate or scale is an object
//   2 - every triangle as drawn; a triangle stays drawn unless the heightfield shows it, and a
//       triangle bordering a hole always stays drawn
//   3 - as 2, with a hole checked for diagonally and at the coarser levels too
//   4 - the heightfield holds only the triangles it shows, and their supports
//   5 - no triangle refused for its size; each goes to a field with a step suited to it
inline constexpr std::uint32_t selection_rules { 5 };

// height value reserved to mean "this sample carries no terrain"; a cooked map is a
// ribbon inside a much larger rectangle, so most edge tiles are partly empty
inline constexpr std::uint16_t nodata { 0xffff };
// the range the remaining values span, shared by every tile through the header's bias
// and scale. over l204's 107 m of relief this puts a sample within 2 mm of the source
inline constexpr std::uint32_t heightrange { 0xfffe };

enum class codec : std::uint16_t {
    raw = 0,
    zstd = 1
};

#pragma pack( push, 1 )

struct file_header {
    char magic[ 8 ];
    std::uint32_t version;
    // samples along a tile side, not counting the shared edge. a tile therefore stores
    // one more sample in each direction than it owns
    std::uint32_t tilesamples;
    float gridstep;     // metres between samples at the finest level
    float heightbias;   // world height of stored value 0
    float heightscale;  // metres per stored unit
    std::int32_t minx, minz, maxx, maxz; // tile coordinates present in the file, inclusive
    std::uint32_t tilecount;
    std::uint32_t materialcount;
    std::uint32_t miplevels; // levels stored per tile, level 0 being the finest
    // which rules decided what counted as terrain when this was cooked. the layout does
    // not depend on it, so a file with an older value still reads; a heightfield the
    // engine baked by itself is rebaked when it disagrees, one named by the scenery is not
    std::uint32_t selection;
    std::uint32_t reserved;
};

struct tile_entry {
    std::int32_t x, z;
    std::uint64_t offset;      // payload position in the file
    std::uint32_t storedsize;  // bytes occupied in the file
    std::uint32_t plainsize;   // bytes once decoded; equals storedsize for codec::raw
    float minheight, maxheight;
    std::uint32_t coverage;    // samples carrying terrain at the finest level
    std::uint16_t tilecodec;   // codec enumerator
    std::uint16_t reserved;
};

#pragma pack( pop )

static_assert( sizeof( file_header ) == 64 );
static_assert( sizeof( tile_entry ) == 40 );

// identity of a tile in any map keyed by tile coordinates. the cooker, the reader and the
// renderer all key tiles this way, so it is spelled out once
constexpr std::int64_t
tile_key( std::int32_t const X, std::int32_t const Z ) {
    return ( static_cast<std::int64_t>( X ) << 32 ) ^ static_cast<std::uint32_t>( Z );
}

// integer division rounding towards negative infinity. tile and sample coordinates go
// negative, and every place that maps one onto the other has to round the same way for
// neighbouring tiles to share an edge exactly
constexpr std::int64_t
floor_div( std::int64_t const Value, std::int64_t const Divisor ) {
    auto const quotient { Value / Divisor };
    return ( ( Value % Divisor != 0 ) && ( ( Value < 0 ) != ( Divisor < 0 ) ) ) ? quotient - 1 : quotient;
}

// samples along a tile side at the given mip level, shared edge included
constexpr std::uint32_t
samples_at_level( std::uint32_t const Tilesamples, std::uint32_t const Level ) {
    return ( Tilesamples >> Level ) + 1;
}

// bytes one mip level occupies: a grid of heights followed by a grid of material indices
constexpr std::uint64_t
level_bytes( std::uint32_t const Tilesamples, std::uint32_t const Level ) {
    std::uint64_t const side { samples_at_level( Tilesamples, Level ) };
    return side * side * ( sizeof( std::uint16_t ) + sizeof( std::uint8_t ) );
}

// bytes a decoded tile payload occupies, mip chain included
constexpr std::uint64_t
tile_bytes( std::uint32_t const Tilesamples, std::uint32_t const Miplevels ) {
    std::uint64_t total { 0 };
    for( std::uint32_t level = 0; level < Miplevels; ++level ) {
        total += level_bytes( Tilesamples, level );
    }
    return total;
}

// byte offset of a mip level inside a decoded tile payload
constexpr std::uint64_t
level_offset( std::uint32_t const Tilesamples, std::uint32_t const Level ) {
    return tile_bytes( Tilesamples, Level );
}

// deepest level that still has at least Minimumside samples along a side
constexpr std::uint32_t
level_count( std::uint32_t const Tilesamples, std::uint32_t const Minimumside ) {
    std::uint32_t levels { 1 };
    while( ( ( Tilesamples >> levels ) >= 1 )
        && ( samples_at_level( Tilesamples, levels ) >= Minimumside ) ) {
        ++levels;
    }
    return levels;
}

// interleaves tile coordinates so that tiles sorted by this value are read in an order
// that keeps a moving camera reading the file forwards rather than seeking
constexpr std::uint64_t
morton( std::int32_t const X, std::int32_t const Z ) {
    auto const spread = []( std::uint32_t Value ) constexpr -> std::uint64_t {
        std::uint64_t result { Value };
        result = ( result | ( result << 16 ) ) & 0x0000ffff0000ffffull;
        result = ( result | ( result << 8 ) ) & 0x00ff00ff00ff00ffull;
        result = ( result | ( result << 4 ) ) & 0x0f0f0f0f0f0f0f0full;
        result = ( result | ( result << 2 ) ) & 0x3333333333333333ull;
        result = ( result | ( result << 1 ) ) & 0x5555555555555555ull;
        return result; };
    // biased into unsigned so that negative tile coordinates keep their relative order
    std::uint32_t const x { static_cast<std::uint32_t>( X + ( 1 << 20 ) ) };
    std::uint32_t const z { static_cast<std::uint32_t>( Z + ( 1 << 20 ) ) };
    return spread( x ) | ( spread( z ) << 1 );
}

} // namespace heightfield
