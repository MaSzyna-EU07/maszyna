/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>

namespace scene::eu7 {
namespace detail {

[[nodiscard]] constexpr std::uint32_t
make_id4( char const a, char const b, char const c, char const d ) noexcept {
    return (
        ( static_cast<std::uint32_t>( static_cast<unsigned char>( d ) ) << 24u ) |
        ( static_cast<std::uint32_t>( static_cast<unsigned char>( c ) ) << 16u ) |
        ( static_cast<std::uint32_t>( static_cast<unsigned char>( b ) ) << 8u ) |
        static_cast<std::uint32_t>( static_cast<unsigned char>( a ) ) );
}

} // namespace detail

constexpr std::uint32_t kEu7Magic { detail::make_id4( 'E', 'U', '7', 'B' ) };
constexpr std::uint32_t kEu7VersionV4 { 4 };
constexpr std::uint32_t kEu7VersionV5 { 5 };
constexpr std::uint32_t kEu7VersionV6 { 6 };
constexpr std::uint32_t kEu7VersionV7 { 7 };
constexpr std::uint32_t kEu7VersionV8 { 8 };

constexpr std::uint32_t kChunkStrs { detail::make_id4( 'S', 'T', 'R', 'S' ) };
constexpr std::uint32_t kChunkIncl { detail::make_id4( 'I', 'N', 'C', 'L' ) };
constexpr std::uint32_t kChunkTrak { detail::make_id4( 'T', 'R', 'A', 'K' ) };
constexpr std::uint32_t kChunkTrac { detail::make_id4( 'T', 'R', 'A', 'C' ) };
constexpr std::uint32_t kChunkPwrs { detail::make_id4( 'P', 'W', 'R', 'S' ) };
constexpr std::uint32_t kChunkTerr { detail::make_id4( 'T', 'E', 'R', 'R' ) };
constexpr std::uint32_t kChunkMesh { detail::make_id4( 'M', 'E', 'S', 'H' ) };
constexpr std::uint32_t kChunkLine { detail::make_id4( 'L', 'I', 'N', 'E' ) };
constexpr std::uint32_t kChunkModl { detail::make_id4( 'M', 'O', 'D', 'L' ) };
constexpr std::uint32_t kChunkMemc { detail::make_id4( 'M', 'E', 'M', 'C' ) };
constexpr std::uint32_t kChunkLaun { detail::make_id4( 'L', 'A', 'U', 'N' ) };
constexpr std::uint32_t kChunkDynm { detail::make_id4( 'D', 'Y', 'N', 'M' ) };
constexpr std::uint32_t kChunkSond { detail::make_id4( 'S', 'O', 'N', 'D' ) };
constexpr std::uint32_t kChunkTrset { detail::make_id4( 'T', 'R', 'S', 'E' ) };
constexpr std::uint32_t kChunkEvnt { detail::make_id4( 'E', 'V', 'N', 'T' ) };
constexpr std::uint32_t kChunkFint { detail::make_id4( 'F', 'I', 'N', 'T' ) };
constexpr std::uint32_t kChunkPlac { detail::make_id4( 'P', 'L', 'A', 'C' ) };

// Indeks sekcji 1 km -> offset w chunku PACK (EU7B v7+).
constexpr std::uint32_t kChunkPidx { detail::make_id4( 'P', 'I', 'D', 'X' ) };
// MODL per sekcja, world-space; payload = concat sekcji (offsety z PIDX).
constexpr std::uint32_t kChunkPack { detail::make_id4( 'P', 'A', 'C', 'K' ) };
// Wspolne definicje modeli (EU7B v8+).
constexpr std::uint32_t kChunkProt { detail::make_id4( 'P', 'R', 'O', 'T' ) };

constexpr std::uint8_t kPackSectionFormatV8 { 1 };
constexpr std::uint8_t kPackSectionFormatV9 { 2 };
constexpr std::uint8_t kPackSectionFormatV10 { 3 };
constexpr std::size_t kPackSectionChunkModels { 512 };

} // namespace scene::eu7
