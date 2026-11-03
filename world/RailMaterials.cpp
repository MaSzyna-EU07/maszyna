/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "world/TrackRails.h"
#include "rendering/renderer.h"
#include "stb/stb_image.h"

namespace track_rails {

material_handle cut_material( material_handle const Material ) {

    static auto const unrolled { std::getenv( "EU07_EDITOR_UNROLLED" ) != nullptr };
    return unrolled ? GfxRenderer->Fetch_Material( "editor_unrolled" ) : GfxRenderer->Matte_Material( Material );
}

material_handle tint_material( material_handle const Material, glm::vec3 const &Colour ) {

    return GfxRenderer->Tinted_Material( Material, Colour );
}

namespace {

struct colour_mean {
    double sum { 0.0 };
    glm::dvec3 colour { 0.0 };
    std::size_t count { 0 };

    void add( float const Red, float const Green, float const Blue ) {
        sum += 0.2126 * Red + 0.7152 * Green + 0.0722 * Blue;
        colour += glm::pow( glm::dvec3{ Red, Green, Blue } / 255.0, glm::dvec3{ 2.2 } );
        ++count; }
};

std::pair<int, int> band_columns( glm::vec2 const &Band, int const Width ) {

    auto const first { std::clamp( static_cast<int>( std::floor( std::min( Band.x, Band.y ) * Width ) ), 0, Width - 1 ) };
    auto const last { std::clamp( static_cast<int>( std::ceil( std::max( Band.x, Band.y ) * Width ) ) - 1, first, Width - 1 ) };
    return { first, last };
}

glm::vec3 unpack_565( unsigned const Color ) {

    return glm::vec3{ ( ( Color >> 11 ) & 31 ) * 255.f / 31.f, ( ( Color >> 5 ) & 63 ) * 255.f / 63.f, ( Color & 31 ) * 255.f / 31.f };
}

void add_dxt_block( unsigned char const *Block, bool const Alpha, int const Column, int const First, int const Last, colour_mean &Mean ) {

    auto const low { static_cast<unsigned>( Block[ 0 ] | Block[ 1 ] << 8 ) };
    auto const top { static_cast<unsigned>( Block[ 2 ] | Block[ 3 ] << 8 ) };
    auto const opaque { Alpha || low > top };
    glm::vec3 const colors[ 4 ] {
        unpack_565( low ),
        unpack_565( top ),
        opaque ? ( 2.f * unpack_565( low ) + unpack_565( top ) ) / 3.f : 0.5f * ( unpack_565( low ) + unpack_565( top ) ),
        opaque ? ( unpack_565( low ) + 2.f * unpack_565( top ) ) / 3.f : glm::vec3{ 0.f } };
    auto const bits { static_cast<std::uint32_t>( Block[ 4 ] ) | static_cast<std::uint32_t>( Block[ 5 ] ) << 8 | static_cast<std::uint32_t>( Block[ 6 ] ) << 16 | static_cast<std::uint32_t>( Block[ 7 ] ) << 24 };
    for( int texel = 0; texel < 16; ++texel ) {
        auto const x { Column * 4 + texel % 4 };
        auto const index { ( bits >> ( 2 * texel ) ) & 3 };
        if( x < First || x > Last || ( false == opaque && index == 3 ) ) {
            continue;
        }
        Mean.add( colors[ index ].r, colors[ index ].g, colors[ index ].b );
    }
}

void sample_dds( std::string const &File, glm::vec2 const &Band, colour_mean &Mean ) {

    std::ifstream input( File, std::ios::binary );
    std::array<unsigned char, 128> header {};
    if( !input.read( reinterpret_cast<char *>( header.data() ), header.size() ) || false == std::equal( header.begin(), header.begin() + 4, "DDS " ) ) {
        return;
    }
    auto const word = [&]( std::size_t const Offset ) {
        return static_cast<std::uint32_t>( header[ Offset ] ) | static_cast<std::uint32_t>( header[ Offset + 1 ] ) << 8 | static_cast<std::uint32_t>( header[ Offset + 2 ] ) << 16 | static_cast<std::uint32_t>( header[ Offset + 3 ] ) << 24; };
    auto const height { static_cast<int>( word( 12 ) ) };
    auto const width { static_cast<int>( word( 16 ) ) };
    auto const fourcc { word( 84 ) };
    auto const blocksize { fourcc == 0x31545844u ? 8 : ( fourcc == 0x33545844u || fourcc == 0x35545844u ) ? 16 : 0 };
    if( blocksize == 0 || width <= 0 || height <= 0 ) {
        return;
    }
    auto const wide { ( width + 3 ) / 4 };
    auto const high { ( height + 3 ) / 4 };
    std::vector<unsigned char> data( static_cast<std::size_t>( wide ) * high * blocksize );
    if( !input.read( reinterpret_cast<char *>( data.data() ), data.size() ) ) {
        return;
    }
    auto const [ first, last ] { band_columns( Band, width ) };
    for( int row = 0; row < high; ++row ) {
        for( int column = first / 4; column <= last / 4; ++column ) {
            add_dxt_block( &data[ ( static_cast<std::size_t>( row ) * wide + column ) * blocksize + ( blocksize - 8 ) ], blocksize == 16, column, first, last, Mean );
        }
    }
}

void sample_image( std::string const &File, glm::vec2 const &Band, colour_mean &Mean ) {

    int width, height, components;
    auto *pixels { stbi_load( File.c_str(), &width, &height, &components, 4 ) };
    if( pixels == nullptr ) {
        return;
    }
    auto const [ first, last ] { band_columns( Band, width ) };
    for( int y = 0; y < height; ++y ) {
        for( int x = first; x <= last; ++x ) {
            auto const *pixel { &pixels[ ( static_cast<std::size_t>( y ) * width + x ) * 4 ] };
            Mean.add( pixel[ 0 ], pixel[ 1 ], pixel[ 2 ] );
        }
    }
    stbi_image_free( pixels );
}

} // namespace

std::pair<float, glm::vec3> const &tread_sample( material_handle const Material, glm::vec2 const &Band ) {

    static std::map<std::tuple<material_handle, float, float>, std::pair<float, glm::vec3>> samples;
    auto const key { std::make_tuple( Material, Band.x, Band.y ) };
    if( auto const lookup { samples.find( key ) }; lookup != samples.end() ) {
        return lookup->second;
    }
    auto &result { samples[ key ] };
    result = { -1.f, glm::vec3{ 0.f } };
    if( Material == null_handle ) {
        return result;
    }
    auto const texture { GfxRenderer->Material( Material )->GetTexture( 0 ) };
    if( texture == null_handle ) {
        return result;
    }
    auto const &image { GfxRenderer->Texture( texture ) };
    auto const type { std::string( image.get_type() ) };
    auto const file { std::string( image.get_name() ) + type };
    colour_mean mean;
    if( type == ".dds" ) {
        sample_dds( file, Band, mean );
    }
    else {
        sample_image( file, Band, mean );
    }
    if( mean.count > 0 ) {
        result = { static_cast<float>( mean.sum / mean.count ), glm::vec3{ mean.colour / static_cast<double>( mean.count ) } };
    }
    return result;
}

float tread_brightness( material_handle const Material, glm::vec2 const &Band ) {

    return tread_sample( Material, Band ).first;
}

glm::vec3 tread_colour( material_handle const Material, glm::vec2 const &Band ) {

    return tread_sample( Material, Band ).second;
}

} // namespace track_rails

material_handle TTrack::rail_material( int const Path ) const {

    return ( Path == 1 && eType == tt_Switch ? m_material2 : m_material1 );
}

glm::vec2 TTrack::tread_band() const {

    glm::vec2 band { 0.45f, 0.55f };
    if( m_profiles.empty() ) {
        return band;
    }
    auto const &outline { track_rail_profile( m_profile1.second ) };
    auto const count { outline.size() / 2 };
    auto top { -1e9f };
    for( std::size_t i = 0; i < count; ++i ) {
        top = std::max( top, outline[ i ].position.y );
    }
    glm::vec2 found { 1e9f, -1e9f };
    for( std::size_t i = 0; i < count; ++i ) {
        if( outline[ i ].position.y > top - 0.0005f ) {
            found = { std::min( found.x, outline[ i ].texture.s ), std::max( found.y, outline[ i ].texture.s ) };
        }
    }
    if( found.x > found.y ) {
        return band;
    }
    auto const middle { 0.5f * ( found.x + found.y ) };
    auto const half { std::max( 0.5f * ( found.y - found.x ), 0.01f ) };
    return { middle - half, middle + half };
}

material_handle TTrack::shared_tread( int const Path, TTrack const &Other, int const Otherpath ) const {

    auto const own { rail_material( Path ) };
    auto const theirs { Other.rail_material( Otherpath ) };
    if( theirs == null_handle || theirs == own ) {
        return own;
    }
    return ( track_rails::tread_brightness( theirs, Other.tread_band() ) > track_rails::tread_brightness( own, tread_band() ) ? theirs : own );
}
