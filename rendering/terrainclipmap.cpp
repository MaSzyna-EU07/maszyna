/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
#include <algorithm>
#include <cmath>
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "glad/glad.h"

#include "scene/heightfieldformat.h"
#include "global_include/interfaces/ITexture_macros.h"
#include "scene/heightfieldreader.h"

module eu07.rendering.terrainclipmap;
import eu07.glm;
import eu07.utilities.logs;
import eu07.rendering.renderer;
import eu07.model.material;

namespace {


// NOTE: glTexImage2D rather than glTexStorage2D. Immutable storage arrives with OpenGL
// 4.2 and this renderer holds a 3.3 context, where the entry point is simply not there -
// calling it jumps through a null pointer instead of failing in any visible way
GLuint
create_texture( GLenum const Internalformat, GLenum const Type, GLsizei const Side, void const *Data ) {

    GLuint texture { 0 };
    ::glGenTextures( 1, &texture );
    ::glBindTexture( GL_TEXTURE_2D, texture );
    // integer samples carry the cooker's values, no-data included, so nothing may filter them
    ::glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
    ::glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
    ::glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
    ::glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
    ::glTexImage2D( GL_TEXTURE_2D, 0, Internalformat, Side, Side, 0, GL_RED_INTEGER, Type, Data );
    return texture;
}

} // namespace

terrain_clipmap::~terrain_clipmap() {
    close();
}

bool
terrain_clipmap::open( std::string const &Path ) {

    close();

    if( ( false == m_reader.open( Path ) )
     || ( false == m_loader.open( Path ) ) ) {
        ErrorLog( "Terrain: cannot open heightfield \"" + Path + "\"" );
        close();
        return false;
    }

    m_path = Path;
    build_indices();
    build_palette();
    resolve_materials();

    try {
        gl::shader vertex( "terrainclipmap.vert" );
        gl::shader fragment( "terrainclipmap.frag" );
        m_shader.emplace( std::vector<std::reference_wrapper<gl::shader const>>( { vertex, fragment } ) );
    }
    catch( gl::shader_exception const &error ) {
        ErrorLog( std::string( "Terrain: shader failed, " ) + error.what() );
        close();
        return false;
    }

    m_vao.emplace();
    resolve_uniforms();
    m_ready = true;

    WriteLog(
        "Terrain: " + Path + ", " + std::to_string( m_reader.tiles().size() ) + " tiles, "
        + std::to_string( m_reader.levels() ) + " levels, grid "
        + std::to_string( m_reader.gridstep() ) + " m, tile " + std::to_string( m_reader.tilesize() ) + " m" );

    return true;
}

// uniform locations do not change once the program is linked, so they are looked up
// here rather than per tile per frame
void
terrain_clipmap::resolve_uniforms() {

    auto const program { static_cast<GLuint>( *m_shader ) };
    m_uniforms.tileorigin = ::glGetUniformLocation( program, "tileorigin" );
    m_uniforms.samplestep = ::glGetUniformLocation( program, "samplestep" );
    m_uniforms.heightbias = ::glGetUniformLocation( program, "heightbias" );
    m_uniforms.heightscale = ::glGetUniformLocation( program, "heightscale" );
    m_uniforms.nodata = ::glGetUniformLocation( program, "nodata" );
    m_uniforms.side = ::glGetUniformLocation( program, "side" );
    m_uniforms.heights = ::glGetUniformLocation( program, "heights" );
    m_uniforms.materials = ::glGetUniformLocation( program, "materials" );
    m_uniforms.palette = ::glGetUniformLocation( program, "palette" );
    m_uniforms.groundtexture = ::glGetUniformLocation( program, "ground" );
    m_uniforms.groundscale = ::glGetUniformLocation( program, "groundscale" );
    m_uniforms.drawnmaterial = ::glGetUniformLocation( program, "drawnmaterial" );
    m_uniforms.tileworld = ::glGetUniformLocation( program, "tileworld" );
    m_uniforms.hastexture = ::glGetUniformLocation( program, "hastexture" );
    m_uniforms.fallback = ::glGetUniformLocation( program, "fallback" );
}

void
terrain_clipmap::close() {

    // the loader first, so nothing arrives for tiles that are being torn down
    m_loader.close();
    m_wanted.clear();
    m_arrived.clear();
    m_inrange.clear();
    m_wantedlevel.clear();
    m_scanrange = -1.f;
    for( auto & [ tilekey, tile ] : m_tiles ) { retire( tile ); }
    m_tiles.clear();
    m_indices.clear();
    m_indexcounts.clear();
    m_vao.reset();
    m_shader.reset();
    m_palette.clear();
    m_reader.close();
    m_stats = {};
    m_ready = false;
}

// index buffers, one per mip level. the grid they describe is the same shape at every
// level, only smaller, and the vertices themselves never exist in memory: the shader
// derives a sample's row and column from gl_VertexID
void
terrain_clipmap::build_indices() {

    m_indices.resize( m_reader.levels() );
    m_indexcounts.resize( m_reader.levels() );

    std::vector<std::uint32_t> indices;
    for( std::uint32_t level = 0; level < m_reader.levels(); ++level ) {

        auto const side { m_reader.samples_per_side( level ) };
        indices.clear();
        indices.reserve( static_cast<std::size_t>( side - 1 ) * ( side - 1 ) * 6 );

        for( std::uint32_t row = 0; row + 1 < side; ++row ) {
            for( std::uint32_t column = 0; column + 1 < side; ++column ) {
                auto const topleft { row * side + column };
                auto const topright { topleft + 1 };
                auto const bottomleft { topleft + side };
                auto const bottomright { bottomleft + 1 };
                indices.push_back( topleft ); indices.push_back( bottomleft ); indices.push_back( topright );
                indices.push_back( topright ); indices.push_back( bottomleft ); indices.push_back( bottomright );
            }
        }

        m_indexcounts[ level ] = static_cast<std::uint32_t>( indices.size() );
        m_indices[ level ] = std::make_unique<gl::buffer>();
        m_indices[ level ]->allocate(
            gl::buffer::ELEMENT_ARRAY_BUFFER, indices.size() * sizeof( std::uint32_t ), GL_STATIC_DRAW );
        m_indices[ level ]->upload(
            gl::buffer::ELEMENT_ARRAY_BUFFER, indices.data(), 0, indices.size() * sizeof( std::uint32_t ) );
    }
}

// placeholder colours, one per cooked material, so that the shape of the terrain and the
// division into surfaces can be judged before the real materials are wired in
void
terrain_clipmap::build_palette() {

    m_palette.clear();
    m_palette.reserve( std::max<std::size_t>( 1, m_reader.materials().size() ) );
    for( std::size_t index = 0; index < m_reader.materials().size(); ++index ) {
        auto const hue { static_cast<float>( index ) * 0.61803399f };
        auto const phase { ( hue - std::floor( hue ) ) * 6.f };
        auto const rising { phase - std::floor( phase ) };
        glm::vec3 colour { 0.45f, 0.5f, 0.4f };
        switch( static_cast<int>( phase ) ) {
            case 0: colour = { 0.55f, 0.45f + 0.2f * rising, 0.3f }; break;
            case 1: colour = { 0.55f - 0.2f * rising, 0.6f, 0.3f }; break;
            case 2: colour = { 0.35f, 0.6f, 0.3f + 0.2f * rising }; break;
            case 3: colour = { 0.35f, 0.6f - 0.15f * rising, 0.5f }; break;
            case 4: colour = { 0.35f + 0.2f * rising, 0.45f, 0.5f }; break;
            default: colour = { 0.55f, 0.45f, 0.5f - 0.15f * rising }; break;
        }
        m_palette.push_back( colour );
    }
    if( true == m_palette.empty() ) { m_palette.push_back( { 0.45f, 0.5f, 0.4f } ); }
}

std::uint32_t
terrain_clipmap::level_for( double const Distance ) const {

    if( Distance <= terrain_finest_range ) { return 0; }
    auto const steps { static_cast<std::uint32_t>( std::log2( Distance / terrain_finest_range ) ) + 1 };
    return std::min( steps, m_reader.levels() - 1 );
}

void
terrain_clipmap::upload( terrain_tile_loader::payload const &Tile ) {

    auto const tilekey { key( Tile.tile.x, Tile.tile.z ) };
    auto const side { m_reader.samples_per_side( Tile.tile.level ) };

    auto const existing { m_tiles.find( tilekey ) };
    if( existing != m_tiles.end() ) {
        // a level change means new textures: the sample count differs
        retire( existing->second );
        m_tiles.erase( existing );
    }

    resident_tile tile;
    tile.x = Tile.tile.x;
    tile.z = Tile.tile.z;
    tile.level = Tile.tile.level;
    tile.side = side;
    tile.lastused = m_frame;
    tile.heighttexture = create_texture(
        GL_R16UI, GL_UNSIGNED_SHORT, static_cast<GLsizei>( side ), Tile.heights.data() );
    tile.materialtexture = create_texture(
        GL_R8UI, GL_UNSIGNED_BYTE, static_cast<GLsizei>( side ), Tile.materials.data() );

    // the tile is drawn once per material it contains, so the set is worked out here
    // rather than rescanned every frame
    std::array<bool, 256> present {};
    for( auto const material : Tile.materials ) { present[ material ] = true; }
    for( std::size_t material = 0; material < present.size(); ++material ) {
        if( true == present[ material ] ) { tile.materials.push_back( static_cast<std::uint8_t>( material ) ); }
    }

    if( auto const *description { m_reader.find( Tile.tile.x, Tile.tile.z ) } ) {
        tile.minheight = description->minheight;
        tile.maxheight = description->maxheight;
    }

    m_stats.texturebytes += static_cast<std::size_t>( side ) * side * 3u;
    ++m_stats.uploaded;
    m_tiles.emplace( tilekey, std::move( tile ) );
}

void
terrain_clipmap::retire( resident_tile &Tile ) {

    if( Tile.heighttexture != 0 ) { ::glDeleteTextures( 1, &Tile.heighttexture ); Tile.heighttexture = 0; }
    if( Tile.materialtexture != 0 ) { ::glDeleteTextures( 1, &Tile.materialtexture ); Tile.materialtexture = 0; }
    auto const bytes { static_cast<std::size_t>( Tile.side ) * Tile.side * 3u };
    m_stats.texturebytes -= std::min( m_stats.texturebytes, bytes );
}

void
terrain_clipmap::scan( glm::dvec3 const &Viewpoint ) {

    m_scanpoint = Viewpoint;
    m_scanrange = m_range;
    m_inrange.clear();
    m_wantedlevel.clear();

    auto const tilesize { static_cast<double>( m_reader.tilesize() ) };
    auto const reach { std::max( 1, static_cast<std::int32_t>( m_range / tilesize ) + 1 ) };
    auto const centrex { m_reader.tile_x( Viewpoint.x ) };
    auto const centrez { m_reader.tile_z( Viewpoint.z ) };
    std::vector<std::pair<double, terrain_tile_loader::request>> wishes;

    for( std::int32_t z = centrez - reach; z <= centrez + reach; ++z ) {
        for( std::int32_t x = centrex - reach; x <= centrex + reach; ++x ) {

            if( false == m_reader.contains( x, z ) ) { continue; }

            // distance to the nearest point of the tile, not to its centre. a regional
            // field has tiles kilometres across, and a tile the camera is standing on
            // can have its centre far outside the draw range
            auto const west { x * tilesize };
            auto const north { z * tilesize };
            auto const nearestx { std::clamp( Viewpoint.x, west, west + tilesize ) };
            auto const nearestz { std::clamp( Viewpoint.z, north, north + tilesize ) };
            auto const distance {
                glm::length( glm::dvec2 { nearestx - Viewpoint.x, nearestz - Viewpoint.z } ) };
            if( distance > m_range ) { continue; }

            auto const tilekey { key( x, z ) };
            auto const level { level_for( distance ) };
            m_inrange.push_back( { tilekey, level } );
            m_wantedlevel.emplace( tilekey, level );

            auto const resident { m_tiles.find( tilekey ) };
            if( ( resident == m_tiles.end() ) || ( resident->second.level != level ) ) {
                wishes.push_back( { distance, { x, z, level } } );
            }
        }
    }

    // nearest first, so the ground under the camera is never waiting behind the horizon
    std::sort( wishes.begin(), wishes.end(),
        []( auto const &Left, auto const &Right ) { return Left.first < Right.first; } );
    m_wanted.clear();
    for( auto const &wish : wishes ) { m_wanted.push_back( wish.second ); }
    m_loader.want( m_wanted );
}

void
terrain_clipmap::update( glm::dvec3 const &Viewpoint ) {

    if( false == m_ready ) { return; }

    ++m_frame;

    auto const moved {
        glm::length( glm::dvec2 { Viewpoint.x - m_scanpoint.x, Viewpoint.z - m_scanpoint.z } ) };
    if( ( m_range != m_scanrange )
     || ( moved > m_reader.tilesize() * 0.25 ) ) {
        scan( Viewpoint );
    }

    // the tiles in range are touched every frame, which is what render() draws and what
    // eviction spares
    m_stats.inview = 0;
    m_stats.wanted = 0;
    m_stats.perlevel.fill( 0 );
    for( auto const &tile : m_inrange ) {
        auto const resident { m_tiles.find( tile.key ) };
        if( resident == m_tiles.end() ) {
            ++m_stats.wanted;
            continue;
        }
        // still wanted, at whatever level it has until the right one arrives
        resident->second.lastused = m_frame;
        ++m_stats.inview;
        ++m_stats.perlevel[ std::min<std::size_t>( resident->second.level, m_stats.perlevel.size() - 1 ) ];
        if( resident->second.level != tile.level ) { ++m_stats.wanted; }
    }

    m_arrived.clear();
    m_loader.collect( m_arrived, m_uploadsperframe );
    for( auto const &tile : m_arrived ) {
        if( false == tile.valid ) { continue; }
        // a tile read for where the camera was is put up only if nothing of it is on the gpu
        // yet - some level is better than a hole. otherwise it is dropped, and the level
        // wanted now is already queued
        auto const tilekey { key( tile.tile.x, tile.tile.z ) };
        auto const wanted { m_wantedlevel.find( tilekey ) };
        auto const stillwanted { ( wanted != m_wantedlevel.end() ) && ( wanted->second == tile.tile.level ) };
        if( stillwanted || ( false == m_tiles.contains( tilekey ) ) ) {
            upload( tile );
            // put up this frame, so drawn this frame
            m_tiles.find( tilekey )->second.lastused = m_frame;
        }
        else {
            ++m_stats.dropped;
        }
    }

    if( ( m_tiles.size() > m_budget ) && ( m_tiles.size() > m_stats.inview ) ) {
        // over budget, and not everything is in view: drop whatever has gone longest without
        // being asked for
        std::vector<std::pair<std::uint64_t, std::int64_t>> aged;
        for( auto const & [ tilekey, tile ] : m_tiles ) {
            if( tile.lastused != m_frame ) { aged.emplace_back( tile.lastused, tilekey ); }
        }
        auto const excess { std::min( aged.size(), m_tiles.size() - m_budget ) };
        std::partial_sort( aged.begin(), aged.begin() + excess, aged.end() );
        for( std::size_t index = 0; index < excess; ++index ) {
            auto const victim { m_tiles.find( aged[ index ].second ) };
            retire( victim->second );
            m_tiles.erase( victim );
        }
    }

    m_stats.resident = m_tiles.size();
    report_residency();
}

// the cooked material names are the ones the scenery used, so the engine resolves them
// exactly as it does for any other geometry. only the diffuse texture is taken: the
// terrain has its own shader and none of the rest of a material applies to it
void
terrain_clipmap::resolve_materials() {

    m_materials.clear();
    auto const &names { m_reader.materials() };
    auto const &scales { m_reader.material_scales() };
    m_materials.reserve( names.size() );

    std::size_t resolved { 0 };
    std::string unresolved;
    for( std::size_t index = 0; index < names.size(); ++index ) {

        material_binding binding;
        binding.scale = ( index < scales.size() ? scales[ index ] : 4.f );
        if( binding.scale < 0.01f ) { binding.scale = 4.f; }

        auto const material { GfxRenderer->Fetch_Material( names[ index ] ) };
        if( material != null_handle ) {
            binding.texture = GfxRenderer->Material( material )->GetTexture( 0 );
        }
        if( binding.texture != null_handle ) {
            ++resolved;
        }
        else {
            // drawn in its placeholder colour instead, so a missing texture shows up as
            // an obvious flat patch rather than as black ground
            unresolved += ( unresolved.empty() ? "" : ", " ) + names[ index ];
        }
        m_materials.push_back( binding );
    }

    WriteLog(
        "Terrain: " + std::to_string( resolved ) + " of " + std::to_string( names.size() )
        + " materials resolved to textures" );
    if( false == unresolved.empty() ) {
        WriteLog( "Terrain: no texture for " + unresolved );
    }
}

// residency settles within a second of the camera stopping and then barely moves, so a
// line is written when it shifts by a noticeable amount rather than every frame. that
// makes the warm figure - what the terrain actually costs - readable from the log
void
terrain_clipmap::report_residency() {

    auto const difference {
        m_stats.resident > m_reportedresidency ?
            m_stats.resident - m_reportedresidency :
            m_reportedresidency - m_stats.resident };
    if( ( m_reported == true ) && ( difference < 16 ) ) { return; }

    m_reported = true;
    m_reportedresidency = m_stats.resident;
    WriteLog(
        "Terrain: " + std::to_string( m_stats.resident ) + " tiles resident, "
        + std::to_string( m_stats.texturebytes / 1048576 ) + " MB of samples, "
        + std::to_string( m_stats.uploaded ) + " uploads so far" );
}

void
terrain_clipmap::render( glm::dvec3 const &Viewpoint ) {

    if( ( false == m_ready ) || ( true == m_tiles.empty() ) ) { return; }

    m_shader->bind();
    m_vao->bind();

    ::glUniform1f( m_uniforms.heightbias, m_reader.height( 0 ) );
    ::glUniform1f( m_uniforms.heightscale, m_reader.height( 1 ) - m_reader.height( 0 ) );
    ::glUniform1ui( m_uniforms.nodata, heightfield::nodata );
    ::glUniform1i( m_uniforms.heights, 0 );
    ::glUniform1i( m_uniforms.materials, 1 );
    ::glUniform1i( m_uniforms.groundtexture, 2 );
    ::glUniform3fv(
        m_uniforms.palette, static_cast<GLsizei>( m_palette.size() ), &m_palette.front().x );

    auto const tilesize { static_cast<double>( m_reader.tilesize() ) };
    auto const gridstep { m_reader.gridstep() };

    for( auto const & [ tilekey, tile ] : m_tiles ) {

        if( tile.lastused != m_frame ) { continue; }

        // tile position relative to the viewpoint, so that the shader never sees a
        // coordinate large enough for single precision to matter
        glm::vec3 const origin {
            static_cast<float>( tile.x * tilesize - Viewpoint.x ),
            static_cast<float>( -Viewpoint.y ) - m_depthbias,
            static_cast<float>( tile.z * tilesize - Viewpoint.z ) };
        ::glUniform3fv( m_uniforms.tileorigin, 1, &origin.x );
        ::glUniform1f( m_uniforms.samplestep, gridstep * static_cast<float>( 1u << tile.level ) );
        ::glUniform1i( m_uniforms.side, static_cast<GLint>( tile.side ) );
        ::glUniform2f(
            m_uniforms.tileworld,
            static_cast<float>( tile.x * tilesize ), static_cast<float>( tile.z * tilesize ) );

        ::glActiveTexture( GL_TEXTURE0 );
        ::glBindTexture( GL_TEXTURE_2D, tile.heighttexture );
        ::glActiveTexture( GL_TEXTURE1 );
        ::glBindTexture( GL_TEXTURE_2D, tile.materialtexture );

        m_indices[ tile.level ]->bind( gl::buffer::ELEMENT_ARRAY_BUFFER );

        // one pass per material in the tile: each wants a different texture bound, and
        // a fragment belonging to any other material is dropped
        for( auto const material : tile.materials ) {

            auto const &binding {
                material < m_materials.size() ? m_materials[ material ] : material_binding {} };
            GfxRenderer->Bind_Texture( 2, binding.texture );
            ::glUniform1f( m_uniforms.groundscale, binding.scale );
            ::glUniform1ui( m_uniforms.drawnmaterial, material );
            ::glUniform1i( m_uniforms.hastexture, binding.texture != null_handle ? 1 : 0 );
            auto const &fallback {
                m_palette[ std::min<std::size_t>( material, m_palette.size() - 1 ) ] };
            ::glUniform3fv( m_uniforms.fallback, 1, &fallback.x );

            ::glDrawElements(
                GL_TRIANGLES, static_cast<GLsizei>( m_indexcounts[ tile.level ] ), GL_UNSIGNED_INT, nullptr );
        }
    }

    ::glActiveTexture( GL_TEXTURE0 );
    m_vao->unbind();
    gl::program::bind( 0 );
}
