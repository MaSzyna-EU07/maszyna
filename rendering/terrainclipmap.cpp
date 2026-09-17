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
#include "scene/heightfieldreader.h"

module eu07.rendering.terrainclipmap;
import eu07.glm;
import eu07.utilities.logs;
import eu07.model.texture;


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
    if( false == m_ground.create( m_reader.materials(), m_reader.material_scales() ) ) {
        close();
        return false;
    }

    for( std::uint32_t level = 0; level < m_reader.levels(); ++level ) {
        m_pools.emplace_back( std::make_unique<terrain_tile_pool>( m_reader.samples_per_side( level ) ) );
    }
    m_instances.resize( m_reader.levels() );
    m_instancebuffer.emplace();
    ::glGenTextures( 1, &m_instancetexture );
    opengl_texture::reset_unit_cache();

    m_vao.emplace();
    resolve_uniforms();
    m_ready = true;

    WriteLog(
        "Terrain: " + Path + ", " + std::to_string( m_reader.tiles().size() ) + " tiles, "
        + std::to_string( m_reader.levels() ) + " levels, grid "
        + std::to_string( m_reader.gridstep() ) + " m, tile " + std::to_string( m_reader.tilesize() ) + " m" );

    return true;
}

// uniform locations do not change once the program is linked, so they are looked up here
// rather than per draw
void
terrain_clipmap::resolve_uniforms() {

    auto const program { static_cast<GLuint>( *m_shader ) };
    m_uniforms.samplestep = ::glGetUniformLocation( program, "samplestep" );
    m_uniforms.side = ::glGetUniformLocation( program, "side" );
    m_uniforms.morph = ::glGetUniformLocation( program, "morph" );
    m_uniforms.heightbias = ::glGetUniformLocation( program, "heightbias" );
    m_uniforms.heightscale = ::glGetUniformLocation( program, "heightscale" );
    m_uniforms.nodata = ::glGetUniformLocation( program, "nodata" );
    m_uniforms.heights = ::glGetUniformLocation( program, "heights" );
    m_uniforms.materials = ::glGetUniformLocation( program, "materials" );
    m_uniforms.instances = ::glGetUniformLocation( program, "instances" );
    m_uniforms.ground = ::glGetUniformLocation( program, "ground" );
    m_uniforms.materialtable = ::glGetUniformLocation( program, "materialtable" );
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
    m_tiles.clear();
    m_pools.clear();
    m_indices.clear();
    m_indexcounts.clear();
    m_instances.clear();
    m_instancebuffer.reset();
    if( m_instancetexture != 0 ) { ::glDeleteTextures( 1, &m_instancetexture ); m_instancetexture = 0; }
    m_ground.destroy();
    m_vao.reset();
    m_shader.reset();
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


std::uint32_t
terrain_clipmap::level_for( double const Distance ) const {

    if( Distance <= m_finest ) { return 0; }
    auto const steps { static_cast<std::uint32_t>( std::log2( Distance / m_finest ) ) + 1 };
    return std::min( steps, m_reader.levels() - 1 );
}

void
terrain_clipmap::upload( terrain_tile_loader::payload const &Tile ) {

    auto const tilekey { key( Tile.tile.x, Tile.tile.z ) };
    auto const existing { m_tiles.find( tilekey ) };
    if( existing != m_tiles.end() ) {
        // a level change moves the tile to another pool
        retire( existing->second );
        m_tiles.erase( existing );
    }

    auto &pool { *m_pools[ Tile.tile.level ] };
    resident_tile tile;
    tile.x = Tile.tile.x;
    tile.z = Tile.tile.z;
    tile.level = Tile.tile.level;
    tile.slot = pool.acquire();
    tile.lastused = m_frame;
    pool.upload( tile.slot, Tile.heights.data(), Tile.materials.data() );

    if( auto const *description { m_reader.find( Tile.tile.x, Tile.tile.z ) } ) {
        tile.minheight = description->minheight;
        tile.maxheight = description->maxheight;
    }

    ++m_stats.uploaded;
    m_tiles.emplace( tilekey, tile );
}

void
terrain_clipmap::retire( resident_tile const &Tile ) {

    m_pools[ Tile.level ]->release( Tile.slot );
}

void
terrain_clipmap::detail( double const Pixelsperunit, double const Pixels ) {

    m_finestwanted = terrain_finest_range( m_reader.gridstep(), m_reader.tilesize(), Pixelsperunit, Pixels );
}

void
terrain_clipmap::scan( glm::dvec3 const &Viewpoint ) {

    m_scanpoint = Viewpoint;
    m_scanrange = m_range;
    m_finest = ( m_finestwanted > 0.0 ? m_finestwanted : terrain_finest_tiles * m_reader.tilesize() );
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
    // a zoom or a resized window changes the levels; a change under a percent is not worth
    // redoing the scan for
    auto const detailchanged {
        ( m_finestwanted > 0.0 ) && ( std::abs( m_finestwanted - m_finest ) > 0.01 * m_finest ) };
    if( ( m_range != m_scanrange )
     || ( true == detailchanged )
     || ( moved > m_reader.tilesize() * terrain_rescan_tiles ) ) {
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
    m_stats.texturebytes = 0;
    for( auto const &pool : m_pools ) { m_stats.texturebytes += pool->bytes(); }
    report_residency();
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
        + std::to_string( m_stats.uploaded ) + " uploads so far, "
        + std::to_string( m_stats.drawn ) + " of " + std::to_string( m_stats.inview ) + " in range drawn, finest level out to "
        + std::to_string( static_cast<int>( m_finest ) ) + " m" );
}

void
terrain_clipmap::render( glm::dvec3 const &Viewpoint, visibility const &Visible ) {

    if( ( false == m_ready ) || ( true == m_tiles.empty() ) ) { return; }

    // textures that finished loading since the last frame go into the ground array first
    m_ground.update();

    // what each level draws: the tiles touched by the last update, and seen. a tile's bounding
    // sphere holds the whole tile, morphing included, since morphing only moves vertices
    // towards samples of the same tile
    auto const tilesize { static_cast<double>( m_reader.tilesize() ) };
    for( auto &instances : m_instances ) { instances.clear(); }
    for( auto const & [ tilekey, tile ] : m_tiles ) {
        if( tile.lastused != m_frame ) { continue; }
        auto const halfheight { 0.5 * ( tile.maxheight - tile.minheight ) };
        glm::dvec3 const centre {
            ( tile.x + 0.5 ) * tilesize, 0.5 * ( tile.minheight + tile.maxheight ), ( tile.z + 0.5 ) * tilesize };
        auto const radius {
            static_cast<float>( std::sqrt( 0.5 * tilesize * tilesize + halfheight * halfheight ) ) };
        if( false == Visible( centre, radius ) ) { continue; }
        // relative to the viewpoint, so that the shader never sees a coordinate large enough
        // for single precision to matter
        m_instances[ tile.level ].insert(
            m_instances[ tile.level ].end(),
            { static_cast<float>( tile.x * tilesize - Viewpoint.x ),
              static_cast<float>( -Viewpoint.y ) - m_depthbias,
              static_cast<float>( tile.z * tilesize - Viewpoint.z ),
              static_cast<float>( tile.slot ),
              static_cast<float>( tile.x * tilesize ),
              static_cast<float>( tile.z * tilesize ),
              0.f, 0.f } );
    }

    m_shader->bind();
    m_vao->bind();

    ::glUniform1f( m_uniforms.heightbias, m_reader.height( 0 ) );
    ::glUniform1f( m_uniforms.heightscale, m_reader.height( 1 ) - m_reader.height( 0 ) );
    ::glUniform1ui( m_uniforms.nodata, heightfield::nodata );
    ::glUniform1i( m_uniforms.heights, 0 );
    ::glUniform1i( m_uniforms.materials, 1 );
    ::glUniform1i( m_uniforms.ground, 2 );
    ::glUniform1i( m_uniforms.materialtable, 3 );
    ::glUniform1i( m_uniforms.instances, 4 );

    ::glActiveTexture( GL_TEXTURE2 );
    ::glBindTexture( GL_TEXTURE_2D_ARRAY, m_ground.textures() );
    ::glActiveTexture( GL_TEXTURE3 );
    ::glBindTexture( GL_TEXTURE_2D, m_ground.table() );

    std::size_t drawn { 0 };
    auto const gridstep { m_reader.gridstep() };
    for( std::uint32_t level = 0; level < m_instances.size(); ++level ) {

        auto const &instances { m_instances[ level ] };
        if( true == instances.empty() ) { continue; }
        auto const count { instances.size() / 8 };
        drawn += count;

        m_instancebuffer->allocate(
            gl::buffer::TEXTURE_BUFFER, static_cast<GLsizeiptr>( instances.size() * sizeof( float ) ), GL_STREAM_DRAW );
        m_instancebuffer->upload(
            gl::buffer::TEXTURE_BUFFER, instances.data(), 0, static_cast<GLsizeiptr>( instances.size() * sizeof( float ) ) );
        ::glActiveTexture( GL_TEXTURE4 );
        ::glBindTexture( GL_TEXTURE_BUFFER, m_instancetexture );
        ::glTexBuffer( GL_TEXTURE_BUFFER, GL_RGBA32F, static_cast<GLuint>( *m_instancebuffer ) );

        auto const &pool { *m_pools[ level ] };
        ::glActiveTexture( GL_TEXTURE0 );
        ::glBindTexture( GL_TEXTURE_2D_ARRAY, pool.heights() );
        ::glActiveTexture( GL_TEXTURE1 );
        ::glBindTexture( GL_TEXTURE_2D_ARRAY, pool.materials() );

        ::glUniform1f( m_uniforms.samplestep, gridstep * static_cast<float>( 1u << level ) );
        ::glUniform1i( m_uniforms.side, static_cast<GLint>( pool.side() ) );
        // the band over which this level turns into the next. the coarsest level has no next
        // one, so its band lies beyond anything that is ever drawn
        if( level + 1 < m_reader.levels() ) {
            ::glUniform2f(
                m_uniforms.morph,
                static_cast<float>( terrain_morph_begin( level, m_finest, tilesize ) ),
                static_cast<float>( terrain_morph_end( level, m_finest, tilesize ) ) );
        }
        else {
            ::glUniform2f( m_uniforms.morph, 1e30f, 2e30f );
        }

        m_indices[ level ]->bind( gl::buffer::ELEMENT_ARRAY_BUFFER );
        ::glDrawElementsInstanced(
            GL_TRIANGLES, static_cast<GLsizei>( m_indexcounts[ level ] ), GL_UNSIGNED_INT, nullptr,
            static_cast<GLsizei>( count ) );
    }
    m_stats.drawn = drawn;

    gl::buffer::unbind( gl::buffer::TEXTURE_BUFFER );
    ::glActiveTexture( GL_TEXTURE0 );
    opengl_texture::reset_unit_cache();
    m_vao->unbind();
    gl::program::bind( 0 );
}
