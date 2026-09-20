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
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "glad/glad.h"

#include "scene/quantizedmeshreader.h"

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

    if( false == m_reader.open( Path ) ) {
        ErrorLog( "Terrain: cannot read cooked terrain \"" + Path + "\"" );
        return false;
    }
    m_table = m_reader.table();
    m_levels = std::min<std::uint32_t>( m_table.levels, terrain_maxlevels );
    m_finesttile = static_cast<float>( m_table.tilesize( m_levels - 1 ) );
    // every vertex of every tile is measured from the middle of the terrain, so that the whole of
    // it fits in floats: at ten kilometres a float still holds a millimetre
    m_origin = glm::dvec3 {
        m_table.west + m_table.side * 0.5,
        ( m_table.lowest + m_table.highest ) * 0.5,
        m_table.north + m_table.side * 0.5 };

    if( false == m_loader.open( Path, m_origin.x, m_origin.y, m_origin.z ) ) {
        ErrorLog( "Terrain: cannot open \"" + Path + "\" for reading" );
        close();
        return false;
    }
    if( false == m_arena.create() ) {
        ErrorLog( "Terrain: cannot allocate the terrain buffers" );
        close();
        return false;
    }

    try {
        gl::shader vertex( "terrainmesh.vert" );
        gl::shader fragment( "terrainmesh.frag" );
        m_shader.emplace( std::vector<std::reference_wrapper<gl::shader const>>( { vertex, fragment } ) );
    }
    catch( gl::shader_exception const &error ) {
        ErrorLog( std::string( "Terrain: shader failed, " ) + error.what() );
        close();
        return false;
    }

    std::vector<std::string> names;
    std::vector<float> repeats;
    names.reserve( m_table.materials.size() );
    repeats.reserve( m_table.materials.size() );
    for( auto const &material : m_table.materials ) {
        names.push_back( material.name );
        repeats.push_back( material.repeat );
    }
    if( false == m_ground.create( names, repeats ) ) {
        ErrorLog( "Terrain: ground textures failed" );
        close();
        return false;
    }

    resolve_uniforms();
    m_path = Path;
    m_ready = true;

    WriteLog(
        "Terrain: " + Path + ", " + std::to_string( m_levels ) + " levels, finest tile "
        + std::to_string( static_cast<int>( m_finesttile ) ) + " m over "
        + std::to_string( static_cast<int>( m_table.side ) ) + " m, "
        + std::to_string( m_reader.present().size() ) + " tiles, "
        + std::to_string( m_table.materials.size() ) + " ground materials" );
    return true;
}

// uniform locations do not change once the program is linked, so they are looked up here rather
// than per draw
void
terrain_clipmap::resolve_uniforms() {

    auto const program { static_cast<GLuint>( *m_shader ) };
    m_uniforms.origin = ::glGetUniformLocation( program, "terrainorigin" );
    m_uniforms.world = ::glGetUniformLocation( program, "terrainworld" );
    m_uniforms.ground = ::glGetUniformLocation( program, "ground" );
    m_uniforms.materials = ::glGetUniformLocation( program, "materialtable" );
}

void
terrain_clipmap::close() {

    // the loader first, so nothing arrives for tiles that are being torn down
    m_loader.close();
    m_tiles.clear();
    m_chosen.clear();
    m_wishes.clear();
    m_wanted.clear();
    m_arrived.clear();
    m_asked.clear();
    m_readiness.clear();
    m_arena.destroy();
    m_ground.destroy();
    m_shader.reset();
    m_reader.close();
    m_table = {};
    m_stats = {};
    m_walked = false;
    m_walkrange = -1.f;
    m_ready = false;
    m_path.clear();
}

float
terrain_clipmap::leveltile( std::uint32_t const Level ) const {

    return static_cast<float>( m_table.tilesize( Level ) );
}

float
terrain_clipmap::levelerror( std::uint32_t const Level ) const {

    return m_table.error( Level );
}

void
terrain_clipmap::detail( double const Pixelsperunit, double const Pixels ) {

    m_pixelsperunit = Pixelsperunit;
    m_detail = std::max( 0.1, Pixels );
}

void
terrain_clipmap::square( quantizedmesh::tile_address const &Tile, double &West, double &North, double &Side ) const {

    Side = m_table.tilesize( Tile.level );
    West = m_table.west + Tile.x * Side;
    North = m_table.north + Tile.z * Side;
}

// the distance to the nearest point of the tile's square, not to its centre: a coarse tile is
// kilometres across, and the one the camera stands on has its centre far outside any draw range
double
terrain_clipmap::distance_to( quantizedmesh::tile_address const &Tile, glm::dvec3 const &Viewpoint ) const {

    double west { 0.0 }, north { 0.0 }, side { 0.0 };
    square( Tile, west, north, side );
    auto const nearestx { std::clamp( Viewpoint.x, west, west + side ) };
    auto const nearestz { std::clamp( Viewpoint.z, north, north + side ) };
    return glm::length( glm::dvec2 { nearestx - Viewpoint.x, nearestz - Viewpoint.z } );
}

bool
terrain_clipmap::present( quantizedmesh::tile_address const &Tile ) const {

    return m_reader.present().contains( quantizedmesh::tile_key( Tile ) );
}

// Whether the ground a tile stands for is close enough to the real thing. The tile's error, at its
// distance, covers so many pixels; while that is fewer than the detail asks for there is nothing to
// be had by splitting it. The tile the camera stands on is always split, whatever its error, so that
// what is underfoot is the ground as the scenery drew it.
bool
terrain_clipmap::good_enough( quantizedmesh::tile_address const &Tile, double const Distance ) const {

    if( Tile.level + 1 >= m_levels ) { return true; }
    if( Distance <= 0.0 ) { return false; }
    return terrain_level_enough( m_table.error( Tile.level ), Distance, m_pixelsperunit, m_detail );
}

// Walks down to the levels the screen wants and asks the loader for whatever of them is not on the
// card, deepest tile of the wanted cut included - not only the tiles one level down. Asking one level
// at a time is what a first attempt at this did, and it stops dead: a tile whose children are all
// resident asks for nothing, while what it is waiting for is their children.
//
// Returns whether the cut under this tile, and the tile itself where the cut ends at it, is there. A
// tile whose subtree is not ready yet is asked for as well, so that something coarse covers the
// ground rather than a hole while the rest is read.
bool
terrain_clipmap::need( quantizedmesh::tile_address const &Tile, glm::dvec3 const &Viewpoint ) {

    if( false == present( Tile ) ) { return true; }   // no ground here to draw

    auto const distance { distance_to( Tile, Viewpoint ) };
    if( distance > m_range ) { return true; }         // nothing wanted this far out

    auto const key { quantizedmesh::tile_key( Tile ) };
    auto const resident { m_tiles.contains( key ) };

    if( true == good_enough( Tile, distance ) ) {
        if( false == resident ) { ask_for( Tile, distance ); }
        m_readiness.emplace( key, resident );
        return resident;
    }

    auto ready { true };
    for( std::int32_t stepz = 0; stepz < 2; ++stepz ) {
        for( std::int32_t stepx = 0; stepx < 2; ++stepx ) {
            // every child is asked, so the whole cut is queued at once rather than a level a frame
            ready = need( { Tile.level + 1, Tile.x * 2 + stepx, Tile.z * 2 + stepz }, Viewpoint ) && ready;
        }
    }
    if( ( false == ready ) && ( false == resident ) ) { ask_for( Tile, distance ); }
    m_readiness.emplace( key, ready );
    return ready;
}

void
terrain_clipmap::choose( quantizedmesh::tile_address const &Tile, glm::dvec3 const &Viewpoint ) {

    if( false == present( Tile ) ) { return; }

    auto const distance { distance_to( Tile, Viewpoint ) };
    if( distance > m_range ) { return; }

    auto const key { quantizedmesh::tile_key( Tile ) };
    auto const ready { m_readiness.find( key ) };
    if( ( ready != m_readiness.end() ) && ( true == ready->second )
     && ( false == good_enough( Tile, distance ) ) ) {
        // the whole cut under this tile is on the card: draw that instead
        for( std::int32_t stepz = 0; stepz < 2; ++stepz ) {
            for( std::int32_t stepx = 0; stepx < 2; ++stepx ) {
                choose( { Tile.level + 1, Tile.x * 2 + stepx, Tile.z * 2 + stepz }, Viewpoint );
            }
        }
        return;
    }
    // either this level is good enough, or what is below it is still being read: this tile covers
    // the ground meanwhile, and nothing under it is drawn, so nothing is drawn twice
    if( true == m_tiles.contains( key ) ) { m_chosen.push_back( key ); }
}

void
terrain_clipmap::ask_for( quantizedmesh::tile_address const &Tile, double const Distance ) {

    if( false == m_asked.insert( quantizedmesh::tile_key( Tile ) ).second ) { return; }
    m_wishes.emplace_back( Distance, Tile );
}

void
terrain_clipmap::walk( glm::dvec3 const &Viewpoint ) {

    m_walkpoint = Viewpoint;
    m_walkrange = m_range;
    m_walkdetail = m_detail;
    m_walked = true;
    m_chosen.clear();
    m_wishes.clear();
    m_asked.clear();
    m_readiness.clear();

    need( { 0, 0, 0 }, Viewpoint );
    choose( { 0, 0, 0 }, Viewpoint );

    // nearest first, so the ground under the camera is never waiting behind the horizon
    std::sort( m_wishes.begin(), m_wishes.end(),
        []( auto const &Left, auto const &Right ) { return Left.first < Right.first; } );
    m_wanted.clear();
    m_wanted.reserve( m_wishes.size() );
    for( auto const &wish : m_wishes ) { m_wanted.push_back( wish.second ); }
    m_loader.want( m_wanted );

    m_stats.chosen = m_chosen.size();
    m_stats.wanted = m_wanted.size();
}

void
terrain_clipmap::put_in( terrain_tile_loader::payload const &Tile ) {

    auto const key { quantizedmesh::tile_key( Tile.data.tile ) };
    auto const existing { m_tiles.find( key ) };
    if( existing != m_tiles.end() ) {
        // already there; nothing about a tile changes between reads
        ++m_stats.dropped;
        return;
    }

    auto piece { m_arena.put( Tile.data ) };
    if( false == piece.held() ) { return; }

    resident_tile tile;
    tile.piece = piece;
    tile.centre = glm::dvec3 { Tile.data.centrex, Tile.data.centrey, Tile.data.centrez };
    tile.radius = static_cast<float>( Tile.data.radius );
    tile.level = Tile.data.tile.level;
    tile.lastused = m_frame;
    m_tiles.emplace( key, tile );
    ++m_stats.uploaded;
}

// What is held is what the walk has been choosing or asking for. Once the buffers are fuller than
// the budget, whatever has gone longest without either is given back - never something the walk
// wants now, because a coarse tile it is drawing is what stands in for its children while they are
// read.
void
terrain_clipmap::retire_unused() {

    if( m_arena.bytes() <= m_budget ) { return; }

    std::vector<std::pair<std::uint64_t, std::int64_t>> aged;
    aged.reserve( m_tiles.size() );
    for( auto const &[ key, tile ] : m_tiles ) {
        if( tile.lastused != m_frame ) { aged.emplace_back( tile.lastused, key ); }
    }
    if( true == aged.empty() ) { return; }
    std::sort( aged.begin(), aged.end() );

    // a fifth of what is held, so that this does not run every frame
    auto const letgo { std::max<std::size_t>( 1, m_tiles.size() / 5 ) };
    for( std::size_t index = 0; ( index < letgo ) && ( index < aged.size() ); ++index ) {
        auto const found { m_tiles.find( aged[ index ].second ) };
        if( found == m_tiles.end() ) { continue; }
        m_arena.take( found->second.piece );
        m_tiles.erase( found );
    }
}

void
terrain_clipmap::update( glm::dvec3 const &Viewpoint ) {

    if( false == m_ready ) { return; }
    ++m_frame;

    m_arrived.clear();
    m_loader.collect( m_arrived, m_putsperframe );
    for( auto const &tile : m_arrived ) {
        if( false == tile.valid ) { continue; }
        // a tile read for where the camera was is still put in: it covers ground, and what is
        // wanted now is already queued
        put_in( tile );
    }

    auto const moved {
        glm::length( glm::dvec2 { Viewpoint.x - m_walkpoint.x, Viewpoint.z - m_walkpoint.z } ) };
    auto const detailchanged { std::abs( m_detail - m_walkdetail ) > 0.01 * m_walkdetail };
    if( ( false == m_walked )
     || ( m_range != m_walkrange )
     || ( true == detailchanged )
     || ( moved > m_finesttile * terrain_rescan_tiles )
     || ( false == m_arrived.empty() ) ) {
        // a tile that has just arrived lets the walk go deeper where it was drawing a coarse one
        walk( Viewpoint );
    }

    // what the walk settled on is wanted now, and is what eviction spares
    m_stats.perlevel.fill( 0 );
    for( auto const key : m_chosen ) {
        auto const found { m_tiles.find( key ) };
        if( found == m_tiles.end() ) { continue; }
        found->second.lastused = m_frame;
        ++m_stats.perlevel[ std::min<std::size_t>( found->second.level, m_stats.perlevel.size() - 1 ) ];
    }

    retire_unused();
    m_stats.resident = m_tiles.size();
    m_stats.gpubytes = m_arena.bytes();
    report_residency();
}

// residency settles within a second of the camera stopping and then barely moves, so a line is
// written when it shifts by a noticeable amount rather than every frame. that makes the warm figure
// - what the terrain actually costs - readable from the log
void
terrain_clipmap::report_residency() {

    auto const apart {
        []( std::size_t const Left, std::size_t const Right ) {
            return Left > Right ? Left - Right : Right - Left; } };
    auto const moved {
        ( apart( m_stats.resident, m_reportedresidency ) >= 8 )
     || ( apart( m_stats.drawn, m_reporteddrawn ) >= 4 ) };
    // at most once every couple of seconds, and only when the picture has actually changed: what
    // this is for is the warm figure, what the terrain settles at, not a line per frame
    if( ( m_reported == true ) && ( ( false == moved ) || ( m_frame < m_reportedframe + 120 ) ) ) { return; }

    m_reported = true;
    m_reportedresidency = m_stats.resident;
    m_reporteddrawn = m_stats.drawn;
    m_reportedframe = m_frame;
    WriteLog(
        "Terrain: " + std::to_string( m_stats.resident ) + " tiles resident, "
        + std::to_string( m_stats.gpubytes / 1048576 ) + " MB of mesh, "
        + std::to_string( m_stats.uploaded ) + " put in so far, "
        + std::to_string( m_stats.drawn ) + " of " + std::to_string( m_stats.chosen ) + " chosen drawn, "
        + std::to_string( m_stats.triangles / 1000 ) + "k triangles, levels "
        + [ this ]() {
            std::string counts;
            for( std::uint32_t level = 0; level < m_levels; ++level ) {
                counts += ( level > 0 ? "/" : "" ) + std::to_string( m_stats.perlevel[ level ] );
            }
            return counts; }() );
}

void
terrain_clipmap::render( glm::dvec3 const &Viewpoint, visibility const &Visible ) {

    if( ( false == m_ready ) || ( true == m_chosen.empty() ) ) { return; }

    // the ground textures the engine has finished loading since the last frame
    m_ground.update();

    m_counts.clear();
    m_offsets.clear();
    m_bases.clear();
    std::size_t triangles { 0 };
    for( auto const key : m_chosen ) {
        auto const found { m_tiles.find( key ) };
        if( found == m_tiles.end() ) { continue; }
        auto const &tile { found->second };
        if( false == Visible( tile.centre, tile.radius ) ) { continue; }
        m_counts.push_back( static_cast<std::int32_t>( tile.piece.indices ) );
        m_offsets.push_back(
            reinterpret_cast<void const *>(
                static_cast<std::uintptr_t>( tile.piece.firstindex ) * sizeof( std::uint32_t ) ) );
        m_bases.push_back( static_cast<std::int32_t>( tile.piece.firstvertex ) );
        triangles += tile.piece.indices / 3;
    }
    m_stats.drawn = m_counts.size();
    m_stats.triangles = triangles;
    if( true == m_counts.empty() ) { return; }

    m_shader->bind();
    glm::vec3 const origin {
        static_cast<float>( m_origin.x - Viewpoint.x ),
        static_cast<float>( m_origin.y - Viewpoint.y ),
        static_cast<float>( m_origin.z - Viewpoint.z ) };
    ::glUniform3fv( m_uniforms.origin, 1, &origin.x );
    glm::vec2 const world { static_cast<float>( m_origin.x ), static_cast<float>( m_origin.z ) };
    ::glUniform2fv( m_uniforms.world, 1, &world.x );
    ::glUniform1i( m_uniforms.ground, 0 );
    ::glUniform1i( m_uniforms.materials, 1 );

    ::glActiveTexture( GL_TEXTURE0 );
    ::glBindTexture( GL_TEXTURE_2D_ARRAY, m_ground.textures() );
    ::glActiveTexture( GL_TEXTURE1 );
    ::glBindTexture( GL_TEXTURE_2D, m_ground.table() );

    // the ground loses the depth test to whatever lies on it, and a coarser terrain to a finer one
    // describing the same ground. the offset is negative because depth runs the other way here
    ::glEnable( GL_POLYGON_OFFSET_FILL );
    ::glPolygonOffset(
        -1.f - 2.f * static_cast<float>( m_depthrank ),
        -2.f - 4.f * static_cast<float>( m_depthrank ) );

    ::glBindVertexArray( m_arena.vertexarray() );
    ::glMultiDrawElementsBaseVertex(
        GL_TRIANGLES, m_counts.data(), GL_UNSIGNED_INT,
        const_cast<void const **>( m_offsets.data() ),
        static_cast<GLsizei>( m_counts.size() ), m_bases.data() );
    ::glBindVertexArray( 0 );

    ::glDisable( GL_POLYGON_OFFSET_FILL );
    ::glActiveTexture( GL_TEXTURE1 );
    ::glBindTexture( GL_TEXTURE_2D, 0 );
    ::glActiveTexture( GL_TEXTURE0 );
    ::glBindTexture( GL_TEXTURE_2D_ARRAY, 0 );
    opengl_texture::reset_unit_cache();
    gl::program::bind( 0 );
}
