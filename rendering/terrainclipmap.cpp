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
    if( ( false == m_arena.create( false ) ) || ( false == m_widearena.create( true ) ) ) {
        ErrorLog( "Terrain: cannot allocate the terrain buffers" );
        close();
        return false;
    }

    try {
        gl::shader vertex( "terrainmesh.vert" );
        gl::shader geometry( "terrainmesh.geom" );
        gl::shader fragment( "terrainmesh.frag" );
        m_shader.emplace(
            std::vector<std::reference_wrapper<gl::shader const>>( { vertex, geometry, fragment } ) );
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
        + std::to_string( m_reader.tilecount() ) + " tiles, "
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
    m_maydescend.clear();
    m_arena.destroy();
    m_widearena.destroy();
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

float
terrain_clipmap::leveledge( std::uint32_t const Level ) const {

    return m_table.edge( Level );
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

// The distance to the nearest point of the tile's box, not to its centre: a coarse tile is
// kilometres across, and the one the camera stands on has its centre far outside any draw range.
//
// The height counts as much as the other two. Measuring on the ground plane alone makes the tile
// underfoot nought metres away from a camera a kilometre above it, and nought metres asks for the
// finest level there is - so a view from the air drew the whole ground at full detail, which is
// more tiles than the buffers hold, and what would not fit came out as holes. The tile's own height
// range is not known until it is read, so the terrain's is used for all of them: too near rather
// than too far, which errs towards detail.
double
terrain_clipmap::distance_to( quantizedmesh::tile_address const &Tile, glm::dvec3 const &Viewpoint ) const {

    double west { 0.0 }, north { 0.0 }, side { 0.0 };
    square( Tile, west, north, side );
    auto const nearestx { std::clamp( Viewpoint.x, west, west + side ) };
    auto const nearestz { std::clamp( Viewpoint.z, north, north + side ) };
    auto const nearesty { std::clamp( Viewpoint.y, m_table.lowest, m_table.highest ) };
    return glm::length( glm::dvec3 {
        nearestx - Viewpoint.x, nearesty - Viewpoint.y, nearestz - Viewpoint.z } );
}

bool
terrain_clipmap::present( quantizedmesh::tile_address const &Tile ) const {

    return m_reader.contains( Tile );
}

// Whether the ground a tile stands for is close enough to the real thing. The tile's error, at its
// distance, covers so many pixels; while that is fewer than the detail asks for there is nothing to
// be had by splitting it. The tile the camera stands on is always split, whatever its error, so that
// what is underfoot is the ground as the scenery drew it.
//
// What is asked is the tile's own error and its own triangles, out of the archive's index, not the
// worst in its level. The two are far apart: a level's figure is set by whichever tile in it holds
// the steepest ground, and with that one figure standing for all of them a flat field twenty
// kilometres out was split as finely as the valley wall that set it.
bool
terrain_clipmap::good_enough( quantizedmesh::tile_address const &Tile, double const Distance ) const {

    if( Tile.level + 1 >= m_levels ) { return true; }
    if( Distance <= 0.0 ) { return false; }
    auto const measure { m_reader.measure( Tile ) };
    return terrain_level_enough(
        measure.error, measure.edge, Distance, m_pixelsperunit, m_detail );
}

// Walks down to the levels the screen wants and asks the loader for whatever of them is not on the
// card - every tile on the way down, not only the deepest. Asking one level at a time is what a first
// attempt at this did, and it stops dead: a tile whose children are all resident asks for nothing,
// while what it is waiting for is their children. And a level fetched only once its children turn out
// to be late is fetched too late to cover anything.
//
// Returns whether this tile's ground can be drawn at all: either everything below it is there, or the
// tile itself is, and then it stands in for the lot. That distinction is the whole point. Answering
// only "is the subtree complete" makes one missing tile twenty kilometres away answer no for its
// parent, and so for its parent's parent, all the way to the root - and then the entire terrain is
// drawn as the root tile until it arrives. Which is what flickering while the camera moved was.
bool
terrain_clipmap::need( quantizedmesh::tile_address const &Tile, glm::dvec3 const &Viewpoint ) {

    if( false == present( Tile ) ) { return true; }   // no ground here to draw

    auto const distance { distance_to( Tile, Viewpoint ) };
    if( distance > m_range ) { return true; }         // nothing wanted this far out

    auto const key { quantizedmesh::tile_key( Tile ) };
    auto const resident { m_tiles.contains( key ) };
    if( false == resident ) { ask_for( Tile, distance ); }

    if( true == good_enough( Tile, distance ) ) {
        m_maydescend.emplace( key, false );
        return resident;
    }

    auto descend { true };
    for( std::int32_t stepz = 0; stepz < 2; ++stepz ) {
        for( std::int32_t stepx = 0; stepx < 2; ++stepx ) {
            // every child is asked, so the whole cut is queued at once rather than a level a frame
            descend = need( { Tile.level + 1, Tile.x * 2 + stepx, Tile.z * 2 + stepz }, Viewpoint ) && descend;
        }
    }
    m_maydescend.emplace( key, descend );
    return descend || resident;
}

void
terrain_clipmap::choose( quantizedmesh::tile_address const &Tile, glm::dvec3 const &Viewpoint ) {

    if( false == present( Tile ) ) { return; }

    auto const distance { distance_to( Tile, Viewpoint ) };
    if( distance > m_range ) { return; }

    auto const key { quantizedmesh::tile_key( Tile ) };
    auto const descend { m_maydescend.find( key ) };
    if( ( descend != m_maydescend.end() ) && ( true == descend->second )
     && ( false == good_enough( Tile, distance ) ) ) {
        // the whole cut under this tile is on the card: draw that instead, and hold on to this one,
        // which is what covers this ground again as soon as the camera moves
        m_kept.push_back( key );
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
    m_kept.clear();
    m_wishes.clear();
    m_asked.clear();
    m_maydescend.clear();

    need( { 0, 0, 0 }, Viewpoint );
    choose( { 0, 0, 0 }, Viewpoint );

    // Coarsest first, and within a level nearest first. What covers ground has to arrive before what
    // refines it: a fine tile read ahead of the level standing in for it leaves a hole for as long as
    // the queue takes, and the ground under the camera is still first among equals.
    std::sort( m_wishes.begin(), m_wishes.end(),
        []( auto const &Left, auto const &Right ) {
            if( Left.second.level != Right.second.level ) { return Left.second.level < Right.second.level; }
            return Left.first < Right.first; } );
    m_wanted.clear();
    m_wanted.reserve( m_wishes.size() );
    for( auto const &wish : m_wishes ) { m_wanted.push_back( wish.second ); }
    m_loader.want( m_wanted );

    // A walk that settles on a handful of tiles where the one before had a hundred is the ground going
    // coarse for a frame: somewhere a tile is missing and every level between it and the root is
    // missing too, so the root is the only thing left to stand in. Rare, and worth knowing about, so it
    // says which tiles it was waiting for.
    if( ( m_stats.chosen > 8 ) && ( m_chosen.size() * 4 < m_stats.chosen ) && ( false == m_wishes.empty() ) ) {
        std::string waiting;
        for( std::size_t index = 0; ( index < m_wishes.size() ) && ( index < 6 ); ++index ) {
            auto const &tile { m_wishes[ index ].second };
            waiting += " " + std::to_string( tile.level ) + "/" + std::to_string( tile.x ) + "/"
                + std::to_string( tile.z ) + " at " + std::to_string( static_cast<int>( m_wishes[ index ].first ) ) + " m";
        }
        WriteLog( "Terrain: the walk fell from " + std::to_string( m_stats.chosen ) + " tiles to "
            + std::to_string( m_chosen.size() ) + ", waiting for" + waiting );
    }
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

    // the narrow arena first; only what will not fit in it costs four bytes an index
    auto const wide { Tile.data.vertices.size() > m_arena.vertexlimit() };
    auto piece { wide ? m_widearena.put( Tile.data ) : m_arena.put( Tile.data ) };
    if( false == piece.held() ) { return; }

    resident_tile tile;
    tile.piece = piece;
    tile.wide = wide;
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

    // Measured against what the tiles take up, not against what the buffers were allocated at. The
    // allocation only ever grows - gl cannot hand a buffer back in pieces - so gating on it means
    // nothing is let go until the buffers reach the cap, and from then on everything is let go every
    // frame and read again. That is how three thousand tiles came to be held for a working set of
    // forty, and why it stopped exactly at the cap.
    if( m_arena.usedbytes() + m_widearena.usedbytes() <= m_budget ) { return; }

    std::vector<std::pair<std::uint64_t, std::int64_t>> aged;
    aged.reserve( m_tiles.size() );
    for( auto const &[ key, tile ] : m_tiles ) {
        if( tile.lastused != m_frame ) { aged.emplace_back( tile.lastused, key ); }
    }
    if( true == aged.empty() ) { return; }
    std::sort( aged.begin(), aged.end() );

    // down to well under the cap rather than just under it, so that this settles instead of running
    // every frame
    auto const target { m_budget * 3 / 4 };
    std::size_t letgo { 0 };
    for( auto const &[ when, key ] : aged ) {
        if( m_arena.usedbytes() + m_widearena.usedbytes() <= target ) { break; }
        auto const found { m_tiles.find( key ) };
        if( found == m_tiles.end() ) { continue; }
        if( true == found->second.wide ) { m_widearena.take( found->second.piece ); }
        else                             { m_arena.take( found->second.piece ); }
        m_tiles.erase( found );
        ++letgo;
    }
    if( letgo > 0 ) {
        WriteLog( "Terrain: over budget, let go of " + std::to_string( letgo ) + " tiles, now holding "
            + std::to_string( ( m_arena.usedbytes() + m_widearena.usedbytes() ) / 1048576 ) + " MB in "
            + std::to_string( ( m_arena.roombytes() + m_widearena.roombytes() ) / 1048576 ) + " MB of buffers" );
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

    // what the walk settled on is wanted now, and is what eviction spares. how many of them end up
    // drawn at each level is counted by the draw itself, which is the one that knows
    for( auto const key : m_chosen ) {
        auto const found { m_tiles.find( key ) };
        if( found == m_tiles.end() ) { continue; }
        found->second.lastused = m_frame;
    }
    // the levels above what is drawn are wanted too, as the cover for wherever the camera turns next
    for( auto const key : m_kept ) {
        auto const found { m_tiles.find( key ) };
        if( found != m_tiles.end() ) { found->second.lastused = m_frame; }
    }

    retire_unused();
    m_stats.resident = m_tiles.size();
    m_stats.gpubytes = m_arena.usedbytes() + m_widearena.usedbytes();
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
        + std::to_string( m_stats.triangles / 1000 ) + "k triangles out to "
        + std::to_string( static_cast<int>( m_stats.furthest ) ) + " m, levels "
        + [ this ]() {
            std::string counts;
            for( std::uint32_t level = 0; level < m_levels; ++level ) {
                counts += ( level > 0 ? "/" : "" ) + std::to_string( m_stats.perlevel[ level ] );
            }
            return counts; }() );
}

void
terrain_clipmap::render( glm::dvec3 const &Viewpoint, visibility const &Visible, bool const Mainview ) {

    if( ( false == m_ready ) || ( true == m_chosen.empty() ) ) { return; }

    // the ground textures the engine has finished loading since the last frame
    m_ground.update();

    for( auto &list : m_counts ) { list.clear(); }
    for( auto &list : m_offsets ) { list.clear(); }
    for( auto &list : m_bases ) { list.clear(); }
    std::size_t triangles { 0 };
    std::array<std::size_t, terrain_maxlevels> pertile {};
    std::array<std::size_t, terrain_maxlevels> pertriangle {};
    auto furthest { 0.0 };
    for( auto const key : m_chosen ) {
        auto const found { m_tiles.find( key ) };
        if( found == m_tiles.end() ) { continue; }
        auto const &tile { found->second };
        if( false == Visible( tile.centre, tile.radius ) ) { continue; }
        auto const level { std::min<std::size_t>( tile.level, terrain_maxlevels - 1 ) };
        ++pertile[ level ];
        pertriangle[ level ] += tile.piece.indices / 3;
        furthest = std::max(
            furthest,
            glm::length( glm::dvec2 { tile.centre.x - Viewpoint.x, tile.centre.z - Viewpoint.z } ) );
        auto const which { tile.wide ? 1 : 0 };
        auto const width { tile.wide ? sizeof( std::uint32_t ) : sizeof( std::uint16_t ) };
        m_counts[ which ].push_back( static_cast<std::int32_t>( tile.piece.indices ) );
        m_offsets[ which ].push_back(
            reinterpret_cast<void const *>( static_cast<std::uintptr_t>( tile.piece.firstindex ) * width ) );
        m_bases[ which ].push_back( static_cast<std::int32_t>( tile.piece.firstvertex ) );
        triangles += tile.piece.indices / 3;
    }
    if( true == Mainview ) {
        m_stats.drawn = m_counts[ 0 ].size() + m_counts[ 1 ].size();
        m_stats.triangles = triangles;
        m_stats.perlevel = pertile;
        m_stats.trianglesperlevel = pertriangle;
        m_stats.furthest = furthest;
    }
    if( ( true == m_counts[ 0 ].empty() ) && ( true == m_counts[ 1 ].empty() ) ) { return; }

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

    if( false == m_counts[ 0 ].empty() ) {
        m_arena.bind();
        ::glMultiDrawElementsBaseVertex(
            GL_TRIANGLES, m_counts[ 0 ].data(), GL_UNSIGNED_SHORT,
            const_cast<void const **>( m_offsets[ 0 ].data() ),
            static_cast<GLsizei>( m_counts[ 0 ].size() ), m_bases[ 0 ].data() );
        m_arena.unbind();
    }
    if( false == m_counts[ 1 ].empty() ) {
        m_widearena.bind();
        ::glMultiDrawElementsBaseVertex(
            GL_TRIANGLES, m_counts[ 1 ].data(), GL_UNSIGNED_INT,
            const_cast<void const **>( m_offsets[ 1 ].data() ),
            static_cast<GLsizei>( m_counts[ 1 ].size() ), m_bases[ 1 ].data() );
        m_widearena.unbind();
    }

    ::glDisable( GL_POLYGON_OFFSET_FILL );
    ::glActiveTexture( GL_TEXTURE1 );
    ::glBindTexture( GL_TEXTURE_2D, 0 );
    ::glActiveTexture( GL_TEXTURE0 );
    ::glBindTexture( GL_TEXTURE_2D_ARRAY, 0 );
    opengl_texture::reset_unit_cache();
    gl::program::bind( 0 );
}
