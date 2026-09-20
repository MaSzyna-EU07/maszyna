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
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <vector>
#include <glad/glad.h>
#include "scene/quantizedmeshreader.h"

module eu07.rendering.terrainmesharena;
import eu07.gl.buffer;
import eu07.utilities.logs;
import eu07.gl.vao;

namespace {

// what the buffers start at, in vertices and indices, and the most they may reach. a vertex is 16
// bytes and an index 4, so the ceiling is about 500 MB of terrain on the card
constexpr std::uint32_t firstvertices { 1u << 19 };
constexpr std::uint32_t firstindices { 1u << 21 };
constexpr std::uint32_t mostvertices { 1u << 24 };
constexpr std::uint32_t mostindices { 1u << 26 };

constexpr int attribute_position { 0 };
constexpr int attribute_material { 1 };
constexpr int attribute_normal { 2 };

} // anonymous namespace

void
terrain_mesh_arena::span_list::give( std::uint32_t const At, std::uint32_t const Count ) {

    if( Count == 0 ) { return; }
    auto placed { m_free.emplace( At, Count ).first };
    // join what now touches, so that the room a tile gave back can hold a larger one
    auto after { std::next( placed ) };
    if( ( after != m_free.end() ) && ( placed->first + placed->second == after->first ) ) {
        placed->second += after->second;
        m_free.erase( after );
    }
    if( placed != m_free.begin() ) {
        auto before { std::prev( placed ) };
        if( before->first + before->second == placed->first ) {
            before->second += placed->second;
            m_free.erase( placed );
        }
    }
}

bool
terrain_mesh_arena::span_list::take( std::uint32_t const Count, std::uint32_t &At ) {

    for( auto span = m_free.begin(); span != m_free.end(); ++span ) {
        if( span->second < Count ) { continue; }
        At = span->first;
        if( span->second == Count ) { m_free.erase( span ); }
        else {
            auto const left { span->second - Count };
            m_free.erase( span );
            m_free.emplace( At + Count, left );
        }
        return true;
    }
    return false;
}

terrain_mesh_arena::~terrain_mesh_arena() {

    destroy();
}

// The material is handed over as an integer that the vertex stage takes as a float: gl::vao sets
// attributes up one way, and going around it to ask for an integer attribute would put the engine's
// idea of what is bound out of step with what is.
void
terrain_mesh_arena::describe_vertices() {

    auto const stride { static_cast<int>( sizeof( quantizedmesh::render_vertex ) ) };
    m_vao->setup_attrib(
        *m_vertexbuffer, attribute_position, 3, GL_FLOAT, stride,
        static_cast<int>( offsetof( quantizedmesh::render_vertex, x ) ) );
    m_vao->setup_attrib(
        *m_vertexbuffer, attribute_material, 1, GL_UNSIGNED_SHORT, stride,
        static_cast<int>( offsetof( quantizedmesh::render_vertex, material ) ) );
    // the normal, still folded onto its octahedron; the vertex stage unfolds it
    m_vao->setup_attrib(
        *m_vertexbuffer, attribute_normal, 2, GL_UNSIGNED_BYTE, stride,
        static_cast<int>( offsetof( quantizedmesh::render_vertex, normal0 ) ) );
}

bool
terrain_mesh_arena::create( bool const Wideindices ) {

    destroy();
    m_wideindices = Wideindices;
    m_vao.emplace();
    m_vertexbuffer = std::make_unique<gl::buffer>();
    m_indexbuffer = std::make_unique<gl::buffer>();

    m_vao->bind();
    m_vertexbuffer->allocate(
        gl::buffer::ARRAY_BUFFER,
        static_cast<GLsizeiptr>( firstvertices ) * sizeof( quantizedmesh::render_vertex ), GL_DYNAMIC_DRAW );
    m_indexbuffer->allocate(
        gl::buffer::ELEMENT_ARRAY_BUFFER,
        static_cast<GLsizeiptr>( firstindices ) * indexwidth(), GL_DYNAMIC_DRAW );
    describe_vertices();
    m_vao->setup_ebo( *m_indexbuffer );
    unbind();

    m_vertexroom = firstvertices;
    m_indexroom = firstindices;
    m_freevertices.clear();
    m_freeindices.clear();
    m_freevertices.give( 0, m_vertexroom );
    m_freeindices.give( 0, m_indexroom );
    m_verticesheld = 0;
    return true;
}

void
terrain_mesh_arena::destroy() {

    if( true == m_vao.has_value() ) { m_vao->unbind(); }
    gl::buffer::unbind( gl::buffer::ARRAY_BUFFER );
    gl::buffer::unbind( gl::buffer::ELEMENT_ARRAY_BUFFER );
    m_vao.reset();
    m_vertexbuffer.reset();
    m_indexbuffer.reset();
    m_vertexroom = m_indexroom = 0;
    m_freevertices.clear();
    m_freeindices.clear();
    m_verticesheld = 0;
    m_indicesheld = 0;
}

void
terrain_mesh_arena::bind() {

    if( false == m_vao.has_value() ) { return; }
    m_vao->bind();
}

void
terrain_mesh_arena::unbind() {

    if( true == m_vao.has_value() ) { m_vao->unbind(); }
    gl::buffer::unbind( gl::buffer::ARRAY_BUFFER );
    gl::buffer::unbind( gl::buffer::COPY_READ_BUFFER );
    gl::buffer::unbind( gl::buffer::COPY_WRITE_BUFFER );
}

// Growing copies what is there into a larger buffer on the card: the pieces keep their places, so
// nothing has to be read back or sent again.
bool
terrain_mesh_arena::grow_vertices( std::uint32_t const Wanted ) {

    auto room { m_vertexroom };
    while( ( room < m_vertexroom + Wanted ) && ( room < mostvertices ) ) { room *= 2; }
    if( room <= m_vertexroom ) { return false; }

    auto grown { std::make_unique<gl::buffer>() };
    grown->allocate(
        gl::buffer::COPY_WRITE_BUFFER,
        static_cast<GLsizeiptr>( room ) * sizeof( quantizedmesh::render_vertex ), GL_DYNAMIC_DRAW );
    m_vertexbuffer->bind( gl::buffer::COPY_READ_BUFFER );
    ::glCopyBufferSubData( GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0,
        static_cast<GLsizeiptr>( m_vertexroom ) * sizeof( quantizedmesh::render_vertex ) );
    gl::buffer::unbind( gl::buffer::COPY_READ_BUFFER );
    gl::buffer::unbind( gl::buffer::COPY_WRITE_BUFFER );

    // the tracker has to forget the buffer before it is destroyed: gl reuses the name of a deleted
    // buffer, so the next one can come back with the same number, and a bind of it would be skipped as
    // already in place while nothing is actually bound
    gl::buffer::unbind( gl::buffer::ARRAY_BUFFER );
    m_vertexbuffer = std::move( grown );
    m_vao->bind();
    describe_vertices();

    m_freevertices.give( m_vertexroom, room - m_vertexroom );
    WriteLog( "Terrain: vertex buffer grown from " + std::to_string( m_vertexroom ) + " to "
        + std::to_string( room ) + " vertices" );
    m_vertexroom = room;
    return true;
}

bool
terrain_mesh_arena::grow_indices( std::uint32_t const Wanted ) {

    auto room { m_indexroom };
    while( ( room < m_indexroom + Wanted ) && ( room < mostindices ) ) { room *= 2; }
    if( room <= m_indexroom ) { return false; }

    auto grown { std::make_unique<gl::buffer>() };
    grown->allocate(
        gl::buffer::COPY_WRITE_BUFFER,
        static_cast<GLsizeiptr>( room ) * indexwidth(), GL_DYNAMIC_DRAW );
    m_indexbuffer->bind( gl::buffer::COPY_READ_BUFFER );
    ::glCopyBufferSubData( GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0,
        static_cast<GLsizeiptr>( m_indexroom ) * indexwidth() );
    gl::buffer::unbind( gl::buffer::COPY_READ_BUFFER );
    gl::buffer::unbind( gl::buffer::COPY_WRITE_BUFFER );

    gl::buffer::unbind( gl::buffer::ELEMENT_ARRAY_BUFFER );
    m_indexbuffer = std::move( grown );
    m_vao->bind();
    m_vao->setup_ebo( *m_indexbuffer );

    m_freeindices.give( m_indexroom, room - m_indexroom );
    WriteLog( "Terrain: index buffer grown from " + std::to_string( m_indexroom ) + " to "
        + std::to_string( room ) + " indices" );
    m_indexroom = room;
    return true;
}

terrain_mesh_arena::piece
terrain_mesh_arena::put( quantizedmesh::reader::tile_data const &Tile ) {

    piece where;
    if( ( false == m_vao.has_value() ) || ( nullptr == m_vertexbuffer ) || ( true == Tile.vertices.empty() ) || ( true == Tile.indices.empty() ) ) {
        return where;
    }
    if( Tile.vertices.size() > vertexlimit() ) { return where; }
    auto const vertices { static_cast<std::uint32_t>( Tile.vertices.size() ) };
    auto const indices { static_cast<std::uint32_t>( Tile.indices.size() ) };

    std::uint32_t atvertex { 0 };
    if( false == m_freevertices.take( vertices, atvertex ) ) {
        if( false == grow_vertices( vertices ) ) { return where; }
        if( false == m_freevertices.take( vertices, atvertex ) ) { return where; }
    }
    std::uint32_t atindex { 0 };
    if( false == m_freeindices.take( indices, atindex ) ) {
        if( false == grow_indices( indices ) ) {
            m_freevertices.give( atvertex, vertices );
            return where;
        }
        if( false == m_freeindices.take( indices, atindex ) ) {
            m_freevertices.give( atvertex, vertices );
            return where;
        }
    }

    // this vertex array first: which element buffer is bound belongs to it, and writing indices
    // with the scenery's array bound would take its own indices away
    m_vao->bind();
    m_vertexbuffer->upload(
        gl::buffer::ARRAY_BUFFER, Tile.vertices.data(),
        static_cast<int>( atvertex * sizeof( quantizedmesh::render_vertex ) ),
        static_cast<GLsizeiptr>( vertices ) * sizeof( quantizedmesh::render_vertex ) );
    // the indices are stored as the tile had them, counted from its own first vertex; where the tile
    // sits in the buffer is given to the draw as its base vertex
    if( true == m_wideindices ) {
        m_indexbuffer->upload(
            gl::buffer::ELEMENT_ARRAY_BUFFER, Tile.indices.data(),
            static_cast<int>( atindex * sizeof( std::uint32_t ) ),
            static_cast<GLsizeiptr>( indices ) * sizeof( std::uint32_t ) );
    }
    else {
        m_narrow.assign( Tile.indices.begin(), Tile.indices.end() );
        m_indexbuffer->upload(
            gl::buffer::ELEMENT_ARRAY_BUFFER, m_narrow.data(),
            static_cast<int>( atindex * sizeof( std::uint16_t ) ),
            static_cast<GLsizeiptr>( indices ) * sizeof( std::uint16_t ) );
    }
    unbind();

    where.firstvertex = atvertex;
    where.vertices = vertices;
    where.firstindex = atindex;
    where.indices = indices;
    m_verticesheld += vertices;
    m_indicesheld += indices;
    return where;
}

void
terrain_mesh_arena::take( piece &Piece ) {

    if( false == Piece.held() ) { return; }
    m_freevertices.give( Piece.firstvertex, Piece.vertices );
    m_freeindices.give( Piece.firstindex, Piece.indices );
    m_verticesheld -= Piece.vertices;
    m_indicesheld -= Piece.indices;
    Piece = piece {};
}
