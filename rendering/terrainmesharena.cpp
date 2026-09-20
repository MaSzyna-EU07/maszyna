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
#include <vector>
#include <glad/glad.h>
#include "scene/quantizedmeshreader.h"

module eu07.rendering.terrainmesharena;
import eu07.utilities.logs;

namespace {

// what the buffers start at, in vertices and indices, and the most they may reach. a vertex is 24
// bytes and an index 4, so the ceiling is about 400 MB of terrain on the card
constexpr std::uint32_t firstvertices { 1u << 19 };
constexpr std::uint32_t firstindices { 1u << 21 };
constexpr std::uint32_t mostvertices { 1u << 24 };
constexpr std::uint32_t mostindices { 1u << 26 };

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

bool
terrain_mesh_arena::create() {

    destroy();
    ::glGenVertexArrays( 1, &m_vao );
    ::glGenBuffers( 1, &m_vertexbuffer );
    ::glGenBuffers( 1, &m_indexbuffer );
    if( ( m_vao == 0 ) || ( m_vertexbuffer == 0 ) || ( m_indexbuffer == 0 ) ) { destroy(); return false; }

    ::glBindVertexArray( m_vao );
    ::glBindBuffer( GL_ARRAY_BUFFER, m_vertexbuffer );
    ::glBufferData( GL_ARRAY_BUFFER,
        static_cast<GLsizeiptr>( firstvertices ) * sizeof( quantizedmesh::render_vertex ), nullptr, GL_DYNAMIC_DRAW );
    ::glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, m_indexbuffer );
    ::glBufferData( GL_ELEMENT_ARRAY_BUFFER,
        static_cast<GLsizeiptr>( firstindices ) * sizeof( std::uint32_t ), nullptr, GL_DYNAMIC_DRAW );

    auto const stride { static_cast<GLsizei>( sizeof( quantizedmesh::render_vertex ) ) };
    ::glEnableVertexAttribArray( 0 );
    ::glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, stride,
        reinterpret_cast<void const *>( offsetof( quantizedmesh::render_vertex, x ) ) );
    ::glEnableVertexAttribArray( 1 );
    ::glVertexAttribIPointer( 1, 1, GL_UNSIGNED_SHORT, stride,
        reinterpret_cast<void const *>( offsetof( quantizedmesh::render_vertex, material ) ) );
    ::glBindVertexArray( 0 );
    ::glBindBuffer( GL_ARRAY_BUFFER, 0 );
    ::glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, 0 );

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

    if( m_vao != 0 ) { ::glDeleteVertexArrays( 1, &m_vao ); m_vao = 0; }
    if( m_vertexbuffer != 0 ) { ::glDeleteBuffers( 1, &m_vertexbuffer ); m_vertexbuffer = 0; }
    if( m_indexbuffer != 0 ) { ::glDeleteBuffers( 1, &m_indexbuffer ); m_indexbuffer = 0; }
    m_vertexroom = m_indexroom = 0;
    m_freevertices.clear();
    m_freeindices.clear();
    m_verticesheld = 0;
}

// Growing copies what is there into a larger buffer on the card: the pieces keep their places, so
// nothing has to be read back or re-uploaded.
bool
terrain_mesh_arena::grow_vertices( std::uint32_t const Wanted ) {

    auto room { m_vertexroom };
    while( ( room < m_vertexroom + Wanted ) && ( room < mostvertices ) ) { room *= 2; }
    if( room <= m_vertexroom ) { return false; }

    std::uint32_t grown { 0 };
    ::glGenBuffers( 1, &grown );
    if( grown == 0 ) { return false; }
    ::glBindBuffer( GL_ARRAY_BUFFER, grown );
    ::glBufferData( GL_ARRAY_BUFFER,
        static_cast<GLsizeiptr>( room ) * sizeof( quantizedmesh::render_vertex ), nullptr, GL_DYNAMIC_DRAW );
    ::glBindBuffer( GL_COPY_READ_BUFFER, m_vertexbuffer );
    ::glBindBuffer( GL_COPY_WRITE_BUFFER, grown );
    ::glCopyBufferSubData( GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0,
        static_cast<GLsizeiptr>( m_vertexroom ) * sizeof( quantizedmesh::render_vertex ) );
    ::glBindBuffer( GL_COPY_READ_BUFFER, 0 );
    ::glBindBuffer( GL_COPY_WRITE_BUFFER, 0 );
    ::glDeleteBuffers( 1, &m_vertexbuffer );
    m_vertexbuffer = grown;

    ::glBindVertexArray( m_vao );
    ::glBindBuffer( GL_ARRAY_BUFFER, m_vertexbuffer );
    auto const stride { static_cast<GLsizei>( sizeof( quantizedmesh::render_vertex ) ) };
    ::glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, stride,
        reinterpret_cast<void const *>( offsetof( quantizedmesh::render_vertex, x ) ) );
    ::glVertexAttribIPointer( 1, 1, GL_UNSIGNED_SHORT, stride,
        reinterpret_cast<void const *>( offsetof( quantizedmesh::render_vertex, material ) ) );
    ::glBindVertexArray( 0 );
    ::glBindBuffer( GL_ARRAY_BUFFER, 0 );

    m_freevertices.give( m_vertexroom, room - m_vertexroom );
    m_vertexroom = room;
    return true;
}

bool
terrain_mesh_arena::grow_indices( std::uint32_t const Wanted ) {

    auto room { m_indexroom };
    while( ( room < m_indexroom + Wanted ) && ( room < mostindices ) ) { room *= 2; }
    if( room <= m_indexroom ) { return false; }

    std::uint32_t grown { 0 };
    ::glGenBuffers( 1, &grown );
    if( grown == 0 ) { return false; }
    ::glBindBuffer( GL_ARRAY_BUFFER, grown );
    ::glBufferData( GL_ARRAY_BUFFER,
        static_cast<GLsizeiptr>( room ) * sizeof( std::uint32_t ), nullptr, GL_DYNAMIC_DRAW );
    ::glBindBuffer( GL_COPY_READ_BUFFER, m_indexbuffer );
    ::glBindBuffer( GL_COPY_WRITE_BUFFER, grown );
    ::glCopyBufferSubData( GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0,
        static_cast<GLsizeiptr>( m_indexroom ) * sizeof( std::uint32_t ) );
    ::glBindBuffer( GL_COPY_READ_BUFFER, 0 );
    ::glBindBuffer( GL_COPY_WRITE_BUFFER, 0 );
    ::glDeleteBuffers( 1, &m_indexbuffer );
    m_indexbuffer = grown;

    ::glBindVertexArray( m_vao );
    ::glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, m_indexbuffer );
    ::glBindVertexArray( 0 );
    ::glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, 0 );

    m_freeindices.give( m_indexroom, room - m_indexroom );
    m_indexroom = room;
    return true;
}

terrain_mesh_arena::piece
terrain_mesh_arena::put( quantizedmesh::reader::tile_data const &Tile ) {

    piece where;
    if( ( m_vao == 0 ) || ( true == Tile.vertices.empty() ) || ( true == Tile.indices.empty() ) ) {
        return where;
    }
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

    ::glBindBuffer( GL_ARRAY_BUFFER, m_vertexbuffer );
    ::glBufferSubData( GL_ARRAY_BUFFER,
        static_cast<GLintptr>( atvertex ) * sizeof( quantizedmesh::render_vertex ),
        static_cast<GLsizeiptr>( vertices ) * sizeof( quantizedmesh::render_vertex ),
        Tile.vertices.data() );
    ::glBindBuffer( GL_ARRAY_BUFFER, 0 );

    // the indices are stored as the tile had them, counted from its own first vertex; where the
    // tile sits in the buffer is given to the draw as its base vertex
    ::glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, m_indexbuffer );
    ::glBufferSubData( GL_ELEMENT_ARRAY_BUFFER,
        static_cast<GLintptr>( atindex ) * sizeof( std::uint32_t ),
        static_cast<GLsizeiptr>( indices ) * sizeof( std::uint32_t ),
        Tile.indices.data() );
    ::glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, 0 );

    where.firstvertex = atvertex;
    where.vertices = vertices;
    where.firstindex = atindex;
    where.indices = indices;
    m_verticesheld += vertices;
    return where;
}

void
terrain_mesh_arena::take( piece &Piece ) {

    if( false == Piece.held() ) { return; }
    m_freevertices.give( Piece.firstvertex, Piece.vertices );
    m_freeindices.give( Piece.firstindex, Piece.indices );
    m_verticesheld -= Piece.vertices;
    Piece = piece {};
}
