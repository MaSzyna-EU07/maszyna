/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <vector>
#include "scene/quantizedmeshreader.h"

export module eu07.rendering.terrainmesharena;
import eu07.gl.buffer;
import eu07.gl.vao;

export {

// One vertex buffer and one index buffer for the whole terrain.
//
// Tiles are put in and taken out as the camera moves, so each buffer is handed out in pieces and
// the pieces are given back. What this buys is the drawing: every resident tile lives in the same
// two buffers behind the same vertex array, so a frame is one glMultiDrawElements rather than a
// bind and a draw per tile. Vertices are measured from one point for the whole terrain, which is
// what makes that possible.
//
// Indices are two bytes wide unless the arena is asked for four. An index is a quarter of a vertex but
// there are some six of them per vertex, so they are the larger half of what the terrain costs on the
// card - and a tile of fewer than sixty-five thousand vertices, which is nearly every tile, has no use
// for four bytes. The few that need more get an arena of their own and a draw call of their own.
//
// The buffers grow when a tile will not fit and nothing can be freed; they never shrink.
//
// Everything here goes through gl::vao and gl::buffer rather than through glBindVertexArray and
// glBindBuffer. The engine remembers what it last bound and skips a bind it believes is already in
// place, so a raw bind here leaves it drawing the scenery's geometry through this vertex array. And
// the element buffer binding belongs to whichever vertex array is bound, so writing indices without
// binding this one first overwrites the scenery's.
class terrain_mesh_arena {

public:
    // where a tile sits in the two buffers
    struct piece {
        std::uint32_t firstvertex { 0 };
        std::uint32_t vertices { 0 };
        std::uint32_t firstindex { 0 };
        std::uint32_t indices { 0 };
        bool held() const { return indices != 0; }
    };

    terrain_mesh_arena() = default;
    ~terrain_mesh_arena();
    terrain_mesh_arena( terrain_mesh_arena const & ) = delete;
    terrain_mesh_arena &operator=( terrain_mesh_arena const & ) = delete;

    // Wideindices asks for four-byte indices, for tiles of more than 65536 vertices
    bool create( bool Wideindices );
    void destroy();
    bool wideindices() const { return m_wideindices; }
    // the most vertices a tile may have to fit this arena
    std::uint32_t vertexlimit() const { return m_wideindices ? ( 1u << 24 ) : 65536u; }

    // puts a tile's mesh in. returns a piece that is not held when it did not fit
    piece put( quantizedmesh::reader::tile_data const &Tile );
    void take( piece &Piece );

    // bound for drawing; the index buffer goes with it
    void bind();
    void unbind();

    // what the two buffers were allocated at. it only grows: gl has no way of giving a buffer back
    // except by making a new one, and the pieces already in it would have to move
    std::size_t indexwidth() const { return m_wideindices ? sizeof( std::uint32_t ) : sizeof( std::uint16_t ); }
    std::size_t roombytes() const {
        return m_vertexroom * sizeof( quantizedmesh::render_vertex ) + m_indexroom * indexwidth(); }
    // and what the tiles in them actually take up, which is what a budget has to be measured against
    std::size_t usedbytes() const {
        return m_verticesheld * sizeof( quantizedmesh::render_vertex ) + m_indicesheld * indexwidth(); }
    std::size_t verticesheld() const { return m_verticesheld; }

private:
    // a run of free slots in one of the buffers, kept in order so that neighbours can be joined
    class span_list {
    public:
        void clear() { m_free.clear(); }
        void give( std::uint32_t At, std::uint32_t Count );
        // the first run large enough, or the count when there is none
        bool take( std::uint32_t Count, std::uint32_t &At );
    private:
        std::map<std::uint32_t, std::uint32_t> m_free;   // where to how many
    };

    bool grow_vertices( std::uint32_t Wanted );
    bool grow_indices( std::uint32_t Wanted );

    // attribute places the shader reads, and what one vertex is made of
    void describe_vertices();

    // gl::buffer cannot be moved - it deletes what it holds - so growing one means putting a new one
    // in its place rather than handing it over
    std::optional<gl::vao> m_vao;
    std::unique_ptr<gl::buffer> m_vertexbuffer;
    std::unique_ptr<gl::buffer> m_indexbuffer;
    std::uint32_t m_vertexroom { 0 };
    std::uint32_t m_indexroom { 0 };
    span_list m_freevertices;
    span_list m_freeindices;
    std::size_t m_verticesheld { 0 };
    std::size_t m_indicesheld { 0 };
    bool m_wideindices { false };
    std::vector<std::uint16_t> m_narrow;   // scratch for the common case, so a put allocates nothing
};

}  // export
