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
#include <vector>
#include "scene/quantizedmeshreader.h"

export module eu07.rendering.terrainmesharena;

export {

// One vertex buffer and one index buffer for the whole terrain.
//
// Tiles are put in and taken out as the camera moves, so each buffer is handed out in pieces and
// the pieces are given back. What this buys is the drawing: every resident tile lives in the same
// two buffers behind the same vertex array, so a frame is one glMultiDrawElements rather than a
// bind and a draw per tile. Vertices are measured from one point for the whole terrain, which is
// what makes that possible.
//
// The buffers grow when a tile will not fit and nothing can be freed; they never shrink.
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

    bool create();
    void destroy();

    // puts a tile's mesh in. returns a piece that is not held when it did not fit
    piece put( quantizedmesh::reader::tile_data const &Tile );
    void take( piece &Piece );

    // bound for drawing; the index buffer goes with it
    std::uint32_t vertexarray() const { return m_vao; }

    std::size_t vertexbytes() const { return m_vertexroom * sizeof( quantizedmesh::render_vertex ); }
    std::size_t indexbytes() const { return m_indexroom * sizeof( std::uint32_t ); }
    std::size_t bytes() const { return vertexbytes() + indexbytes(); }
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

    std::uint32_t m_vao { 0 };
    std::uint32_t m_vertexbuffer { 0 };
    std::uint32_t m_indexbuffer { 0 };
    std::uint32_t m_vertexroom { 0 };
    std::uint32_t m_indexroom { 0 };
    span_list m_freevertices;
    span_list m_freeindices;
    std::size_t m_verticesheld { 0 };
};

}  // export
