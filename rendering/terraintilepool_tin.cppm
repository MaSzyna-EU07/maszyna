/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

export module eu07.rendering.terraintilepool_tin;
import eu07.gl.buffer;
import eu07.glm;

export {

// GPU storage for TIN terrain tiles - vertex and index buffers.
// Replaces texture array approach used by heightfield.
//
// Each tile gets its own VBO/IBO pair. Tiles are uploaded as they arrive from the loader
// and freed when they leave range. Budget is vertex/index count rather than texture memory.
class terrain_tile_pool_tin {

public:
    // Vertex format for terrain
    struct vertex {
        float x, y, z;     // Position (relative to tile origin)
        float u, v;        // Texture coordinates
        std::uint8_t material;  // Material index
        std::uint8_t pad[3];    // Alignment to 24 bytes
    };
    
    static_assert( sizeof( vertex ) == 24 );
    
    // Handle to uploaded tile
    struct tile_handle {
        std::uint32_t vbo_id { 0 };
        std::uint32_t ibo_id { 0 };
        std::uint32_t vao_id { 0 };
        std::uint32_t vertex_count { 0 };
        std::uint32_t index_count { 0 };
        glm::dvec3 origin { 0.0 };  // World origin for camera-relative rendering
        
        bool valid() const { return vbo_id != 0; }
    };
    
    terrain_tile_pool_tin();
    ~terrain_tile_pool_tin();
    
    terrain_tile_pool_tin( terrain_tile_pool_tin const & ) = delete;
    terrain_tile_pool_tin &operator=( terrain_tile_pool_tin const & ) = delete;
    
    // Upload tile data to GPU
    tile_handle upload(
        std::vector<double> const &positions_x,
        std::vector<double> const &positions_y,
        std::vector<double> const &positions_z,
        std::vector<float> const &uvs_u,
        std::vector<float> const &uvs_v,
        std::vector<std::uint8_t> const &materials,
        std::vector<std::uint32_t> const &indices,
        glm::dvec3 const &origin  // Tile center for relative positioning
    );
    
    // Free tile from GPU
    void free( tile_handle &Handle );
    
    // Statistics
    std::size_t allocated_vertices() const { return m_allocated_vertices; }
    std::size_t allocated_indices() const { return m_allocated_indices; }
    std::size_t allocated_bytes() const {
        return m_allocated_vertices * sizeof( vertex ) + m_allocated_indices * sizeof( std::uint32_t );
    }

private:
    std::size_t m_allocated_vertices { 0 };
    std::size_t m_allocated_indices { 0 };
};

}  // export
