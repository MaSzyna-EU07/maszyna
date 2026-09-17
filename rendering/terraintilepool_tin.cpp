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
#include <cstdint>
#include <vector>
#include <glad/glad.h>

module eu07.rendering.terraintilepool_tin;
import eu07.glm;

terrain_tile_pool_tin::terrain_tile_pool_tin() = default;

terrain_tile_pool_tin::~terrain_tile_pool_tin() = default;

terrain_tile_pool_tin::tile_handle
terrain_tile_pool_tin::upload(
    std::vector<double> const &positions_x,
    std::vector<double> const &positions_y,
    std::vector<double> const &positions_z,
    std::vector<float> const &uvs_u,
    std::vector<float> const &uvs_v,
    std::vector<std::uint8_t> const &materials,
    std::vector<std::uint32_t> const &indices,
    glm::dvec3 const &origin
) {
    tile_handle handle;
    handle.origin = origin;
    
    if( positions_x.empty() || indices.empty() ) {
        return handle;  // Invalid
    }
    
    std::size_t const vertex_count = positions_x.size();
    handle.vertex_count = static_cast<std::uint32_t>( vertex_count );
    handle.index_count = static_cast<std::uint32_t>( indices.size() );
    
    // Pack vertices into interleaved format
    std::vector<vertex> vertices;
    vertices.reserve( vertex_count );
    
    for( std::size_t i = 0; i < vertex_count; ++i ) {
        vertex v;
        // Store relative to origin for camera-relative rendering
        v.x = static_cast<float>( positions_x[ i ] - origin.x );
        v.y = static_cast<float>( positions_y[ i ] - origin.y );
        v.z = static_cast<float>( positions_z[ i ] - origin.z );
        v.u = uvs_u[ i ];
        v.v = uvs_v[ i ];
        v.material = i < materials.size() ? materials[ i ] : 0;
        v.pad[0] = v.pad[1] = v.pad[2] = 0;
        vertices.push_back( v );
    }
    
    // Create VAO
    glGenVertexArrays( 1, &handle.vao_id );
    glBindVertexArray( handle.vao_id );
    
    // Create and upload VBO
    glGenBuffers( 1, &handle.vbo_id );
    glBindBuffer( GL_ARRAY_BUFFER, handle.vbo_id );
    glBufferData( GL_ARRAY_BUFFER,
        vertices.size() * sizeof( vertex ),
        vertices.data(),
        GL_STATIC_DRAW );
    
    // Position attribute (location 0)
    glEnableVertexAttribArray( 0 );
    glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( vertex ),
        reinterpret_cast<void *>( offsetof( vertex, x ) ) );
    
    // UV attribute (location 1)
    glEnableVertexAttribArray( 1 );
    glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, sizeof( vertex ),
        reinterpret_cast<void *>( offsetof( vertex, u ) ) );
    
    // Material attribute (location 2)
    glEnableVertexAttribArray( 2 );
    glVertexAttribIPointer( 2, 1, GL_UNSIGNED_BYTE, sizeof( vertex ),
        reinterpret_cast<void *>( offsetof( vertex, material ) ) );
    
    // Create and upload IBO
    glGenBuffers( 1, &handle.ibo_id );
    glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, handle.ibo_id );
    glBufferData( GL_ELEMENT_ARRAY_BUFFER,
        indices.size() * sizeof( std::uint32_t ),
        indices.data(),
        GL_STATIC_DRAW );
    
    // Cleanup
    glBindVertexArray( 0 );
    glBindBuffer( GL_ARRAY_BUFFER, 0 );
    glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, 0 );
    
    // Track allocation
    m_allocated_vertices += vertex_count;
    m_allocated_indices += indices.size();
    
    return handle;
}

void
terrain_tile_pool_tin::free( tile_handle &Handle ) {
    if( !Handle.valid() ) return;
    
    // Free GL resources
    if( Handle.vao_id != 0 ) {
        glDeleteVertexArrays( 1, &Handle.vao_id );
    }
    if( Handle.vbo_id != 0 ) {
        glDeleteBuffers( 1, &Handle.vbo_id );
    }
    if( Handle.ibo_id != 0 ) {
        glDeleteBuffers( 1, &Handle.ibo_id );
    }
    
    // Update stats
    m_allocated_vertices -= Handle.vertex_count;
    m_allocated_indices -= Handle.index_count;
    
    // Invalidate handle
    Handle = tile_handle{};
}
