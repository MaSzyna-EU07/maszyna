/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// Reads Cesium Quantized Mesh tiles - replaces heightfield reader entirely.
// Used by terrain_tile_loader to stream TIN tiles from disk.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <memory>

#include "scene/quantizedmeshformat.h"

namespace quantizedmesh {

// Reader for cooked TIN terrain tiles (.qm files)
// Single-threaded, meant to be used from terrain_tile_loader worker thread
class reader {
public:
    reader() = default;
    ~reader() { close(); }
    
    reader( reader const & ) = delete;
    reader &operator=( reader const & ) = delete;
    
    // Open tile directory (contains terrain_X_Z_lodN.qm files)
    bool open( std::string const &Directory );
    void close();
    
    // Read single LOD of a tile
    struct tile_data {
        // Dequantized positions
        std::vector<double> positions_x;
        std::vector<double> positions_y;
        std::vector<double> positions_z;
        
        // Texture coordinates
        std::vector<float> uvs_u;
        std::vector<float> uvs_v;
        
        // Triangle indices
        std::vector<std::uint32_t> indices;
        
        // Material per vertex (index into material table)
        std::vector<std::uint8_t> materials;
        
        // Edge indices for stitching
        std::vector<std::uint16_t> north_edge;
        std::vector<std::uint16_t> south_edge;
        std::vector<std::uint16_t> west_edge;
        std::vector<std::uint16_t> east_edge;
        
        // Tile bounds (for positioning)
        double center_x, center_y, center_z;
        double min_x, min_y, min_z;
        double max_x, max_y, max_z;
        
        double geometric_error;
    };
    
    bool read_tile( std::int32_t X, std::int32_t Z, std::uint32_t LOD, tile_data &Out );
    
    // Check if tile exists
    bool contains( std::int32_t X, std::int32_t Z, std::uint32_t LOD ) const;
    
    std::string const &directory() const { return m_directory; }
    
private:
    std::string tile_path( std::int32_t X, std::int32_t Z, std::uint32_t LOD ) const;
    
    std::string m_directory;
};

// Implementation

inline bool reader::open( std::string const &Directory ) {
    close();
    m_directory = Directory;
    return true;  // Lazy - tiles opened on demand
}

inline void reader::close() {
    m_directory.clear();
}

inline std::string reader::tile_path( std::int32_t X, std::int32_t Z, std::uint32_t LOD ) const {
    char filename[ 256 ];
    std::snprintf( filename, sizeof( filename ), "terrain_%d_%d_lod%u.qm", X, Z, LOD );
    return m_directory + "/" + filename;
}

inline bool reader::contains( std::int32_t X, std::int32_t Z, std::uint32_t LOD ) const {
    auto const path = tile_path( X, Z, LOD );
    FILE *f = std::fopen( path.c_str(), "rb" );
    if( !f ) return false;
    std::fclose( f );
    return true;
}

inline bool reader::read_tile( std::int32_t X, std::int32_t Z, std::uint32_t LOD, tile_data &Out ) {
    auto const path = tile_path( X, Z, LOD );
    
    FILE *file = std::fopen( path.c_str(), "rb" );
    if( !file ) return false;
    
    // Read header
    file_header header;
    if( std::fread( &header, sizeof( header ), 1, file ) != 1 ) {
        std::fclose( file );
        return false;
    }
    
    // Verify magic
    if( std::memcmp( header.magic, magic, sizeof( magic ) ) != 0 ) {
        std::fclose( file );
        return false;
    }
    
    if( header.version != version ) {
        std::fclose( file );
        return false;
    }
    
    // Store bounds
    Out.center_x = header.center_x;
    Out.center_y = header.center_y;
    Out.center_z = header.center_z;
    Out.min_x = header.min_x;
    Out.min_y = header.min_y;
    Out.min_z = header.min_z;
    Out.max_x = header.max_x;
    Out.max_y = header.max_y;
    Out.max_z = header.max_z;
    Out.geometric_error = header.horizont_error;
    
    // Read quantized positions
    std::fseek( file, header.vertex_data_offset, SEEK_SET );
    
    std::vector<std::uint16_t> quantized_x( header.vertex_count );
    std::vector<std::uint16_t> quantized_y( header.vertex_count );
    std::vector<std::uint16_t> quantized_z( header.vertex_count );
    
    for( std::uint32_t i = 0; i < header.vertex_count; ++i ) {
        std::fread( &quantized_x[ i ], sizeof( std::uint16_t ), 1, file );
        std::fread( &quantized_y[ i ], sizeof( std::uint16_t ), 1, file );
        std::fread( &quantized_z[ i ], sizeof( std::uint16_t ), 1, file );
    }
    
    // Dequantize positions
    Out.positions_x.resize( header.vertex_count );
    Out.positions_y.resize( header.vertex_count );
    Out.positions_z.resize( header.vertex_count );
    
    for( std::uint32_t i = 0; i < header.vertex_count; ++i ) {
        Out.positions_x[ i ] = dequantize( quantized_x[ i ], header.min_x, header.max_x );
        Out.positions_y[ i ] = dequantize( quantized_y[ i ], header.min_y, header.max_y );
        Out.positions_z[ i ] = dequantize( quantized_z[ i ], header.min_z, header.max_z );
    }
    
    // Read UVs
    std::fseek( file, header.uv_data_offset, SEEK_SET );
    
    Out.uvs_u.resize( header.vertex_count );
    Out.uvs_v.resize( header.vertex_count );
    
    for( std::uint32_t i = 0; i < header.vertex_count; ++i ) {
        std::uint16_t u, v;
        std::fread( &u, sizeof( std::uint16_t ), 1, file );
        std::fread( &v, sizeof( std::uint16_t ), 1, file );
        Out.uvs_u[ i ] = static_cast<float>( dequantize( u, 0.0, 1.0 ) );
        Out.uvs_v[ i ] = static_cast<float>( dequantize( v, 0.0, 1.0 ) );
    }
    
    // Read indices
    std::fseek( file, header.index_data_offset, SEEK_SET );
    
    Out.indices.resize( header.triangle_count * 3 );
    
    if( header.index_size == 2 ) {
        for( std::uint32_t i = 0; i < header.triangle_count * 3; ++i ) {
            std::uint16_t idx;
            std::fread( &idx, sizeof( std::uint16_t ), 1, file );
            Out.indices[ i ] = idx;
        }
    } else {
        std::fread( Out.indices.data(), sizeof( std::uint32_t ), header.triangle_count * 3, file );
    }
    
    // Read edge indices
    std::fseek( file, header.edge_indices_offset, SEEK_SET );
    
    Out.north_edge.resize( header.edge_count_north );
    Out.south_edge.resize( header.edge_count_south );
    Out.west_edge.resize( header.edge_count_west );
    Out.east_edge.resize( header.edge_count_east );
    
    if( header.edge_count_north > 0 )
        std::fread( Out.north_edge.data(), sizeof( std::uint16_t ), header.edge_count_north, file );
    if( header.edge_count_south > 0 )
        std::fread( Out.south_edge.data(), sizeof( std::uint16_t ), header.edge_count_south, file );
    if( header.edge_count_west > 0 )
        std::fread( Out.west_edge.data(), sizeof( std::uint16_t ), header.edge_count_west, file );
    if( header.edge_count_east > 0 )
        std::fread( Out.east_edge.data(), sizeof( std::uint16_t ), header.edge_count_east, file );
    
    // Materials - for now, all zeros (placeholder)
    Out.materials.resize( header.vertex_count, 0 );
    
    std::fclose( file );
    return true;
}

} // namespace quantizedmesh
