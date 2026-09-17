/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// Cesium Quantized Mesh format - TIN-based terrain tiles with LOD.
// Replaces regular heightfield with adaptive triangulation.
//
// Format: https://github.com/CesiumGS/quantized-mesh
// 
// File layout:
//   header (88 bytes)
//   vertex data (positions, uvs, indices)
//   edge indices (for seamless LOD stitching)
//   extensions (optional metadata)
//
// Advantages over heightfield:
// - Variable density: fine detail where needed (track corridor), coarse elsewhere
// - ~50% fewer vertices for same visual quality on mixed-detail terrain
// - Built-in LOD with proper edge constraints (no T-junctions)
// - Industry-standard format, tooling available
//
// Disadvantages:
// - More complex to generate (requires constrained triangulation)
// - Slightly larger per-tile overhead (index buffer)

#include <cstdint>
#include <cmath>

namespace quantizedmesh {

// "EU07QMSH" - distinguishes from other formats
inline constexpr char magic[ 8 ] { 'E', 'U', '0', '7', 'Q', 'M', 'S', 'H' };
inline constexpr std::uint32_t version { 1 };

#pragma pack( push, 1 )

struct file_header {
    char magic[ 8 ];
    std::uint32_t version;
    
    // Tile bounds in world space (metres)
    double center_x, center_y, center_z;       // tile centre
    double min_x, min_y, min_z;                // bounding box min
    double max_x, max_y, max_z;                // bounding box max
    
    // Quantization ranges - vertices stored as uint16, decoded to this range
    double horizont_error;                     // maximum geometric error for this LOD
    
    std::uint32_t vertex_count;
    std::uint32_t triangle_count;
    
    // Offsets into file (all after this header)
    std::uint32_t vertex_data_offset;          // packed uint16[3] positions
    std::uint32_t uv_data_offset;              // packed uint16[2] texture coords  
    std::uint32_t index_data_offset;           // uint16 indices (or uint32 if >65535 verts)
    std::uint32_t edge_indices_offset;         // seamless stitching data
    std::uint32_t extension_offset;            // optional metadata
    
    std::uint16_t index_size;                  // 2 (uint16) or 4 (uint32)
    std::uint16_t edge_count_north;
    std::uint16_t edge_count_south;
    std::uint16_t edge_count_west;
    std::uint16_t edge_count_east;
    std::uint16_t reserved;                    // padding for future use
};

static_assert( sizeof( file_header ) == 132 );  // Actual size with current layout

// Edge indices for seamless LOD transitions
// Each edge stores indices of vertices lying on that tile boundary, in order
// Parent LOD uses these to stitch to child tiles without gaps
struct edge_indices {
    std::uint16_t count;
    std::uint16_t indices[];  // variable length
};

// Optional extensions (material splat, metadata, etc.)
struct extension_header {
    std::uint32_t extension_id;
    std::uint32_t extension_size;
};

// Extension IDs
enum extension_id : std::uint32_t {
    ext_material_splat = 1,    // RGBA8 per-vertex material weights
    ext_metadata = 2,           // JSON metadata
    ext_meshlet = 3,            // Meshlet data for mesh shading
};

#pragma pack( pop )

// Helper functions

// Quantize position to uint16
inline std::uint16_t quantize( double value, double min, double max ) {
    if( max <= min ) return 0;
    double const normalized = std::clamp( ( value - min ) / ( max - min ), 0.0, 1.0 );
    return static_cast<std::uint16_t>( normalized * 65535.0 + 0.5 );
}

// Dequantize uint16 to position
inline double dequantize( std::uint16_t encoded, double min, double max ) {
    double const normalized = static_cast<double>( encoded ) / 65535.0;
    return min + normalized * ( max - min );
}

// ZigZag encode for delta-compression of indices
inline std::uint32_t zigzag_encode( std::int32_t value ) {
    return ( value << 1 ) ^ ( value >> 31 );
}

inline std::int32_t zigzag_decode( std::uint32_t value ) {
    return ( value >> 1 ) ^ -static_cast<std::int32_t>( value & 1 );
}

// Octahedral encoding for normals (optional, saves space)
inline void encode_oct( float nx, float ny, float nz, std::int16_t &out_x, std::int16_t &out_y ) {
    float const l1norm = std::abs(nx) + std::abs(ny) + std::abs(nz);
    float px = nx / l1norm;
    float py = ny / l1norm;
    
    if( nz < 0.0f ) {
        float const old_x = px;
        px = ( 1.0f - std::abs(py) ) * ( old_x >= 0.0f ? 1.0f : -1.0f );
        py = ( 1.0f - std::abs(old_x) ) * ( py >= 0.0f ? 1.0f : -1.0f );
    }
    
    out_x = static_cast<std::int16_t>( std::clamp( px * 32767.0f, -32767.0f, 32767.0f ) );
    out_y = static_cast<std::int16_t>( std::clamp( py * 32767.0f, -32767.0f, 32767.0f ) );
}

inline void decode_oct( std::int16_t enc_x, std::int16_t enc_y, float &nx, float &ny, float &nz ) {
    float px = static_cast<float>( enc_x ) / 32767.0f;
    float py = static_cast<float>( enc_y ) / 32767.0f;
    
    nz = 1.0f - std::abs(px) - std::abs(py);
    
    if( nz < 0.0f ) {
        float const old_x = px;
        px = ( 1.0f - std::abs(py) ) * ( old_x >= 0.0f ? 1.0f : -1.0f );
        py = ( 1.0f - std::abs(old_x) ) * ( py >= 0.0f ? 1.0f : -1.0f );
    }
    
    float const len = std::sqrt( px*px + py*py + nz*nz );
    nx = px / len;
    ny = py / len;
    nz = nz / len;
}

// Tile key - same as heightfield for compatibility
constexpr std::int64_t
tile_key( std::int32_t const X, std::int32_t const Z ) {
    return ( static_cast<std::int64_t>( X ) << 32 ) ^ static_cast<std::uint32_t>( Z );
}

} // namespace quantizedmesh
