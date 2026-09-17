/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "scene/terraincooker_tin.h"
#include "scene/terraincooker.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <queue>
#include <unordered_set>

namespace terrain {

namespace {

// Hash for int64 tile key
struct tile_key_hash {
    std::size_t operator()( std::int64_t key ) const noexcept {
        return static_cast<std::size_t>( key ^ ( key >> 32 ) );
    }
};

// Calculate tile coordinates from world position
std::int32_t tile_coord( double world_pos, double tile_size ) {
    return static_cast<std::int32_t>( std::floor( world_pos / tile_size ) );
}

std::int64_t make_tile_key( std::int32_t x, std::int32_t z ) {
    return ( static_cast<std::int64_t>( x ) << 32 ) | static_cast<std::uint32_t>( z );
}

} // anonymous namespace

void tin_cooker::configure( 
    double TileSize,
    std::uint32_t LodLevels,
    double ErrorMultiplier
) {
    m_tilesize = TileSize;
    m_lodlevels = std::max( 1u, LodLevels );
    m_errormultiplier = std::max( 1.1, ErrorMultiplier );
}

void tin_cooker::rasterize( 
    vertex const &A,
    vertex const &B,
    vertex const &C,
    std::string_view Material
) {
    ++m_triangles_in;
    
    // Find centroid to determine tile
    double const cx = ( A.x + B.x + C.x ) / 3.0;
    double const cz = ( A.z + B.z + C.z ) / 3.0;
    
    std::int32_t const tile_x = tile_coord( cx, m_tilesize );
    std::int32_t const tile_z = tile_coord( cz, m_tilesize );
    
    std::int64_t const key = make_tile_key( tile_x, tile_z );
    
    auto &tile = m_tiles[ key ];
    tile.triangles.push_back( { { A, B, C }, std::string( Material ) } );
    tile.dirty = true;
}

void tin_cooker::add_constraint( vertex const &A, vertex const &B ) {
    // Add constraint to all tiles it intersects
    double const minx = std::min( A.x, B.x );
    double const maxx = std::max( A.x, B.x );
    double const minz = std::min( A.z, B.z );
    double const maxz = std::max( A.z, B.z );
    
    std::int32_t const tile_x0 = tile_coord( minx, m_tilesize );
    std::int32_t const tile_z0 = tile_coord( minz, m_tilesize );
    std::int32_t const tile_x1 = tile_coord( maxx, m_tilesize );
    std::int32_t const tile_z1 = tile_coord( maxz, m_tilesize );
    
    for( std::int32_t tx = tile_x0; tx <= tile_x1; ++tx ) {
        for( std::int32_t tz = tile_z0; tz <= tile_z1; ++tz ) {
            std::int64_t const key = make_tile_key( tx, tz );
            auto &tile = m_tiles[ key ];
            tile.constraints.push_back( { A, B } );
            tile.dirty = true;
        }
    }
}

std::vector<tin_tile> tin_cooker::finish() {
    std::vector<tin_tile> result;
    result.reserve( m_tiles.size() );
    
    std::size_t total_tiles = m_tiles.size();
    std::size_t processed = 0;
    
    for( auto &[ key, accumulator ] : m_tiles ) {
        if( !accumulator.dirty || accumulator.triangles.empty() ) {
            continue;
        }
        
        ++processed;
        if( processed % 100 == 0 || processed == total_tiles ) {
            std::printf( "\rTriangulating tiles: %zu/%zu", processed, total_tiles );
            std::fflush( stdout );
        }
        
        std::int32_t const tile_x = static_cast<std::int32_t>( key >> 32 );
        std::int32_t const tile_z = static_cast<std::int32_t>( key & 0xffffffff );
        
        tin_tile tile;
        tile.x = tile_x;
        tile.z = tile_z;
        
        // Tile bounds
        tile.minx = tile_x * m_tilesize;
        tile.minz = tile_z * m_tilesize;
        tile.maxx = tile.minx + m_tilesize;
        tile.maxz = tile.minz + m_tilesize;
        tile.miny = std::numeric_limits<double>::max();
        tile.maxy = std::numeric_limits<double>::lowest();
        
        // Collect unique vertices from triangles
        std::vector<vertex> points;
        points.reserve( accumulator.triangles.size() * 3 );
        
        for( auto const &tri : accumulator.triangles ) {
            for( auto const &v : tri.v ) {
                points.push_back( v );
                tile.miny = std::min( tile.miny, v.y );
                tile.maxy = std::max( tile.maxy, v.y );
            }
        }
        
        // Build material table
        for( auto const &tri : accumulator.triangles ) {
            get_material_index( tile, tri.material );
        }
        
        // Generate LODs from finest to coarsest
        double current_error = 0.1;  // Start with very fine detail
        
        for( std::uint32_t lod = 0; lod < m_lodlevels; ++lod ) {
            tin_lod level;
            
            if( lod == 0 ) {
                // Finest LOD: triangulate all points with constraints
                level = triangulate_lod( points, accumulator.constraints, current_error );
            } else {
                // Coarser LODs: simplify previous level
                level = simplify_lod( tile.lods.back(), current_error );
            }
            
            level.geometric_error = current_error;
            extract_edges( level, tile.minx, tile.minz, tile.maxx, tile.maxz );
            
            tile.lods.push_back( std::move( level ) );
            
            m_triangles_out += level.indices.size() / 3;
            m_vertices_out += level.vertices.size();
            
            current_error *= m_errormultiplier;
        }
        
        result.push_back( std::move( tile ) );
    }
    
    std::printf( "\n" );
    std::printf( "Triangulation complete:\n" );
    std::printf( "  Input triangles:  %zu\n", m_triangles_in );
    std::printf( "  Output triangles: %zu\n", m_triangles_out );
    std::printf( "  Output vertices:  %zu\n", m_vertices_out );
    std::printf( "  Reduction:        %.1f%%\n",
        m_triangles_in > 0 ? 100.0 * ( 1.0 - static_cast<double>( m_triangles_out ) / m_triangles_in ) : 0.0 );
    
    return result;
}

tin_lod tin_cooker::triangulate_lod(
    std::vector<vertex> const &Points,
    std::vector<tin_constraint> const &Constraints,
    double TargetError
) {
    tin_lod result;
    
    if( Points.empty() ) {
        return result;
    }
    
    // Convert points to CDT format, removing duplicates
    std::vector<CDT::V2d<double>> cdt_points;
    std::vector<CDT::Edge> cdt_edges;
    
    cdt_points.reserve( Points.size() );
    std::unordered_map<std::uint64_t, std::uint32_t> point_map;
    
    auto hash_point = []( vertex const &v ) -> std::uint64_t {
        // Hash to 1cm precision
        std::uint32_t const x = static_cast<std::uint32_t>( std::round( v.x * 100.0 ) );
        std::uint32_t const z = static_cast<std::uint32_t>( std::round( v.z * 100.0 ) );
        return ( static_cast<std::uint64_t>( x ) << 32 ) | z;
    };
    
    for( auto const &v : Points ) {
        std::uint64_t const h = hash_point( v );
        auto const [ it, inserted ] = point_map.try_emplace( h, static_cast<std::uint32_t>( cdt_points.size() ) );
        
        if( inserted ) {
            cdt_points.push_back( to_cdt_point( v ) );
            result.vertices.push_back( v );
        }
    }
    
    // Add constraint edges
    for( auto const &constraint : Constraints ) {
        std::uint64_t const h0 = hash_point( constraint.a );
        std::uint64_t const h1 = hash_point( constraint.b );
        
        auto it0 = point_map.find( h0 );
        auto it1 = point_map.find( h1 );
        
        if( it0 != point_map.end() && it1 != point_map.end() && it0->second != it1->second ) {
            cdt_edges.push_back( CDT::Edge( it0->second, it1->second ) );
        }
    }
    
    // Run CDT
    CDT::Triangulation<double> cdt;
    cdt.insertVertices( cdt_points );
    
    if( !cdt_edges.empty() ) {
        cdt.insertEdges( cdt_edges );
        cdt.eraseOuterTrianglesAndHoles();
    }
    
    // Extract triangles
    result.indices.reserve( cdt.triangles.size() * 3 );
    
    for( auto const &tri : cdt.triangles ) {
        result.indices.push_back( tri.vertices[ 0 ] );
        result.indices.push_back( tri.vertices[ 1 ] );
        result.indices.push_back( tri.vertices[ 2 ] );
    }
    
    // Assign material indices (simplified - use most common material for now)
    result.materials.resize( result.vertices.size(), 0 );
    
    return result;
}

tin_lod tin_cooker::simplify_lod(
    tin_lod const &FineLod,
    double TargetError
) {
    // Edge collapse simplification
    // For now: simple vertex removal based on geometric error
    
    tin_lod result = FineLod;  // Start with copy
    
    if( FineLod.vertices.size() < 100 ) {
        // Too small to simplify meaningfully
        return result;
    }
    
    // Build adjacency information
    std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> adjacency;
    
    for( std::size_t i = 0; i < FineLod.indices.size(); i += 3 ) {
        std::uint32_t const i0 = FineLod.indices[ i + 0 ];
        std::uint32_t const i1 = FineLod.indices[ i + 1 ];
        std::uint32_t const i2 = FineLod.indices[ i + 2 ];
        
        adjacency[ i0 ].push_back( i1 );
        adjacency[ i0 ].push_back( i2 );
        adjacency[ i1 ].push_back( i0 );
        adjacency[ i1 ].push_back( i2 );
        adjacency[ i2 ].push_back( i0 );
        adjacency[ i2 ].push_back( i1 );
    }
    
    // Calculate error for each vertex
    struct vertex_error {
        std::uint32_t index;
        double error;
        
        bool operator>( vertex_error const &other ) const {
            return error > other.error;  // Min-heap
        }
    };
    
    std::priority_queue<vertex_error, std::vector<vertex_error>, std::greater<vertex_error>> error_queue;
    std::unordered_set<std::uint32_t> removed;
    
    for( std::uint32_t i = 0; i < FineLod.vertices.size(); ++i ) {
        auto it = adjacency.find( i );
        if( it == adjacency.end() || it->second.empty() ) {
            continue;
        }
        
        std::vector<vertex> neighbours;
        for( auto const j : it->second ) {
            neighbours.push_back( FineLod.vertices[ j ] );
        }
        
        double const error = calculate_error( FineLod.vertices[ i ], neighbours );
        
        if( error < TargetError ) {
            error_queue.push( { i, error } );
        }
    }
    
    // Remove vertices with low error (target: remove ~30%)
    std::size_t const target_removals = FineLod.vertices.size() * 3 / 10;
    
    while( !error_queue.empty() && removed.size() < target_removals ) {
        auto const top = error_queue.top();
        error_queue.pop();
        
        if( removed.count( top.index ) > 0 ) {
            continue;
        }
        
        removed.insert( top.index );
    }
    
    // Rebuild mesh without removed vertices
    std::unordered_map<std::uint32_t, std::uint32_t> index_remap;
    std::vector<vertex> new_vertices;
    std::vector<std::uint8_t> new_materials;
    
    for( std::uint32_t i = 0; i < FineLod.vertices.size(); ++i ) {
        if( removed.count( i ) == 0 ) {
            index_remap[ i ] = static_cast<std::uint32_t>( new_vertices.size() );
            new_vertices.push_back( FineLod.vertices[ i ] );
            if( i < FineLod.materials.size() ) {
                new_materials.push_back( FineLod.materials[ i ] );
            }
        }
    }
    
    result.vertices = std::move( new_vertices );
    result.materials = std::move( new_materials );
    
    // Re-triangulate (simple approach: use CDT again)
    std::vector<CDT::V2d<double>> cdt_points;
    cdt_points.reserve( result.vertices.size() );
    
    for( auto const &v : result.vertices ) {
        cdt_points.push_back( to_cdt_point( v ) );
    }
    
    CDT::Triangulation<double> cdt;
    cdt.insertVertices( cdt_points );
    
    result.indices.clear();
    result.indices.reserve( cdt.triangles.size() * 3 );
    
    for( auto const &tri : cdt.triangles ) {
        result.indices.push_back( tri.vertices[ 0 ] );
        result.indices.push_back( tri.vertices[ 1 ] );
        result.indices.push_back( tri.vertices[ 2 ] );
    }
    
    return result;
}

double tin_cooker::calculate_error(
    vertex const &V,
    std::vector<vertex> const &Neighbours
) {
    if( Neighbours.size() < 3 ) {
        return std::numeric_limits<double>::max();
    }
    
    // Calculate maximum distance to plane fitted through neighbours
    // Simplified: just use average height difference
    
    double sum_y = 0.0;
    for( auto const &n : Neighbours ) {
        sum_y += n.y;
    }
    double const avg_y = sum_y / Neighbours.size();
    
    return std::abs( V.y - avg_y );
}

void tin_cooker::extract_edges( 
    tin_lod &Lod,
    double TileMinX,
    double TileMinZ,
    double TileMaxX,
    double TileMaxZ
) {
    constexpr double edge_epsilon = 0.01;
    
    Lod.north_edge.clear();
    Lod.south_edge.clear();
    Lod.west_edge.clear();
    Lod.east_edge.clear();
    
    for( std::uint32_t i = 0; i < Lod.vertices.size(); ++i ) {
        auto const &v = Lod.vertices[ i ];
        
        bool on_edge = false;
        
        if( std::abs( v.x - TileMinX ) < edge_epsilon ) {
            Lod.west_edge.push_back( static_cast<std::uint16_t>( i ) );
            on_edge = true;
        }
        if( std::abs( v.x - TileMaxX ) < edge_epsilon ) {
            Lod.east_edge.push_back( static_cast<std::uint16_t>( i ) );
            on_edge = true;
        }
        if( std::abs( v.z - TileMinZ ) < edge_epsilon ) {
            Lod.south_edge.push_back( static_cast<std::uint16_t>( i ) );
            on_edge = true;
        }
        if( std::abs( v.z - TileMaxZ ) < edge_epsilon ) {
            Lod.north_edge.push_back( static_cast<std::uint16_t>( i ) );
            on_edge = true;
        }
    }
    
    // Sort edges by position (west/east by Z, north/south by X)
    auto sort_west_east = [ &Lod ]( std::uint16_t a, std::uint16_t b ) {
        return Lod.vertices[ a ].z < Lod.vertices[ b ].z;
    };
    auto sort_north_south = [ &Lod ]( std::uint16_t a, std::uint16_t b ) {
        return Lod.vertices[ a ].x < Lod.vertices[ b ].x;
    };
    
    std::sort( Lod.west_edge.begin(), Lod.west_edge.end(), sort_west_east );
    std::sort( Lod.east_edge.begin(), Lod.east_edge.end(), sort_west_east );
    std::sort( Lod.north_edge.begin(), Lod.north_edge.end(), sort_north_south );
    std::sort( Lod.south_edge.begin(), Lod.south_edge.end(), sort_north_south );
}

std::uint8_t tin_cooker::get_material_index( tin_tile &Tile, std::string const &Material ) {
    auto it = Tile.material_lookup.find( Material );
    
    if( it != Tile.material_lookup.end() ) {
        return it->second;
    }
    
    std::uint8_t const index = static_cast<std::uint8_t>( Tile.materials.size() );
    Tile.materials.push_back( Material );
    Tile.material_lookup[ Material ] = index;
    
    return index;
}

bool tin_cooker::write_tile(
    tin_tile const &Tile,
    std::uint32_t LodIndex,
    std::string const &Path,
    bool Compress
) {
    if( LodIndex >= Tile.lods.size() ) {
        return false;
    }
    
    auto const &lod = Tile.lods[ LodIndex ];
    
    // Calculate bounding box centre
    double const cx = ( Tile.minx + Tile.maxx ) * 0.5;
    double const cy = ( Tile.miny + Tile.maxy ) * 0.5;
    double const cz = ( Tile.minz + Tile.maxz ) * 0.5;
    
    // Build header
    quantizedmesh::file_header header{};
    std::memcpy( header.magic, quantizedmesh::magic, sizeof( header.magic ) );
    header.version = quantizedmesh::version;
    
    header.center_x = cx;
    header.center_y = cy;
    header.center_z = cz;
    
    header.min_x = Tile.minx;
    header.min_y = Tile.miny;
    header.min_z = Tile.minz;
    
    header.max_x = Tile.maxx;
    header.max_y = Tile.maxy;
    header.max_z = Tile.maxz;
    
    header.horizont_error = lod.geometric_error;
    header.vertex_count = static_cast<std::uint32_t>( lod.vertices.size() );
    header.triangle_count = static_cast<std::uint32_t>( lod.indices.size() / 3 );
    
    header.index_size = lod.vertices.size() > 65535 ? 4 : 2;
    header.edge_count_north = static_cast<std::uint16_t>( lod.north_edge.size() );
    header.edge_count_south = static_cast<std::uint16_t>( lod.south_edge.size() );
    header.edge_count_west = static_cast<std::uint16_t>( lod.west_edge.size() );
    header.edge_count_east = static_cast<std::uint16_t>( lod.east_edge.size() );
    
    // Calculate offsets
    std::uint32_t offset = sizeof( header );
    
    header.vertex_data_offset = offset;
    offset += header.vertex_count * 3 * sizeof( std::uint16_t );
    
    header.uv_data_offset = offset;
    offset += header.vertex_count * 2 * sizeof( std::uint16_t );
    
    header.index_data_offset = offset;
    offset += header.triangle_count * 3 * header.index_size;
    
    header.edge_indices_offset = offset;
    std::uint32_t const edge_data_size = 
        ( header.edge_count_north + header.edge_count_south +
          header.edge_count_west + header.edge_count_east ) * sizeof( std::uint16_t );
    offset += edge_data_size;
    
    header.extension_offset = offset;
    
    // Write file
    std::ofstream file( Path, std::ios::binary );
    if( !file.good() ) {
        return false;
    }
    
    file.write( reinterpret_cast<char const *>( &header ), sizeof( header ) );
    
    // Write quantized vertex positions
    for( auto const &v : lod.vertices ) {
        std::uint16_t const qx = quantizedmesh::quantize( v.x, Tile.minx, Tile.maxx );
        std::uint16_t const qy = quantizedmesh::quantize( v.y, Tile.miny, Tile.maxy );
        std::uint16_t const qz = quantizedmesh::quantize( v.z, Tile.minz, Tile.maxz );
        
        file.write( reinterpret_cast<char const *>( &qx ), sizeof( qx ) );
        file.write( reinterpret_cast<char const *>( &qy ), sizeof( qy ) );
        file.write( reinterpret_cast<char const *>( &qz ), sizeof( qz ) );
    }
    
    // Write UV coordinates (from texture coords in vertex)
    for( auto const &v : lod.vertices ) {
        std::uint16_t const u = quantizedmesh::quantize( v.u, 0.0, 1.0 );
        std::uint16_t const uv = quantizedmesh::quantize( v.v, 0.0, 1.0 );
        
        file.write( reinterpret_cast<char const *>( &u ), sizeof( u ) );
        file.write( reinterpret_cast<char const *>( &uv ), sizeof( uv ) );
    }
    
    // Write indices
    for( auto const idx : lod.indices ) {
        if( header.index_size == 2 ) {
            std::uint16_t const idx16 = static_cast<std::uint16_t>( idx );
            file.write( reinterpret_cast<char const *>( &idx16 ), sizeof( idx16 ) );
        } else {
            file.write( reinterpret_cast<char const *>( &idx ), sizeof( idx ) );
        }
    }
    
    // Write edge indices
    for( auto const idx : lod.north_edge ) {
        file.write( reinterpret_cast<char const *>( &idx ), sizeof( idx ) );
    }
    for( auto const idx : lod.south_edge ) {
        file.write( reinterpret_cast<char const *>( &idx ), sizeof( idx ) );
    }
    for( auto const idx : lod.west_edge ) {
        file.write( reinterpret_cast<char const *>( &idx ), sizeof( idx ) );
    }
    for( auto const idx : lod.east_edge ) {
        file.write( reinterpret_cast<char const *>( &idx ), sizeof( idx ) );
    }
    
    file.close();
    return file.good();
}

} // namespace terrain
