/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// Triangulates terrain into TIN with multiple LODs using Constrained Delaunay.
// Input: triangle soup from scenery (same as heightfield cooker)
// Output: quantized mesh tiles with proper edge constraints for seamless LOD

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <CDT.h>

#include "scene/quantizedmeshformat.h"
#include "scene/terraincooker.h"  // For terrain::vertex definition

namespace terrain {

// Triangle with material
struct tin_triangle {
    vertex v[ 3 ];
    std::string material;
};

// Constraint edge (e.g. track corridor boundary)
struct tin_constraint {
    vertex a, b;
};

// Single LOD level of a TIN tile
struct tin_lod {
    std::vector<vertex> vertices;
    std::vector<std::uint32_t> indices;
    std::vector<std::uint8_t> materials;  // Per-vertex material indices
    
    // Edge vertices for seamless stitching (indices into vertices array)
    std::vector<std::uint16_t> north_edge;
    std::vector<std::uint16_t> south_edge;
    std::vector<std::uint16_t> west_edge;
    std::vector<std::uint16_t> east_edge;
    
    double geometric_error;  // Maximum error vs. finer LOD
};

// Complete TIN tile with all LODs
struct tin_tile {
    std::int32_t x, z;  // Tile coordinates
    
    std::vector<tin_lod> lods;  // Finest to coarsest
    
    // Bounding box
    double minx, miny, minz;
    double maxx, maxy, maxz;
    
    // Material table (shared across LODs)
    std::vector<std::string> materials;
    std::unordered_map<std::string, std::uint8_t> material_lookup;
};

class tin_cooker {
public:
    tin_cooker() = default;
    
    // Configure tile size and LOD parameters
    void configure( 
        double TileSize,      // Metres per tile side
        std::uint32_t LodLevels = 4,  // Number of LOD levels
        double ErrorMultiplier = 2.0   // Geometric error growth per LOD
    );
    
    // Accumulate terrain triangles (same interface as heightfield cooker)
    void rasterize( 
        vertex const &A, 
        vertex const &B, 
        vertex const &C,
        std::string_view Material 
    );
    
    // Add constraint edges (track corridors, roads, water boundaries)
    void add_constraint( vertex const &A, vertex const &B );
    
    // Generate all tiles with LODs
    std::vector<tin_tile> finish();
    
    // Export single tile to Quantized Mesh format
    static bool write_tile( 
        tin_tile const &Tile,
        std::uint32_t LodIndex,
        std::string const &Path,
        bool Compress = true
    );

private:
    struct tile_accumulator {
        std::vector<tin_triangle> triangles;
        std::vector<tin_constraint> constraints;
        bool dirty { false };
    };
    
    // Build single LOD level using CDT
    tin_lod triangulate_lod(
        std::vector<vertex> const &Points,
        std::vector<tin_constraint> const &Constraints,
        double TargetError
    );
    
    // Simplify LOD by removing vertices with low geometric error
    tin_lod simplify_lod(
        tin_lod const &FineLod,
        double TargetError
    );
    
    // Calculate geometric error of removing a vertex
    double calculate_error(
        vertex const &V,
        std::vector<vertex> const &Neighbours
    );
    
    // Extract edge vertices for stitching
    void extract_edges( tin_lod &Lod, double TileMinX, double TileMinZ, double TileMaxX, double TileMaxZ );
    
    // Find material index, adding to table if new
    std::uint8_t get_material_index( tin_tile &Tile, std::string const &Material );
    
    double m_tilesize { 256.0 };
    std::uint32_t m_lodlevels { 4 };
    double m_errormultiplier { 2.0 };
    
    std::unordered_map<std::int64_t, tile_accumulator> m_tiles;
    
    // Statistics
    std::size_t m_triangles_in { 0 };
    std::size_t m_triangles_out { 0 };
    std::size_t m_vertices_out { 0 };
};

// Helper: convert CDT point to our vertex
inline vertex from_cdt_point( CDT::V2d<double> const &P, double Y = 0.0 ) {
    return vertex{ P.x, Y, P.y };
}

// Helper: convert vertex to CDT point
inline CDT::V2d<double> to_cdt_point( vertex const &V ) {
    return CDT::V2d<double>::make( V.x, V.z );
}

// Helper: calculate distance from point to line segment
inline double point_to_segment_distance( 
    vertex const &Point,
    vertex const &SegmentA,
    vertex const &SegmentB 
) {
    double const dx = SegmentB.x - SegmentA.x;
    double const dz = SegmentB.z - SegmentA.z;
    double const len_sq = dx * dx + dz * dz;
    
    if( len_sq < 1e-10 ) {
        // Degenerate segment
        double const pdx = Point.x - SegmentA.x;
        double const pdz = Point.z - SegmentA.z;
        return std::sqrt( pdx * pdx + pdz * pdz );
    }
    
    // Project point onto line
    double const t = std::clamp(
        ( ( Point.x - SegmentA.x ) * dx + ( Point.z - SegmentA.z ) * dz ) / len_sq,
        0.0, 1.0
    );
    
    double const proj_x = SegmentA.x + t * dx;
    double const proj_z = SegmentA.z + t * dz;
    
    double const pdx = Point.x - proj_x;
    double const pdy = Point.y - ( SegmentA.y + t * ( SegmentB.y - SegmentA.y ) );
    double const pdz = Point.z - proj_z;
    
    return std::sqrt( pdx * pdx + pdy * pdy + pdz * pdz );
}

} // namespace terrain
