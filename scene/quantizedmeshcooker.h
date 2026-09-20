/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// Cuts a scenery's ground into Cesium Quantized Mesh tiles.
//
// The source is already a mesh, so this does not triangulate anything: it cuts the triangles it
// is given along the tile grid, welds what coincides, and builds each coarser level by taking
// the four tiles below it and collapsing edges until the tile costs what one tile should. What
// it never does is invent ground - a hole in the scenery stays a hole - or move a vertex that
// sits on a tile's border, which is what lets neighbours drawn at different levels meet.
//
// Triangles standing upright have no ground in them, and are refused rather than laid: they are
// walls, and the scenery goes on drawing them itself. refusals() says which.
//
// Shared by the engine's first-run bake and the offline tool, so the two cannot drift apart.
// Millions of triangles do not fit in memory, so what is added is spilled to a scratch file and
// read back a tile at a time.

#include <cstdint>
#include <fstream>
#include <functional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "scene/quantizedmeshformat.h"

namespace quantizedmesh {

class cooker {

public:
    // a corner of a source triangle, in world space, with the texture coordinates the scenery
    // gave it
    struct corner {
        double x, y, z;
        float u, v;
    };

    // one finished tile, handed over as the bytes of its file
    using sink = std::function<void( tile_address const &, std::vector<std::uint8_t> const & )>;

    cooker() = default;
    ~cooker();
    cooker( cooker const & ) = delete;
    cooker &operator=( cooker const & ) = delete;

    // Tilebudget is the most triangles a tile of any level may hold. The side of a tile at the
    // finest level is not given: it is worked out from how large the triangles turn out to be, since
    // a tile much smaller than them would spend most of its triangles on the cuts along its own
    // borders. Opens the scratch file; returns false when it cannot be written
    bool begin( std::string const &Scratchpath, std::size_t Tilebudget = 8192 );
    // the side of a tile at the finest level, once finish() has worked it out
    double finesttile() const { return m_finesttile; }
    // stops talking to standard output, for the engine
    void quiet() { m_quiet = true; }

    // lays one triangle of ground. the id counts from zero in the order they are added, and is
    // what refusals() speaks of
    void add( corner const &A, corner const &B, corner const &C, std::string_view Material );
    std::size_t added() const { return m_added; }

    // cuts, welds, simplifies and hands every tile to the sink, then fills the table. the
    // scratch file is read several times over and removed afterwards
    bool finish( sink const &Sink, terrain_table &Table );

    // triangles that carry no ground, by the id add() gave them, in ascending order. only
    // meaningful once finish() has run
    std::vector<std::uint32_t> const &refusals() const { return m_refusals; }

    // what the cook did, for the log
    struct report {
        std::size_t triangles { 0 };     // laid, after cutting along the grid
        std::size_t refused { 0 };
        std::size_t tiles { 0 };
        std::size_t vertices { 0 };
        std::size_t bytes { 0 };
        std::uint32_t levels { 1 };
    };
    report const &result() const { return m_report; }

    // a vertex of a tile's mesh while it is being worked on, in metres
    struct meshvertex {
        double x, y, z;
        std::uint16_t material;
        // which of the tile's four sides it sits on, one bit each. a vertex on a side is shared
        // with the tile beyond it, so it survives into every level; and an edge whose two ends
        // share a side is a piece of the border itself, and survives too
        std::uint8_t sides { 0 };
        bool locked() const { return sides != 0; }
    };

    struct mesh {
        std::vector<meshvertex> vertices;
        std::vector<std::uint32_t> indices;
        double error { 0.0 };    // metres this mesh departs from the finest one
        bool empty() const { return indices.empty(); }
    };

private:
    // a triangle as it waits in the scratch file. fixed size, so a tile's triangles can be read
    // back by their place in it. only the corners' positions matter by then: the texture
    // coordinates have already been turned into the material's tiling
    struct spilled {
        double corners[ 9 ];
        std::uint16_t material;
        std::uint16_t padding[ 3 ];
    };

    std::uint16_t material_index( std::string_view Material );
    // how many metres of ground one repeat of a material covers, from the texture coordinates
    // the scenery drew it with: the ratio of world distance to texture distance along an edge
    void measure_repeat( std::uint16_t Material, corner const ( &Corners )[ 3 ] );
    // the square the root tile covers, from what was added
    void measure();
    // cooks a tile and everything under it, depth first, and hands each to the sink. returns the
    // tile's own mesh, which its parent merges
    mesh cook( tile_address const &Tile, sink const &Sink );
    // the finest level: the triangles of the scratch file that fall on this tile, cut to it
    mesh lay( tile_address const &Tile );
    // the four tiles below this one, welded into one mesh
    mesh merge( tile_address const &Tile, sink const &Sink );
    // collapses edges until the mesh fits the budget, or until the ground would move further than
    // Allowed, whichever comes first. locked vertices are left where they are
    void simplify( mesh &Mesh, std::size_t Budget, double Allowed );
    // the bytes of a tile's file
    void encode( tile_address const &Tile, mesh const &Mesh, std::vector<std::uint8_t> &Out ) const;
    // where a tile's square lies
    void square( tile_address const &Tile, double &West, double &North, double &Side ) const;

    std::string m_scratchpath;
    std::fstream m_scratch;
    std::size_t m_added { 0 };
    bool m_quiet { false };

    double m_finesttile { 256.0 };
    std::size_t m_tilebudget { 8192 };
    // a sample of how long a triangle's edges are in the plan, which decides the finest tile
    std::vector<double> m_edges;

    std::vector<std::string> m_materials;
    std::unordered_map<std::string, std::uint16_t> m_materiallookup;
    std::vector<double> m_repeatsums;
    std::vector<double> m_repeatsquares;
    std::vector<std::size_t> m_repeatcounts;

    // extent and height range of everything added
    double m_minx { 0.0 }, m_maxx { 0.0 };
    double m_minz { 0.0 }, m_maxz { 0.0 };
    double m_lowest { 0.0 }, m_highest { 0.0 };
    bool m_anything { false };

    // the root square and the depth under it
    double m_west { 0.0 }, m_north { 0.0 }, m_side { 0.0 };
    std::uint32_t m_levels { 1 };

    // which triangles of the scratch file touch which finest tile, sorted by tile. built once,
    // walked as the finest level is laid
    std::vector<std::pair<std::int64_t, std::uint32_t>> m_placement;

    std::vector<std::uint32_t> m_refusals;
    // the worst error any tile of each level came out with
    std::vector<double> m_errors;
    report m_report;
};

} // namespace quantizedmesh
