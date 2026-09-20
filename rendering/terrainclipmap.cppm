/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "scene/quantizedmeshreader.h"

export module eu07.rendering.terrainclipmap;
import eu07.gl.shader;
import eu07.glm;
import eu07.rendering.terraintileloader;
import eu07.rendering.terrainstatus;
import eu07.rendering.terrainmesharena;
import eu07.rendering.terrainground;

export {

// Draws a scenery's cooked terrain: a quadtree of mesh tiles, each a piece of the ground as the
// scenery drew it, thinned out the further from the camera it is drawn.
//
// Each frame a walk of the quadtree settles on the tiles to draw: a tile is split when the ground
// it stands for would be off by more pixels than the detail setting allows, and only when all four
// of its children are on the card - otherwise it is drawn itself and the children are asked for, so
// that streaming shows coarse ground rather than a hole. Because every vertex on a tile's border
// survives into all of its coarser levels, two neighbours drawn at different levels share their
// border exactly, and nothing has to be stitched or morphed.
//
// Every resident tile lives in the same pair of buffers, so the whole terrain is drawn in one call.
class terrain_clipmap {

public:
    // whether a sphere, in world space, can be seen: centre and radius
    using visibility = std::function<bool( glm::dvec3 const &, float )>;

    terrain_clipmap() = default;
    ~terrain_clipmap();
    terrain_clipmap( terrain_clipmap const & ) = delete;
    terrain_clipmap &operator=( terrain_clipmap const & ) = delete;

    // opens a cooked terrain - an archive or a directory of tiles - and builds the gl resources.
    // returns false when it cannot be read, leaving the object unusable but safe to destroy
    bool open( std::string const &Path );
    bool ready() const { return m_ready; }
    void close();

    // walks the quadtree, asks for what is missing, puts in what has arrived and retires what has
    // not been wanted for a while. safe to call every frame: the reading happens on the loader's
    // thread
    void update( glm::dvec3 const &Viewpoint );
    // draws the tiles the walk settled on that the test says can be seen. Mainview says whether this
    // is the view residency is decided by, and so whether what it drew is worth counting: a shadow
    // pass sees the same tiles through a frustum of its own
    void render( glm::dvec3 const &Viewpoint, visibility const &Visible, bool Mainview );

    // how far from the viewpoint terrain is drawn, in metres
    void range( float const Range ) { m_range = Range; }
    float range() const { return m_range; }
    // what the screen can resolve: the viewport's pixels per unit of size over distance, and how
    // many pixels a tile's error may cover before a finer level is used
    void detail( double const Pixelsperunit, double const Pixels );
    // side of a tile at the finest level, in metres. terrains are drawn finest first, so that the
    // depth test keeps the better surface where two of them describe the same ground
    float gridstep() const { return m_finesttile; }
    // pushes this terrain back in depth by the given rank, so that geometry lying on it wins the
    // depth test, and a coarser terrain loses to a finer one describing the same ground
    void depthrank( unsigned const Rank ) { m_depthrank = Rank; }

    terrain_statistics const &stats() const { return m_stats; }
    std::string const &path() const { return m_path; }
    float tilesize() const { return m_finesttile; }
    std::uint32_t levels() const { return m_levels; }
    std::size_t backlog() const { return m_loader.backlog(); }
    float detailpixels() const { return static_cast<float>( m_detail ); }
    // side of a tile at each level, and how far that level's ground is off, for the debug panel
    float leveltile( std::uint32_t const Level ) const;
    float levelerror( std::uint32_t const Level ) const;
    float leveledge( std::uint32_t const Level ) const;

private:
    // one tile held in the buffers
    struct resident_tile {
        terrain_mesh_arena::piece piece;
        bool wide { false };   // which of the two arenas it is in
        // for culling, in world space
        glm::dvec3 centre { 0.0 };
        float radius { 0.f };
        std::uint32_t level { 0 };
        std::uint64_t lastused { 0 };
    };

    // the tiles the walk settled on, in the order they will be drawn
    void walk( glm::dvec3 const &Viewpoint );
    // asks the loader for every tile the wanted cut of the quadtree is missing, and says whether this
    // tile's ground can be drawn at all - at the level wanted, or by the tile itself standing in
    bool need( quantizedmesh::tile_address const &Tile, glm::dvec3 const &Viewpoint );
    // whether the level this tile stands for is close enough to the ground below it
    bool good_enough( quantizedmesh::tile_address const &Tile, double Distance ) const;
    void choose( quantizedmesh::tile_address const &Tile, glm::dvec3 const &Viewpoint );
    // where a tile's square is, and how far the viewpoint is from it
    void square( quantizedmesh::tile_address const &Tile, double &West, double &North, double &Side ) const;
    double distance_to( quantizedmesh::tile_address const &Tile, glm::dvec3 const &Viewpoint ) const;
    bool present( quantizedmesh::tile_address const &Tile ) const;
    void ask_for( quantizedmesh::tile_address const &Tile, double Distance );
    void put_in( terrain_tile_loader::payload const &Tile );
    void retire_unused();
    void resolve_uniforms();
    void report_residency();

    quantizedmesh::reader m_reader;      // the table and which tiles there are; reading is the loader's
    terrain_tile_loader m_loader;
    // Nearly every tile fits two-byte indices and lives in the first; a tile of more than sixty-five
    // thousand vertices - a patch of survey data, usually - goes in the second, which pays four bytes
    // for them. One draw call each, and the second is empty on most sceneries.
    terrain_mesh_arena m_arena;
    terrain_mesh_arena m_widearena;
    terrain_ground m_ground;

    quantizedmesh::terrain_table m_table;
    glm::dvec3 m_origin { 0.0 };         // what the vertices in the buffers are measured from
    float m_finesttile { 256.f };
    std::uint32_t m_levels { 1 };

    std::unordered_map<std::int64_t, resident_tile> m_tiles;
    std::vector<std::int64_t> m_chosen;
    // the tiles the walk passed through on its way down. They are not drawn - what is below them is -
    // but they are held all the same, because the moment the camera turns towards ground whose fine
    // tiles have not arrived, one of them is what covers it. Without them that ground is a hole until
    // the fine tiles come, which is what flickering while the camera moves was.
    std::vector<std::int64_t> m_kept;
    std::vector<std::pair<double, quantizedmesh::tile_address>> m_wishes;
    std::vector<quantizedmesh::tile_address> m_wanted;
    std::vector<terrain_tile_loader::payload> m_arrived;
    std::unordered_set<std::int64_t> m_asked;    // what the last walk asked the loader for
    // per tile of the last walk, whether all four of its children can be drawn, and so whether the
    // walk may go past it. Worked out once and read again when the tiles to draw are picked
    std::unordered_map<std::int64_t, bool> m_maydescend;

    // what the draw hands to gl, kept between frames so that a frame allocates nothing. one set for
    // each arena
    std::vector<std::int32_t> m_counts[ 2 ];
    std::vector<void const *> m_offsets[ 2 ];
    std::vector<std::int32_t> m_bases[ 2 ];

    glm::dvec3 m_walkpoint { 0.0 };
    bool m_walked { false };
    double m_pixelsperunit { 0.0 };
    double m_detail { 3.0 };
    double m_walkdetail { 0.0 };
    float m_range { 6000.f };
    float m_walkrange { -1.f };

    std::optional<gl::program> m_shader;
    struct uniform_locations {
        int origin { -1 };       // where the terrain's origin is, relative to the camera
        int world { -1 };        // and where it is in the world
        int ground { -1 };
        int materials { -1 };
    } m_uniforms;

    terrain_statistics m_stats;
    std::uint64_t m_frame { 0 };
    std::size_t m_reportedresidency { 0 };
    std::size_t m_reporteddrawn { 0 };
    std::uint64_t m_reportedframe { 0 };
    bool m_reported { false };
    unsigned m_depthrank { 0 };
    // what the buffers may hold before tiles nobody asked for lately are let go
    std::size_t m_budget { 192u << 20 };
    // Tiles put on the card in one frame. Generous on purpose: what is being absorbed is a read that
    // has already happened, and the sooner the levels between a new tile and the root are all there,
    // the shorter the window in which the ground has to be drawn coarse
    std::size_t m_putsperframe { 64 };
    bool m_ready { false };
    std::string m_path;
};

}  // export
