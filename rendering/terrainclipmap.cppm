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
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "scene/heightfieldreader.h"

export module eu07.rendering.terrainclipmap;
import eu07.gl.shader;
import eu07.gl.buffer;
import eu07.gl.vao;
import eu07.glm;
import eu07.rendering.terraintileloader;
import eu07.rendering.terrainstatus;
import eu07.rendering.terraintilepool;
import eu07.rendering.terrainground;

export {

// Draws the cooked terrain heightfield.
//
// Not a toroidal clipmap, despite what the plan called it. The measurements say the terrain is
// a ribbon: 390 km2 of surface inside a 51 x 46 km rectangle, twelve percent of the tile
// bounding box occupied. Clipmap rings sized for that rectangle would be mostly empty, so this
// draws the cooked tiles themselves, each at the mip level its distance earns (CDLOD), morphing
// into the next level as it nears the end of its range.
//
// Tiles live on the gpu in one texture array per level (terrain_tile_pool), and the ground
// textures of every material in another (terrain_ground). Each frame the tiles in view are
// culled against the frustum and gathered per level into a buffer of instance data, and each
// level is drawn in one instanced call; the fragment shader picks and blends the materials of
// the four samples around it.
//
// The class owns its gl resources and the file handle, and knows nothing about the scenery,
// the camera class or the rest of the renderer: it is given a viewpoint and a visibility test,
// and draws.
class terrain_clipmap {

public:
    // whether a sphere, in world space, can be seen: centre and radius
    using visibility = std::function<bool( glm::dvec3 const &, float )>;

    terrain_clipmap() = default;
    ~terrain_clipmap();
    terrain_clipmap( terrain_clipmap const & ) = delete;
    terrain_clipmap &operator=( terrain_clipmap const & ) = delete;

    // opens a cooked heightfield and builds the gl resources. returns false when the
    // file cannot be read, leaving the object unusable but safe to destroy
    bool open( std::string const &Path );
    bool ready() const { return m_ready; }
    void close();

    // asks for the tiles around the viewpoint, uploads a bounded number of those that have
    // arrived, and retires the ones that left range. safe to call every frame: the reading
    // and decoding happen on the loader's thread, and a tile waiting for a finer level keeps
    // being drawn at the level it has
    void update( glm::dvec3 const &Viewpoint );
    // draws the resident tiles in range that the test says can be seen. the caller is
    // expected to have the scene and model uniform blocks already set for a camera-relative
    // pass, the way the rest of the renderer does: tile positions reach the shader relative
    // to the viewpoint, so nothing here needs a matrix of its own
    void render( glm::dvec3 const &Viewpoint, visibility const &Visible );

    // how far from the viewpoint terrain is drawn, in metres
    void range( float const Range ) { m_range = Range; }
    float range() const { return m_range; }
    // what the screen can resolve: the viewport's pixels per unit of size over distance, and
    // how many pixels a sample spacing may cover. decides how far each level reaches
    void detail( double const Pixelsperunit, double const Pixels );
    // the finest level's range the tiles in range were last worked out with
    float finest() const { return static_cast<float>( m_finest ); }

    // metres between samples at the finest level; a scenery's fields are drawn from the
    // finest to the coarsest so that the depth test keeps the better surface
    float gridstep() const { return m_reader.gridstep(); }
    // pushes this field back in depth by the given rank, so that geometry lying on it wins the
    // depth test, and a coarser field loses to a finer one describing the same ground. the
    // offset is in depth, scaled by slope, so it holds at any distance and on any incline
    void depthrank( unsigned const Rank ) { m_depthrank = Rank; }

    terrain_statistics const &stats() const { return m_stats; }
    std::string const &path() const { return m_path; }
    float tilesize() const { return static_cast<float>( m_reader.tilesize() ); }
    std::uint32_t levels() const { return m_reader.levels(); }
    std::size_t backlog() const { return m_loader.backlog(); }

private:
    // one tile resident on the gpu: a layer in its level's pool
    struct resident_tile {
        std::int32_t x { 0 }, z { 0 };
        std::uint32_t level { 0 };
        std::uint32_t slot { 0 };
        float minheight { 0.f }, maxheight { 0.f };
        std::uint64_t lastused { 0 };
    };

    // one tile within range, and the level its distance earns
    struct in_range {
        std::int64_t key;
        std::uint32_t level;
    };

    // mip level a tile at the given distance is drawn at
    std::uint32_t level_for( double const Distance ) const;
    // works out which tiles are in range and at what level, and tells the loader what is
    // missing. the answer only changes once the camera has moved a fair part of a tile
    void scan( glm::dvec3 const &Viewpoint );
    // puts a tile the loader has read on the gpu, replacing whatever level it had before
    void upload( terrain_tile_loader::payload const &Tile );
    void retire( resident_tile const &Tile );
    void build_indices();
    void resolve_uniforms();
    void report_residency();

    static std::int64_t key( std::int32_t const X, std::int32_t const Z ) { return heightfield::tile_key( X, Z ); }

    // for the file's layout only: tile coordinates, levels, materials. tiles themselves are
    // read by the loader, on its own handle
    heightfield::reader m_reader;
    terrain_tile_loader m_loader;
    std::vector<terrain_tile_loader::request> m_wanted;
    std::vector<terrain_tile_loader::payload> m_arrived;
    // what the last scan found in range, and the level wanted for each tile, to tell a useful
    // arrival from one the camera has already moved past
    std::vector<in_range> m_inrange;
    std::unordered_map<std::int64_t, std::uint32_t> m_wantedlevel;
    glm::dvec3 m_scanpoint { 0.0 };
    float m_scanrange { -1.f };
    // the finest level's range the screen asks for, and the one the last scan used. levels and
    // morphing both follow the scanned one, so they always agree
    double m_finestwanted { 0.0 };
    double m_finest { 0.0 };
    std::unordered_map<std::int64_t, resident_tile> m_tiles;

    // per level: the tiles' storage, the index buffer over that level's grid, and the
    // instances gathered for the draw. vertices are computed in the shader from gl_VertexID,
    // so there is no vertex buffer at all. held by pointer because the gl wrappers own names
    // they cannot hand to a copy
    std::vector<std::unique_ptr<terrain_tile_pool>> m_pools;
    std::vector<std::unique_ptr<gl::buffer>> m_indices;
    std::vector<std::uint32_t> m_indexcounts;
    std::vector<std::vector<float>> m_instances;
    // instance data for the draw being made, read by the vertex shader as a buffer texture
    std::optional<gl::buffer> m_instancebuffer;
    std::uint32_t m_instancetexture { 0 };
    terrain_ground m_ground;
    std::optional<gl::vao> m_vao;
    std::optional<gl::program> m_shader;
    // resolved once at link time; -1 for a uniform the compiler dropped, which gl ignores
    struct uniform_locations {
        int samplestep { -1 }, side { -1 }, morph { -1 };
        int heightbias { -1 }, heightscale { -1 }, nodata { -1 };
        int heights { -1 }, materials { -1 }, instances { -1 };
        int ground { -1 }, materialtable { -1 };
    } m_uniforms;

    terrain_statistics m_stats;
    std::uint64_t m_frame { 0 };
    std::size_t m_reportedresidency { 0 };
    bool m_reported { false };
    float m_range { 6000.f };
    unsigned m_depthrank { 0 };
    // tiles allowed to stay resident; at 129 samples a tile costs about 50 kB of texture
    std::size_t m_budget { 2048 };
    // tiles put on the gpu per frame. a tile is about 50 kB of texture, so this is not about
    // the upload itself but about never spending a frame on terrain the camera will reach
    // later anyway
    std::size_t m_uploadsperframe { 32 };
    bool m_ready { false };
    std::string m_path;
};

}  // export
