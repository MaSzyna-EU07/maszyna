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

#include "scene/quantizedmeshreader.h"

export module eu07.rendering.terrainclipmap;
import eu07.gl.shader;
import eu07.gl.buffer;
import eu07.gl.vao;
import eu07.glm;
import eu07.rendering.terraintileloader;
import eu07.rendering.terrainstatus;
import eu07.rendering.terraintilepool_tin;
import eu07.rendering.terrainground;

export {

// Draws TIN (Triangulated Irregular Network) terrain from Quantized Mesh tiles.
//
// Replaces the old heightfield texture array approach with vertex/index buffers per tile.
// Each tile is an independent triangulated mesh with multiple LOD levels. The terrain adapts
// density to detail: fine triangulation along track corridors, coarse in open fields.
//
// CDLOD (Continuous Distance LOD): tiles select their LOD based on distance from camera,
// with edge stitching to prevent cracks between LOD transitions.
//
// The class owns GL resources and file handles. Given a viewpoint and visibility test,
// it loads, uploads, and renders appropriate tiles each frame.
class terrain_clipmap {

public:
    // whether a sphere, in world space, can be seen: centre and radius
    using visibility = std::function<bool( glm::dvec3 const &, float )>;

    terrain_clipmap() = default;
    ~terrain_clipmap();
    terrain_clipmap( terrain_clipmap const & ) = delete;
    terrain_clipmap &operator=( terrain_clipmap const & ) = delete;

    // opens a cooked TIN directory and builds the gl resources. returns false when the
    // directory cannot be read, leaving the object unusable but safe to destroy
    bool open( std::string const &Path );
    bool ready() const { return m_ready; }
    void close();

    // asks for the tiles around the viewpoint, uploads a bounded number of those that have
    // arrived, and retires the ones that left range. safe to call every frame: the reading
    // and decoding happen on the loader's thread
    void update( glm::dvec3 const &Viewpoint );
    // draws the resident tiles in range that the test says can be seen. the caller is
    // expected to have the scene and model uniform blocks already set for a camera-relative
    // pass. tile positions are passed as uniforms per draw
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
    // metres per tile side
    float tilesize() const { return m_tilesize; }
    // number of LOD levels available
    std::uint32_t levels() const { return m_lodlevels; }
    std::size_t backlog() const { return m_loader.backlog(); }

private:
    // one tile resident on the gpu: VBO/IBO handle
    struct resident_tile {
        std::int32_t x { 0 }, z { 0 };
        std::uint32_t level { 0 };
        terrain_tile_pool_tin::tile_handle gpu_data;
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
    void resolve_uniforms();
    void report_residency();

    static std::int64_t key( std::int32_t const X, std::int32_t const Z ) {
        return ( static_cast<std::int64_t>( X ) << 32 ) ^ static_cast<std::uint32_t>( Z );
    }

    // Tile management
    quantizedmesh::reader m_reader;
    terrain_tile_loader m_loader;
    terrain_tile_pool_tin m_pool;
    
    std::vector<terrain_tile_loader::request> m_wanted;
    std::vector<terrain_tile_loader::payload> m_arrived;
    // what the last scan found in range, and the level wanted for each tile
    std::vector<in_range> m_inrange;
    std::unordered_map<std::int64_t, std::uint32_t> m_wantedlevel;
    glm::dvec3 m_scanpoint { 0.0 };
    float m_scanrange { -1.f };
    double m_finestwanted { 0.0 };
    double m_finest { 0.0 };
    
    // Resident tiles (key = tile_key(x,z))
    std::unordered_map<std::int64_t, resident_tile> m_tiles;
    
    // Ground texture array (materials)
    terrain_ground m_ground;
    
    // Shader and uniforms
    std::optional<gl::program> m_shader;
    struct uniform_locations {
        int tile_offset { -1 };      // vec3: tile origin relative to camera
        int ground { -1 };            // sampler2DArray: material textures
    } m_uniforms;

    // Configuration
    float m_tilesize { 256.0f };
    std::uint32_t m_lodlevels { 4 };
    terrain_statistics m_stats;
    std::uint64_t m_frame { 0 };
    std::size_t m_reportedresidency { 0 };
    bool m_reported { false };
    float m_range { 6000.f };
    unsigned m_depthrank { 0 };
    std::size_t m_budget { 2048 };         // tiles allowed resident
    std::size_t m_uploadsperframe { 32 };
    bool m_ready { false };
    std::string m_path;
};

}  // export
