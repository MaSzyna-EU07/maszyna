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
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "global_include/interfaces/ITexture_macros.h"
#include "scene/heightfieldreader.h"

export module eu07.rendering.terrainclipmap;
import eu07.gl.shader;
import eu07.gl.buffer;
import eu07.gl.vao;
import eu07.glm;
import eu07.rendering.terraintileloader;
import eu07.rendering.terrainstatus;
import eu07.model.material;
import eu07.global_include.interfaces.itexture;

export {

// Draws the cooked terrain heightfield.
//
// Not a toroidal clipmap, despite what the plan called it. The measurements say the
// terrain is a ribbon: 390 km2 of surface inside a 51 x 46 km rectangle, twelve percent
// of the tile bounding box occupied. Clipmap rings sized for that rectangle would be
// mostly empty, so this draws the cooked tiles themselves, each at the mip level its
// distance earns. The tiles were laid out for exactly this - sorted by morton code, one
// shared sample of overlap so neighbouring levels meet without skirts.
//
// The class owns its gl resources and the file handle, and knows nothing about the
// scenery, the camera class or the rest of the renderer: it is given a viewpoint and a
// pair of matrices, and draws.
class terrain_clipmap {

public:
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
    // draws the resident tiles. the caller is expected to have the scene and model
    // uniform blocks already set for a camera-relative pass, the way the rest of the
    // renderer does: tile positions reach the shader relative to the viewpoint, so
    // nothing here needs a matrix of its own
    void render( glm::dvec3 const &Viewpoint );

    // how far from the viewpoint terrain is drawn, in metres
    void range( float const Range ) { m_range = Range; }
    float range() const { return m_range; }

    // metres between samples at the finest level; a scenery's fields are drawn from the
    // finest to the coarsest so that the depth test keeps the better surface
    float gridstep() const { return m_reader.gridstep(); }
    // sinks this field by the given amount, so a coarse field cannot win fragments from
    // a finer one describing the same ground
    void depthbias( float const Metres ) { m_depthbias = Metres; }

    terrain_statistics const &stats() const { return m_stats; }
    std::string const &path() const { return m_path; }
    float tilesize() const { return static_cast<float>( m_reader.tilesize() ); }
    std::uint32_t levels() const { return m_reader.levels(); }
    std::size_t backlog() const { return m_loader.backlog(); }

private:
    // one tile resident on the gpu. heights are unsigned so that the no-data value stays
    // exactly what the cooker wrote instead of being normalised into a float
    struct resident_tile {
        std::int32_t x { 0 }, z { 0 };
        std::uint32_t level { 0 };      // mip level currently uploaded
        std::uint32_t side { 0 };       // samples per side at that level
        std::uint32_t heighttexture { 0 };
        std::uint32_t materialtexture { 0 };
        float minheight { 0.f }, maxheight { 0.f };
        std::uint64_t lastused { 0 };
        // materials appearing in this tile. a tile is drawn once per entry, since each
        // needs its own texture bound, and most tiles carry only one or two
        std::vector<std::uint8_t> materials;
    };

    // mip level a tile at the given distance is drawn at
    std::uint32_t level_for( double const Distance ) const;
    // works out which tiles are in range and at what level, and tells the loader what is
    // missing. the answer only changes once the camera has moved a fair part of a tile
    void scan( glm::dvec3 const &Viewpoint );
    // puts a tile the loader has read on the gpu, replacing whatever level it had before
    void upload( terrain_tile_loader::payload const &Tile );
    void retire( resident_tile &Tile );
    void build_indices();
    void build_palette();
    // resolves the cooked material names to the engine's materials, once
    void resolve_materials();
    void resolve_uniforms();
    void report_residency();

    static std::int64_t key( std::int32_t const X, std::int32_t const Z ) { return heightfield::tile_key( X, Z ); }

    // for the file's layout only: tile coordinates, levels, materials. tiles themselves are
    // read by the loader, on its own handle
    heightfield::reader m_reader;
    terrain_tile_loader m_loader;
    std::vector<terrain_tile_loader::request> m_wanted;
    std::vector<terrain_tile_loader::payload> m_arrived;
    // one tile within range, and the level its distance earns
    struct in_range {
        std::int64_t key;
        std::uint32_t level;
    };
    // what the last scan found in range, and the level wanted for each tile, to tell a useful
    // arrival from one the camera has already moved past
    std::vector<in_range> m_inrange;
    std::unordered_map<std::int64_t, std::uint32_t> m_wantedlevel;
    glm::dvec3 m_scanpoint { 0.0 };
    float m_scanrange { -1.f };
    std::unordered_map<std::int64_t, resident_tile> m_tiles;
    // one index buffer per mip level, over a grid of that level's sample count. vertices
    // are computed in the shader from gl_VertexID, so there is no vertex buffer at all.
    // held by pointer because a gl::buffer owns a name it cannot hand to a copy
    std::vector<std::unique_ptr<gl::buffer>> m_indices;
    std::vector<std::uint32_t> m_indexcounts;
    std::optional<gl::vao> m_vao;
    std::optional<gl::program> m_shader;
    // resolved once at link time; -1 for a uniform the compiler dropped, which gl ignores
    struct uniform_locations {
        int tileorigin { -1 }, samplestep { -1 };
        int heightbias { -1 }, heightscale { -1 }, nodata { -1 }, side { -1 }, morph { -1 };
        int heights { -1 }, materials { -1 }, palette { -1 };
        int groundtexture { -1 }, groundscale { -1 }, drawnmaterial { -1 }, tileworld { -1 };
        int hastexture { -1 }, fallback { -1 };
    } m_uniforms;
    std::vector<glm::vec3> m_palette;
    // one entry per cooked material: the engine texture to bind and how much ground one
    // repeat of it covers
    struct material_binding {
        texture_handle texture { null_handle };
        float scale { 4.f };
    };
    std::vector<material_binding> m_materials;

    terrain_statistics m_stats;
    std::uint64_t m_frame { 0 };
    std::size_t m_reportedresidency { 0 };
    bool m_reported { false };
    float m_range { 6000.f };
    float m_depthbias { 0.f };
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
