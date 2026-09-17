/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "global_include/interfaces/ITexture_macros.h"

export module eu07.rendering.terrainground;
import eu07.gl.shader;
import eu07.gl.vao;
import eu07.glm;
import eu07.utilities.classes;

export {

// The ground textures of one heightfield, gathered into a single texture array so that a tile
// of any number of materials is drawn in one call, and a table of what the shader needs per
// material: the colour to use while there is no texture, and how many metres one repeat
// covers.
//
// The scenery's textures are the engine's own - compressed, loaded in the background, in
// whatever sizes their authors made them - so they cannot be blitted or copied into an array
// directly. Each one is drawn into its layer by a small shader once the engine has it ready;
// until then the material shows its placeholder colour. The layers are sRGB, as the sources
// are.
class terrain_ground {

public:
    terrain_ground() = default;
    ~terrain_ground();
    terrain_ground( terrain_ground const & ) = delete;
    terrain_ground &operator=( terrain_ground const & ) = delete;

    // resolves the cooked material names to the engine's textures and allocates the array.
    // returns false when the copy shader cannot be built
    bool create( std::vector<std::string> const &Names, std::vector<float> const &Scales );
    void destroy();
    // copies into the array every texture that has become ready since the last call. changes
    // gl state for the duration and puts back what it touched
    void update();

    std::uint32_t textures() const { return m_array; }
    std::uint32_t table() const { return m_table; }

private:
    struct material {
        texture_handle texture { null_handle };
        bool copied { false };
        glm::vec3 fallback { 0.45f, 0.5f, 0.4f };
        float scale { 4.f };
    };

    void upload_table();

    // side of a layer, halved when many materials would make the array large
    static constexpr std::uint32_t layerside { 512 };
    static constexpr std::size_t budgetbytes { 96u << 20 };

    std::vector<material> m_materials;
    std::uint32_t m_side { layerside };
    std::uint32_t m_array { 0 };
    std::uint32_t m_table { 0 };
    std::uint32_t m_framebuffer { 0 };
    std::size_t m_pending { 0 };
    std::optional<gl::program> m_copy;
    std::optional<gl::vao> m_vao;
};

}
