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
#include <chrono>
#include <deque>
#include <limits>
#include <mutex>
#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <cstddef>
#include <cstdint>
#include <vector>
#include <optional>

export module eu07.rendering.opengl33precipitation;
import eu07.glm;
import eu07.gl.buffer;
import eu07.global_include.interfaces.itexture;
import eu07.model.texture;
import eu07.gl.vao;
import eu07.gl.shader;

export {



class opengl33_precipitation {

public:
// constructors
    opengl33_precipitation() = default;
// destructor
	~opengl33_precipitation();
// methods
    void
        update();
	void
        render();

private:
// methods
    void create( int const Tesselation );
// members
    std::vector<glm::vec3> m_vertices;
    std::vector<glm::vec2> m_uvs;
    std::vector<std::uint16_t> m_indices;
    texture_handle m_texture { -1 };
    float m_overcast { -1.f }; // cached overcast level, difference from current state triggers texture update
    std::optional<gl::buffer> m_vertexbuffer;
    std::optional<gl::buffer> m_uvbuffer;
    std::optional<gl::buffer> m_indexbuffer;
    std::optional<gl::program> m_shader;
    std::optional<gl::vao> m_vao;
};

}  // export
