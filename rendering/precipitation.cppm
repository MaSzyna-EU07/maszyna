/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/



// based on "Rendering Falling Rain and Snow"
// by Niniane Wang, Bretton Wade
module;
#include <array>
#include <chrono>
#include <cstddef>
#include <deque>
#include <limits>
#include <mutex>
#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

export module eu07.rendering.precipitation;
import eu07.glm;

export {

class basic_precipitation {

public:
// constructors
    basic_precipitation() = default;
// destructor
	~basic_precipitation();
// methods
    bool
        init();
    void
        update();
    float
        get_textureoffset() const;

    glm::dvec3 m_cameramove{ 0.0 };

private:
// members
    float m_textureoffset { 0.f };
    float m_moverate { 30 * 0.001f };
    float m_moverateweathertypefactor { 1.f }; // medium-dependent; 1.0 for snow, faster for rain
    glm::dvec3 m_camerapos { 0.0 };
    bool m_freeflymode { true };
    bool m_windowopen { true };
    int m_activecab{ 0 };
    glm::dvec3 m_cabcameramove{ 0.0 };
};

}  // export
