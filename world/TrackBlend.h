/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>

#include <glm/glm.hpp>

#include "utilities/Classes.h"
#include "interfaces/ITexture.h"
#include "rendering/geometrybank.h"

class TTrack;

// the rails and the ballast of a track blended into those of the tracks joined at its ends, where their textures differ: near
// an end the vertices carry how far they are from it (y, z, w of their user data for the ends p1, p2 and p4 of a
// switch) and the track is drawn with a material fading its texture into theirs. a plain track blends its rails and its
// ballast, a switch the trackbed made for it
namespace track_blend {

// how far from an end the textures blend, by default
float constexpr default_length { 3.f };

// the order of the lengths of TTrack::m_blend
enum class kind {
    rails,
    ballast
};

struct blending {
    struct end {
        bool active { false };
        glm::dvec3 point { 0.0 }; // relative to the origin of the track
        glm::dvec3 inward { 0.0 }; // unit vector from the end into the track
        double length { 0.0 };
        float peak { 0.f }; // the weight at the end: a half where the track joined there blends back, whole where it doesn't
    };
    std::array<end, 3> ends;
    material_handle material { null_handle }; // the material the blending is drawn with, null where nothing's blended

    bool any() const {
        return material != null_handle; }
    // how far the vertices (relative to the origin of the track) are into it from each end, in the length of its blending, put
    // in their user data: distance / length where the track joined blends back, single_mark - distance / length where it doesn't,
    // none_mark where nothing's blended at the end. a distance runs straight along the vertices, so it's led between them exactly
    // and the shader (trackblend.glsl) works the weight out of it for each fragment, however far apart the vertices are. the user
    // data are made as long as the vertices where they're shorter; what they hold at x is kept
    void weigh( gfx::vertex_array const &Vertices, gfx::userdata_array &Userdata ) const;
    static float constexpr single_mark { -100.f };
    static float constexpr none_mark { 10000.f };
};

// how the rails or the ballast of the track blend into those of the tracks joined to it, nothing for the tracks which don't
blending setup( TTrack const &Track, kind const Kind );

// while one is alive, the chunks of geometry inserted with track_rails::insert_chunk() get the weights of the blending
class weighing {
public:
    explicit weighing( blending const *Blending );
    ~weighing();
    weighing( weighing const & ) = delete;
    weighing &operator=( weighing const & ) = delete;
private:
    blending const *m_previous { nullptr };
};
// the blending the chunks inserted now are weighed by, null when they aren't
blending const *weighed();

} // namespace track_blend
