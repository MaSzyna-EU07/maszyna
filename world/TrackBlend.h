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
// an end the vertices carry the weight of the track joined there (y, z, w of their user data for the ends p1, p2 and p4 of a
// switch) and the track is drawn with a material blending its texture with theirs. the ballast comes in as grains, the rails
// fade into each other. a plain track blends its rails and its ballast, a switch the trackbed made for it
namespace track_blend {

// how far from an end the textures blend, by default
float constexpr default_length { 3.f };

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
    // the weights of the ends put in the user data of the vertices (relative to the origin of the track), made as long as the
    // vertices where it's shorter; what the user data hold at x is kept
    void weigh( gfx::vertex_array const &Vertices, gfx::userdata_array &Userdata ) const;
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
