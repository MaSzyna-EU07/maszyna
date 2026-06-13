/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "scene/eu7/eu7_types.h"

#include <cmath>
#include <vector>

namespace scene::eu7 {

namespace detail {

inline constexpr double kPi { 3.14159265358979323846 };

struct ScratchLocation {
    std::vector<glm::dvec3> offset_stack;
    std::vector<glm::dvec3> scale_stack;
    glm::dvec3 rotation{};
};

[[nodiscard]] inline ScratchLocation
scratch_from_transform( Eu7TransformContext const &transform ) {
    ScratchLocation scratch;
    scratch.offset_stack = transform.origin_stack;
    scratch.scale_stack = transform.scale_stack;
    scratch.rotation = transform.rotation;
    return scratch;
}

} // namespace detail

[[nodiscard]] inline glm::dvec3
inverse_transform_point( glm::dvec3 location, Eu7TransformContext const &transform ) {
    auto const scratch { detail::scratch_from_transform( transform ) };

    if( !scratch.offset_stack.empty() ) {
        auto const &off { scratch.offset_stack.back() };
        location.x -= off.x;
        location.y -= off.y;
        location.z -= off.z;
    }

    if( !scratch.scale_stack.empty() ) {
        auto const &sc { scratch.scale_stack.back() };
        if( sc.x != 0.0 ) {
            location.x /= sc.x;
        }
        if( sc.y != 0.0 ) {
            location.y /= sc.y;
        }
        if( sc.z != 0.0 ) {
            location.z /= sc.z;
        }
    }

    if( scratch.rotation.y != 0.0 ) {
        auto const rad { -scratch.rotation.y * ( detail::kPi / 180.0 ) };
        auto const c { std::cos( rad ) };
        auto const s { std::sin( rad ) };
        auto const x { location.x * c + location.z * s };
        auto const z { -location.x * s + location.z * c };
        location.x = x;
        location.z = z;
    }

    return location;
}

inline void
inverse_transform_shape_vertices(
    std::vector<Eu7WorldVertex> &vertices,
    Eu7TransformContext const &transform ) {
    auto const scratch { detail::scratch_from_transform( transform ) };

    if( !scratch.offset_stack.empty() ) {
        auto const &off { scratch.offset_stack.back() };
        for( auto &vtx : vertices ) {
            vtx.position.x -= off.x;
            vtx.position.y -= off.y;
            vtx.position.z -= off.z;
        }
    }

    if( scratch.rotation.x != 0.0 || scratch.rotation.y != 0.0 || scratch.rotation.z != 0.0 ) {
        auto const rx { -scratch.rotation.x * ( detail::kPi / 180.0 ) };
        auto const ry { -scratch.rotation.y * ( detail::kPi / 180.0 ) };
        auto const rz { -scratch.rotation.z * ( detail::kPi / 180.0 ) };

        auto rotate_y = [&]( glm::dvec3 &v ) {
            auto const c { std::cos( ry ) };
            auto const s { std::sin( ry ) };
            auto const x { v.x * c + v.z * s };
            auto const z { -v.x * s + v.z * c };
            v.x = x;
            v.z = z;
        };
        auto rotate_x = [&]( glm::dvec3 &v ) {
            auto const c { std::cos( rx ) };
            auto const s { std::sin( rx ) };
            auto const y { v.y * c - v.z * s };
            auto const z { v.y * s + v.z * c };
            v.y = y;
            v.z = z;
        };
        auto rotate_z = [&]( glm::dvec3 &v ) {
            auto const c { std::cos( rz ) };
            auto const s { std::sin( rz ) };
            auto const x { v.x * c - v.y * s };
            auto const y { v.x * s + v.y * c };
            v.x = x;
            v.y = y;
        };

        for( auto &vtx : vertices ) {
            rotate_y( vtx.position );
            rotate_x( vtx.position );
            rotate_z( vtx.position );
            glm::dvec3 n { vtx.normal.x, vtx.normal.y, vtx.normal.z };
            rotate_y( n );
            rotate_x( n );
            rotate_z( n );
            vtx.normal = glm::vec3( static_cast<float>( n.x ), static_cast<float>( n.y ), static_cast<float>( n.z ) );
        }
    }
}

[[nodiscard]] inline bool
transform_is_empty( Eu7TransformContext const &transform ) {
    return (
        transform.origin_stack.empty() &&
        transform.scale_stack.empty() &&
        transform.rotation.x == 0.0 &&
        transform.rotation.y == 0.0 &&
        transform.rotation.z == 0.0 &&
        transform.group_depth == 0 );
}

[[nodiscard]] inline glm::dvec3
transform_point( glm::dvec3 location, Eu7TransformContext const &transform ) {
    if( transform.rotation.y != 0.0 ) {
        auto const rad { transform.rotation.y * ( detail::kPi / 180.0 ) };
        auto const c { std::cos( rad ) };
        auto const s { std::sin( rad ) };
        auto const x { location.x * c + location.z * s };
        auto const z { -location.x * s + location.z * c };
        location.x = x;
        location.z = z;
    }

    if( !transform.scale_stack.empty() ) {
        auto const &sc { transform.scale_stack.back() };
        if( sc.x != 0.0 ) {
            location.x *= sc.x;
        }
        if( sc.y != 0.0 ) {
            location.y *= sc.y;
        }
        if( sc.z != 0.0 ) {
            location.z *= sc.z;
        }
    }

    if( !transform.origin_stack.empty() ) {
        auto const &off { transform.origin_stack.back() };
        location.x += off.x;
        location.y += off.y;
        location.z += off.z;
    }

    return location;
}

// Wzgledne wektory kontrolne toru (cp_out/cp_in) — rotacja i skala, bez translacji origin.
[[nodiscard]] inline glm::dvec3
transform_vector( glm::dvec3 vector, Eu7TransformContext const &transform ) {
    if( transform.rotation.y != 0.0 ) {
        auto const rad { transform.rotation.y * ( detail::kPi / 180.0 ) };
        auto const c { std::cos( rad ) };
        auto const s { std::sin( rad ) };
        auto const x { vector.x * c + vector.z * s };
        auto const z { -vector.x * s + vector.z * c };
        vector.x = x;
        vector.z = z;
    }

    if( !transform.scale_stack.empty() ) {
        auto const &sc { transform.scale_stack.back() };
        if( sc.x != 0.0 ) {
            vector.x *= sc.x;
        }
        if( sc.y != 0.0 ) {
            vector.y *= sc.y;
        }
        if( sc.z != 0.0 ) {
            vector.z *= sc.z;
        }
    }

    return vector;
}

inline void
compose_node_transform( Eu7TransformContext &node, Eu7TransformContext const &prefix ) {
    if( transform_is_empty( prefix ) ) {
        return;
    }

    auto const parent_origin {
        prefix.origin_stack.empty() ?
            glm::dvec3{} :
            prefix.origin_stack.back() };
    for( auto &offset : node.origin_stack ) {
        offset.x += parent_origin.x;
        offset.y += parent_origin.y;
        offset.z += parent_origin.z;
    }

    auto const parent_scale {
        prefix.scale_stack.empty() ?
            glm::dvec3{ 1.0, 1.0, 1.0 } :
            prefix.scale_stack.back() };
    for( auto &scale : node.scale_stack ) {
        scale.x *= parent_scale.x;
        scale.y *= parent_scale.y;
        scale.z *= parent_scale.z;
    }

    node.rotation.x += prefix.rotation.x;
    node.rotation.y += prefix.rotation.y;
    node.rotation.z += prefix.rotation.z;

    node.group_depth += prefix.group_depth;
}

void
compose_scene_with_include_prefix( Eu7Scene &scene, Eu7TransformContext const &prefix );

[[nodiscard]] inline glm::dvec3
subtract_origin_offset( glm::dvec3 const &world, Eu7TransformContext const &transform ) {
    glm::dvec3 local { world };
    if( !transform.origin_stack.empty() ) {
        auto const &off { transform.origin_stack.back() };
        local.x -= off.x;
        local.y -= off.y;
        local.z -= off.z;
    }
    return local;
}

} // namespace scene::eu7
