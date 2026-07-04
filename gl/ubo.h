#pragma once

#include "object.h"
#include "bindable.h"
#include "buffer.h"
#include <glm/glm.hpp>

#define UBS_PAD(x) uint8_t PAD[x]

// 12-byte vec3 for structs that must match an exact binary layout (UBOs, vertex data).
// Stays packed regardless of GLM_FORCE_DEFAULT_ALIGNED_GENTYPES.
// Guard prevents redefinition when stdafx.h also defines it.
#ifndef PACKED_VEC3_DEFINED
#define PACKED_VEC3_DEFINED
using packed_vec3 = glm::vec<3, float, glm::packed_highp>;
#endif

namespace gl
{
    class ubo : public buffer
    {
        int index;

    public:
	    ubo(size_t size, int index, GLenum hint = GL_DYNAMIC_DRAW);

        void bind_uniform();

        void update(const uint8_t *data, int offset, GLsizeiptr size);
        template <typename T> void update(const T &data, size_t offset = 0)
        {
            update(reinterpret_cast<const uint8_t*>(&data), offset, sizeof(data));
        }
    };

    // layout std140
    // structs must match with GLSL
    // ordered to minimize padding

#pragma pack(push, 1)

    constexpr size_t MAX_TEXTURES = 8;
    constexpr size_t ENVMAP_SIZE = 1024;
    constexpr size_t MAX_CASCADES = 3;
    constexpr size_t HELPER_TEXTURES = 4;
    constexpr size_t SHADOW_TEX = MAX_TEXTURES + 0;
    constexpr size_t ENV_TEX = MAX_TEXTURES + 1;
    constexpr size_t HEADLIGHT_TEX = MAX_TEXTURES + 2;

    struct scene_ubs
    {
        glm::mat4 projection;
        glm::mat4 inv_view;
        glm::mat4 lightview[MAX_CASCADES];
        packed_vec3 cascade_end;
        float time;
    	glm::vec4 rain_params;
    	glm::vec4 wiper_pos;
    	glm::vec4 wiper_timer_out;
    	glm::vec4 wiper_timer_return;
    };

    static_assert(sizeof(scene_ubs) == 400, "bad size of ubs");

    constexpr size_t MAX_PARAMS = 3;

    struct model_ubs
    {
        glm::mat4 modelview;
        glm::mat3x4 modelviewnormal;
        glm::vec4 param[MAX_PARAMS];

        glm::mat4 future;
        float opacity;
        float emission;
        float fog_density;
        float alpha_mult;
        float shadow_tone;
        UBS_PAD(12);

        void set_modelview(const glm::mat4 &mv)
        {
            modelview = mv;
            // normal matrix = transpose(inverse(modelview)). The modelview is
            // always affine, so its 3x3 normal matrix depends only on the
            // upper-left 3x3 block; inverting that mat3 directly is markedly
            // cheaper than a full mat4 inverse and yields an identical result
            // (for affine M, mat3(inverse(M)) == inverse(mat3(M))).
            modelviewnormal = glm::mat3x4(glm::transpose(glm::inverse(glm::mat3(mv))));
        }
    };

    static_assert(sizeof(model_ubs) == 208 + 16 * MAX_PARAMS, "bad size of ubs");

    // maximum number of instances per single GPU-instanced draw call.
    // 256 mat4 = 16 KiB, the guaranteed minimum UBO size. Bigger batches must split.
    constexpr size_t MAX_INSTANCES_PER_BATCH = 256;

    struct instance_ubs
    {
        glm::mat4 instance_modelview[MAX_INSTANCES_PER_BATCH];
    };

    static_assert(sizeof(instance_ubs) == 64 * MAX_INSTANCES_PER_BATCH, "bad size of instance_ubs");

    struct light_element_ubs
    {
        enum type_e
        {
            SPOT = 0,
            POINT,
            DIR,
            HEADLIGHTS
        };

        packed_vec3 pos;
        type_e type;

        packed_vec3 dir;
        float in_cutoff;

        packed_vec3 color;
        float out_cutoff;

        float linear;
        float quadratic;

		float intensity;
		float ambient;

        glm::mat4 headlight_projection;
        glm::vec4 headlight_weights;
    };

    static_assert(sizeof(light_element_ubs) == 144, "bad size of ubs");

    constexpr size_t MAX_LIGHTS = 8;

    struct light_ubs
    {
        packed_vec3 ambient;
		UBS_PAD(4);

        packed_vec3 fog_color;
        uint32_t lights_count;

        light_element_ubs lights[MAX_LIGHTS];
    };

    static_assert(sizeof(light_ubs) == 32 + sizeof(light_element_ubs) * MAX_LIGHTS, "bad size of ubs");

#pragma pack(pop)
}
