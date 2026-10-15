#pragma once

#include "object.h"
#include "bindable.h"
#include "buffer.h"
#include <glm/glm.hpp>
#include <cstddef>

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
            update(reinterpret_cast<const uint8_t*>(&data), static_cast<int>(offset), sizeof(data));
        }
    };

    // layout std140
    // structs must match with GLSL
    // ordered to minimize padding

#pragma pack(push, 1)

    const size_t MAX_TEXTURES = 8;
    const size_t ENVMAP_SIZE = 1024;
    const size_t MAX_CASCADES = 3;
    const size_t HELPER_TEXTURES = 4;
    const size_t SHADOW_TEX = MAX_TEXTURES + 0;
    const size_t ENV_TEX = MAX_TEXTURES + 1;
    const size_t HEADLIGHT_TEX = MAX_TEXTURES + 2;

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

    const size_t MAX_PARAMS = 3;

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
    const size_t MAX_INSTANCES_PER_BATCH = 256;

    struct instance_ubs
    {
        glm::mat4 instance_modelview[MAX_INSTANCES_PER_BATCH];
    };

    static_assert(sizeof(instance_ubs) == 64 * MAX_INSTANCES_PER_BATCH, "bad size of instance_ubs");

    // layout std430 (shader storage buffers)
    // structs must match with GLSL, offsets checked below

    // binding points of the shader storage blocks: layout(std430, binding = ...) in glsl_common
    const GLuint LIGHT_SSBO_BINDING = 0;

    // one light source, element of light_ssbo.lights[] (GLSL: light_s)
    struct light_element
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

    // std430 array stride of a struct is its size rounded up to its alignment (16, from mat4/vec4)
    static_assert(sizeof(light_element) == 144, "bad size of light_element");
    static_assert(sizeof(light_element) % 16 == 0, "light_element must be a whole std430 array stride");
    static_assert(offsetof(light_element, pos) == 0, "bad layout of light_element");
    static_assert(offsetof(light_element, type) == 12, "bad layout of light_element");
    static_assert(offsetof(light_element, dir) == 16, "bad layout of light_element");
    static_assert(offsetof(light_element, in_cutoff) == 28, "bad layout of light_element");
    static_assert(offsetof(light_element, color) == 32, "bad layout of light_element");
    static_assert(offsetof(light_element, out_cutoff) == 44, "bad layout of light_element");
    static_assert(offsetof(light_element, linear) == 48, "bad layout of light_element");
    static_assert(offsetof(light_element, quadratic) == 52, "bad layout of light_element");
    static_assert(offsetof(light_element, intensity) == 56, "bad layout of light_element");
    static_assert(offsetof(light_element, ambient) == 60, "bad layout of light_element");
    static_assert(offsetof(light_element, headlight_projection) == 64, "bad layout of light_element");
    static_assert(offsetof(light_element, headlight_weights) == 128, "bad layout of light_element");

    // capacity of light_data.lights; slot 0 is the sun
    const size_t MAX_LIGHTS = 8;

    // contents of light_ssbo: global lighting header followed by the light array
    struct light_data
    {
        packed_vec3 ambient;
		UBS_PAD(4);

        packed_vec3 fog_color;
        uint32_t lights_count;

        light_element lights[MAX_LIGHTS];
    };

    static_assert(offsetof(light_data, ambient) == 0, "bad layout of light_data");
    static_assert(offsetof(light_data, fog_color) == 16, "bad layout of light_data");
    static_assert(offsetof(light_data, lights_count) == 28, "bad layout of light_data");
    static_assert(offsetof(light_data, lights) == 32, "bad layout of light_data");
    static_assert(sizeof(light_data) == 32 + sizeof(light_element) * MAX_LIGHTS, "bad size of light_data");

    // light clusters (see gl/cluster_grid.h): the view frustum is cut into CLUSTER_GRID_X x CLUSTER_GRID_Y screen
    // tiles and CLUSTER_GRID_Z depth slices, spaced exponentially between the near and far plane of the pass.
    // cluster index = x + CLUSTER_GRID_X * (y + CLUSTER_GRID_Y * z), with x and y counted from the lower left corner
    const GLuint CLUSTER_GRID_X = 16;
    const GLuint CLUSTER_GRID_Y = 9;
    const GLuint CLUSTER_GRID_Z = 24;
    const GLuint CLUSTER_COUNT = CLUSTER_GRID_X * CLUSTER_GRID_Y * CLUSTER_GRID_Z;
    // capacity of the light list of a single cluster
    const GLuint MAX_LIGHTS_PER_CLUSTER = 64;
    // work group size of cluster_build.comp, which handles one cluster per invocation
    const GLuint CLUSTER_BUILD_GROUP_SIZE = 64;

    // binding points of the cluster buffers: layout(binding = ...) in glsl_common.
    // the uniform block numbers go together with scene (0), model (1) and instance (3) blocks
    const GLuint CLUSTER_UBO_BINDING = 2;
    const GLuint CLUSTER_AABB_SSBO_BINDING = 1;
    const GLuint CLUSTER_RANGE_SSBO_BINDING = 2;
    const GLuint CLUSTER_INDEX_SSBO_BINDING = 3;
    const GLuint CLUSTER_COUNTER_SSBO_BINDING = 4;

    // layout std140: grid parameters for the pass it was built for (GLSL: cluster_ubo)
    struct cluster_ubs
    {
        glm::mat4 projection;
        float znear;
        float zfar;
        // depth slice of a point with clip space w: floor(log(w) * slice_scale + slice_bias), znear..zfar covers all slices
        float slice_scale;
        float slice_bias;
        // size of the render target the tiles are laid over, in pixels
        float viewport_width;
        float viewport_height;
        // 0 if there is no grid for the pass (orthographic projection)
        uint32_t valid;
        UBS_PAD(4);
    };

    static_assert(sizeof(cluster_ubs) == 96, "bad size of cluster_ubs");
    static_assert(offsetof(cluster_ubs, projection) == 0, "bad layout of cluster_ubs");
    static_assert(offsetof(cluster_ubs, znear) == 64, "bad layout of cluster_ubs");
    static_assert(offsetof(cluster_ubs, zfar) == 68, "bad layout of cluster_ubs");
    static_assert(offsetof(cluster_ubs, slice_scale) == 72, "bad layout of cluster_ubs");
    static_assert(offsetof(cluster_ubs, slice_bias) == 76, "bad layout of cluster_ubs");
    static_assert(offsetof(cluster_ubs, viewport_width) == 80, "bad layout of cluster_ubs");
    static_assert(offsetof(cluster_ubs, viewport_height) == 84, "bad layout of cluster_ubs");
    static_assert(offsetof(cluster_ubs, valid) == 88, "bad layout of cluster_ubs");

    // layout std430: bounding box of a cluster in view space, element of cluster_aabb_ssbo (GLSL: cluster_aabb_s), w unused
    struct cluster_aabb
    {
        glm::vec4 lower;
        glm::vec4 upper;
    };

    static_assert(sizeof(cluster_aabb) == 32, "bad size of cluster_aabb");
    static_assert(offsetof(cluster_aabb, lower) == 0, "bad layout of cluster_aabb");
    static_assert(offsetof(cluster_aabb, upper) == 16, "bad layout of cluster_aabb");

    // layout std430: lights of a cluster, element of cluster_range_ssbo (GLSL: cluster_range_s). the lights are
    // light_data.lights indices stored in the cluster_index_ssbo list from offset on
    struct cluster_range
    {
        uint32_t offset;
        uint32_t count;
    };

    static_assert(sizeof(cluster_range) == 8, "bad size of cluster_range");
    static_assert(offsetof(cluster_range, offset) == 0, "bad layout of cluster_range");
    static_assert(offsetof(cluster_range, count) == 4, "bad layout of cluster_range");

    // layout std430: cluster_counter_ssbo. index_count is the allocator of the shared index list, overflow_count
    // the number of lights dropped from clusters which were full
    struct cluster_counters
    {
        uint32_t index_count;
        uint32_t overflow_count;
    };

    static_assert(sizeof(cluster_counters) == 8, "bad size of cluster_counters");
    static_assert(offsetof(cluster_counters, index_count) == 0, "bad layout of cluster_counters");
    static_assert(offsetof(cluster_counters, overflow_count) == 4, "bad layout of cluster_counters");

#pragma pack(pop)
}
