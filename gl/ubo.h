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
    // environment map prefiltered with the GGX lobe, see envmapping.glsl
    const size_t ENV_GGX_TEX = MAX_TEXTURES + 3;
    // the prefiltered map: size of its first level and the number of levels. level k holds the reflection of a surface of
    // perceptual roughness ENVMAP_GGX_ROUGHNESS_MIN + (1 - ENVMAP_GGX_ROUGHNESS_MIN) * k / (ENVMAP_GGX_LEVELS - 1); smoother
    // surfaces read the mipmaps of the plain map, which differ little from the lobe there
    const size_t ENVMAP_GGX_SIZE = 256;
    const size_t ENVMAP_GGX_LEVELS = 7;
    const float ENVMAP_GGX_ROUGHNESS_MIN = 0.2f;
    const size_t ENVMAP_GGX_GROUP_SIZE = 8;
    // sky-view table of the atmosphere (sky_atmosphere.frag). past the units the texture cache keeps track of, bound directly
    const size_t SKY_VIEW_TEX = MAX_TEXTURES + HELPER_TEXTURES + 0;
    // cube shadow maps of the lamps (cube map array, depth compared; opengl46_local_shadows). bound directly, like the one above
    const size_t LOCAL_SHADOW_TEX = MAX_TEXTURES + HELPER_TEXTURES + 1;

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

        // HEADLIGHTS: view space to the space of the beam texture, and the weights of its channels.
        // SPOT and POINT (lamps): view space to the space of the cube shadow map of the lamp (origin at the lamp), and
        // x: layer of the cube map array the map is in, below 0 when the lamp has no shadows; y: near plane of the map;
        // z: size of a texel of the map one metre from the lamp; w: radius of the filter, in texels
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

    // capacity of light_data.lights; slot 0 is the sun. a buffer size, not a limit of the shaders: they take the
    // number of lights from the header
    const size_t MAX_LIGHTS = 1024;

    // contents of light_ssbo: global lighting header followed by the light array
    struct light_data
    {
        packed_vec3 ambient;
        // number of lights stored in lights[] (the sun included): what the light clusters are culled from
        uint32_t lights_total;

        packed_vec3 fog_color;
        // number of lights (the sun included) the shading loop over lights[] goes through when the lights are not clustered
        uint32_t lights_count;
        // 1 if the shading takes the lights of a fragment from the list of its cluster (lights_count is not used then), 0 if not.
        // set per pass: the cluster grid exists only for the color pass of a perspective view
        uint32_t lights_clustered;
        // scale of the directional part of the sky ambient light (sky_ssbo): the factor the ambient light of the object
        // is scaled with (tunnels), 0 where the ambient light is not the sky's (model previews)
        float sky_ambient_factor;
        // 1 if the shading reads rough reflections from the prefiltered environment map, 0 if from the mipmaps of the plain one
        uint32_t env_ggx;
        // lights[] begins at a multiple of 16 bytes
        uint32_t padding;

        light_element lights[MAX_LIGHTS];
    };

    static_assert(offsetof(light_data, ambient) == 0, "bad layout of light_data");
    static_assert(offsetof(light_data, lights_total) == 12, "bad layout of light_data");
    static_assert(offsetof(light_data, fog_color) == 16, "bad layout of light_data");
    static_assert(offsetof(light_data, lights_count) == 28, "bad layout of light_data");
    static_assert(offsetof(light_data, lights_clustered) == 32, "bad layout of light_data");
    static_assert(offsetof(light_data, sky_ambient_factor) == 36, "bad layout of light_data");
    static_assert(offsetof(light_data, env_ggx) == 40, "bad layout of light_data");
    static_assert(offsetof(light_data, lights) == 48, "bad layout of light_data");
    static_assert(sizeof(light_data) == 48 + sizeof(light_element) * MAX_LIGHTS, "bad size of light_data");

    // light clusters (see gl/cluster_grid.h): the view frustum is cut into CLUSTER_GRID_X x CLUSTER_GRID_Y screen
    // tiles and CLUSTER_GRID_Z depth slices, spaced exponentially between the near and far plane of the pass.
    // cluster index = x + CLUSTER_GRID_X * (y + CLUSTER_GRID_Y * z), with x and y counted from the lower left corner
    const GLuint CLUSTER_GRID_X = 16;
    const GLuint CLUSTER_GRID_Y = 9;
    const GLuint CLUSTER_GRID_Z = 24;
    const GLuint CLUSTER_COUNT = CLUSTER_GRID_X * CLUSTER_GRID_Y * CLUSTER_GRID_Z;
    // capacity of the light list of a single cluster
    const GLuint MAX_LIGHTS_PER_CLUSTER = 64;
    // work group size of cluster_build.comp and cluster_cull.comp, which handle one cluster per invocation
    const GLuint CLUSTER_BUILD_GROUP_SIZE = 64;
    const GLuint CLUSTER_CULL_GROUP_SIZE = 64;

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

    // layout std430: cluster_counter_ssbo, zeroed before every culling pass. index_count is the allocator of the shared
    // index list, overflow_count the number of (cluster, light) pairs dropped because the cluster already held
    // MAX_LIGHTS_PER_CLUSTER lights
    struct cluster_counters
    {
        uint32_t index_count;
        uint32_t overflow_count;
    };

    static_assert(sizeof(cluster_counters) == 8, "bad size of cluster_counters");
    static_assert(offsetof(cluster_counters, index_count) == 0, "bad layout of cluster_counters");
    static_assert(offsetof(cluster_counters, overflow_count) == 4, "bad layout of cluster_counters");

    // binding points of the sky, exposure and atmosphere buffers: layout(binding = ...) in glsl_common
    const GLuint SKY_SSBO_BINDING = 5;
    const GLuint EXPOSURE_SSBO_BINDING = 6;
    const GLuint ATMOSPHERE_RESULT_SSBO_BINDING = 7;
    const GLuint ATMOSPHERE_UBO_BINDING = 4;

    // binding point of the per-instance modelview matrices of instanced draws (glsl: instance_ssbo, mat4 per instance).
    // element 0 is identity, the matrix of every draw which isn't instanced
    const GLuint INSTANCE_SSBO_BINDING = 8;

    // number of coefficients of the spherical harmonics of order 2 (bands 0..2)
    const size_t SKY_SH_COEFFICIENTS = 9;

    // layout std430: directional part of the sky ambient light (GLSL: sky_ssbo). the coefficients of the real spherical
    // harmonics of the light a surface facing a world space direction gets, in units of light_data.ambient, without the
    // constant band (that one is light_data.ambient itself); rgb used, w is padding
    struct sky_data
    {
        glm::vec4 ambient_sh[SKY_SH_COEFFICIENTS];
    };

    static_assert(sizeof(sky_data) == 144, "bad size of sky_data");

    // layout std430: luminance histogram of the image and the exposure worked out from it (GLSL: exposure_ssbo)
    const GLuint EXPOSURE_HISTOGRAM_BINS = 256;
    const GLuint EXPOSURE_HISTOGRAM_GROUP_SIZE = 16;

    struct exposure_data
    {
        uint32_t histogram[EXPOSURE_HISTOGRAM_BINS];
        // luminance the exposure is adapted to, the time-smoothed average of the image
        float adapted_luminance;
        // multiplier the tonemapping applies to the image
        float exposure;
        // average luminance of the last image
        float average_luminance;
        // 0 until the first average was taken, the adaptation starts from it
        uint32_t valid;
    };

    static_assert(sizeof(exposure_data) == 4 * EXPOSURE_HISTOGRAM_BINS + 16, "bad size of exposure_data");
    static_assert(offsetof(exposure_data, adapted_luminance) == 4 * EXPOSURE_HISTOGRAM_BINS, "bad layout of exposure_data");

    // sizes of the look-up tables of the atmosphere (Hillaire 2020)
    const GLuint ATMOSPHERE_TRANSMITTANCE_WIDTH = 256;
    const GLuint ATMOSPHERE_TRANSMITTANCE_HEIGHT = 64;
    const GLuint ATMOSPHERE_MULTISCATTERING_SIZE = 32;
    const GLuint ATMOSPHERE_SKYVIEW_WIDTH = 192;
    const GLuint ATMOSPHERE_SKYVIEW_HEIGHT = 108;
    // work group of atmosphere_ambient.comp, which projects the sky on the spherical harmonics in one group
    const GLuint ATMOSPHERE_AMBIENT_GROUP_SIZE = 256;
    // binding points of the look-up tables in the compute passes (images and samplers)
    const GLuint ATMOSPHERE_TRANSMITTANCE_UNIT = 0;
    const GLuint ATMOSPHERE_MULTISCATTERING_UNIT = 1;
    const GLuint ATMOSPHERE_SKYVIEW_UNIT = 2;

    // layout std140: parameters of the atmosphere (GLSL: atmosphere_ubo). distances in kilometres, coefficients per kilometre
    struct atmosphere_ubs
    {
        // rgb: rayleigh scattering, w: rayleigh scale height
        glm::vec4 rayleigh;
        // x: mie scattering, y: mie extinction, z: mie scale height, w: mie phase asymmetry
        glm::vec4 mie;
        // rgb: ozone absorption, w: altitude of the middle of the ozone layer
        glm::vec4 ozone;
        // rgb: albedo of the ground, w: half width of the ozone layer
        glm::vec4 ground;
        // world space direction towards the sun (y up), w unused
        glm::vec4 sun_direction;
        float bottom_radius;
        float top_radius;
        // distance of the observer from the centre of the planet
        float observer_radius;
        float padding;
    };

    static_assert(sizeof(atmosphere_ubs) == 96, "bad size of atmosphere_ubs");

    // layout std430: what atmosphere_ambient.comp works out for the cpu (GLSL: atmosphere_result_ssbo). radiance with
    // the sun of illuminance 1
    struct atmosphere_result
    {
        // radiance arriving at the observer from every direction, ground included, as real spherical harmonics (world, y up)
        glm::vec4 radiance_sh[SKY_SH_COEFFICIENTS];
        // transmittance of the atmosphere towards the sun at the observer
        glm::vec4 sun_transmittance;
        // average radiance of the sky just above the horizon
        glm::vec4 horizon;
        // average radiance of the upper hemisphere
        glm::vec4 average;
        // transmittance of the atmosphere straight up from the observer: the colour of the sun high in the sky
        glm::vec4 zenith_transmittance;
    };

    static_assert(sizeof(atmosphere_result) == 16 * (SKY_SH_COEFFICIENTS + 4), "bad size of atmosphere_result");

    // culling of model instances in compute passes (instance_cull.comp, instance_cull_offsets.comp). binding points of
    // the shader storage blocks, apart from the ones bound for the shading so the passes don't disturb them
    const GLuint INSTANCE_RECORD_SSBO_BINDING = 9; // scene::basic_cell::instance_record of the queued buckets
    const GLuint INSTANCE_JOB_SSBO_BINDING = 10; // instance_cull_job, one per queued bucket
    const GLuint INSTANCE_GROUP_SSBO_BINDING = 11; // instance_cull_group, one per model and skins
    const GLuint INSTANCE_LOD_SSBO_BINDING = 12; // sorted lod bounds (squared distances) of the groups
    const GLuint INSTANCE_COUNTER_SSBO_BINDING = 13; // header, then count[entries] and cursor[entries]
    const GLuint INSTANCE_OUTPUT_SSBO_BINDING = 14; // modelview matrices of the instances which passed, by entry
    const GLuint INSTANCE_CULL_GROUP_SIZE = 64;
    const GLuint INSTANCE_OFFSETS_GROUP_SIZE = 1024;
    // uints of the header of the counter buffer: frame number, survivors in total, entries, unused.
    // an entry is a lod band of a group: survivors of group g in band b are counted at entry entryfirst(g) + b
    const GLuint INSTANCE_COUNTER_HEADER = 4;

    // layout std430: a bucket of a scene cell queued in the pass (GLSL: instance_cull_job)
    struct instance_cull_job
    {
        packed_vec3 origin; // origin of the records of the bucket, relative to the camera of the pass
        uint32_t first; // first record of the bucket in the record buffer
        packed_vec3 distanceorigin; // the same origin, relative to the camera the distance is measured from
        uint32_t count; // records of the bucket
        uint32_t group;
        uint32_t candidatestart; // records of the jobs before this one
        uint32_t pad[2];
    };

    static_assert(sizeof(instance_cull_job) == 48, "bad size of instance_cull_job");

    // layout std430: instances of one model with one set of skins, the unit drawn together (GLSL: instance_cull_group)
    struct instance_cull_group
    {
        uint32_t lodfirst; // first bound in the lod buffer
        uint32_t lodcount; // bounds; the group has lodcount + 1 bands
        uint32_t entryfirst; // first entry of the counters
        uint32_t pad;
    };

    static_assert(sizeof(instance_cull_group) == 16, "bad size of instance_cull_group");

#pragma pack(pop)
}
