#pragma once

#include "object.h"
#include "bindable.h"
#include "buffer.h"
#include <glm/glm.hpp>
#include <cstddef>
#include <array>
#include <utility>

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
    // the same for the lamps of scenery models (a texture of its own: the maps have another size)
    const size_t LOCAL_SHADOW_LAMPS_TEX = MAX_TEXTURES + HELPER_TEXTURES + 2;
    // shadow maps of the headlights of vehicles (2d array, depth compared; opengl46_local_shadows), and how many of them there may be
    const size_t HEADLIGHT_SHADOW_TEX = MAX_TEXTURES + HELPER_TEXTURES + 3;
    const size_t MAX_HEADLIGHT_SHADOWS = 4;
    // weather (opengl46_weather): the fog of the main view (3d, apply_fog.glsl) and the shadows the clouds cast (2d), read by the
    // shading. bound directly, like the ones above
    const size_t FOG_VOLUME_TEX = MAX_TEXTURES + HELPER_TEXTURES + 4;
    const size_t CLOUD_SHADOW_TEX = MAX_TEXTURES + HELPER_TEXTURES + 5;
    // the clouds drawn into the sky, and the noise the clouds and the fog are made of; bound by the passes which read them
    const size_t CLOUD_TEX = MAX_TEXTURES + HELPER_TEXTURES + 6;
    const size_t CLOUD_SHAPE_TEX = MAX_TEXTURES + HELPER_TEXTURES + 7;
    const size_t CLOUD_DETAIL_TEX = MAX_TEXTURES + HELPER_TEXTURES + 8;
    const size_t CLOUD_COVERAGE_TEX = MAX_TEXTURES + HELPER_TEXTURES + 9;
    const size_t CLOUD_HISTORY_TEX = MAX_TEXTURES + HELPER_TEXTURES + 10;
    // the depth of the opaque scene of the main view drawn before its colour, resolved, for the contact shadows (light_common.glsl).
    // bound directly, like the ones above
    const size_t SCENE_DEPTH_TEX = MAX_TEXTURES + HELPER_TEXTURES + 11;
    // the image of the last frame of the main view with its mipmaps, which the screen space reflections take their colour from
    // (light_common.glsl, gl::scene_ubs::ssr_params). bound directly, like the ones above
    const size_t SSR_HISTORY_TEX = MAX_TEXTURES + HELPER_TEXTURES + 12;
    // (MAX_TEXTURES + HELPER_TEXTURES + 13 is WATER_MASK_TEX, below)
    // the ambient occlusion of the main view worked out from the depth in SCENE_DEPTH_TEX before the colour, half its size, which
    // the shading takes into the ambient light (gfx.postfx.ao.lighting, gl::scene_ubs::ao_params). bound directly, like the ones above
    const size_t SCENE_AO_TEX = MAX_TEXTURES + HELPER_TEXTURES + 14;
    // the probe of the occupied cab (gfx::cab_probe, gl::scene_ubs::cab_probe_box): cube maps of the light of its lamps reflected by
    // the cab and of its albedo. bound directly, like the ones above
    const size_t CAB_PROBE_TEX = MAX_TEXTURES + HELPER_TEXTURES + 15;
    const size_t CAB_PROBE_ALBEDO_TEX = MAX_TEXTURES + HELPER_TEXTURES + 16;
    // the occlusion of the sky and of the sun by the scenery around the camera (gfx::sky_occlusion, gl::scene_ubs::sky_occlusion_*):
    // the height of the top of the scenery (R32F) and the shares of the sky and the limits of the key light (RGBA16F), toroidal maps
    // read texel by texel with textureGather. bound directly, like the ones above
    const size_t SKY_OCCLUSION_HEIGHT_TEX = MAX_TEXTURES + HELPER_TEXTURES + 17;
    const size_t SKY_OCCLUSION_TEX = MAX_TEXTURES + HELPER_TEXTURES + 18;

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
        // view space of the pass to the space of the shadow map of a set of headlights (HEADLIGHTS lights with headlight_weights.w
        // above 0 read the map w - 1): x, y 0..1 across the map, z / w the depth of the map, w the distance along the axis of the
        // projector. and x: size of a texel of the map one metre from the projector, y: radius of the filter in texels
        glm::mat4 headlight_shadow[MAX_HEADLIGHT_SHADOWS];
        glm::vec4 headlight_shadow_params[MAX_HEADLIGHT_SHADOWS];
        // contact shadows, read from SCENE_DEPTH_TEX; set only for the colour pass of the main view, zero elsewhere.
        // x: length of the rays towards the lights, metres (0: off), y: distance from the camera at which they end, z: steps
        // of the ray towards the sun, w: steps of the rays towards lamps and headlights
        glm::vec4 contact_shadow_params;
        // x: how many lamps and headlights a pixel traces the rays to (0: only the sun), yzw unused
        glm::vec4 contact_shadow_lamps;
        // x: 1 when the cascades of the sun shadows are fitted to the depth of the view (gfx.shadow.sdsm, opengl46_shadow_fit):
        // a point outside the map of its cascade goes to the next cascade whose map holds it. yzw unused
        glm::vec4 shadow_fit;
        // the screen space reflections (gfx.ssr, gfx::screen_reflections), set only for the colour pass of the main view, zero
        // elsewhere: the view space of the pass to the clip space of the frame whose image the history (SSR_HISTORY_TEX) holds, for
        // the world standing still and for the cab with its vehicle, which moves with the camera
        glm::mat4 ssr_reprojection;
        glm::mat4 ssr_reprojection_cab;
        // the view space of the pass to the box around the vehicle of the cab, -1 .. 1 inside: what is in it is taken to the history
        // with ssr_reprojection_cab
        glm::mat4 ssr_cab_box;
        // x: steps of a ray (0: off), y: the longest ray, metres, z: the roughest surface they're traced for, w: the noise of the
        // frame, 0 .. 1
        glm::vec4 ssr_params;
        // x: the last level of the mipmaps of the history, y: 1 with the box of the cab, z: 1 when the ray tracing fills in what the
        // screen doesn't hold (the vulkan renderer, gfx.raytracing), w: the longest traced ray, metres
        glm::vec4 ssr_params2;
        // the reflection of the surroundings the screen space reflections and the ray tracing don't find, for every pass: x: albedo
        // of the ground below the horizon of the environment map (gfx.ambient.groundalbedo), yzw unused
        glm::vec4 env_params;
        // the water on the windscreens (mat_rain_windscreen): xyz the velocity of the air against the vehicle of the camera (the
        // share of the wind the particles of the rain drift with, less the velocity of the vehicle), world space, m/s; w unused
        glm::vec4 rain_wind;
        // the ambient occlusion in the light of the surfaces (gfx.postfx.ao.lighting), set only for the colour pass of the main view,
        // zero elsewhere: x 1 when SCENE_AO_TEX holds the occlusion of the depth in SCENE_DEPTH_TEX, y the power it's raised to
        // (gfx.postfx.ao.strength), zw unused
        glm::vec4 ao_params;
        // the probe of the occupied cab (gfx::cab_probe), set only for the colour pass of the main view, zero elsewhere: the view
        // space of the pass to the space of the probe (the axes of the cab model, metres, the origin in the middle of its box)
        glm::mat4 cab_probe_view;
        // xyz half the size of the box of the cab, w 1 when the probe is there
        glm::vec4 cab_probe_box;
        // x the share of the light of the lamps the cab reflects which lights it again (gfx.cabprobe.bounce), y 1 when the cab reflects
        // itself (gfx.cabprobe.reflections), z the last level of the mipmaps of the probe, w unused
        glm::vec4 cab_probe_params;
        // the drawing of the probe (cabprobe.glsl), set for it only: xyz the cab lamp in the space of the probe, w 1 / its range
        // squared (0: no lamp); rgb its light, a 1 with its shadow map (LOCAL_SHADOW_TEX); the headlight_weights of that map
        glm::vec4 cab_probe_lamp;
        glm::vec4 cab_probe_lamp_color;
        glm::vec4 cab_probe_lamp_shadow;
        // the occlusion of the sky and of the sun by the scenery around the camera (gfx.skyvisibility, gfx::sky_occlusion), zero when
        // there are no maps: x, y the world x, z of the camera of the pass wrapped to the size of the maps (the place of the camera on
        // them), metres; z the world height of the camera; w 1 with the maps, 2 when the volumetric fog reads them too
        glm::vec4 sky_occlusion_origin;
        // x the size of the maps, metres, y their texels across, z the distance from the middle of the maps (the larger of the ones
        // along x and z) the occlusion fades out from, w the distance it fades over
        glm::vec4 sky_occlusion_params;
        // xy the middle of the maps (the main camera they follow) less the camera of the pass, world x and z, metres; zw unused
        glm::vec4 sky_occlusion_center;
    };

    static_assert(sizeof(scene_ubs) == 912 + 80 * MAX_HEADLIGHT_SHADOWS, "bad size of ubs");

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
        // 1 for the parts of the occupied cab, which the probe of the cab lights and reflects (gl::scene_ubs::cab_probe_box)
        float cab_interior{0.f};
        UBS_PAD(8);

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

        // HEADLIGHTS: view space to the space of the beam texture, and the weights of its channels; w: 1 + the shadow map of the
        // headlights (scene_ubs::headlight_shadow), 0 when they have none.
        // SPOT and POINT (lamps): view space to the space of the cube shadow map of the lamp (origin at the lamp), and
        // x: layer of the cube map array the map is in, below 0 when the lamp has no shadows; y: near plane of the map;
        // z: size of a texel of the map one metre from the lamp; w: radius of the filter, in texels. the map is in the array of
        // the scenery lamps (LOCAL_SHADOW_LAMPS_TEX) when linear is 1, in the one of the cab lamp (LOCAL_SHADOW_TEX) otherwise
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

    // binding points of the blocks of the scene and of the model being drawn (common.glsl)
    const GLuint SCENE_UBO_BINDING = 0;
    const GLuint MODEL_UBO_BINDING = 1;

    // binding points of the cluster buffers: layout(binding = ...) in common.glsl.
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

    // layout std430: the light of the lamps the occupied cab reflects, which lights the cab again (GLSL: cab_probe_ssbo, written by
    // cabprobe_sh.comp from the probe of the cab): the coefficients of the real spherical harmonics of the light a surface facing a
    // direction of the space of the probe gets, bands 0..2, in units of light_data.ambient; rgb used, w is padding
    struct cab_probe_data
    {
        glm::vec4 bounce_sh[SKY_SH_COEFFICIENTS];
    };
    // binding point of the opengl renderer, bound with the colour passes
    const GLuint CAB_PROBE_SSBO_BINDING = 17;
    // the face of the level of the probe the spherical harmonics are taken from, at most, and the work group of cabprobe_sh.comp
    const size_t CAB_PROBE_SH_SIZE = 16;
    const size_t CAB_PROBE_SH_GROUP_SIZE = 64;

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

    // weather of the opengl 4.6 renderer (opengl46_weather, weather_common.glsl): fog, clouds, their shadows
    const GLuint WEATHER_UBO_BINDING = 5;
    // layers of clouds drawn, see weather_conditions::max_cloud_layers
    const size_t MAX_CLOUD_LAYERS = 4;
    // colours of the fog towards the horizon, by the angle from the azimuth of the sun (0-180 degrees)
    const size_t FOG_HORIZON_SAMPLES = 32;
    // work groups of the weather passes: 8x8 for the images, 8x8x1 for the fog volume
    const GLuint WEATHER_GROUP_SIZE = 8;
    // the slices of the fog volume are spaced exponentially in the view depth plus this many metres: fewer of them go to the first
    // metres in front of the camera, more to the distance, where thick slices draw the shafts of light in steps
    const float FOG_VOLUME_BIAS = 10.0f;

    // layout std140: one layer of clouds (GLSL: cloud_layer_s), see weather_common.glsl
    struct cloud_layer_ubs
    {
        glm::vec4 heights;
        glm::vec4 shape;
        glm::vec4 profile;
        glm::vec4 motion;
        glm::vec4 extra;
    };

    static_assert(sizeof(cloud_layer_ubs) == 80, "bad size of cloud_layer_ubs");

    // layout std140: the weather for the shading and the weather passes (GLSL: weather_ubo), see weather_common.glsl
    struct weather_ubs
    {
        glm::vec4 fog_haze;
        glm::vec4 fog_ground;
        glm::vec4 fog_noise;
        glm::vec4 fog_light;
        glm::vec4 fog_volume_params;
        glm::vec4 fog_volume_target;
        glm::vec4 weather_camera;
        glm::vec4 weather_sun_direction;
        glm::vec4 weather_sun_color;
        glm::vec4 weather_ambient;
        glm::vec4 cloud_shadow_params;
        glm::vec4 cloud_wind;
        cloud_layer_ubs cloud_layers[MAX_CLOUD_LAYERS];
        // filled on the gpu (fog_horizon.comp) while the atmosphere is used, by the cpu otherwise
        glm::vec4 fog_horizon[FOG_HORIZON_SAMPLES];
    };

    static_assert(offsetof(weather_ubs, cloud_layers) == 192, "bad layout of weather_ubs");
    static_assert(offsetof(weather_ubs, fog_horizon) == 192 + 80 * MAX_CLOUD_LAYERS, "bad layout of weather_ubs");
    static_assert(sizeof(weather_ubs) == 192 + 80 * MAX_CLOUD_LAYERS + 16 * FOG_HORIZON_SAMPLES, "bad size of weather_ubs");

    // the weather buffer is written by fog_horizon.comp as a storage block bound here for the pass. the point is one of the
    // instance culling, which binds its buffers before every use
    const GLuint WEATHER_SCRATCH_SSBO_BINDING = 9;

    // culling of model instances in compute passes (instance_cull.comp, instance_cull_offsets.comp). binding points of
    // the shader storage blocks, apart from the ones bound for the shading so the passes don't disturb them
    const GLuint INSTANCE_RECORD_SSBO_BINDING = 9; // scene::basic_cell::instance_record of the queued buckets
    const GLuint INSTANCE_JOB_SSBO_BINDING = 10; // instance_cull_job, one per queued bucket (or track with sleepers)
    const GLuint INSTANCE_GROUP_SSBO_BINDING = 11; // instance_cull_group, one per model and skins
    const GLuint INSTANCE_LOD_SSBO_BINDING = 12; // sorted lod bounds (squared distances) of the groups
    const GLuint INSTANCE_COUNTER_SSBO_BINDING = 13; // header, then count[entries], cursor[entries] and first[entries]
    const GLuint INSTANCE_OUTPUT_SSBO_BINDING = 14; // modelview matrices of the instances which passed, by entry
    const GLuint INSTANCE_COMMAND_SSBO_BINDING = 15; // indirect draw commands of the entries (instance_cull_commands.comp)
    // the model blocks of the draws of a multi draw of the instances drawn from the gpu culling, one gl::model_ubs per draw
    // (std430, the same layout), read by gl_DrawID in programs built with MODEL_DRAWS. the point is the one of the jobs of the
    // instance culling, which binds its buffers before every use; the draws bind theirs before every multi draw
    const GLuint MODEL_DRAW_SSBO_BINDING = 10;
    const GLuint INSTANCE_CULL_GROUP_SIZE = 64;
    const GLuint INSTANCE_OFFSETS_GROUP_SIZE = 1024;
    const GLuint INSTANCE_COMMAND_GROUP_SIZE = 64;
    // uints of the header of the counter buffer: frame number, survivors of the tests but the occlusion one in total,
    // entries, instances left out as hidden behind the depth already drawn (occlusion test), instances drawn first in two
    // phases (seen in the previous frame), unused.
    // an entry is a lod band of a group: survivors of group g in band b are counted at entry entryfirst(g) + b
    const GLuint INSTANCE_COUNTER_HEADER = 8;
    // work group of the passes which make the depth pyramid of the occlusion test (hiz_build.comp), 8x8
    const GLuint HIZ_GROUP_SIZE = 8;

    // the cascades of the sun shadows fitted to the depth of the main view (opengl46_shadow_fit, shadow_fit_*.comp): work group
    // of its passes, 16x16 pixels, and the binding point of its buffer. the point is one of the instance culling, which binds
    // its buffers before every use; the fit binds its own before its passes
    const GLuint SHADOW_FIT_GROUP_SIZE = 16;
    const GLuint SHADOW_FIT_SSBO_BINDING = 9;

    // the particles of the gpu (opengl46_gpu_particles, particles_*.comp, particle_common.glsl): work group of their passes and the
    // binding points of their buffers. the points are ones of the instance culling, which binds its buffers before every use; the
    // particles bind their own before their passes. the draws read the particles as attributes of instances, not from storage
    const GLuint PARTICLE_GROUP_SIZE = 64;
    const GLuint PARTICLE_SSBO_BINDING = 9; // the smoke particles
    const GLuint PARTICLE_SOURCE_SSBO_BINDING = 10; // their sources, as the last update of the simulation left them
    const GLuint PARTICLE_OWNER_SSBO_BINDING = 11; // the source of every place of the particles
    const GLuint PARTICLE_OUTPUT_SSBO_BINDING = 12; // what the draw reads, a particle_draw per particle drawn
    const GLuint PARTICLE_COMMAND_SSBO_BINDING = 13; // the indirect draw command, whose instances the passes count

    // layout std430: GLSL smoke_particle_s
    struct particle_smoke
    {
        glm::vec4 position;
        glm::vec4 velocity;
        glm::vec4 state;
    };
    static_assert(sizeof(particle_smoke) == 48, "bad size of particle_smoke");

    // layout std430: GLSL smoke_source_s
    struct particle_smoke_source
    {
        glm::uvec4 pool;
        glm::vec4 location;
        glm::vec4 previous;
        glm::vec4 launch;
        glm::vec4 initial;
        glm::vec4 sizechange;
        glm::vec4 opacitychange;
        glm::vec4 motion;
        glm::vec4 settling;
        glm::vec4 color;
        glm::vec4 oldcolor;
    };
    static_assert(sizeof(particle_smoke_source) == 176, "bad size of particle_smoke_source");

    // layout std430: GLSL particle_draw_s, read by the draws as attributes of instances
    struct particle_draw
    {
        glm::vec4 place;
        glm::vec4 shape;
        glm::vec4 color;
    };
    static_assert(sizeof(particle_draw) == 48, "bad size of particle_draw");

    // the water on the windscreens (gfx::windscreen_water, water_*.comp, water.glsl, mat_rain_windscreen.frag): the glasses of the
    // cab of the train simulated at once (each a material of mat_rain_windscreen), the drops a glass holds, the cells of the grid
    // the drops are looked up by along each coordinate of the texture of the glass (uv) and the drops a cell lists, the texels of
    // the map of the wetted paths and of the map of the glass (its axes, the blades of the wipers) along each coordinate, the work
    // group of the passes
    const GLuint WATER_SLOTS = 4;
    const GLuint WATER_DROPS = 65536;
    const GLuint WATER_GRID = 128;
    const GLuint WATER_CELL_DROPS = 32;
    const GLuint WATER_TRAIL = 1024;
    const GLuint WATER_MAP = 256;
    const GLuint WATER_GROUP_SIZE = 64;
    // the binding points of the opengl renderer: the storage of the water (read by the glasses, written by the passes), the block
    // of the step of the simulation (water_ubs) and the texture unit of the mask of the wipers the map is baked from. the storage
    // point is used by nothing else, bound with the colour passes
    const GLuint WATER_SSBO_BINDING = 16;
    const GLuint WATER_UBO_BINDING = 6;
    const size_t WATER_MASK_TEX = MAX_TEXTURES + HELPER_TEXTURES + 13;
    // the parts of the storage of the water (std430, GLSL water_ssbo), offsets in bytes: the header of every glass (vec4), the
    // counts of every glass (uvec4), the drops (two vec4 each), the axes of the map (two vec4 a texel), the blades of the map (vec4
    // a texel), the state of the drops (uint), the drops they flow into (uint), what they gather (four int), what they keep
    // (vec4), then the cells of the grids (a count and the drops of each cell) and the wetted paths (uint)
    const size_t WATER_SLOT_DROPS = static_cast<size_t>(WATER_SLOTS) * WATER_DROPS;
    const size_t WATER_SLOT_TEXELS = static_cast<size_t>(WATER_SLOTS) * WATER_MAP * WATER_MAP;
    const size_t WATER_HEADER_OFFSET = 0;
    const size_t WATER_COUNTS_OFFSET = WATER_HEADER_OFFSET + 16 * WATER_SLOTS;
    const size_t WATER_DROPS_OFFSET = WATER_COUNTS_OFFSET + 16 * WATER_SLOTS;
    const size_t WATER_AXES_OFFSET = WATER_DROPS_OFFSET + 32 * WATER_SLOT_DROPS;
    const size_t WATER_BLADES_OFFSET = WATER_AXES_OFFSET + 32 * WATER_SLOT_TEXELS;
    const size_t WATER_STATE_OFFSET = WATER_BLADES_OFFSET + 16 * WATER_SLOT_TEXELS;
    const size_t WATER_TARGET_OFFSET = WATER_STATE_OFFSET + 4 * WATER_SLOT_DROPS;
    const size_t WATER_GATHER_OFFSET = WATER_TARGET_OFFSET + 4 * WATER_SLOT_DROPS;
    const size_t WATER_EXTRA_OFFSET = WATER_GATHER_OFFSET + 16 * WATER_SLOT_DROPS;
    const size_t WATER_CELLS_OFFSET = WATER_EXTRA_OFFSET + 16 * WATER_SLOT_DROPS;
    const size_t WATER_CELLS_SIZE = 4 * static_cast<size_t>(WATER_SLOTS) * WATER_GRID * WATER_GRID * (1 + WATER_CELL_DROPS);
    const size_t WATER_TRAIL_OFFSET = WATER_CELLS_OFFSET + WATER_CELLS_SIZE;
    const size_t WATER_STORAGE_SIZE = WATER_TRAIL_OFFSET + 4 * static_cast<size_t>(WATER_SLOTS) * WATER_TRAIL * WATER_TRAIL;
    // the parts of a glass in the storage: its drops and what the passes keep of them, its map (the axes the second part), its
    // cells and its paths, its header and its counts, (offset, size) in bytes
    inline std::array<std::pair<size_t, size_t>, 11> water_slot_parts(size_t const Slot)
    {
        auto const drops{static_cast<size_t>(WATER_DROPS)};
        auto const texels{static_cast<size_t>(WATER_MAP) * WATER_MAP};
        auto const cells{4 * static_cast<size_t>(WATER_GRID) * WATER_GRID * (1 + WATER_CELL_DROPS)};
        auto const trail{4 * static_cast<size_t>(WATER_TRAIL) * WATER_TRAIL};
        return {{{WATER_DROPS_OFFSET + 32 * drops * Slot, 32 * drops},
                 {WATER_AXES_OFFSET + 32 * texels * Slot, 32 * texels},
                 {WATER_BLADES_OFFSET + 16 * texels * Slot, 16 * texels},
                 {WATER_STATE_OFFSET + 4 * drops * Slot, 4 * drops},
                 {WATER_TARGET_OFFSET + 4 * drops * Slot, 4 * drops},
                 {WATER_GATHER_OFFSET + 16 * drops * Slot, 16 * drops},
                 {WATER_EXTRA_OFFSET + 16 * drops * Slot, 16 * drops},
                 {WATER_CELLS_OFFSET + cells * Slot, cells},
                 {WATER_TRAIL_OFFSET + trail * Slot, trail},
                 {WATER_HEADER_OFFSET + 16 * Slot, 16},
                 {WATER_COUNTS_OFFSET + 16 * Slot, 16}}};
    }

    // layout std140: the step of the simulation of the water on the windscreens (GLSL water_ubo), every vector in the space of the
    // model of the cab
    struct water_ubs
    {
        // x: seconds of the step, y: seconds the simulation runs (the wetted paths are marked with it), z: how heavy the rain or
        // the snow is, 0 .. 1, w: 1 for snow
        glm::vec4 step;
        // xyz: up, w: the temperature of the air, degrees Celsius
        glm::vec4 up;
        // xyz: the air against the cab (the share of the wind the rain drifts with, less the velocity of the vehicle), m/s; w: how
        // fast the rain or the snow falls, m/s
        glm::vec4 air;
        // the blades of the wipers (the channels of their mask) now and at the step before: 0 at rest .. 1 at the far end of
        // their sweep, below 0 where there's none
        glm::vec4 blades;
        glm::vec4 blades_before;
        // the drops of the rain: x the smallest diameter which lands as a drop of its own, mm (the smaller ones are the mist of the
        // glass), y: 1 / the mean of the diameters above it, 1 / mm (they go as Marshall-Palmer), zw: unused
        glm::vec4 rain;
        // x: random seed of the step, y: the most landings tried on a glass in the step, z: the glass the passes for a single glass
        // work on (the baking of the mask of the wipers)
        glm::uvec4 counts;
        // every glass: x the landings tried in the step, y 1 / the most metres squared of the glass a unit of its uv holds (the
        // landings tried at the densest place are all taken), z 1 when it's simulated, w the size of its drops (the param
        // rain_scale of its material)
        glm::vec4 slots[WATER_SLOTS];
    };
    static_assert(sizeof(water_ubs) == 112 + 16 * WATER_SLOTS, "bad size of water_ubs");

    // layout std430: a bucket of a scene cell queued in the pass (GLSL: instance_cull_job)
    struct instance_cull_job
    {
        packed_vec3 origin; // origin of the records of the bucket, relative to the camera of the pass
        uint32_t first; // first record of the bucket in the record buffer
        packed_vec3 distanceorigin; // the same origin, relative to the camera the distance is measured from
        uint32_t count; // records of the bucket
        uint32_t group;
        uint32_t candidatestart; // records of the jobs before this one
        uint32_t flags; // instance_cull_job_flags
        uint32_t pad;
    };

    enum instance_cull_job_flags : uint32_t
    {
        // the sleepers of a track (opengl46_renderer::Render_Sleepers()): the lod goes by the plain distance from the camera of
        // the pass, without the range and size tests of the model instances (the range of the whole track is tested on the cpu)
        INSTANCE_JOB_SLEEPERS = 1u
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

    // layout std430: an indirect draw of the submodel of a group in one of its lod bands. the first five fields are the
    // command of glDrawElementsIndirect (count, instance count, first index, base vertex, base instance), or with indexed 0
    // the one of glDrawArraysIndirect (count, instance count, first vertex, base instance). the cpu fills in what it knows
    // of the geometry, instance_cull_commands.comp the instances, from the counts of the entry. the command buffer holds the
    // commands of a pass three times over (instances drawn first, all, drawn second), see opengl46_instance_cull
    struct instance_draw_command
    {
        uint32_t count;
        uint32_t instancecount;
        uint32_t first;
        uint32_t basevertex; // the base instance of an array draw
        uint32_t baseinstance;
        uint32_t entry;
        uint32_t indexed;
        uint32_t pad;
    };

    static_assert(sizeof(instance_draw_command) == 32, "bad size of instance_draw_command");

    // layout std430: what the passes of the cascades fitted to the depth of the view leave for the cpu (GLSL: shadow_fit_ssbo).
    // the cpu writes it before the passes: the number of the measurement, no pixels, the nearest distance infinite, the bounds
    // empty. the passes take the extremes with atomic operations on the bits: positive floats order as uints, and the bounds,
    // which can be negative, are stored in an order preserving form (shadow_fit_order() in shadow_fit_bounds.comp)
    struct shadow_fit_data
    {
        uint32_t measurement; // number given by the cpu, to find the camera and the light it was made with
        uint32_t samples; // pixels of the view which aren't the sky
        float depthmin; // nearest and farthest distance of those from the camera, metres
        float depthmax;
        float splits[4]; // distances the cascades go from and to (MAX_CASCADES + 1 of them), from the two above
        uint32_t counts[4]; // pixels taken into the bounds of each cascade (MAX_CASCADES of them)
        // per cascade the lowest x, y, z, unused, then the highest x, y, z, unused, of the pixels it takes, in metres in the
        // space of the light (its rotation only), relative to the camera
        uint32_t bounds[MAX_CASCADES * 8];
    };

    static_assert(MAX_CASCADES + 1 <= 4, "shadow_fit_data holds the splits of at most 3 cascades");
    static_assert(offsetof(shadow_fit_data, splits) == 16, "bad layout of shadow_fit_data");
    static_assert(offsetof(shadow_fit_data, counts) == 32, "bad layout of shadow_fit_data");
    static_assert(offsetof(shadow_fit_data, bounds) == 48, "bad layout of shadow_fit_data");
    static_assert(sizeof(shadow_fit_data) == 48 + 32 * MAX_CASCADES, "bad size of shadow_fit_data");

#pragma pack(pop)
}
