// what the shaders of the opengl 4.6 renderer share: the settings, the constants of the game, the lights, the clusters, the
// blocks of the scene and of the model being drawn. the constants below are the values of the game (gl/ubo.h and others):
// compiled from the source, the game puts its own values before the source (gl::glsl_common) and they are taken instead;
// the SPIR-V binaries (shaders/ogl46bin) are compiled with the values written here, so they have to stay the same as in the
// game, which checks them when it starts (and doesn't load the binaries when they differ).
//
// the locations of the inputs and outputs between the stages, the same for a name in every shader (SPIR-V links the stages by
// the locations, not by the names): 0 f_normal, 1 f_normal_raw, 2 f_coord, 3 f_pos, 4 f_clip_pos, 5 f_clip_future_pos,
// 6 TangentFragPos, 7 UserData, 8-10 f_tbn, 11 f_model_draw, 12 f_albedo, 13 f_color, 14 f_coords, 15 f_local, 16 f_soft,
// 17 f_streak, 18 g_coord, 19 g_coords, 20 g_pos, 21 g_normal
#ifndef COMMON_GLSL
#define COMMON_GLSL

#if !defined(STAGE_VERTEX) && !defined(STAGE_FRAGMENT) && !defined(STAGE_GEOMETRY) && !defined(STAGE_COMPUTE)
#error "the stage of the shader isn't defined: #define STAGE_VERTEX 1 (STAGE_FRAGMENT, STAGE_GEOMETRY, STAGE_COMPUTE) before the include of common.glsl"
#endif

#include "settings.glsl"

#ifndef MASZYNA_CONSTANTS
#define MASZYNA_CONSTANTS 1
#define MAX_LIGHTS 1024U
#define MAX_CASCADES 3U
#define MAX_PARAMS 3U
#define SHADOW_TEX 8
#define ENV_TEX 9
#define HEADLIGHT_TEX 10
#define ENV_GGX_TEX 11
#define SCENE_UBO_BINDING 0
#define MODEL_UBO_BINDING 1
#define INSTANCE_SSBO_BINDING 8
#define INSTANCE_RECORD_SSBO_BINDING 9
#define INSTANCE_JOB_SSBO_BINDING 10
#define INSTANCE_GROUP_SSBO_BINDING 11
#define INSTANCE_LOD_SSBO_BINDING 12
#define INSTANCE_COUNTER_SSBO_BINDING 13
#define INSTANCE_OUTPUT_SSBO_BINDING 14
#define INSTANCE_COMMAND_SSBO_BINDING 15
#define MODEL_DRAW_SSBO_BINDING 10
#define INSTANCE_CULL_GROUP_SIZE 64
#define INSTANCE_OFFSETS_GROUP_SIZE 1024
#define INSTANCE_COMMAND_GROUP_SIZE 64
#define HIZ_GROUP_SIZE 8
#define SHADOW_FIT_GROUP_SIZE 16
#define SHADOW_FIT_SSBO_BINDING 9
#define PARTICLE_GROUP_SIZE 64
#define PARTICLE_SSBO_BINDING 9
#define PARTICLE_SOURCE_SSBO_BINDING 10
#define PARTICLE_OWNER_SSBO_BINDING 11
#define PARTICLE_OUTPUT_SSBO_BINDING 12
#define PARTICLE_COMMAND_SSBO_BINDING 13
#define WATER_SLOTS 4
#define WATER_DROPS 65536
#define WATER_GRID 128
#define WATER_CELL_DROPS 32
#define WATER_TRAIL 1024
#define WATER_MAP 256
#define WATER_GROUP_SIZE 64
#define WATER_SSBO_BINDING 16
#define WATER_UBO_BINDING 6
#define WATER_MASK_TEX 25
#define INSTANCE_COUNTER_HEADER 8
#define INSTANCE_JOB_SLEEPERS 1U
#define LIGHT_SSBO_BINDING 0
#define CLUSTER_GRID_X 16
#define CLUSTER_GRID_Y 9
#define CLUSTER_GRID_Z 24
#define CLUSTER_COUNT 3456
#define MAX_LIGHTS_PER_CLUSTER 64
#define CLUSTER_BUILD_GROUP_SIZE 64
#define CLUSTER_CULL_GROUP_SIZE 64
#define CLUSTER_UBO_BINDING 2
#define CLUSTER_AABB_SSBO_BINDING 1
#define CLUSTER_RANGE_SSBO_BINDING 2
#define CLUSTER_INDEX_SSBO_BINDING 3
#define CLUSTER_COUNTER_SSBO_BINDING 4
#define ENVMAP_GGX_LEVELS 7
#define ENVMAP_GGX_ROUGHNESS_MIN 0.200000
#define ENVMAP_GGX_GROUP_SIZE 8
#define SKY_SSBO_BINDING 5
#define EXPOSURE_SSBO_BINDING 6
#define EXPOSURE_HISTOGRAM_BINS 256
#define EXPOSURE_HISTOGRAM_GROUP_SIZE 16
#define ATMOSPHERE_UBO_BINDING 4
#define ATMOSPHERE_RESULT_SSBO_BINDING 7
#define ATMOSPHERE_TRANSMITTANCE_WIDTH 256
#define ATMOSPHERE_TRANSMITTANCE_HEIGHT 64
#define ATMOSPHERE_MULTISCATTERING_SIZE 32
#define ATMOSPHERE_SKYVIEW_WIDTH 192
#define ATMOSPHERE_SKYVIEW_HEIGHT 108
#define ATMOSPHERE_AMBIENT_GROUP_SIZE 256
#define ATMOSPHERE_TRANSMITTANCE_UNIT 0
#define ATMOSPHERE_MULTISCATTERING_UNIT 1
#define ATMOSPHERE_SKYVIEW_UNIT 2
#define SKY_VIEW_TEX 12
#define LOCAL_SHADOW_TEX 13
#define LOCAL_SHADOW_LAMPS_TEX 14
#define HEADLIGHT_SHADOW_TEX 15
#define MAX_HEADLIGHT_SHADOWS 4U
#define FOG_VOLUME_TEX 16
#define CLOUD_SHADOW_TEX 17
#define CLOUD_TEX 18
#define CLOUD_SHAPE_TEX 19
#define CLOUD_DETAIL_TEX 20
#define CLOUD_COVERAGE_TEX 21
#define CLOUD_HISTORY_TEX 22
#define SCENE_DEPTH_TEX 23
#define SSR_HISTORY_TEX 24
#define SCENE_AO_TEX 26
#define CAB_PROBE_TEX 27
#define CAB_PROBE_ALBEDO_TEX 28
#define CAB_PROBE_SSBO_BINDING 17
#define CAB_PROBE_SH_SIZE 16
#define CAB_PROBE_SH_GROUP_SIZE 64
#define WEATHER_UBO_BINDING 5
#define WEATHER_SCRATCH_SSBO_BINDING 9
#define MAX_CLOUD_LAYERS 4
#define FOG_HORIZON_SAMPLES 32
#define WEATHER_GROUP_SIZE 8
#define FOG_VOLUME_BIAS 10.000000
#endif

const uint LIGHT_SPOT = 0U;
const uint LIGHT_POINT = 1U;
const uint LIGHT_DIR = 2U;
const uint LIGHT_HEADLIGHTS = 3U;

struct light_s
{
        vec3 pos;
        uint type;

        vec3 dir;
        float in_cutoff;

        vec3 color;
        float out_cutoff;

        float linear;
        float quadratic;

        float intensity;
        float ambient;

        mat4 headlight_projection;
        vec4 headlight_weights;
};

// storage blocks are guaranteed only in fragment and compute shaders (GL 4.6 minimum for the other stages is 0)
#if defined(STAGE_FRAGMENT) || defined(STAGE_COMPUTE)
layout(std430, binding = LIGHT_SSBO_BINDING) readonly buffer light_ssbo
{
        vec3 ambient;
        // lights stored in lights[], what the cluster culling goes through
        uint lights_total;

        vec3 fog_color;
        // lights the shading loop goes through when the lights are not clustered
        uint lights_count;
        // 1 if the shading takes the lights from the list of the cluster of a fragment, see cluster_ubo
        uint lights_clustered;
        // scale of the directional part of the sky ambient light, see sky_ssbo
        float sky_ambient_factor;
        // 1 if rough reflections come from the prefiltered environment map
        uint env_ggx;

        light_s lights[];
};

// directional part of the sky ambient light: real spherical harmonics (bands 1 and 2, band 0 is ambient) of the light
// a surface facing a world space direction gets, in units of ambient. see gl::sky_data
layout(std430, binding = SKY_SSBO_BINDING) readonly buffer sky_ssbo
{
        vec4 sky_ambient_sh[9];
};
#endif

// light clusters, see gl/cluster_grid.h
struct cluster_aabb_s
{
        vec4 lower;
        vec4 upper;
};

struct cluster_range_s
{
        uint offset;
        uint count;
};

#if defined(STAGE_FRAGMENT) || defined(STAGE_COMPUTE)
// parameters of the grid for the pass being drawn
layout(std140, binding = CLUSTER_UBO_BINDING) uniform cluster_ubo
{
        mat4 cluster_projection;
        float cluster_znear;
        float cluster_zfar;
        float cluster_slice_scale;
        float cluster_slice_bias;
        vec2 cluster_viewport_size;
        uint cluster_valid;
};

// the compute passes fill the cluster buffers, shading only reads them
#ifdef STAGE_COMPUTE
#define CLUSTER_SSBO_ACCESS
layout(std430, binding = CLUSTER_AABB_SSBO_BINDING) buffer cluster_aabb_ssbo
{
        cluster_aabb_s cluster_aabbs[];
};

layout(std430, binding = CLUSTER_COUNTER_SSBO_BINDING) buffer cluster_counter_ssbo
{
        uint cluster_index_count;
        uint cluster_overflow_count;
};
#else
#define CLUSTER_SSBO_ACCESS readonly
#endif

layout(std430, binding = CLUSTER_RANGE_SSBO_BINDING) CLUSTER_SSBO_ACCESS buffer cluster_range_ssbo
{
        cluster_range_s cluster_ranges[];
};

layout(std430, binding = CLUSTER_INDEX_SSBO_BINDING) CLUSTER_SSBO_ACCESS buffer cluster_index_ssbo
{
        uint cluster_light_indices[];
};
#endif

#ifdef MODEL_DRAWS
// programs of the multi draws of the instances drawn from the gpu culling: the model block of every draw comes from a
// storage buffer, picked by the number of the draw in the multi draw. the fragment stage gets the number from the vertex
// stage (which writes f_model_draw), and the names of the uniform block below stand for the fields of the draw
struct model_draw_s
{
        mat4 modelview;
        mat3 modelviewnormal;
        vec4 param[MAX_PARAMS];

        mat4 future;
        float opacity;
        float emission;
        float fog_density;
        float alpha_mult;
        float shadow_tone;
        // 1 for the parts of the occupied cab, see gl::model_ubs
        float cab_interior;
};

layout(std430, binding = MODEL_DRAW_SSBO_BINDING) readonly buffer model_draw_ssbo
{
        model_draw_s model_draws[];
};

#if defined(STAGE_VERTEX)
layout(location = 11) flat out uint f_model_draw;
#define MODEL_DRAW gl_DrawID
#elif defined(STAGE_FRAGMENT)
layout(location = 11) flat in uint f_model_draw;
#define MODEL_DRAW f_model_draw
#else
#define MODEL_DRAW 0
#endif

#define modelview model_draws[MODEL_DRAW].modelview
#define modelviewnormal model_draws[MODEL_DRAW].modelviewnormal
#define param model_draws[MODEL_DRAW].param
#define future model_draws[MODEL_DRAW].future
#define opacity model_draws[MODEL_DRAW].opacity
#define emission model_draws[MODEL_DRAW].emission
#define fog_density model_draws[MODEL_DRAW].fog_density
#define alpha_mult model_draws[MODEL_DRAW].alpha_mult
#define shadow_tone model_draws[MODEL_DRAW].shadow_tone
#define cab_interior model_draws[MODEL_DRAW].cab_interior
#else
layout(std140, binding = MODEL_UBO_BINDING) uniform model_ubo
{
        mat4 modelview;
        mat3 modelviewnormal;
        vec4 param[MAX_PARAMS];

        mat4 future;
        float opacity;
        float emission;
        float fog_density;
        float alpha_mult;
        float shadow_tone;
        // 1 for the parts of the occupied cab, see gl::model_ubs
        float cab_interior;
};
#endif

layout(std140, binding = SCENE_UBO_BINDING) uniform scene_ubo
{
    mat4 projection;
    mat4 inv_view;
    mat4 lightview[MAX_CASCADES];
    vec3 cascade_end;
    float time;
    vec4 rain_params;
    vec4 wiper_pos;
    vec4 wiper_timer_out;
    vec4 wiper_timer_return;
    // shadow maps of the headlights, see gl::scene_ubs
    mat4 headlight_shadow[MAX_HEADLIGHT_SHADOWS];
    vec4 headlight_shadow_params[MAX_HEADLIGHT_SHADOWS];
    // contact shadows, see gl::scene_ubs
    vec4 contact_shadow_params;
    vec4 contact_shadow_lamps;
    // cascades of the sun shadows fitted to the depth of the view, see gl::scene_ubs
    vec4 shadow_fit;
    // screen space reflections and the reflection of the surroundings, see gl::scene_ubs
    mat4 ssr_reprojection;
    mat4 ssr_reprojection_cab;
    mat4 ssr_cab_box;
    vec4 ssr_params;
    vec4 ssr_params2;
    vec4 env_params;
    // the air against the vehicle of the camera, see gl::scene_ubs
    vec4 rain_wind;
    // the ambient occlusion in the light of the surfaces, see gl::scene_ubs
    vec4 ao_params;
    // the probe of the occupied cab, and the lamp it's drawn with, see gl::scene_ubs
    mat4 cab_probe_view;
    vec4 cab_probe_box;
    vec4 cab_probe_params;
    vec4 cab_probe_lamp;
    vec4 cab_probe_lamp_color;
    vec4 cab_probe_lamp_shadow;
};

#ifdef STAGE_VERTEX
// Per-instance modelview matrices for GPU-instanced draws, element 0 is identity.
// Draws which aren't instanced have gl_BaseInstance == 0 and gl_InstanceID == 0,
// so effective_modelview() reduces to model_ubo.modelview. For instanced draws
// the C++ side uploads camera-space root transforms of the instances and draws
// them from base instance onwards, with model_ubo.modelview set to the submodel-local
// chain (computed from identity, NOT from the camera/instance), so the product
// gives the correct final transform per instance per submodel.
layout(std430, binding = INSTANCE_SSBO_BINDING) readonly buffer instance_ssbo
{
    mat4 instance_modelview[];
};

mat4 effective_modelview() {
    return instance_modelview[gl_BaseInstance + gl_InstanceID] * modelview;
}
mat3 effective_modelviewnormal() {
    // instance_modelview can include per-instance scale (uniform or per-axis),
    // so its upper-3x3 is NOT its own inverse-transpose in the non-uniform
    // case. Compute the correct normal matrix via transpose(inverse(...)).
    // For pure rotation+translation this still produces the rotation matrix
    // (one extra mat3 inverse per vertex — modern GPUs handle it cheaply).
    mat3 instance_mv3 = mat3(instance_modelview[gl_BaseInstance + gl_InstanceID]);
    mat3 instance_normal = transpose(inverse(instance_mv3));
    return instance_normal * modelviewnormal;
}
#endif

#endif
