#include "stdafx.h"
#include "glsl_common.h"

std::string gl::glsl_common;

void gl::glsl_common_setup()
{
    glsl_common =
    "#define SHADOWMAP_ENABLED " + std::to_string((int)Global.gfx_shadowmap_enabled) + "\n" +
    "#define ENVMAP_ENABLED " + std::to_string((int)Global.gfx_envmap_enabled) + "\n" +
    "#define MOTIONBLUR_ENABLED " + std::to_string((int)Global.gfx_postfx_motionblur_enabled) + "\n" +
    "#define POSTFX_ENABLED " + std::to_string((int)!Global.gfx_skippipeline) + "\n" +
    "#define EXTRAEFFECTS_ENABLED " + std::to_string((int)Global.gfx_extraeffects) + "\n" +
    "#define MAX_LIGHTS " + std::to_string(MAX_LIGHTS) + "U\n" +
    "#define MAX_CASCADES " + std::to_string(MAX_CASCADES) + "U\n" +
    "#define MAX_PARAMS " + std::to_string(MAX_PARAMS) + "U\n" +
    "#define INSTANCE_SSBO_BINDING " + std::to_string(INSTANCE_SSBO_BINDING) + "\n" +
    "#define INSTANCE_RECORD_SSBO_BINDING " + std::to_string(INSTANCE_RECORD_SSBO_BINDING) + "\n" +
    "#define INSTANCE_JOB_SSBO_BINDING " + std::to_string(INSTANCE_JOB_SSBO_BINDING) + "\n" +
    "#define INSTANCE_GROUP_SSBO_BINDING " + std::to_string(INSTANCE_GROUP_SSBO_BINDING) + "\n" +
    "#define INSTANCE_LOD_SSBO_BINDING " + std::to_string(INSTANCE_LOD_SSBO_BINDING) + "\n" +
    "#define INSTANCE_COUNTER_SSBO_BINDING " + std::to_string(INSTANCE_COUNTER_SSBO_BINDING) + "\n" +
    "#define INSTANCE_OUTPUT_SSBO_BINDING " + std::to_string(INSTANCE_OUTPUT_SSBO_BINDING) + "\n" +
    "#define INSTANCE_COMMAND_SSBO_BINDING " + std::to_string(INSTANCE_COMMAND_SSBO_BINDING) + "\n" +
    "#define INSTANCE_CULL_GROUP_SIZE " + std::to_string(INSTANCE_CULL_GROUP_SIZE) + "\n" +
    "#define INSTANCE_OFFSETS_GROUP_SIZE " + std::to_string(INSTANCE_OFFSETS_GROUP_SIZE) + "\n" +
    "#define INSTANCE_COMMAND_GROUP_SIZE " + std::to_string(INSTANCE_COMMAND_GROUP_SIZE) + "\n" +
    "#define HIZ_GROUP_SIZE " + std::to_string(HIZ_GROUP_SIZE) + "\n" +
    "#define INSTANCE_COUNTER_HEADER " + std::to_string(INSTANCE_COUNTER_HEADER) + "\n" +
    "#define LIGHT_SSBO_BINDING " + std::to_string(LIGHT_SSBO_BINDING) + "\n" +
    "#define CLUSTER_GRID_X " + std::to_string(CLUSTER_GRID_X) + "\n" +
    "#define CLUSTER_GRID_Y " + std::to_string(CLUSTER_GRID_Y) + "\n" +
    "#define CLUSTER_GRID_Z " + std::to_string(CLUSTER_GRID_Z) + "\n" +
    "#define CLUSTER_COUNT " + std::to_string(CLUSTER_COUNT) + "\n" +
    "#define MAX_LIGHTS_PER_CLUSTER " + std::to_string(MAX_LIGHTS_PER_CLUSTER) + "\n" +
    "#define CLUSTER_BUILD_GROUP_SIZE " + std::to_string(CLUSTER_BUILD_GROUP_SIZE) + "\n" +
    "#define CLUSTER_CULL_GROUP_SIZE " + std::to_string(CLUSTER_CULL_GROUP_SIZE) + "\n" +
    "#define CLUSTER_UBO_BINDING " + std::to_string(CLUSTER_UBO_BINDING) + "\n" +
    "#define CLUSTER_AABB_SSBO_BINDING " + std::to_string(CLUSTER_AABB_SSBO_BINDING) + "\n" +
    "#define CLUSTER_RANGE_SSBO_BINDING " + std::to_string(CLUSTER_RANGE_SSBO_BINDING) + "\n" +
    "#define CLUSTER_INDEX_SSBO_BINDING " + std::to_string(CLUSTER_INDEX_SSBO_BINDING) + "\n" +
    "#define CLUSTER_COUNTER_SSBO_BINDING " + std::to_string(CLUSTER_COUNTER_SSBO_BINDING) + "\n" +
    "#define ENVMAP_GGX_LEVELS " + std::to_string(ENVMAP_GGX_LEVELS) + "\n" +
    "#define ENVMAP_GGX_ROUGHNESS_MIN " + std::to_string(ENVMAP_GGX_ROUGHNESS_MIN) + "\n" +
    "#define ENVMAP_GGX_GROUP_SIZE " + std::to_string(ENVMAP_GGX_GROUP_SIZE) + "\n" +
    "#define SKY_SSBO_BINDING " + std::to_string(SKY_SSBO_BINDING) + "\n" +
    "#define EXPOSURE_SSBO_BINDING " + std::to_string(EXPOSURE_SSBO_BINDING) + "\n" +
    "#define EXPOSURE_HISTOGRAM_BINS " + std::to_string(EXPOSURE_HISTOGRAM_BINS) + "\n" +
    "#define EXPOSURE_HISTOGRAM_GROUP_SIZE " + std::to_string(EXPOSURE_HISTOGRAM_GROUP_SIZE) + "\n" +
    "#define ATMOSPHERE_UBO_BINDING " + std::to_string(ATMOSPHERE_UBO_BINDING) + "\n" +
    "#define ATMOSPHERE_RESULT_SSBO_BINDING " + std::to_string(ATMOSPHERE_RESULT_SSBO_BINDING) + "\n" +
    "#define ATMOSPHERE_TRANSMITTANCE_WIDTH " + std::to_string(ATMOSPHERE_TRANSMITTANCE_WIDTH) + "\n" +
    "#define ATMOSPHERE_TRANSMITTANCE_HEIGHT " + std::to_string(ATMOSPHERE_TRANSMITTANCE_HEIGHT) + "\n" +
    "#define ATMOSPHERE_MULTISCATTERING_SIZE " + std::to_string(ATMOSPHERE_MULTISCATTERING_SIZE) + "\n" +
    "#define ATMOSPHERE_SKYVIEW_WIDTH " + std::to_string(ATMOSPHERE_SKYVIEW_WIDTH) + "\n" +
    "#define ATMOSPHERE_SKYVIEW_HEIGHT " + std::to_string(ATMOSPHERE_SKYVIEW_HEIGHT) + "\n" +
    "#define ATMOSPHERE_AMBIENT_GROUP_SIZE " + std::to_string(ATMOSPHERE_AMBIENT_GROUP_SIZE) + "\n" +
    "#define ATMOSPHERE_TRANSMITTANCE_UNIT " + std::to_string(ATMOSPHERE_TRANSMITTANCE_UNIT) + "\n" +
    "#define ATMOSPHERE_MULTISCATTERING_UNIT " + std::to_string(ATMOSPHERE_MULTISCATTERING_UNIT) + "\n" +
    "#define ATMOSPHERE_SKYVIEW_UNIT " + std::to_string(ATMOSPHERE_SKYVIEW_UNIT) + "\n" +
    "#define SKY_VIEW_TEX " + std::to_string(SKY_VIEW_TEX) + "\n" +
    "#define LOCAL_SHADOW_TEX " + std::to_string(LOCAL_SHADOW_TEX) + "\n" +
    "#define LOCAL_SHADOW_LAMPS_TEX " + std::to_string(LOCAL_SHADOW_LAMPS_TEX) + "\n" +
    "#define HEADLIGHT_SHADOW_TEX " + std::to_string(HEADLIGHT_SHADOW_TEX) + "\n" +
    "#define MAX_HEADLIGHT_SHADOWS " + std::to_string(MAX_HEADLIGHT_SHADOWS) + "U\n" +
    R"STRING(
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

    layout (std140) uniform model_ubo
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
    };

    layout (std140) uniform scene_ubo
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

    )STRING";
}
