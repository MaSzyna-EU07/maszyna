// what the shaders of the vulkan 1.4 renderer share: the bindings of rendering/vulkan14/vulkan14bindings.h, the constants of the
// game, the settings, the push constants and the resources of the pipeline layout a shader is drawn with.
// compiled by the game, the bindings come from the renderer (vk14::binding_preamble()) and are taken instead of these; the
// SPIR-V binaries (shaders/vk14bin) are compiled with the values written here, so they have to stay the same as in the game,
// which checks them when it starts (and doesn't load the binaries when they differ)
//
// the pipeline layout is picked by a define before the include of this file:
//   (none)         the passes of the scene (materials, shadows, picking, lines, sprites, sky, particles): set PASS_SET with
//                  the blocks of the scene, of the draws, of the instances, the lights, the clusters, the shadow,
//                  environment and headlight maps, the light of the sky, the weather, the shadows of the lamps and the depth
//                  of the scene; set MATERIAL_SET with the textures of the material
//   LAYOUT_COMPUTE the compute kernels (vk14::compute_kernel): set PASS_SET as above (the block of the scene, the lights, the
//                  clusters, the light of the sky; the weather and the atmosphere with their includes), set COMPUTE_SET
//                  pushed with the bindings the kernel declares from 0 on, its parameters in push constants of its own
//   LAYOUT_POST    the post-processing and the copies of whole images: set POST_SET pushed with the images read and the
//                  exposure, the parameters in the push constants (param stands for pc.data)
//   LAYOUT_CLUSTER the compute passes of the light clusters: set 0 pushed at the CL_* bindings
//   LAYOUT_UI      the user interface: none of the above, its own push constants
//
// the locations of the inputs and outputs between the stages are the table of shaders/ogl46/common.glsl, the same for a name
// in every shader: 0 f_normal, 1 f_normal_raw, 2 f_coord, 3 f_pos, 4 f_clip_pos, 5 f_clip_future_pos, 6 TangentFragPos,
// 7 UserData, 8-10 f_tbn, 12 f_albedo, 13 f_color, 14 f_coords, 15 f_local, 16 f_soft, 17 f_streak (11 f_model_draw is not
// used: the draw is in the push constants)
#ifndef COMMON_GLSL
#define COMMON_GLSL

#if !defined(STAGE_VERTEX) && !defined(STAGE_FRAGMENT) && !defined(STAGE_GEOMETRY) && !defined(STAGE_COMPUTE)
#error "the stage of the shader isn't defined: #define STAGE_VERTEX 1 (STAGE_FRAGMENT, STAGE_GEOMETRY, STAGE_COMPUTE) before the include of common.glsl"
#endif

#ifndef MASZYNA_CONSTANTS
#define MASZYNA_CONSTANTS 1
#define UI_SET 0
#define UI_TEXTURE 0
#define PASS_SET 0
#define MATERIAL_SET 1
#define SCENE_UBO 0
#define DRAW_SSBO 1
#define INSTANCE_SSBO 2
#define LIGHT_SSBO 3
#define CLUSTER_UBO 4
#define CLUSTER_RANGE_SSBO 5
#define CLUSTER_INDEX_SSBO 6
#define SHADOW_TEX 7
#define ENV_TEX 8
#define HEADLIGHT_TEX 9
#define SKY_SSBO 10
#define ATMOSPHERE_UBO 11
#define SKY_VIEW_TEX 12
#define ENV_GGX_TEX 13
#define WEATHER_UBO 14
#define WEATHER_HORIZON_UBO 15
#define FOG_VOLUME_TEX 16
#define CLOUD_SHADOW_TEX 17
#define CLOUD_SHAPE_TEX 18
#define CLOUD_DETAIL_TEX 19
#define CLOUD_COVERAGE_TEX 20
#define LOCAL_SHADOW_TEX 21
#define LOCAL_SHADOW_LAMPS_TEX 22
#define HEADLIGHT_SHADOW_TEX 23
#define SCENE_DEPTH_TEX 24
#define SSR_HISTORY_TEX 25
#define WATER_SSBO 26
#define SCENE_AO_TEX 27
#define CAB_PROBE_TEX 28
#define CAB_PROBE_ALBEDO_TEX 29
#define CAB_PROBE_SSBO 30
#define SKY_OCCLUSION_HEIGHT_TEX 31
#define SKY_OCCLUSION_TEX 32
#define PASS_BINDING_COUNT 33
#define MATERIAL_TEXTURE_FIRST 0
#define CL_CLUSTER_UBO 0
#define CL_AABB_SSBO 1
#define CL_RANGE_SSBO 2
#define CL_INDEX_SSBO 3
#define CL_COUNTER_SSBO 4
#define CL_LIGHT_SSBO 5
#define POST_SET 0
#define POST_TEXTURE_FIRST 0
#define POST_TEXTURES 4
#define POST_EXPOSURE_SSBO 4
#define COMPUTE_SET 1
#define RT_SET 2
#define RT_SCENE 0
#define RT_RECORDS 1
#define RT_TERRAIN 2
#define RT_TEXTURES 3
#define PUSH_CONSTANTS_SIZE 112
#define SPEC_OUTPUT_GAMMA 0
#define SPEC_SHADOWMAP_ENABLED 1
#define SPEC_ENVMAP_ENABLED 2
#define SPEC_MOTIONBLUR_ENABLED 3
#define SPEC_POSTFX_ENABLED 4
#define SPEC_EXTRAEFFECTS_ENABLED 5
#define SPEC_COUNT 6
#define MAX_LIGHTS 1024U
#define MAX_CASCADES 3U
#define MAX_PARAMS 3U
#define MAX_HEADLIGHT_SHADOWS 4U
#define CLUSTER_GRID_X 16
#define CLUSTER_GRID_Y 9
#define CLUSTER_GRID_Z 24
#define CLUSTER_COUNT 3456
#define MAX_LIGHTS_PER_CLUSTER 64
#define CLUSTER_BUILD_GROUP_SIZE 64
#define CLUSTER_CULL_GROUP_SIZE 64
#define ENVMAP_GGX_LEVELS 7U
#define ENVMAP_GGX_ROUGHNESS_MIN 0.200000
#define ENVMAP_GGX_GROUP_SIZE 8
#define EXPOSURE_HISTOGRAM_BINS 256
#define EXPOSURE_HISTOGRAM_GROUP_SIZE 16
#define ATMOSPHERE_TRANSMITTANCE_WIDTH 256
#define ATMOSPHERE_TRANSMITTANCE_HEIGHT 64
#define ATMOSPHERE_MULTISCATTERING_SIZE 32
#define ATMOSPHERE_SKYVIEW_WIDTH 192
#define ATMOSPHERE_SKYVIEW_HEIGHT 108
#define ATMOSPHERE_AMBIENT_GROUP_SIZE 256
#define MAX_CLOUD_LAYERS 4
#define FOG_HORIZON_SAMPLES 32
#define WEATHER_GROUP_SIZE 8
#define FOG_VOLUME_BIAS 10.000000
#define INSTANCE_CULL_GROUP_SIZE 64
#define INSTANCE_OFFSETS_GROUP_SIZE 1024
#define INSTANCE_COMMAND_GROUP_SIZE 64
#define INSTANCE_COUNTER_HEADER 8
#define INSTANCE_JOB_SLEEPERS 1U
#define HIZ_GROUP_SIZE 8
#define SHADOW_FIT_GROUP_SIZE 16
#define LOCAL_SHADOW_NEAR 0.150000
#define PARTICLE_GROUP_SIZE 64
#define WATER_SLOTS 4
#define WATER_DROPS 65536
#define WATER_GRID 128
#define WATER_CELL_DROPS 32
#define WATER_TRAIL 1024
#define WATER_MAP 256
#define WATER_GROUP_SIZE 64
#define CAB_PROBE_SH_SIZE 16
#define CAB_PROBE_SH_GROUP_SIZE 64
#endif

#include "settings.glsl"

#if !defined(LAYOUT_UI) && !defined(LAYOUT_COMPUTE)
// vk14::draw_constants, PUSH_CONSTANTS_SIZE bytes, visible in every stage. draw: the element of model_draws[] the draw reads
// (the passes of the scene); flags and data: the parameters of the special shaders, each says what it takes from them
layout(push_constant) uniform draw_constants_block
{
	uint draw;
	uint flags;
	uint pad0;
	uint pad1;
	vec4 data[6];
} pc;
#endif

const uint LIGHT_SPOT = 0U;
const uint LIGHT_POINT = 1U;
const uint LIGHT_DIR = 2U;
const uint LIGHT_HEADLIGHTS = 3U;

// gl::light_element
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

#if defined(LAYOUT_CLUSTER)

layout(std140, set = 0, binding = CL_CLUSTER_UBO) uniform cluster_ubo
{
	mat4 cluster_projection;
	float cluster_znear;
	float cluster_zfar;
	float cluster_slice_scale;
	float cluster_slice_bias;
	vec2 cluster_viewport_size;
	uint cluster_valid;
};

layout(std430, set = 0, binding = CL_AABB_SSBO) buffer cluster_aabb_ssbo
{
	cluster_aabb_s cluster_aabbs[];
};

layout(std430, set = 0, binding = CL_RANGE_SSBO) buffer cluster_range_ssbo
{
	cluster_range_s cluster_ranges[];
};

layout(std430, set = 0, binding = CL_INDEX_SSBO) buffer cluster_index_ssbo
{
	uint cluster_light_indices[];
};

layout(std430, set = 0, binding = CL_COUNTER_SSBO) buffer cluster_counter_ssbo
{
	uint cluster_index_count;
	uint cluster_overflow_count;
};

layout(std430, set = 0, binding = CL_LIGHT_SSBO) readonly buffer light_ssbo
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

#elif defined(LAYOUT_POST)

// the post-processing reads the parameters the opengl renderer gives it in the model block from the push constants
#define param pc.data

// exposure worked out by exposure_average.comp, gl::exposure_data
layout(std430, set = POST_SET, binding = POST_EXPOSURE_SSBO) readonly buffer exposure_ssbo
{
	uint histogram[EXPOSURE_HISTOGRAM_BINS];
	float adapted_luminance;
	float exposure;
	float average_luminance;
	uint valid;
};

#elif !defined(LAYOUT_UI)

// the block of the scene, gl::scene_ubs (every member kept, the layout is the one of the c++ struct)
layout(std140, set = PASS_SET, binding = SCENE_UBO) uniform scene_ubo
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
	mat4 headlight_shadow[MAX_HEADLIGHT_SHADOWS];
	vec4 headlight_shadow_params[MAX_HEADLIGHT_SHADOWS];
	vec4 contact_shadow_params;
	vec4 contact_shadow_lamps;
	vec4 shadow_fit;
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
	// the occlusion of the sky and of the sun by the scenery around the camera, see gl::scene_ubs
	vec4 sky_occlusion_origin;
	vec4 sky_occlusion_params;
	vec4 sky_occlusion_center;
};

#ifndef LAYOUT_COMPUTE
// the model block of a draw, gl::model_ubs (256 bytes), the one of the draw picked by pc.draw (MODEL_DRAW). the names below stand for its
// fields, so the shaders read them as the uniform block of the opengl renderer
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

layout(std430, set = PASS_SET, binding = DRAW_SSBO) readonly buffer model_draw_ssbo
{
	model_draw_s model_draws[];
};

#ifdef MODEL_DRAWS
// programs of the multi draws of the instances drawn from the gpu culling: the blocks of the draws of a multi draw follow one
// another from the one of pc.draw, picked by the number of the draw in it. the fragment stage gets the block from the vertex
// stage (which writes f_model_draw)
#if defined(STAGE_VERTEX)
layout(location = 11) flat out uint f_model_draw;
#define MODEL_DRAW (pc.draw + uint(gl_DrawID))
#elif defined(STAGE_FRAGMENT)
layout(location = 11) flat in uint f_model_draw;
#define MODEL_DRAW f_model_draw
#else
#define MODEL_DRAW pc.draw
#endif
#else
#define MODEL_DRAW pc.draw
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

// per-instance modelview matrices of the instanced draws, element 0 the identity. a draw which isn't instanced is drawn with
// firstInstance 0, an instanced one from its base instance on, with modelview set to the submodel-local chain (computed from
// the identity, not from the camera or the instance), so the product is the final transform per instance per submodel
layout(std430, set = PASS_SET, binding = INSTANCE_SSBO) readonly buffer instance_ssbo
{
	mat4 instance_modelview[];
};
#endif

// the light buffer of the pass, gl::light_data
layout(std430, set = PASS_SET, binding = LIGHT_SSBO) readonly buffer light_ssbo
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
layout(std430, set = PASS_SET, binding = SKY_SSBO) readonly buffer sky_ssbo
{
	vec4 sky_ambient_sh[9];
};

// the grid of the light clusters of the pass, gl::cluster_ubs; the compute passes fill the lists, shading only reads them
layout(std140, set = PASS_SET, binding = CLUSTER_UBO) uniform cluster_ubo
{
	mat4 cluster_projection;
	float cluster_znear;
	float cluster_zfar;
	float cluster_slice_scale;
	float cluster_slice_bias;
	vec2 cluster_viewport_size;
	uint cluster_valid;
};

layout(std430, set = PASS_SET, binding = CLUSTER_RANGE_SSBO) readonly buffer cluster_range_ssbo
{
	cluster_range_s cluster_ranges[];
};

layout(std430, set = PASS_SET, binding = CLUSTER_INDEX_SSBO) readonly buffer cluster_index_ssbo
{
	uint cluster_light_indices[];
};

#ifdef STAGE_VERTEX
mat4 effective_modelview()
{
	return instance_modelview[gl_InstanceIndex] * modelview;
}

mat3 effective_modelviewnormal()
{
	// instance_modelview can include per-instance scale (uniform or per-axis), so its upper 3x3 is not its own inverse
	// transpose in the non-uniform case
	mat3 instance_mv3 = mat3(instance_modelview[gl_InstanceIndex]);
	mat3 instance_normal = transpose(inverse(instance_mv3));
	return instance_normal * modelviewnormal;
}
#endif

#endif

#endif
