// what the shaders of the vulkan 1.4 renderer share: the bindings of rendering/vulkan14/vulkan14bindings.h, the constants of the
// game, the settings, the push constants and the resources of the pipeline layout a shader is drawn with.
// compiled by the game, the bindings come from the renderer (vk14::binding_preamble()) and are taken instead of these; the
// SPIR-V binaries (shaders/vk14bin) are compiled with the values written here, so they have to stay the same as in the game,
// which checks them when it starts (and doesn't load the binaries when they differ)
//
// the pipeline layout is picked by a define before the include of this file:
//   (none)         the passes of the scene (materials, shadows, picking, lines, sprites, sky, particles): set PASS_SET with
//                  the blocks of the scene, of the draws, of the instances, the lights, the clusters and the shadow,
//                  environment and headlight maps; set MATERIAL_SET with the textures of the material
//   LAYOUT_POST    the post-processing and the copies of whole images: set POST_SET pushed with the images read, the
//                  parameters in the push constants (param stands for pc.data)
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
#define PASS_BINDING_COUNT 10
#define MATERIAL_TEXTURE_FIRST 0
#define CL_CLUSTER_UBO 0
#define CL_AABB_SSBO 1
#define CL_RANGE_SSBO 2
#define CL_INDEX_SSBO 3
#define CL_COUNTER_SSBO 4
#define CL_LIGHT_SSBO 5
#define POST_SET 0
#define POST_TEXTURE_FIRST 0
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
#endif

#include "settings.glsl"

#ifndef LAYOUT_UI
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
	// not used by the shaders of this renderer yet (the sky ambient light and the prefiltered environment map come later)
	float sky_ambient_factor;
	uint env_ggx;

	light_s lights[];
};

#elif defined(LAYOUT_POST)

// the post-processing reads the parameters the opengl renderer gives it in the model block from the push constants
#define param pc.data

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
};

// the model block of a draw, gl::model_ubs (256 bytes), the one of the draw picked by pc.draw. the names below stand for its
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
};

layout(std430, set = PASS_SET, binding = DRAW_SSBO) readonly buffer model_draw_ssbo
{
	model_draw_s model_draws[];
};

#define modelview model_draws[pc.draw].modelview
#define modelviewnormal model_draws[pc.draw].modelviewnormal
#define param model_draws[pc.draw].param
#define future model_draws[pc.draw].future
#define opacity model_draws[pc.draw].opacity
#define emission model_draws[pc.draw].emission
#define fog_density model_draws[pc.draw].fog_density
#define alpha_mult model_draws[pc.draw].alpha_mult
#define shadow_tone model_draws[pc.draw].shadow_tone

// per-instance modelview matrices of the instanced draws, element 0 the identity. a draw which isn't instanced is drawn with
// firstInstance 0, an instanced one from its base instance on, with modelview set to the submodel-local chain (computed from
// the identity, not from the camera or the instance), so the product is the final transform per instance per submodel
layout(std430, set = PASS_SET, binding = INSTANCE_SSBO) readonly buffer instance_ssbo
{
	mat4 instance_modelview[];
};

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
	// not used by the shaders of this renderer yet (the sky ambient light and the prefiltered environment map come later)
	float sky_ambient_factor;
	uint env_ggx;

	light_s lights[];
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
