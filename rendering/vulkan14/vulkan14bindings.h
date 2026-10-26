/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the bindings of the shaders of the vulkan renderer, in one place for the c++ code and the shaders: the renderer puts them
// before the sources of shaders/vk14 as #define lines (vk14::binding_preamble()), and shaders/vk14/common.glsl writes the
// same values for the SPIR-V binaries, which glslc compiles without the renderer (the renderer checks they are the same).
//
// the descriptor sets (decision K6 of the plan): set 0 holds the resources of a pass, made when the renderer starts and bound
// with dynamic offsets in the ring of the frame; set 1 is pushed with the draws (vkCmdPushDescriptorSet2): the textures of
// the material. a layout can have only one set pushed, so the passes without a set of their own (the user interface, the
// compute passes, the post-processing) push set 0

#pragma once

#include <cstdint>
#include <string>

namespace vk14::bindings
{

// the user interface (imgui): one pushed set with the texture, the transform in the push constants
constexpr std::uint32_t UI_SET = 0;
constexpr std::uint32_t UI_TEXTURE = 0;

// the sets of the passes of the scene
constexpr std::uint32_t PASS_SET = 0;
constexpr std::uint32_t MATERIAL_SET = 1;
// set 0 of the scene: the block of the scene (gl::scene_ubs, dynamic offset in the ring of the frame)
constexpr std::uint32_t SCENE_UBO = 0;
// the blocks of the draws of the frame (gl::model_ubs each), the one of a draw picked by the push constant draw
constexpr std::uint32_t DRAW_SSBO = 1;
// the modelview matrices of the instances of the frame, element 0 the identity (draws which aren't instanced)
constexpr std::uint32_t INSTANCE_SSBO = 2;
// the light buffer of the pass (gl::light_data, dynamic offset)
constexpr std::uint32_t LIGHT_SSBO = 3;
// the light clusters: the grid of the pass (gl::cluster_ubs, dynamic offset), the lists of the lights of the clusters
constexpr std::uint32_t CLUSTER_UBO = 4;
constexpr std::uint32_t CLUSTER_RANGE_SSBO = 5;
constexpr std::uint32_t CLUSTER_INDEX_SSBO = 6;
// the cascades of the sun shadows (depth compared), the environment map, the beams of the headlights
constexpr std::uint32_t SHADOW_TEX = 7;
constexpr std::uint32_t ENV_TEX = 8;
constexpr std::uint32_t HEADLIGHT_TEX = 9;
// the light of the sky (the stage P1): the directional part of the ambient light (gl::sky_data), the parameters of the
// atmosphere (gl::atmosphere_ubs), its sky-view table, the environment map prefiltered with the GGX lobe
constexpr std::uint32_t SKY_SSBO = 10;
constexpr std::uint32_t ATMOSPHERE_UBO = 11;
constexpr std::uint32_t SKY_VIEW_TEX = 12;
constexpr std::uint32_t ENV_GGX_TEX = 13;
// the weather (the stage P2): its block (gl::weather_ubs without the colours of the horizon, dynamic offset: it differs from
// pass to pass), the colours of the fog towards the horizon (written by fog_horizon.comp or by the cpu), the fog volume of the
// main view, the shadows of the clouds and the noises of the clouds
constexpr std::uint32_t WEATHER_UBO = 14;
constexpr std::uint32_t WEATHER_HORIZON_UBO = 15;
constexpr std::uint32_t FOG_VOLUME_TEX = 16;
constexpr std::uint32_t CLOUD_SHADOW_TEX = 17;
constexpr std::uint32_t CLOUD_SHAPE_TEX = 18;
constexpr std::uint32_t CLOUD_DETAIL_TEX = 19;
constexpr std::uint32_t CLOUD_COVERAGE_TEX = 20;
// the shadows of the lamps (the stage P4): the cube maps of the cab lamp and of the scenery lamps, the maps of the headlights
constexpr std::uint32_t LOCAL_SHADOW_TEX = 21;
constexpr std::uint32_t LOCAL_SHADOW_LAMPS_TEX = 22;
constexpr std::uint32_t HEADLIGHT_SHADOW_TEX = 23;
// the depth of the opaque scene drawn before its colour (the stage P3: the contact shadows; the soft particles of P6)
constexpr std::uint32_t SCENE_DEPTH_TEX = 24;
// the image of the last frame of the main view with its mipmaps, which the screen space reflections take their colour from
constexpr std::uint32_t SSR_HISTORY_TEX = 25;
// the water on the windscreens (gfx::windscreen_water): the drops, their grids and the wetted paths the glasses read
constexpr std::uint32_t WATER_SSBO = 26;
constexpr std::uint32_t PASS_BINDING_COUNT = 27;
// the offsets of the bindings above with dynamic offsets, in the order of the bindings
constexpr std::uint32_t PASS_DYNAMIC_OFFSETS = 6;
// the textures of a material take the bindings of their units (#pragma texture(name, unit, ...)), 0 .. MAX_TEXTURES - 1
constexpr std::uint32_t MATERIAL_TEXTURE_FIRST = 0;

// the compute passes of the light clusters, set 0 pushed
constexpr std::uint32_t CL_CLUSTER_UBO = 0;
constexpr std::uint32_t CL_AABB_SSBO = 1;
constexpr std::uint32_t CL_RANGE_SSBO = 2;
constexpr std::uint32_t CL_INDEX_SSBO = 3;
constexpr std::uint32_t CL_COUNTER_SSBO = 4;
constexpr std::uint32_t CL_LIGHT_SSBO = 5;

// the post-processing and the copies of whole images: set 0 pushed, the images it reads from binding 0 on, and the exposure
// worked out on the gpu (gl::exposure_data, the stage P1)
constexpr std::uint32_t POST_SET = 0;
constexpr std::uint32_t POST_TEXTURE_FIRST = 0;
constexpr std::uint32_t POST_TEXTURES = 4;
constexpr std::uint32_t POST_EXPOSURE_SSBO = 4;

// the compute kernels (vk14::compute_kernel): set 0 the pass set of the scene, set 1 pushed with the dispatch, its bindings
// those the shader declares from 0 on
constexpr std::uint32_t COMPUTE_SET = 1;

// the ray traced reflections (gfx.raytracing, vk14::ray_tracing): set 2 of the pipelines of the scene, one for each frame in flight,
// written with the frame: the acceleration structure of the scenery around the camera, the records of its instances, the layers
// of the heightmap terrain (the texture array of their diffuse maps, partly bound) and the textures they read (an array of as many
// as the device allows, partly bound, the last binding)
constexpr std::uint32_t RT_SET = 2;
constexpr std::uint32_t RT_SCENE = 0;
constexpr std::uint32_t RT_RECORDS = 1;
constexpr std::uint32_t RT_TERRAIN = 2;
constexpr std::uint32_t RT_TEXTURES = 3;

// the push constants of the graphics pipelines of the scene and of the post-processing (vk14::draw_constants): the draw, and
// the parameters of the special shaders (the celestial bodies, the post-processing)
constexpr std::uint32_t PUSH_CONSTANTS_SIZE = 112;

// the specialization constants of the shaders
// 1: the colour written to the window is encoded by the shader (a window without an srgb format, gfx.shadergamma yes)
constexpr std::uint32_t SPEC_OUTPUT_GAMMA = 0;
// the settings of the game (gfx.* in eu07.ini) the shaders of the scene depend on, as in shaders/ogl46/settings.glsl
constexpr std::uint32_t SPEC_SHADOWMAP_ENABLED = 1;
constexpr std::uint32_t SPEC_ENVMAP_ENABLED = 2;
constexpr std::uint32_t SPEC_MOTIONBLUR_ENABLED = 3;
constexpr std::uint32_t SPEC_POSTFX_ENABLED = 4;
constexpr std::uint32_t SPEC_EXTRAEFFECTS_ENABLED = 5;
constexpr std::uint32_t SPEC_COUNT = 6;

} // namespace vk14::bindings

namespace vk14
{

// the bindings above as #define lines, with the constants of the game the shaders share with the opengl renderer (the sizes of
// the blocks, the cluster grid), and MASZYNA_CONSTANTS, which tells shaders/vk14/common.glsl not to write its own. Raytracing:
// the shaders of the scene trace the reflections the screen doesn't hold (REFLECTION_RAYS, with the extensions it needs)
std::string binding_preamble(bool const Raytracing = false);

} // namespace vk14
