// the materials of the metallic and roughness workflow, which take the textures the way the texturing tools export them:
//   diffuse    the base colour (sRGB), the opacity in its alpha
//   normalmap  the normals of the tangent space in the red and green, with the green pointing up the texture (the opengl
//              convention)
//   orm        the occlusion of the ambient light in the red, the roughness in the green and the metalness in the blue, the
//              packing of unreal engine and gltf. without the texture: no occlusion, roughness: alone, no metal
//   emissive   the light the surface emits (sRGB), whatever falls on it. without the texture: none
// and the parameters
//   specular    the specular level of the parts which aren't metal, 1 for the reflectance of 4% (the level 0.5 of the tools)
//   reflection  how much of the reflected surroundings is the environment map and the scene, the rest the ambient light
//   emissive    scales the emissive texture
//   roughness   scales the roughness of the texture
//
// mat_pbr.frag and its variants are this file with what the variant adds, picked by a define before the include:
//   PBR_DETAIL    a second normal map (detailnormalmap) repeated detail_scale times over the first, of the strength
//                 detail_height_scale
//   PBR_PARALLAX  the height of the surface (height: white the highest) shifts the textures by height_scale
//   PBR_COAT      a clear varnish over the surface: coat scales how much of it there is (the red of coatmap), coat_roughness
//                 is added to its roughness (the green of coatmap). without the texture: varnish all over, as smooth as
//                 coat_roughness says
// the variant declares its own textures and parameters before the include (and after the one of common.glsl, which has the
// set and the bindings of the textures): every #pragma texture and #pragma param in a file is read, whatever #ifdef it is in.
// shaders/ogl46/mat_pbr.glsl with the bindings of the vulkan renderer
#include "common.glsl"
layout(location = 0) in vec3 f_normal;
layout(location = 2) in vec2 f_coord;
layout(location = 3) in vec4 f_pos;
layout(location = 8) in mat3 f_tbn;

layout(location = 4) in vec4 f_clip_pos;
layout(location = 5) in vec4 f_clip_future_pos;
#ifdef PBR_PARALLAX
layout(location = 6) in vec3 TangentFragPos;
#endif


layout(location = 0) out vec4 out_color;
#ifdef NO_MOTION_OUTPUT
// a pass without the velocity target: the writes go nowhere
vec4 out_motion;
#else
layout(location = 1) out vec4 out_motion;
#endif

#pragma param(color, 0, 0, 4, diffuse)
#pragma param(diffuse, 1, 0, 1, diffuse)
#pragma param(specular, 1, 1, 1, one)
#pragma param(reflection, 1, 2, 1, one)
#pragma param(emissive, 1, 3, 1, one)
#pragma param(roughness, 2, 0, 1, one)

#pragma texture(diffuse, 0, sRGB_A)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 0) uniform sampler2D diffuse;

#pragma texture(normalmap, 1, RGBA)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 1) uniform sampler2D normalmap;

#pragma texture(orm, 2, RGBA, yellow)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 2) uniform sampler2D orm;

#pragma texture(emissive, 3, sRGB_A, black)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 3) uniform sampler2D emissive;

// lit as the tools show the surface, see light_common.glsl
#define PBR_SURFACE
#include "light_common.glsl"
#include "apply_fog.glsl"
#include "tonemapping.glsl"

// the same file for both renderers
#include "mat_pbr_surface.glsl"
