#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
// the probe of the occupied cab (gfx::cab_probe), the materials without the alpha test (cabprobe.glsl)
layout(location = 0) in vec3 f_normal;
layout(location = 2) in vec2 f_coord;
layout(location = 3) in vec4 f_pos;

#pragma texture(diffuse, 0, sRGB_A)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 0) uniform sampler2D diffuse;
// the cube shadow map of the cab lamp
layout(set = PASS_SET, binding = LOCAL_SHADOW_TEX) uniform samplerCubeArrayShadow probe_shadowmap;
// the probe drawn last, where the windows of the cab are
layout(set = PASS_SET, binding = CAB_PROBE_MASK_TEX) uniform samplerCube cab_probe_mask;

#include "weather_common.glsl"
#include "cabprobe.glsl"

void main()
{
	probe_shade(texture(diffuse, f_coord));
}
