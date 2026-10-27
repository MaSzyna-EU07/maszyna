#version 460 core
#define STAGE_FRAGMENT 1
// the probe of the occupied cab (gfx::cab_probe), the materials without the alpha test (cabprobe.glsl)
layout(location = 0) in vec3 f_normal;
layout(location = 2) in vec2 f_coord;
layout(location = 3) in vec4 f_pos;

#include "common.glsl"

#pragma texture(diffuse, 0, sRGB_A)
layout(binding = 0) uniform sampler2D diffuse;
// the cube shadow map of the cab lamp, see gl::LOCAL_SHADOW_TEX
layout(binding = LOCAL_SHADOW_TEX) uniform samplerCubeArrayShadow probe_shadowmap;

#include "cabprobe.glsl"

void main()
{
	probe_shade(texture(diffuse, f_coord));
}
