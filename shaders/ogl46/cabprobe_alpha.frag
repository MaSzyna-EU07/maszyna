#version 460 core
#define STAGE_FRAGMENT 1
// the probe of the occupied cab (gfx::cab_probe), the materials whose colour shader tests the alpha of the diffuse texture against
// the opacity of the material, with the test of depthprepass_alpha.frag (cabprobe.glsl)
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
	vec4 tex_color = texture(diffuse, f_coord);
	if (tex_color.a < opacity + 1.0 / 255.0)
		discard;
	probe_shade(tex_color);
}
