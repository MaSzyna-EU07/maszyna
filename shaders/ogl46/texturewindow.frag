#version 460 core
#define STAGE_FRAGMENT 1
layout(location = 14) in vec2 f_coords;

layout(location = 0) out vec4 out_color;

#include "settings.glsl"

#pragma texture(tex1, 0, sRGB)
layout(binding = 0) uniform sampler2D tex1;

void main()
{
	vec2 texcoord = f_coords;
	vec3 color = texture(tex1, texcoord).rgb;

	out_color = FBOUT(vec4(color, 1.0));
}  
