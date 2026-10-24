#version 460 core
#define STAGE_FRAGMENT 1
layout(location = 2) in vec2 f_coord;

#pragma texture(tex1, 0, sRGB_A)
layout(binding = 0) uniform sampler2D tex1;

#include "common.glsl"
#include "tonemapping.glsl"

layout(location = 0) out vec4 out_color;
layout(location = 1) out vec4 out_motion;

void main()
{
	vec4 tex_color = texture(tex1, f_coord);
	if (POSTFX_ENABLED)
	{
		out_color = tex_color * param[0];
	}
	else
	{
	    out_color = tonemap(tex_color * param[0]);
	}
	if (MOTIONBLUR_ENABLED)
	{
		out_motion = vec4(0.0f);
	}
}
