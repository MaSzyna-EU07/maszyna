#version 460 core
#define STAGE_FRAGMENT 1
// a star: a soft point added to the sky

layout(location = 13) in vec3 f_color;

#include "common.glsl"
#include "tonemapping.glsl"

layout(location = 0) out vec4 out_color;
layout(location = 1) out vec4 out_motion;

void main()
{
	vec2 offset = gl_PointCoord - 0.5;
	float spot = exp(-dot(offset, offset) * 14.0);
	vec3 color = f_color * spot;
	if (POSTFX_ENABLED)
	{
		out_color = vec4(color, 0.0);
	}
	else
	{
		out_color = vec4(tonemap(vec4(color, 1.0)).rgb, 0.0);
	}
	if (MOTIONBLUR_ENABLED)
	{
		out_motion = vec4(0.0);
	}
}
