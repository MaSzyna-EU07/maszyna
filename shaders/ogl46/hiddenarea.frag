#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"

layout(location = 0) out vec4 out_color;
layout(location = 1) out vec4 out_motion;

void main()
{
out_color = vec4(vec3(0.0f), 1.0f);
	if (MOTIONBLUR_ENABLED)
	{
		out_motion = vec4(0.0f);
	}
gl_FragDepth = 1.0f;
}
