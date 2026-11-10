#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
layout(location = 13) in vec3 f_color;
layout(location = 3) in vec4 f_pos;

#include "apply_fog.glsl"
#include "tonemapping.glsl"

layout(location = 0) out vec4 out_color;
#ifdef NO_MOTION_OUTPUT
// a pass without the velocity target: the writes go nowhere
vec4 out_motion;
#else
layout(location = 1) out vec4 out_motion;
#endif

#include "sky_shoulder.glsl"

void main()
{
	vec3 col = sky_shoulder(pow(max(f_color.rgb, vec3(0.0)), vec3(2.2)));
	if (POSTFX_ENABLED)
	{
		out_color = vec4(apply_fog(col), 1.0f);
	}
	else
	{
	    out_color = tonemap(vec4(apply_fog(col), 1.0f));
	}
	if (MOTIONBLUR_ENABLED)
	{
		out_motion = vec4(0.0f);
	}
}
