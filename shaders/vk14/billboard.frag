#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
layout(location = 2) in vec2 f_coord;

#pragma texture(tex1, 0, sRGB_A)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 0) uniform sampler2D tex1;

#include "tonemapping.glsl"

layout(location = 0) out vec4 out_color;
#ifdef NO_MOTION_OUTPUT
// a pass without the velocity target: the writes go nowhere
vec4 out_motion;
#else
layout(location = 1) out vec4 out_motion;
#endif

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
