#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
#include "tonemapping.glsl"

layout(location = 1) flat in vec3 f_normal_raw;

layout(location = 0) out vec4 out_color;
layout(location = 1) out vec4 out_motion;

void main()
{
	float x = (gl_PointCoord.x - 0.5f) * 2.0f;
	float y = (gl_PointCoord.y - 0.5f) * 2.0f;
	float dist2 = abs(x * x + y * y);
	if (dist2 > 0.5f * 0.5f)
		discard;

	// color data space is shared with normals, ugh
	vec4 color = vec4(pow(f_normal_raw.rgb, vec3(2.2)), 1.0f);
    if (POSTFX_ENABLED)
    {
        out_color = color;
    }
    else
    {
        out_color = tonemap(color);
    }
	if (MOTIONBLUR_ENABLED)
	{
		out_motion = vec4(0.0f);
	}
}
