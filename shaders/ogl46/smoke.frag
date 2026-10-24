#version 460 core
#define STAGE_FRAGMENT 1
layout(location = 13) in vec4 f_color;
layout(location = 2) in vec2 f_coord;
layout(location = 3) in vec4 f_pos;

layout(location = 4) in vec4 f_clip_pos;
layout(location = 5) in vec4 f_clip_future_pos;

#pragma texture(tex1, 0, sRGB_A)
layout(binding = 0) uniform sampler2D tex1;

#include "common.glsl"
#include "apply_fog.glsl"
#include "tonemapping.glsl"

layout(location = 0) out vec4 out_color;
layout(location = 1) out vec4 out_motion;

void main()
{
	vec4 tex_color = texture(tex1, f_coord) * f_color;
	if (POSTFX_ENABLED)
	{
		out_color = vec4(apply_fog(tex_color.rgb), tex_color.a);
	}
	else
	{
	    out_color = tonemap(vec4(apply_fog(tex_color.rgb), tex_color.a));
	}
	if (MOTIONBLUR_ENABLED)
	{
        vec2 a = (f_clip_future_pos.xy / f_clip_future_pos.w) * 0.5 + 0.5;;
        vec2 b = (f_clip_pos.xy / f_clip_pos.w) * 0.5 + 0.5;;
        
        out_motion = vec4(a - b, 0.0f, tex_color.a * alpha_mult);
	}
}
