#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
layout(location = 2) in vec2 f_coord;

#pragma texture(tex1, 0, sRGB_A)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 0) uniform sampler2D tex1;


layout(location = 0) out vec4 out_color;
#ifdef NO_MOTION_OUTPUT
// a pass without the velocity target: the writes go nowhere
vec4 out_motion;
#else
layout(location = 1) out vec4 out_motion;
#endif

layout(location = 4) in vec4 f_clip_pos;
layout(location = 5) in vec4 f_clip_future_pos;

#include "tonemapping.glsl"

void main()
{
	vec4 tex_color = texture(tex1, vec2(f_coord.x, f_coord.y + param[1].x));
	vec4 color = tex_color * param[0];
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
        vec2 a = (f_clip_future_pos.xy / f_clip_future_pos.w) * 0.5 + 0.5;
        vec2 b = (f_clip_pos.xy / f_clip_pos.w) * 0.5 + 0.5;
        
        out_motion = vec4(a - b, 0.0f, tex_color.a * alpha_mult);
	}
}
