#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"

layout(location = 3) in vec4 f_pos;
layout(location = 4) in vec4 f_clip_pos;
layout(location = 5) in vec4 f_clip_future_pos;

layout(location = 0) out vec4 out_color;
layout(location = 1) out vec4 out_motion;

#include "tonemapping.glsl"
#include "apply_fog.glsl"

void main()
{
	vec3 wire = pow(param[0].rgb, vec3(2.2));
	// the ice on the wires (surface_state.w, mm: the ice the pantographs meet, environment/snowcover.h) whitens them
	wire = mix(wire, vec3(0.62, 0.66, 0.70), smoothstep(0.5, 8.0, surface_state.w));
	vec4 color = vec4(apply_fog(wire), param[0].a);
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
        vec2 a = (f_clip_future_pos.xy / f_clip_future_pos.w) * 0.5 + 0.5;;
        vec2 b = (f_clip_pos.xy / f_clip_pos.w) * 0.5 + 0.5;;
        
        out_motion = vec4(a - b, 0.0f, 0.0f);
	}
}
