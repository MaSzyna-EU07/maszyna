#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"

layout(location = 3) in vec4 f_pos;
layout(location = 4) in vec4 f_clip_pos;
layout(location = 5) in vec4 f_clip_future_pos;

layout(location = 0) out vec4 out_color;
#ifdef NO_MOTION_OUTPUT
// a pass without the velocity target: the writes go nowhere
vec4 out_motion;
#else
layout(location = 1) out vec4 out_motion;
#endif

#include "tonemapping.glsl"
#include "apply_fog.glsl"

void main()
{
	vec3 wire = pow(param[0].rgb, vec3(2.2));
	// the ice on the wires, the ice the pantographs meet (environment/snowcover.h), param[2] each kind, mm: the glaze clear and grey
	// like glass, the hard rime white, the soft rime and the hoarfrost whiter still, the wet snow a greyish white
	float ice = param[2].x + param[2].y + param[2].z + param[2].w;
	if (ice > 0.0)
	{
		vec3 icecolor = (vec3(0.50, 0.56, 0.62) * param[2].x + vec3(0.80, 0.83, 0.86) * param[2].y + vec3(0.90, 0.92, 0.95) * param[2].z + vec3(0.74, 0.76, 0.78) * param[2].w) / ice;
		wire = mix(wire, icecolor, smoothstep(0.3, 6.0, ice));
	}
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
