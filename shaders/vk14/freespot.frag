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
	float x = (gl_PointCoord.x - 0.5f) * 2.0f;
	float y = (gl_PointCoord.y - 0.5f) * 2.0f;
	float dist = sqrt(x * x + y * y);
	if (dist > 0.5f)
		discard;
//	vec4 color = vec4(param[0].rgb * emission, mix(param[0].a, 0.0f, dist * 2.0f));
	vec4 color = vec4(apply_fog(pow(param[0].rgb, vec3(2.2)) * emission), mix(param[0].a, 0.0f, dist * 2.0f));
//	vec4 color = vec4(add_fog(param[0].rgb * emission * (1.0 - get_fog_amount()), 1.0), mix(param[0].a, 0.0f, dist * 2.0f));
    out_color = FBOUT(color);
	if (MOTIONBLUR_ENABLED)
	{
        vec2 a = (f_clip_future_pos.xy / f_clip_future_pos.w) * 0.5 + 0.5;;
        vec2 b = (f_clip_pos.xy / f_clip_pos.w) * 0.5 + 0.5;;
        
        out_motion = vec4(a - b, 0.0f, 0.0f);
	}
}
