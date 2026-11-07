#version 460 core
#define STAGE_FRAGMENT 1
// grass of the heightmap terrain (opengl46_grass, grass.vert): the blades lit like the rest of the scene

layout(location = 0) in vec3 f_normal;
layout(location = 3) in vec4 f_pos;
layout(location = 12) in vec3 f_albedo;
layout(location = 4) in vec4 f_clip_pos;
layout(location = 5) in vec4 f_clip_future_pos;

#include "common.glsl"

layout(location = 0) out vec4 out_color;
layout(location = 1) out vec4 out_motion;

#include "light_common.glsl"
#include "apply_fog.glsl"
#include "tonemapping.glsl"

void main()
{
	vec3 fragnormal = normalize(f_normal);
	glossiness = phong_glossiness(6.0);
	metalic = 0.0;
	vec3 fragcolor = apply_lights(ambient, fragnormal, f_albedo, 0.0, 0.2, shadow_tone);

	vec4 color = vec4(apply_fog(fragcolor), 1.0);
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

		out_motion = vec4(a - b, 0.0f, 0.0f);
	}
}
