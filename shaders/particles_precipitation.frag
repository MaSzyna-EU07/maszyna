// rain and snow of the gpu (opengl46_gpu_particles, particles_precipitation.vert): a streak rounded at its ends, fainter towards
// its tail, in the colour the compute pass lit it with, faded in front of the scene, fogged

in vec4 f_pos;
in vec4 f_color;
in vec4 f_streak;
in float f_soft;

in vec4 f_clip_pos;
in vec4 f_clip_future_pos;

#include <common>
#include <apply_fog.glsl>
#include <tonemapping.glsl>
#include <particle_soft.glsl>

layout(location = 0) out vec4 out_color;
#if MOTIONBLUR_ENABLED
layout(location = 1) out vec4 out_motion;
#endif

void main()
{
	// distance from the axis of the streak, in its radii
	float along = clamp(f_streak.y, 0.0, f_streak.w);
	float offaxis = length(vec2(f_streak.x, f_streak.y - along)) / max(f_streak.z, 1e-6);
	float alpha = f_color.a * (1.0 - smoothstep(0.35, 1.0, offaxis));
	alpha *= mix(1.0, 0.4, along / max(f_streak.w, 1e-6));
	alpha *= particle_soft_fade(f_soft, -f_pos.z);
	if (alpha <= 0.0)
		discard;
	vec4 color = vec4(apply_fog(f_color.rgb), alpha);
#if POSTFX_ENABLED
	out_color = color;
#else
	out_color = tonemap(color);
#endif
#if MOTIONBLUR_ENABLED
	{
		vec2 a = (f_clip_future_pos.xy / f_clip_future_pos.w) * 0.5 + 0.5;
		vec2 b = (f_clip_pos.xy / f_clip_pos.w) * 0.5 + 0.5;

		out_motion = vec4(a - b, 0.0f, alpha * alpha_mult);
	}
#endif
}
