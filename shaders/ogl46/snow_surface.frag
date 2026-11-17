#version 460 core
#define STAGE_FRAGMENT 1
// the surface of the drifts (opengl46_snow, snow_surface.vert): snow all over, lit like the rest of the scene. where it gets thin
// it gives way in patches to the surface under it, which the shading covers with the same snow

layout(location = 0) in vec3 f_normal;
layout(location = 3) in vec4 f_pos;
layout(location = 4) in vec4 f_clip_pos;
layout(location = 5) in vec4 f_clip_future_pos;
layout(location = 12) in vec4 f_snow;

#include "common.glsl"

layout(location = 0) out vec4 out_color;
layout(location = 1) out vec4 out_motion;

#include "light_common.glsl"
#include "apply_fog.glsl"
#include "tonemapping.glsl"

void main()
{
	mat3 toworld = mat3(inv_view);
	vec3 world = toworld * f_pos.xyz;
	vec3 stable = vec3(world.x + sky_occlusion_origin.x, world.y + sky_occlusion_origin.z, world.z + sky_occlusion_origin.y);
	// the thin edges of the drifts break up into patches
	float depth = f_snow.x;
	float edge = snow_fbm(stable.xz * 1.3);
	if (depth < 0.05 + 0.08 * edge)
		discard;

	float exposure = f_snow.y;
	float tracked = 0.0;
	if (exposure > SNOW_TRACKED * 0.5)
	{
		tracked = 1.0;
		exposure -= SNOW_TRACKED;
	}
	vec3 worldnormal = normalize(toworld * normalize(f_normal));
	vec3 texturecolor = vec3(1.0);
	float reflectivity = 0.0;
	float specularity = 0.3;
	glossiness = snow_glossiness(0.6);
	metalic = 0.0;
	float pixel = length(fwidth(world));
	snow_material(1.0, stable, exposure, tracked, pixel, worldnormal, texturecolor, reflectivity, specularity);
	vec3 fragnormal = normalize(transpose(toworld) * worldnormal);
	// the snow is put on already
	snow_receiver = false;
	vec3 fragcolor = apply_lights(ambient, fragnormal, texturecolor, reflectivity, specularity, shadow_tone);

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
