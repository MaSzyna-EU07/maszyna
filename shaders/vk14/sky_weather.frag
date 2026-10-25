#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
// laid over the sky after the dome, the stars, the sun and the moon (opengl46_weather::draw_sky): the clouds (clouds_march.comp)
// and the fog in front of them, which hides the sky and the clouds towards the horizon. blended with (1, source alpha): the colour is
// what the clouds and the fog add, the alpha what they let through of the sky behind.
// mode 0: no clouds, 1: the clouds of the main view (an image over the screen), 2: the panorama of the clouds around the camera

layout(location = 14) in vec2 f_coords;

#include "fog_volume.glsl"
#include "tonemapping.glsl"

// the clouds of the view or the panorama, the first texture of the draw
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 0) uniform sampler2D clouds;

// the clip space to the view space of the camera of the pass (pc.data[0-3]), the view space to world axes (the rotation of the
// block of the scene of the sky pass), the mode (pc.data[4].x)
#define inverse_projection mat4(pc.data[0], pc.data[1], pc.data[2], pc.data[3])
#define view_to_world inv_view
#define mode int(pc.data[4].x)

layout(location = 0) out vec4 out_color;
#ifdef NO_MOTION_OUTPUT
// a pass without the velocity target: the writes go nowhere
vec4 out_motion;
#else
layout(location = 1) out vec4 out_motion;
#endif

// as in clouds_march.comp
const float PANORAMA_LOW = -0.1;

void main()
{
	vec4 near = inverse_projection * vec4(f_coords * 2.0 - 1.0, 1.0, 1.0);
	vec3 direction = normalize(mat3(view_to_world) * normalize(near.xyz / near.w));
	vec4 cloud = vec4(0.0, 0.0, 0.0, 1.0);
	if (mode == 1)
		cloud = textureLod(clouds, f_coords, 0.0);
	else if (mode == 2)
	{
		float elevation = asin(clamp(direction.y, -1.0, 1.0));
		float v = sqrt(clamp((elevation - PANORAMA_LOW) / (0.5 * WEATHER_PI - PANORAMA_LOW), 0.0, 1.0));
		float u = atan(direction.x, direction.z) / (2.0 * WEATHER_PI);
		cloud = textureLod(clouds, vec2(fract(u), v), 0.0);
	}
	// the sky and the clouds are far: the fog all the way, without the haze the atmosphere has already (the clouds got it)
	vec4 fog = fog_sky(near.xyz / near.w, gl_FragCoord.xy);
	vec3 color = fog.rgb + fog.a * cloud.rgb;
	float through = fog.a * cloud.a;
	if (POSTFX_ENABLED)
	{
		out_color = vec4(color, through);
	}
	else
	{
		out_color = vec4(tonemap(vec4(color, 1.0)).rgb, through);
	}
	if (MOTIONBLUR_ENABLED)
	{
		out_motion = vec4(0.0);
	}
}
