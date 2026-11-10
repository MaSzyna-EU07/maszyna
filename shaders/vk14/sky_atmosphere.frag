#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
// the sky dome drawn with the light of the physical atmosphere (atmosphere_common.glsl): the sky-view table looked up in
// the direction of each fragment. the colour of the old dome (vertex colours, see color.frag) is kept for the night,
// which the atmosphere does not cover (no moon, no stars), and the two are blended by the height of the sun.
// param[0].x: share of the atmosphere, param[0].y: brightness of the atmosphere, param[0].z: how much the overcast flattens
// the sky, param[0].w: luminance the flattened sky tends to. param[1].rgb: white balance of the atmosphere

layout(location = 13) in vec3 f_color;
layout(location = 3) in vec4 f_pos;

#include "atmosphere_common.glsl"
#include "apply_fog.glsl"
#include "tonemapping.glsl"
#include "sky_shoulder.glsl"

layout(set = PASS_SET, binding = SKY_VIEW_TEX) uniform sampler2D sky_view_lut;

layout(location = 0) out vec4 out_color;
#ifdef NO_MOTION_OUTPUT
// a pass without the velocity target: the writes go nowhere
vec4 out_motion;
#else
layout(location = 1) out vec4 out_motion;
#endif

// the brightest the sky gets, a guard against the circumsolar peak of the mie scattering blowing up the bloom
const float SKY_RADIANCE_MAX = 64.0;

void main()
{
	vec3 domecolor = sky_shoulder(pow(max(f_color.rgb, vec3(0.0)), vec3(2.2)));

	// the dome is drawn around the camera with the rotation of the view only, so the view space position is the direction
	vec3 direction = normalize(mat3(inv_view) * f_pos.xyz);
	// the bottom of the dome reaches a little under the horizon, where the table holds the dark air in front of the ground;
	// it gets the light of the horizon instead, as the old dome did
	direction = normalize(vec3(direction.x, max(direction.y, 0.002), direction.z));
	vec3 sundirection = normalize(atmo_sun_direction.xyz);
	vec3 atmosphere = textureLod(sky_view_lut, atmo_skyview_uv_world(direction, sundirection), 0.0).rgb;
	atmosphere *= param[1].rgb * param[0].y;
	// an overcast sky is greyer and more even than the clear one the atmosphere describes
	vec3 overcast = vec3(param[0].w) * (0.75 + 0.5 * clamp(direction.y, 0.0, 1.0));
	atmosphere = mix(atmosphere, overcast, param[0].z);
	atmosphere = min(atmosphere, vec3(SKY_RADIANCE_MAX));

	vec3 col = mix(domecolor, atmosphere, param[0].x);
	if (any(isnan(col)) || any(isinf(col)))
		col = domecolor;
	if (POSTFX_ENABLED)
	{
		out_color = vec4(apply_fog(col), 1.0f);
	}
	else
	{
		out_color = tonemap(vec4(apply_fog(col), 1.0f));
	}
	if (MOTIONBLUR_ENABLED)
	{
		out_motion = vec4(0.0f);
	}
}
