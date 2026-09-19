in vec3 f_color;
in vec4 f_pos;

#include <common>
#include <apply_fog.glsl>
#include <tonemapping.glsl>

layout(location = 0) out vec4 out_color;
#if MOTIONBLUR_ENABLED
layout(location = 1) out vec4 out_motion;
#endif

// the skydome vertex colours carry Perez luminance scaled by a fixed exposure
// (see CSkyDome::SetExposure), so the circumsolar region and the horizon band
// routinely land well above 1.0. hard-clipping them, as this shader used to do,
// flattened everything within ~20 degrees of the sun - and most of the lower sky -
// into one solid white patch. compress the top end instead: below the knee the
// response is unchanged, above it it rolls off towards SKY_SHOULDER_WHITE, so the
// brightness gradient around the sun survives and the patch keeps a soft edge.
const float SKY_SHOULDER_KNEE = 0.25;
const float SKY_SHOULDER_WHITE = 0.95;

vec3 sky_shoulder(vec3 color)
{
	const float range = SKY_SHOULDER_WHITE - SKY_SHOULDER_KNEE;
	vec3 excess = max(color - vec3(SKY_SHOULDER_KNEE), vec3(0.0));
	return min(color, vec3(SKY_SHOULDER_KNEE)) + range * (vec3(1.0) - exp(-excess / range));
}

void main()
{
	vec3 col = sky_shoulder(pow(max(f_color.rgb, vec3(0.0)), vec3(2.2)));
#if POSTFX_ENABLED
	out_color = vec4(apply_fog(col), 1.0f);
#else
    out_color = tonemap(vec4(apply_fog(col), 1.0f));
#endif
#if MOTIONBLUR_ENABLED
	out_motion = vec4(0.0f);
#endif
}
