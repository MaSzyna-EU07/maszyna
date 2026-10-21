// a track drawn with the textures of the tracks joined at its ends blended in near them (world/TrackBlend.h): the weights of the
// tracks joined at the ends p1, p2 and p4 of a switch in UserData.yzw, their textures in diffuse1..3, lit as mat_default.
// TRACKBLEND_GRAIN: the others come in as grains, as the stones of two ballasts would mix, rather than faded in
in vec3 f_normal;
in vec2 f_coord;
in vec4 f_pos;

in vec4 f_clip_pos;
in vec4 f_clip_future_pos;

#include <common>

#param (color, 0, 0, 4, diffuse)
#param (diffuse, 1, 0, 1, diffuse)
#param (specular, 1, 1, 1, specular)
#param (reflection, 1, 2, 1, zero)
#param (glossiness, 1, 3, 1, glossiness)

#texture (diffuse, 0, sRGB_A)
uniform sampler2D diffuse;
#texture (diffuse1, 1, sRGB_A)
uniform sampler2D diffuse1;
#texture (diffuse2, 2, sRGB_A)
uniform sampler2D diffuse2;
#texture (diffuse3, 3, sRGB_A)
uniform sampler2D diffuse3;

layout(location = 0) out vec4 out_color;
#if MOTIONBLUR_ENABLED
layout(location = 1) out vec4 out_motion;
#endif

#include <light_common.glsl>
#include <apply_fog.glsl>
#include <tonemapping.glsl>

#if TRACKBLEND_GRAIN
// grains per repeat of the texture: a ballast texture repeats every few metres, a grain is a handful of its stones
const float GRAIN_CELLS = 28.0;

vec2 grain_hash2(vec2 p)
{
	vec3 q = fract(vec3(p.xyx) * vec3(0.1031, 0.1030, 0.0973));
	q += dot(q, q.yzx + 33.33);
	return fract((q.xx + q.yz) * q.zy);
}

float grain_hash1(vec2 p)
{
	vec3 q = fract(vec3(p.xyx) * 0.1031);
	q += dot(q, q.yzx + 33.33);
	return fract((q.x + q.y) * q.z);
}

// the value of the grain the point lies in, 0..1: cells around points jittered on a grid, each of one value
float grain_value(vec2 p)
{
	vec2 cell = floor(p);
	vec2 inside = fract(p);
	float nearest = 8.0;
	float value = 0.0;
	for (int y = -1; y <= 1; ++y)
	{
		for (int x = -1; x <= 1; ++x)
		{
			vec2 offset = vec2(float(x), float(y));
			vec2 r = offset + grain_hash2(cell + offset) - inside;
			float d = dot(r, r);
			if (d < nearest)
			{
				nearest = d;
				value = grain_hash1(cell + offset + 17.31);
			}
		}
	}
	return value;
}
#endif

void main()
{
	vec3 weights = clamp(UserData.yzw, 0.0, 1.0);
	float total = weights.x + weights.y + weights.z;
	if (total > 1.0)
	{
		weights /= total;
		total = 1.0;
	}
	// the gradients are taken outside the branches the textures are sampled in
	vec2 dx = dFdx(f_coord);
	vec2 dy = dFdy(f_coord);
#if TRACKBLEND_GRAIN
	vec2 grainpos = f_coord * GRAIN_CELLS;
	// grains smaller than a pixel would shimmer, there the textures fade into each other instead
	float footprint = max(length(dx), length(dy)) * GRAIN_CELLS;
	float grained = 1.0 - smoothstep(0.35, 0.9, footprint);
#endif
	vec4 tex_color = textureGrad(diffuse, f_coord, dx, dy);
	if (total > 0.0)
	{
#if TRACKBLEND_GRAIN
		// the grain is of the track joined at the end whose share its value falls in, of the track itself past them
		float pick = grain_value(grainpos);
		float first = weights.x;
		float second = weights.x + weights.y;
		vec3 grain = vec3(float(pick < first), float(pick >= first && pick < second), float(pick >= second && pick < total));
		weights = mix(weights, grain, grained);
		total = weights.x + weights.y + weights.z;
#endif
		vec4 blended = tex_color * (1.0 - total);
		if (weights.x > 0.0)
			blended += textureGrad(diffuse1, f_coord, dx, dy) * weights.x;
		if (weights.y > 0.0)
			blended += textureGrad(diffuse2, f_coord, dx, dy) * weights.y;
		if (weights.z > 0.0)
			blended += textureGrad(diffuse3, f_coord, dx, dy) * weights.z;
		tex_color = blended;
	}

	bool alphatestfail = ( opacity >= 0.0 ? (tex_color.a < opacity) : (tex_color.a >= -opacity) );
	if(alphatestfail)
		discard;

	vec3 fragcolor = ambient;
	vec3 fragnormal = normalize(f_normal);
	matte = true;

	fragcolor = apply_lights(fragcolor, fragnormal, tex_color.rgb, 0.0, 0.0, shadow_tone);
	vec4 color = vec4(apply_fog(fragcolor), tex_color.a * alpha_mult);
#if POSTFX_ENABLED
    out_color = color;
#else
    out_color = tonemap(color);
#endif

#if MOTIONBLUR_ENABLED
	{
        vec2 a = (f_clip_future_pos.xy / f_clip_future_pos.w) * 0.5 + 0.5;
        vec2 b = (f_clip_pos.xy / f_clip_pos.w) * 0.5 + 0.5;

        out_motion = vec4(a - b, 0.0f, tex_color.a * alpha_mult);
	}
#endif
}
