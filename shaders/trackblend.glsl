// a track drawn with the textures of the tracks joined at its ends blended in near them (world/TrackBlend.h): the weights of the
// tracks joined at the ends p1, p2 and p4 of a switch in UserData.yzw, their textures in diffuse1..3, normalmap1..3. the
// including shader declares the textures and sets:
//   TRACKBLEND_GRAIN      the others come in as grains, as the stones of two ballasts would mix, rather than faded in
//   TRACKBLEND_NORMALMAP  with the normal maps of the four, lit as the shader of the material of the track: param[2].x 0 as
//                         mat_default, 1 normalmap, 2 sunlessnormalmap, 3 shadowlessnormalmap
//   TRACKBLEND_SPECGLOSS  the specgloss map of the track itself as well, the textures of the ends p1 and p2 alone (no room
//                         for a third set of them)
in vec3 f_normal;
in vec2 f_coord;
in vec4 f_pos;
#ifdef TRACKBLEND_NORMALMAP
in mat3 f_tbn;
#endif

in vec4 f_clip_pos;
in vec4 f_clip_future_pos;

#include <common>

#param (color, 0, 0, 4, diffuse)
#param (diffuse, 1, 0, 1, diffuse)
#param (specular, 1, 1, 1, specular)
#param (reflection, 1, 2, 1, zero)
#param (glossiness, 1, 3, 1, glossiness)
#param (blendlighting, 2, 0, 1, zero)

layout(location = 0) out vec4 out_color;
#if MOTIONBLUR_ENABLED
layout(location = 1) out vec4 out_motion;
#endif

#include <light_common.glsl>
#include <apply_fog.glsl>
#include <tonemapping.glsl>

#ifdef TRACKBLEND_GRAIN
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
#ifdef TRACKBLEND_SPECGLOSS
	vec3 weights = clamp(vec3(UserData.yz, 0.0), 0.0, 1.0);
#else
	vec3 weights = clamp(UserData.yzw, 0.0, 1.0);
#endif
	float total = weights.x + weights.y + weights.z;
	if (total > 1.0)
	{
		weights /= total;
		total = 1.0;
	}
	// the gradients are taken outside the branches the textures are sampled in
	vec2 dx = dFdx(f_coord);
	vec2 dy = dFdy(f_coord);
#ifdef TRACKBLEND_GRAIN
	vec2 grainpos = f_coord * GRAIN_CELLS;
	// grains smaller than a pixel would shimmer, there the textures fade into each other instead
	float footprint = max(length(dx), length(dy)) * GRAIN_CELLS;
	float grained = 1.0 - smoothstep(0.35, 0.9, footprint);
#endif
	vec4 tex_color = textureGrad(diffuse, f_coord, dx, dy);
#ifdef TRACKBLEND_NORMALMAP
	vec4 tex_normal = textureGrad(normalmap, f_coord, dx, dy);
#endif
	if (total > 0.0)
	{
#ifdef TRACKBLEND_GRAIN
		// the grain is of the track joined at the end whose share its value falls in, of the track itself past them
		float pick = grain_value(grainpos);
		float first = weights.x;
		float second = weights.x + weights.y;
		vec3 grain = vec3(float(pick < first), float(pick >= first && pick < second), float(pick >= second && pick < total));
		weights = mix(weights, grain, grained);
		total = weights.x + weights.y + weights.z;
#endif
		vec4 blended = tex_color * (1.0 - total);
#ifdef TRACKBLEND_NORMALMAP
		vec4 blendednormal = tex_normal * (1.0 - total);
#endif
		if (weights.x > 0.0)
		{
			blended += textureGrad(diffuse1, f_coord, dx, dy) * weights.x;
#ifdef TRACKBLEND_NORMALMAP
			blendednormal += textureGrad(normalmap1, f_coord, dx, dy) * weights.x;
#endif
		}
		if (weights.y > 0.0)
		{
			blended += textureGrad(diffuse2, f_coord, dx, dy) * weights.y;
#ifdef TRACKBLEND_NORMALMAP
			blendednormal += textureGrad(normalmap2, f_coord, dx, dy) * weights.y;
#endif
		}
#ifndef TRACKBLEND_SPECGLOSS
		if (weights.z > 0.0)
		{
			blended += textureGrad(diffuse3, f_coord, dx, dy) * weights.z;
#ifdef TRACKBLEND_NORMALMAP
			blendednormal += textureGrad(normalmap3, f_coord, dx, dy) * weights.z;
#endif
		}
#endif
		tex_color = blended;
#ifdef TRACKBLEND_NORMALMAP
		tex_normal = blendednormal;
#endif
	}

	bool alphatestfail = ( opacity >= 0.0 ? (tex_color.a < opacity) : (tex_color.a >= -opacity) );
	if(alphatestfail)
		discard;

	vec3 fragcolor = ambient;
#ifdef TRACKBLEND_NORMALMAP
	vec3 normal;
	normal.xy = (tex_normal.rg * 2.0 - 1.0);
	normal.z = sqrt(1.0 - clamp((dot(normal.xy, normal.xy)), 0.0, 1.0));
	vec3 fragnormal = normalize(f_tbn * normalize(normal.xyz));
	float reflectivity = param[1].z * tex_normal.a;
#ifdef TRACKBLEND_SPECGLOSS
	vec4 tex_specgloss = textureGrad(specgloss, f_coord, dx, dy);
	float specularity = tex_specgloss.r;
	glossiness = tex_specgloss.g * abs(param[1].w);
	metalic = tex_specgloss.b;
#else
	float specularity = (tex_color.r + tex_color.g + tex_color.b) * 0.5;
	legacy_gloss();
#endif
	int lighting = int(param[2].x + 0.5);
	if (lighting == 2)
		fragcolor = apply_lights_sunless(fragcolor, fragnormal, tex_color.rgb, reflectivity, specularity, shadow_tone);
	else
		fragcolor = apply_lights(fragcolor, fragnormal, tex_color.rgb, reflectivity, specularity, (lighting == 3 ? 1.0 : shadow_tone));
#else
	vec3 fragnormal = normalize(f_normal);
	matte = true;

	fragcolor = apply_lights(fragcolor, fragnormal, tex_color.rgb, 0.0, 0.0, shadow_tone);
#endif
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
