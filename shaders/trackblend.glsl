// a track drawn with the textures of the tracks joined at its ends blended in near them (world/TrackBlend.h): the weights of the
// tracks joined at the ends p1, p2 and p4 of a switch in UserData.yzw, their textures in diffuse1..3, normalmap1..3, faded into
// the texture of the track. the including shader declares the textures and sets:
//   TRACKBLEND_NORMALMAP  with the normal maps of the four, lit as the shader of the material of the track: param[2].x 0 as
//                         mat_default, 1 normalmap, 2 sunlessnormalmap, 3 shadowlessnormalmap
//   TRACKBLEND_SPECGLOSS  the specgloss map of the track itself as well, the textures of the ends p1 and p2 alone (no room
//                         for a third set of them)
//   TRACKBLEND_PARALLAX   with the normal maps, as mat_parallax: each texture shifted by the heights in its own normal map
in vec3 f_normal;
in vec2 f_coord;
in vec4 f_pos;
#ifdef TRACKBLEND_NORMALMAP
in mat3 f_tbn;
#endif
#ifdef TRACKBLEND_PARALLAX
in vec3 TangentFragPos;
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

#ifdef TRACKBLEND_PARALLAX
// the texture coordinates shifted by the heights (b) of the normal map, as ParallaxMapping() of mat_parallax.frag
vec2 parallax_coord(sampler2D heights, vec2 coord, vec3 viewdir, vec2 dcoorddx, vec2 dcoorddy)
{
	float pos_len = length(f_pos.xyz);
	if (pos_len > 100.0)
		return coord;
	float fade = 1.0 - smoothstep(80.0, 100.0, pos_len);
#if EXTRAEFFECTS_ENABLED
	const float minLayers = 8.0;
	const float maxLayers = 32.0;
	float LayersWeight = pos_len / 20.0;
	vec2 currentTexCoords = coord;
	float currentDepthMapValue = textureGrad(heights, currentTexCoords, dcoorddx, dcoorddy).b;
	LayersWeight = min(abs(dot(vec3(0.0, 0.0, 1.0), viewdir)), LayersWeight);
	float numLayers = mix(maxLayers, minLayers, clamp(LayersWeight, 0.0, 1.0));
	float layerDepth = 1.0 / numLayers;
	float currentLayerDepth = 0.0;
	vec2 P = viewdir.xy * param[2].y * fade;
	vec2 deltaTexCoords = P / numLayers;
	while (currentLayerDepth < currentDepthMapValue)
	{
		currentTexCoords -= deltaTexCoords;
		currentDepthMapValue = textureGrad(heights, currentTexCoords, dcoorddx, dcoorddy).b;
		currentLayerDepth += layerDepth;
	}
	vec2 prevTexCoords = currentTexCoords + deltaTexCoords;
	float afterDepth = currentDepthMapValue - currentLayerDepth;
	float beforeDepth = textureGrad(heights, prevTexCoords, dcoorddx, dcoorddy).b - currentLayerDepth + layerDepth;
	float weight = afterDepth / (afterDepth - beforeDepth);
	return prevTexCoords * weight + currentTexCoords * (1.0 - weight);
#else
	float height = textureGrad(heights, coord, dcoorddx, dcoorddy).b;
	vec2 p = viewdir.xy / viewdir.z * (height * (param[2].y - param[2].z) * 0.2) * fade;
	return coord - p;
#endif
}
// each texture at the coordinates shifted by the heights of its own normal map
#define TRACKBLEND_COORD(heights) parallax_coord(heights, f_coord, viewdir, dx, dy)
#else
#define TRACKBLEND_COORD(heights) f_coord
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
#ifdef TRACKBLEND_PARALLAX
	vec3 viewdir = normalize(vec3(0.0) - TangentFragPos);
#endif
	vec2 coord = TRACKBLEND_COORD(normalmap);
	vec4 tex_color = textureGrad(diffuse, coord, dx, dy);
#ifdef TRACKBLEND_NORMALMAP
	vec4 tex_normal = textureGrad(normalmap, coord, dx, dy);
#endif
	if (total > 0.0)
	{
		vec4 blended = tex_color * (1.0 - total);
#ifdef TRACKBLEND_NORMALMAP
		vec4 blendednormal = tex_normal * (1.0 - total);
#endif
		if (weights.x > 0.0)
		{
			vec2 coord1 = TRACKBLEND_COORD(normalmap1);
			blended += textureGrad(diffuse1, coord1, dx, dy) * weights.x;
#ifdef TRACKBLEND_NORMALMAP
			blendednormal += textureGrad(normalmap1, coord1, dx, dy) * weights.x;
#endif
		}
		if (weights.y > 0.0)
		{
			vec2 coord2 = TRACKBLEND_COORD(normalmap2);
			blended += textureGrad(diffuse2, coord2, dx, dy) * weights.y;
#ifdef TRACKBLEND_NORMALMAP
			blendednormal += textureGrad(normalmap2, coord2, dx, dy) * weights.y;
#endif
		}
#ifndef TRACKBLEND_SPECGLOSS
		if (weights.z > 0.0)
		{
			vec2 coord3 = TRACKBLEND_COORD(normalmap3);
			blended += textureGrad(diffuse3, coord3, dx, dy) * weights.z;
#ifdef TRACKBLEND_NORMALMAP
			blendednormal += textureGrad(normalmap3, coord3, dx, dy) * weights.z;
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
	vec4 tex_specgloss = textureGrad(specgloss, coord, dx, dy);
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
