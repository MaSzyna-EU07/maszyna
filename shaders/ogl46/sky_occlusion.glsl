// the occlusion of the sky and of the sun by the scenery around the camera (gfx.skyvisibility, gfx::sky_occlusion, the track T):
// toroidal maps of the square around the camera, read texel by texel. SKY_OCCLUSION_HEIGHT_TEX holds the height of the top of the
// scenery over a texel (the highest surface, world space), SKY_OCCLUSION_TEX for the texel: x the share of the sky a surface facing
// up sees above the top, y the same under the ceiling (the vault of a tunnel, the deck of a bridge), z the height above the top over
// which the key light (the sun, the moon) comes, w the height above the top under which it comes in under the ceilings.
// the same as gfx::sky_occlusion::sample() of rendering/common/skyocclusion.cpp, which the cpu works out the light around the
// vehicles with: a change here goes there as well
#ifndef SKY_OCCLUSION_GLSL
#define SKY_OCCLUSION_GLSL

layout(binding = SKY_OCCLUSION_HEIGHT_TEX) uniform sampler2D sky_occlusion_height;
layout(binding = SKY_OCCLUSION_TEX) uniform sampler2D sky_occlusion_map;

// a point this far below the top of a texel is under it, fully this much further; half the height over which the key light comes
// in at the edge of its shadow
const float SKY_OCCLUSION_COVER_START = 0.75;
const float SKY_OCCLUSION_COVER_SPAN = 1.0;
const float SKY_OCCLUSION_SUN_SOFTNESS = 0.6;

// the share of the sky (x) and of the key light (y) a point gets, 0..1. World: its offset from the camera along the world axes,
// Worldnormal: the normal of its surface in the world space (zero for a point in the air), the texels are read a little in front of
// the surface (a wall reads the ground in front of it, not the roof above it)
vec2 sky_occlusion_world(vec3 world, vec3 worldnormal)
{
	if (sky_occlusion_origin.w <= 0.0)
		return vec2(1.0);
	float fade = clamp((max(abs(world.x - sky_occlusion_center.x), abs(world.z - sky_occlusion_center.y)) - sky_occlusion_params.z) / max(sky_occlusion_params.w, 1e-3), 0.0, 1.0);
	if (fade >= 1.0)
		return vec2(1.0);
	float texel = sky_occlusion_params.x / sky_occlusion_params.y;
	vec3 point = world + worldnormal * (0.75 * texel);
	vec2 coords = (point.xz + sky_occlusion_origin.xy) / texel - 0.5;
	vec2 base = floor(coords);
	vec2 f = coords - base;
	vec2 uv = (base + 1.0) / sky_occlusion_params.y;
	// the four texels around the point, each with its own top: under one of them a point gets what is under it, above it what is
	// above, and the four are blended as bilinear filtering would
	vec4 tops = textureGather(sky_occlusion_height, uv, 0);
	vec4 skytop = textureGather(sky_occlusion_map, uv, 0);
	vec4 skyunder = textureGather(sky_occlusion_map, uv, 1);
	vec4 litabove = textureGather(sky_occlusion_map, uv, 2);
	vec4 litbelow = textureGather(sky_occlusion_map, uv, 3);
	float height = point.y + sky_occlusion_origin.z;
	vec4 cover = clamp((tops - vec4(height) - SKY_OCCLUSION_COVER_START) / SKY_OCCLUSION_COVER_SPAN, 0.0, 1.0);
	vec4 sky = mix(skytop, skyunder, cover);
	vec4 suntop = smoothstep(vec4(-SKY_OCCLUSION_SUN_SOFTNESS), vec4(SKY_OCCLUSION_SUN_SOFTNESS), vec4(height) - (tops + litabove));
	vec4 sununder = smoothstep(vec4(-SKY_OCCLUSION_SUN_SOFTNESS), vec4(SKY_OCCLUSION_SUN_SOFTNESS), (tops + litbelow) - vec4(height));
	vec4 sun = mix(suntop, sununder, cover);
	// the order of the texels of a gather: (0, 1), (1, 1), (1, 0), (0, 0)
	vec4 weights = vec4((1.0 - f.x) * f.y, f.x * f.y, f.x * (1.0 - f.y), (1.0 - f.x) * (1.0 - f.y));
	return mix(vec2(dot(sky, weights), dot(sun, weights)), vec2(1.0), fade);
}

// how much of a point in the air is under the scenery (0 in the open, 1 under a roof, a bridge, in a tunnel), for what falls from
// the sky. World: its offset from the camera along the world axes
float sky_occlusion_cover(vec3 world)
{
	if (sky_occlusion_origin.w <= 0.0)
		return 0.0;
	float fade = clamp((max(abs(world.x - sky_occlusion_center.x), abs(world.z - sky_occlusion_center.y)) - sky_occlusion_params.z) / max(sky_occlusion_params.w, 1e-3), 0.0, 1.0);
	if (fade >= 1.0)
		return 0.0;
	float texel = sky_occlusion_params.x / sky_occlusion_params.y;
	vec2 coords = (world.xz + sky_occlusion_origin.xy) / texel - 0.5;
	vec2 base = floor(coords);
	vec2 f = coords - base;
	vec4 tops = textureGather(sky_occlusion_height, (base + 1.0) / sky_occlusion_params.y, 0);
	vec4 cover = clamp((tops - vec4(world.y + sky_occlusion_origin.z) - SKY_OCCLUSION_COVER_START) / SKY_OCCLUSION_COVER_SPAN, 0.0, 1.0);
	vec4 weights = vec4((1.0 - f.x) * f.y, f.x * f.y, f.x * (1.0 - f.y), (1.0 - f.x) * (1.0 - f.y));
	return dot(cover, weights) * (1.0 - fade);
}

#endif
