// the occlusion of the sky and of the sun by the scenery around the camera (gfx.skyvisibility, gfx::sky_occlusion, the track T):
// toroidal maps of the square around the camera, read texel by texel. SKY_OCCLUSION_HEIGHT_TEX holds for a texel, world space: x the
// height of the top of the scenery (the highest surface), y of the ceiling (the lowest surface facing down: the vault of a tunnel, the
// deck of a bridge; 1e4 for none), z the height under the ceiling below which the key light (the sun, the moon) comes in, w the height
// the deep share of the sky under the ceiling was worked out at (a metre above the ground around). SKY_OCCLUSION_TEX: x the share of
// the sky above the top, y under the ceiling near it, z deep under it, w the height above the top over which the key light comes.
// the same as gfx::sky_occlusion::sample() of rendering/common/skyocclusion.cpp, which the cpu works out the light around the
// vehicles with: a change here goes there as well
#ifndef SKY_OCCLUSION_GLSL
#define SKY_OCCLUSION_GLSL

layout(binding = SKY_OCCLUSION_HEIGHT_TEX) uniform sampler2D sky_occlusion_height;
layout(binding = SKY_OCCLUSION_TEX) uniform sampler2D sky_occlusion_map;

// a point this far below the top of a texel is under it, fully this much further; how far below the ceiling the near share under
// it was worked out; half the height over which the key light comes in at the edge of its shadow
const float SKY_OCCLUSION_COVER_START = 0.75;
const float SKY_OCCLUSION_COVER_SPAN = 1.0;
const float SKY_OCCLUSION_BELOW_CEILING = 2.5;
const float SKY_OCCLUSION_SUN_SOFTNESS = 0.6;

// the coordinates of the gather of the four texels around a world point (offset from the camera) and the bilinear weights of
// the texels in the order of a gather: (0, 1), (1, 1), (1, 0), (0, 0). false out of the square (Fade 1)
bool sky_occlusion_texels(vec2 world, out vec2 uv, out vec4 weights, out float fade)
{
	fade = clamp((max(abs(world.x - sky_occlusion_center.x), abs(world.y - sky_occlusion_center.y)) - sky_occlusion_params.z) / max(sky_occlusion_params.w, 1e-3), 0.0, 1.0);
	float texel = sky_occlusion_params.x / sky_occlusion_params.y;
	vec2 coords = (world + sky_occlusion_origin.xy) / texel - 0.5;
	vec2 base = floor(coords);
	vec2 f = coords - base;
	uv = (base + 1.0) / sky_occlusion_params.y;
	weights = vec4((1.0 - f.x) * f.y, f.x * f.y, f.x * (1.0 - f.y), (1.0 - f.x) * (1.0 - f.y));
	return fade < 1.0;
}

// the share of the sky (x) and of the key light (y) a point gets, 0..1. World: its offset from the camera along the world axes,
// Worldnormal: the normal of its surface in the world space (zero for a point in the air), the texels are read a little in front of
// the surface (a wall reads the ground in front of it, not the roof above it)
vec2 sky_occlusion_world(vec3 world, vec3 worldnormal)
{
	if (sky_occlusion_origin.w <= 0.0)
		return vec2(1.0);
	vec3 point = world + worldnormal * (0.75 * sky_occlusion_params.x / sky_occlusion_params.y);
	vec2 uv;
	vec4 weights;
	float fade;
	if (false == sky_occlusion_texels(point.xz, uv, weights, fade))
		return vec2(1.0);
	// the four texels around the point, each with its own heights: under the top of one a point gets what is under it (between the
	// near and the deep share, by its height), above it what is above, and the four are blended as bilinear filtering would
	vec4 tops = textureGather(sky_occlusion_height, uv, 0);
	vec4 ceilings = textureGather(sky_occlusion_height, uv, 1);
	vec4 litbelow = textureGather(sky_occlusion_height, uv, 2);
	vec4 deeps = textureGather(sky_occlusion_height, uv, 3);
	vec4 skytop = textureGather(sky_occlusion_map, uv, 0);
	vec4 skynear = textureGather(sky_occlusion_map, uv, 1);
	vec4 skydeep = textureGather(sky_occlusion_map, uv, 2);
	vec4 litabove = textureGather(sky_occlusion_map, uv, 3);
	vec4 height = vec4(point.y + sky_occlusion_origin.z);
	vec4 cover = clamp((tops - height - SKY_OCCLUSION_COVER_START) / SKY_OCCLUSION_COVER_SPAN, 0.0, 1.0);
	vec4 nearheight = ceilings - SKY_OCCLUSION_BELOW_CEILING;
	vec4 skyunder = mix(skydeep, skynear, clamp((height - deeps) / max(nearheight - deeps, vec4(1e-3)), 0.0, 1.0));
	// no ceiling: the inside of what is solid
	skyunder *= step(ceilings, vec4(5000.0));
	vec4 sky = mix(skytop, skyunder, cover);
	vec4 suntop = smoothstep(vec4(-SKY_OCCLUSION_SUN_SOFTNESS), vec4(SKY_OCCLUSION_SUN_SOFTNESS), height - (tops + litabove));
	vec4 sununder = smoothstep(vec4(-SKY_OCCLUSION_SUN_SOFTNESS), vec4(SKY_OCCLUSION_SUN_SOFTNESS), litbelow - height);
	vec4 sun = mix(suntop, sununder, cover);
	return mix(vec2(dot(sky, weights), dot(sun, weights)), vec2(1.0), fade);
}

// how much of a point in the air is under the scenery (0 in the open, 1 under a roof, a bridge, in a tunnel), for what falls from
// the sky. World: its offset from the camera along the world axes
float sky_occlusion_cover(vec3 world)
{
	if (sky_occlusion_origin.w <= 0.0)
		return 0.0;
	vec2 uv;
	vec4 weights;
	float fade;
	if (false == sky_occlusion_texels(world.xz, uv, weights, fade))
		return 0.0;
	vec4 tops = textureGather(sky_occlusion_height, uv, 0);
	vec4 cover = clamp((tops - vec4(world.y + sky_occlusion_origin.z) - SKY_OCCLUSION_COVER_START) / SKY_OCCLUSION_COVER_SPAN, 0.0, 1.0);
	return dot(cover, weights) * (1.0 - fade);
}

#endif
