// the snow lying on the scenery (environment/snowcover.h, opengl46_snow): the cells of the drifts around the camera, which
// snow_drift.comp and snow_settle.comp work out from the surface of the scenery seen from above (gfx::sky_occlusion), and the snow
// put on every surface the shading lights (light_common.glsl, shade_surface()) and on the surface of the drifts (snow_surface.*).
// a cell for each texel of the maps of the occlusion of the sky, toroidal as they are: x the world height of the surface of the
// snow (SNOW_NONE: nothing known there), y the depth of the snow, z the world height of the ceiling over it (SNOW_NO_CEILING:
// none), w how much it is blown off (-1) or blown in (1), 4 more over a track or a road which keeps it down; z is then the height
// it keeps the snow below (3 cm under the top of the rails, 3 cm over a road)
// needs common.glsl (the block of the scene) and weather_common.glsl (the block of the weather)
#ifndef SNOW_GLSL
#define SNOW_GLSL

const float SNOW_NONE = -5000.0;
const float SNOW_NO_CEILING = 5000.0;
const float SNOW_TRACKED = 4.0;
// m a surface may stand over the height a track or a road keeps the snow below and still be kept down by it (snow_drift.comp,
// snow_settle.comp): higher than that is something else over its width, a platform, a wall, which holds its snow
const float SNOW_CAP_OVER = 0.3;

#ifdef SNOW_WRITE
layout(std430, binding = SNOW_SSBO_BINDING) writeonly buffer snow_ssbo
#else
layout(std430, binding = SNOW_SSBO_BINDING) readonly buffer snow_ssbo
#endif
{
	vec4 snow_cells[];
};

// the hash and the value noise the snow is broken up with
float snow_hash(vec2 p)
{
	vec3 q = fract(vec3(p.xyx) * 0.1031);
	q += dot(q, q.yzx + 33.33);
	return fract((q.x + q.y) * q.z);
}

float snow_hash3(vec3 p)
{
	p = fract(p * 0.1031);
	p += dot(p, p.zyx + 31.32);
	return fract((p.x + p.y) * p.z);
}

float snow_noise(vec2 p)
{
	vec2 i = floor(p);
	vec2 f = fract(p);
	vec2 u = f * f * (3.0 - 2.0 * f);
	float a = snow_hash(i);
	float b = snow_hash(i + vec2(1.0, 0.0));
	float c = snow_hash(i + vec2(0.0, 1.0));
	float d = snow_hash(i + vec2(1.0, 1.0));
	return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

// two octaves, 0..1
float snow_fbm(vec2 p)
{
	return snow_noise(p) * 0.65 + snow_noise(p * 2.03 + vec2(17.1, 5.3)) * 0.35;
}

#ifndef SNOW_WRITE

// the snow lies on the scenery at all
bool snow_present()
{
	return (snow_cover.x > 0.0005) || (snow_wind.w > 0.01);
}

// the maps of the drifts are there (they follow the maps of the occlusion of the sky)
bool snow_maps()
{
	return (sky_occlusion_origin.w > 0.0) && (sky_occlusion_params.y > 0.5);
}

// the place of a point of the world in the maps: a world point (relative to the camera of the pass) is stable in Stable, the four
// cells around it in Cells and their bilinear weights in Weights; Fade 1 where the maps end. false without the maps
bool snow_place(vec2 world, out vec2 stable, out ivec4 cells, out vec4 weights, out float fade)
{
	stable = world + sky_occlusion_origin.xy;
	cells = ivec4(0);
	weights = vec4(0.0);
	fade = 1.0;
	if (false == snow_maps())
		return false;
	fade = clamp((max(abs(world.x - sky_occlusion_center.x), abs(world.y - sky_occlusion_center.y)) - sky_occlusion_params.z) / max(sky_occlusion_params.w, 1e-3), 0.0, 1.0);
	int side = int(sky_occlusion_params.y + 0.5);
	float texel = sky_occlusion_params.x / sky_occlusion_params.y;
	vec2 coords = stable / texel - 0.5;
	vec2 base = floor(coords);
	vec2 f = coords - base;
	ivec2 b = ivec2(base);
	ivec2 x = ivec2((b.x % side + side) % side, ((b.x + 1) % side + side) % side);
	ivec2 z = ivec2((b.y % side + side) % side, ((b.y + 1) % side + side) % side);
	cells = ivec4(z.x * side + x.x, z.x * side + x.y, z.y * side + x.x, z.y * side + x.y);
	weights = vec4((1.0 - f.x) * (1.0 - f.y), f.x * (1.0 - f.y), (1.0 - f.x) * f.y, f.x * f.y);
	return fade < 1.0;
}

// how much a depth of snow covers a surface turned to the sky by Up (the y of its normal): the deeper, the steeper the slopes it
// lies on; thin snow lies in patches, between which the ground shows. Spots: a noise of the place, 0..1
float snow_lying(float depth, float up, float spots)
{
	float deep = clamp(depth / 0.12, 0.0, 1.0);
	float slope = smoothstep(mix(0.80, 0.30, deep), mix(0.92, 0.55, deep), up);
	float spotted = smoothstep(spots * 0.035, spots * 0.035 + 0.015, depth);
	return slope * spotted;
}

// the snow stuck to a face turned to the wind which stuck it there (snow_cover::plaster()), streaky
float snow_plastered(vec3 worldnormal, vec2 stable, float height)
{
	float stuck = snow_wind.w;
	if (stuck <= 0.01)
		return 0.0;
	vec2 from = -snow_drift.xy;
	float facing = smoothstep(0.15, 0.65, dot(worldnormal.xz, from)) * (1.0 - smoothstep(0.6, 0.95, abs(worldnormal.y)));
	float streaks = smoothstep(0.3, 0.7, snow_fbm(vec2(dot(stable, vec2(-from.y, from.x)) * 6.0, height * 1.5)));
	return clamp(stuck * 1.4, 0.0, 1.0) * facing * mix(0.5, 1.0, streaks);
}

// how much snow covers a point of a surface, 0..1, and what kind: Exposure how much the snow there is blown off (-1) or in (1),
// Tracked 1 over a track or a road. World: relative to the camera of the pass, Worldnormal: the normal of the surface, world axes
float snow_coverage(vec3 world, vec3 worldnormal, out float depth, out float exposure, out float tracked)
{
	depth = snow_cover.x;
	exposure = 0.0;
	tracked = 0.0;
	vec2 stable;
	ivec4 cells;
	vec4 weights;
	float fade;
	bool mapped = snow_place(world.xz, stable, cells, weights, fade);
	float spots = snow_fbm(stable * 1.7);
	float plaster = snow_plastered(worldnormal, stable, world.y);
	// what a surface over the snow lying (a roof of a vehicle, the top of a wall too thin to be in the maps) holds: what fell on it
	float open = snow_lying(min(snow_cover.x, 0.15), worldnormal.y, spots);
	if (false == mapped)
		return max(open, plaster);
	float height = world.y + sky_occlusion_origin.z;
	float cover = 0.0;
	float weightsum = 0.0;
	float depthsum = 0.0;
	float exposuresum = 0.0;
	float trackedsum = 0.0;
	for (int i = 0; i < 4; ++i)
	{
		vec4 cell = snow_cells[cells[i]];
		float w = weights[i];
		if (cell.x < SNOW_NONE)
		{
			cover += open * w;
			depthsum += snow_cover.x * w;
			weightsum += w;
			continue;
		}
		float lying = 0.0;
		float ground = cell.x - cell.y;
		float kind = cell.w;
		float istracked = (kind > SNOW_TRACKED * 0.5) ? 1.0 : 0.0;
		// under a roof, the deck of a bridge, the vault of a tunnel: none
		bool roofed = (istracked < 0.5) && (cell.z < SNOW_NO_CEILING) && (height < cell.z - 0.1);
		// the heads of the rails the wheels keep clean
		bool railhead = (istracked > 0.5) && (height > cell.z + 0.01) && (height < cell.z + 0.06);
		if ((false == roofed) && (false == railhead) && (height > ground - 0.5))
		{
			// in the snow (the foot of a pole, the rails of a buried track): all of it, with an edge a few centimetres high
			float inside = 1.0 - smoothstep(cell.x - 0.05, cell.x + 0.005, height);
			// on it: by the slope and the depth; well over it: what fell on the thing
			float on = snow_lying(cell.y, worldnormal.y, spots);
			lying = max(inside, mix(on, open, smoothstep(0.3, 1.0, height - cell.x)));
		}
		cover += lying * w;
		depthsum += cell.y * w;
		exposuresum += (kind - istracked * SNOW_TRACKED) * w;
		trackedsum += istracked * w;
		weightsum += w;
	}
	cover /= max(weightsum, 1e-4);
	// away from the camera the maps end, and the snow is as on the open flat ground
	cover = mix(cover, open, fade);
	depth = mix(depthsum / max(weightsum, 1e-4), snow_cover.x, fade);
	exposure = mix(exposuresum / max(weightsum, 1e-4), 0.0, fade);
	tracked = mix(trackedsum / max(weightsum, 1e-4), 0.0, fade);
	return max(cover, plaster);
}

#ifdef STAGE_FRAGMENT

// sets the glossiness the shading turns into Roughness (light_common.glsl takes 1 - glossiness / param[1].w)
float snow_glossiness(float roughness)
{
	return (1.0 - roughness) * max(abs(param[1].w), 1.0);
}

// the look of the snow over a surface: Cover how much of it (0 nothing changes), Stable: the world place for the noise, Exposure and
// Tracked as snow_coverage() gives them, Worldnormal: the normal of the surface, world axes, which the snow smooths, Pixel: the size
// of a pixel there, m (the glints are left out where they'd be smaller than it). sets glossiness and metalic of light_common.glsl
void snow_material(float cover, vec3 stable, float exposure, float tracked, float pixel, inout vec3 worldnormal, inout vec3 texturecolor, inout float reflectivity, inout float specularity)
{
	if (cover <= 0.0)
		return;
	float wet = snow_cover.z;
	float fresh = snow_cover.y;
	// the white of new dry snow, greyer as it gets old and wet, darker still where the trains and the cars soil it
	vec3 white = mix(vec3(0.80, 0.82, 0.86), vec3(0.70, 0.72, 0.75), (1.0 - fresh) * 0.6);
	white = mix(white, vec3(0.52, 0.55, 0.58), wet * 0.7);
	white = mix(white, vec3(0.42, 0.40, 0.37), tracked * 0.55);
	// the crust the wind packs on the exposed places is a little greyer and glossier than the soft snow it piles in the lee
	float crust = clamp(-exposure, 0.0, 1.0);
	white *= mix(1.0, 0.94, crust);
	// a little unevenness
	white *= 0.95 + 0.1 * snow_fbm(stable.xz * 0.5);

	// the snow blown along the ground in a strong wind: brighter streaks running with it
	float blowing = snow_wind.z;
	vec3 surface = vec3(0.0, 1.0, 0.0);
	if (blowing > 0.02)
	{
		vec2 wind = snow_wind.xy;
		float speed = max(length(wind), 1e-3);
		vec2 along = wind / speed;
		vec2 across = vec2(-along.y, along.x);
		float t = weather_camera.y;
		float streak = snow_fbm(vec2(dot(stable.xz, along) * 0.35 - t * speed * 0.6, dot(stable.xz, across) * 2.5));
		float drifting = smoothstep(0.55, 0.85, streak) * blowing;
		white = mix(white, vec3(0.92, 0.94, 0.97), drifting * 0.6);
	}

	// wind ripples across the wind on the drifts and the crust, a few centimetres high, seen up close
	float near = 1.0 - smoothstep(0.01, 0.06, pixel);
	if ((near > 0.0) && (snow_drift.z > 0.05))
	{
		vec2 along = snow_drift.xy;
		float phase = dot(stable.xz, along) * 50.0 + snow_noise(stable.xz * 2.0) * 6.0;
		float slope = cos(phase) * 0.18 * snow_drift.z * (0.4 + 0.6 * crust) * near;
		surface = normalize(vec3(along.x * slope, 1.0, along.y * slope));
	}

	// the normal: the snow smooths the shape under it, the more the deeper it lies
	vec3 smoothed = normalize(mix(worldnormal, surface, 0.6 * cover));
	worldnormal = normalize(mix(worldnormal, smoothed, cover));
	texturecolor = mix(texturecolor, white, cover);

	// rough and dull, but for the glints of the crystals in the sun; wet snow glossy
	float roughness = mix(0.62, 0.30, max(wet, crust * 0.4));
	float sparkle = 0.0;
	if (pixel < 0.02)
	{
		// one crystal in a few dozen, each a centimetre or less, tilted its own way
		vec3 cell = floor(stable * 128.0);
		float h = snow_hash3(cell);
		sparkle = step(0.97, h) * (1.0 - smoothstep(0.004, 0.02, pixel)) * (1.0 - wet);
		if (sparkle > 0.0)
		{
			vec3 tilt = vec3(snow_hash3(cell + 7.1), 0.0, snow_hash3(cell + 3.7)) - vec3(0.5, 0.0, 0.5);
			worldnormal = normalize(mix(worldnormal, normalize(worldnormal + tilt * 0.9), cover));
		}
	}
	roughness = mix(roughness, 0.08, sparkle);
	glossiness = mix(glossiness, snow_glossiness(roughness), cover);
	metalic = mix(metalic, 0.0, cover);
	specularity = mix(specularity, mix(0.25, 0.45, max(wet, sparkle)), cover);
	reflectivity = mix(reflectivity, mix(0.02, 0.12, wet), cover);
}

#endif

#endif

#endif
