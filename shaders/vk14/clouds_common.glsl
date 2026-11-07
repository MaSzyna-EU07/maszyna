// clouds of the weather (opengl46_weather): density of the layers of weather_common.glsl, made of three tiling noises.
// the shape noise (3d, clouds_noise.comp) gives the lumps of a cloud, the detail noise (3d) eats into their edges, the coverage
// noise (2d) spreads the clouds of a layer over the sky so they take up its coverage. positions are world offsets from the
// camera, in metres, y up; the earth curves away from the camera, so far clouds sink below the horizon
#include "weather_common.glsl"

layout(set = PASS_SET, binding = CLOUD_SHAPE_TEX) uniform sampler3D cloud_shape_noise;
layout(set = PASS_SET, binding = CLOUD_DETAIL_TEX) uniform sampler3D cloud_detail_noise;
layout(set = PASS_SET, binding = CLOUD_COVERAGE_TEX) uniform sampler2D cloud_coverage_noise;

const float CLOUD_EARTH_RADIUS = 6360000.0;
// how much of the shape noise is left standing at the edge and in the core of a cloud area (cloud_layer_density)
const float CLOUD_FILL_EDGE = 0.45;
const float CLOUD_FILL_CORE = 0.85;
// how fast the density rises inside the edge of a cloud: above 1 the edges are crisper, the inside keeps its density
const float CLOUD_EDGE_SHARPNESS = 3.0;
// how far the base and the top of the single clouds stand off those of their layer, up or down, as shares of the depth of the layer
// and at most in metres: the clouds of a layer don't all end at one height, some reach a little out of it
// how high the base of a cloud curves up at its edge and how far into the cloud area (cloud_coverage: y) it does, and how much higher
// the base of a billow is between the cells of the shape noise than in their middles; shares of the height of the cloud
const float CLOUD_BASE_ROUND = 0.35;
const float CLOUD_BASE_ROUND_DEPTH = 0.3;
const float CLOUD_BASE_LUMPS = 0.08;
const float CLOUD_BASE_SHIFT = 0.07;
const float CLOUD_BASE_SHIFT_MAX = 100.0;
const float CLOUD_TOP_SHIFT = 0.15;
const float CLOUD_TOP_SHIFT_MAX = 400.0;
// the detail noise: its tile against the tile of the shape noise and how deep it eats into the edges (times the erosion of the
// genus and the tuning of the debug window, cloud_wind.w); the finer octave of the thin edges: its tile against the detail tile,
// how far the coarser octave pushes its lookup about (in its tiles), and the density below which it is used
const float CLOUD_DETAIL_SCALE = 6.0;
const float CLOUD_EROSION = 2.0;
const float CLOUD_FINE_SCALE = 3.7;
const float CLOUD_FINE_PUSH = 0.6;
const float CLOUD_FINE_BELOW = 0.6;

float cloud_remap(float value, float low, float high, float newlow, float newhigh)
{
	return newlow + (value - low) / max(high - low, 1e-5) * (newhigh - newlow);
}

// height of a point above the ground reference, the earth curving down with the square of the distance
float cloud_height(vec3 position)
{
	return weather_camera.x + position.y - dot(position.xz, position.xz) * (0.5 / CLOUD_EARTH_RADIUS);
}

// distances along a ray (unit Direction from the camera) where it is at Height: the height goes as
// camera + up * t - (1 - up^2) / (2 R) * t^2. returns the number of crossings ahead (0-2), sorted into T
int cloud_ray_height(float up, float height, out vec2 t)
{
	t = vec2(-1.0);
	float a = (1.0 - up * up) * (0.5 / CLOUD_EARTH_RADIUS);
	float b = -up;
	float c = height - weather_camera.x;
	// a t^2 + b t + c = 0
	if (a < 1e-12)
	{
		if (abs(b) < 1e-8)
			return 0;
		float root = -c / b;
		t = vec2(root, -1.0);
		return root > 0.0 ? 1 : 0;
	}
	float discriminant = b * b - 4.0 * a * c;
	if (discriminant < 0.0)
		return 0;
	float q = -0.5 * (b + (b >= 0.0 ? 1.0 : -1.0) * sqrt(discriminant));
	float r0 = q / a;
	float r1 = (abs(q) > 1e-12) ? c / q : r0;
	vec2 roots = vec2(min(r0, r1), max(r0, r1));
	if (roots.y <= 0.0)
		return 0;
	if (roots.x <= 0.0)
	{
		t = vec2(roots.y, -1.0);
		return 1;
	}
	t = roots;
	return 2;
}

// the first stretch of a ray within the heights of a layer, x to y; y <= x when it never gets there
vec2 cloud_layer_segment(float up, float base, float top)
{
	vec2 tb, tt;
	int nb = cloud_ray_height(up, base, tb);
	int nt = cloud_ray_height(up, top, tt);
	// the crossings in order; the ray goes in and out of the layer at each
	float crossings[4];
	int count = 0;
	for (int i = 0; i < nb; i++)
		crossings[count++] = tb[i];
	for (int i = 0; i < nt; i++)
		crossings[count++] = tt[i];
	for (int i = 1; i < count; i++)
		for (int j = i; j > 0 && crossings[j - 1] > crossings[j]; j--)
		{
			float swap = crossings[j];
			crossings[j] = crossings[j - 1];
			crossings[j - 1] = swap;
		}
	bool inside = (weather_camera.x >= base) && (weather_camera.x <= top);
	float start = 0.0;
	for (int i = 0; i < count; i++)
	{
		if (inside)
			return vec2(start, crossings[i]);
		inside = true;
		start = crossings[i];
	}
	return inside ? vec2(start, 1e9) : vec2(0.0, -1.0);
}

// the position in the noise of a layer: the camera's place in it and the offset from the camera, stretched along the wind
vec2 cloud_noise_plane(cloud_layer_s layer, vec2 offset)
{
	vec2 wind = cloud_wind.xy;
	float along = dot(offset, wind);
	vec2 across = offset - wind * along;
	return across + wind * (along / max(layer.motion.w, 1.0));
}

// how far the base (x) and the top (y) of the clouds of a layer may stand off those of the layer, metres
vec2 cloud_layer_shift(cloud_layer_s layer)
{
	float thickness = layer.heights.y - layer.heights.x;
	return vec2(min(thickness * CLOUD_BASE_SHIFT, CLOUD_BASE_SHIFT_MAX), min(thickness * CLOUD_TOP_SHIFT, CLOUD_TOP_SHIFT_MAX));
}

// the lowest base (x) and the highest top (y) the clouds of a layer may reach
vec2 cloud_layer_bounds(cloud_layer_s layer)
{
	vec2 shift = cloud_layer_shift(layer);
	return vec2(layer.heights.x - shift.x, layer.heights.y + shift.y);
}

// share of the layer filled at a place, x: the soft mask of the coverage, y: how far into a cloud area (0 at its edge). Shift: how
// far the base (x) and the top (y) of the cloud there stand off those of the layer, from the second and the third noise of the
// coverage texture, so the clouds which reach lower don't all reach higher too
vec2 cloud_coverage(cloud_layer_s layer, vec2 offset, out vec2 shift)
{
	vec2 coords = (layer.extra.zw + cloud_noise_plane(layer, offset)) * layer.motion.z;
	vec3 noise = textureLod(cloud_coverage_noise, coords + layer.extra.y, 0.0).rgb;
	shift = (noise.yz * 2.0 - 1.0) * cloud_layer_shift(layer);
	float coverage = layer.shape.x;
	float threshold = 1.0 - coverage;
	float mask = smoothstep(threshold - 0.06, threshold + 0.06, noise.x);
	float depth = clamp((noise.x - threshold) / max(coverage, 0.05), 0.0, 1.0);
	return vec2(mask, depth);
}

vec2 cloud_coverage(cloud_layer_s layer, vec2 offset)
{
	vec2 shift;
	return cloud_coverage(layer, offset, shift);
}

// extinction (1/m) of a volumetric layer at a point; Detail eats the edges with the small noise
float cloud_layer_density(int index, vec3 position, float height, bool detail)
{
	cloud_layer_s layer = cloud_layers[index];
	vec2 bounds = cloud_layer_bounds(layer);
	if (height <= bounds.x || height >= bounds.y)
		return 0.0;
	vec2 shift;
	vec2 coverage = cloud_coverage(layer, position.xz, shift);
	float bottom = layer.heights.x + shift.x;
	float h = (height - bottom) / max(layer.heights.y + shift.y - bottom, 1.0);
	if (h <= 0.0 || h >= 1.0)
		return 0.0;
	// the anvil spreads the top of the storm clouds out of their area
	if (layer.profile.w > 0.0)
	{
		float anvil = layer.profile.w * smoothstep(0.7, 0.92, h) * (1.0 - smoothstep(0.97, 1.0, h));
		coverage.x = max(coverage.x, anvil * smoothstep(0.0, 0.5, coverage.y + 0.35));
		coverage.y = max(coverage.y, anvil * 0.6);
	}
	if (coverage.x <= 0.0)
		return 0.0;
	// the tops of the single clouds are of different heights, the larger ones (deeper in their area) the taller
	float top = mix(1.0 - layer.profile.z, 1.0, coverage.y);
	float fall = 1.0 - smoothstep(top * layer.profile.y, top, h);
	if (fall <= 0.0)
		return 0.0;
	vec2 plane = layer.motion.xy + cloud_noise_plane(layer, position.xz);
	vec3 coords = vec3(plane.x, height + layer.extra.y * 4096.0, plane.y) * layer.shape.z;
	vec4 shape = textureLod(cloud_shape_noise, coords, 0.0);
	// the smaller cells of the worley octaves bite billows into the heaps of the perlin-worley noise; the octaves keep to a narrow
	// band (clouds_noise.comp: WORLEY_FBM_LOW, WORLEY_FBM_HIGH), spread over 0-1 first
	float worley = clamp((shape.g * 0.625 + shape.b * 0.25 + shape.a * 0.125 - 0.30) / 0.35, 0.0, 1.0);
	// the base is not cut flat: it curves up towards the edge of the cloud into its sides, and every billow starts a little higher or
	// lower; both as shares of the height of the cloud, so the small ones are rounded too and not cut away. the sheets (little
	// erosion) keep a flatter base
	float lift = (CLOUD_BASE_ROUND * (1.0 - smoothstep(0.0, CLOUD_BASE_ROUND_DEPTH, coverage.y)) + CLOUD_BASE_LUMPS * (1.0 - worley)) * top * clamp(layer.shape.w * 2.5, 0.0, 1.0);
	float gradient = smoothstep(lift, lift + layer.profile.x, h) * fall;
	if (gradient <= 0.0)
		return 0.0;
	float base = clamp(cloud_remap(shape.r, (1.0 - worley) * 0.35, 1.0, 0.0, 1.0), 0.0, 1.0);
	// the sheets (little erosion) keep few holes, the heaps are made of lumps
	base = mix(1.0, base, clamp(layer.shape.w * 2.5, 0.25, 1.0));
	// inside its area a cloud is not a solid block: the lumps of the shape noise are cut out of it, the more the nearer its edge,
	// so it breaks up into single heaps with rounded tops and gaps between them. the sheets keep their base above the cut, and a
	// layer closing up (broken to overcast) keeps its lumps whole, so it has no more gaps than its coverage
	float fill = coverage.x * mix(mix(CLOUD_FILL_EDGE, CLOUD_FILL_CORE, coverage.y), 1.0, smoothstep(0.5, 1.0, layer.shape.x));
	float density = clamp(cloud_remap(base * gradient, 1.0 - fill, 1.0, 0.0, 1.0), 0.0, 1.0) * coverage.x;
	if (detail && density > 0.0 && layer.shape.w > 0.0)
	{
		vec3 detailcoords = coords * CLOUD_DETAIL_SCALE;
		vec4 small = textureLod(cloud_detail_noise, detailcoords, 0.0);
		// spread over 0-1 as the worley octaves of the shape; left in their narrow band they would only thin the cloud evenly
		float fbm = clamp((small.r * 0.625 + small.g * 0.25 + small.b * 0.125 - 0.30) / 0.35, 0.0, 1.0);
		// the thin outer part of a cloud gets a finer octave too, its lookup pushed about by the coarser one, so the edges are frayed
		// into small billows and torn bits rather than evenly rounded. the dense inside is never eaten into, so it is skipped there
		float fineshare = 1.0 - smoothstep(CLOUD_FINE_BELOW * 0.5, CLOUD_FINE_BELOW, density);
		if (fineshare > 0.0)
		{
			vec3 push = (small.gbr - 0.5) * CLOUD_FINE_PUSH;
			vec4 fine = textureLod(cloud_detail_noise, detailcoords * CLOUD_FINE_SCALE + push, 0.0);
			float finefbm = clamp((fine.r * 0.625 + fine.g * 0.25 + fine.b * 0.125 - 0.30) / 0.35, 0.0, 1.0);
			fbm = clamp(mix(fbm, fbm * 0.65 + finefbm * 0.35, fineshare), 0.0, 1.0);
		}
		// wispy at the base, billowing at the top
		float erosion = mix(fbm, 1.0 - fbm, clamp(h * 4.0, 0.0, 1.0));
		density = clamp(cloud_remap(density, erosion * layer.shape.w * CLOUD_EROSION * cloud_wind.w, 1.0, 0.0, 1.0), 0.0, 1.0);
	}
	return min(density * CLOUD_EDGE_SHARPNESS, 1.0) * layer.shape.y;
}

// the same without the detail, cheaper, for the shadows and the fog
float cloud_density_coarse(vec3 position)
{
	float height = cloud_height(position);
	float density = 0.0;
	for (int i = 0; i < int(weather_camera.w); i++)
	{
		if (cloud_layers[i].extra.x > 0.5)
			continue;
		density += cloud_layer_density(i, position, height, false);
	}
	return density;
}

// optical depth of a flat layer (cirrus and the like) where a ray crosses its middle, Up the vertical part of the ray there
float cloud_flat_depth(int index, vec3 position)
{
	cloud_layer_s layer = cloud_layers[index];
	vec2 coverage = cloud_coverage(layer, position.xz);
	if (coverage.x <= 0.0)
		return 0.0;
	vec2 plane = layer.motion.xy + cloud_noise_plane(layer, position.xz);
	vec2 coords = plane * layer.shape.z;
	// strands and ripples out of slices of the noise
	vec4 shape = textureLod(cloud_shape_noise, vec3(coords, 0.37 + layer.extra.y), 0.0);
	vec4 small = textureLod(cloud_detail_noise, vec3(coords * 4.0, 0.61), 0.0);
	float strands = clamp(cloud_remap(shape.r * 0.7 + small.r * 0.3, 0.35 * layer.shape.w + (1.0 - coverage.x) * 0.5, 1.0, 0.0, 1.0), 0.0, 1.0);
	return strands * coverage.x * layer.shape.y;
}

// the shadow of the clouds at a point: what they let through towards the key light. Steps per layer
float cloud_transmittance_to_sun(vec3 position, int steps)
{
	vec3 sun = weather_sun_direction.xyz;
	if (sun.y <= 0.01)
		return fog_light.y;
	float height = cloud_height(position);
	float depth = 0.0;
	for (int i = 0; i < int(weather_camera.w); i++)
	{
		cloud_layer_s layer = cloud_layers[i];
		if (layer.extra.x > 0.5)
		{
			float middle = 0.5 * (layer.heights.x + layer.heights.y);
			if (middle > height)
				depth += cloud_flat_depth(i, position + sun * ((middle - height) / sun.y)) / sun.y;
			continue;
		}
		vec2 bounds = cloud_layer_bounds(layer);
		float enter = max(bounds.x - height, 0.0) / sun.y;
		float leave = (bounds.y - height) / sun.y;
		if (leave <= 0.0)
			continue;
		float step = (leave - enter) / float(steps);
		for (int s = 0; s < steps; s++)
		{
			vec3 point = position + sun * (enter + step * (float(s) + 0.5));
			depth += cloud_layer_density(i, point, cloud_height(point), false) * step;
		}
	}
	return exp(-depth);
}
