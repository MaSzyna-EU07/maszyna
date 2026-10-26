// what the passes of the simulation of the water on the windscreens share (water_*.comp): the block of the step, the random
// numbers, the map of the glass, the forces on a drop
//
// the physics (the drops are caps of spheres flat on the glass, 40 degrees at their edge, untreated weathered glass):
// - a drop lies held by its line of contact: the hold is the surface tension times the width of the drop times the difference
//   of the cosines of its angles at the front and the rear (WATER_HOLD), so it grows with the radius, while the weight grows
//   with the cube of it. on a vertical glass a drop slides once its footprint is about 7 mm across (25 mm3), on a glass leaning
//   back later. the hold is uneven over the glass (dirt, scratches: the drops stop and go) and much less where the water went
//   lately (a wetted path: the next drops follow it and it becomes a rivulet)
// - the air going along the glass drags the drops (with the area they stand up into it, within the layer of slow air at the
//   glass), and of the air meeting the glass head on a share turns up along it: at speed the water goes up the glass, the sooner
//   the more it leans back
// - a sliding drop is held back by the viscosity in proportion to its speed and radius; it leaves droplets behind as it goes
//   (it thins and stops in the end) and wets its path
// - drops which touch flow together into one (surface tension), the water and the momentum of the places kept
// - a raindrop hits the glass far above the speed at which it splashes: it spreads and throws droplets around, which stay
// - a blade going over a drop takes it along in front of itself; the water it gathers flows together into a bead along it, left
//   at the ends of its sweep (where it runs down as streams) and pushed off where the glass ends

#ifndef WATER_SIM_GLSL
#define WATER_SIM_GLSL

#include "water.glsl"

// gl::water_ubs, every vector in the space of the model of the cab
layout(std140, set = COMPUTE_SET, binding = 1) uniform water_ubo
{
	// x: seconds of the step, y: seconds the simulation runs, z: how heavy the rain or the snow is 0 .. 1, w: 1 for snow
	vec4 water_step;
	// xyz: up, w: the temperature of the air, degrees Celsius
	vec4 water_up;
	// xyz: the air against the cab, m/s; w: how fast the rain or the snow falls, m/s
	vec4 water_air;
	// the blades now and at the step before: 0 .. 1 across their sweep, below 0 where there's none
	vec4 water_blades_now;
	vec4 water_blades_before;
	// x: the smallest diameter of a raindrop landing as a drop of its own, mm, y: 1 / the mean of the diameters above it, 1 / mm
	vec4 water_rain;
	// x: random seed of the step, y: the most landings tried on a glass, z: the glass a single glass pass works on
	uvec4 water_counts;
	// every glass: x the landings tried in the step, y 1 / the most metres squared a unit of its uv holds, z 1 when it's simulated,
	// w the size of its drops (rain_scale of its material)
	vec4 water_slots[WATER_SLOTS];
};

const float WATER_GRAVITY = 9.81;
// the surface tension of water, N/m, and the share of it a drop is held by for the width of its footprint
const float WATER_TENSION = 0.072;
const float WATER_HOLD = 0.5;
// the viscous drag of a sliding drop for its radius and speed, N s / m2
const float WATER_VISCOUS = 1.3;
// the share of the speed of the air the drops feel in the slow layer at the glass, and of the air meeting the glass head on
// which turns up along it
const float WATER_BOUNDARY = 0.3;
const float WATER_HEAD_ON = 0.35;
// the drops below this much water (mm3) dry at once; a raindrop or a droplet of a splash smaller than WATER_SEEN (0.5 mm across
// its footprint, hardly a pixel) landing where there's no drop is left to the mist of the glass
const float WATER_LEAST = 0.004;
const float WATER_SEEN = 0.075;

uvec4 water_pcg4d(uvec4 v)
{
	v = v * 1664525u + 1013904223u;
	v.x += v.y * v.w;
	v.y += v.z * v.x;
	v.z += v.x * v.y;
	v.w += v.y * v.z;
	v ^= v >> 16u;
	v.x += v.y * v.w;
	v.y += v.z * v.x;
	v.z += v.x * v.y;
	v.w += v.y * v.z;
	return v;
}

// four random numbers 0 .. 1
vec4 water_random(uint a, uint b, uint c, uint d)
{
	return vec4(water_pcg4d(uvec4(a, b, c, d))) * (1.0 / 4294967296.0);
}

// value noise 0 .. 1
float water_noise(vec2 p, uint layer)
{
	vec2 i = floor(p);
	vec2 f = p - i;
	vec2 u = f * f * (3.0 - 2.0 * f);
	uvec2 c = uvec2(ivec2(i));
	float a = water_random(c.x, c.y, layer, 0u).x;
	float b = water_random(c.x + 1u, c.y, layer, 0u).x;
	float d = water_random(c.x, c.y + 1u, layer, 0u).x;
	float e = water_random(c.x + 1u, c.y + 1u, layer, 0u).x;
	return mix(mix(a, b, u.x), mix(d, e, u.x), u.y);
}

// the glass at a place of the uv: the steps along it for the steps of u and v (metres), the way out of the cab and whether the
// place is on the glass (1), next to it (0.5) or off it (0)
struct water_frame_s
{
	vec3 tu;
	vec3 tv;
	vec3 normal;
	float valid;
};

water_frame_s water_frame(uint slot, vec2 uv)
{
	water_frame_s frame = water_frame_s(vec3(0.0), vec3(0.0), vec3(0.0, 0.0, 1.0), 0.0);
	if (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0))))
		return frame;
	uint base = slot * uint(WATER_MAP * WATER_MAP);
	vec2 p = uv * float(WATER_MAP) - 0.5;
	ivec2 i = ivec2(floor(p));
	vec2 f = p - vec2(i);
	float total = 0.0;
	float side = 0.0;
	for (int k = 0; k < 4; ++k)
	{
		ivec2 texel = clamp(i + ivec2(k & 1, k >> 1), ivec2(0), ivec2(WATER_MAP - 1));
		uint index = base + uint(texel.y * WATER_MAP + texel.x);
		vec4 a = water_axes[index * 2u];
		vec4 b = water_axes[index * 2u + 1u];
		if (a.w <= 0.0)
			continue;
		float w = (((k & 1) != 0) ? f.x : 1.0 - f.x) * (((k >> 1) != 0) ? f.y : 1.0 - f.y) + 1e-4;
		frame.tu += a.xyz * w;
		frame.tv += b.xyz * w;
		side += b.w * w;
		total += w;
	}
	if (total <= 0.0)
		return frame;
	frame.tu /= total;
	frame.tv /= total;
	vec3 n = cross(frame.tu, frame.tv);
	float length_n = length(n);
	if (length_n < 1e-12)
		return frame;
	frame.normal = n * (((side < 0.0) ? -1.0 : 1.0) / length_n);
	// on the glass by the texel the place is in
	ivec2 nearest = clamp(ivec2(floor(uv * float(WATER_MAP))), ivec2(0), ivec2(WATER_MAP - 1));
	frame.valid = water_axes[(base + uint(nearest.y * WATER_MAP + nearest.x)) * 2u].w;
	return frame;
}

// a step along the glass (metres, the space of the cab) as a step of the uv
vec2 water_to_uv(water_frame_s frame, vec3 step)
{
	float a = dot(frame.tu, frame.tu);
	float b = dot(frame.tu, frame.tv);
	float c = dot(frame.tv, frame.tv);
	float det = a * c - b * b;
	if (abs(det) < 1e-20)
		return vec2(0.0);
	vec2 r = vec2(dot(frame.tu, step), dot(frame.tv, step));
	return vec2(c * r.x - b * r.y, a * r.y - b * r.x) / det;
}

// the blades at a place of the uv, filtered between the texels of the map
vec4 water_blades_at(uint slot, vec2 uv)
{
	uint base = slot * uint(WATER_MAP * WATER_MAP);
	vec2 p = clamp(uv, vec2(0.0), vec2(1.0)) * float(WATER_MAP) - 0.5;
	ivec2 i = ivec2(floor(p));
	vec2 f = p - vec2(i);
	vec4 result = vec4(0.0);
	for (int k = 0; k < 4; ++k)
	{
		ivec2 texel = clamp(i + ivec2(k & 1, k >> 1), ivec2(0), ivec2(WATER_MAP - 1));
		float w = (((k & 1) != 0) ? f.x : 1.0 - f.x) * (((k >> 1) != 0) ? f.y : 1.0 - f.y);
		result += water_blades[base + uint(texel.y * WATER_MAP + texel.x)] * w;
	}
	return result;
}

// the blades at the texel of the map holding a place of the uv, unfiltered: where a blade reaches it's at least 0.001 (the
// filtered values fall off across a texel at the edge of the sweep)
vec4 water_blades_texel(uint slot, vec2 uv)
{
	ivec2 texel = clamp(ivec2(floor(uv * float(WATER_MAP))), ivec2(0), ivec2(WATER_MAP - 1));
	return water_blades[slot * uint(WATER_MAP * WATER_MAP) + uint(texel.y * WATER_MAP + texel.x)];
}

// a free place for a drop of the glass, taken; ~0u when none was found
uint water_claim(uint slot, uint seed)
{
	for (uint k = 0u; k < 8u; ++k)
	{
		uint place = water_pcg4d(uvec4(seed, k, slot, 77u)).x % uint(WATER_DROPS);
		if (atomicCompSwap(water_state[slot * uint(WATER_DROPS) + place], 0u, 1u) == 0u)
			return place;
	}
	return ~0u;
}

// a new drop of the glass: Volume mm3 at a place, a flake of snow when Melted is below 1. false when the glass is full. the
// landing rain and snow get only nine tenths of the places, so there are always some for the droplets the moving drops leave
bool water_add(uint slot, uint seed, vec2 uv, float volume, float melted, float shape, bool landing)
{
	if ((volume < WATER_LEAST) || any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0))))
		return false;
	if (landing && (water_counts_of[slot].x >= uint(WATER_DROPS) / 10u * 9u))
		return false;
	uint place = water_claim(slot, seed);
	if (place == ~0u)
		return false;
	atomicAdd(water_counts_of[slot].x, 1u);
	uint index = slot * uint(WATER_DROPS) + place;
	water_drops[index].place = vec4(uv, volume, 0.0);
	water_drops[index].motion = vec4(0.0, 0.0, shape, melted);
	water_extra[index] = vec4(0.0, 0.004 * (0.6 + 0.8 * shape), 0.0, 0.0);
	water_target[index] = place;
	for (uint k = 0u; k < 4u; ++k)
		water_gather[index * 4u + k] = 0;
	return true;
}

// the slot of the drop on the glass freed, with what was flowing into it
void water_remove(uint index)
{
	water_drops[index].place.z = 0.0;
	water_state[index] = 0u;
	atomicAdd(water_counts_of[index / uint(WATER_DROPS)].x, ~0u);
	for (uint k = 0u; k < 4u; ++k)
		water_gather[index * 4u + k] = 0;
}

// Volume mm3 of water flowing into a drop (index in the storage) from Offset (uv) away from its middle, taken by
// water_settle.comp
void water_pour(uint index, float volume, vec2 offset)
{
	atomicAdd(water_gather[index * 4u], int(volume * 1e4));
	atomicAdd(water_gather[index * 4u + 1u], int(offset.x * volume * 1e6));
	atomicAdd(water_gather[index * 4u + 2u], int(offset.y * volume * 1e6));
}

// the drop lying at a place of the glass which water of Radius (metres) landing there touches, the largest of them, by the grid
// of the step before (~0u: none)
uint water_landing_on(uint slot, water_frame_s frame, vec2 uv, float radius)
{
	uint cell = water_cell(uv);
	uint base = water_grid_base(slot);
	uint count = min(water_cells[base + cell], uint(WATER_CELL_DROPS));
	uint best = ~0u;
	float most = 0.0;
	for (uint k = 0u; k < count; ++k)
	{
		uint other = water_cells[base + uint(WATER_GRID * WATER_GRID) + cell * uint(WATER_CELL_DROPS) + k];
		water_drop_s drop = water_drops[slot * uint(WATER_DROPS) + other];
		if ((drop.place.z <= most) || (drop.motion.w < 1.0))
			continue;
		vec2 step = drop.place.xy - uv;
		if (length(frame.tu * step.x + frame.tv * step.y) < 0.9 * (radius + water_radius(drop.place.z)))
		{
			best = other;
			most = drop.place.z;
		}
	}
	return best;
}

// water landing on the glass: it flows into a drop it touches, else it's a new drop where it's large enough to be seen
void water_land(uint slot, uint seed, water_frame_s frame, vec2 uv, float volume, float shape)
{
	if (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0))))
		return;
	uint onto = water_landing_on(slot, frame, uv, water_radius(volume));
	if (onto != ~0u)
	{
		uint index = slot * uint(WATER_DROPS) + onto;
		water_pour(index, volume, uv - water_drops[index].place.xy);
	}
	else if (volume >= WATER_SEEN)
	{
		water_add(slot, seed, uv, volume, 1.0, shape, true);
	}
}

#endif
