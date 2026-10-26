// the water on the windscreens (gfx::windscreen_water, gl::WATER_* in gl/ubo.h, the vulkan renderer: vk14::windscreen_water): what the passes of the simulation (water_*.comp)
// keep and the glasses (mat_rain_windscreen.frag) read. a glass is a material of mat_rain_windscreen in the cab of the train, a
// slot of the storage; its drops lie in the coordinates of its texture (uv), as its mask of the wipers does, so they stay where
// they lie on the glass whatever the vehicle and the camera do. the map of the glass tells what a step of the uv is along the
// glass (metres, in the space of the model of the cab) and where the blades go over it; the drops are looked up through a grid
// of cells over the uv, each listing the drops which reach into it; the wetted paths are marked on a map of their own with the
// time the water went there

#ifndef WATER_GLSL
#define WATER_GLSL

struct water_drop_s
{
	// xy: the place, uv; z: the water, mm3 (0: no drop there); w: seconds it lies on the glass
	vec4 place;
	// xy: the velocity, uv per second; z: random 0 .. 1, its shape; w: 0 .. 1 how far it melted from a flake of snow (1: water)
	vec4 motion;
};

#if defined(STAGE_COMPUTE)
// the compute kernels: set COMPUTE_SET pushed with the dispatch, the storage at 0, the block of the step at 1 (water_sim.glsl)
layout(std430, set = COMPUTE_SET, binding = 0) buffer water_ssbo
#else
layout(std430, set = PASS_SET, binding = WATER_SSBO) readonly buffer water_ssbo
#endif
{
	// every glass: x 1 when it's simulated, y seconds its simulation runs (the time of the wetted paths), zw unused
	vec4 water_header[WATER_SLOTS];
	// every glass: x the drops on it, yzw unused
	uvec4 water_counts_of[WATER_SLOTS];
	water_drop_s water_drops[WATER_SLOTS * WATER_DROPS];
	// the map, two for a texel: xyz the step along the glass for a step of u (metres), w 1 on the glass, 0.5 next to it (the edges
	// spread out, for the filtering), 0 elsewhere; xyz the same for v, w 1 when the cross product of the two points out of the cab,
	// -1 when it points in
	vec4 water_axes[WATER_SLOTS * WATER_MAP * WATER_MAP * 2];
	// the mask of the wipers baked into the map: the place of a blade over its sweep at which it goes over the texel, 0 where it
	// doesn't reach
	vec4 water_blades[WATER_SLOTS * WATER_MAP * WATER_MAP];
	// every drop: 0 the place is free, else bit 0 set, bits 1 .. 3 the blade carrying it (1 + its number, 0 none), bit 4 set when
	// the blade carries it towards the far end of its sweep
	uint water_state[WATER_SLOTS * WATER_DROPS];
	// the drop it flows into (itself when none)
	uint water_target[WATER_SLOTS * WATER_DROPS];
	// what flowed into it: the water (1e-4 mm3) and the moment of the places of the drops about its own (1e-6 uv mm3)
	int water_gather[WATER_SLOTS * WATER_DROPS * 4];
	// x: the way it went since it left a droplet behind, metres; y: the way it goes on before it leaves the next one, metres
	vec4 water_extra[WATER_SLOTS * WATER_DROPS];
	// the grids of the glasses, WATER_GRID x WATER_GRID cells each: the counts of the drops of the cells, then the drops of each
	// cell (WATER_CELL_DROPS for a cell, the ones over the count aren't there). then the wetted paths of the glasses,
	// WATER_TRAIL x WATER_TRAIL each: in the upper 24 bits 1 + the time the water went over the texel last, 1/2 s (0: never),
	// in the lower 8 how near the middle of the path the texel is, 0 .. 255 (filtered between the texels the edges of the paths
	// come out smooth wherever they go across the texels; the time is coarse, so a drop going along leaves the nearness of its
	// whole passage, not of the step which happened to come last)
	uint water_cells[];
};

// the radius of the footprint of a drop holding Volume mm3 of water, metres: a cap of a sphere meeting the glass at 40 degrees
float water_radius(float volume)
{
	return 0.001 * pow(1.675 * max(volume, 0.0), 1.0 / 3.0);
}

// the diameter of a flake of snow holding Volume mm3 of water, metres
float water_flake(float volume)
{
	return 0.006 * pow(max(volume, 0.0), 1.0 / 3.0);
}

// where the grid of a glass begins in water_cells: the count of a cell at the base + the cell, its drops from the base +
// WATER_GRID * WATER_GRID + the cell * WATER_CELL_DROPS
uint water_grid_base(uint slot)
{
	return slot * uint(WATER_GRID * WATER_GRID * (1 + WATER_CELL_DROPS));
}

uint water_trail_base(uint slot)
{
	return uint(WATER_SLOTS * WATER_GRID * WATER_GRID * (1 + WATER_CELL_DROPS)) + slot * uint(WATER_TRAIL * WATER_TRAIL);
}

// the cell of the grid holding a place of the glass
uint water_cell(vec2 uv)
{
	ivec2 cell = clamp(ivec2(floor(uv * float(WATER_GRID))), ivec2(0), ivec2(WATER_GRID - 1));
	return uint(cell.y * WATER_GRID + cell.x);
}

// how fresh the wetted path is at a texel of the map of the paths, 1 just wetted in its middle .. 0 dry long ago or never; Now:
// seconds the simulation of the glass runs, Lasting: seconds in which the freshness falls to 1/e
float water_wetted(uint slot, ivec2 texel, float now, float lasting)
{
	texel = clamp(texel, ivec2(0), ivec2(WATER_TRAIL - 1));
	uint value = water_cells[water_trail_base(slot) + uint(texel.y * WATER_TRAIL + texel.x)];
	uint stamp = value >> 8u;
	if (stamp == 0u)
		return 0.0;
	float age = max(now - float(stamp - 1u) / 2.0, 0.0);
	return exp(-age / lasting) * (float(value & 255u) / 255.0);
}

// the same filtered between the texels at a place of the glass
float water_wetted_at(uint slot, vec2 uv, float now, float lasting)
{
	vec2 p = uv * float(WATER_TRAIL) - 0.5;
	ivec2 i = ivec2(floor(p));
	vec2 f = p - vec2(i);
	float a = water_wetted(slot, i, now, lasting);
	float b = water_wetted(slot, i + ivec2(1, 0), now, lasting);
	float c = water_wetted(slot, i + ivec2(0, 1), now, lasting);
	float d = water_wetted(slot, i + ivec2(1, 1), now, lasting);
	return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

#endif
