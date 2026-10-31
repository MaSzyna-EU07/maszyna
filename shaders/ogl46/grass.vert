#version 460 core
#define STAGE_VERTEX 1
// grass of the heightmap terrain (opengl46_grass): a blade made of the numbers of its vertex and its instance.
//
// the instances of a draw stand in a tile of a chunk. a blade takes its place in the tile from a sequence which spreads
// any number of its first members evenly (R2, shifted for each tile), so drawing fewer of them thins the grass out
// evenly; a blade picks its kind by the densities painted there, and gets its height, its turn and its lean from its own
// numbers, so it's the same in every frame. blades of no kind, in the holes, or past the share drawn at their distance are
// collapsed into a point outside the view. the vertices go up the blade in pairs, the last one is its tip.
// the chunk is drawn relative to its corner: x, z in metres from the corner, y the height as it is

#include "common.glsl"

// x, z: corner of the tile in the chunk, metres; y: side of the tile; w: blades of the tile where its densest kind is painted fully
layout(location = 0) uniform vec4 grass_tile;
// x: the share of the densest kind at the densest place of the tile (the blades of the tile are counted for it); y: segments
// of a blade; z, w: number of the tile in the world
layout(location = 1) uniform vec4 grass_part;
// x: side of the chunk, metres; y: cells of its grid along a side; z: grass samples along a side less one
layout(location = 2) uniform vec4 grass_chunk;
// the camera relative to the corner of the chunk (y as it is)
layout(location = 3) uniform vec4 grass_camera;
// x: distance the grass starts to thin out at, for speed; y: distance the fading starts at (gfx.grass.distance); z: where the
// grass ends (gfx.grass.fadedistance)
layout(location = 4) uniform vec4 grass_lod;
// x, y: direction of the wind (x, z of the world); z: its strength, 0..1; w: time, seconds
layout(location = 5) uniform vec4 grass_wind;
// the kinds of the chunk: colour at the root and at the tip (linear rgb), and height, its variation, width, lean
layout(location = 6) uniform vec4 grass_base[4];
layout(location = 10) uniform vec4 grass_tip[4];
layout(location = 14) uniform vec4 grass_shape[4];
// share of the blades of each kind against the densest one of the chunk
layout(location = 18) uniform vec4 grass_share;
// the colour of each kind dried out (linear rgb), and how much it follows the seasons
layout(location = 19) uniform vec4 grass_dry[4];
// the time of the year: x: how dry the grass is, y: how fresh (spring), z: how grown (share of the height)
layout(location = 23) uniform vec4 grass_season;

layout(binding = 0) uniform sampler2D grass_heights; // heights of the grid points, R32F
layout(binding = 1) uniform sampler2D grass_density; // densities of the kinds, RGBA8
layout(binding = 2) uniform sampler2D grass_holes; // holes cut in the ground, R8

layout(location = 0) out vec3 f_normal;
layout(location = 3) out vec4 f_pos;
layout(location = 12) out vec3 f_albedo;
layout(location = 4) out vec4 f_clip_pos;
layout(location = 5) out vec4 f_clip_future_pos;
layout(location = 7) out vec4 UserData;

uint hash(uint x)
{
	x ^= x >> 16;
	x *= 0x7feb352dU;
	x ^= x >> 15;
	x *= 0x846ca68bU;
	x ^= x >> 16;
	return x;
}

float random(inout uint state)
{
	state = hash(state);
	return float(state >> 8) * (1.0 / 16777216.0);
}

// the share of the blades drawn at specified distance for speed: fewer further away, each standing for more of them
float lod_thinning(float distance)
{
	return distance <= grass_lod.x ? 1.0 : pow(grass_lod.x / distance, 1.5);
}

// the share of the blades drawn: the above, and fewer and fewer over the fading distance, where the grass really gets
// sparser; the same as lod_share() of opengl46grass.cpp
float lod_share(float distance)
{
	return lod_thinning(distance) * (1.0 - smoothstep(grass_lod.y, grass_lod.z, distance));
}

// height of the ground at a point of the chunk and its normal, the way the full grid is triangulated
float ground(vec2 point, out vec3 normal)
{
	float cells = grass_chunk.y;
	float spacing = grass_chunk.x / cells;
	vec2 place = clamp(point / spacing, vec2(0.0), vec2(cells));
	ivec2 cell = min(ivec2(place), ivec2(int(cells) - 1));
	vec2 t = place - vec2(cell);
	float h00 = texelFetch(grass_heights, cell, 0).r;
	float h10 = texelFetch(grass_heights, cell + ivec2(1, 0), 0).r;
	float h01 = texelFetch(grass_heights, cell + ivec2(0, 1), 0).r;
	float h11 = texelFetch(grass_heights, cell + ivec2(1, 1), 0).r;
	// the quads are cut along the diagonal from (x0, z1) to (x1, z0), as the mesh is made
	if (t.x + t.y <= 1.0)
	{
		normal = normalize(vec3(-(h10 - h00) / spacing, 1.0, -(h01 - h00) / spacing));
		return h00 + t.x * (h10 - h00) + t.y * (h01 - h00);
	}
	normal = normalize(vec3(-(h11 - h01) / spacing, 1.0, -(h11 - h10) / spacing));
	return h11 + (1.0 - t.x) * (h01 - h11) + (1.0 - t.y) * (h10 - h11);
}

void collapse()
{
	gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
	f_normal = vec3(0.0, 1.0, 0.0);
	f_pos = vec4(0.0);
	f_albedo = vec3(0.0);
	f_clip_pos = gl_Position;
	f_clip_future_pos = gl_Position;
	UserData = vec4(0.0);
}

void main()
{
	int blade = gl_InstanceID;
	// the numbers of the blade, drawn in the same order whatever becomes of them
	uint state = hash(uint(blade) * 0x9E3779B9U ^ hash(uint(int(grass_part.z)) * 0x85EBCA6BU ^ hash(uint(int(grass_part.w)) + 0x27D4EB2FU)));
	uint tilestate = hash(uint(int(grass_part.z)) * 0x85EBCA6BU ^ hash(uint(int(grass_part.w)) + 0x165667B1U));
	vec2 shift = vec2(random(tilestate), random(tilestate));
	vec2 jitter = vec2(random(state), random(state)) - 0.5;
	float pick = random(state);
	float heightrandom = random(state) * 2.0 - 1.0;
	float widthrandom = random(state);
	float turnrandom = random(state) * 6.2831853;
	float leanrandom = random(state) * 6.2831853;
	float leanamount = random(state);
	float shade = random(state);
	float phase = random(state);
	float early = random(state);

	// place: R2 spreads the first blades evenly at any count, a little jitter breaks its lattice
	vec2 spot = fract(shift + float(blade) * vec2(0.7548776662466927, 0.5698402909980532));
	vec2 point = grass_tile.xy + fract(spot + jitter * (0.7 / sqrt(grass_tile.w))) * grass_tile.z;
	if (any(lessThan(point, vec2(0.0))) || any(greaterThanEqual(point, vec2(grass_chunk.x))))
	{
		collapse();
		return;
	}
	vec2 local = point / grass_chunk.x;
	float samples = grass_chunk.z;
	vec4 density = texture(grass_density, (local * samples + 0.5) / (samples + 1.0));
	float holeside = float(textureSize(grass_holes, 0).x);
	float hole = texture(grass_holes, (local * (holeside - 1.0) + 0.5) / holeside).r;

	// the kind: the blades of the tile are shared among the kinds by their densities here
	vec4 cover = density * grass_share / max(grass_part.x, 1e-3);
	float covered = cover.x + cover.y + cover.z + cover.w;
	if (covered > 1.0)
		cover /= covered;
	int kind = -1;
	float sum = 0.0;
	for (int i = 0; i < 4; ++i)
	{
		sum += cover[i];
		if (pick < sum)
		{
			kind = i;
			break;
		}
	}

	vec3 groundnormal;
	vec3 root = vec3(point.x, ground(point, groundnormal), point.y);
	float distance = length(root - grass_camera.xyz);
	float share = lod_share(distance);
	float rank = (float(blade) + 0.5) / grass_tile.w;
	if (kind < 0 || hole > 0.5 || rank >= share || distance > grass_lod.z)
	{
		collapse();
		return;
	}

	vec4 shape = grass_shape[kind];
	float height = shape.x * (1.0 + shape.y * heightrandom);
	// the last of the blades drawn at the distance grow in, and the grass fades out at the end of its reach
	height *= 1.0 - smoothstep(share * 0.8, share, rank);
	// lower in winter, growing in spring
	float seasonal = grass_dry[kind].a;
	height *= mix(1.0, grass_season.z, seasonal);
	// where the paint thins out the blades get shorter
	height *= mix(0.5, 1.0, clamp(density[kind] * 1.5, 0.0, 1.0));
	// the blades drawn further away stand for more of them, so they're wider
	float width = shape.z * (0.8 + 0.4 * widthrandom) / sqrt(lod_thinning(distance));

	// the face of the blade turns towards the camera further away, so the blades don't vanish edge on
	vec2 facing = vec2(cos(turnrandom), sin(turnrandom));
	vec2 toview = grass_camera.xz - root.xz;
	toview = toview / max(length(toview), 1e-3);
	facing = mix(facing, toview, smoothstep(grass_lod.x * 0.5, grass_lod.x * 2.0, distance));
	facing = facing / max(length(facing), 1e-3);
	vec2 across = vec2(-facing.y, facing.x);

	// the lean of the blade and the wind: a steady push along it, with gusts running across the field, and a flutter
	vec2 lean = vec2(cos(leanrandom), sin(leanrandom)) * (shape.w * (0.4 + 0.6 * leanamount));
	float gust = 0.5 + 0.5 * sin(grass_wind.w * 0.9 - dot(root.xz, grass_wind.xy) * 0.12);
	float flutter = sin(grass_wind.w * (2.0 + phase) + phase * 6.2831853);
	vec2 sway = grass_wind.xy * grass_wind.z * (0.15 + 0.45 * gust + 0.08 * flutter);
	vec2 bend = lean + sway;
	// a long lean takes some of the height
	float bent = min(length(bend), 0.9);

	int segments = int(grass_part.y);
	int level = min(gl_VertexID / 2, segments);
	float side = (gl_VertexID >= segments * 2) ? 0.0 : ((gl_VertexID & 1) == 0 ? -0.5 : 0.5);
	float t = float(level) / float(segments);
	float halfwidth = width * (1.0 - pow(t, 1.5));
	vec2 xz = root.xz + across * (side * halfwidth) + bend * (height * t * t);
	float y = root.y - 0.02 + height * t * (1.0 - 0.35 * bent * t);
	vec3 position = vec3(xz.x, y, xz.y);

	// lit mostly as the ground under it, which keeps a field of blades from shimmering
	vec3 normal = normalize(groundnormal * 0.8 + vec3(facing.x, 0.0, facing.y) * (side == 0.0 ? 0.0 : 0.25) + vec3(0.0, 0.3, 0.0));
	// the seasons: some blades dry out (and green up) earlier than others, so a field turns gradually; a dry blade takes the
	// dry colour, darker at its root. in spring the green is fresher
	float dry = clamp(grass_season.x * seasonal * (0.6 + 0.8 * early), 0.0, 1.0);
	vec3 bottom = mix(grass_base[kind].rgb, grass_dry[kind].rgb * 0.45, dry);
	vec3 top = mix(grass_tip[kind].rgb, grass_dry[kind].rgb, dry);
	top *= mix(vec3(1.0), vec3(1.08, 1.22, 0.85), grass_season.y * seasonal * (1.0 - dry));
	vec3 albedo = mix(bottom, top, t) * (0.85 + 0.3 * shade);
	// the lower part of the blades is shaded by the others
	albedo *= mix(0.7, 1.0, t);

	f_pos = modelview * vec4(position, 1.0);
	f_normal = normalize(modelviewnormal * normal);
	f_albedo = albedo;
	f_clip_pos = projection * f_pos;
	f_clip_future_pos = (projection * future) * f_pos;
	gl_Position = f_clip_pos;
	UserData = vec4(0.0);
}
