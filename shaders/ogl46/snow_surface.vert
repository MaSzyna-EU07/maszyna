#version 460 core
#define STAGE_VERTEX 1
// the surface of the drifts (opengl46_snow): a grid over the maps around the camera, a vertex in the middle of every texel, at the
// height of the surface of the snow there (snow.glsl). made of the numbers of the vertices, two triangles a square of four texels,
// the squares numbered by blocks of the grid (opengl46_snow draws the blocks in the view only). the triangles over the places with
// too little snow and across the steps of the scenery (the snow lies on a roof and on the ground beside the house, not on a sheet
// hung between them) are left out: each vertex looks at the three corners of its triangle, and all three put a triangle left out
// at the same place outside of the view, where it has no area to draw. drawn with the camera of the pass at the origin; it sinks
// into the ground towards the edge of the maps

layout(location = 0) out vec3 f_normal;
layout(location = 3) out vec4 f_pos;
layout(location = 4) out vec4 f_clip_pos;
layout(location = 5) out vec4 f_clip_future_pos;
layout(location = 7) out vec4 UserData;
// x: the depth, y: the kind (snow.glsl), z: 1 where the vertex is to be drawn, w: the height of the surface of the snow over the
// camera, m
layout(location = 12) out vec4 f_snow;

#include "common.glsl"
#include "weather_common.glsl"
#include "snow.glsl"

// x: the vertices across the grid (fewer than the texels across the maps, which reach further than they're filled), y: the texels
// skipped between two vertices, z: the squares across a block of the grid, w: the blocks across the grid
layout(location = 0) uniform ivec4 grid;

const int CORNERS[6] = int[](0, 1, 2, 2, 1, 3);
// m the surface may rise over a texel, with what the steepest drifts and slopes of the ground make
const float RISE_PER_METRE = 1.5;
const float RISE = 0.35;

float texel_size()
{
	return sky_occlusion_params.x / sky_occlusion_params.y;
}

vec4 cell_at(ivec2 texel)
{
	int side = int(sky_occlusion_params.y + 0.5);
	ivec2 t = ((texel % side) + side) % side;
	return snow_cells[t.y * side + t.x];
}

// the height of the surface of the snow at a texel, or Fallback where there's none
float surface_at(ivec2 texel, float fallback)
{
	vec4 cell = cell_at(texel);
	return ((cell.x < SNOW_NONE) || (abs(cell.x - fallback) > 2.0)) ? fallback : cell.x;
}

// the texel of a corner (0..3) of the square Quad of the grid, the camera at the texel Camera
ivec2 corner_texel(ivec2 quad, int corner, ivec2 camera)
{
	ivec2 cellindex = quad + ivec2(corner & 1, corner >> 1);
	return camera + (cellindex - ivec2(grid.x / 2)) * max(grid.y, 1);
}

// the place of a texel relative to the camera, m across the ground
vec2 place_of(ivec2 t, float texel)
{
	return (vec2(t) + 0.5) * texel - sky_occlusion_origin.xy;
}

// how much of the snow at a texel is drawn: towards the edge of the maps the snow sinks into the ground, and the thin snow is left to
// the surfaces under it the farther the more, so the two don't fight over the depth
float edge_of(vec2 place)
{
	return 1.0 - smoothstep(sky_occlusion_params.z * 0.85, sky_occlusion_params.z, max(abs(place.x - sky_occlusion_center.x), abs(place.y - sky_occlusion_center.y)));
}

// -1 where nothing is known (the triangle is left out), else 1 with enough snow and 0 without, interpolated across a triangle: the
// fragments cut the surface where it falls under a half, so its edge runs across the texels (along a track at any angle) instead of
// going in steps of whole triangles
float drawn_at(vec4 cell, vec2 place)
{
	float least = 0.04 + length(place) * 0.0006;
	return (cell.x > SNOW_NONE) ? ((cell.y * edge_of(place) > least) ? 1.0 : 0.0) : -1.0;
}

// whether the triangle with the corners First..First+2 of CORNERS over the square Quad is drawn: within the grid, something known
// under each corner, enough snow under one at least, and no step of the scenery across it
bool triangle_kept(ivec2 quad, int first, ivec2 camera, float texel)
{
	if ((quad.x >= grid.x - 1) || (quad.y >= grid.x - 1))
		return false;
	float low = 1.0e9;
	float high = -1.0e9;
	bool anysnow = false;
	for (int i = 0; i < 3; ++i)
	{
		ivec2 t = corner_texel(quad, CORNERS[first + i], camera);
		vec4 cell = cell_at(t);
		float drawn = drawn_at(cell, place_of(t, texel));
		if (drawn < -0.5)
			return false;
		anysnow = anysnow || (drawn > 0.5);
		low = min(low, cell.x);
		high = max(high, cell.x);
	}
	return anysnow && (high - low <= texel * float(max(grid.y, 1)) * RISE_PER_METRE + RISE);
}

void main()
{
	int blocksize = max(grid.z, 1);
	int squares = blocksize * blocksize;
	int quadindex = gl_VertexID / 6;
	int inquad = gl_VertexID % 6;
	// the square: its block, and its place in the block
	int blockindex = quadindex / squares;
	int inblock = quadindex % squares;
	ivec2 quad = ivec2(blockindex % max(grid.w, 1), blockindex / max(grid.w, 1)) * blocksize + ivec2(inblock % blocksize, inblock / blocksize);
	int stride = max(grid.y, 1);
	float texel = texel_size();
	// the texel of the camera in the maps, and the vertex around it
	ivec2 camera = ivec2(floor(sky_occlusion_origin.xy / texel));
	ivec2 t = corner_texel(quad, CORNERS[inquad], camera);
	vec4 cell = cell_at(t);
	vec2 place = place_of(t, texel);
	float ground = cell.x - cell.y;
	float drawn = drawn_at(cell, place);
	float height = ground + cell.y * edge_of(place);

	// the normal from the heights around
	float h = cell.x;
	float hl = surface_at(t - ivec2(stride, 0), h);
	float hr = surface_at(t + ivec2(stride, 0), h);
	float hd = surface_at(t - ivec2(0, stride), h);
	float hu = surface_at(t + ivec2(0, stride), h);
	float span = 2.0 * texel * float(stride);
	vec3 normal = normalize(vec3((hl - hr) / span, 1.0, (hd - hu) / span));

	vec3 position = vec3(place.x, height - sky_occlusion_origin.z, place.y);
	f_pos = modelview * vec4(position, 1.0);
	f_normal = normalize(modelviewnormal * normal);
	f_clip_pos = projection * f_pos;
	f_clip_future_pos = (projection * future) * f_pos;
	gl_Position = f_clip_pos;
	UserData = vec4(0.0);
	f_snow = vec4(cell.y, cell.w, drawn, cell.x - sky_occlusion_origin.z);
	// the triangle left out: all three of its vertices at the same place out of the view
	if (false == triangle_kept(quad, (inquad < 3) ? 0 : 3, camera, texel))
		gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
}
