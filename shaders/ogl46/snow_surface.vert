#version 460 core
#define STAGE_VERTEX 1
// the surface of the drifts (opengl46_snow): a grid over the maps around the camera, a vertex in the middle of every texel, at the
// height of the surface of the snow there (snow.glsl). made of the numbers of the vertices, two triangles a square of four texels;
// snow_surface.geom leaves out the triangles over the places with too little snow and across the steps of the scenery (the snow
// lies on a roof and on the ground beside the house, not on a sheet hung between them). drawn with the camera of the pass at the
// origin; it sinks into the ground towards the edge of the maps

layout(location = 0) out vec3 f_normal;
layout(location = 3) out vec4 f_pos;
layout(location = 4) out vec4 f_clip_pos;
layout(location = 5) out vec4 f_clip_future_pos;
layout(location = 7) out vec4 UserData;
// x: the depth, y: the kind (snow.glsl), z: 1 where the vertex is to be drawn, w: the height of the surface of the snow over the
// camera, m, which snow_surface.geom compares across a triangle
layout(location = 12) out vec4 f_snow;

#include "common.glsl"
#include "weather_common.glsl"
#include "snow.glsl"

// x: the texels across the grid (fewer than across the maps, which reach further than they're filled), y: the texels skipped
// between two vertices
layout(location = 0) uniform ivec4 grid;

const int CORNERS[6] = int[](0, 1, 2, 2, 1, 3);

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

void main()
{
	int across = grid.x;
	int stride = max(grid.y, 1);
	int quad = gl_VertexID / 6;
	int corner = CORNERS[gl_VertexID % 6];
	ivec2 cellindex = ivec2(quad % (across - 1), quad / (across - 1)) + ivec2(corner & 1, corner >> 1);
	float texel = texel_size();
	// the texel of the camera in the maps, and the vertex around it
	ivec2 camera = ivec2(floor(sky_occlusion_origin.xy / texel));
	ivec2 local = (cellindex - ivec2(across / 2)) * stride;
	ivec2 t = camera + local;
	vec4 cell = cell_at(t);
	vec2 place = (vec2(t) + 0.5) * texel - sky_occlusion_origin.xy;
	float ground = cell.x - cell.y;
	// towards the edge of the maps the snow sinks into the ground, and the thin snow is left to the surfaces under it the farther the
	// more, so the two don't fight over the depth
	float away = length(place);
	float edge = 1.0 - smoothstep(sky_occlusion_params.z * 0.85, sky_occlusion_params.z, max(abs(place.x - sky_occlusion_center.x), abs(place.y - sky_occlusion_center.y)));
	float least = 0.04 + away * 0.0006;
	float drawn = ((cell.x > SNOW_NONE) && (cell.y * edge > least)) ? 1.0 : 0.0;
	float height = ground + cell.y * edge;

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
}
