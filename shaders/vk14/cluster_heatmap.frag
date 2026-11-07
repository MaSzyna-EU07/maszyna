#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
// debug view of the light clusters: tints every pixel of the scene with the colour of the number of lights which
// were assigned to its cluster (see gl/cluster_grid.h), pixels of clusters without lights stay as they are.
// drawn over the output of the viewport, with the layout of the scene (the clusters of the pass set; the depth as the texture of
// a material)

layout(location = 14) in vec2 f_coords;

#include "common.glsl"

#pragma texture(depth_tex, 0, R)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 0) uniform sampler2D depth_tex;

layout(location = 0) out vec4 out_color;

const float TINT = 0.55;

// blue (few) through green to red (many)
vec3 ramp(float t)
{
	return clamp(vec3(1.5) - abs(4.0 * t - vec3(3.0, 2.0, 1.0)), 0.0, 1.0);
}

void main()
{
	// reverse z: 0 is the far plane at infinity, i.e. the sky, which has no cluster
	float depth = texelFetch(depth_tex, ivec2(f_coords * vec2(textureSize(depth_tex, 0))), 0).r;
	if (depth <= 0.0)
		discard;

	// depth is znear / w
	float w = cluster_znear / depth;
	int slice = clamp(int(floor(log(w) * cluster_slice_scale + cluster_slice_bias)), 0, CLUSTER_GRID_Z - 1);
	ivec2 tile = clamp(ivec2(floor(f_coords * vec2(CLUSTER_GRID_X, CLUSTER_GRID_Y))), ivec2(0), ivec2(CLUSTER_GRID_X - 1, CLUSTER_GRID_Y - 1));

	uint count = cluster_ranges[tile.x + CLUSTER_GRID_X * (tile.y + CLUSTER_GRID_Y * slice)].count;
	if (count == 0U)
		discard;

	// a full cluster (lights beyond its capacity are dropped) is white; the rest goes by the logarithm of the count
	vec3 color = (count >= uint(MAX_LIGHTS_PER_CLUSTER))
		? vec3(1.0)
		: ramp(log2(1.0 + float(count)) / log2(1.0 + float(MAX_LIGHTS_PER_CLUSTER)));

	out_color = vec4(color, TINT);
}
