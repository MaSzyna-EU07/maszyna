// debug view of the light clusters: tints every pixel of the scene with the colour of its depth slice,
// and darkens every other screen tile to show the grid. drawn over the resolved image, see gl/cluster_grid.h

in vec2 f_coords;

#include <common>

#texture (depth_tex, 0, R)
uniform sampler2D depth_tex;

layout(location = 0) out vec4 out_color;

const float TINT = 0.55;

// blue (near) through green to red (far)
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

	vec3 color = ramp(float(slice) / float(CLUSTER_GRID_Z - 1));
	if (((tile.x + tile.y) & 1) != 0)
		color *= 0.8;

	out_color = vec4(color, TINT);
}
