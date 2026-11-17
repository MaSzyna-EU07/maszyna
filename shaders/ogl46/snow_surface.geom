#version 460 core
#define STAGE_GEOMETRY 1
// the surface of the drifts (snow_surface.vert): the triangles over the places with too little snow and across the steps of the
// scenery (the eaves of a roof, the edge of a platform, a wall) are left out; the ground and the roofs keep the snow the shading puts
// on them there

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

layout(location = 0) in vec3 g_normal[];
layout(location = 3) in vec4 g_pos[];
layout(location = 4) in vec4 g_clip_pos[];
layout(location = 5) in vec4 g_clip_future_pos[];
layout(location = 7) in vec4 g_userdata[];
layout(location = 12) in vec4 g_snow[];

layout(location = 0) out vec3 f_normal;
layout(location = 3) out vec4 f_pos;
layout(location = 4) out vec4 f_clip_pos;
layout(location = 5) out vec4 f_clip_future_pos;
layout(location = 7) out vec4 UserData;
layout(location = 12) out vec4 f_snow;

#include "common.glsl"

layout(location = 0) uniform ivec4 grid;

// m the surface may rise over a texel, with what the steepest drifts and slopes of the ground make
const float RISE_PER_METRE = 1.5;
const float RISE = 0.35;

void main()
{
	if ((g_snow[0].z < 0.5) || (g_snow[1].z < 0.5) || (g_snow[2].z < 0.5))
		return;
	float low = min(g_snow[0].w, min(g_snow[1].w, g_snow[2].w));
	float high = max(g_snow[0].w, max(g_snow[1].w, g_snow[2].w));
	float texel = sky_occlusion_params.x / max(sky_occlusion_params.y, 1.0);
	if (high - low > texel * float(max(grid.y, 1)) * RISE_PER_METRE + RISE)
		return;
	for (int i = 0; i < 3; ++i)
	{
		f_normal = g_normal[i];
		f_pos = g_pos[i];
		f_clip_pos = g_clip_pos[i];
		f_clip_future_pos = g_clip_future_pos[i];
		UserData = g_userdata[i];
		f_snow = g_snow[i];
		gl_Position = gl_in[i].gl_Position;
		EmitVertex();
	}
	EndPrimitive();
}
