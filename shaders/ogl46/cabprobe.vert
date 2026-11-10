#version 460 core
#define STAGE_VERTEX 1
// the occupied cab drawn into the six faces of its probe at once (gfx::cab_probe, cabprobe.geom picks the faces). the modelview
// takes the cab model to the space of the probe: the axes of the model, the origin in the middle of the box of the cab
layout(location = 0) in vec3 v_vert;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec2 v_coord;

layout(location = 20) out vec3 g_pos;
layout(location = 21) out vec3 g_normal;
layout(location = 18) out vec2 g_coord;

#include "common.glsl"

void main()
{
	g_pos = (effective_modelview() * vec4(v_vert, 1.0)).xyz;
	g_normal = effective_modelviewnormal() * v_normal;
	g_coord = v_coord;
	gl_Position = vec4(g_pos, 1.0);
}
