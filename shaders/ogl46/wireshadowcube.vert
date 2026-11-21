#version 460 core
// the shadow of a span of the catenary in the map of a lamp: shadowcube.vert with the wires blown aside by the wind as they are drawn (wire_sway.glsl)
#define STAGE_VERTEX 1
layout(location = 0) in vec3 v_vert;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec2 v_coord;

// position in the space of the cube shadow map being drawn (origin at the lamp), see shadowcube.geom
layout(location = 20) out vec3 g_pos;
layout(location = 18) out vec2 g_coord;

#include "common.glsl"

#include "wire_sway.glsl"

void main()
{
	g_pos = (effective_modelview() * vec4(wire_sway(v_vert, v_normal, v_coord), 1.0f)).xyz;
	g_coord = v_coord;
	gl_Position = vec4(g_pos, 1.0f);
}
