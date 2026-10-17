layout(location = 0) in vec3 v_vert;
layout(location = 2) in vec2 v_coord;

// position in the space of the cube shadow map being drawn (origin at the lamp), see shadowcube.geom
out vec3 g_pos;
out vec2 g_coord;

#include <common>

void main()
{
	g_pos = (effective_modelview() * vec4(v_vert, 1.0f)).xyz;
	g_coord = v_coord;
	gl_Position = vec4(g_pos, 1.0f);
}
