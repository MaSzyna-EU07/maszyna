// depth prepass of the instances drawn from the gpu culling (opengl46_renderer::Render_Instances_Indirect()).
// the position is worked out the way vertex.vert of the materials does it, and declared invariant in both, so the colour
// pass which follows gets the same depth for the same surface

layout(location = 0) in vec3 v_vert;
layout(location = 2) in vec2 v_coord;

#include <common>

invariant gl_Position;

out vec2 f_coord;

void main()
{
	mat4 mv = effective_modelview();
	vec4 position = mv * vec4(v_vert, 1.0);
	gl_Position = projection * position;
	f_coord = v_coord;
}
