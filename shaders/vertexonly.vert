layout(location = 0) in vec3 v_vert;

#include <common>

void main()
{
	gl_Position = (projection * effective_modelview()) * vec4(v_vert, 1.0);
}
