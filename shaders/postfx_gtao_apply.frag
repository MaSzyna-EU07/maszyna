in vec2 f_coords;

layout(location = 0) out vec4 out_color;

#texture (ao_tex, 0, R)
uniform sampler2D ao_tex;

#include <common>

void main()
{
	out_color = vec4(vec3(pow(clamp(texture(ao_tex, f_coords).r, 0.0, 1.0), param[0].y)), 1.0);
}
