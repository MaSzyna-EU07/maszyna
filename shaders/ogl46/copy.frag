#version 460 core
#define STAGE_FRAGMENT 1
layout(location = 14) in vec2 f_coords;

#pragma texture(tex1, 0, sRGB)
layout(binding = 0) uniform sampler2D tex1;

layout(location = 0) out vec4 out_color;

void main()
{
	out_color = texture(tex1, f_coords);
}
