#version 460 core
#define STAGE_FRAGMENT 1
#define LAYOUT_POST 1
#include "common.glsl"
layout(location = 14) in vec2 f_coords;

#pragma texture(tex1, 0, sRGB)
layout(set = POST_SET, binding = POST_TEXTURE_FIRST + 0) uniform sampler2D tex1;

layout(location = 0) out vec4 out_color;

void main()
{
	out_color = texture(tex1, f_coords);
}
