#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"

layout(location = 0) out vec4 out_color;

void main()
{
	out_color = vec4(vec3(cascade_end), 1.0f);
}
