#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
layout(location = 2) in vec2 f_coord;

#pragma texture(tex1, 0, sRGB_A)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 0) uniform sampler2D tex1;

void main()
{
	if (texture(tex1, f_coord).a < 0.5f)
		discard;
}
