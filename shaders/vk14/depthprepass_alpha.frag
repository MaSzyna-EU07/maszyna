#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
// depth prepass of materials whose colour shader tests the alpha of the diffuse texture against the opacity of the
// material (mat_default.frag and the like). the test here is stricter by one step of 8-bit alpha: a fragment which writes
// depth here is never discarded by the colour pass, which would leave a hole with nothing drawn behind it

layout(location = 2) in vec2 f_coord;


#pragma texture(diffuse, 0, sRGB_A)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 0) uniform sampler2D diffuse;

void main()
{
	if (texture(diffuse, f_coord).a < opacity + 1.0 / 255.0)
		discard;
}
