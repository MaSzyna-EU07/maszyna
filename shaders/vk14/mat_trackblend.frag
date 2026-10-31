#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
// the rails and the ballast of a track faded into those of the tracks joined at its ends, see trackblend.glsl
#pragma texture(diffuse, 0, sRGB_A)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 0) uniform sampler2D diffuse;
#pragma texture(diffuse1, 1, sRGB_A)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 1) uniform sampler2D diffuse1;
#pragma texture(diffuse2, 2, sRGB_A)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 2) uniform sampler2D diffuse2;
#pragma texture(diffuse3, 3, sRGB_A)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 3) uniform sampler2D diffuse3;
#include "trackblend.glsl"
