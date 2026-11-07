#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
// the rails and the ballast of a track of a parallax material faded into those of the tracks joined at its ends, see trackblend.glsl
#define TRACKBLEND_NORMALMAP 1
#define TRACKBLEND_PARALLAX 1
#pragma param(height_scale, 2, 1, 1, zero)
#pragma param(height_offset, 2, 2, 1, zero)
#pragma texture(diffuse, 0, sRGB_A)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 0) uniform sampler2D diffuse;
#pragma texture(diffuse1, 1, sRGB_A)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 1) uniform sampler2D diffuse1;
#pragma texture(diffuse2, 2, sRGB_A)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 2) uniform sampler2D diffuse2;
#pragma texture(diffuse3, 3, sRGB_A)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 3) uniform sampler2D diffuse3;
#pragma texture(normalmap, 4, RGBA)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 4) uniform sampler2D normalmap;
#pragma texture(normalmap1, 5, RGBA)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 5) uniform sampler2D normalmap1;
#pragma texture(normalmap2, 6, RGBA)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 6) uniform sampler2D normalmap2;
#pragma texture(normalmap3, 7, RGBA)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 7) uniform sampler2D normalmap3;
#include "trackblend.glsl"
