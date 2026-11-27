#version 460 core
#define STAGE_FRAGMENT 1
// a second normal map repeated over the first, see mat_pbr.glsl
#define PBR_DETAIL
#pragma param(detail_scale, 2, 1, 1, one)
#pragma param(detail_height_scale, 2, 2, 1, one)

#pragma texture(detailnormalmap, 4, RGBA)
layout(binding = 4) uniform sampler2D detailnormalmap;
#include "mat_pbr.glsl"
