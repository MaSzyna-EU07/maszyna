#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
// the height of the surface shifts the textures, see mat_pbr.glsl
#define PBR_PARALLAX
#pragma param(height_scale, 2, 1, 1, zero)

#pragma texture(heightmap, 4, R)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 4) uniform sampler2D heightmap;
#include "mat_pbr.glsl"
