#version 460 core
#define STAGE_FRAGMENT 1
// a clear varnish over the surface, see mat_pbr.glsl
#define PBR_COAT
// param[2].w is the roughness of the varnish, not the weathering
#define NO_WEATHERING
#pragma param(coat, 2, 1, 1, one)
#pragma param(coat_roughness, 2, 3, 1, zero)

#pragma texture(coatmap, 4, RGBA, red)
layout(binding = 4) uniform sampler2D coatmap;
#include "mat_pbr.glsl"
