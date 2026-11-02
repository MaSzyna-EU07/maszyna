#version 460 core
#define STAGE_FRAGMENT 1
// animals and people animated by textures (mat_vat.vert moves the vertices): drawn as mat_default. the textures and the param of
// the animation are declared here too, as in the vertex shader
#pragma texture(vatpos_hi, 1, RGBA)
#pragma texture(vatpos_lo, 2, RGBA)
#pragma texture(vatnormal, 3, RGBA)
#pragma param(vat_speed, 2, 0, 1, one)
#include "mat_default.glsl"
