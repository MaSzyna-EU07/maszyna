#version 460 core
#define STAGE_FRAGMENT 1
// the temporal anti-aliasing (taa_resolve.glsl). param[0].x: 1 with a history of the last frame, param[0].y: the near plane;
// future: the clip space of this frame to the one of the last frame, for the world standing still (gfx::temporal_aa::reprojection())
layout(location = 14) in vec2 f_coords;
layout(location = 0) out vec4 out_color;

#pragma texture(color_tex, 0, RGB)
layout(binding = 0) uniform sampler2D color_tex;

#pragma texture(velocity_tex, 1, RG)
layout(binding = 1) uniform sampler2D velocity_tex;

#pragma texture(depth_tex, 2, R)
layout(binding = 2) uniform sampler2D depth_tex;

#pragma texture(history_tex, 3, RGBA)
layout(binding = 3) uniform sampler2D history_tex;

#include "common.glsl"

#define TAA_HISTORY_VALID (param[0].x > 0.5)
#define TAA_REPROJECTION future
#define TAA_NEAR param[0].y
#include "taa_resolve.glsl"
