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
#include "rain_windscreen_haze.glsl"

// the resolve of the taa under another name: main() below gives the hazy windscreen its blur first
#define main taa_resolve_main
#include "taa_resolve.glsl"
#undef main

// the drops and the haze are made from the frame alone (the history of the taa would smear them over the frames): drawn with the
// offsets of the taa they shimmer from frame to frame. they stay where they are on the screen, so the result of the last frame at
// the same pixel is blended in: fully while the view stands still, half while it moves, kept near this frame's result (no ghosts
// of a wiper or of drops gone)
const float WS_TEMPORAL_SHARE = 0.2;    // the share of this frame while the view stands still
const float WS_TEMPORAL_MOVING = 0.5;   // ... and while it moves
const float WS_TEMPORAL_RANGE = 0.12;   // the last frame kept within this of this frame (colours squeezed into 0..1)

vec3 wsStabilize(vec3 current, ivec2 pixel)
{
    if (false == TAA_HISTORY_VALID)
        return current;
    vec4 last = texelFetch(history_tex, pixel, 0);
    if (any(isnan(last)) || (false == wsWasHazed(last.a)))   // not a drop or haze pixel in the last frame
        return current;
    vec2 size = vec2(textureSize(color_tex, 0));
    float depth = texelFetch(depth_tex, pixel, 0).r;
    vec4 world = TAA_REPROJECTION * vec4(f_coords * 2.0 - 1.0, depth, 1.0);
    bool still = (world.w > 0.0) && (length((world.xy / world.w * 0.5 + 0.5 - f_coords) * size) < 0.5);
    vec3 c = taa_compress(current);
    vec3 h = clamp(taa_compress(taa_clean(last.rgb)), c - WS_TEMPORAL_RANGE, c + WS_TEMPORAL_RANGE);
    return taa_decompress(clamp(mix(h, c, still ? WS_TEMPORAL_SHARE : WS_TEMPORAL_MOVING), vec3(0.0), vec3(1.0 - 1.0 / TAA_HALF_MAX)));
}

void main()
{
    // the hazy wet windscreen (mat_rain_windscreen.frag): the picture of this frame blurred over the hazy glass, without the
    // history (else it would blur on frame after frame); marked for the motion blur after
    ivec2 pixel = ivec2(f_coords * vec2(textureSize(velocity_tex, 0)));
    vec2 raw = texelFetch(velocity_tex, pixel, 0).xy;
    if (wsIsDrop(raw))
    {
        out_color = vec4(wsStabilize(taa_clean(wsDropRefraction(color_tex, velocity_tex, depth_tex, f_coords, raw)), pixel), WS_HAZED);
        return;
    }
    float blurAmt = wsBlur(raw);
    if (blurAmt > 0.0)
    {
        out_color = vec4(wsStabilize(taa_clean(wsHaze(color_tex, velocity_tex, f_coords, blurAmt)), pixel), WS_HAZED);
        return;
    }
    taa_resolve_main();
}
