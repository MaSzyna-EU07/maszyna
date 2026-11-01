#version 460 core
#define STAGE_FRAGMENT 1
layout(location = 14) in vec2 f_coords;
layout(location = 0) out vec4 out_color;

#pragma texture(color_tex, 0, RGB)
layout(binding = 0) uniform sampler2D color_tex;

#pragma texture(velocity_tex, 1, RG)
layout(binding = 1) uniform sampler2D velocity_tex;

#pragma texture(depth_tex, 2, R)
layout(binding = 2) uniform sampler2D depth_tex;

#include "common.glsl"
#include "rain_windscreen_haze.glsl"

void main()
{
    // the hazy wet windscreen (mat_rain_windscreen.frag): blurred here without the taa, left as it is when the taa blurred it
    vec2 rawv = texture(velocity_tex, f_coords).xy;
    if (wsIsDrop(rawv))
    {
        vec4 picture = texture(color_tex, f_coords);
        out_color = vec4((picture.a > 0.5 * (1.0 + WS_HAZED)) ? picture.rgb
                                                              : wsDropRefraction(color_tex, velocity_tex, depth_tex, f_coords, rawv), 1.0);
        return;
    }
    float blurAmt = wsBlur(rawv);
    if (blurAmt > 0.0)
    {
        vec4 picture = texture(color_tex, f_coords);
        out_color = vec4((picture.a > 0.5 * (1.0 + WS_HAZED)) ? picture.rgb : wsHaze(color_tex, velocity_tex, f_coords, blurAmt), 1.0);
        return;
    }

    float ori_depth = texture(depth_tex, f_coords).x;

    vec2 texelSize = 1.0 / vec2(textureSize(color_tex, 0));
    vec2 velocity = rawv * param[0].x;
    // edge pixels of the windscreen can carry a fraction of a blur request: never use that as motion
    if (any(greaterThan(abs(rawv), vec2(WS_FLOOR * WS_ENC))))
        velocity = vec2(0.0);
    
    float speed = length(velocity / texelSize);
    int nSamples = clamp(int(speed), 1, 32);
    
    float depth_diff = 0.001 * 8.0;
    vec4 result = vec4(0.0);

    result += vec4(texture(color_tex, f_coords).rgb, 1.0);

    for (int i = 1; i < nSamples; ++i)
    {
        vec2 offset = velocity * (float(i) / float(nSamples - 1) - 0.5);

        vec2 coord = f_coords + offset;
        // a sample off the picture would read its edge row again and again: a fast movement towards the bottom
        // of the screen (the ground racing under a low free camera) smeared it into a bright band there
        if (any(lessThan(coord, 0.5 * texelSize)) || any(greaterThan(coord, vec2(1.0) - 0.5 * texelSize)))
            continue;
        float depth = texture(depth_tex, coord).x;

        if (abs(depth - ori_depth) < depth_diff)
            result += vec4(texture(color_tex, coord).rgb, 1.0);
    }

    out_color = vec4((result.rgb / result.w), 1.0);
}
