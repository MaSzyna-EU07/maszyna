#version 460 core
#define STAGE_FRAGMENT 1
layout(location = 14) in vec2 f_coords;
layout(location = 0) out vec4 out_color;

#pragma texture(ssao_tex, 0, R)
layout(binding = 0) uniform sampler2D ssao_tex;

void main() {
    vec2 texel = 1.0 / vec2(textureSize(ssao_tex, 0));
    float sum = 0.0;
    for (int x = -2; x < 2; ++x)
    for (int y = -2; y < 2; ++y)
        sum += texture(ssao_tex, f_coords + vec2(x, y) * texel).r;
    out_color = vec4(sum / 16.0);
}