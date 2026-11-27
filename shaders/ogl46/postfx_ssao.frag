#version 460 core
#define STAGE_FRAGMENT 1
layout(location = 14) in vec2 f_coords;
layout(location = 0) out vec4 out_color;

#pragma texture(depth_tex, 0, R)
layout(binding = 0) uniform sampler2D depth_tex;

#pragma texture(noise_tex, 1, RGB)
layout(binding = 1) uniform sampler2D noise_tex;

#include "common.glsl"

// the depth is reversed (glClipControl GL_ZERO_TO_ONE, 1 at the near plane, 0 at infinity), so it is the clip z itself
vec3 view_pos_from_depth(vec2 uv, float depth) {
    vec4 clip = vec4(uv * 2.0 - 1.0, depth, 1.0);
    vec4 view = inverse(projection) * clip;
    return view.xyz / view.w;
}

// Deterministic hemisphere sample (z > 0), mimics the C++ kernel generator
vec3 kernel_sample(int i) {
    float fi = float(i);
    vec3 dir = normalize(vec3(
        fract(sin(fi * 12.9898) * 43758.5453) * 2.0 - 1.0,
        fract(sin(fi * 78.2330) * 43758.5453) * 2.0 - 1.0,
        fract(sin(fi * 37.7190) * 43758.5453)           // z >= 0 -> hemisphere
    ));
    float len   = fract(sin(fi * 94.6720) * 43758.5453);
    float scale = fi / 32.0;
    scale = mix(0.1, 1.0, scale * scale);               // bias samples inward
    return dir * len * scale;
}

void main() {
    float d = texture(depth_tex, f_coords).r;
    if (d <= 0.0) { out_color = vec4(1.0); return; }   // skybox -> no occlusion

    vec3 pos = view_pos_from_depth(f_coords, d);

    vec3 ddx = dFdx(pos);
    vec3 ddy = dFdy(pos);
    vec3 n   = normalize(cross(ddx, ddy));

    // derive screen size from the depth texture instead of a UBO field
    vec2 screen_size = vec2(textureSize(depth_tex, 0));
    vec2 noise_uv    = f_coords * (screen_size / 4.0);
    vec3 rvec        = texture(noise_tex, noise_uv).xyz;

    vec3 t = normalize(rvec - n * dot(rvec, n));
    vec3 b = cross(n, t);
    mat3 TBN = mat3(t, b, n);

    const int   KERNEL = 32;
    const float RADIUS = 0.5;
    const float BIAS   = 0.025;

    float occ = 0.0;
    for (int i = 0; i < KERNEL; ++i) {
        vec3 sp = TBN * kernel_sample(i);
        sp = pos + sp * RADIUS;

        vec4 clip = projection * vec4(sp, 1.0);
        vec3 ndc  = clip.xyz / clip.w;
        vec2 suv  = ndc.xy * 0.5 + 0.5;

        float sd  = texture(depth_tex, suv).r;
        float szv = view_pos_from_depth(suv, sd).z;

        float range = smoothstep(0.0, 1.0, RADIUS / abs(pos.z - szv));
        occ += (szv >= sp.z + BIAS ? 1.0 : 0.0) * range;
    }

    out_color = vec4(1.0 - occ / float(KERNEL));
}