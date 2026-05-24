in vec2 f_uv;
in vec4 f_pos;

#texture (tex1, 0, sRGB_A)
uniform sampler2D tex1;

#include <common>
#include <apply_fog.glsl>
#include <tonemapping.glsl>

layout(location = 0) out vec4 out_color;
#if MOTIONBLUR_ENABLED
layout(location = 1) out vec4 out_motion;
#endif

void main()
{
    vec4 col = texture(tex1, f_uv);
    if (col.a < 0.01) discard;
#if POSTFX_ENABLED
    out_color = vec4(apply_fog(col.rgb), col.a);
#else
    out_color = tonemap(vec4(apply_fog(col.rgb), col.a));
#endif
#if MOTIONBLUR_ENABLED
    out_motion = vec4(0.0);
#endif
}
