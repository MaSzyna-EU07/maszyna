in vec3 f_color;
in vec4 f_pos;

#include <common>
#include <apply_fog.glsl>
#include <tonemapping.glsl>

layout(location = 0) out vec4 out_color;
#if MOTIONBLUR_ENABLED
layout(location = 1) out vec4 out_motion;
#endif

void main()
{
    vec3 col = f_color;
#if POSTFX_ENABLED
    out_color = vec4(apply_fog(col), 1.0);
#else
    out_color = tonemap(vec4(apply_fog(col), 1.0));
#endif
#if MOTIONBLUR_ENABLED
    out_motion = vec4(0.0);
#endif
}
