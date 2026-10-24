// a star: a soft point added to the sky

in vec3 f_color;

#include <common>
#include <tonemapping.glsl>

layout(location = 0) out vec4 out_color;
#if MOTIONBLUR_ENABLED
layout(location = 1) out vec4 out_motion;
#endif

void main()
{
	vec2 offset = gl_PointCoord - 0.5;
	float spot = exp(-dot(offset, offset) * 14.0);
	vec3 color = f_color * spot;
#if POSTFX_ENABLED
	out_color = vec4(color, 0.0);
#else
	out_color = vec4(tonemap(vec4(color, 1.0)).rgb, 0.0);
#endif
#if MOTIONBLUR_ENABLED
	out_motion = vec4(0.0);
#endif
}
