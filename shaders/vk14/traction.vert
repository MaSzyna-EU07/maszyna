#version 460 core
#define STAGE_VERTEX 1
#include "common.glsl"
layout(location = 0) in vec3 v_vert;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec2 v_coord;


layout(location = 4) out vec4 f_clip_pos;
layout(location = 5) out vec4 f_clip_future_pos;
layout(location = 3) out vec4 f_pos;

#include "wire_sway.glsl"

void main()
{
	// the wind blowing the span aside, and the iced wires drawn a pixel wider
	vec3 vertex = wire_sway(v_vert, v_normal, v_coord);
	vec4 view = modelview * vec4(vertex, 1.0f);
	view.xyz += wire_pixel(view.xyz);
	f_pos = view;
	f_clip_pos = projection * view;
	f_clip_future_pos = (projection * future) * view;
	
	gl_Position = f_clip_pos;
}
