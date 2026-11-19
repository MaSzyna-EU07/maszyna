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
	// the wind blowing the span aside
	vec3 vertex = wire_sway(v_vert, v_coord);
	f_pos = modelview * vec4(vertex, 1.0f);
	f_clip_pos = (projection * modelview) * vec4(vertex, 1.0f);
	f_clip_future_pos = (projection * future * modelview) * vec4(vertex, 1.0f);
	
	gl_Position = f_clip_pos;
}
