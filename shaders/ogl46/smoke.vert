#version 460 core
#define STAGE_VERTEX 1
layout(location = 0) in vec3 v_vert;
layout(location = 1) in vec4 v_color;
layout(location = 2) in vec2 v_coord;

layout(location = 3) out vec4 f_pos;
layout(location = 13) out vec4 f_color;
layout(location = 2) out vec2 f_coord;

layout(location = 4) out vec4 f_clip_pos;
layout(location = 5) out vec4 f_clip_future_pos;

#include "common.glsl"

void main()
{
	f_pos = modelview * vec4(v_vert, 1.0f);
	f_clip_pos = (projection * modelview) * vec4(v_vert, 1.0f);
	f_clip_future_pos = (projection * future * modelview) * vec4(v_vert, 1.0f);

	gl_Position = f_clip_pos;
	f_coord = v_coord;
	f_color = v_color;
}
