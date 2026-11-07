#version 460 core
#define STAGE_VERTEX 1
layout(location = 0) in vec3 v_vert;
layout(location = 1) in vec3 v_color;

layout(location = 13) out vec3 f_color;
layout(location = 3) out vec4 f_pos;
layout(location = 4) out vec4 f_clip_pos;

#include "common.glsl"

void main()
{
	f_pos = modelview * vec4(v_vert, 1.0);
	f_clip_pos = (projection * modelview) * vec4(v_vert, 1.0);
	f_color = pow(v_color, vec3(1.0));

	gl_Position = f_clip_pos;
}
