#version 460 core
#define STAGE_VERTEX 1
layout(location = 0) in vec3 v_vert;
layout(location = 2) in vec2 v_coord;

layout(location = 2) out vec2 f_coord;

#include "common.glsl"

void main()
{
	gl_Position = (projection * effective_modelview()) * vec4(v_vert, 1.0f);
	f_coord = v_coord;
#ifdef MODEL_DRAWS
	f_model_draw = uint(gl_DrawID);
#endif
}
