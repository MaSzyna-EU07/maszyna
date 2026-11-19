#version 460 core
// the shadow of a span of the catenary: simpleuv.vert with the wires blown aside by the wind as they are drawn (wire_sway.glsl)
#define STAGE_VERTEX 1
layout(location = 0) in vec3 v_vert;
layout(location = 2) in vec2 v_coord;

layout(location = 2) out vec2 f_coord;

#include "common.glsl"

#include "wire_sway.glsl"

void main()
{
	gl_Position = (projection * effective_modelview()) * vec4(wire_sway(v_vert, v_coord), 1.0f);
	f_coord = v_coord;
#ifdef MODEL_DRAWS
	f_model_draw = uint(gl_DrawID);
#endif
}
