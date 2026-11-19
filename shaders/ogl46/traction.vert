#version 460 core
#define STAGE_VERTEX 1
layout(location = 0) in vec3 v_vert;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec2 v_coord;

#include "common.glsl"

layout(location = 4) out vec4 f_clip_pos;
layout(location = 5) out vec4 f_clip_future_pos;
layout(location = 3) out vec4 f_pos;

void main()
{
	// the wind blowing the span aside (TTraction::sway(), the same the pantographs meet the wire at): param[1] xy the world x and z of
	// the horizontal across the span, z how far its middle goes along it, m; v_coord.x the place of the vertex along the span, 0..1
	float u = clamp(v_coord.x, 0.0, 1.0);
	vec3 vertex = v_vert + vec3(param[1].x, 0.0, param[1].y) * (param[1].z * 4.0 * u * (1.0 - u));
	f_pos = modelview * vec4(vertex, 1.0f);
	f_clip_pos = (projection * modelview) * vec4(vertex, 1.0f);
	f_clip_future_pos = (projection * future * modelview) * vec4(vertex, 1.0f);
	
	gl_Position = f_clip_pos;
}
