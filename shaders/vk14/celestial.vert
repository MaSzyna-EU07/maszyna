#version 460 core
#define STAGE_VERTEX 1
#include "common.glsl"
const vec2 vert[4] = vec2[]
(
  vec2(-1.0,  1.0),
  vec2(-1.0, -1.0),
  vec2( 1.0,  1.0),
  vec2( 1.0, -1.0)
);

layout(location = 2) out vec2 f_coord;
// the corner of the quad, -1...1 across it
layout(location = 15) out vec2 f_local;


void main()
{
	gl_Position = projection * (vec4(param[1].xyz, 1.0) + vec4(vert[gl_VertexIndex] * param[1].w, 0.0, 0.0));

	f_local = vert[gl_VertexIndex];

	vec2 coord = param[2].xy;
	float size = param[2].z;

	if (gl_VertexIndex == 0)
		f_coord = vec2(coord.x, coord.y);
	if (gl_VertexIndex == 1)
		f_coord = vec2(coord.x, coord.y - size);
	if (gl_VertexIndex == 2)
		f_coord = vec2(coord.x + size, coord.y);
	if (gl_VertexIndex == 3)
		f_coord = vec2(coord.x + size, coord.y - size);
}
