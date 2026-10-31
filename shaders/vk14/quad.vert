#version 460 core
#define STAGE_VERTEX 1
// the full screen quad of the post-processing, or with QUAD_SCENE of a pass of the scene (the layout of the scene)
#ifndef QUAD_SCENE
#define LAYOUT_POST 1
#endif
#include "common.glsl"
const vec2 vert[4] = vec2[]
(
  vec2(-1.0,  1.0),
  vec2(-1.0, -1.0),
  vec2( 1.0,  1.0),
  vec2( 1.0, -1.0)
);

const vec2 uv[4] = vec2[]
(
  vec2(0.0, 1.0),
  vec2(0.0, 0.0),
  vec2(1.0, 1.0),
  vec2(1.0, 0.0)
);

layout(location = 14) out vec2 f_coords;

void main()
{
    gl_Position = vec4(vert[gl_VertexIndex], 0.0, 1.0); 
    f_coords = uv[gl_VertexIndex];
}  
