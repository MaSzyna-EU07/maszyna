#version 460 core
#define STAGE_FRAGMENT 1
#define LAYOUT_POST 1
#include "common.glsl"
// the finished image to the window (with quad.vert). the renderer draws with the matrices of the opengl one, so its images
// have the rows in the memory order of opengl, the bottom of the picture first; the window shows the first row at the top.
// this is the only place the picture is turned upside down. no push constants
layout(location = 14) in vec2 f_coords;

layout(set = POST_SET, binding = POST_TEXTURE_FIRST + 0) uniform sampler2D tex1;

layout(location = 0) out vec4 out_color;

void main()
{
	out_color = FBOUT(texture(tex1, vec2(f_coords.x, 1.0 - f_coords.y)));
}
