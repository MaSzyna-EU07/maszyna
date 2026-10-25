#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
// depth prepass of materials which never discard a fragment (no alpha test)

layout(location = 2) in vec2 f_coord;

void main()
{
}
