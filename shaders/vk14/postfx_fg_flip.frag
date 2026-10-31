#version 460 core
#define STAGE_FRAGMENT 1
#define LAYOUT_POST 1
#include "common.glsl"
// the depth or the velocities of the scene for the frame generation of dlss (rendering/vulkan14/vulkan14dlss.h), turned the right
// way up: the images of the renderer go from the bottom of the picture up (decision K8), the image the frame generation is given
// (what the window shows) from the top down. the texel as it is; the velocities keep their sign, their scale turns them over
layout(location = 0) out vec4 out_value;

#pragma texture(source_tex, 0, RGBA)
layout(set = POST_SET, binding = POST_TEXTURE_FIRST + 0) uniform sampler2D source_tex;

void main()
{
	ivec2 size = textureSize(source_tex, 0);
	ivec2 pixel = ivec2(gl_FragCoord.xy);
	out_value = texelFetch(source_tex, ivec2(pixel.x, size.y - 1 - pixel.y), 0);
}
