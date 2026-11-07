#version 460 core
#define STAGE_FRAGMENT 1
#define LAYOUT_UI 1
// the user interface (imgui): the colour of the vertices times the texture of the draw (the font, an image, a texture), pushed
// with the draw

#include "common.glsl"

layout(location = 0) in vec2 f_uv;
layout(location = 1) in vec4 f_colour;

layout(set = UI_SET, binding = UI_TEXTURE) uniform sampler2D ui_texture;

layout(location = 0) out vec4 out_colour;

void main()
{
	out_colour = FBOUT(f_colour * texture(ui_texture, f_uv));
}
