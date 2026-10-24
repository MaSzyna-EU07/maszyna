#version 460 core
#define STAGE_VERTEX 1
// the user interface (imgui), drawn by vk14::imgui_backend over the image of the window

#include "common.glsl"

layout(location = 0) in vec2 v_position;
layout(location = 1) in vec2 v_uv;
layout(location = 2) in vec4 v_colour;

// the pixels of the window to the clip space
layout(push_constant) uniform ui_transform
{
	vec2 scale;
	vec2 translate;
} transform;

layout(location = 0) out vec2 f_uv;
layout(location = 1) out vec4 f_colour;

void main()
{
	f_uv = v_uv;
	// the colours of imgui are srgb; they are mixed in linear light, as the user interface of the opengl renderer does it
	f_colour = vec4(pow(v_colour.rgb, vec3(2.2)), v_colour.a);
	gl_Position = vec4(v_position * transform.scale + transform.translate, 0.0, 1.0);
}
