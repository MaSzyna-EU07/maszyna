#version 460 core
#define STAGE_FRAGMENT 1
#define LAYOUT_POST 1
#include "common.glsl"
layout(location = 14) in vec2 f_coords;

layout(location = 0) out vec4 out_color;

#pragma texture(tex1, 0, RGB)
layout(set = POST_SET, binding = POST_TEXTURE_FIRST + 0) uniform sampler2D tex1;

// glow of the bright parts of the image (postfx_bloom_*.frag)
#pragma texture(bloom_tex, 1, RGB)
layout(set = POST_SET, binding = POST_TEXTURE_FIRST + 1) uniform sampler2D bloom_tex;


#include "tonemapping.glsl"
#include "dither.glsl"

// param is pc.data of the push constants (the model block in the opengl renderer).
// param[0].x: share of the bloom in the image (0: no bloom, the texture is not read), param[0].y: 1 if the automatic
// exposure is applied (exposure_ssbo of common.glsl, worked out by exposure_average.comp), param[0].z: fixed exposure multiplier, param[0].w: scale of
// the bloom texture (it holds the sum of the levels of the chain), param[1].x: 1 when this is the last pass before the
// window, which takes the dither, param[1].yzw: gains of the white balance (0: none)
void main()
{
	vec2 texcoord = f_coords;
	vec3 hdr_color = texture(tex1, texcoord).xyz;
	if (param[0].x > 0.0)
		hdr_color = mix(hdr_color, texture(bloom_tex, texcoord).rgb * param[0].w, param[0].x);
	if (param[1].y > 0.0)
		hdr_color = white_balance(hdr_color, param[1].yzw);
	float exposurescale = param[0].z;
	if (param[0].y > 0.5 && exposure > 0.0)
		exposurescale *= exposure;
	out_color = tonemap(vec4(hdr_color * exposurescale, 1.0));
	if (param[1].x > 0.5)
		out_color.rgb = dither_output(out_color.rgb);
}
