in vec2 f_coords;

layout(location = 0) out vec4 out_color;

#texture (tex1, 0, RGB)
uniform sampler2D tex1;

// glow of the bright parts of the image (postfx_bloom_*.frag)
#texture (bloom_tex, 1, RGB)
uniform sampler2D bloom_tex;

#include <common>

// exposure worked out by exposure_average.comp
layout(std430, binding = EXPOSURE_SSBO_BINDING) readonly buffer exposure_ssbo
{
	uint histogram[EXPOSURE_HISTOGRAM_BINS];
	float adapted_luminance;
	float exposure;
	float average_luminance;
	uint valid;
};

#include <tonemapping.glsl>

// param[0].x: share of the bloom in the image (0: no bloom, the texture is not read), param[0].y: 1 if the automatic
// exposure is applied, param[0].z: fixed exposure multiplier, param[0].w: scale of the bloom texture (it holds the sum of
// the levels of the chain)
void main()
{
	vec2 texcoord = f_coords;
	vec3 hdr_color = texture(tex1, texcoord).xyz;
	if (param[0].x > 0.0)
		hdr_color = mix(hdr_color, texture(bloom_tex, texcoord).rgb * param[0].w, param[0].x);
	float exposurescale = param[0].z;
	if (param[0].y > 0.5 && exposure > 0.0)
		exposurescale *= exposure;
	out_color = tonemap(vec4(hdr_color * exposurescale, 1.0));
}
