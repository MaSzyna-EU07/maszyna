#version 460 core
#define STAGE_FRAGMENT 1
#define LAYOUT_POST 1
#include "common.glsl"
// the exposure the tonemapping applies to the image (postfx_tonemapping.frag), in the texel dlss takes it from
// (rendering/vulkan14/vulkan14dlss.h). param is pc.data of the push constants: param[0].y 1 if the automatic exposure is applied
// (exposure_ssbo of common.glsl), param[0].z the fixed exposure multiplier
layout(location = 0) out vec4 out_exposure;

void main()
{
	float scale = param[0].z;
	if (param[0].y > 0.5 && exposure > 0.0)
		scale *= exposure;
	out_exposure = vec4(scale, 0.0, 0.0, 1.0);
}
