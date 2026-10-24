// noise of half a step of the 8-bit output, added to the colour about to be written to the window: smooth dark gradients (the
// fog at night) show no bands, which otherwise crawl over the image with the smallest change of the light. interleaved gradient
// noise (Jimenez 2014) of the pixel, the same in every frame. the colour is the one written to the window: encoded already with
// OUTPUT_GAMMA, linear otherwise (the sRGB framebuffer encodes it, so the step is taken in the encoded values)
#ifndef DITHER_GLSL
#define DITHER_GLSL

#include "common.glsl"

vec3 dither_output(vec3 color)
{
	float noise = (fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715)))) - 0.5) / 255.0;
	if (OUTPUT_GAMMA)
	{
		return max(color + noise, vec3(0.0));
	}
	else
	{
		vec3 linear = max(color, vec3(0.0));
		vec3 encoded = mix(linear * 12.92, 1.055 * pow(linear, vec3(1.0 / 2.4)) - 0.055, step(vec3(0.0031308), linear));
		encoded = max(encoded + noise, vec3(0.0));
		return mix(encoded / 12.92, pow((encoded + 0.055) / 1.055, vec3(2.4)), step(vec3(0.04045), encoded));
	}
}

#endif
