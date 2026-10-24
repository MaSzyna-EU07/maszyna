// one step down the bloom chain: a 13 tap filter (Jimenez, "Next generation post processing in Call of Duty: Advanced
// Warfare", 2014) from a level twice as large. the first step, from the image itself, weighs the groups of taps by their
// brightness (Karis average) so single bright pixels do not flicker, and applies the threshold.
// param[0].x: 1 for the first step, param[0].y: threshold (0: none), param[0].z: soft knee

in vec2 f_coords;
layout(location = 0) out vec4 out_color;

#texture (source_tex, 0, RGB)
uniform sampler2D source_tex;

#include <common>

vec3 tap(vec2 uv)
{
	vec3 color = texture(source_tex, uv).rgb;
	if (any(isnan(color)) || any(isinf(color)))
		return vec3(0.0);
	return clamp(color, vec3(0.0), vec3(65000.0));
}

float karis_weight(vec3 color)
{
	return 1.0 / (1.0 + dot(color, vec3(0.2126, 0.7152, 0.0722)));
}

void main()
{
	vec2 texel = 1.0 / vec2(textureSize(source_tex, 0));
	vec2 uv = f_coords;

	vec3 a = tap(uv + texel * vec2(-2.0, 2.0));
	vec3 b = tap(uv + texel * vec2(0.0, 2.0));
	vec3 c = tap(uv + texel * vec2(2.0, 2.0));
	vec3 d = tap(uv + texel * vec2(-2.0, 0.0));
	vec3 e = tap(uv);
	vec3 f = tap(uv + texel * vec2(2.0, 0.0));
	vec3 g = tap(uv + texel * vec2(-2.0, -2.0));
	vec3 h = tap(uv + texel * vec2(0.0, -2.0));
	vec3 i = tap(uv + texel * vec2(2.0, -2.0));
	vec3 j = tap(uv + texel * vec2(-1.0, 1.0));
	vec3 k = tap(uv + texel * vec2(1.0, 1.0));
	vec3 l = tap(uv + texel * vec2(-1.0, -1.0));
	vec3 m = tap(uv + texel * vec2(1.0, -1.0));

	vec3 color;
	if (param[0].x > 0.5)
	{
		vec3 g0 = (j + k + l + m) * 0.25;
		vec3 g1 = (a + b + d + e) * 0.25;
		vec3 g2 = (b + c + e + f) * 0.25;
		vec3 g3 = (d + e + g + h) * 0.25;
		vec3 g4 = (e + f + h + i) * 0.25;
		float w0 = 0.5 * karis_weight(g0);
		float w1 = 0.125 * karis_weight(g1);
		float w2 = 0.125 * karis_weight(g2);
		float w3 = 0.125 * karis_weight(g3);
		float w4 = 0.125 * karis_weight(g4);
		color = (g0 * w0 + g1 * w1 + g2 * w2 + g3 * w3 + g4 * w4) / (w0 + w1 + w2 + w3 + w4);

		float threshold = param[0].y;
		if (threshold > 0.0)
		{
			// quadratic soft knee below the threshold
			float knee = max(threshold * param[0].z, 1e-4);
			float brightness = max(color.r, max(color.g, color.b));
			float soft = clamp(brightness - threshold + knee, 0.0, 2.0 * knee);
			soft = soft * soft / (4.0 * knee);
			color *= max(soft, brightness - threshold) / max(brightness, 1e-4);
		}
	}
	else
	{
		color = e * 0.125 + (a + c + g + i) * 0.03125 + (b + d + f + h) * 0.0625 + (j + k + l + m) * 0.125;
	}
	out_color = vec4(color, 1.0);
}
