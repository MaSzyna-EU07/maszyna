in vec2 f_coords;

layout(location = 0) out vec4 out_color;

#texture (ao_tex, 0, RGBA)
uniform sampler2D ao_tex;

#include <common>
#include <gtao_common.glsl>

void main()
{
	ivec2 size = textureSize(ao_tex, 0);
	ivec2 center = ivec2(f_coords * vec2(size));
	vec4 centersample = texelFetch(ao_tex, center, 0);
	float centerdistance = centersample.g;
	if (centerdistance >= 60000.0) {
		out_color = centersample;
		return;
	}
	float visibility = 0.0;
	vec3 bent = vec3(0.0);
	float weights = 0.0;
	for (int y = -2; y <= 2; ++y)
	{
		for (int x = -2; x <= 2; ++x)
		{
			vec4 tap = texelFetch(ao_tex, clamp(center + ivec2(x, y), ivec2(0), size - 1), 0);
			float difference = (tap.g - centerdistance) / (centerdistance * 0.02 + 0.01);
			float weight = exp(-float(x * x + y * y) / 4.5 - difference * difference);
			visibility += tap.r * weight;
			bent += octahedral_decode(tap.ba) * weight;
			weights += weight;
		}
	}
	bent = (dot(bent, bent) > 1e-8 ? normalize(bent) : octahedral_decode(centersample.ba));
	out_color = vec4(visibility / max(weights, 1e-4), centerdistance, octahedral_encode(bent));
}
