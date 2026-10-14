in vec2 f_coords;

layout(location = 0) out vec4 out_color;

#texture (ao_tex, 0, R)
uniform sampler2D ao_tex;

#texture (depth_tex, 1, R)
uniform sampler2D depth_tex;

#include <common>

float view_distance(vec2 uv)
{
	ivec2 size = textureSize(depth_tex, 0);
	float depth = texelFetch(depth_tex, clamp(ivec2(uv * vec2(size)), ivec2(0), size - 1), 0).r;
	if (depth <= 0.0)
		return 1e6;
	float ndcz = (param[1].x > 0.5 ? depth : depth * 2.0 - 1.0);
	return projection[3][2] / (ndcz + projection[2][2]);
}

void main()
{
	vec2 texel = 1.0 / vec2(textureSize(ao_tex, 0));
	float center = view_distance(f_coords);
	if (center >= 1e6) {
		out_color = vec4(1.0);
		return;
	}
	float sum = 0.0;
	float weights = 0.0;
	for (int y = -2; y <= 2; ++y)
	{
		for (int x = -2; x <= 2; ++x)
		{
			vec2 uv = f_coords + vec2(float(x), float(y)) * texel;
			float difference = (view_distance(uv) - center) / (center * 0.02 + 0.01);
			float weight = exp(-float(x * x + y * y) / 4.5 - difference * difference);
			sum += texture(ao_tex, uv).r * weight;
			weights += weight;
		}
	}
	out_color = vec4(sum / max(weights, 1e-4));
}
