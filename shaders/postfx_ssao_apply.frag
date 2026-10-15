// applies the ambient occlusion of the half size target to the image at full size, as a multiplier: drawn with the
// blending dst * src over the image (opengl46_ao::apply). the four nearest pixels of the occlusion are weighed by how
// close they are and by how close their distance is to that of the pixel, so the occlusion does not spill over the
// edges of objects. each pixel of the occlusion stands for the top left one of the four full size pixels it covers
// params.x: 0 ground truth ambient occlusion, 1 the older ssao (its own fixed curve)
// params.y: strength, a power of the occlusion (1 = as computed); params.z: albedo assumed by the multiple bounce term

in vec2 f_coords;
layout(location = 0) out vec4 out_color;

layout(binding = 0) uniform sampler2D depth_tex;
layout(binding = 1) uniform sampler2D ao_tex;

layout(location = 0) uniform mat4 inv_projection;
layout(location = 1) uniform vec4 params;

float view_distance(ivec2 pixel, float depth)
{
	ivec2 full_size = textureSize(depth_tex, 0);
	vec2 uv = (vec2(pixel) + 0.5) / vec2(full_size);
	vec4 position = inv_projection * vec4(uv * 2.0 - 1.0, depth, 1.0);
	return length(position.xyz / position.w);
}

// light which comes back after more than one bounce off surfaces of the given albedo, fit of Jimenez et al. 2016
float multi_bounce(float visibility, float albedo)
{
	float a = 2.0404 * albedo - 0.3324;
	float b = -4.7951 * albedo + 0.6417;
	float c = 2.7552 * albedo + 0.6903;
	return max(visibility, ((visibility * a + b) * visibility + c) * visibility);
}

void main()
{
	ivec2 full_size = textureSize(depth_tex, 0);
	ivec2 half_size = textureSize(ao_tex, 0);
	ivec2 pixel = ivec2(gl_FragCoord.xy);
	float depth = texelFetch(depth_tex, pixel, 0).r;
	if (depth <= 0.0)
	{
		// the sky
		out_color = vec4(1.0);
		return;
	}
	float distance = view_distance(pixel, depth);

	vec2 position = vec2(pixel) * 0.5;
	ivec2 base = ivec2(floor(position));
	vec2 fraction = position - vec2(base);
	float sum = 0.0;
	float weights = 0.0;
	for (int y = 0; y < 2; ++y)
	{
		for (int x = 0; x < 2; ++x)
		{
			ivec2 texel = clamp(base + ivec2(x, y), ivec2(0), half_size - 1);
			ivec2 texelpixel = min(texel * 2, full_size - 1);
			float texeldepth = texelFetch(depth_tex, texelpixel, 0).r;
			float texeldistance = (texeldepth > 0.0) ? view_distance(texelpixel, texeldepth) : 1e30;
			float bilinear = (x == 0 ? 1.0 - fraction.x : fraction.x) * (y == 0 ? 1.0 - fraction.y : fraction.y);
			float weight = (bilinear + 1e-3) / (1e-3 + abs(texeldistance - distance) / distance);
			sum += texelFetch(ao_tex, texel, 0).r * weight;
			weights += weight;
		}
	}
	float ao = clamp(sum / max(weights, 1e-6), 0.0, 1.0);

	if (params.x > 0.5)
	{
		// the curve the older ssao was applied with, a floor so the shadows do not go black
		ao = mix(0.3, 1.0, pow(ao, 0.8));
	}
	else
	{
		ao = pow(ao, params.y);
		ao = multi_bounce(ao, params.z);
	}
	out_color = vec4(vec3(ao), 1.0);
}
