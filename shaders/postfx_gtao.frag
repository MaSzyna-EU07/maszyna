in vec2 f_coords;

layout(location = 0) out vec4 out_color;

#texture (depth_tex, 0, R)
uniform sampler2D depth_tex;

#include <common>

#include <gtao_common.glsl>

const float PI = 3.14159265;
const float HALF_PI = 1.57079633;

vec3 view_position(ivec2 texel)
{
	vec2 size = vec2(textureSize(depth_tex, 0));
	texel = clamp(texel, ivec2(0), ivec2(size) - 1);
	float depth = max(texelFetch(depth_tex, texel, 0).r, 1e-6);
	float ndcz = (param[1].x > 0.5 ? depth : depth * 2.0 - 1.0);
	float z = -projection[3][2] / (ndcz + projection[2][2]);
	vec2 ndc = (vec2(texel) + 0.5) / size * 2.0 - 1.0;
	return vec3((-ndc.x * z - projection[2][0] * z) / projection[0][0],
	            (-ndc.y * z - projection[2][1] * z) / projection[1][1],
	            z);
}

void main()
{
	vec2 size = vec2(textureSize(depth_tex, 0));
	ivec2 center = ivec2(f_coords * size);
	if (texelFetch(depth_tex, center, 0).r <= 0.0) {
		out_color = vec4(1.0, 60000.0, 0.0, 0.0);
		return;
	}

	vec3 P = view_position(center);
	vec3 V = normalize(-P);
	vec3 right = view_position(center + ivec2(1, 0)) - P;
	vec3 left = P - view_position(center - ivec2(1, 0));
	vec3 up = view_position(center + ivec2(0, 1)) - P;
	vec3 down = P - view_position(center - ivec2(0, 1));
	vec3 N = normalize(cross(abs(right.z) < abs(left.z) ? right : left, abs(up.z) < abs(down.z) ? up : down));
	if (dot(N, V) < 0.0)
		N = -N;

	float radius = param[0].x;
	float pixels = radius * projection[1][1] * 0.5 * size.y / -P.z;
	if (pixels < 2.0) {
		out_color = vec4(1.0, -P.z, octahedral_encode(mat3(inv_view) * N));
		return;
	}
	pixels = min(pixels, size.y * 0.25);

	int directions = max(int(param[0].z), 1);
	int steps = max(int(param[0].w), 1);
	float falloffrange = 0.6 * radius;
	float falloffmul = -1.0 / falloffrange;
	float falloffadd = (radius - falloffrange) / falloffrange + 1.0;
	float noise = fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));
	float jitter = fract(noise * 1.61803399);

	float visibility = 0.0;
	vec3 bent = vec3(0.0);
	for (int d = 0; d < directions; ++d)
	{
		float phi = (float(d) + noise) * PI / float(directions);
		vec2 omega = vec2(cos(phi), sin(phi));
		vec3 direction = vec3(omega, 0.0);
		vec3 ortho = direction - dot(direction, V) * V;
		vec3 axis = normalize(cross(ortho, V));
		vec3 projected = N - axis * dot(N, axis);
		float projectedlength = length(projected);
		float cosn = clamp(dot(projected, V) / max(projectedlength, 1e-4), 0.0, 1.0);
		float n = sign(dot(ortho, projected)) * acos(cosn);
		float lowcos0 = cos(n + HALF_PI);
		float lowcos1 = cos(n - HALF_PI);
		float horizoncos0 = lowcos0;
		float horizoncos1 = lowcos1;
		for (int s = 0; s < steps; ++s)
		{
			float t = (float(s) + jitter) / float(steps);
			t = t * t;
			vec2 offset = omega * max(t * pixels, 1.5);
			vec3 delta0 = view_position(center + ivec2(round(offset))) - P;
			vec3 delta1 = view_position(center - ivec2(round(offset))) - P;
			float length0 = length(delta0);
			float length1 = length(delta1);
			float weight0 = clamp(length(vec3(delta0.xy, delta0.z * (delta0.z > 0.0 ? 5.0 : 1.0))) * falloffmul + falloffadd, 0.0, 1.0);
			float weight1 = clamp(length(vec3(delta1.xy, delta1.z * (delta1.z > 0.0 ? 5.0 : 1.0))) * falloffmul + falloffadd, 0.0, 1.0);
			horizoncos0 = max(horizoncos0, mix(lowcos0, dot(delta0, V) / max(length0, 1e-4), weight0));
			horizoncos1 = max(horizoncos1, mix(lowcos1, dot(delta1, V) / max(length1, 1e-4), weight1));
		}
		float h0 = -acos(clamp(horizoncos1, -1.0, 1.0));
		float h1 = acos(clamp(horizoncos0, -1.0, 1.0));
		h0 = n + clamp(h0 - n, -HALF_PI, HALF_PI);
		h1 = n + clamp(h1 - n, -HALF_PI, HALF_PI);
		float arc0 = (cosn + 2.0 * h0 * sin(n) - cos(2.0 * h0 - n)) * 0.25;
		float arc1 = (cosn + 2.0 * h1 * sin(n) - cos(2.0 * h1 - n)) * 0.25;
		visibility += projectedlength * (arc0 + arc1);
		float middle = 0.5 * (h0 + h1);
		bent += (V * cos(middle) + normalize(ortho) * sin(middle)) * projectedlength * (arc0 + arc1);
	}
	visibility = clamp(visibility / float(directions), 0.0, 1.0);
	float fade = smoothstep(param[1].y * 0.75, param[1].y, -P.z);
	bent = (dot(bent, bent) > 1e-8 ? normalize(bent) : N);
	bent = normalize(mix(bent, N, fade));
	out_color = vec4(mix(visibility, 1.0, fade), -P.z, octahedral_encode(mat3(inv_view) * bent));
}
