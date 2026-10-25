// screen space reflections (gfx.ssr) of the colour pass of the main view, included by light_common.glsl. the reflection of a
// surface is traced as a ray through the depth of the opaque scene drawn before the colour (scene_depth: the reverse z of a
// projection with the far plane at infinity, a depth d is a surface n / d away along the axis of the view), and its colour is
// taken from the image of the last frame (ssr_history) at the place the point hit had in that frame. what the screen doesn't
// hold (off the image, behind something nearer, too far, too rough) is left to the caller: the alpha of the result says how much
// of the reflection the screen gave. scene_ubs.ssr_params is set only for the main view, zero elsewhere
layout(set = PASS_SET, binding = SSR_HISTORY_TEX) uniform sampler2D ssr_history;

// the steps of a ray that found something are refined this many times, halving the step
const int SSR_REFINE_STEPS = 5;

// a value 0 .. 1 different for the neighbouring pixels (interleaved gradient noise), and from frame to frame with the temporal
// anti-aliasing, which blends the steps of the rays into a smooth image
float ssr_noise()
{
	vec2 pixel = gl_FragCoord.xy + 5.588238 * ssr_params.w;
	return fract(52.9829189 * fract(dot(pixel, vec2(0.06711056, 0.00583715))));
}

// the distance along the axis of the view of the surface in the depth at a place of the image (0 .. 1), 0: nothing there (the sky)
float ssr_surface(vec2 coords, vec2 size, float near)
{
	float depth = texelFetch(scene_depth, clamp(ivec2(coords * size), ivec2(0), ivec2(size) - 1), 0).r;
	return (depth > 0.0 ? near / depth : 0.0);
}

// the reflection the screen holds, of a surface at Position (view space) with the geometric normal Normal (view space, either
// side), for a ray along Direction (view space, normalized) and the perceptual roughness Roughness: the colour (rgb) times how
// much of it the screen holds (a)
vec4 ssr_trace(vec3 position, vec3 normal, vec3 direction, float roughness)
{
	float steps = ssr_params.x;
	if ((steps < 1.0) || (roughness > ssr_params.z))
		return vec4(0.0);
	// the side of the surface the camera sees; a ray going into it (a normal map turned far from the face) finds nothing
	if (dot(normal, position) > 0.0)
		normal = -normal;
	if (dot(normal, direction) <= 0.0)
		return vec4(0.0);
	vec2 size = vec2(textureSize(scene_depth, 0));
	float near = projection[3][2];
	float pixelscale = 2.0 / (abs(projection[1][1]) * size.y);
	// the ray starts two pixels off the surface, so the depth of the surface itself doesn't stop it
	vec4 start = projection * vec4(position, 1.0);
	vec3 origin = position + normal * (start.w * pixelscale * 2.0);
	vec4 c0 = projection * vec4(origin, 1.0);
	// the distance along the axis of the view changes with the ray: it ends in front of the near plane
	float dw = (projection * vec4(direction, 0.0)).w;
	float raylength = ssr_params.y;
	if (dw < 0.0)
		raylength = min(raylength, (c0.w - near * 2.0) / -dw);
	if (raylength <= 0.0)
		return vec4(0.0);
	vec3 end = origin + direction * raylength;
	vec4 c1 = projection * vec4(end, 1.0);
	// the points of the ray are interpolated across the image (u) with 1 / w, which is linear there
	float k0 = 1.0 / c0.w;
	float k1 = 1.0 / c1.w;
	vec2 s0 = c0.xy * k0;
	vec2 s1 = c1.xy * k1;
	vec2 delta = s1 - s0;
	// the part of the ray over the image
	float umax = 1.0;
	if (abs(delta.x) > 1e-6)
		umax = min(umax, ((delta.x > 0.0 ? 1.0 : -1.0) - s0.x) / delta.x);
	if (abs(delta.y) > 1e-6)
		umax = min(umax, ((delta.y > 0.0 ? 1.0 : -1.0) - s0.y) / delta.y);
	if (umax <= 0.0)
		return vec4(0.0);
	// not more steps than the pixels the ray crosses
	float pixels = length(delta * umax * size * 0.5);
	float count = clamp(floor(pixels), 1.0, steps);
	float jitter = ssr_noise();

	float uprev = 0.0;
	float wprev = c0.w;
	float uhit = -1.0;
	for (float i = 0.0; i < count; i += 1.0)
	{
		float u = umax * min((i + jitter) / count, 1.0);
		float w = 1.0 / mix(k0, k1, u);
		vec2 coords = mix(s0, s1, u) * 0.5 + 0.5;
		float surface = ssr_surface(coords, size, near);
		float bias = w * pixelscale * 0.5;
		// the things in the depth are taken to be this thick behind their faces, with the length of the step on top
		float thickness = max(0.25, surface * 0.02) + abs(w - wprev);
		if ((surface > 0.0) && (w - surface > bias) && (w - surface < thickness))
		{
			// refined between the last step in front of the surface and this one behind it
			float lo = uprev;
			float hi = u;
			for (int r = 0; r < SSR_REFINE_STEPS; ++r)
			{
				float mid = 0.5 * (lo + hi);
				float wmid = 1.0 / mix(k0, k1, mid);
				float smid = ssr_surface(mix(s0, s1, mid) * 0.5 + 0.5, size, near);
				if ((smid > 0.0) && (wmid - smid > 0.0))
					hi = mid;
				else
					lo = mid;
			}
			// what the ray went behind has to be close to it there: otherwise the ray went behind something thin, and goes on
			float whit = 1.0 / mix(k0, k1, hi);
			float shit = ssr_surface(mix(s0, s1, hi) * 0.5 + 0.5, size, near);
			if ((shit > 0.0) && (abs(whit - shit) < max(0.25, shit * 0.02) + whit * pixelscale * 2.0))
			{
				uhit = hi;
				break;
			}
		}
		uprev = u;
		wprev = w;
	}
	if (uhit < 0.0)
		return vec4(0.0);

	// the point hit, view space
	vec3 hit = mix(origin * k0, end * k1, uhit) / mix(k0, k1, uhit);
	vec2 hitcoords = mix(s0, s1, uhit) * 0.5 + 0.5;
	// where it was in the last frame: the cab moves with the camera, the world stands still
	mat4 reprojection = ssr_reprojection;
	if (ssr_params2.y > 0.0)
	{
		vec3 box = (ssr_cab_box * vec4(hit, 1.0)).xyz;
		if (all(lessThanEqual(abs(box), vec3(1.0))))
			reprojection = ssr_reprojection_cab;
	}
	vec4 previous = reprojection * vec4(hit, 1.0);
	if (previous.w <= 0.0)
		return vec4(0.0);
	vec2 coords = (previous.xy / previous.w) * 0.5 + 0.5;

	float share = 1.0;
	// faded towards the edges of the image, where it was hit and where it was in the last frame
	vec2 edges = min(min(hitcoords, 1.0 - hitcoords), min(coords, 1.0 - coords));
	share *= smoothstep(0.0, 0.04, min(edges.x, edges.y));
	// and towards the end of the ray
	float travelled = length(hit - origin);
	share *= 1.0 - smoothstep(0.75, 1.0, travelled / ssr_params.y);
	// the rougher surfaces hand over to the environment map
	share *= 1.0 - smoothstep(ssr_params.z * 0.7, ssr_params.z, roughness);
	// a ray towards the camera meets the backs of what the image shows
	share *= 1.0 - smoothstep(0.4, 0.9, direction.z);
	if (share <= 0.0)
		return vec4(0.0);

	// the lobe of a rough surface covers more of the image the farther the point hit is: a blurrier level of the history
	float alpha = roughness * roughness;
	float hitpixel = max((projection * vec4(hit, 1.0)).w, near) * pixelscale;
	float lod = clamp(log2(max(2.0 * travelled * alpha / hitpixel, 1.0)), 0.0, ssr_params2.x);
	vec3 color = textureLod(ssr_history, coords, lod).rgb;
	if (any(isnan(color)) || any(isinf(color)))
		return vec4(0.0);
	return vec4(max(color, vec3(0.0)) * share, share);
}
