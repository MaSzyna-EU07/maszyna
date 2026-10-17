// fog between the camera and a point of the view space (weather_common.glsl): from the fog volume of the main view
// (fog_inject.comp, fog_integrate.comp) up to its far end and along the ray past it, or along the ray all the way in the
// other passes. Fragcoord: the pixel of the target the point is seen at
#ifndef FOG_VOLUME_GLSL
#define FOG_VOLUME_GLSL

#include <weather_common.glsl>

layout(binding = FOG_VOLUME_TEX) uniform sampler3D fog_volume;

// the volume at a point of the view (0-1) and a coordinate along the slices, filtered across the view with a cubic B-spline
// (four fetches of the linear filter): a cell is several pixels wide, and the linear filter alone draws the shafts of light and
// the edges of shadows in the fog as steps
vec4 fog_volume_at(vec2 coords, float slice)
{
	vec2 size = vec2(textureSize(fog_volume, 0).xy);
	vec2 texel = coords * size - 0.5;
	vec2 base = floor(texel);
	vec2 f = texel - base;
	vec2 f2 = f * f;
	vec2 f3 = f2 * f;
	vec2 w0 = (1.0 - 3.0 * f + 3.0 * f2 - f3) / 6.0;
	vec2 w1 = (4.0 - 6.0 * f2 + 3.0 * f3) / 6.0;
	vec2 w2 = (1.0 + 3.0 * f + 3.0 * f2 - 3.0 * f3) / 6.0;
	vec2 w3 = f3 / 6.0;
	vec2 g0 = w0 + w1;
	vec2 g1 = w2 + w3;
	vec2 h0 = (base - 0.5 + w1 / g0) / size;
	vec2 h1 = (base + 1.5 + w3 / g1) / size;
	return g0.y * (g0.x * textureLod(fog_volume, vec3(h0.x, h0.y, slice), 0.0) + g1.x * textureLod(fog_volume, vec3(h1.x, h0.y, slice), 0.0))
	     + g1.y * (g0.x * textureLod(fog_volume, vec3(h0.x, h1.y, slice), 0.0) + g1.x * textureLod(fog_volume, vec3(h1.x, h1.y, slice), 0.0));
}

// rgb: light the fog adds, a: what it lets through
vec4 fog_along_view(vec3 view, vec2 fragcoord)
{
	float span_distance = length(view);
	if (span_distance <= 0.0)
		return vec4(0.0, 0.0, 0.0, 1.0);
	vec3 direction = mat3(inv_view) * (view / span_distance);
	if (fog_volume_params.x < 0.5)
		return fog_analytic(direction, 0.0, span_distance, false);
	// slices are spaced exponentially in the view depth plus FOG_VOLUME_BIAS; slice k holds the fog up to its far side
	float depth = max(-view.z, 1e-4);
	float slices = fog_volume_params.w;
	float slice = log(max((depth + FOG_VOLUME_BIAS) / (fog_volume_params.y + FOG_VOLUME_BIAS), 1e-6)) * fog_volume_params.z;
	vec2 coords = fragcoord * fog_volume_target.xy;
	if (slice < 1.0)
		return mix(vec4(0.0, 0.0, 0.0, 1.0), fog_volume_at(coords, 0.5 / slices), max(slice, 0.0));
	if (slice < slices)
		return fog_volume_at(coords, (slice - 0.5) / slices);
	vec4 volume = fog_volume_at(coords, (slices - 0.5) / slices);
	vec4 rest = fog_analytic(direction, fog_volume_target.z * span_distance / depth, span_distance, false);
	return vec4(volume.rgb + volume.a * rest.rgb, volume.a * rest.a);
}

// how fast the fog of the volume over the sky gives way to the fog without the clouds, per share of the light the clouds in it hold back
const float FOG_SKY_CLOUD_CUT = 4.0;

// fog in front of the sky in a view space direction: through the fog volume to its far end, then along the ray without the
// haze the atmosphere draws already. the far fog of the volume and the fog over the sky behind it are the same then
vec4 fog_sky(vec3 viewdirection, vec2 fragcoord)
{
	vec3 view = normalize(viewdirection);
	vec3 direction = mat3(inv_view) * view;
	if (fog_volume_params.x < 0.5 || view.z > -1e-3)
		return fog_analytic(direction, 0.0, 200000.0, true);
	float slices = fog_volume_params.w;
	vec4 volume = fog_volume_at(fragcoord * fog_volume_target.xy, (slices - 0.5) / slices);
	float far = fog_volume_target.z / -view.z;
	// the volume holds the low clouds within its reach (fog_inject.comp), for what stands in them; over the sky they are drawn by the
	// ray march (clouds_march.comp) already. laid on a second time at the size of the cells they would show blurred and grey, with a
	// light rim, and cut into bands where the slices cross them. where the volume lets through less than the fog without the clouds,
	// that fog is taken instead
	if (weather_camera.z > 0.5)
	{
		vec4 clear = fog_analytic(direction, 0.0, far, false);
		float clouds = clamp((1.0 - volume.a / max(clear.a, 1e-4)) * FOG_SKY_CLOUD_CUT, 0.0, 1.0);
		volume = mix(volume, clear, clouds);
	}
	vec4 rest = fog_analytic(direction, far, 200000.0, true);
	return vec4(volume.rgb + volume.a * rest.rgb, volume.a * rest.a);
}

#endif
