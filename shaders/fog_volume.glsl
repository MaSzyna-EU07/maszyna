// fog between the camera and a point of the view space (weather_common.glsl): from the fog volume of the main view
// (fog_inject.comp, fog_integrate.comp) up to its far end and along the ray past it, or along the ray all the way in the
// other passes. Fragcoord: the pixel of the target the point is seen at
#ifndef FOG_VOLUME_GLSL
#define FOG_VOLUME_GLSL

#include <weather_common.glsl>

layout(binding = FOG_VOLUME_TEX) uniform sampler3D fog_volume;

// rgb: light the fog adds, a: what it lets through
vec4 fog_along_view(vec3 view, vec2 fragcoord)
{
	float span_distance = length(view);
	if (span_distance <= 0.0)
		return vec4(0.0, 0.0, 0.0, 1.0);
	vec3 direction = mat3(inv_view) * (view / span_distance);
	if (fog_volume_params.x < 0.5)
		return fog_analytic(direction, 0.0, span_distance, false);
	// slices are spaced exponentially in the view depth; slice k holds the fog up to its far side
	float depth = max(-view.z, 1e-4);
	float slices = fog_volume_params.w;
	float slice = log(max(depth / fog_volume_params.y, 1e-6)) * fog_volume_params.z;
	vec2 coords = fragcoord * fog_volume_target.xy;
	if (slice < 1.0)
		return mix(vec4(0.0, 0.0, 0.0, 1.0), textureLod(fog_volume, vec3(coords, 0.5 / slices), 0.0), max(slice, 0.0));
	if (slice < slices)
		return textureLod(fog_volume, vec3(coords, (slice - 0.5) / slices), 0.0);
	vec4 volume = textureLod(fog_volume, vec3(coords, (slices - 0.5) / slices), 0.0);
	vec4 rest = fog_analytic(direction, fog_volume_target.z * span_distance / depth, span_distance, false);
	return vec4(volume.rgb + volume.a * rest.rgb, volume.a * rest.a);
}

// fog in front of the sky in a view space direction: through the fog volume to its far end, then along the ray without the
// haze the atmosphere draws already. the far fog of the volume and the fog over the sky behind it are the same then
vec4 fog_sky(vec3 viewdirection, vec2 fragcoord)
{
	vec3 view = normalize(viewdirection);
	vec3 direction = mat3(inv_view) * view;
	if (fog_volume_params.x < 0.5 || view.z > -1e-3)
		return fog_analytic(direction, 0.0, 200000.0, true);
	float slices = fog_volume_params.w;
	vec4 volume = textureLod(fog_volume, vec3(fragcoord * fog_volume_target.xy, (slices - 0.5) / slices), 0.0);
	vec4 rest = fog_analytic(direction, fog_volume_target.z / -view.z, 200000.0, true);
	return vec4(volume.rgb + volume.a * rest.rgb, volume.a * rest.a);
}

#endif
