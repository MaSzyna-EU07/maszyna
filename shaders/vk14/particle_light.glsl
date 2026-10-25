// light of the particles of the gpu (gfx::gpu_particles: smoke, rain, snow) at a point of the view space of the pass, as a
// particle takes it as a whole, at its middle: the ambient light, the sun (the moon at night) held back by the shadow map and
// by the clouds, and the lamps and headlights of the light cluster the point is in (the first lights of lights[] without the
// clusters). each light is scattered towards the camera by the phase of the particle, Forward: 0 evenly, towards 1 mostly
// onwards. the scale is the one of the shading of the surfaces (light_common.glsl): a white particle takes as much light as
// a white face turned half way to it. for the compute passes; needs <common> and <weather_common.glsl>

#ifndef PARTICLE_LIGHT_GLSL
#define PARTICLE_LIGHT_GLSL

layout(set = PASS_SET, binding = SHADOW_TEX) uniform sampler2DArrayShadow shadowmap;
layout(set = PASS_SET, binding = HEADLIGHT_TEX) uniform sampler2D headlightmap;
layout(set = PASS_SET, binding = HEADLIGHT_SHADOW_TEX) uniform sampler2DArrayShadow headlight_shadowmap;

// AMBIENT_SCALE and SUN_DIFFUSE_SCALE of light_common.glsl
const float PARTICLE_AMBIENT_SCALE = 0.65;
const float PARTICLE_SUN_SCALE = 1.5;

// Henyey-Greenstein, 1 for every angle when G is 0
float particle_phase(float cosangle, float g)
{
	float g2 = g * g;
	return (1.0 - g2) / pow(max(1.0 + g2 - 2.0 * g * cosangle, 1e-4), 1.5);
}

// share of the sunlight the shadow map holds back at a point of the view space, one comparison (as fog_inject.comp)
float particle_sun_shadow(vec3 position)
{
	if (SHADOWMAP_ENABLED)
	{
		float span_distance = dot(position, position);
		uint cascade;
		for (cascade = 0U; cascade < MAX_CASCADES; cascade++)
			if (span_distance <= cascade_end[cascade])
				break;
		for (; cascade < MAX_CASCADES; cascade++)
		{
			vec4 lightpos = lightview[cascade] * vec4(position, 1.0);
			vec3 coords = lightpos.xyz / lightpos.w;
			if (all(greaterThanEqual(coords.xy, vec2(0.0))) && all(lessThanEqual(coords.xy, vec2(1.0))))
			{
				if (coords.z < 0.0)
					return 0.0;
				return texture(shadowmap, vec4(coords.xy, float(cascade), coords.z + 0.0001 * float(cascade + 1U)));
			}
			// outside the map: lit, unless the cascades are fitted to the depth of the view (gfx.shadow.sdsm), whose maps cover only
			// the area of the surfaces seen; then the first further cascade whose map holds the point
			if (shadow_fit.x <= 0.0)
				return 0.0;
		}
		return 0.0;
	}
	else
	{
		return 0.0;
	}
}

// light of a lamp or of the beam of headlights reaching a point of the view space (as fog_inject.comp; the beams with their
// shadow maps, the lamps without)
vec3 particle_lamp(light_s light, vec3 position)
{
	if (light.type == LIGHT_HEADLIGHTS)
	{
		vec4 projected = light.headlight_projection * vec4(position, 1.0);
		vec3 coords = projected.xyz / projected.w;
		if (coords.z < 0.0 || coords.z > 1.0 || any(lessThan(coords.xy, vec2(0.0))) || any(greaterThan(coords.xy, vec2(1.0))))
			return vec3(0.0);
		float span_distance = length(light.pos - position);
		float atten = 1.0 / (1.0 + light.linear * span_distance + light.quadratic * span_distance * span_distance);
		atten *= mix(1.0, 0.0, clamp((coords.z - 0.998) * 500.0, 0.0, 1.0));
		vec3 beam = textureLod(headlightmap, coords.xy, 1.0).rgb * light.headlight_weights.rgb;
		float intensity = max(max(beam.r, beam.g), beam.b);
		if ((light.headlight_weights.w > 0.5) && (intensity > 0.0))
		{
			uint map = uint(light.headlight_weights.w - 0.5);
			vec4 shadowpos = headlight_shadow[map] * vec4(position, 1.0);
			if (shadowpos.w > 0.0)
			{
				vec2 shadowcoords = shadowpos.xy / shadowpos.w;
				if (all(greaterThanEqual(shadowcoords, vec2(0.0))) && all(lessThanEqual(shadowcoords, vec2(1.0))))
				{
					float texel = headlight_shadow_params[map].x * shadowpos.w;
					float reference = shadowpos.z / max(shadowpos.w - texel * 1.5, 1e-3);
					intensity *= 1.0 - texture(headlight_shadowmap, vec4(shadowcoords, float(map), reference));
				}
			}
		}
		return light.color * (atten * intensity * light.intensity);
	}
	vec3 tolight = light.pos - position;
	float distance2 = dot(tolight, tolight);
	float edge = clamp(1.0 - distance2 * distance2 * (light.quadratic * light.quadratic), 0.0, 1.0);
	float atten = edge * edge / max(distance2, 0.01);
	if (light.type == LIGHT_SPOT)
		atten *= smoothstep(light.out_cutoff, light.in_cutoff, dot(-tolight * inversesqrt(max(distance2, 1e-8)), normalize(light.dir)));
	return light.color * (atten * light.intensity);
}

// the light cluster of a point of the view space in front of the camera, as fragment_cluster() of light_common.glsl finds it for
// the fragment drawn there
uint particle_cluster(vec3 position)
{
	vec4 clip = cluster_projection * vec4(position, 1.0);
	vec2 screen = clip.xy / clip.w * 0.5 + 0.5;
	ivec2 tile = clamp(ivec2(screen * vec2(CLUSTER_GRID_X, CLUSTER_GRID_Y)), ivec2(0), ivec2(CLUSTER_GRID_X - 1, CLUSTER_GRID_Y - 1));
	int slice = clamp(int(floor(cluster_slice_bias + log(max(clip.w, 1e-4)) * cluster_slice_scale)), 0, CLUSTER_GRID_Z - 1);
	return uint(tile.x + CLUSTER_GRID_X * (tile.y + CLUSTER_GRID_Y * slice));
}

// Position: of the particle in the view space; World: its offset from the camera along the world axes
vec3 particle_light(vec3 position, vec3 world, float forward)
{
	vec3 tocamera = -normalize(position);
	vec3 light = ambient * PARTICLE_AMBIENT_SCALE;
	if (lights_count > 0U)
	{
		float sunlit = cloud_shadow(world) * (1.0 - particle_sun_shadow(position));
		if (sunlit > 0.0)
			light += lights[0].color * (0.5 * PARTICLE_SUN_SCALE * lights[0].intensity * sunlit * particle_phase(dot(normalize(lights[0].dir), tocamera), forward));
	}
	vec3 lamps = vec3(0.0);
	if (lights_clustered != 0U)
	{
		if ((cluster_valid != 0U) && (position.z < 0.0))
		{
			cluster_range_s range = cluster_ranges[particle_cluster(position)];
			for (uint i = 0U; i < range.count; i++)
			{
				light_s lamp = lights[cluster_light_indices[range.offset + i]];
				vec3 fromlamp = position - lamp.pos;
				lamps += particle_lamp(lamp, position) * particle_phase(dot(fromlamp, tocamera) * inversesqrt(max(dot(fromlamp, fromlamp), 1e-8)), forward);
			}
		}
	}
	else
	{
		for (uint i = 1U; i < lights_count; i++)
		{
			vec3 fromlamp = position - lights[i].pos;
			lamps += particle_lamp(lights[i], position) * particle_phase(dot(fromlamp, tocamera) * inversesqrt(max(dot(fromlamp, fromlamp), 1e-8)), forward);
		}
	}
	return light + lamps * 0.5;
}

#endif
