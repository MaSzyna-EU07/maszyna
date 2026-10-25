// weather of the vulkan renderer (the stage P2), shaders/ogl46/weather_common.glsl with the bindings of the pass set: the block
// of the weather of the pass (gl::weather_ubs without the colours of the horizon, which are a block of their own, written by
// fog_horizon.comp or by the cpu): fog, clouds and their shadows.
// world axes around the camera: the positions here are world space offsets from the camera, in metres, y up; heights are
// above the ground reference of the weather. the fog is a haze whose density falls exponentially with the height and a fog
// lying on the ground, both of a constant density below the reference (in the valleys), lit with the colour the atmosphere
// has at the horizon in the direction of the view (fog_horizon), less the part of the sunlight held back by shadows
#ifndef WEATHER_COMMON_GLSL
#define WEATHER_COMMON_GLSL

struct cloud_layer_s
{
	// x: base, y: top (metres above the ground reference), z: 1 / (top - base), w: genus (environment/weather.h)
	vec4 heights;
	// x: coverage 0-1, y: extinction at density 1 (1/m), or the optical depth of a flat layer, z: 1 / (size of the shape noise
	// tile), w: erosion by the detail noise
	vec4 shape;
	// x: share of the depth over which the density rises from the base, y: share from which it falls towards the top,
	// z: variation of the height of the tops, w: spread of the top (anvil)
	vec4 profile;
	// xy: position of the camera in the shape noise (metres, world x and z, wrapped to the tile), z: 1 / (size of the coverage
	// noise tile), w: stretch along the wind
	vec4 motion;
	// x: 1 for a flat layer, y: offset of the layer in the noise (seed), zw: position of the camera in the coverage noise
	vec4 extra;
};

layout(std140, set = PASS_SET, binding = WEATHER_UBO) uniform weather_ubo
{
	// x: extinction of the haze at the ground reference (1/m), y: 1 / its scale height, z: extinction the sky is not fogged
	// with (the atmosphere has that haze already), w: albedo of the fog
	vec4 fog_haze;
	// x: extinction of the ground fog (1/m), y: 1 / scale height of its top, z: height its density starts to fall at, w: patchiness
	vec4 fog_ground;
	// xyz: position of the camera in the noise of the fog (metres, moved by the wind), w: 1 / size of the patches
	vec4 fog_noise;
	// x: share of the light of the fog which comes from the sun (and is held back by shadows), y: transmittance of the clouds
	// for the sun on average, z: how much the dense ground fog is whitened, w: fog density of a regular draw (fog_density
	// of a draw over it scales the optical depth)
	vec4 fog_light;
	// x: 1 when the fog volume holds the fog of the pass, y: near, z: slices / log(far / near), w: slices
	vec4 fog_volume_params;
	// xy: 1 / size of the target of the pass (pixels), z: far end of the volume (view depth), w: share of the old sun glow
	vec4 fog_volume_target;
	// x: height of the camera above the ground reference, y: seconds of simulation, z: 1 when the clouds are drawn, w: layers
	vec4 weather_camera;
	// xyz: world space direction towards the key light (the sun, or the moon at night), w: 1 when it is above the horizon
	vec4 weather_sun_direction;
	// rgb: colour of the key light as the shading gets it, without the shadow of the clouds; w: scale of the sunlight of the clouds
	vec4 weather_sun_color;
	// rgb: ambient light of the sky above; w: scale of the sky light of the clouds
	vec4 weather_ambient;
	// xy: centre of the cloud shadow map (world x and z relative to the camera), z: 1 / its size (metres), w: 1 when it holds shadows
	vec4 cloud_shadow_params;
	// xy: world x and z of the direction the wind at the clouds blows to (unit), z: its speed, m/s, w: scale of the erosion of the
	// edges of the clouds by the detail noise
	vec4 cloud_wind;
	cloud_layer_s cloud_layers[MAX_CLOUD_LAYERS];
};

layout(std140, set = PASS_SET, binding = WEATHER_HORIZON_UBO) uniform weather_horizon_ubo
{
	// colour of the fog towards the horizon by the angle from the azimuth of the sun, 0 to 180 degrees, in the units of the
	// scene. filled on the gpu from the sky-view table of the atmosphere
	vec4 fog_horizon[FOG_HORIZON_SAMPLES];
};

layout(set = PASS_SET, binding = CLOUD_SHADOW_TEX) uniform sampler2D cloud_shadow_map;

const float WEATHER_PI = 3.14159265359;

// integral of the density of an exponential layer from the reference up to Height, the density being constant below Shift and
// falling with Rate (1 / scale height) above it
float fog_column(float height, float extinction, float rate, float shift)
{
	float above = height - shift;
	if (above <= 0.0)
		return extinction * height;
	return extinction * (shift + (1.0 - exp(-rate * above)) / rate);
}

// optical depth of the fog along a ray from the height Origin (above the reference), direction with the vertical part Up,
// between the distances Near and Far, for the haze and the ground fog of the given extinctions at the reference: x of the
// haze, y of the ground fog
vec2 fog_optical_depth(float origin, float up, float near, float far, float haze, float ground)
{
	float span = far - near;
	if (span <= 0.0)
		return vec2(0.0);
	float h0 = origin + up * near;
	float h1 = origin + up * far;
	if (abs(up) * span < 0.5)
	{
		// nearly level: the density in the middle
		float h = 0.5 * (h0 + h1);
		return vec2(haze * exp(-fog_haze.y * max(h, 0.0)), ground * exp(-fog_ground.y * max(h - fog_ground.z, 0.0))) * span;
	}
	float hazedepth = fog_column(h1, haze, fog_haze.y, 0.0) - fog_column(h0, haze, fog_haze.y, 0.0);
	float grounddepth = fog_column(h1, ground, fog_ground.y, fog_ground.z) - fog_column(h0, ground, fog_ground.y, fog_ground.z);
	return max(vec2(hazedepth, grounddepth) / up, vec2(0.0));
}

// light of the fog in a world space direction: the colour of the horizon the atmosphere has that way, the old sun glow where
// the old sky is used. the dense ground fog is whiter than the air
vec3 fog_color_world(vec3 direction)
{
	vec3 sun = weather_sun_direction.xyz;
	// the light of the haze goes with the angle from the sun, not with the azimuth alone: the table is taken at the point of the
	// horizon as far from the sun as the view is. level views get the colour of their azimuth as before; a view high up over the
	// sun no longer gets the glow of the horizon under it (a column of light standing over a low sun)
	float sunflat = length(sun.xz);
	float cosangle = 1.0;
	if (sunflat > 1e-4)
		cosangle = clamp(dot(normalize(direction), sun) / sunflat, -1.0, 1.0);
	float position = acos(clamp(cosangle, -1.0, 1.0)) / WEATHER_PI * float(FOG_HORIZON_SAMPLES - 1);
	int index = int(floor(position));
	vec3 color = mix(fog_horizon[clamp(index, 0, FOG_HORIZON_SAMPLES - 1)].rgb, fog_horizon[clamp(index + 1, 0, FOG_HORIZON_SAMPLES - 1)].rgb, fract(position));
	if (fog_volume_target.w > 0.0)
		color += weather_sun_color.rgb * pow(max(dot(direction, sun), 0.0), 30.0) * fog_volume_target.w;
	return color;
}

// the light the fog takes from the sun, 0-1: the share the shadows may hold back is scaled by Sunlit
float fog_light_factor(float sunlit)
{
	return (1.0 - fog_light.x) + fog_light.x * sunlit;
}

// the colour of the fog whitened by the share Ground of the dense ground fog in it
vec3 fog_whiten(vec3 color, float ground)
{
	float luminance = dot(color, vec3(0.2126, 0.7152, 0.0722));
	return mix(color, vec3(luminance), clamp(ground, 0.0, 1.0) * fog_light.z);
}

// fog along a ray of the view between two distances, of the haze and the ground fog of the given extinctions: rgb the light it
// adds, a what it lets through
vec4 fog_analytic_with(vec3 direction, float near, float far, float haze, float ground)
{
	vec2 depth = fog_optical_depth(weather_camera.x, direction.y, near, far, haze, ground);
	float total = depth.x + depth.y;
	float transmittance = exp(-total);
	vec3 light = fog_whiten(fog_color_world(direction), depth.y / max(total, 1e-6)) * fog_light_factor(fog_light.y) * fog_haze.w;
	return vec4(light * (1.0 - transmittance), transmittance);
}

// fog along a ray of the view between two distances. Sky leaves out the haze the atmosphere draws in the sky already
vec4 fog_analytic(vec3 direction, float near, float far, bool sky)
{
	return fog_analytic_with(direction, near, far, sky ? max(fog_haze.x - fog_haze.z, 0.0) : fog_haze.x, fog_ground.x);
}

// share of the key light the clouds let through to a point, World its offset from the camera
float cloud_shadow(vec3 world)
{
	if (cloud_shadow_params.w < 0.5)
		return 1.0;
	vec3 sun = weather_sun_direction.xyz;
	if (sun.y < 0.02)
		return fog_light.y;
	// the map holds the shadow on the ground reference: down the ray of the sun to it
	float height = weather_camera.x + world.y;
	vec2 ground = world.xz - sun.xz * (height / sun.y);
	vec2 coords = (ground - cloud_shadow_params.xy) * cloud_shadow_params.z + 0.5;
	// towards the edges of the map the shadow fades into the average
	vec2 edge = abs(coords - 0.5) * 2.0;
	float fade = smoothstep(0.8, 1.0, max(edge.x, edge.y));
	float shadow = textureLod(cloud_shadow_map, coords, 0.0).r;
	return mix(shadow, fog_light.y, fade);
}

#endif
