// the water of the scenery as gfx.water.realistic draws it (gl::scene_ubs::env_params.w), for mat_water.frag and
// mat_water_specgloss.frag; shaders/ogl46/water_surface.glsl with the reflection of the scene of the vulkan renderer, which takes
// its weight for the ray tracing. included after light_common.glsl, apply_fog.glsl and tonemapping.glsl, with the textures of the
// material (normalmap, dudvmap, diffuse) and the outputs (out_color, out_motion) declared before.
// the surface is a mirror rippled by three layers of the normal map going different ways at different scales (the dudv map bends
// them, so they don't slide along as rigid sheets), reflecting the scene where the screen space reflections (or the ray tracing)
// find it and the sky elsewhere, with the fresnel of water. under it the water lets the bottom through, less the deeper it goes,
// and gives back the light it scatters in the colour of the material: how deep it is comes from the depth of the opaque scene
// drawn before the colour (scene_depth), where the frame has it, and the surface fades out where it meets the ground, so the
// shore has no hard edge. without that depth the water is as clear as the alpha of its colour says, as the old water was.
// the material: param_color rgb the colour of the water, a how murky it is (0 clear .. 1 one sees half a metre into it);
// param_wave_strength how steep the ripples are (0.05 as shipped) and how much the dudv map bends them, param_wave_speed how
// fast they go (texture coordinates per second); param_reflection the share of the reflection of the surroundings, param_specular
// the highlights of the sun and the lamps (1 and more: all of them), param_glossiness the exponent of the old highlight, which
// sets how smooth the surface is between the ripples

#ifndef WATER_SURFACE_GLSL
#define WATER_SURFACE_GLSL

// the specular level of water (light_common.glsl: 0.08 times it at normal incidence, 2%)
const float WATER_SPECULARITY = 0.25;
// how far one sees into the clearest and the murkiest water, metres
const float WATER_CLEAR_VISIBILITY = 12.0;
const float WATER_MURKY_VISIBILITY = 0.5;
// the depth at which the surface fades in from the shore, metres, and how much it grows with the distance from the camera
const float WATER_SHORE_FADE = 0.1;
const float WATER_SHORE_FADE_DISTANCE = 0.002;

// the water drawn as realistic (gfx.water.realistic)
bool water_realistic()
{
	return env_params.w > 0.5;
}

// the roughness of the surface between the ripples, from the exponent of the old highlight; not below the one which keeps the
// highlight of the sun from getting so small and bright that the blending divides it out of range
float water_roughness()
{
	return clamp(phong_roughness(abs(param[1].w)) * 0.5, 0.06, 1.0);
}

// how much the dudv map shifts the coordinates of the material, slowly moving
vec2 water_flow(vec2 uv)
{
	vec2 moving = fract(vec2(0.31, 0.17) * (param[2].z * time));
	return (texture(dudvmap, uv * 0.5 + moving).rg * 2.0 - 1.0) * (param[2].y * 0.6);
}

// a texel of the normal map as a slope in the space of the tangents (x / z, y / z)
vec2 water_slope(vec2 uv)
{
	vec2 xy = texture(normalmap, uv).rg * 2.0 - 1.0;
	float z = sqrt(1.0 - clamp(dot(xy, xy), 0.0, 1.0));
	return xy / max(z, 0.2);
}

// the normal of the ripples in the space of the tangents at the coordinates Uv of the material shifted by Flow, Distance metres
// from the camera: three layers of the normal map at different scales going different ways, the second turned by 60 degrees so
// the tilings don't line up. the slopes add up; far away they're flattened, where the normal map would only shimmer
vec3 water_wave_normal(vec2 uv, vec2 flow, float distance)
{
	float speed = param[2].z;
	float strength = clamp(param[2].y / 0.05, 0.0, 4.0) * mix(1.0, 0.4, smoothstep(40.0, 600.0, distance));
	// the rotation by 60 degrees, and back for the slopes read through it
	mat2 turn = mat2(0.5, 0.866025, -0.866025, 0.5);
	vec2 a = uv + flow + fract(vec2(1.0, 0.35) * (speed * time));
	vec2 b = turn * (uv * 2.37 - flow * 1.7) + fract(vec2(-0.45, 1.0) * (speed * 1.31 * time));
	vec2 c = uv * 0.23 + fract(vec2(0.2, -0.6) * (speed * 0.5 * time));
	vec2 slope = water_slope(a) + transpose(turn) * water_slope(b) * 0.7 + water_slope(c) * 0.6;
	return normalize(vec3(slope * (strength * 0.6), 1.0));
}

// the water between the surface and what is behind it in the depth of the opaque scene drawn before the colour, metres: x along
// the ray of the view, y straight down (from the angle the view meets the surface at). negative where it isn't known: the frame
// has no such depth (it's there with the contact shadows, the screen space reflections or the ambient occlusion in the light of
// the surfaces, only in the colour pass of the main view), or it doesn't hold what is behind the surface
vec2 water_thickness(vec3 geonormal)
{
	if ((contact_shadow_params.x <= 0.0) && (ssr_params.x <= 0.0) && (ao_params.x <= 0.0))
		return vec2(-1.0);
	// the perspective with the far plane at infinity and the reverse z, a depth d is a surface n / d away along the axis of the view
	float near = projection[3][2];
	if (near <= 0.0)
		return vec2(-1.0);
	ivec2 size = textureSize(scene_depth, 0);
	float depth = texelFetch(scene_depth, clamp(ivec2(gl_FragCoord.xy), ivec2(0), size - 1), 0).r;
	float surface = max(-f_pos.z, 1e-3);
	// nothing behind (the sky, or past what the depth was drawn for): as deep as it gets
	if (depth <= 0.0)
		return vec2(1.0e4);
	float along = near / depth - surface;
	if (along < -0.25)
		return vec2(-1.0);
	float ray = max(along, 0.0) * length(f_pos.xyz) / surface;
	return vec2(ray, ray * abs(dot(normalize(f_pos.xyz), geonormal)));
}

// the light falling on the water from above which the water scatters back: the ambient light of the sky, the sun (where its
// shadow and the clouds let it) and the lamps (Lampdiffuse), on a surface facing Geonormal
vec3 water_light(vec3 geonormal, vec2 skyoccl, float shadow, float cloudsun, vec3 lampdiffuse)
{
	vec3 light = add_sky_ambient(ambient, geonormal) * (AMBIENT_SCALE * skyoccl.x);
	if (lights_count > 0U)
	{
		vec3 light_dir = normalize(-lights[0].dir);
		float NdotL = max(dot(geonormal, light_dir), 0.0);
		vec3 sunlight = lights[0].color * (SUN_DIFFUSE_SCALE * lights[0].intensity * cloudsun * skyoccl.y);
		light += sunlight * (NdotL * (1.0 - (1.0 - shadow_tone) * shadow));
	}
	return light + lampdiffuse;
}

// the colour of the water the light scattered in it takes: the colour of the material with the detail of its texture (the texel
// against the average of the texture, its last mipmap)
vec3 water_colour(vec2 coords)
{
	vec3 texel = texture(diffuse, coords).rgb;
	vec3 average = textureLod(diffuse, coords, 16.0).rgb;
	vec3 detail = clamp(texel / max(average, vec3(1e-3)), 0.0, 3.0);
	return pow(max(param[0].rgb, vec3(0.0)), vec3(2.2)) * mix(vec3(1.0), detail, 0.5);
}

// how much of the light the water lets through over a metre, for each channel: the murkier the water, the more; the channels its
// colour is strong in less, so what is seen through it takes the colour of the water the deeper it lies
vec3 water_absorption(vec3 colour)
{
	float visibility = mix(WATER_CLEAR_VISIBILITY, WATER_MURKY_VISIBILITY, clamp(param[0].a, 0.0, 1.0));
	vec3 tint = colour / max(max(colour.r, max(colour.g, colour.b)), 1e-4);
	return (3.0 / visibility) * (vec3(1.0) - 0.75 * tint);
}

// the reflection of the surroundings in the rippled surface: the scene where the screen space reflections (or the ray tracing)
// find it, the sky and the ground of the environment map elsewhere. Weight: how much of it is seen
vec3 water_reflection(vec3 fragnormal, float roughness, float weight, float skylight)
{
	vec3 surroundings = reflection_surroundings(fragnormal, roughness) * skylight;
	vec4 scene = vec4(0.0);
	if (((ssr_params.x > 0.0) || (ssr_params2.z > 0.0)) && (weight > REFLECTION_SCENE_WEIGHT))
		scene = reflection_scene(fragnormal, roughness, weight);
	return scene.rgb + surroundings * (1.0 - scene.a);
}

// shades the water at the coordinates of the material Coords with the normal of the ripples Wavenormal (the space of the tangents),
// the specular level Specularity and the roughness between the ripples Roughness, and writes the outputs. the colour is blended
// over what is behind (src_alpha, one_minus_src_alpha): the bottom stays seen as much as the water lets it through and the surface
// doesn't reflect it, the rest is the reflection and the light the water scatters back
void water_surface(vec2 coords, vec3 wavenormal, float specularity, float roughness)
{
	// the side of the surface the camera sees
	vec3 view_dir = normalize(-f_pos.xyz);
	vec3 geonormal = normalize(f_normal);
	float side = (dot(geonormal, view_dir) < 0.0) ? -1.0 : 1.0;
	geonormal *= side;
	vec3 fragnormal = normalize(f_tbn * wavenormal) * side;
	float NdotV = max(dot(fragnormal, view_dir), 1e-4);

	roughness = max(specular_antialiasing(fragnormal, roughness), MIN_ROUGHNESS);
	float f0 = 0.08 * specularity;
	// the share of the light the surface reflects, for its roughness and the angle it's seen at
	float fresnel = clamp(EnvBRDFApprox(vec3(f0), roughness, NdotV).x, 0.0, 1.0);
	float highlight = clamp(param[1].y, 0.0, 1.0);

	vec3 world = mat3(inv_view) * f_pos.xyz;
	vec2 skyoccl = sky_occlusion_world(world, normalize(mat3(inv_view) * fragnormal));
	float shadow = 0.0;
	if ((lights_count > 0U) && (shadow_tone < 1.0))
		shadow = clamp(calc_shadow(), 0.0, 1.0);
	float cloudsun = (lights_count > 0U) ? cloud_shadow(world) : 0.0;

	// the lamps and the headlights: their light the water scatters, and their highlights. add_local_light() scales the highlight by
	// param[1].y, which the water materials give as the exponent of the old highlight
	spec_f0 = vec3(f0);
	spec_roughness = roughness;
	vec3 lampdiffuse = vec3(0.0);
	vec3 lampspecular = vec3(0.0);
	add_local_lights(fragnormal, lampdiffuse, lampspecular);
	lampspecular *= highlight / max(param[1].y, 1e-4);

	// what is reflected: the surroundings and the highlight of the sun
	float reflectionshare = clamp(param[1].z, 0.0, 1.0);
	vec3 reflected = water_reflection(fragnormal, roughness, fresnel * reflectionshare, skyoccl.x) * (fresnel * reflectionshare);
	if (lights_count > 0U)
	{
		vec3 sunlight = lights[0].color * (SUN_DIFFUSE_SCALE * lights[0].intensity * cloudsun * skyoccl.y);
		reflected += sunlight * specular_light(normalize(-lights[0].dir), fragnormal, vec3(f0), roughness, SUN_BRDF_LIMIT) * ((1.0 - shadow) * highlight);
	}
	reflected += lampspecular;

	// how much of the bottom the water lets through, and the light it scatters back meanwhile
	vec3 colour = water_colour(coords);
	vec2 thickness = water_thickness(geonormal);
	vec3 transmitted = vec3(1.0 - clamp(param[0].a, 0.0, 1.0));
	float shore = 1.0;
	if (thickness.x >= 0.0)
	{
		transmitted = exp(-water_absorption(colour) * min(thickness.x, 1.0e3));
		shore = smoothstep(0.0, WATER_SHORE_FADE + WATER_SHORE_FADE_DISTANCE * length(f_pos.xyz), thickness.y);
	}
	vec3 scattered = water_light(geonormal, skyoccl, shadow, cloudsun, lampdiffuse) * colour * (vec3(1.0) - transmitted);

	// blended over the bottom: alpha the share of it the surface hides, the colour what the surface adds divided by that share
	float through = dot(transmitted, vec3(0.2126, 0.7152, 0.0722)) * (1.0 - fresnel);
	float alpha = max(1.0 - through, 0.01);
	vec3 fragcolor = min((reflected + scattered * (1.0 - fresnel)) / alpha, vec3(32000.0));

	vec4 color = vec4(apply_fog(fragcolor), clamp(alpha * shore, 0.0, 1.0));
	if (POSTFX_ENABLED)
		out_color = color;
	else
		out_color = tonemap(color);
	if (MOTIONBLUR_ENABLED)
	{
		vec2 a = (f_clip_future_pos.xy / f_clip_future_pos.w) * 0.5 + 0.5;
		vec2 b = (f_clip_pos.xy / f_clip_pos.w) * 0.5 + 0.5;
		out_motion = vec4(a - b, 0.0, 0.0);
	}
}

#endif
