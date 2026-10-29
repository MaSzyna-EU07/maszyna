// the shading of the probe of the occupied cab (gfx::cab_probe, as shaders/ogl46/cabprobe.glsl), for cabprobe.frag and cabprobe_alpha.frag: what the cab shows
// towards the middle of its box lit by the lights of the cab alone (its lamp or the light points of the cab model, the brightest
// MAX_CAB_PROBE_LAMPS, each with its cube shadow map; and what glows in it), and its albedo, each with 1 in the alpha: the probe is cleared to nothing, so the alpha of a direction tells how
// much of it the cab fills, and both are premultiplied by it. the light of a light of the cab is about the one the shading gives
// it (calc_local_light() in light_common.glsl): its cone, its sphere or soft falloff, its shadows. the sun (the moon) lights the cab where it comes in through the windows
// (gl::scene_ubs::cab_probe_sun), so the light which bounces around the cab holds it; the ambient light is left out, the cab gets it as
// before. the parameters of the lights and of the sun are in the block of the scene (gl::scene_ubs::cab_probe_lamp, cab_probe_sun), in
// the space of the probe. the including shader declares cab_probe_mask (CAB_PROBE_MASK_TEX) and includes weather_common.glsl

layout(location = 0) out vec4 out_light;
layout(location = 1) out vec4 out_albedo;

// closest distance (squared) the light of the lamp is computed for, as lamp_falloff() of common.glsl takes it for a lamp without a radius
const float PROBE_LIGHT_MIN_DISTANCE2 = 0.01;
// the light of the sun on a diffuse surface, as SUN_DIFFUSE_SCALE and SUN_NDOTL_SHARPNESS of light_common.glsl
const float PROBE_SUN_SCALE = 1.5;
const float PROBE_SUN_NDOTL_SHARPNESS = 1.25;

// the diffuse colour of the submodel or shape, as material_diffuse() of light_common.glsl
vec3 probe_material_diffuse()
{
	vec3 diffusecolor = pow(max(param[0].rgb, vec3(0.0)), vec3(2.2));
	float value = max(param[0].r, max(param[0].g, param[0].b));
	return (value > 0.0 ? diffusecolor * (param[1].x / value) : vec3(0.0));
}

// share of the light of the light Lamp of the cab held back before the point of the space of its cube shadow map (origin at the
// light the map was drawn for), 0..1, as local_light_shadow() of light_common.glsl takes it with a single sample
float probe_lamp_shadow(int lamp, vec3 position, vec3 normal)
{
	vec4 weights = cab_probe_lamp_shadow[lamp];
	float distance = length(position);
	float texel = weights.z * distance;
	vec3 tolamp = -position / max(distance, 1e-4);
	float NdotL = clamp(dot(normal, tolamp), 0.0, 1.0);
	position += normal * (texel * (0.5 + 1.5 * (1.0 - NdotL)));
	vec3 axes = abs(position);
	float major = max(axes.x, max(axes.y, axes.z));
	float reference = weights.y / max(major - texel * 1.5, weights.y);
	return texture(probe_shadowmap, vec4(position, weights.x), reference);
}

// share of the sun which reaches a point of the space of the probe through the windows of the cab: the ray towards the sun is taken
// to the box of the cab, and the probe drawn last (the mask) tells how much of the direction of the point it leaves the box at the cab
// fills, seen from the middle of the box. what is in the way inside the cab (the desk, the seats) holds the sun back only where it
// hides that point from the middle
float probe_sun_through_windows(vec3 position, vec3 tosun)
{
	vec3 box = cab_probe_sun_box.xyz;
	position = clamp(position, -box, box);
	vec3 invdir = (step(0.0, tosun) * 2.0 - 1.0) / max(abs(tosun), vec3(1e-5));
	vec3 far = max((box - position) * invdir, (-box - position) * invdir);
	float reach = max(min(min(far.x, far.y), far.z), 0.0);
	return 1.0 - textureLod(cab_probe_mask, position + tosun * reach, 0.0).a;
}

void probe_shade(vec4 tex_color)
{
	vec3 albedo = tex_color.rgb * probe_material_diffuse();
	vec3 position = f_pos.xyz;
	vec3 normal = normalize(f_normal);
	// the walls are drawn from both sides: the side the probe sees
	if (dot(normal, position) > 0.0)
		normal = -normal;
	vec3 light = vec3(emission);
	for (int i = 0; i < int(MAX_CAB_PROBE_LAMPS); i++)
	{
		vec4 lamp = cab_probe_lamp[i];
		if (lamp.w <= 0.0)
			break;
		vec3 tolight = lamp.xyz - position;
		float distance2 = dot(tolight, tolight);
		vec3 light_dir = tolight * inversesqrt(max(distance2, 1e-8));
		float edge = clamp(1.0 - distance2 * distance2 * (lamp.w * lamp.w), 0.0, 1.0);
		// as lamp_falloff() of common.glsl: the soft falloff (a negative radius), or the light of the sphere, which is the one of a
		// point outside it
		float radius = cab_probe_lamp_shape[i].y;
		float falloff = (radius < 0.0) ? 1.0 / max(distance2 + radius * radius, 1e-6) : 1.0 / max(distance2, (radius > 0.0) ? radius * radius : PROBE_LIGHT_MIN_DISTANCE2);
		float atten = edge * edge * falloff;
		// the cone of a spot light
		vec4 cone = cab_probe_lamp_cone[i];
		if (cone.w > -1.0)
			atten *= smoothstep(cone.w, cab_probe_lamp_shape[i].x, dot(-light_dir, normalize(cone.xyz)));
		float NdotL = max(dot(normal, light_dir), 0.0);
		if ((atten > 0.0) && (NdotL > 0.0) && (cab_probe_lamp_color[i].a > 0.5))
			atten *= 1.0 - probe_lamp_shadow(i, position - cab_probe_lamp_map[i].xyz, normal);
		light += cab_probe_lamp_color[i].rgb * (NdotL * atten);
	}
	if (cab_probe_sun.w > 0.0)
	{
		float NdotL = max(dot(normal, cab_probe_sun.xyz), 0.0);
		if (NdotL > 0.0)
			light += cab_probe_sun_color.rgb * (PROBE_SUN_SCALE * pow(NdotL, PROBE_SUN_NDOTL_SHARPNESS) * probe_sun_through_windows(position, cab_probe_sun.xyz) * cloud_shadow(cab_probe_sun_world.xyz));
	}
	out_light = vec4(albedo * light, 1.0);
	out_albedo = vec4(albedo, 1.0);
}
