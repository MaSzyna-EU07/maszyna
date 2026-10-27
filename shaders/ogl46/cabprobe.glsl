// the shading of the probe of the occupied cab (gfx::cab_probe), for cabprobe.frag and cabprobe_alpha.frag: what the cab shows
// towards the middle of its box lit by the cab lamp alone, with the cube shadow map of the lamp (and what glows in it, the lights
// of the desk), and its albedo, each with 1 in the alpha: the probe is cleared to nothing, so the alpha of a direction tells how
// much of it the cab fills, and both are premultiplied by it. the light of the cab lamp is the one the shading gives it
// (calc_local_light() in light_common.glsl); the ambient light and the sun are left out, the cab gets those as before. the
// parameters of the lamp are in the block of the scene (gl::scene_ubs::cab_probe_lamp), in the space of the probe

layout(location = 0) out vec4 out_light;
layout(location = 1) out vec4 out_albedo;

// closest distance (squared) the light of the lamp is computed for, as LOCAL_LIGHT_MIN_DISTANCE2 of light_common.glsl
const float PROBE_LIGHT_MIN_DISTANCE2 = 0.01;

// the diffuse colour of the submodel or shape, as material_diffuse() of light_common.glsl
vec3 probe_material_diffuse()
{
	vec3 diffusecolor = pow(max(param[0].rgb, vec3(0.0)), vec3(2.2));
	float value = max(param[0].r, max(param[0].g, param[0].b));
	return (value > 0.0 ? diffusecolor * (param[1].x / value) : vec3(0.0));
}

// share of the light of the cab lamp held back before the point of the space of its cube shadow map (origin at the lamp), 0..1, as
// local_light_shadow() of light_common.glsl takes it with a single sample
float probe_lamp_shadow(vec3 position, vec3 normal)
{
	vec4 weights = cab_probe_lamp_shadow;
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

void probe_shade(vec4 tex_color)
{
	vec3 albedo = tex_color.rgb * probe_material_diffuse();
	vec3 position = f_pos.xyz;
	vec3 normal = normalize(f_normal);
	// the walls are drawn from both sides: the side the probe sees
	if (dot(normal, position) > 0.0)
		normal = -normal;
	vec3 light = vec3(emission);
	if (cab_probe_lamp.w > 0.0)
	{
		vec3 tolight = cab_probe_lamp.xyz - position;
		float distance2 = dot(tolight, tolight);
		vec3 light_dir = tolight * inversesqrt(max(distance2, 1e-8));
		float edge = clamp(1.0 - distance2 * distance2 * (cab_probe_lamp.w * cab_probe_lamp.w), 0.0, 1.0);
		float atten = edge * edge / max(distance2, PROBE_LIGHT_MIN_DISTANCE2);
		float NdotL = max(dot(normal, light_dir), 0.0);
		if ((atten > 0.0) && (NdotL > 0.0) && (cab_probe_lamp_color.a > 0.5))
			atten *= 1.0 - probe_lamp_shadow(-tolight, normal);
		light += cab_probe_lamp_color.rgb * (NdotL * atten);
	}
	out_light = vec4(albedo * light, 1.0);
	out_albedo = vec4(albedo, 1.0);
}
