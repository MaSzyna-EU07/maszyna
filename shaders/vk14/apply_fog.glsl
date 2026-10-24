// fog between the camera and a fragment, the api of shaders/ogl46/apply_fog.glsl. the fog volume and the weather of the opengl
// renderer come with the stage P2 of the plan; until then the fog is analytic: exponential with the distance, of the density of
// the draw (fog_density of the model block, a draw with 0 is not fogged), of the colour fog_color of the light buffer, turned
// towards the colour of the sun around its direction
// rgb: light the fog adds, a: what it lets through
vec4 fog_at_fragment()
{
	if (fog_density <= 0.0)
		return vec4(0.0, 0.0, 0.0, 1.0);
	float amount = 1.0 - clamp(exp(-length(f_pos.xyz) * fog_density), 0.0, 1.0);
	vec3 color = fog_color;
	if ((lights_count >= 1U) && (lights[0].type == LIGHT_DIR))
	{
		float sunward = pow(max(dot(normalize(f_pos.xyz), normalize(-lights[0].dir)), 0.0), 30.0);
		color = mix(color, lights[0].color, sunward);
	}
	return vec4(color * amount, 1.0 - amount);
}

vec3 get_fog_color()
{
	vec4 fog = fog_at_fragment();
	return fog.rgb / max(1.0 - fog.a, 1e-4);
}

float get_fog_amount()
{
	return 1.0 - fog_at_fragment().a;
}

vec3 apply_fog(vec3 color)
{
	vec4 fog = fog_at_fragment();
	return color * fog.a + fog.rgb;
}

vec3 add_fog(vec3 color, float amount)
{
	return color + fog_at_fragment().rgb * amount;
}
