// fog between the camera and a fragment, see fog_volume.glsl and weather_common.glsl. a draw with a fog_density of 0 is not
// fogged; one with another density than the regular one (the glare of lamps) gets the optical depth scaled by the ratio
#include "fog_volume.glsl"

// rgb: light the fog adds, a: what it lets through
vec4 fog_at_fragment()
{
	if (fog_density <= 0.0)
		return vec4(0.0, 0.0, 0.0, 1.0);
	vec4 fog = fog_along_view(f_pos.xyz, gl_FragCoord.xy);
	float scale = fog_density / max(fog_light.w, 1e-12);
	if (abs(scale - 1.0) > 1e-3)
	{
		float transmittance = exp(log(max(fog.a, 1e-8)) * scale);
		fog.rgb *= (1.0 - transmittance) / max(1.0 - fog.a, 1e-4);
		fog.a = transmittance;
	}
	return fog;
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
