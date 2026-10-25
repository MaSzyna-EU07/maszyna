// scattering along a ray through the atmosphere, see atmosphere_common.glsl. the includer declares the samplers
// atmo_transmittance_lut and (when multiple scattering is used) atmo_multiscattering_lut

// above the ground by this much (km), so the test for the shadow of the planet does not hit the point itself
const float ATMO_PLANET_RADIUS_OFFSET = 0.01;

struct atmo_scattering_s
{
	// light scattered towards the origin
	vec3 luminance;
	// transmittance from the origin to the end of the ray
	vec3 transmittance;
	// light scattered once more, as if the light arriving at every point was 1 (multiple scattering table)
	vec3 multiscattering_as_1;
};

vec3 atmo_transmittance_to_top(float radius, float cossunzenith)
{
	return texture(atmo_transmittance_lut, atmo_transmittance_uv(radius, cossunzenith)).rgb;
}

// Origin and Direction (unit) in the frame of the planet. with Ground the light the ground reflects at the end of the ray
// is included. Physicalphase: rayleigh and mie phase functions, otherwise isotropic (multiple scattering table).
// Multiscattering: light scattered more than once comes from the multiple scattering table
atmo_scattering_s atmo_integrate(vec3 origin, vec3 direction, vec3 sundirection, bool ground, int samples, bool physicalphase, bool multiscattering)
{
	atmo_scattering_s result;
	result.luminance = vec3(0.0);
	result.transmittance = vec3(1.0);
	result.multiscattering_as_1 = vec3(0.0);

	float tbottom = atmo_ray_sphere_nearest(origin, direction, atmo_bottom_radius);
	float ttop = atmo_ray_sphere_nearest(origin, direction, atmo_top_radius);
	float tmax;
	if (tbottom < 0.0)
	{
		if (ttop < 0.0)
			return result;
		tmax = ttop;
	}
	else
	{
		tmax = (ttop > 0.0) ? min(ttop, tbottom) : tbottom;
	}

	float cosangle = dot(direction, sundirection);
	const float uniformphase = 1.0 / (4.0 * ATMO_PI);
	float miephase = physicalphase ? atmo_mie_phase(cosangle) : uniformphase;
	float rayleighphase = physicalphase ? atmo_rayleigh_phase(cosangle) : uniformphase;

	vec3 throughput = vec3(1.0);
	float invsamples = 1.0 / float(samples);
	for (int s = 0; s < samples; ++s)
	{
		// the steps grow with the distance: the medium near the observer matters most
		float t0 = float(s) * invsamples;
		float t1 = float(s + 1) * invsamples;
		t0 = t0 * t0 * tmax;
		t1 = t1 * t1 * tmax;
		float dt = t1 - t0;
		float t = t0 + dt * 0.3;

		vec3 position = origin + t * direction;
		float radius = length(position);
		vec3 up = position / radius;
		atmo_medium_s medium = atmo_sample_medium(radius);
		vec3 extinction = max(medium.extinction, vec3(1e-7));
		vec3 sampletransmittance = exp(-extinction * dt);

		float cossunzenith = dot(sundirection, up);
		vec3 suntransmittance = atmo_transmittance_to_top(radius, cossunzenith);
		float earthshadow = (atmo_ray_sphere_nearest(position + up * ATMO_PLANET_RADIUS_OFFSET, sundirection, atmo_bottom_radius) >= 0.0) ? 0.0 : 1.0;

		vec3 multiscatteredluminance = vec3(0.0);
#ifdef ATMO_MULTISCATTERING_LUT
		if (multiscattering)
			multiscatteredluminance = texture(atmo_multiscattering_lut, atmo_multiscattering_uv(radius, cossunzenith)).rgb;
#endif

		vec3 phasescattering = medium.mie_scattering * miephase + medium.rayleigh_scattering * rayleighphase;
		vec3 S = earthshadow * suntransmittance * phasescattering + multiscatteredluminance * medium.scattering;
		// analytic integral over the step of the light scattered at the sample, attenuated on the way to its start
		vec3 Sint = (S - S * sampletransmittance) / extinction;
		result.luminance += throughput * Sint;

		vec3 MSint = (medium.scattering - medium.scattering * sampletransmittance) / extinction;
		result.multiscattering_as_1 += throughput * MSint;

		throughput *= sampletransmittance;
	}

	if (ground && tbottom > 0.0 && tmax == tbottom)
	{
		vec3 position = origin + tbottom * direction;
		float radius = length(position);
		vec3 up = position / radius;
		float cossunzenith = dot(sundirection, up);
		vec3 suntransmittance = atmo_transmittance_to_top(radius, cossunzenith);
		float NdotL = clamp(dot(up, sundirection), 0.0, 1.0);
		result.luminance += suntransmittance * throughput * NdotL * atmo_ground.rgb / ATMO_PI;
	}

	result.transmittance = throughput;
	return result;
}
