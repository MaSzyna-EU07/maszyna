// physically based sky after S. Hillaire, "A Scalable and Production Ready Sky and Atmosphere Rendering Technique"
// (EGSR 2020): transmittance, multiple scattering and sky-view look-up tables. distances in kilometres, coefficients per
// kilometre. the planet is centred at the origin; the look-up tables are worked out in a frame with z up at the observer,
// the world (y up) is mapped to it by the callers. radiance is that of a sun of illuminance 1

layout(std140, set = PASS_SET, binding = ATMOSPHERE_UBO) uniform atmosphere_ubo
{
	// rgb: rayleigh scattering, w: rayleigh scale height
	vec4 atmo_rayleigh;
	// x: mie scattering, y: mie extinction, z: mie scale height, w: mie phase asymmetry
	vec4 atmo_mie;
	// rgb: ozone absorption, w: altitude of the middle of the ozone layer
	vec4 atmo_ozone;
	// rgb: albedo of the ground, w: half width of the ozone layer
	vec4 atmo_ground;
	// world space direction towards the sun (y up)
	vec4 atmo_sun_direction;
	float atmo_bottom_radius;
	float atmo_top_radius;
	// distance of the observer from the centre of the planet
	float atmo_observer_radius;
};

const float ATMO_PI = 3.14159265359;

struct atmo_medium_s
{
	vec3 scattering;
	vec3 extinction;
	vec3 rayleigh_scattering;
	vec3 mie_scattering;
};

atmo_medium_s atmo_sample_medium(float radius)
{
	float height = max(radius - atmo_bottom_radius, 0.0);
	float rayleighdensity = exp(-height / atmo_rayleigh.w);
	float miedensity = exp(-height / atmo_mie.z);
	float ozonedensity = max(0.0, 1.0 - abs(height - atmo_ozone.w) / atmo_ground.w);

	atmo_medium_s medium;
	medium.rayleigh_scattering = atmo_rayleigh.rgb * rayleighdensity;
	medium.mie_scattering = vec3(atmo_mie.x * miedensity);
	medium.scattering = medium.rayleigh_scattering + medium.mie_scattering;
	medium.extinction = medium.rayleigh_scattering + vec3(atmo_mie.y * miedensity) + atmo_ozone.rgb * ozonedensity;
	return medium;
}

// distance along a ray (unit direction) to the nearest point ahead of the origin where it meets the sphere, -1 if it does not
float atmo_ray_sphere_nearest(vec3 origin, vec3 direction, float radius)
{
	float b = dot(origin, direction);
	float c = dot(origin, origin) - radius * radius;
	float discriminant = b * b - c;
	if (discriminant < 0.0)
		return -1.0;
	float root = sqrt(discriminant);
	float t0 = -b - root;
	float t1 = -b + root;
	if (t0 < 0.0 && t1 < 0.0)
		return -1.0;
	if (t0 < 0.0)
		return t1;
	if (t1 < 0.0)
		return t0;
	return min(t0, t1);
}

float atmo_rayleigh_phase(float cosangle)
{
	return 3.0 / (16.0 * ATMO_PI) * (1.0 + cosangle * cosangle);
}

// Cornette-Shanks
float atmo_mie_phase(float cosangle)
{
	float g = atmo_mie.w;
	float k = 3.0 / (8.0 * ATMO_PI) * (1.0 - g * g) / (2.0 + g * g);
	return k * (1.0 + cosangle * cosangle) / pow(max(1.0 + g * g - 2.0 * g * cosangle, 1e-4), 1.5);
}

// texel centres of a table of the given resolution span [0, 1] of the parameter
float atmo_unit_to_sub_uv(float u, float resolution)
{
	return (u + 0.5 / resolution) * (resolution / (resolution + 1.0));
}

float atmo_sub_uv_to_unit(float u, float resolution)
{
	return (u - 0.5 / resolution) * (resolution / (resolution - 1.0));
}

// transmittance table: x the distance to the top of the atmosphere along the view, y the height
vec2 atmo_transmittance_uv(float radius, float cosviewzenith)
{
	float H = sqrt(max(0.0, atmo_top_radius * atmo_top_radius - atmo_bottom_radius * atmo_bottom_radius));
	float rho = sqrt(max(0.0, radius * radius - atmo_bottom_radius * atmo_bottom_radius));
	float discriminant = radius * radius * (cosviewzenith * cosviewzenith - 1.0) + atmo_top_radius * atmo_top_radius;
	float d = max(0.0, -radius * cosviewzenith + sqrt(max(discriminant, 0.0)));
	float dmin = atmo_top_radius - radius;
	float dmax = rho + H;
	return vec2((d - dmin) / max(dmax - dmin, 1e-6), rho / H);
}

void atmo_transmittance_params(vec2 uv, out float radius, out float cosviewzenith)
{
	float H = sqrt(max(0.0, atmo_top_radius * atmo_top_radius - atmo_bottom_radius * atmo_bottom_radius));
	float rho = H * uv.y;
	radius = sqrt(rho * rho + atmo_bottom_radius * atmo_bottom_radius);
	float dmin = atmo_top_radius - radius;
	float dmax = rho + H;
	float d = dmin + uv.x * (dmax - dmin);
	cosviewzenith = (d == 0.0) ? 1.0 : (H * H - rho * rho - d * d) / (2.0 * radius * d);
	cosviewzenith = clamp(cosviewzenith, -1.0, 1.0);
}

// multiple scattering table: x the cosine of the sun zenith angle, y the height
vec2 atmo_multiscattering_uv(float radius, float cossunzenith)
{
	vec2 uv = vec2(cossunzenith * 0.5 + 0.5, (radius - atmo_bottom_radius) / (atmo_top_radius - atmo_bottom_radius));
	return clamp(vec2(atmo_unit_to_sub_uv(uv.x, float(ATMOSPHERE_MULTISCATTERING_SIZE)), atmo_unit_to_sub_uv(uv.y, float(ATMOSPHERE_MULTISCATTERING_SIZE))), 0.0, 1.0);
}

// sky-view table: x the angle between the view and the sun around the zenith (sqrt spaced), y the angle of the view from
// the zenith, packed towards the horizon, which splits the table in the sky (lower half) and the ground (upper half)
vec2 atmo_skyview_uv(bool intersectsground, float cosviewzenith, float coslightview, float radius)
{
	float vhorizon = sqrt(max(radius * radius - atmo_bottom_radius * atmo_bottom_radius, 0.0));
	float cosbeta = vhorizon / radius;
	float beta = acos(clamp(cosbeta, -1.0, 1.0));
	float zenithhorizonangle = ATMO_PI - beta;

	vec2 uv;
	if (!intersectsground)
	{
		float coord = acos(clamp(cosviewzenith, -1.0, 1.0)) / zenithhorizonangle;
		coord = 1.0 - coord;
		coord = sqrt(max(coord, 0.0));
		coord = 1.0 - coord;
		uv.y = coord * 0.5;
	}
	else
	{
		float coord = (acos(clamp(cosviewzenith, -1.0, 1.0)) - zenithhorizonangle) / max(beta, 1e-6);
		coord = sqrt(max(coord, 0.0));
		uv.y = coord * 0.5 + 0.5;
	}
	uv.x = sqrt(clamp(-coslightview * 0.5 + 0.5, 0.0, 1.0));
	return vec2(atmo_unit_to_sub_uv(uv.x, float(ATMOSPHERE_SKYVIEW_WIDTH)), atmo_unit_to_sub_uv(uv.y, float(ATMOSPHERE_SKYVIEW_HEIGHT)));
}

void atmo_skyview_params(vec2 uv, float radius, out float cosviewzenith, out float coslightview)
{
	uv = vec2(atmo_sub_uv_to_unit(uv.x, float(ATMOSPHERE_SKYVIEW_WIDTH)), atmo_sub_uv_to_unit(uv.y, float(ATMOSPHERE_SKYVIEW_HEIGHT)));
	uv = clamp(uv, 0.0, 1.0);

	float vhorizon = sqrt(max(radius * radius - atmo_bottom_radius * atmo_bottom_radius, 0.0));
	float cosbeta = vhorizon / radius;
	float beta = acos(clamp(cosbeta, -1.0, 1.0));
	float zenithhorizonangle = ATMO_PI - beta;

	if (uv.y < 0.5)
	{
		float coord = 2.0 * uv.y;
		coord = 1.0 - coord;
		coord *= coord;
		coord = 1.0 - coord;
		cosviewzenith = cos(zenithhorizonangle * coord);
	}
	else
	{
		float coord = uv.y * 2.0 - 1.0;
		coord *= coord;
		cosviewzenith = cos(zenithhorizonangle + beta * coord);
	}
	float coord = uv.x * uv.x;
	coslightview = -(coord * 2.0 - 1.0);
}

// the same mapping for a world space view direction and sun direction (y up), as seen by the observer
vec2 atmo_skyview_uv_world(vec3 viewdirection, vec3 sundirection)
{
	float radius = atmo_observer_radius;
	vec3 origin = vec3(0.0, 0.0, radius);
	float cosviewzenith = viewdirection.y;
	// angle between the view and the sun around the zenith
	vec2 viewflat = viewdirection.xz;
	vec2 sunflat = sundirection.xz;
	float lengths = length(viewflat) * length(sunflat);
	float coslightview = (lengths > 1e-6) ? clamp(dot(viewflat, sunflat) / lengths, -1.0, 1.0) : 1.0;
	vec3 localview = vec3(sqrt(max(1.0 - cosviewzenith * cosviewzenith, 0.0)), 0.0, cosviewzenith);
	bool intersectsground = atmo_ray_sphere_nearest(origin, localview, atmo_bottom_radius) >= 0.0;
	return atmo_skyview_uv(intersectsground, cosviewzenith, coslightview, radius);
}
