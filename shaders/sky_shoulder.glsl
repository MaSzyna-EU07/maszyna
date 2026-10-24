// the skydome vertex colours carry Perez luminance scaled by a fixed exposure
// (see CSkyDome::SetExposure), so the circumsolar region and the horizon band
// routinely land well above 1.0. hard-clipping them, as this shader used to do,
// flattened everything within ~20 degrees of the sun - and most of the lower sky -
// into one solid white patch. compress the top end instead: below the knee the
// response is unchanged, above it it rolls off towards SKY_SHOULDER_WHITE, so the
// brightness gradient around the sun survives and the patch keeps a soft edge.
const float SKY_SHOULDER_KNEE = 0.25;
const float SKY_SHOULDER_WHITE = 0.95;

vec3 sky_shoulder(vec3 color)
{
	const float range = SKY_SHOULDER_WHITE - SKY_SHOULDER_KNEE;
	vec3 excess = max(color - vec3(SKY_SHOULDER_KNEE), vec3(0.0));
	return min(color, vec3(SKY_SHOULDER_KNEE)) + range * (vec3(1.0) - exp(-excess / range));
}
