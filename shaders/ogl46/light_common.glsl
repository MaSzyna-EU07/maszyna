layout(binding = SHADOW_TEX) uniform sampler2DArrayShadow shadowmap;
layout(binding = HEADLIGHT_TEX) uniform sampler2D headlightmap;
// cube shadow maps of the lamps, see gl::LOCAL_SHADOW_TEX and opengl46_local_shadows
layout(binding = LOCAL_SHADOW_TEX) uniform samplerCubeArrayShadow local_shadowmap;
// the same for the lamps of scenery models, the lights with linear 1
layout(binding = LOCAL_SHADOW_LAMPS_TEX) uniform samplerCubeArrayShadow local_shadowmap_lamps;
// shadow maps of the headlights of vehicles, see gl::HEADLIGHT_SHADOW_TEX and gl::scene_ubs::headlight_shadow
layout(binding = HEADLIGHT_SHADOW_TEX) uniform sampler2DArrayShadow headlight_shadowmap;
// depth of the opaque scene drawn before the colour, for the contact shadows (gl::scene_ubs::contact_shadow_params)
layout(binding = SCENE_DEPTH_TEX) uniform sampler2D scene_depth;
// the ambient occlusion of the main view, half its size, for the ambient light (gl::scene_ubs::ao_params)
layout(binding = SCENE_AO_TEX) uniform sampler2D scene_ao;
// the probe of the occupied cab (gl::scene_ubs::cab_probe_box, gfx::cab_probe): the light of its lamps the cab reflects and its
// albedo, each premultiplied by the share of the direction the cab fills, which is in their alpha, and the light the cab lights
// itself with again (gl::cab_probe_data)
layout(binding = CAB_PROBE_TEX) uniform samplerCube cab_probe_light;
layout(binding = CAB_PROBE_ALBEDO_TEX) uniform samplerCube cab_probe_albedo;
layout(std430, binding = CAB_PROBE_SSBO_BINDING) readonly buffer cab_probe_ssbo
{
    vec4 cab_probe_sh[9];
};

#include "envmapping.glsl"
#include "ssr.glsl"
#include "conversion.glsl"
#include "weather_common.glsl"
#include "sky_occlusion.glsl"

float glossiness = 1.0;
float metalic = 0.0;
layout(location = 7) in vec4 UserData;
#pragma param(weathering, 2, 3, 1, zero)

// ---------------------------------------------------------------------
// Lighting balance tunables - tweak these to control overall scene
// exposure without touching tonemapping.glsl.
//
// AMBIENT_SCALE:        brightness of SHADED faces (indirect/sky term).
//                       Lower -> deeper shadows, less burn under bright
//                       textures. Higher -> flatter / brighter shading.
//
// SUN_DIFFUSE_SCALE:    brightness of UNSHADED (sun-lit) faces. Lower
//                       this to dim hot surfaces in direct sunlight
//                       without affecting shaded areas. Was 3.5; 2.5
//                       calmly fits the ACES tonemap shoulder.
//
// SUN_NDOTL_SHARPNESS:  N.L curve on the sun. 1.0 = pure Lambert, higher
//                       = sharper terminator (more contrast between
//                       lit and shaded faces of the same surface).
// ---------------------------------------------------------------------
const float AMBIENT_SCALE       = 0.65;
const float SUN_DIFFUSE_SCALE   = 1.5;
const float SUN_NDOTL_SHARPNESS = 1.25;

float length2(vec3 v)
{
        return dot(v, v);
}

// the largest difference of the distance of a texel of the ambient occlusion from the one of the fragment (a share of it) the
// texel is taken to be of the surface of the fragment at
const float AO_SURFACE_MATCH = 0.05;

// how many more lamps and headlights the fragment traces contact shadows to, see add_headlights()
int contact_shadow_budget = 0;

// contact shadow of a light in direction Light_dir (view space, towards the light), 0..1: whether the depth of the scene drawn
// before the colour (scene_depth) holds a surface between the fragment and a point up to Length metres towards the light. it
// catches what the shadow maps are too coarse for (rails on the sleepers, bolts, edges of panels, where a thing stands on the
// ground), and things too small or near for them. only what is in the depth casts it, and only within the image.
// the ray starts a pixel and a half off the visible side of the surface: the depth there, sampled anywhere within a pixel of
// the projected point, then stays behind the ray on a flat surface, so it doesn't shadow itself. a surface nearer than the
// ray by more than the thickness is taken for something in front of the ray, not something it goes through.
// the shadow fades over the last part of the ray (it doesn't end with an edge) and over the last quarter of the range
float contact_shadow(vec3 light_dir, float raylength, float steps)
{
	float range = contact_shadow_params.y;
	vec3 position = f_pos.xyz;
	float fade = 1.0 - smoothstep(range * 0.75, range, length(position));
	if ((fade <= 0.0) || (raylength <= 0.0) || (steps < 1.0))
		return 0.0;
	vec3 normal = normalize(f_normal);
	if (dot(normal, position) > 0.0)
		normal = -normal;
	// (not "<= 0.0": a model with a normal of zero length gives NaN here, and the march with it would fetch far out of the depth)
	if (!(dot(normal, light_dir) > 0.0))
		return 0.0;
	vec2 size = vec2(textureSize(scene_depth, 0));
	// the projection is perspective with the depth n / w (reverse z, infinite far plane): w is the distance along the axis
	// of the view, and a depth d is a surface at n / d
	float near = projection[3][2];
	float distance = (projection * vec4(position, 1.0)).w;
	float pixel = 2.0 * distance / (abs(projection[1][1]) * size.y);
	vec4 start = projection * vec4(position + normal * (pixel * 1.5), 1.0);
	vec4 ray = projection * vec4(light_dir * raylength, 0.0);
	float thickness = 0.3 + pixel * 4.0;
	float bias = pixel * 0.5;
	// a different start along the ray in neighbouring pixels: the steps give noise instead of bands
	float jitter = fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));
	for (float i = 0.0; i < steps; i += 1.0)
	{
		float t = (i + jitter) / steps;
		vec4 clip = start + ray * t;
		if (clip.w <= near)
			break;
		vec2 coords = (clip.xy / clip.w) * 0.5 + 0.5;
		if (any(lessThan(coords, vec2(0.0))) || any(greaterThan(coords, vec2(1.0))))
			break;
		float depth = texelFetch(scene_depth, clamp(ivec2(coords * size), ivec2(0), ivec2(size) - 1), 0).r;
		// nothing drawn there (the sky)
		if (depth <= 0.0)
			continue;
		float nearer = clip.w - near / depth;
		if ((nearer > bias) && (nearer < thickness))
			return fade * (1.0 - smoothstep(0.6, 1.0, t));
	}
	return 0.0;
}

// contact shadow of a lamp or headlight Distance metres away, whose light reaches the fragment with Strength, for the first
// few lamps of a fragment (contact_shadow_lamps.x) which light it noticeably. the ray goes at most half way to the light,
// so the lamp doesn't shadow with its own fitting
float contact_shadow_lamp(vec3 light_dir, float distance, float strength)
{
	if ((contact_shadow_budget <= 0) || (strength < 0.005))
		return 0.0;
	--contact_shadow_budget;
	return contact_shadow(light_dir, min(contact_shadow_params.x, distance * 0.5), contact_shadow_params.w);
}

// share of the sun (the moon) held back before the fragment by the shadow map and the contact shadow, 0..1
float calc_shadow_map();
float calc_shadow()
{
	float shadow = calc_shadow_map();
	if ((contact_shadow_params.x > 0.0) && (shadow < 1.0))
		shadow = max(shadow, contact_shadow(normalize(-lights[0].dir), contact_shadow_params.x, contact_shadow_params.z));
	return shadow;
}

float calc_shadow_map()
{
	if (SHADOWMAP_ENABLED)
	{
		float distance = dot(f_pos.xyz, f_pos.xyz);
		uint cascade;
		for (cascade = 0U; cascade < MAX_CASCADES; cascade++)
			if (distance <= cascade_end[cascade])
				break;
		// cascades fitted to the depth of the view of an earlier frame (gfx.shadow.sdsm) cover only the area of the points seen
		// then: a point at the edge of the view or one which came closer since may be outside the map of its cascade. it goes to
		// the first further cascade whose map holds it with the reach of the filter, and is lit past the last one
		if (shadow_fit.x > 0.0)
		{
			float border = 4.0 / float(textureSize(shadowmap, 0).x);
			for (; cascade < MAX_CASCADES; cascade++)
			{
				vec2 fitcoords = (lightview[cascade] * vec4(f_pos.xyz, 1.0)).xy;
				if (all(greaterThanEqual(fitcoords, vec2(border))) && all(lessThanEqual(fitcoords, vec2(1.0 - border))))
					break;
			}
			if (cascade >= MAX_CASCADES)
				return 0.0;
		}
		// Normal offset: sample the shadow map from a point pushed along the surface normal
		// by a fraction of the cascade's texel footprint. A constant depth bias can't keep up
		// with large texels (far cascades, small shadow maps) on surfaces at a grazing angle
		// to the sun, which then show shadow acne stripes. lightview maps view space metres
		// to [0,1] shadow map coordinates, so its x/y row lengths give texels per metre.
		vec3 shadownormal = normalize(f_normal);
		float shadowNdotL = clamp(dot(shadownormal, normalize(-lights[0].dir)), 0.0, 1.0);
		mat4 shadowview = lightview[cascade];
		float texelspermetre = min(length(vec3(shadowview[0][0], shadowview[1][0], shadowview[2][0])),
		                            length(vec3(shadowview[0][1], shadowview[1][1], shadowview[2][1]))) * float(textureSize(shadowmap, 0).x);
		float texelsize = 1.0 / texelspermetre;
		vec3 shadowpos = f_pos.xyz + shadownormal * (texelsize * 1.5 * (1.0 - shadowNdotL));
		vec4 lightpos = shadowview * vec4(shadowpos, 1.0);
		vec3 coords = lightpos.xyz / lightpos.w;
		if (coords.z < 0.0)
			return 0.0f;
		
		

		float bias = 0.00005f * float(cascade + 1U);

		// Fast path: 4 textureGather() calls cover a 4x4 texel block. Raw gathered
		// comparisons must be weighted by the sub-texel position (bilinear weights),
		// otherwise the result is constant across each shadow map texel and shadow
		// edges turn into texel-sized stair steps. With per-axis weights
		// (1-f, 1, 1, f) this equals a 3x3 grid of hardware bilinear PCF taps.
		vec2 size = vec2(textureSize(shadowmap, 0).xy);
		vec2 uv = coords.xy * size - 0.5;
		vec2 base = floor(uv);
		vec2 f = uv - base;
		float refz = coords.z + bias;
		float layer = float(cascade);
		// sampling at texel corners selects blocks with lower-left texel base-1 and base+1
		vec4 g00 = textureGather(shadowmap, vec3((base + vec2(0.0, 0.0)) / size, layer), refz);
		vec4 g10 = textureGather(shadowmap, vec3((base + vec2(2.0, 0.0)) / size, layer), refz);
		vec4 g01 = textureGather(shadowmap, vec3((base + vec2(0.0, 2.0)) / size, layer), refz);
		vec4 g11 = textureGather(shadowmap, vec3((base + vec2(2.0, 2.0)) / size, layer), refz);
		// gather component order: x=(0,1) y=(1,1) z=(1,0) w=(0,0)
		float shadow = dot(g00, vec4(1.0 - f.x, 1.0, 1.0 - f.y, (1.0 - f.x) * (1.0 - f.y)))
		             + dot(g10, vec4(1.0, f.x, f.x * (1.0 - f.y), 1.0 - f.y))
		             + dot(g01, vec4((1.0 - f.x) * f.y, f.y, 1.0, 1.0 - f.x))
		             + dot(g11, vec4(f.y, f.x * f.y, f.x, 1.0));
		return shadow / 9.0;
	}
	else
	{
		return 0.0;
	}
}

// -----------------------------------------------------------------------
// GGX Microfacet BRDF helpers (Cook-Torrance)
// -----------------------------------------------------------------------

// Trowbridge-Reitz (GGX) Normal Distribution Function
//   D(N,H,α) = α⁴ / (π · ((NdotH)²·(α⁴−1)+1)²)
//   α = roughness² (perceptual remapping so the slider feels linear)
float D_GGX(float NdotH, float roughness)
{
    float a  = roughness * roughness;   // perceptual -> linear roughness
    float a2 = a * a;
    float d  = (NdotH * NdotH) * (a2 - 1.0) + 1.0;
    return a2 / (3.14159265359 * d * d);
}

// Schlick-GGX single-term masking/shadowing (k remapped for direct lighting)
float G_SchlickGGX(float NdotX, float roughness)
{
    float r = roughness + 1.0;
    float k = (r * r) * (1.0 / 8.0);   // k_direct = (roughness+1)²/8
    return NdotX / (NdotX * (1.0 - k) + k);
}

// Separable Smith geometry term (not height-correlated)
//   G(N,V,L) = G_SchlickGGX(NdotV) · G_SchlickGGX(NdotL)
float G_Smith(float NdotV, float NdotL, float roughness)
{
    return G_SchlickGGX(NdotV, roughness) * G_SchlickGGX(NdotL, roughness);
}

const float BRDF_PI = 3.14159265359;
// the smoothest surface the lights are reflected by: a mirror would make a highlight smaller than a pixel
const float MIN_ROUGHNESS = 0.045;
// cap of the brdf for the lamps and headlights (see specular_light()): a tiny light next to a polished surface
const float LOCAL_LIGHT_BRDF_LIMIT = 16.0;
// no cap for the sun
const float SUN_BRDF_LIMIT = 1.0e6;

// the specular reflectance at normal incidence and the perceptual roughness the lights are reflected with, worked out by
// apply_lights() for the fragment before it adds the lamps (add_local_lights())
vec3 spec_f0 = vec3(0.04);
float spec_roughness = 0.5;

// a matte material (mat_default) reflects nothing: neither the surroundings nor the sun and the lamps, only the diffuse light
bool matte = false;

// light of a light from Light_dir reflected by the surface, in the units of the diffuse shading (a white lambertian surface
// facing the light reflects 1 of it): pi times the cook-torrance brdf with schlick's fresnel of F0, times n.l. Limit caps the
// brdf without the n.l
vec3 specular_light(vec3 light_dir, vec3 fragnormal, vec3 f0, float roughness, float limit)
{
    float NdotL = dot(fragnormal, light_dir);
    if (NdotL <= 0.0)
        return vec3(0.0);
    vec3 V = normalize(-f_pos.xyz);
    vec3 H = normalize(light_dir + V);
    float NdotV = max(dot(fragnormal, V), 1e-4);
    float NdotH = max(dot(fragnormal, H), 0.0);
    float VdotH = max(dot(V, H), 0.0);
    float brdf = D_GGX(NdotH, roughness) * G_Smith(NdotV, NdotL, roughness) / (4.0 * NdotL * NdotV);
    vec3 fresnel = f0 + (1.0 - f0) * pow(1.0 - VdotH, 5.0);
    return fresnel * (min(brdf, limit) * NdotL * BRDF_PI);
}

// perceptual roughness raised where the normal changes much from a pixel to the next (normal maps, tight curves), so the
// highlight of a detail smaller than a pixel spreads over the pixel instead of flickering. Kaplanyan, Tokuyoshi, "Improved
// geometric specular antialiasing", 2019
float specular_antialiasing(vec3 fragnormal, float roughness)
{
    vec3 dndx = dFdx(fragnormal);
    vec3 dndy = dFdy(fragnormal);
    float variance = 0.25 * (dot(dndx, dndx) + dot(dndy, dndy));
    float kernel = min(2.0 * variance, 0.18);
    float a2 = roughness * roughness * roughness * roughness;
    return sqrt(sqrt(clamp(a2 + kernel, 0.0, 1.0)));
}

// the materials without a specgloss map give glossiness: (param[1].w, 10 by default) as the exponent of the blinn-phong
// highlight of the old renderer. a lobe of exponent n is about a ggx one of alpha sqrt(2 / (n + 2))
float phong_roughness(float exponent)
{
    return sqrt(sqrt(2.0 / (max(exponent, 0.0) + 2.0)));
}

// glossiness which apply_lights() turns into the roughness of a blinn-phong exponent (it takes 1 - glossiness / param[1].w)
float phong_glossiness(float exponent)
{
    return (1.0 - phong_roughness(exponent)) * max(abs(param[1].w), 1.0);
}

// the reflection of the surroundings of a material without a specgloss map is a smooth varnish over the rough surface: the
// old renderer showed it as a sharp reflection, as much of it as reflection: (and the masks of the material) asked for
bool legacy_coat = false;

// set up the gloss of a material without a specgloss map, before apply_lights()
void legacy_gloss()
{
    glossiness = phong_glossiness(abs(param[1].w));
    legacy_coat = true;
}

// Returns vec2(diffuse, specular) for a single punctual light.
//
//   diffuse  – Lambert N·L (Fresnel-weighted diffuse is handled per-material
//              in apply_lights, so we return raw N·L here).
//   specular – Cook-Torrance GGX: D·G / (4·NdotL·NdotV).
//              The Fresnel factor (F) is intentionally omitted here;
//              apply_lights already carries a per-material Fresnel term
//              that is applied to env reflections and can be routed to
//              direct specular there.
//
// Roughness is derived identically to env_roughness in apply_lights so
// that direct and indirect specular highlights read consistently.
vec2 calc_light(vec3 light_dir, vec3 fragnormal)
{
    vec3 N = fragnormal;
    vec3 L = light_dir;
    vec3 V = normalize(-f_pos.xyz);
    vec3 H = normalize(L + V);

    float NdotL = max(dot(N, L), 0.0);
    float NdotV = max(dot(N, V), 1e-4);
    float NdotH = max(dot(N, H), 0.0);

    float diffuse_v = NdotL;

    // Mirror the env-map roughness derivation so direct and indirect lobes match.
    // glossiness == param[1].w  →  roughness == 0.04 (near-mirror)
    // glossiness == 0           →  roughness == 1.0  (fully diffuse)
    float roughness = clamp(1.0 - glossiness / max(abs(param[1].w), 1.0), 0.04, 1.0);

    // Cook-Torrance specular (no Fresnel — see above):
    //   f_spec = D(N,H,α) · G(N,V,L,α) / (4 · NdotL · NdotV)
    float D = D_GGX(NdotH, roughness);
    float G = G_Smith(NdotV, NdotL, roughness);
    float specular_v = (NdotL > 0.0)
        ? (D * G) / max(4.0 * NdotL * NdotV, 1e-4)
        : 0.0;

    return vec2(diffuse_v, specular_v);
}

vec2 calc_point_light(light_s light, vec3 fragnormal)
{
	vec3 light_dir = normalize(light.pos - f_pos.xyz);
	vec2 val = calc_light(light_dir, fragnormal);
	val.x += light.ambient;
	val *= light.intensity;
	
	float distance = length(light.pos - f_pos.xyz);
	float atten = 1.0f / (distance * distance);
	//float atten = 1.0f / (1.0f + light.linear * distance + light.quadratic * (distance * distance));
	
	return val * atten;
}

vec2 calc_spot_light(light_s light, vec3 fragnormal)
{
	vec3 light_dir = normalize(light.pos - f_pos.xyz);
	
	float theta = dot(light_dir, normalize(-light.dir));
	float epsilon = light.in_cutoff - light.out_cutoff;
	float intensity = clamp((theta - light.out_cutoff) / epsilon, 0.0, 1.0);

	vec2 point = calc_point_light(light, fragnormal);	
	return point * intensity;
}

vec2 calc_dir_light(light_s light, vec3 fragnormal)
{
	vec3 light_dir = normalize(-light.dir);
	return calc_light(light_dir, fragnormal);
}

// samples of the soft shadows of the lamps and headlights: a disc, rotated per pixel
const vec2 SHADOW_DISC[8] = vec2[](vec2(-0.326, -0.406), vec2(-0.840, -0.074), vec2(-0.696, 0.457), vec2(-0.203, 0.621),
                                   vec2(0.962, -0.195), vec2(0.473, -0.480), vec2(0.519, 0.767), vec2(0.185, -0.893));

float shadow_disc_angle()
{
	return 6.2831853 * fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));
}

// share of the beam of headlights with a shadow map (Map, headlight_weights.w - 1 of the light) held back before the fragment,
// 0..1. the map was drawn from the projector of the beam: x and y of the projection across it, z / w the depth (near plane over
// the distance along the axis, as the reverse z of the scene), w the distance. the comparison gives 1 where the map holds a
// value at least as large as the reference: a surface closer to the projector
float headlight_beam_shadow(uint map, vec3 fragnormal, vec3 light_dir)
{
	mat4 tomap = headlight_shadow[map];
	vec4 params = headlight_shadow_params[map];
	// a texel of the map is as large as this here. the point is moved along the normal by about a texel, and the reference towards
	// the projector by a part of one. only the back faces are in the map, which leaves the lit faces little to shadow themselves with
	float texel = params.x * max((tomap * vec4(f_pos.xyz, 1.0)).w, 0.0);
	float NdotL = clamp(dot(fragnormal, light_dir), 0.0, 1.0);
	vec4 position = tomap * vec4(f_pos.xyz + fragnormal * (texel * (0.5 + 1.5 * (1.0 - NdotL))), 1.0);
	if (position.w <= 0.0)
		return 0.0;
	vec2 coords = position.xy / position.w;
	if (any(lessThan(coords, vec2(0.0))) || any(greaterThan(coords, vec2(1.0))))
		return 0.0;
	// nearer to the projector than its near plane (the vehicle itself) the reference is past anything in the map
	float reference = position.z / max(position.w - texel * 1.5, 1e-3);
	float layer = float(map);
	float radius = params.y;
	if (radius <= 0.0)
		return texture(headlight_shadowmap, vec4(coords, layer, reference));
	float angle = shadow_disc_angle();
	vec2 rotation = vec2(cos(angle), sin(angle));
	float spread = radius / float(textureSize(headlight_shadowmap, 0).x);
	int count = (radius > 1.5) ? 8 : 4;
	float shadow = 0.0;
	for (int i = 0; i < count; i++)
	{
		vec2 offset = vec2(SHADOW_DISC[i].x * rotation.x - SHADOW_DISC[i].y * rotation.y, SHADOW_DISC[i].x * rotation.y + SHADOW_DISC[i].y * rotation.x) * spread;
		shadow += texture(headlight_shadowmap, vec4(coords + offset, layer, reference));
	}
	return shadow / float(count);
}

// dpdx and dpdy are the change of the view space position of the fragment from a pixel to the next one. the beam texture is
// sampled with gradients worked out from them instead of the ones of the hardware: those are undefined where the fragments of
// a quad do not go through the same lights, which is what happens at the edges of the clusters
vec2 calc_headlights(light_s light, vec3 fragnormal, vec3 dpdx, vec3 dpdy)
{
	vec4 headlightpos = light.headlight_projection * f_pos;
	vec3 coords = headlightpos.xyz / headlightpos.w;

	if (coords.z > 1.0)
		return vec2(0.0);
	if (coords.z < 0.0)
		return vec2(0.0);
	// the beam is what the projection takes in. beyond its sides the texture would be clamped to its edge, and near the projector
	// (where a pixel spans many texels) its coarsest levels spill the average of the whole beam over the surroundings
	if (coords.x < 0.0 || coords.x > 1.0 || coords.y < 0.0 || coords.y > 1.0)
		return vec2(0.0);

	vec3 light_dir = normalize(light.pos - f_pos.xyz);
	// Tighter wrap (was +0.25): faces angled away from the headlight cone
	// fall off to dark much faster, so cab/exterior surfaces read with a
	// clear directional shape instead of a flat half-lit wash.
	vec2 part = vec2(1.0) * clamp(dot(fragnormal, light_dir) + 0.10, 0.0, 1.0);
	float distance = length(light.pos - f_pos.xyz);
	float atten = 1.0f / (1.0f + light.linear * distance + light.quadratic * (distance * distance));
	atten *= mix(1.0, 0.0, clamp((coords.z - 0.998) * 500.0, 0.0, 1.0));
	vec2 beamcoords = headlightpos.xy / headlightpos.w;
	vec4 dheadlightposdx = light.headlight_projection * vec4(dpdx, 0.0);
	vec4 dheadlightposdy = light.headlight_projection * vec4(dpdy, 0.0);
	vec2 dbeamdx = (dheadlightposdx.xy - beamcoords * dheadlightposdx.w) / headlightpos.w;
	vec2 dbeamdy = (dheadlightposdy.xy - beamcoords * dheadlightposdy.w) / headlightpos.w;
	vec3 lights = textureGrad(headlightmap, beamcoords, dbeamdx, dbeamdy).rgb * light.headlight_weights.rgb;
	float lightintensity = max(max(lights.r, lights.g), lights.b);
	// headlights with a shadow map: what is in the way holds the beam back
	if ((light.headlight_weights.w > 0.5) && (lightintensity > 0.0) && (part.x > 0.0))
		lightintensity *= 1.0 - headlight_beam_shadow(uint(light.headlight_weights.w - 0.5), fragnormal, light_dir);
	if ((contact_shadow_budget > 0) && (lightintensity > 0.0) && (part.x > 0.0))
		lightintensity *= 1.0 - contact_shadow_lamp(light_dir, distance,
		                                            part.x * atten * lightintensity * light.intensity * max(light.color.r, max(light.color.g, light.color.b)));
	// x the diffuse light, y the light the specular reflection takes (specular_light() brings its own n.l)
	return vec2(part.x, 1.0) * (atten * lightintensity);
}

// one comparison with the cube shadow map at the point of the space of the map, moved towards the lamp by Bias (metres). the
// map is in the array of the scenery lamps when Lamps, in the one of the cab lamp otherwise
float local_light_shadow_tap(vec3 position, float layer, float near, float bias, bool lamps)
{
	vec3 axes = abs(position);
	float major = max(axes.x, max(axes.y, axes.z));
	float reference = near / max(major - bias, near);
	return lamps ? texture(local_shadowmap_lamps, vec4(position, layer), reference) : texture(local_shadowmap, vec4(position, layer), reference);
}

// share of the light of a lamp with a cube shadow map (headlight_weights.x is its layer) which reaches the fragment, 0..1.
// the map holds n / d of the closest surface, d the distance along the axis of the face, n the near plane: the farther, the
// smaller (as the depth of the scene, reverse z). the comparison gives 1 where the map holds a value at least as large as
// the reference, i.e. a surface at most as far: in shadow
float local_light_shadow(light_s light, vec3 fragnormal)
{
	// the space of the map: origin at the lamp, axes of the model the shadows are cast by
	vec3 position = (light.headlight_projection * vec4(f_pos.xyz, 1.0)).xyz;
	vec3 normal = normalize(mat3(light.headlight_projection) * fragnormal);
	float distance = length(position);
	// a texel of the map is as large as this here. the point is moved along the normal by about a texel, which keeps a surface
	// from shadowing itself at grazing angles, and the reference is moved towards the lamp by a part of a texel
	float texel = light.headlight_weights.z * distance;
	vec3 tolamp = -position / max(distance, 1e-4);
	float NdotL = clamp(dot(normal, tolamp), 0.0, 1.0);
	position += normal * (texel * (0.5 + 1.5 * (1.0 - NdotL)));
	float bias = texel * 1.5;
	float layer = light.headlight_weights.x;
	float radius = light.headlight_weights.w;
	bool lamps = (light.linear > 0.5);
	if (radius <= 0.0)
		return local_light_shadow_tap(position, layer, light.headlight_weights.y, bias, lamps);
	// a disc of samples in the plane of the surface, each compared with its own distance: points of the surface itself, which
	// samples spread across the direction to the lamp would not be at grazing angles. rotated per pixel to turn the banding into noise
	vec3 side = normalize(cross(normal, (abs(normal.y) < 0.99) ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0)));
	vec3 up = cross(normal, side);
	float angle = shadow_disc_angle();
	vec2 rotation = vec2(cos(angle), sin(angle));
	float spread = texel * radius;
	int count = (radius > 1.5) ? 8 : 4;
	float shadow = 0.0;
	for (int i = 0; i < count; i++)
	{
		vec2 offset = vec2(SHADOW_DISC[i].x * rotation.x - SHADOW_DISC[i].y * rotation.y, SHADOW_DISC[i].x * rotation.y + SHADOW_DISC[i].y * rotation.x) * spread;
		shadow += local_light_shadow_tap(position + side * offset.x + up * offset.y, layer, light.headlight_weights.y, bias, lamps);
	}
	return shadow / float(count);
}

// light reaching a surface from a sphere of uniform brightness (a lamp with a radius, as a sphere light of blender), in the units
// in which a point gives n.l / d^2: the same while the sphere is whole above the horizon of the surface, the part above it at
// grazing angles, and the light from all around within the sphere. Snyder, "Area light sources for real-time graphics", 1996, in
// the form of Lagarde, de Rousiers, "Moving Frostbite to physically based rendering", 2014
float sphere_light_irradiance(float NdotL, float distance2, float radius)
{
	float radius2 = radius * radius;
	if (distance2 <= radius2)
		return 1.0 / radius2;
	float sin_sigma2 = radius2 / distance2;
	if (NdotL * NdotL > sin_sigma2)
		return max(NdotL, 0.0) / distance2;
	float sin_theta = sqrt(max(1.0 - NdotL * NdotL, 1e-8));
	float x = sqrt(max(1.0 / sin_sigma2 - 1.0, 1e-8));
	float y = clamp(-x * NdotL / sin_theta, -1.0, 1.0);
	float sin_theta_sqrt_y = sin_theta * sqrt(max(1.0 - y * y, 0.0));
	float form = (NdotL * acos(y) - x * sin_theta_sqrt_y) * sin_sigma2 + atan(sin_theta_sqrt_y / x);
	return max(form, 0.0) / (BRDF_PI * radius2);
}

// the point of the sphere of a lamp (Radius, around the end of Tolight, view space) closest to the reflected ray of the view, where
// its highlight is, and in Energy the share of the light of the highlight widened by the sphere which the highlight of a point
// takes. Karis, "Real shading in Unreal Engine 4", 2013
vec3 sphere_light_specular(vec3 tolight, vec3 fragnormal, float radius, float roughness, out float energy)
{
	vec3 reflected = reflect(normalize(f_pos.xyz), fragnormal);
	vec3 toray = dot(tolight, reflected) * reflected - tolight;
	vec3 closest = tolight + toray * clamp(radius * inversesqrt(max(dot(toray, toray), 1e-8)), 0.0, 1.0);
	float alpha = roughness * roughness;
	float widened = clamp(alpha + 0.5 * radius * inversesqrt(max(dot(tolight, tolight), 1e-8)), 1e-4, 1.0);
	energy = (alpha / widened) * (alpha / widened);
	return closest;
}

// light of a lamp (point or spot light: the lamps of models and vehicles, the cab lamp) reaching the fragment, x the diffuse
// part and y the light the specular reflection takes (specular_light() brings its own n.l). it falls off with the square of the
// distance (lamp_falloff(), a lamp with a radius shines from its sphere: sphere_light_irradiance()), and smoothly to nothing at the
// range of the light, 1 / sqrt(quadratic), past which the culling of the clusters does not take it. a spot light fades from its
// inner cone to the outer one. a lamp with a shadow map is held back by what is between it and the fragment
vec2 calc_local_light(light_s light, vec3 fragnormal)
{
	vec3 tolight = light.pos - f_pos.xyz;
	float distance2 = dot(tolight, tolight);
	vec3 light_dir = tolight * inversesqrt(max(distance2, 1e-8));
	float edge = clamp(1.0 - distance2 * distance2 * (light.quadratic * light.quadratic), 0.0, 1.0);
	float atten = edge * edge;
	if (light.type == LIGHT_SPOT)
		atten *= smoothstep(light.out_cutoff, light.in_cutoff, dot(-light_dir, normalize(light.dir)));
	float falloff = lamp_falloff(light, distance2);
	float NdotL = dot(fragnormal, light_dir);
	float diffuse = (light.ambient > 0.0) ? sphere_light_irradiance(NdotL, distance2, light.ambient) : max(NdotL, 0.0) * falloff;
	if ((atten > 0.0) && (light.headlight_weights.x >= 0.0) && (diffuse > 0.0))
		atten *= 1.0 - local_light_shadow(light, fragnormal);
	if ((atten > 0.0) && (contact_shadow_budget > 0) && lamp_casts_shadows(light))
		atten *= 1.0 - contact_shadow_lamp(light_dir, sqrt(distance2),
		                                   atten * diffuse * light.intensity * max(light.color.r, max(light.color.g, light.color.b)));
	if (atten <= 0.0)
		return vec2(0.0);
	return vec2(diffuse, falloff) * atten;
}

// index of the light cluster the fragment is in, see gl/cluster_grid.h: its screen tile and slice of depth. the depth is the clip
// space w, which is 1 / gl_FragCoord.w. a tile is kept in the grid, as the far edge of the render target and rounding
// right below it give one past the last
uint fragment_cluster()
{
	ivec2 tile = ivec2(gl_FragCoord.xy * (vec2(CLUSTER_GRID_X, CLUSTER_GRID_Y) / cluster_viewport_size));
	tile = clamp(tile, ivec2(0), ivec2(CLUSTER_GRID_X - 1, CLUSTER_GRID_Y - 1));
	int slice = clamp(int(floor(cluster_slice_bias - log(gl_FragCoord.w) * cluster_slice_scale)), 0, CLUSTER_GRID_Z - 1);
	return uint(tile.x + CLUSTER_GRID_X * (tile.y + CLUSTER_GRID_Y * slice));
}

// adds the light of a headlight or lamp to the diffuse and the specular light of the fragment
void add_local_light(light_s light, vec2 part, vec3 fragnormal, inout vec3 diffuse, inout vec3 specular)
{
	if (part.y <= 0.0)
		return;
	vec3 color = light.color * light.intensity;
	diffuse += color * part.x;
	if (matte)
		return;
	// the highlight of a lamp with a radius is that of its sphere
	vec3 tolight = light.pos - f_pos.xyz;
	float energy = 1.0;
	if ((light.type != LIGHT_HEADLIGHTS) && (light.ambient != 0.0))
		tolight = sphere_light_specular(tolight, fragnormal, abs(light.ambient), spec_roughness, energy);
	specular += color * (part.y * param[1].y * energy) * specular_light(normalize(tolight), fragnormal, spec_f0, spec_roughness, LOCAL_LIGHT_BRDF_LIMIT);
}

// adds the light of the headlights and lamps which reach the fragment to its diffuse and specular light. these are the ones in
// the list of its cluster when the lights are clustered (the sun is not on the list, the callers take care of it), and the
// first lights_count - 1 of lights[] otherwise, which are headlights only. the specular light is reflected with spec_f0 and
// spec_roughness, and is already coloured by the fresnel term
void add_local_lights(vec3 fragnormal, inout vec3 diffuse, inout vec3 specular)
{
	// worked out before the loops, which the fragments of a quad do not leave together
	vec3 dpdx = dFdx(f_pos.xyz);
	vec3 dpdy = dFdy(f_pos.xyz);
	contact_shadow_budget = (contact_shadow_params.x > 0.0) ? int(contact_shadow_lamps.x) : 0;

	if (lights_clustered != 0U)
	{
		cluster_range_s range = cluster_ranges[fragment_cluster()];
		for (uint i = 0U; i < range.count; i++)
		{
			light_s light = lights[cluster_light_indices[range.offset + i]];
			vec2 part = (light.type == LIGHT_HEADLIGHTS) ? calc_headlights(light, fragnormal, dpdx, dpdy) : calc_local_light(light, fragnormal);
			add_local_light(light, part, fragnormal, diffuse, specular);
		}
	}
	else
	{
		for (uint i = 1U; i < lights_count; i++)
		{
			light_s light = lights[i];
			add_local_light(light, calc_headlights(light, fragnormal, dpdx, dpdy), fragnormal, diffuse, specular);
		}
	}
}

// the light of the headlights and lamps added to Fragcolor, diffuse and specular together (for shaders which don't tell
// the albedo from the reflection)
vec3 add_headlights(vec3 fragcolor, vec3 fragnormal)
{
	vec3 diffuse = vec3(0.0);
	vec3 specular = vec3(0.0);
	add_local_lights(fragnormal, diffuse, specular);
	return fragcolor + diffuse + specular;
}

// -----------------------------------------------------------------------
// Split-sum environment BRDF (Karis / UE4 analytic approximation).
//
// This is the missing piece that made matte specgloss surfaces "shine like
// crazy": previously the env reflection was added at full strength
// (envcolor * fresnel * reflectivity) and roughness only blurred the mip,
// never dimmed the energy. A rough surface therefore mirrored the bright
// sky just as strongly as a polished one.
//
// EnvBRDFApprox returns the pre-integrated specular scale (the "DFG" term)
// for a given F0, roughness and view angle. For rough surfaces it collapses
// toward ~0, so low-glossiness materials reflect almost nothing — matching
// what Substance 3D Painter shows. Polished surfaces keep their full
// reflection, and the grazing-angle Fresnel edge is preserved.
//   roughness 1.0 (matte)  -> scale ~0.015  (virtually no reflection)
//   roughness 0.0 (mirror) -> scale ~F0..1  (full reflection + Fresnel rim)
vec3 EnvBRDFApprox(vec3 F0, float roughness, float NoV)
{
    const vec4 c0 = vec4(-1.0, -0.0275, -0.572,  0.022);
    const vec4 c1 = vec4( 1.0,  0.0425,  1.040, -0.040);
    vec4 r = roughness * c0 + c1;
    float a004 = min(r.x * r.x, exp2(-9.28 * NoV)) * r.x + r.y;
    vec2 AB = vec2(-1.04, 1.04) * a004 + r.zw;
    return F0 * AB.x + AB.y;
}

// Diffuse colour of the submodel (t3d/e3d) or shape, used as the albedo multiplier:
// untextured materials take it as their colour, textured ones multiply the base texture by it
// (same as the legacy renderer's GL_MODULATE). param[0].rgb holds the authored sRGB colour,
// param[1].x its intensity (HSV value unless a material overrides it).
vec3 material_diffuse()
{
    vec3 diffusecolor = pow(max(param[0].rgb, vec3(0.0)), vec3(2.2));
    float value = max(param[0].r, max(param[0].g, param[0].b));
    return (value > 0.0 ? diffusecolor * (param[1].x / value) : vec3(0.0));
}

// directional part of the sky ambient light (sky_ssbo, bands 1 and 2 of the spherical harmonics) for a world space direction
vec3 sky_ambient_directional(vec3 n)
{
    return sky_ambient_sh[1].rgb * (0.488603 * n.y)
         + sky_ambient_sh[2].rgb * (0.488603 * n.z)
         + sky_ambient_sh[3].rgb * (0.488603 * n.x)
         + sky_ambient_sh[4].rgb * (1.092548 * n.x * n.y)
         + sky_ambient_sh[5].rgb * (1.092548 * n.y * n.z)
         + sky_ambient_sh[6].rgb * (0.315392 * (3.0 * n.z * n.z - 1.0))
         + sky_ambient_sh[7].rgb * (1.092548 * n.x * n.z)
         + sky_ambient_sh[8].rgb * (0.546274 * (n.x * n.x - n.y * n.y));
}

// adds the directional part of the sky light to the ambient light the material starts from (the uniform ambient, or a share
// of it, like the water takes). the same share of the directional part is added, so with the directional part at zero
// (gfx.ambient.sh off) the result is what it always was
vec3 add_sky_ambient(vec3 basecolor, vec3 fragnormal)
{
    if (sky_ambient_factor == 0.0)
        return basecolor;
    float share = dot(basecolor, vec3(1.0)) / max(dot(ambient, vec3(1.0)), 1e-4);
    vec3 worldnormal = normalize(mat3(inv_view) * fragnormal);
    return max(basecolor + sky_ambient_directional(worldnormal) * (sky_ambient_factor * share), vec3(0.0));
}

// the light of the ground below the horizon of the environment map, which holds the sky alone: the ground of the albedo
// env_params.x lit by the sky and the sun, as a surface facing up
vec3 reflection_ground()
{
    vec3 light = ambient;
    if (sky_ambient_factor != 0.0)
        light = max(light + sky_ambient_directional(vec3(0.0, 1.0, 0.0)) * sky_ambient_factor, vec3(0.0));
    light *= AMBIENT_SCALE;
    if (lights_count > 0U)
    {
        vec3 sun = -normalize(mat3(inv_view) * lights[0].dir);
        light += lights[0].color * (SUN_DIFFUSE_SCALE * lights[0].intensity * max(sun.y, 0.0));
    }
    return light * env_params.x;
}

// the reflection of the surroundings the scene doesn't give (the screen space reflections, the ray tracing): the environment
// map above the horizon (the sky and the clouds), the ground below it
vec3 reflection_surroundings(vec3 fragnormal, float roughness)
{
    vec3 direction = normalize(envmap_direction(fragnormal));
    vec3 sky = envmap_sample_rough(normalize(vec3(direction.x, max(direction.y, 0.0), direction.z)), roughness);
    float below = smoothstep(0.0, 0.25, -direction.y);
    if ((below <= 0.0) || (false == ENVMAP_ENABLED))
        return sky;
    return mix(sky, reflection_ground(), below);
}

// the least weight of the reflection of the surroundings (its specular colour times its share) a surface traces the scene for
const float REFLECTION_SCENE_WEIGHT = 0.004;

// the reflection of the scene in a surface of the given perceptual roughness: the colour times how much of the reflection the
// scene gives (rgb), and that share (a), the rest is left to reflection_surroundings()
vec4 reflection_scene(vec3 fragnormal, float roughness)
{
    vec3 direction = normalize(reflect(f_pos.xyz, fragnormal));
    vec4 result = ssr_trace(f_pos.xyz, normalize(f_normal), direction, roughness);
#ifdef REFLECTION_RAYS_ACTIVE
    // what the screen doesn't hold is traced through the scenery (shaders/vk14/rt_reflections.glsl)
    if ((result.a < 0.99) && (ssr_params2.z > 0.0) && (roughness <= ssr_params.z))
    {
        vec4 traced = rt_trace(f_pos.xyz, normalize(f_normal), direction, roughness);
        result.rgb += traced.rgb * (1.0 - result.a);
        result.a += traced.a * (1.0 - result.a);
    }
#endif
    return result;
}

// the ambient occlusion of the view in the ambient light of the fragment, 0..1 (1: none): the occlusion of the main view worked out
// from the depth of the opaque scene drawn before its colour (gfx.postfx.ao.lighting), half its size. the four nearest texels are
// weighed by how close they are and by how close their distance is to the one of the fragment (as in postfx_ssao_apply.frag); where
// none of them is of the surface of the fragment (what the depth before the colour doesn't hold: the blended surfaces, the grass,
// the cab drawn after the precipitation, the edges of objects) there's no occlusion. the distance is the one along the axis of the
// view, near / depth with the far plane at infinity
float ambient_occlusion()
{
    if (ao_params.x <= 0.0)
        return 1.0;
    ivec2 full_size = textureSize(scene_depth, 0);
    ivec2 half_size = textureSize(scene_ao, 0);
    ivec2 pixel = ivec2(gl_FragCoord.xy);
    float distance = max(-f_pos.z, 1e-3);
    vec2 position = vec2(pixel) * 0.5;
    ivec2 base = ivec2(floor(position));
    vec2 fraction = position - vec2(base);
    float sum = 0.0;
    float weights = 0.0;
    float closest = 1e30;
    for (int y = 0; y < 2; ++y)
    {
        for (int x = 0; x < 2; ++x)
        {
            ivec2 texel = clamp(base + ivec2(x, y), ivec2(0), half_size - 1);
            float depth = texelFetch(scene_depth, min(texel * 2, full_size - 1), 0).r;
            float difference = (depth > 0.0) ? abs(projection[3][2] / depth - distance) / distance : 1e30;
            closest = min(closest, difference);
            float bilinear = (x == 0 ? 1.0 - fraction.x : fraction.x) * (y == 0 ? 1.0 - fraction.y : fraction.y);
            float weight = (bilinear + 1e-3) / (1e-3 + difference);
            sum += texelFetch(scene_ao, texel, 0).r * weight;
            weights += weight;
        }
    }
    if (closest > AO_SURFACE_MATCH)
        return 1.0;
    return pow(clamp(sum / max(weights, 1e-6), 0.0, 1.0), ao_params.y);
}

// the light which comes back after more than one bounce off surfaces of the given albedo, for the occlusion of the ambient light
// (Jimenez et al. 2016), for each channel
vec3 ao_multi_bounce(float visibility, vec3 albedo)
{
    vec3 a = 2.0404 * albedo - 0.3324;
    vec3 b = -4.7951 * albedo + 0.6417;
    vec3 c = 2.7552 * albedo + 0.6903;
    return max(vec3(visibility), ((visibility * a + b) * visibility + c) * visibility);
}

// the share of the reflection of the surroundings the occlusion leaves to a surface of the perceptual roughness seen at the angle
// NdotV (Lagarde and de Rousiers 2014)
float ao_specular(float visibility, float NdotV, float roughness)
{
    return clamp(pow(NdotV + visibility, exp2(-16.0 * roughness - 1.0)) - 1.0 + visibility, 0.0, 1.0);
}

// whether the fragment is a part of the occupied cab with its probe (gl::scene_ubs::cab_probe_box, gl::model_ubs::cab_interior)
bool cab_probe_active()
{
    return (cab_probe_box.w > 0.0) && (cab_interior > 0.5) && (cab_interior < 1.5);
}

// the light of the lamps the cab reflects which reaches a surface of the cab facing the view space Fragnormal, in the units of the
// ambient light: the spherical harmonics cabprobe_sh.comp worked out from the probe, in the space of the probe
vec3 cab_probe_bounce(vec3 fragnormal)
{
    vec3 n = normalize(mat3(cab_probe_view) * fragnormal);
    vec3 light = cab_probe_sh[0].rgb * 0.282095
               + cab_probe_sh[1].rgb * (0.488603 * n.y)
               + cab_probe_sh[2].rgb * (0.488603 * n.z)
               + cab_probe_sh[3].rgb * (0.488603 * n.x)
               + cab_probe_sh[4].rgb * (1.092548 * n.x * n.y)
               + cab_probe_sh[5].rgb * (1.092548 * n.y * n.z)
               + cab_probe_sh[6].rgb * (0.315392 * (3.0 * n.z * n.z - 1.0))
               + cab_probe_sh[7].rgb * (1.092548 * n.x * n.z)
               + cab_probe_sh[8].rgb * (0.546274 * (n.x * n.x - n.y * n.y));
    return max(light, vec3(0.0)) * cab_probe_params.x;
}

// the reflection of the cab in a surface of the cab of the perceptual roughness, over the reflection of the surroundings, where the
// probe holds the cab: the reflected ray from the fragment to the box of the cab, looked up from the middle of the box, where the
// probe was drawn from (the box stands for the walls of the cab). the cab shows the light of its lamps it reflects (the probe) and
// its albedo lit by the ambient light of the fragment (Ambientlight), the light the cab gets from the outside
vec3 cab_probe_reflection(vec3 surroundings, vec3 fragnormal, float roughness, vec3 ambientlight)
{
    vec3 position = (cab_probe_view * vec4(f_pos.xyz, 1.0)).xyz;
    vec3 direction = normalize(mat3(cab_probe_view) * reflect(normalize(f_pos.xyz), fragnormal));
    vec3 invdir = (step(0.0, direction) * 2.0 - 1.0) / max(abs(direction), vec3(1e-5));
    vec3 far = max((cab_probe_box.xyz - position) * invdir, (-cab_probe_box.xyz - position) * invdir);
    float reach = max(min(min(far.x, far.y), far.z), 0.0);
    vec3 lookup = position + direction * reach;
    float level = roughness * cab_probe_params.z;
    vec4 light = textureLod(cab_probe_light, lookup, level);
    vec4 albedo = textureLod(cab_probe_albedo, lookup, level);
    return surroundings * (1.0 - albedo.a) + light.rgb + albedo.rgb * ambientlight;
}

// lights the surface: the ambient light, the sun (the moon), the headlights and lamps and the reflection of the surroundings.
// the units are the ones of the ambient light, the irradiance over pi: a white lambertian surface lit by 1 of it shows 1.
// Texturecolor is the albedo (the base colour of a metal), Specularity the specular level of a dielectric (0.5 is the
// reflectance of 4% at normal incidence), Reflectivity how much of the reflected surroundings is the cubemap rather than the
// ambient light, Shadowtone the share of the sun left in its shadow (1 for the materials which take no shadows).
// a Sunless surface (the interior of a cab) gets no light of the sun, where the sun falls it brightens the ambient light
vec3 shade_surface(vec3 fragcolor, vec3 fragnormal, vec3 texturecolor, float reflectivity, float specularity, float shadowtone, bool sunless)
{
    // Diffuse colour is applied once, to the albedo; the light terms below are not scaled by it again
    texturecolor *= material_diffuse();
    float polished = 0.0;
    if (sunless)
    {
        // Cab interior: dim ambient less than the exterior path - ambient
        // is the dominant indoor illumination, but it was still too bright.
        fragcolor *= 0.80;
    }
    else
    {
        // the sky lights the faces turned up and towards the bright part of the sky more than the ones turned to the ground
        fragcolor = add_sky_ambient(fragcolor, fragnormal);
#ifndef NO_WEATHERING
        float weathered = (param[2].w > 0.0 ? clamp(param[2].w * UserData.x, 0.0, 1.0) : 0.0);
        float tinted = (param[2].w < 0.0 ? clamp(-param[2].w * UserData.x, 0.0, 1.0) : 0.0);
        // a tint only scales the colour of the texture, the steel keeps its own polish
        polished = weathered * smoothstep(0.03, 0.12, dot(texturecolor, vec3(0.2126, 0.7152, 0.0722)));
        float weatheredluma = min(dot(texturecolor, vec3(0.2126, 0.7152, 0.0722)), 0.05);
        vec3 unused = texturecolor * (weatheredluma / max(dot(texturecolor, vec3(0.2126, 0.7152, 0.0722)), 1e-4));
        unused = mix(unused, vec3(1.0, 0.82, 0.74) * (weatheredluma / 0.8525), 0.5);
        texturecolor = mix(texturecolor, unused, smoothstep(0.0, 0.4, weathered));
        texturecolor = (tinted > 0.0 ? texturecolor * mix(vec3(1.0), param[2].rgb, tinted) : texturecolor);
        specularity = mix(specularity, min(specularity, 0.03), smoothstep(0.0, 0.4, polished));
        metalic = mix(metalic, 0.0, polished);
        glossiness = mix(glossiness, min(glossiness, 0.06 * abs(param[1].w)), polished);
        reflectivity = mix(reflectivity, min(reflectivity, 0.08 * param[1].z), smoothstep(0.0, 0.4, polished));
#endif
        // Scale ambient before it gets tinted by the texture.
        // Sun, headlights and emission are added afterwards so they are NOT
        // attenuated by AMBIENT_SCALE - this only dims the indirect term.
        fragcolor *= AMBIENT_SCALE;
    }
    // the scenery around holds back the sky (x) and the key light (y): tunnels, the ground under bridges and roofs, cuttings. not
    // the light of the lamps and the headlights, nor of the cab lamp bounced around the cab, which come in below
    vec2 skyoccl = sky_occlusion_world(mat3(inv_view) * f_pos.xyz, normalize(mat3(inv_view) * fragnormal));
    fragcolor *= skyoccl.x;

    vec3 emissioncolor = vec3(emission);
    vec3 view_dir = normalize(-f_pos.xyz);
    float NdotV = max(dot(fragnormal, view_dir), 1e-4);

    // glossiness == param[1].w -> mirror, glossiness == 0 -> fully rough
    float roughness = clamp(1.0 - glossiness / max(abs(param[1].w), 1.0), 0.0, 1.0);
    roughness = max(specular_antialiasing(fragnormal, roughness), MIN_ROUGHNESS);
    // metals reflect with their base colour, dielectrics with the specular level (0.5 -> 0.04)
    vec3 f0 = mix(vec3(0.08 * specularity), texturecolor, metalic);
    spec_f0 = f0;
    spec_roughness = roughness;

    // the light of the lamps the cab reflects, which lights the cab again (the probe of the cab), joins the ambient light
    bool cabprobe = cab_probe_active();
    vec3 cabambient = fragcolor;
    if (cabprobe)
        fragcolor += cab_probe_bounce(fragnormal);
    // the ambient occlusion of the view holds back the ambient light of the diffuse layer (with the light coming back from more
    // bounces off surfaces of the albedo) and the reflection of the surroundings; not the light of the sun and the lamps, nor the
    // reflection of the scene, which the rays find as it is
    float occlusion = ambient_occlusion();
    // in the cab and the low poly interiors walls and things are near all around, and the occlusion of the view darkens the ambient
    // light a lot; they take a share of it (gfx.postfx.ao.cab), the light which bounces inside them comes from the probe or below
    if (cab_interior > 0.5)
        occlusion = mix(1.0, occlusion, ao_params.z);
    vec3 diffuse_occlusion = vec3(1.0);
    float specular_occlusion = 1.0;
    if (occlusion < 1.0)
    {
        diffuse_occlusion = ao_multi_bounce(occlusion, texturecolor);
        specular_occlusion = ao_specular(occlusion, NdotV, roughness);
    }

    // Pre-integrated env BRDF: how much of the surroundings the surface reflects, for its F0, roughness and the view angle.
    // the surroundings are the cubemap where the material asks for it (reflectivity) and the ambient light elsewhere, so a
    // surface reflects them always, the rough one barely
    vec3 env_spec = EnvBRDFApprox(f0, roughness, NdotV);
    float env_roughness = roughness;
    vec3 cubemap_spec = env_spec;
    float cubemap_share = max(reflectivity, 0.0);
    float ambient_share = clamp(1.0 - reflectivity, 0.0, 1.0);
    if (matte)
    {
        env_spec = vec3(0.0);
        cubemap_share = 0.0;
    }
    // what the diffuse layer is left with
    vec3 diffuse_albedo = texturecolor * (1.0 - metalic) * (1.0 - env_spec);
    if (legacy_coat)
    {
        // the varnish: a smooth dielectric layer reflecting the cubemap, worn off where the material is weathered
        env_roughness = specular_antialiasing(fragnormal, roughness * smoothstep(0.0, 0.4, polished));
        cubemap_spec = EnvBRDFApprox(mix(vec3(0.04), texturecolor, metalic), env_roughness, NdotV);
        ambient_share = 1.0;
        diffuse_albedo *= 1.0 - cubemap_spec * min(cubemap_share, 1.0);
    }
    // the reflection of the surroundings: the scene where the screen (or the ray tracing) gives it, the sky and the ground of the
    // environment map elsewhere
    vec3 envcolor = vec3(0.0);
    vec4 scenecolor = vec4(0.0);
    if (!matte)
    {
        // the sky the surface would reflect is held back as the sky which lights it
        envcolor = reflection_surroundings(fragnormal, env_roughness) * skyoccl.x;
        // inside the cab the cab is what the surroundings are, where the probe holds it
        if (cabprobe && (cab_probe_params.y > 0.0))
            envcolor = cab_probe_reflection(envcolor, fragnormal, env_roughness, cabambient);
        if (((ssr_params.x > 0.0) || (ssr_params2.z > 0.0)) && (cubemap_share * dot(cubemap_spec, vec3(1.0 / 3.0)) > REFLECTION_SCENE_WEIGHT))
            scenecolor = reflection_scene(fragnormal, env_roughness);
    }

    if (lights_count == 0U)
        return fragcolor * diffuse_occlusion * diffuse_albedo
             + emissioncolor * texturecolor
             + fragcolor * env_spec * ambient_share * specular_occlusion
             + (scenecolor.rgb + envcolor * ((1.0 - scenecolor.a) * specular_occlusion)) * cubemap_spec * cubemap_share;

    vec3 light_dir = normalize(-lights[0].dir);
    float NdotL = max(dot(fragnormal, light_dir), 0.0);
    // Sharpen sun N.L falloff so the lit-to-shaded terminator on cab
    // panels, vehicle bodies and terrain reads as a clear edge rather
    // than a soft Lambertian ramp. Tunable via SUN_NDOTL_SHARPNESS.
    float sun_NdotL = pow(NdotL, SUN_NDOTL_SHARPNESS);
    // the clouds hold back the sun (the moon at night) where their shadow falls
    float cloudsun = cloud_shadow(mat3(inv_view) * f_pos.xyz);
    // Sun HDR scale -> SUN_DIFFUSE_SCALE, the same for the diffuse and the specular light
    cloudsun *= skyoccl.y;
    vec3 sunlight = lights[0].color * (SUN_DIFFUSE_SCALE * lights[0].intensity * cloudsun);

    float shadow = 0.0;
    if (shadowtone < 1.0)
        shadow = clamp(calc_shadow(), 0.0, 1.0);

    // the environment map knows nothing of what shades the fragment: it is dimmed a little in the shadow of the sun (the scene the
    // reflection finds is lit as it is)
    float cubemap_shadow = 1.0;
    if (sunless)
    {
        // the sun adds no light of its own in the cab; where it falls the ambient light is raised as much as the shadow used to lower
        // it (the share Shadowtone, at most 4 times), so the shadows of the walls of the cab and of the clouds leave the ambient light
        // as it is instead of darkening the cab
        float diffuseamount = sun_NdotL * lights[0].intensity;
        float sunlit = (shadowtone < 1.0 ? 1.0 - max(shadow, 1.0 - cloudsun) : 0.0);
        fragcolor *= 1.0 + clamp(diffuseamount * sunlit, 0.0, 1.0) * (1.0 - shadowtone) / max(shadowtone, 0.25);
    }
    else
        cubemap_shadow = 1.0 - (1.0 - shadowtone) * shadow * 0.5;

    // the reflection of the surroundings, before the light of the sun and the lamps joins the ambient light
    vec3 env_reflection = fragcolor * env_spec * ambient_share * specular_occlusion
                        + (scenecolor.rgb + envcolor * ((1.0 - scenecolor.a) * cubemap_shadow * specular_occlusion)) * cubemap_spec * cubemap_share;
    // the sun coming in through the windows and bouncing around a low poly interior, or the cab without its probe (which gives that
    // light from where the sun falls in the cab): a share of the light of the sun outside joins the ambient light (gfx.interior.bounce)
    if ((cab_interior > 0.5) && (false == cabprobe))
        fragcolor += sunlight * env_params.z;
    // the occlusion of the ambient light, before the light of the sun and the lamps joins it
    fragcolor *= diffuse_occlusion;

    // the shadow leaves the share of the sun Shadowtone tells (the old shadow colour of the scenery) to the diffuse light,
    // nothing to the highlight
    if (!sunless)
        fragcolor += sunlight * (sun_NdotL * (1.0 - (1.0 - shadowtone) * shadow));
    vec3 specular = matte ? vec3(0.0) : sunlight * specular_light(light_dir, fragnormal, f0, roughness, SUN_BRDF_LIMIT) * (1.0 - shadow);

    vec3 lampspecular = vec3(0.0);
    add_local_lights(fragnormal, fragcolor, lampspecular);

    return fragcolor * diffuse_albedo     // ambient, sun and lamps on the diffuse layer
         + specular * param[1].y        // highlight of the sun, fresnel coloured (metals take their base colour)
         + lampspecular                 // highlights of the lamps, param[1].y already applied
         + emissioncolor * texturecolor
         + env_reflection;
}

vec3 apply_lights(vec3 fragcolor, vec3 fragnormal, vec3 texturecolor, float reflectivity, float specularity, float shadowtone)
{
    return shade_surface(fragcolor, fragnormal, texturecolor, reflectivity, specularity, shadowtone, false);
}

// the interior of a cab: no light of the sun, where it falls it brightens the ambient light
vec3 apply_lights_sunless(vec3 fragcolor, vec3 fragnormal, vec3 texturecolor, float reflectivity, float specularity, float shadowtone)
{
    return shade_surface(fragcolor, fragnormal, texturecolor, reflectivity, specularity, shadowtone, true);
}
