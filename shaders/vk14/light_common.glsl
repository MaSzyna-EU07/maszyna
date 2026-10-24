// stage D of the vulkan renderer: the sun with its cascaded shadows, the environment map, the headlights and the lamps (in the
// clusters or not). the cube shadow maps of the lamps, the shadow maps of the headlights, the contact shadows, the shadows of
// the clouds, the directional sky ambient light and the prefiltered environment map of shaders/ogl46/light_common.glsl come
// with the later stages; the code below is the one of the opengl renderer with those left out, it shades as that one does
// with them off
layout(set = PASS_SET, binding = SHADOW_TEX) uniform sampler2DArrayShadow shadowmap;
layout(set = PASS_SET, binding = HEADLIGHT_TEX) uniform sampler2D headlightmap;

#include "envmapping.glsl"
#include "conversion.glsl"

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

// share of the sun (the moon) held back before the fragment by the shadow map, 0..1
float calc_shadow_map();
float calc_shadow()
{
	return calc_shadow_map();
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
	// x the diffuse light, y the light the specular reflection takes (specular_light() brings its own n.l)
	return vec2(part.x, 1.0) * (atten * lightintensity);
}

// closest distance (squared) the light of a lamp is computed for: nearer the inverse square would grow without a limit
const float LOCAL_LIGHT_MIN_DISTANCE2 = 0.01;

// light of a lamp (point or spot light: the lamps of models and vehicles, the cab lamp) reaching the fragment, x the diffuse
// part and y the light the specular reflection takes (specular_light() brings its own n.l). it falls off with the square of the distance, and smoothly to nothing at the range of the light,
// 1 / sqrt(quadratic), past which the culling of the clusters does not take it. a spot light fades from its inner cone to the
// outer one
vec2 calc_local_light(light_s light, vec3 fragnormal)
{
	vec3 tolight = light.pos - f_pos.xyz;
	float distance2 = dot(tolight, tolight);
	vec3 light_dir = tolight * inversesqrt(max(distance2, 1e-8));
	float edge = clamp(1.0 - distance2 * distance2 * (light.quadratic * light.quadratic), 0.0, 1.0);
	float atten = edge * edge / max(distance2, LOCAL_LIGHT_MIN_DISTANCE2);
	if (light.type == LIGHT_SPOT)
		atten *= smoothstep(light.out_cutoff, light.in_cutoff, dot(-light_dir, normalize(light.dir)));
	if (atten <= 0.0)
		return vec2(0.0);
	return vec2(max(dot(fragnormal, light_dir), 0.0), 1.0) * atten;
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
	specular += color * (part.y * param[1].y) * specular_light(normalize(light.pos - f_pos.xyz), fragnormal, spec_f0, spec_roughness, LOCAL_LIGHT_BRDF_LIMIT);
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

// lights the surface: the ambient light, the sun (the moon), the headlights and lamps and the reflection of the surroundings.
// the units are the ones of the ambient light, the irradiance over pi: a white lambertian surface lit by 1 of it shows 1.
// Texturecolor is the albedo (the base colour of a metal), Specularity the specular level of a dielectric (0.5 is the
// reflectance of 4% at normal incidence), Reflectivity how much of the reflected surroundings is the cubemap rather than the
// ambient light, Shadowtone the share of the sun left in its shadow (1 for the materials which take no shadows).
// a Sunless surface (the interior of a cab) gets no light of the sun, its shadow only darkens the ambient light
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
    vec3 envcolor = (matte ? vec3(0.0) : envmap_color_rough(fragnormal, env_roughness));

    if (lights_count == 0U)
        return fragcolor * diffuse_albedo
             + emissioncolor * texturecolor
             + fragcolor * env_spec * ambient_share
             + envcolor * cubemap_spec * cubemap_share;

    vec3 light_dir = normalize(-lights[0].dir);
    float NdotL = max(dot(fragnormal, light_dir), 0.0);
    // Sharpen sun N.L falloff so the lit-to-shaded terminator on cab
    // panels, vehicle bodies and terrain reads as a clear edge rather
    // than a soft Lambertian ramp. Tunable via SUN_NDOTL_SHARPNESS.
    float sun_NdotL = pow(NdotL, SUN_NDOTL_SHARPNESS);
    // the shadows of the clouds come with the weather (stage P2): nothing holds the sun back yet
    float cloudsun = 1.0;
    // Sun HDR scale -> SUN_DIFFUSE_SCALE, the same for the diffuse and the specular light
    vec3 sunlight = lights[0].color * (SUN_DIFFUSE_SCALE * lights[0].intensity * cloudsun);

    float shadow = 0.0;
    if (shadowtone < 1.0)
        shadow = clamp(calc_shadow(), 0.0, 1.0);

    // the cubemap knows nothing of what shades the fragment: it is dimmed a little in the shadow of the sun
    float cubemap_shadow = 1.0;
    if (sunless)
    {
        // the sun adds no light in the cab, where it would fall its shadow darkens the ambient light
        float diffuseamount = sun_NdotL * lights[0].intensity;
        float darkness = (shadowtone < 1.0 ? max(shadow, 1.0 - cloudsun) : 0.0);
        fragcolor = mix(fragcolor, fragcolor * shadowtone, clamp(diffuseamount * darkness, 0.0, 1.0));
    }
    else
        cubemap_shadow = 1.0 - (1.0 - shadowtone) * shadow * 0.5;

    // the reflection of the surroundings, before the light of the sun and the lamps joins the ambient light
    vec3 env_reflection = fragcolor * env_spec * ambient_share
                        + envcolor * cubemap_spec * (cubemap_share * cubemap_shadow);

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

// the interior of a cab: no light of the sun, its shadow darkens the ambient light
vec3 apply_lights_sunless(vec3 fragcolor, vec3 fragnormal, vec3 texturecolor, float reflectivity, float specularity, float shadowtone)
{
    return shade_surface(fragcolor, fragnormal, texturecolor, reflectivity, specularity, shadowtone, true);
}
