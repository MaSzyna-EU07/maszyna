#if SHADOWMAP_ENABLED
uniform sampler2DArrayShadow shadowmap;
#endif
uniform sampler2D headlightmap;
// cube shadow maps of the lamps, see gl::LOCAL_SHADOW_TEX and opengl46_local_shadows
layout(binding = LOCAL_SHADOW_TEX) uniform samplerCubeArrayShadow local_shadowmap;
// the same for the lamps of scenery models, the lights with linear 1
layout(binding = LOCAL_SHADOW_LAMPS_TEX) uniform samplerCubeArrayShadow local_shadowmap_lamps;
// shadow maps of the headlights of vehicles, see gl::HEADLIGHT_SHADOW_TEX and gl::scene_ubs::headlight_shadow
layout(binding = HEADLIGHT_SHADOW_TEX) uniform sampler2DArrayShadow headlight_shadowmap;

#include <envmapping.glsl>
#include <conversion.glsl>
#include <weather_common.glsl>

float glossiness = 1.0;
float metalic = 0.0;

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

float calc_shadow()
{
#if SHADOWMAP_ENABLED
	float distance = dot(f_pos.xyz, f_pos.xyz);
	uint cascade;
	for (cascade = 0U; cascade < MAX_CASCADES; cascade++)
		if (distance <= cascade_end[cascade])
			break;
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
#else
	return 0.0;
#endif
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

// Height-correlated Smith geometry term
//   G(N,V,L) = G_SchlickGGX(NdotV) · G_SchlickGGX(NdotL)
float G_Smith(float NdotV, float NdotL, float roughness)
{
    return G_SchlickGGX(NdotV, roughness) * G_SchlickGGX(NdotL, roughness);
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
	return part * atten * lightintensity;
}

// closest distance (squared) the light of a lamp is computed for: nearer the inverse square would grow without a limit
const float LOCAL_LIGHT_MIN_DISTANCE2 = 0.01;

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

// light of a lamp (point or spot light: the lamps of models and vehicles, the cab lamp) reaching the fragment, x the diffuse
// and y the specular part. it falls off with the square of the distance, and smoothly to nothing at the range of the light,
// 1 / sqrt(quadratic), past which the culling of the clusters does not take it. a spot light fades from its inner cone to the
// outer one. a lamp with a shadow map is held back by what is between it and the fragment
vec2 calc_local_light(light_s light, vec3 fragnormal)
{
	vec3 tolight = light.pos - f_pos.xyz;
	float distance2 = dot(tolight, tolight);
	vec3 light_dir = tolight * inversesqrt(max(distance2, 1e-8));
	float edge = clamp(1.0 - distance2 * distance2 * (light.quadratic * light.quadratic), 0.0, 1.0);
	float atten = edge * edge / max(distance2, LOCAL_LIGHT_MIN_DISTANCE2);
	if (light.type == LIGHT_SPOT)
		atten *= smoothstep(light.out_cutoff, light.in_cutoff, dot(-light_dir, normalize(light.dir)));
	if ((atten > 0.0) && (light.headlight_weights.x >= 0.0) && (dot(fragnormal, light_dir) > 0.0))
		atten *= 1.0 - local_light_shadow(light, fragnormal);
	if (atten <= 0.0)
		return vec2(0.0);
	vec2 part = calc_light(light_dir, fragnormal);
	// the highlight of a tiny light on a polished surface is kept within what the image can take
	return vec2(part.x, min(part.y, 16.0) * part.x) * atten;
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

// adds the light of the headlights and lamps which reach the fragment to its colour. these are the ones in the list of its cluster
// when the lights are clustered (the sun is not on the list, the callers take care of it), and the first lights_count - 1 of
// lights[] otherwise, which are headlights only
vec3 add_headlights(vec3 fragcolor, vec3 fragnormal)
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
			fragcolor += light.color * (part.x + part.y * param[1].y) * light.intensity;
		}
	}
	else
	{
		for (uint i = 1U; i < lights_count; i++)
		{
			light_s light = lights[i];
			vec2 part = calc_headlights(light, fragnormal, dpdx, dpdy);
			fragcolor += light.color * (part.x + part.y * param[1].y) * light.intensity;
		}
	}
	return fragcolor;
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

// [0] - diffuse, [1] - specular
// do magic here
vec3 apply_lights(vec3 fragcolor, vec3 fragnormal, vec3 texturecolor, float reflectivity, float specularity, float shadowtone)
{
    // Diffuse colour is applied once, to the albedo; the light terms below are not scaled by it again
    texturecolor *= material_diffuse();
    // the sky lights the faces turned up and towards the bright part of the sky more than the ones turned to the ground
    fragcolor = add_sky_ambient(fragcolor, fragnormal);
    // Scale ambient before it gets tinted by the texture.
    // Sun, headlights and emission are added afterwards so they are NOT
    // attenuated by AMBIENT_SCALE - this only dims the indirect term.
    fragcolor *= AMBIENT_SCALE;

    vec3 emissioncolor = vec3(emission);

    vec3 view_dir = normalize(-f_pos.xyz);
    float NdotV = max(dot(fragnormal, view_dir), 0.0);
    vec3 F0 = mix(vec3(0.04), texturecolor, metalic);
    vec3 fresnel = F0 + (1.0 - F0) * pow(1.0 - NdotV, 5.0);

    float env_roughness = 1.0 - clamp(glossiness / max(abs(param[1].w), 1.0), 0.0, 1.0);
    vec3 envcolor = envmap_color_rough(fragnormal, env_roughness);

    // Pre-integrated env BRDF: roughness/F0/view-dependent specular scale.
    // Replaces the old raw `fresnel` weighting so matte surfaces stop
    // mirroring the sky. `env_spec` is the colour to multiply the cubemap by.
    vec3 env_spec = EnvBRDFApprox(F0, env_roughness, NdotV);
    float env_spec_w = max(env_spec.r, max(env_spec.g, env_spec.b));

    // Tint texture toward fully-saturated under strong env, weighted by the
    // env BRDF (so a rough/matte surface no longer gets washed toward env hue)
    vec3 texturecoloryuv = rgb2yuv(texturecolor);
    vec3 texturecolorfullv = yuv2rgb(vec3(0.2176, texturecoloryuv.gb));
    vec3 envyuv = rgb2yuv(envcolor);
    texturecolor = mix(texturecolor, texturecolorfullv, envyuv.r * reflectivity * env_spec_w);

    if (lights_count == 0U)
        // Metals carry no diffuse term; env reflection is gated by the env BRDF
        // (F0-tinted, roughness-attenuated) so matte surfaces barely reflect.
        return fragcolor * texturecolor * (1.0 - metalic)
             + emissioncolor * texturecolor
             + envcolor * env_spec * reflectivity;

    vec2 sunlight = calc_dir_light(lights[0], fragnormal);
    // Sharpen sun N.L falloff so the lit-to-shaded terminator on cab
    // panels, vehicle bodies and terrain reads as a clear edge rather
    // than a soft Lambertian ramp. Tunable via SUN_NDOTL_SHARPNESS.
    float sun_NdotL = pow(sunlight.x, SUN_NDOTL_SHARPNESS);
    // the clouds hold back the sun (the moon at night) where their shadow falls
    float cloudsun = cloud_shadow(mat3(inv_view) * f_pos.xyz);
    float diffuseamount = sun_NdotL * lights[0].intensity * cloudsun;

    float shadow1 = 0.0;
    if (shadowtone < 1.0)
        shadow1 = (1.0 - shadowtone) * clamp(calc_shadow(), 0.0, 1.0);

    // Sun HDR scale -> SUN_DIFFUSE_SCALE (default 2.5). Controls how
    // bright sun-lit (unshaded) faces get. Lower this if surfaces in
    // direct sun read as too hot/burnt; raise it for more punch.
    fragcolor += lights[0].color * SUN_DIFFUSE_SCALE * (1.0 - shadow1) * diffuseamount;

    fragcolor = add_headlights(fragcolor, fragnormal);

    float specularamount = sunlight.y * param[1].y * specularity * lights[0].intensity * cloudsun
                         * clamp(1.0 - shadowtone, 0.0, 1.0);
    if (shadowtone < 1.0)
        specularamount *= clamp(1.0 - shadow1, 0.0, 1.0);

    vec3 specularcolor = specularamount * lights[0].color;

    // Env reflection tracked separately — must NOT go through the albedo multiply
    // below. Gated by the pre-integrated env BRDF (env_spec) so reflection energy
    // falls off with roughness; F0 inside env_spec already tints metals by albedo.
    vec3 env_reflection = envcolor * env_spec * reflectivity * (1.0 - shadow1 * 0.5);

    // --- Physically-based metal/rough combine (Substance 3D Painter parity) ---
    // Dielectrics: keep the full diffuse albedo; the direct sun highlight stays
    //              light-coloured because dielectric F0 is achromatic (~0.04) and
    //              must NOT be tinted by the base colour.
    // Metals:      drop the diffuse term entirely and tint the direct highlight
    //              with the albedo (metal F0 == base colour).
    // The highlight *strength* (specularamount) is deliberately left untouched so
    // existing material tuning is preserved — only the colour/energy split that
    // was previously inverted gets corrected.
    vec3 diffuse_albedo = texturecolor * (1.0 - metalic);
    vec3 spec_tint      = mix(vec3(1.0), texturecolor, metalic);

    vec3 result = fragcolor      * diffuse_albedo   // sun + ambient + headlight diffuse
                + specularcolor  * spec_tint        // direct sun highlight
                + emissioncolor  * texturecolor     // emissive glow (albedo-tinted, unchanged)
                + env_reflection;                   // env reflection (env_spec already F0-tinted)

    return result;
}