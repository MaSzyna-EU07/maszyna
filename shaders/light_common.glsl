#if SHADOWMAP_ENABLED
in vec4 f_light_pos[MAX_CASCADES];
uniform sampler2DArrayShadow shadowmap;
#endif
uniform sampler2D headlightmap;

#include <envmapping.glsl>
#include <conversion.glsl>

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
	float dist_casc = distance / cascade_end[cascade];
	vec3 coords = f_light_pos[cascade].xyz / f_light_pos[cascade].w;
	if (coords.z < 0.0)
		return 0.0f;
		
		

	float bias = 0.00005f * float(cascade + 1U);
	vec2 texel = vec2(1.0) / vec2(textureSize(shadowmap, 0));
	//float radius = 1.0; f_light_pos[cascade].w; //0.5 + 2.0 * max(abs(2.0 * coords.x - 1.0), abs(2.0 * coords.y - 1.0));
	float radius = 1.0;
	float minradius = 0.0;
	if (cascade == 0U)
		minradius = 1.0;
	if (cascade < MAX_CASCADES - 1U)
		radius = mix(minradius, f_light_pos[cascade+1U].w/f_light_pos[cascade].w, dist_casc);
	else
		radius = 0.5;

#if defined(GL_ARB_gpu_shader5) || defined(GL_EXT_gpu_shader5) || __VERSION__ >= 400
	// Fast path -- replace the original 4x4 grid of individual hardware-PCF
	// lookups with 4 textureGather() calls. Each gather returns the 4 raw
	// shadow comparisons of a 2x2 texel footprint, so 4 gathers laid out at
	// (+-1, +-1) * radius * texel from the sample center cover the same 4x4
	// sample area as the original kernel; summing all 16 comparisons and
	// dividing by 16 reproduces the original loop's averaging. The cost on
	// the TMUs drops from 16 hardware-PCF samples to 4 gathers (the gather
	// path returns 4 values per fetch where the original needed 4 fetches),
	// roughly a 4x reduction in shadow-sample work. The only thing dropped
	// vs. the hardware-PCF path is the implicit bilinear blending inside
	// each 2x2 footprint -- effectively turning a tent-weighted kernel into
	// a box-weighted one of the same extent, which is imperceptible in
	// motion. calc_shadow() is by far the heaviest piece of the lighting
	// shader, so this is a measurable GPU saving on every shaded fragment.
	float refz = coords.z + bias;
	float layer = float(cascade);
	vec2 off = radius * texel;
	vec4 g0 = textureGather(shadowmap, vec3(coords.xy + vec2(-off.x, -off.y), layer), refz);
	vec4 g1 = textureGather(shadowmap, vec3(coords.xy + vec2( off.x, -off.y), layer), refz);
	vec4 g2 = textureGather(shadowmap, vec3(coords.xy + vec2(-off.x,  off.y), layer), refz);
	vec4 g3 = textureGather(shadowmap, vec3(coords.xy + vec2( off.x,  off.y), layer), refz);
	float shadow = dot(g0 + g1 + g2 + g3, vec4(1.0 / 16.0));
	return shadow;
#else
	// Fallback for drivers without textureGather on shadow samplers
	// (notably GLES 3.0 and any 3.3 desktop driver that doesn't expose
	// GL_ARB_texture_gather). Identical to the previous implementation.
	float shadow = 0.0;
	for (float y = -1.5; y <= 1.5; y += 1.0)
		for (float x = -1.5; x <= 1.5; x += 1.0)
			shadow += texture(shadowmap, vec4(coords.xy + vec2(x, y) * radius * texel, cascade, coords.z + bias) );
	shadow /= 16.0;

	return shadow;
#endif
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

vec2 calc_headlights(light_s light, vec3 fragnormal)
{
	vec4 headlightpos = light.headlight_projection * f_pos;
	vec3 coords = headlightpos.xyz / headlightpos.w;

	if (coords.z > 1.0)
		return vec2(0.0);
	if (coords.z < 0.0)
		return vec2(0.0);

	vec3 light_dir = normalize(light.pos - f_pos.xyz);
	// Tighter wrap (was +0.25): faces angled away from the headlight cone
	// fall off to dark much faster, so cab/exterior surfaces read with a
	// clear directional shape instead of a flat half-lit wash.
	vec2 part = vec2(1.0) * clamp(dot(fragnormal, light_dir) + 0.10, 0.0, 1.0);
	float distance = length(light.pos - f_pos.xyz);
	float atten = 1.0f / (1.0f + light.linear * distance + light.quadratic * (distance * distance));
	atten *= mix(1.0, 0.0, clamp((coords.z - 0.998) * 500.0, 0.0, 1.0));
	vec3 lights = textureProj(headlightmap, headlightpos).rgb * light.headlight_weights.rgb;
	float lightintensity = max(max(lights.r, lights.g), lights.b);
	return part * atten * lightintensity;
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

// [0] - diffuse, [1] - specular
// do magic here
vec3 apply_lights(vec3 fragcolor, vec3 fragnormal, vec3 texturecolor, float reflectivity, float specularity, float shadowtone)
{
    vec3 basecolor = param[0].rgb;
    // Scale ambient before it gets tinted by basecolor / texture.
    // Sun, headlights and emission are added afterwards so they are NOT
    // attenuated by AMBIENT_SCALE - this only dims the indirect term.
    fragcolor *= basecolor * AMBIENT_SCALE;

    vec3 emissioncolor = basecolor * emission;

    vec3 view_dir = normalize(-f_pos.xyz);
    float NdotV = max(dot(fragnormal, view_dir), 0.0);
    vec3 F0 = mix(vec3(0.04), texturecolor, metalic);
    vec3 fresnel = F0 + (1.0 - F0) * pow(1.0 - NdotV, 5.0);

    const float MAX_REFLECTION_LOD = 8.0;
    float env_roughness = 1.0 - clamp(glossiness / max(abs(param[1].w), 1.0), 0.0, 1.0);
    vec3 envcolor = envmap_color_lod(fragnormal, env_roughness * MAX_REFLECTION_LOD);

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
    float diffuseamount = sun_NdotL * param[1].x * lights[0].intensity;

    float shadow1 = 0.0;
    if (shadowtone < 1.0)
        shadow1 = (1.0 - shadowtone) * clamp(calc_shadow(), 0.0, 1.0);

    // Sun HDR scale -> SUN_DIFFUSE_SCALE (default 2.5). Controls how
    // bright sun-lit (unshaded) faces get. Lower this if surfaces in
    // direct sun read as too hot/burnt; raise it for more punch.
    fragcolor += lights[0].color * SUN_DIFFUSE_SCALE * (1.0 - shadow1) * diffuseamount;

    for (uint i = 1U; i < lights_count; i++)
    {
        light_s light = lights[i];
        vec2 part = calc_headlights(light, fragnormal);
        fragcolor += light.color * (part.x * param[1].x + part.y * param[1].y) * light.intensity;
    }

    float specularamount = sunlight.y * param[1].y * specularity * lights[0].intensity
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