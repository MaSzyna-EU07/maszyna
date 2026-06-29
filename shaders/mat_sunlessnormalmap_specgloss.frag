in vec3 f_normal;
in vec2 f_coord;
in vec4 f_pos;
in mat3 f_tbn;

in vec4 f_clip_pos;
in vec4 f_clip_future_pos;

#include <common>

layout(location = 0) out vec4 out_color;
#if MOTIONBLUR_ENABLED
layout(location = 1) out vec4 out_motion;
#endif

#param (color, 0, 0, 4, diffuse)
#param (diffuse, 1, 0, 1, diffuse)
#param (specular, 1, 1, 1, specular)
#param (reflection, 1, 2, 1, zero)
#param (glossiness, 1, 3, 1, glossiness)

#texture (diffuse, 0, sRGB_A)
uniform sampler2D diffuse;

#texture (normalmap, 1, RGBA)
uniform sampler2D normalmap;

#texture (specgloss, 2, RGBA)
uniform sampler2D specgloss;


#define NORMALMAP
#include <light_common.glsl>
#include <apply_fog.glsl>
#include <tonemapping.glsl>

vec3 apply_lights_sunless(vec3 fragcolor, vec3 fragnormal, vec3 texturecolor, float reflectivity, float specularity, float shadowtone)
{
    vec3 basecolor = param[0].rgb;
    // Cab interior: dim ambient less than the exterior path - ambient
    // is the dominant indoor illumination, but it was still too bright.
    fragcolor *= basecolor * 0.80;

    vec3 emissioncolor = basecolor * emission;

    vec3 view_dir = normalize(-f_pos.xyz);
    float NdotV = max(dot(fragnormal, view_dir), 0.0);
    vec3 F0 = mix(vec3(0.04), texturecolor, metalic);
    vec3 fresnel = F0 + (1.0 - F0) * pow(1.0 - NdotV, 5.0);

    const float MAX_REFLECTION_LOD = 8.0;
    float env_roughness = 1.0 - clamp(glossiness / max(abs(param[1].w), 1.0), 0.0, 1.0);
    vec3 envcolor = envmap_color_lod(fragnormal, env_roughness * MAX_REFLECTION_LOD);

    // Pre-integrated env BRDF (matches apply_lights): roughness-attenuated
    // reflection so matte cab surfaces no longer mirror the environment.
    vec3 env_spec = EnvBRDFApprox(F0, env_roughness, NdotV);
    float env_spec_w = max(env_spec.r, max(env_spec.g, env_spec.b));

    vec3 texturecoloryuv = rgb2yuv(texturecolor);
    vec3 texturecolorfullv = yuv2rgb(vec3(0.2176, texturecoloryuv.gb));
    vec3 envyuv = rgb2yuv(envcolor);
    texturecolor = mix(texturecolor, texturecolorfullv, envyuv.r * reflectivity * env_spec_w);

    if (lights_count == 0U)
        return fragcolor * texturecolor * (1.0 - metalic)
             + emissioncolor * texturecolor
             + envcolor * env_spec * reflectivity;

    vec2 sunlight = calc_dir_light(lights[0], fragnormal);
    // Sharpen N.L for stronger contrast between lit and shaded cab
    // surfaces (uses SUN_NDOTL_SHARPNESS from light_common.glsl).
    float sun_NdotL = pow(sunlight.x, SUN_NDOTL_SHARPNESS);
    float diffuseamount = sun_NdotL * param[1].x * lights[0].intensity;

    float specularamount = sunlight.y * param[1].y * specularity * lights[0].intensity;

    for (uint i = 1U; i < lights_count; i++)
    {
        light_s light = lights[i];
        vec2 part = calc_headlights(light, fragnormal);
        fragcolor += light.color * (part.x * param[1].x + part.y * param[1].y) * light.intensity;
    }

    if (shadowtone < 1.0)
    {
        float shadow = calc_shadow();
        specularamount *= clamp(1.0 - shadow, 0.0, 1.0);
        fragcolor = mix(fragcolor, fragcolor * shadowtone, clamp(diffuseamount * shadow, 0.0, 1.0));
    }

    vec3 specularcolor = specularamount * lights[0].color;

    // Env reflection gated by env BRDF (roughness-attenuated, F0-tinted).
    vec3 env_reflection = envcolor * env_spec * reflectivity;

    // Physically-based metal/rough combine (matches apply_lights / Substance):
    // dielectric -> light-coloured highlight + full diffuse; metal -> albedo-tinted
    // highlight, no diffuse term. Highlight strength left unchanged.
    vec3 diffuse_albedo = texturecolor * (1.0 - metalic);
    vec3 spec_tint      = mix(vec3(1.0), texturecolor, metalic);

    vec3 result = fragcolor      * diffuse_albedo
                + specularcolor  * spec_tint
                + emissioncolor  * texturecolor
                + env_reflection;   // env_spec already F0-tinted for metals

    return result;
}

void main()
{
	vec4 tex_color = texture(diffuse, f_coord);

	bool alphatestfail = ( opacity >= 0.0 ? (tex_color.a < opacity) : (tex_color.a >= -opacity) );
	if(alphatestfail)
		discard;
//	if (tex_color.a < opacity)
//		discard;

	vec3 fragcolor = ambient;

	vec3 normal;
	normal.xy = (texture(normalmap, f_coord).rg * 2.0 - 1.0);
	normal.z = sqrt(1.0 - clamp((dot(normal.xy, normal.xy)), 0.0, 1.0));
	vec3 fragnormal = normalize(f_tbn * normalize(normal.xyz));
	float reflectivity = param[1].z * texture(normalmap, f_coord).a;
	float specularity = texture(specgloss, f_coord).r;
	glossiness = texture(specgloss, f_coord).g * abs(param[1].w);
	metalic = texture(specgloss, f_coord).b;
	
	fragcolor = apply_lights_sunless(fragcolor, fragnormal, tex_color.rgb, reflectivity, specularity, shadow_tone);
	vec4 color = vec4(apply_fog(fragcolor), tex_color.a * alpha_mult);
#if POSTFX_ENABLED
    out_color = color;
#else
    out_color = tonemap(color);
#endif
#if MOTIONBLUR_ENABLED
	{
        vec2 a = (f_clip_future_pos.xy / f_clip_future_pos.w) * 0.5 + 0.5;;
        vec2 b = (f_clip_pos.xy / f_clip_pos.w) * 0.5 + 0.5;;
        
        out_motion = vec4(a - b, 0.0f, 0.0f);
	}
#endif
}
