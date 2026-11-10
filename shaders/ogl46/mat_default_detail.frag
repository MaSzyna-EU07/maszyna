#version 460 core
#define STAGE_FRAGMENT 1
layout(location = 0) in vec3 f_normal;
layout(location = 2) in vec2 f_coord;
layout(location = 3) in vec4 f_pos;
layout(location = 8) in mat3 f_tbn;

layout(location = 4) in vec4 f_clip_pos;
layout(location = 5) in vec4 f_clip_future_pos;

#include "common.glsl"

#pragma param(color, 0, 0, 4, diffuse)
#pragma param(diffuse, 1, 0, 1, diffuse)
#pragma param(specular, 1, 1, 1, specular)
#pragma param(reflection, 1, 2, 1, zero)
#pragma param(glossiness, 1, 3, 1, glossiness)
#pragma param(detail_scale, 2, 0, 1, one)

#pragma texture(diffuse, 0, sRGB_A)
layout(binding = 0) uniform sampler2D diffuse;

layout(location = 0) out vec4 out_color;
layout(location = 1) out vec4 out_motion;

#pragma texture(detailnormalmap, 1, RGBA)
layout(binding = 1) uniform sampler2D detailnormalmap;

#define NORMALMAP
#include "light_common.glsl"
#include "apply_fog.glsl"
#include "tonemapping.glsl"

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
	normal.xy = (texture(detailnormalmap, f_coord * param[2].x).rg * 2.0 - 1.0);
	normal.z = sqrt(1.0 - clamp((dot(normal.xy, normal.xy)), 0.0, 1.0));
	vec3 fragnormal = normalize(f_tbn * normalize(normal.xyz));
	float reflectivity = param[1].z;
	float specularity = (tex_color.r + tex_color.g + tex_color.b) * 0.5;
	legacy_gloss();
	
	fragcolor = apply_lights(fragcolor, fragnormal, tex_color.rgb, reflectivity, specularity, shadow_tone);
	vec4 color = vec4(apply_fog(fragcolor), tex_color.a * alpha_mult);
    if (POSTFX_ENABLED)
    {
        out_color = color;
    }
    else
    {
        out_color = tonemap(color);
    }

	if (MOTIONBLUR_ENABLED)
	{
        vec2 a = (f_clip_future_pos.xy / f_clip_future_pos.w) * 0.5 + 0.5;;
        vec2 b = (f_clip_pos.xy / f_clip_pos.w) * 0.5 + 0.5;;
        
        out_motion = vec4(a - b, 0.0f, tex_color.a * alpha_mult);
	}
}
