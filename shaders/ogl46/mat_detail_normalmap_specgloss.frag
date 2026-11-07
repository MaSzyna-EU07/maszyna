#version 460 core
#define STAGE_FRAGMENT 1
layout(location = 0) in vec3 f_normal;
layout(location = 2) in vec2 f_coord;
layout(location = 3) in vec4 f_pos;
layout(location = 8) in mat3 f_tbn;

layout(location = 4) in vec4 f_clip_pos;
layout(location = 5) in vec4 f_clip_future_pos;

#include "common.glsl"

layout(location = 0) out vec4 out_color;
layout(location = 1) out vec4 out_motion;

#pragma param(color, 0, 0, 4, diffuse)
#pragma param(diffuse, 1, 0, 1, diffuse)
#pragma param(specular, 1, 1, 1, specular)
#pragma param(reflection, 1, 2, 1, one)
#pragma param(glossiness, 1, 3, 1, glossiness)
#pragma param(detail_scale, 2, 0, 1, one)
#pragma param(detail_height_scale, 2, 1, 1, one)

#pragma texture(diffuse, 0, sRGB_A)
layout(binding = 0) uniform sampler2D diffuse;

#pragma texture(normalmap, 1, RGBA)
layout(binding = 1) uniform sampler2D normalmap;

#pragma texture(detailnormalmap, 2, RGBA)
layout(binding = 2) uniform sampler2D detailnormalmap;

#pragma texture(specgloss, 3, RGBA)
layout(binding = 3) uniform sampler2D specgloss;

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
	
	vec4 normal_map = texture(normalmap, f_coord);
	vec4 detailnormal_map = texture(detailnormalmap, f_coord * param[2].x);
	vec4 specgloss_map = texture(specgloss, f_coord);
	vec3 normal;
	vec3 normaldetail;
	
	normaldetail.xy = detailnormal_map.rg* 2.0 - 1.0;
	normaldetail.z = sqrt(1.0 - clamp((dot(normaldetail.xy, normaldetail.xy)), 0.0, 1.0));
	normaldetail.xyz = normaldetail.xyz * param[2].y;
	normal.xy =       normal_map.rg* 2.0 - 1.0;
	normal.z = sqrt(1.0 - clamp((dot(normal.xy, normal.xy)), 0.0, 1.0));
	
	vec3 fragnormal = normalize(f_tbn * normalize(vec3(normal.xy + normaldetail.xy, normal.z)));
	
	float reflblend = (normal_map.a + detailnormal_map.a);
	float reflectivity = reflblend > 1.0 ? param[1].z * 1.0 : param[1].z * reflblend;
	float specularity = specgloss_map.r;
	glossiness = specgloss_map.g * abs(param[1].w);
	metalic = specgloss_map.b;
	
	
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
        
        out_motion = vec4(a - b, 0.0f, 0.0f);
	}
}
