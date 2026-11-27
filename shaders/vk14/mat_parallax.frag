#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
layout(location = 0) in vec3 f_normal;
layout(location = 2) in vec2 f_coord;
layout(location = 3) in vec4 f_pos;
layout(location = 8) in mat3 f_tbn; //tangent matrix nietransponowany; mnożyć przez f_tbn dla TangentLightPos; TangentViewPos; TangentFragPos;
layout(location = 4) in vec4 f_clip_pos;
layout(location = 5) in vec4 f_clip_future_pos;
layout(location = 6) in vec3 TangentFragPos;


layout(location = 0) out vec4 out_color;
#ifdef NO_MOTION_OUTPUT
// a pass without the velocity target: the writes go nowhere
vec4 out_motion;
#else
layout(location = 1) out vec4 out_motion;
#endif

#pragma param(color, 0, 0, 4, diffuse)
#pragma param(diffuse, 1, 0, 1, diffuse)
#pragma param(specular, 1, 1, 1, specular)
#pragma param(reflection, 1, 2, 1, one)
#pragma param(glossiness, 1, 3, 1, glossiness)
#pragma param(height_scale, 2, 1, 1, zero)
#pragma param(height_offset, 2, 2, 1, zero)

#pragma texture(diffuse, 0, sRGB_A)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 0) uniform sampler2D diffuse;

#pragma texture(normalmap, 1, RGBA)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 1) uniform sampler2D normalmap;

#define PARALLAX
#include "light_common.glsl"
#include "apply_fog.glsl"
#include "tonemapping.glsl"

vec2 ParallaxMapping(vec2 f_coord, vec3 viewDir);

void main()
{
	//parallax mapping
	vec3 viewDir = normalize(vec3(0.0f, 0.0f, 0.0f) - TangentFragPos); //tangent view pos - tangent frag pos
	vec2 f_coord_p = ParallaxMapping(f_coord, viewDir);

	vec4 tex_color = texture(diffuse, f_coord_p);

	bool alphatestfail = ( opacity >= 0.0 ? (tex_color.a < opacity) : (tex_color.a >= -opacity) );
	if(alphatestfail)
		discard;
//	if (tex_color.a < opacity)
//		discard;

	vec3 fragcolor = ambient;

	vec3 normal;
	normal.xy = (texture(normalmap, f_coord_p).rg * 2.0 - 1.0);
	normal.z = sqrt(1.0 - clamp((dot(normal.xy, normal.xy)), 0.0, 1.0));
	vec3 fragnormal = normalize(f_tbn * normalize(normal.xyz));
	float reflectivity = param[1].z * texture(normalmap, f_coord_p).a;
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

		out_motion = vec4(a - b, 0.0f, 0.0f);
	}
}

vec2 ParallaxMapping(vec2 f_coord, vec3 viewDir)
{
	float pos_len = length(f_pos.xyz);
	// gradients of the coordinates taken here: inside the loop the fragments of a quad don't go the same way
	vec2 dcoorddx = dFdx(f_coord);
	vec2 dcoorddy = dFdy(f_coord);

	if (pos_len > 100.0) {
		return f_coord;
	}
	// the shift fades out before the distance it stops at, so it doesn't switch off at once there
	float fade = 1.0 - smoothstep(80.0, 100.0, pos_len);

	if (EXTRAEFFECTS_ENABLED)
	{
		const float minLayers = 8.0;
		const float maxLayers = 32.0;
		float LayersWeight = pos_len / 20.0;
		vec2  currentTexCoords = f_coord;
		float currentDepthMapValue = textureGrad(normalmap, currentTexCoords, dcoorddx, dcoorddy).b;
		LayersWeight = min(abs(dot(vec3(0.0, 0.0, 1.0), viewDir)),LayersWeight);
		float numLayers = mix(maxLayers, minLayers, clamp(LayersWeight, 0.0, 1.0)); // number of depth layers
		float layerDepth = 1.0 / numLayers; // calculate the size of each layer
		float currentLayerDepth = 0.0; // depth of current layer
		vec2 P = viewDir.xy * param[2].y * fade; // the amount to shift the texture coordinates per layer (from vector P)
		vec2 deltaTexCoords = P / numLayers;

	  
		while(currentLayerDepth < currentDepthMapValue)
		{
			currentTexCoords -= deltaTexCoords;		// shift texture coordinates along direction of P
			currentDepthMapValue = textureGrad(normalmap, currentTexCoords, dcoorddx, dcoorddy).b;		// get depthmap value at current texture coordinates
			currentLayerDepth += layerDepth;  		// get depth of next layer
		}
	
		vec2 prevTexCoords = currentTexCoords + deltaTexCoords; // get texture coordinates before collision (reverse operations)

		float afterDepth  = currentDepthMapValue - currentLayerDepth; // get depth after and before collision for linear interpolation
		float beforeDepth = textureGrad(normalmap, prevTexCoords, dcoorddx, dcoorddy).b - currentLayerDepth + layerDepth;
	 
		float weight = afterDepth / (afterDepth - beforeDepth); // interpolation of texture coordinates
		vec2 finalTexCoords = prevTexCoords * weight + currentTexCoords * (1.0 - weight);

		return finalTexCoords;
	}
	else
	{
		float height = texture(normalmap, f_coord).b;
		vec2 p = viewDir.xy / viewDir.z * (height * (param[2].y - param[2].z) * 0.2) * fade;
		return f_coord - p;
	}
} 
