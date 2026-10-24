layout(binding = ENV_TEX) uniform samplerCube envmap;
// the same map prefiltered with the GGX lobe (envmap_prefilter.comp), used when env_ggx is 1
layout(binding = ENV_GGX_TEX) uniform samplerCube envmap_ggx;

vec3 envmap_color( vec3 normal )
{
	vec3 envcolor;
	if (ENVMAP_ENABLED)
	{
		vec3 refvec = reflect(f_pos.xyz, normal); // view space
		refvec = vec3(inv_view * vec4(refvec, 0.0)); // world space
		envcolor = texture(envmap, refvec).rgb;
		// Sanitize. The env cubemap can briefly contain NaN/Inf after a
		// reflection-pass regeneration (m_empty_cubemap was bound during the
		// face render and on NVIDIA can sample as undefined). Without this
		// guard, a single bad texel propagates through env_reflection in
		// apply_lights() and produces one-frame black flashes on glossy
		// (esp. specgloss) surfaces.
		if (any(isnan(envcolor)) || any(isinf(envcolor))) envcolor = vec3(0.0);
		envcolor = max(envcolor, vec3(0.0));
	}
	else
	{
		envcolor = vec3(0.5);
	}
	return envcolor;
}

// Roughness-aware env map lookup — uses mip levels for blurry reflections.
// lod 0.0 = mirror sharp, lod ~8.0 = fully diffuse blur.
vec3 envmap_color_lod(vec3 fragnormal, float lod)
{
    if (ENVMAP_ENABLED)
    {
        vec3 refvec = reflect(f_pos.xyz, fragnormal); // view space — matches envmap_color exactly
        refvec = vec3(inv_view * vec4(refvec, 0.0));  // world space — was missing
        vec3 envcolor = textureLod(envmap, refvec, lod).rgb;
        // See envmap_color() above — same NaN/Inf sanitize, also needed here
        // because mipmap generation propagates a single NaN texel across the
        // whole mip chain.
        if (any(isnan(envcolor)) || any(isinf(envcolor))) envcolor = vec3(0.0);
        return max(envcolor, vec3(0.0));
    }
    else
    {
        return vec3(0.5); // was vec3(0.0), match the non-LOD fallback
    }
}

// reflection of the surroundings in a surface of the given perceptual roughness (0 mirror, 1 matte)
vec3 envmap_color_rough(vec3 fragnormal, float roughness)
{
	// the plain map, its mipmaps (box filtered) standing for the roughness
	const float MAX_REFLECTION_LOD = 8.0;
	vec3 boxcolor = envmap_color_lod(fragnormal, roughness * MAX_REFLECTION_LOD);
	if (ENVMAP_ENABLED)
	{
		if (env_ggx == 0U || roughness <= ENVMAP_GGX_ROUGHNESS_MIN)
			return boxcolor;
		vec3 refvec = reflect(f_pos.xyz, fragnormal);
		refvec = vec3(inv_view * vec4(refvec, 0.0));
		float level = (roughness - ENVMAP_GGX_ROUGHNESS_MIN) / (1.0 - ENVMAP_GGX_ROUGHNESS_MIN) * float(ENVMAP_GGX_LEVELS - 1U);
		vec3 ggxcolor = textureLod(envmap_ggx, refvec, level).rgb;
		if (any(isnan(ggxcolor)) || any(isinf(ggxcolor))) ggxcolor = vec3(0.0);
		ggxcolor = max(ggxcolor, vec3(0.0));
		// the prefiltered map takes over over the first tenth of the roughness above its lowest level
		return mix(boxcolor, ggxcolor, clamp((roughness - ENVMAP_GGX_ROUGHNESS_MIN) * 10.0, 0.0, 1.0));
	}
	else
	{
		return boxcolor;
	}
}
