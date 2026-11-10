// the environment map: the sky and the clouds around the camera (the reflection pass draws nothing else into it since the
// screen space reflections took over the scenery), with its mipmaps and the levels prefiltered with the GGX lobe
layout(binding = ENV_TEX) uniform samplerCube envmap;
// the same map prefiltered with the GGX lobe (envmap_prefilter.comp), used when env_ggx is 1
layout(binding = ENV_GGX_TEX) uniform samplerCube envmap_ggx;

// the world space direction of the reflection of the view off a surface of the normal (view space)
vec3 envmap_direction(vec3 normal)
{
	vec3 refvec = reflect(f_pos.xyz, normal); // view space
	return vec3(inv_view * vec4(refvec, 0.0)); // world space
}

// the map in a world space direction, at a level of its mipmaps
vec3 envmap_sample_lod(vec3 direction, float lod)
{
	if (ENVMAP_ENABLED)
	{
		vec3 envcolor = textureLod(envmap, direction, lod).rgb;
		// Sanitize. The env cubemap can briefly contain NaN/Inf after a
		// reflection-pass regeneration (m_empty_cubemap was bound during the
		// face render and on NVIDIA can sample as undefined), and the mipmap
		// generation propagates a single NaN texel across the whole mip chain.
		// Without this guard a single bad texel produces one-frame black
		// flashes on glossy (esp. specgloss) surfaces.
		if (any(isnan(envcolor)) || any(isinf(envcolor))) envcolor = vec3(0.0);
		return max(envcolor, vec3(0.0));
	}
	else
	{
		return vec3(0.5);
	}
}

// the map in a world space direction for a surface of the given perceptual roughness (0 mirror, 1 matte)
vec3 envmap_sample_rough(vec3 direction, float roughness)
{
	// the plain map, its mipmaps (box filtered) standing for the roughness
	const float MAX_REFLECTION_LOD = 8.0;
	vec3 boxcolor = envmap_sample_lod(direction, roughness * MAX_REFLECTION_LOD);
	if (ENVMAP_ENABLED)
	{
		if (env_ggx == 0U || roughness <= ENVMAP_GGX_ROUGHNESS_MIN)
			return boxcolor;
		float level = (roughness - ENVMAP_GGX_ROUGHNESS_MIN) / (1.0 - ENVMAP_GGX_ROUGHNESS_MIN) * float(ENVMAP_GGX_LEVELS - 1U);
		vec3 ggxcolor = textureLod(envmap_ggx, direction, level).rgb;
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

vec3 envmap_color( vec3 normal )
{
	return envmap_sample_lod(envmap_direction(normal), 0.0);
}

// Roughness-aware env map lookup — uses mip levels for blurry reflections.
// lod 0.0 = mirror sharp, lod ~8.0 = fully diffuse blur.
vec3 envmap_color_lod(vec3 fragnormal, float lod)
{
	return envmap_sample_lod(envmap_direction(fragnormal), lod);
}

// reflection of the surroundings in a surface of the given perceptual roughness (0 mirror, 1 matte)
vec3 envmap_color_rough(vec3 fragnormal, float roughness)
{
	return envmap_sample_rough(envmap_direction(fragnormal), roughness);
}
