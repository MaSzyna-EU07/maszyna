#if ENVMAP_ENABLED
uniform samplerCube envmap;
#endif

vec3 envmap_color( vec3 normal )
{
#if ENVMAP_ENABLED
	vec3 refvec = reflect(f_pos.xyz, normal); // view space
	refvec = vec3(inv_view * vec4(refvec, 0.0)); // world space
	vec3 envcolor = texture(envmap, refvec).rgb;
	// Sanitize. The env cubemap can briefly contain NaN/Inf after a
	// reflection-pass regeneration (m_empty_cubemap was bound during the
	// face render and on NVIDIA can sample as undefined). Without this
	// guard, a single bad texel propagates through env_reflection in
	// apply_lights() and produces one-frame black flashes on glossy
	// (esp. specgloss) surfaces.
	if (any(isnan(envcolor)) || any(isinf(envcolor))) envcolor = vec3(0.0);
	envcolor = max(envcolor, vec3(0.0));
#else
	vec3 envcolor = vec3(0.5);
#endif
	return envcolor;
}

// Roughness-aware env map lookup — uses mip levels for blurry reflections.
// lod 0.0 = mirror sharp, lod ~8.0 = fully diffuse blur.
vec3 envmap_color_lod(vec3 fragnormal, float lod)
{
#if ENVMAP_ENABLED
    vec3 refvec = reflect(f_pos.xyz, fragnormal); // view space — matches envmap_color exactly
    refvec = vec3(inv_view * vec4(refvec, 0.0));  // world space — was missing
    vec3 envcolor = textureLod(envmap, refvec, lod).rgb;
    // See envmap_color() above — same NaN/Inf sanitize, also needed here
    // because mipmap generation propagates a single NaN texel across the
    // whole mip chain.
    if (any(isnan(envcolor)) || any(isinf(envcolor))) envcolor = vec3(0.0);
    return max(envcolor, vec3(0.0));
#else
    return vec3(0.5); // was vec3(0.0), match the non-LOD fallback
#endif
}
