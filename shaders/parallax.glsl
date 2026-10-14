vec2 ParallaxMapping(vec2 f_coord, vec3 viewDir, float scale, float offset)
{
	vec2 dx = dFdx(f_coord);
	vec2 dy = dFdy(f_coord);
	float fade = (1.0 - smoothstep(60.0, 100.0, length(f_pos.xyz))) * smoothstep(0.0, 0.1, viewDir.z);
	if (fade <= 0.0) {
		return f_coord;
	}

#if EXTRAEFFECTS_ENABLED
	const float minLayers = 8.0;
	const float maxLayers = 32.0;
	float numLayers = mix(maxLayers, minLayers, clamp(viewDir.z, 0.0, 1.0));
	float layerDepth = 1.0 / numLayers;
	vec2 deltaTexCoords = viewDir.xy * scale * fade / numLayers;
	vec2 currentTexCoords = f_coord;
	float currentLayerDepth = 0.0;
	float currentDepthMapValue = textureGrad(normalmap, currentTexCoords, dx, dy).b;
	for (int i = 0; i < 40 && currentLayerDepth < currentDepthMapValue; ++i)
	{
		currentTexCoords -= deltaTexCoords;
		currentLayerDepth += layerDepth;
		currentDepthMapValue = textureGrad(normalmap, currentTexCoords, dx, dy).b;
	}
	for (int i = 0; i < 5; ++i)
	{
		deltaTexCoords *= 0.5;
		layerDepth *= 0.5;
		if (currentLayerDepth < currentDepthMapValue) {
			currentTexCoords -= deltaTexCoords;
			currentLayerDepth += layerDepth;
		}
		else {
			currentTexCoords += deltaTexCoords;
			currentLayerDepth -= layerDepth;
		}
		currentDepthMapValue = textureGrad(normalmap, currentTexCoords, dx, dy).b;
	}
	return currentTexCoords;
#else
	float height = textureGrad(normalmap, f_coord, dx, dy).b;
	vec2 p = viewDir.xy / max(viewDir.z, 0.2) * (height * (scale - offset) * 0.2 * fade);
	return f_coord - p;
#endif
}
