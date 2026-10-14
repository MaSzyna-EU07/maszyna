float ParallaxDepth(vec2 f_coord, float lod)
{
	vec2 size = vec2(textureSize(normalmap, int(lod)));
	vec2 st = f_coord * size - 0.5;
	vec2 cell = floor(st);
	vec2 f = st - cell;
	vec2 w0 = (1.0 - f) * (1.0 - f) * (1.0 - f) / 6.0;
	vec2 w1 = (4.0 - 6.0 * f * f + 3.0 * f * f * f) / 6.0;
	vec2 w3 = f * f * f / 6.0;
	vec2 w2 = 1.0 - w0 - w1 - w3;
	vec2 g0 = w0 + w1;
	vec2 g1 = w2 + w3;
	vec2 h0 = (cell - 0.5 + w1 / g0) / size;
	vec2 h1 = (cell + 1.5 + w3 / g1) / size;
	return g0.y * (g0.x * textureLod(normalmap, h0, lod).b + g1.x * textureLod(normalmap, vec2(h1.x, h0.y), lod).b)
	     + g1.y * (g0.x * textureLod(normalmap, vec2(h0.x, h1.y), lod).b + g1.x * textureLod(normalmap, h1, lod).b);
}

float ParallaxShadow(vec2 f_coord, float depth, float lod, float scale)
{
	if (lights_count == 0U || depth <= 0.0) {
		return 0.0;
	}
	vec3 lightDir = normalize(transpose(f_tbn) * normalize(-lights[0].dir));
	if (lightDir.z <= 0.0) {
		return 0.0;
	}
	const int steps = 12;
	float layerDepth = depth / float(steps);
	vec2 deltaTexCoords = lightDir.xy / max(lightDir.z, 0.25) * scale * layerDepth;
	float shadow = 0.0;
	for (int i = 1; i <= steps; ++i)
	{
		float rayDepth = depth - layerDepth * float(i);
		float surface = ParallaxDepth(f_coord + deltaTexCoords * float(i), lod);
		shadow = max(shadow, (rayDepth - surface) * 12.0 * (1.0 - float(i) / float(steps + 1)));
	}
	return clamp(shadow, 0.0, 1.0);
}

vec2 ParallaxMapping(vec2 f_coord, vec3 viewDir, float scale, float offset)
{
	vec2 size = vec2(textureSize(normalmap, 0));
	float footprint = max(length(dFdx(f_coord) * size), length(dFdy(f_coord) * size));
	float lod = clamp(log2(max(footprint, 1e-4)) + 2.0, 0.0, log2(max(size.x, size.y)));
	float fade = (1.0 - smoothstep(60.0, 100.0, length(f_pos.xyz))) * smoothstep(0.0, 0.1, viewDir.z);
	if (fade <= 0.0) {
		return f_coord;
	}
	float meanDepth = textureLod(normalmap, f_coord, log2(max(size.x, size.y))).b;

#if EXTRAEFFECTS_ENABLED
	const float minLayers = 16.0;
	const float maxLayers = 64.0;
	float numLayers = mix(maxLayers, minLayers, clamp(viewDir.z, 0.0, 1.0));
	float layerDepth = 1.0 / numLayers;
	vec2 deltaTexCoords = viewDir.xy * scale * fade / numLayers;
	vec2 currentTexCoords = f_coord;
	float currentLayerDepth = 0.0;
	float currentDepthMapValue = textureLod(normalmap, currentTexCoords, lod).b;
	for (int i = 0; i < 100 && currentLayerDepth < currentDepthMapValue; ++i)
	{
		currentTexCoords -= deltaTexCoords;
		currentLayerDepth += layerDepth;
		currentDepthMapValue = textureLod(normalmap, currentTexCoords, lod).b;
	}
	currentDepthMapValue = ParallaxDepth(currentTexCoords, lod);
	for (int i = 0; i < 6; ++i)
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
		currentDepthMapValue = ParallaxDepth(currentTexCoords, lod);
	}
	parallax_occlusion = clamp(0.8 * (currentLayerDepth - meanDepth), -0.3, 0.6) * fade;
	parallax_shadow = ParallaxShadow(currentTexCoords, currentLayerDepth, lod, scale * fade) * fade;
	return currentTexCoords;
#else
	float height = ParallaxDepth(f_coord, lod);
	parallax_occlusion = clamp(0.8 * (height - meanDepth), -0.3, 0.6) * fade;
	vec2 p = viewDir.xy / max(viewDir.z, 0.2) * (height * (scale - offset) * 0.2 * fade);
	return f_coord - p;
#endif
}
