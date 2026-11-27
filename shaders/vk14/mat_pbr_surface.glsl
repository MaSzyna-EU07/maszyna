// the fragment stage of the pbr materials, see mat_pbr.glsl. the same file for the opengl and the vulkan renderer: the
// textures, the inputs and the outputs are declared by the files which include it

#ifdef PBR_PARALLAX
// the depth of the surface below its highest points, 0..1: the height map holds the height, white the highest
float parallax_depth(vec2 coord, vec2 dcoorddx, vec2 dcoorddy)
{
	return 1.0 - textureGrad(heightmap, coord, dcoorddx, dcoorddy).r;
}

// the texture coordinates of the point of the surface the view finds, with the surface sunk below the geometry by its depth
// (param[2].y: the depth of the black of the height map, as a share of the size of the texture). Viewdir: the direction
// to the eye in the tangent space
vec2 parallax_coord(vec2 coord, vec3 viewdir)
{
	float range = length(f_pos.xyz);
	if (range > 100.0)
		return coord;
	// gradients of the coordinates taken here: inside the loop the fragments of a quad don't go the same way
	vec2 dcoorddx = dFdx(coord);
	vec2 dcoorddy = dFdy(coord);
	// the shift fades out before the distance it stops at, so it doesn't switch off at once there
	vec2 shift = viewdir.xy * (param[2].y * (1.0 - smoothstep(80.0, 100.0, range)));

	if (false == EXTRAEFFECTS_ENABLED)
	{
		// one step: the depth at the point seen, taken for the depth of the point found
		return coord - shift * parallax_depth(coord, dcoorddx, dcoorddy);
	}

	// the ray is followed down through layers of the depth, more of them at grazing angles and close by, until it gets below the
	// surface; the point is then found between the last two steps
	float layers = mix(32.0, 8.0, clamp(min(abs(viewdir.z), range / 20.0), 0.0, 1.0));
	float layerdepth = 1.0 / layers;
	vec2 stride = shift / layers;
	vec2 current = coord;
	float found = parallax_depth(current, dcoorddx, dcoorddy);
	float reached = 0.0;
	while (reached < found)
	{
		current -= stride;
		found = parallax_depth(current, dcoorddx, dcoorddy);
		reached += layerdepth;
	}
	vec2 previous = current + stride;
	float after = found - reached;
	float before = parallax_depth(previous, dcoorddx, dcoorddy) - reached + layerdepth;
	float weight = after / min(after - before, -1e-5);
	return mix(current, previous, clamp(weight, 0.0, 1.0));
}
#endif

// the normal of the tangent space a normal map holds in its red and green
vec3 unpack_normal(vec2 texel)
{
	vec2 xy = texel * 2.0 - 1.0;
	return vec3(xy, sqrt(1.0 - clamp(dot(xy, xy), 0.0, 1.0)));
}

void main()
{
	vec2 coord = f_coord;
#ifdef PBR_PARALLAX
	coord = parallax_coord(f_coord, normalize(-TangentFragPos));
#endif

	vec4 tex_color = texture(diffuse, coord);

	bool alphatestfail = ( opacity >= 0.0 ? (tex_color.a < opacity) : (tex_color.a >= -opacity) );
	if(alphatestfail)
		discard;

	vec3 fragcolor = ambient;

	vec3 normal = unpack_normal(texture(normalmap, coord).rg);
#ifdef PBR_DETAIL
	// the detail tilts the normal further (the whiteout blend of the two)
	vec3 detail = unpack_normal(texture(detailnormalmap, coord * param[2].y).rg);
	normal = vec3(normal.xy + detail.xy * param[2].z, normal.z * detail.z);
#endif
	vec3 fragnormal = normalize(f_tbn * normalize(normal));

	vec3 surface = texture(orm, coord).rgb;
	pbr_occlusion = surface.r;
	pbr_roughness = surface.g * param[2].x;
	metalic = surface.b;
	pbr_emission = texture(emissive, coord).rgb * param[1].w;
#ifdef PBR_COAT
	vec2 varnish = texture(coatmap, coord).rg;
	coat_weight = varnish.r * param[2].y;
	coat_roughness = varnish.g + param[2].w;
#endif
	// the specular level of the tools: 0.5 is the reflectance of 4%
	float specularity = 0.5 * param[1].y;
	float reflectivity = param[1].z;

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
		vec2 a = (f_clip_future_pos.xy / f_clip_future_pos.w) * 0.5 + 0.5;
		vec2 b = (f_clip_pos.xy / f_clip_pos.w) * 0.5 + 0.5;

		out_motion = vec4(a - b, 0.0f, 0.0f);
	}
}
