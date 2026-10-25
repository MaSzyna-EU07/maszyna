// the ray traced reflections of the vulkan renderer (gfx.raytracing, vk14::ray_tracing), included by light_common.glsl in the
// fragment shaders of the scene when the renderer puts REFLECTION_RAYS before the sources (with the extensions: GL_EXT_ray_query for
// the rays, GL_EXT_buffer_reference2 and _uvec2 for the vertices read by their device addresses, GL_EXT_nonuniform_qualifier for the
// textures picked by the instance hit). what the screen space reflections don't find is traced through the scenery around the camera
// the renderer gathered for the frame (the walk of the scene in its reflection mode: the sections, the models and the vehicles within
// gfx.raytracing.range, as gfx.reflections.fidelity says), and the point hit is lit by the sky, the sun (its shadow map where it
// holds the point, a ray of its own towards it elsewhere) and its own emission. the space of the rays is the world around the camera: the axes of the world, the camera at the origin

layout(set = RT_SET, binding = RT_SCENE) uniform accelerationStructureEXT rt_scene;

// what the renderer wrote for an instance (vk14::ray_tracing::record), picked by its custom index
struct rt_record
{
	uvec2 vertices; // the device address of the first vertex of the chunk (gfx::basic_vertex, 12 floats)
	uvec2 indices; // of its first index, zero: the vertices go one after another
	uint texindex; // in rt_textures, 0: none (white)
	uint flags; // 1: the texture is alpha tested with cutoff
	float cutoff; // the alpha test, as opacity of the model block
	float glow; // the light of its own, as emission of the model block
	vec4 colour; // the diffuse colour of the material, linear, as material_diffuse() makes it
};

layout(std430, set = RT_SET, binding = RT_RECORDS) readonly buffer rt_record_ssbo
{
	rt_record rt_records[];
};

layout(set = RT_SET, binding = RT_TEXTURES) uniform sampler2D rt_textures[];

layout(buffer_reference, std430, buffer_reference_align = 4) readonly buffer rt_vertex_data
{
	float rt_vertex[];
};

layout(buffer_reference, std430, buffer_reference_align = 4) readonly buffer rt_index_data
{
	uint rt_index[];
};

const uint RT_FLAG_ALPHA = 1U;
const uint RT_VERTEX_FLOATS = 12U;
// the masks of the instances (vk14::ray_tracing): the opaque ones, the alpha tested ones (the foliage). the shadow rays take the
// opaque ones only, and the reflections leave out the alpha tested ones when the renderer says so (env_params.y, 1)
const uint RT_MASK_OPAQUE = 1U;
const uint RT_MASK_ALPHA = 2U;
// the longest ray towards the sun from a point hit, metres
const float RT_SHADOW_REACH = 100.0;

// the vertices of a triangle of the record
uvec3 rt_triangle(rt_record record, uint primitive)
{
	if ((record.indices.x == 0U) && (record.indices.y == 0U))
		return uvec3(primitive * 3U) + uvec3(0U, 1U, 2U);
	rt_index_data indices = rt_index_data(record.indices);
	return uvec3(indices.rt_index[primitive * 3U], indices.rt_index[primitive * 3U + 1U], indices.rt_index[primitive * 3U + 2U]);
}

// the texture coordinates of a point of a triangle, its barycentric coordinates Weights
vec2 rt_coords(rt_record record, uvec3 triangle, vec2 weights)
{
	rt_vertex_data vertices = rt_vertex_data(record.vertices);
	vec2 a = vec2(vertices.rt_vertex[triangle.x * RT_VERTEX_FLOATS + 6U], vertices.rt_vertex[triangle.x * RT_VERTEX_FLOATS + 7U]);
	vec2 b = vec2(vertices.rt_vertex[triangle.y * RT_VERTEX_FLOATS + 6U], vertices.rt_vertex[triangle.y * RT_VERTEX_FLOATS + 7U]);
	vec2 c = vec2(vertices.rt_vertex[triangle.z * RT_VERTEX_FLOATS + 6U], vertices.rt_vertex[triangle.z * RT_VERTEX_FLOATS + 7U]);
	return a * (1.0 - weights.x - weights.y) + b * weights.x + c * weights.y;
}

vec3 rt_normal(rt_record record, uvec3 triangle, vec2 weights)
{
	rt_vertex_data vertices = rt_vertex_data(record.vertices);
	vec3 a = vec3(vertices.rt_vertex[triangle.x * RT_VERTEX_FLOATS + 3U], vertices.rt_vertex[triangle.x * RT_VERTEX_FLOATS + 4U], vertices.rt_vertex[triangle.x * RT_VERTEX_FLOATS + 5U]);
	vec3 b = vec3(vertices.rt_vertex[triangle.y * RT_VERTEX_FLOATS + 3U], vertices.rt_vertex[triangle.y * RT_VERTEX_FLOATS + 4U], vertices.rt_vertex[triangle.y * RT_VERTEX_FLOATS + 5U]);
	vec3 c = vec3(vertices.rt_vertex[triangle.z * RT_VERTEX_FLOATS + 3U], vertices.rt_vertex[triangle.z * RT_VERTEX_FLOATS + 4U], vertices.rt_vertex[triangle.z * RT_VERTEX_FLOATS + 5U]);
	return a * (1.0 - weights.x - weights.y) + b * weights.x + c * weights.y;
}

// the share of the sun reaching a point hit (Hit in the world around the camera, Normal facing the ray): the shadow map of the sun
// where it holds the point, which costs no ray, otherwise a ray of its own towards the sun (Sun)
float rt_sun_visible(vec3 hit, vec3 normal, vec3 sun, float distance)
{
	if (SHADOWMAP_ENABLED)
	{
		mat3 toview = transpose(mat3(inv_view));
		vec3 position = toview * hit;
		vec3 viewnormal = toview * normal;
		float span = dot(position, position);
		uint cascade;
		for (cascade = 0U; cascade < MAX_CASCADES; cascade++)
			if (span <= cascade_end[cascade])
				break;
		for (; cascade < MAX_CASCADES; cascade++)
		{
			mat4 shadowview = lightview[cascade];
			// the point pushed off its surface by a texel and a half of the map, as calc_shadow_map() does
			float texelspermetre = min(length(vec3(shadowview[0][0], shadowview[1][0], shadowview[2][0])), length(vec3(shadowview[0][1], shadowview[1][1], shadowview[2][1]))) *
			                       float(textureSize(shadowmap, 0).x);
			vec4 lightpos = shadowview * vec4(position + viewnormal * (1.5 / max(texelspermetre, 1e-3)), 1.0);
			vec3 coords = lightpos.xyz / lightpos.w;
			if (all(greaterThanEqual(coords.xy, vec2(0.0))) && all(lessThanEqual(coords.xy, vec2(1.0))))
			{
				if (coords.z < 0.0)
					return 1.0;
				return 1.0 - texture(shadowmap, vec4(coords.xy, float(cascade), coords.z + 0.0001 * float(cascade + 1U)));
			}
			// the cascades fitted to the depth of the view (gfx.shadow.sdsm) cover the area of the surfaces seen only: a further one may
			// hold the point
			if (shadow_fit.x <= 0.0)
				break;
		}
	}
	rayQueryEXT shadow;
	rayQueryInitializeEXT(shadow, rt_scene, gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT, RT_MASK_OPAQUE, hit + normal * (0.02 + distance * 0.0005), 0.0, sun,
	                      min(ssr_params2.w, RT_SHADOW_REACH));
	while (rayQueryProceedEXT(shadow))
	{
	}
	return (rayQueryGetIntersectionTypeEXT(shadow, true) == gl_RayQueryCommittedIntersectionNoneEXT ? 1.0 : 0.0);
}

// the reflection the scenery gives, of a surface at Position (view space) with the geometric normal Normal (view space, either side),
// for a ray along Direction (view space, normalized) and the perceptual roughness Roughness: the colour (rgb, times a) and whether
// the ray hit something (a)
vec4 rt_trace(vec3 position, vec3 normal, vec3 direction, float roughness)
{
	mat3 toworld = mat3(inv_view);
	vec3 point = toworld * position;
	vec3 face = normalize(toworld * normal);
	// the side of the surface the camera sees, the ray starting a little off it
	if (dot(face, point) > 0.0)
		face = -face;
	vec3 ray = normalize(toworld * direction);
	vec3 origin = point + face * (0.02 + length(point) * 0.0005);

	rayQueryEXT query;
	uint mask = ((uint(env_params.y) & 1U) != 0U ? RT_MASK_OPAQUE : (RT_MASK_OPAQUE | RT_MASK_ALPHA));
	rayQueryInitializeEXT(query, rt_scene, gl_RayFlagsNoneEXT, mask, origin, 0.0, ray, ssr_params2.w);
	while (rayQueryProceedEXT(query))
	{
		// the instances with alpha tested textures aren't opaque: the texture tells whether the triangle is hit there
		if (rayQueryGetIntersectionTypeEXT(query, false) == gl_RayQueryCandidateIntersectionTriangleEXT)
		{
			rt_record record = rt_records[rayQueryGetIntersectionInstanceCustomIndexEXT(query, false)];
			if ((record.flags & RT_FLAG_ALPHA) == 0U)
			{
				rayQueryConfirmIntersectionEXT(query);
				continue;
			}
			uvec3 triangle = rt_triangle(record, uint(rayQueryGetIntersectionPrimitiveIndexEXT(query, false)));
			vec2 coords = rt_coords(record, triangle, rayQueryGetIntersectionBarycentricsEXT(query, false));
			float alpha = textureLod(rt_textures[nonuniformEXT(record.texindex)], coords, 0.0).a;
			if (alpha >= record.cutoff)
				rayQueryConfirmIntersectionEXT(query);
		}
	}
	if (rayQueryGetIntersectionTypeEXT(query, true) != gl_RayQueryCommittedIntersectionTriangleEXT)
		return vec4(0.0);

	float distance = rayQueryGetIntersectionTEXT(query, true);
	rt_record record = rt_records[rayQueryGetIntersectionInstanceCustomIndexEXT(query, true)];
	uvec3 triangle = rt_triangle(record, uint(rayQueryGetIntersectionPrimitiveIndexEXT(query, true)));
	vec2 weights = rayQueryGetIntersectionBarycentricsEXT(query, true);
	mat4x3 objecttoworld = rayQueryGetIntersectionObjectToWorldEXT(query, true);
	vec3 hitnormal = normalize(transpose(inverse(mat3(objecttoworld))) * rt_normal(record, triangle, weights));
	if (dot(hitnormal, ray) > 0.0)
		hitnormal = -hitnormal;
	vec3 hit = origin + ray * distance;

	// the colour of the surface: the farther and the rougher, the blurrier the level of the texture
	vec3 albedo = record.colour.rgb;
	if (record.texindex != 0U)
	{
		vec2 coords = rt_coords(record, triangle, weights);
		float lod = clamp(log2(1.0 + distance * 0.05) * 2.0 + roughness * 6.0, 0.0, 12.0);
		albedo *= textureLod(rt_textures[nonuniformEXT(record.texindex)], coords, lod).rgb;
	}
	// the light of the sky, the sun where nothing stands between the point and it, and the light of its own
	vec3 light = max(ambient + sky_ambient_directional(hitnormal) * sky_ambient_factor, vec3(0.0)) * AMBIENT_SCALE;
	if (lights_count > 0U)
	{
		vec3 sun = -normalize(toworld * lights[0].dir);
		float facing = dot(hitnormal, sun);
		if (facing > 0.0)
		{
			float visible = rt_sun_visible(hit, hitnormal, sun, distance);
			if (visible > 0.0)
				light += lights[0].color * (SUN_DIFFUSE_SCALE * lights[0].intensity * visible * cloud_shadow(hit) * pow(facing, SUN_NDOTL_SHARPNESS));
		}
	}
	vec3 color = albedo * (light + vec3(record.glow));
	if (any(isnan(color)) || any(isinf(color)))
		return vec4(0.0);
	return vec4(max(color, vec3(0.0)), 1.0);
}
