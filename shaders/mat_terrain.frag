in vec3 f_normal;
in vec2 f_coord;
in vec4 f_pos;
in mat3 f_tbn;

in vec4 f_clip_pos;
in vec4 f_clip_future_pos;

#include <common>

layout(location = 0) out vec4 out_color;
#if MOTIONBLUR_ENABLED
layout(location = 1) out vec4 out_motion;
#endif

// heightmap terrain: up to 8 materials (layers) painted over a chunk, blended by their weights.
// the textures of all layers of the terrain are kept in texture arrays; the table of the chunk tells which layers it has:
//   row 0, per layer: index in the arrays, 1 / metres the textures repeat at, has normal map, has specular/gloss map
//   row 1, per layer: reflection, cosine and sine of the angle the textures are turned by
// f_coord holds metres from the corner of the pack of chunks, along x and -z

#param (color, 0, 0, 4, diffuse)
#param (diffuse, 1, 0, 1, diffuse)
#param (specular, 1, 1, 1, specular)
#param (reflection, 1, 2, 1, one)
#param (glossiness, 1, 3, 1, glossiness)
// corner of the chunk (x, z) in texture coordinates, 1 / size of the chunk, paint samples along a side less one
#param (placement, 2, 0, 4, zero)

#texture (opaque, 0, RGBA)
uniform sampler2D opaque;
#texture (weights0, 1, RGBA)
uniform sampler2D weights0;
#texture (weights1, 2, RGBA)
uniform sampler2D weights1;
#texture (layerdiffuse, 3, sRGB_A)
uniform highp sampler2DArray layerdiffuse;
#texture (layernormal, 4, RGBA)
uniform highp sampler2DArray layernormal;
#texture (layerspecgloss, 5, RGBA)
uniform highp sampler2DArray layerspecgloss;
#texture (layertable, 6, RGBA)
uniform highp sampler2D layertable;

#define NORMALMAP
#include <light_common.glsl>
#include <apply_fog.glsl>
#include <tonemapping.glsl>

void main()
{
	// place within the chunk, 0..1 along x and z, and the paint samples around it
	vec2 local = clamp(vec2(f_coord.x - param[2].x, -f_coord.y - param[2].y) * param[2].z, 0.0, 1.0);
	float samples = param[2].w;
	vec2 paintcoord = (local * samples + 0.5) / (samples + 1.0);
	vec4 w0 = texture(weights0, paintcoord);
	vec4 w1 = texture(weights1, paintcoord);
	float weights[8] = float[8](w0.r, w0.g, w0.b, w0.a, w1.r, w1.g, w1.b, w1.a);
	float painted = dot(w0, vec4(1.0)) + dot(w1, vec4(1.0));

	// the layers are sampled only where they're painted, so the gradients are taken here, where every pixel gets them
	vec2 coorddx = dFdx(f_coord);
	vec2 coorddy = dFdy(f_coord);

	vec3 albedo = vec3(0.0);
	vec3 normal = vec3(0.0);
	float reflectivity = 0.0;
	float specularity = 0.0;
	float gloss = 0.0;
	float metal = 0.0;
	float total = 0.0;
	for (int i = 0; i < 8; ++i)
	{
		// a chunk without weights shows its first layer
		float weight = (painted < 0.004 ? (i == 0 ? 1.0 : 0.0) : weights[i]);
		if (weight < 0.004)
			continue;
		vec4 info = texelFetch(layertable, ivec2(i, 0), 0);
		vec4 extra = texelFetch(layertable, ivec2(i, 1), 0);
		// turned counterclockwise seen from above; f_coord runs along x and -z, so that's clockwise in it
		vec2 turn = (extra.y == 0.0 && extra.z == 0.0) ? vec2(1.0, 0.0) : extra.yz;
		mat2 rotation = mat2(turn.x, -turn.y, turn.y, turn.x);
		vec3 coord = vec3(rotation * f_coord * info.y, info.x);
		vec2 dx = rotation * coorddx * info.y;
		vec2 dy = rotation * coorddy * info.y;
		albedo += textureGrad(layerdiffuse, coord, dx, dy).rgb * weight;
		vec3 layernormal_ts = vec3(0.0, 0.0, 1.0);
		float layerreflection = extra.x;
		if (info.z > 0.5)
		{
			vec4 normalmap = textureGrad(layernormal, coord, dx, dy);
			layernormal_ts.xy = normalmap.rg * 2.0 - 1.0;
			layernormal_ts.z = sqrt(1.0 - clamp(dot(layernormal_ts.xy, layernormal_ts.xy), 0.0, 1.0));
			// the normal map leans along the turned texture; turned back to the axes of the terrain
			layernormal_ts.xy = layernormal_ts.xy * rotation;
			layerreflection *= normalmap.a;
		}
		normal += layernormal_ts * weight;
		if (info.w > 0.5)
		{
			vec4 specgloss = textureGrad(layerspecgloss, coord, dx, dy);
			specularity += specgloss.r * weight;
			gloss += specgloss.g * weight;
			metal += specgloss.b * weight;
		}
		else
		{
			gloss += weight;
		}
		reflectivity += layerreflection * weight;
		total += weight;
	}
	total = max(total, 0.004);
	albedo /= total;
	specularity /= total;
	metal /= total;
	reflectivity = param[1].z * reflectivity / total;
	glossiness = (gloss / total) * abs(param[1].w);
	metalic = metal;

	vec3 fragnormal = normalize(f_tbn * normalize(normal));
	vec3 fragcolor = ambient;
	fragcolor = apply_lights(fragcolor, fragnormal, albedo, reflectivity, specularity, shadow_tone);

	vec4 color = vec4(apply_fog(fragcolor), alpha_mult);
#if POSTFX_ENABLED
	out_color = color;
#else
	out_color = tonemap(color);
#endif
#if MOTIONBLUR_ENABLED
	{
		vec2 a = (f_clip_future_pos.xy / f_clip_future_pos.w) * 0.5 + 0.5;
		vec2 b = (f_clip_pos.xy / f_clip_pos.w) * 0.5 + 0.5;

		out_motion = vec4(a - b, 0.0f, 0.0f);
	}
#endif
}
