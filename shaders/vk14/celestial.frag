#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
// the sun, the moon and the textured sprite of the easter egg, drawn over the sky on a quad around the direction of the body
// (celestial.vert). param[0]: colour (rgb) and alpha, param[1]: position in view space and half the size of the quad,
// param[2].xy: corner (sprite) or centre (moon) of the picture in the texture, param[2].z: its size (sprite) or the radius of
// the disc in it (moon), in texture units, param[2].w: kind of body, 0: textured sprite, 1: sun, 2: moon.
// the colour goes out premultiplied, the alpha is how much of the sky behind the body it hides (blending ONE, ONE_MINUS_SRC_ALPHA)

layout(location = 2) in vec2 f_coord;
layout(location = 15) in vec2 f_local;

#pragma texture(tex1, 0, sRGB_A)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 0) uniform sampler2D tex1;

#include "tonemapping.glsl"

// shape and light of the sun and the moon, pc.data[0..2] of the push constants (a loose uniform vec4 body[3] in the opengl
// renderer, set by opengl46_renderer::Render(world_environment *)):
// [0].xy: direction to the zenith on the quad, [0].z: height over width of the disc (flattened by the refraction near the
// horizon), [0].w: radius of the disc in units of half the quad
// [1].xyz: direction to the sun in view space, [1].w: light of the earth on the dark part of the moon, against the light of
// the sun on the lit part
// [2].rgb: limb darkening of the sun per channel (linear law)
#define body pc.data

layout(location = 0) out vec4 out_color;
#ifdef NO_MOTION_OUTPUT
// a pass without the velocity target: the writes go nowhere
vec4 out_motion;
#else
layout(location = 1) out vec4 out_motion;
#endif

void main()
{
	int kind = int(param[2].w + 0.5);
	vec4 color = vec4(0.0);
	if (kind == 0)
	{
		vec4 tex_color = texture(tex1, f_coord) * param[0];
		color = vec4(tex_color.rgb * tex_color.a, 0.0);
	}
	else
	{
		// the point on the disc in its radii, x along the horizon, y to the zenith, with the flattening undone
		vec2 zenith = body[0].xy;
		vec2 across = vec2(zenith.y, -zenith.x);
		vec2 p = vec2(dot(f_local, across), dot(f_local, zenith)) / body[0].w;
		p.y /= body[0].z;
		float r = length(p);
		float coverage = clamp((1.0 - r) / max(fwidth(r), 1e-4) + 0.5, 0.0, 1.0);
		// the horizon hides what is under it
		vec3 direction = normalize(param[1].xyz + vec3(f_local * param[1].w, 0.0));
		float height = (mat3(inv_view) * direction).y;
		coverage *= clamp(height / max(fwidth(height), 1e-7) + 0.5, 0.0, 1.0);
		// cosine of the angle between the line of sight and the surface of the sphere
		float mu = sqrt(max(1.0 - r * r, 0.0));
		if (kind == 1)
		{
			// limb darkening, kept to the average brightness of the disc
			vec3 limb = (vec3(1.0) - body[2].rgb * (1.0 - mu)) / (vec3(1.0) - body[2].rgb / 3.0);
			color = vec4(param[0].rgb * limb * coverage, 0.0);
		}
		else
		{
			// the moon as a sphere lit from the direction of the sun. the frame of the disc in view space, z to the viewer
			vec3 z = -normalize(param[1].xyz);
			vec3 x = vec3(across, 0.0);
			x = normalize(x - z * dot(x, z));
			vec3 y = cross(z, x);
			vec2 q = p / max(r, 1.0);
			vec3 normal = q.x * x + q.y * y + mu * z;
			float mu0 = max(dot(normal, normalize(body[1].xyz)), 0.0);
			// Lommel-Seeliger law: an even disc at the full moon, as the real one, no darkening towards the limb
			float lit = 2.0 * mu0 / (mu0 + max(mu, 1e-3));
			vec3 albedo = texture(tex1, param[2].xy + p * param[2].z).rgb;
			color = vec4(albedo * param[0].rgb * ((lit + body[1].w) * coverage), coverage * param[0].a);
		}
	}
	if (POSTFX_ENABLED)
	{
		out_color = color;
	}
	else
	{
		out_color = vec4(tonemap(vec4(color.rgb, 1.0)).rgb, color.a);
	}
	if (MOTIONBLUR_ENABLED)
	{
		out_motion = vec4(0.0f);
	}
}
