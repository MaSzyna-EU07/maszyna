#version 460 core
#define STAGE_VERTEX 1
// rain and snow of the gpu (opengl46_gpu_particles): a streak per drop or flake the compute pass added to the draw, from where
// it is to where it was a moment ago as the moving camera saw it, turned to face the camera, rounded at its ends. a particle
// thinner than a pixel and a half is drawn that wide and as much fainter, so the distant ones don't flicker

layout(location = 0) in vec4 v_place; // xyz: the head, metres from the camera (world axes), w: width
layout(location = 1) in vec4 v_shape; // xyz: from the head to the tail
layout(location = 2) in vec4 v_color; // rgb: lit colour, a: opacity

layout(location = 3) out vec4 f_pos;
layout(location = 13) out vec4 f_color;
// x: across the streak, y: along it from the head, metres; z: its radius, w: its length
layout(location = 17) out vec4 f_streak;
layout(location = 16) out float f_soft;

layout(location = 4) out vec4 f_clip_pos;
layout(location = 5) out vec4 f_clip_future_pos;

#include "common.glsl"

// x: metres it fades out over in front of the scene, 0: none
layout(location = 0) uniform vec4 particle_soft;
// x: width of a pixel one metre in front of the camera, metres
layout(location = 1) uniform vec4 particle_pixel;

void main()
{
	vec3 head = (modelview * vec4(v_place.xyz, 1.0)).xyz;
	vec3 axis = mat3(modelview) * v_shape.xyz;
	float span = length(axis);
	vec3 direction = (span > 1e-5) ? axis / span : vec3(0.0, -1.0, 0.0);
	vec3 middle = head + 0.5 * axis;

	float width = v_place.w;
	float alpha = v_color.a;
	float least = 1.5 * particle_pixel.x * max(-middle.z, 1e-3);
	if (width < least)
	{
		alpha *= width / least;
		width = least;
	}
	float radius = 0.5 * width;
	vec3 side = cross(direction, normalize(-middle));
	float sidelength = length(side);
	side = (sidelength > 1e-4) ? side / sidelength : vec3(1.0, 0.0, 0.0);

	// a triangle strip of four corners: across, along
	vec2 corner = vec2(float(gl_VertexID & 1), float(gl_VertexID >> 1));
	float across = (corner.x * 2.0 - 1.0) * radius;
	float along = mix(-radius, span + radius, corner.y);
	f_pos = vec4(head + direction * along + side * across, 1.0);
	f_clip_pos = projection * f_pos;
	f_clip_future_pos = (projection * future) * f_pos;

	gl_Position = f_clip_pos;
	f_streak = vec4(across, along, radius, span);
	f_color = vec4(v_color.rgb, alpha);
	f_soft = particle_soft.x;
}
