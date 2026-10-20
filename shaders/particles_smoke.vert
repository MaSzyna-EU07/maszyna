// the smoke of the gpu (opengl46_gpu_particles): a billboard per particle the compute pass added to the draw, turned about the
// axis of the view by the rotation of the particle (as opengl46_particles builds them on the cpu)

layout(location = 0) in vec4 v_place; // xyz: metres from the camera (world axes), w: size
layout(location = 1) in vec4 v_shape; // x: rotation, radians
layout(location = 2) in vec4 v_color; // rgb: lit colour, a: opacity

out vec4 f_pos;
out vec4 f_color;
out vec2 f_coord;
// metres in front of the scene the particle fades out over, see particle_soft.glsl
out float f_soft;

out vec4 f_clip_pos;
out vec4 f_clip_future_pos;

#include <common>

// x: share of the size of the particle it fades out over in front of the scene (gfx.particles.soft), 0: none
layout(location = 0) uniform vec4 particle_soft;

void main()
{
	// a triangle strip of four corners
	vec2 corner = vec2(float(gl_VertexID & 1), float(gl_VertexID >> 1)) - 0.5;
	float c = cos(v_shape.x);
	float s = sin(v_shape.x);
	vec2 offset = vec2(c * corner.x - s * corner.y, s * corner.x + c * corner.y) * v_place.w;
	f_pos = modelview * vec4(v_place.xyz, 1.0) + vec4(offset, 0.0, 0.0);
	f_clip_pos = projection * f_pos;
	f_clip_future_pos = (projection * future) * f_pos;

	gl_Position = f_clip_pos;
	f_coord = corner + 0.5;
	f_color = v_color;
	f_soft = particle_soft.x * 0.5 * v_place.w;
}
