// storage of the particles of the gpu (opengl46_gpu_particles, gl::particle_* in gl/ubo.h) for the compute passes

#ifndef PARTICLE_COMMON_GLSL
#define PARTICLE_COMMON_GLSL

// a particle of smoke; it's dead (or its place was never used) when its opacity is 0 or less
struct smoke_particle_s
{
	vec4 position; // xyz: metres from the origin of the particles (world axes), w: rotation, radians
	vec4 velocity; // xyz: m/s, w: size of the billboard, metres
	vec4 state; // x: opacity, y: age, seconds
};

// a source of smoke as the last update of the simulation left it (smoke_source::gpu_state)
struct smoke_source_s
{
	uvec4 pool; // x: first place of the source in the storage, y: number of its places, z: place of the first particle born in the update, w: particles born
	vec4 location; // xyz: emitter at the update, from the origin
	vec4 previous; // xyz: emitter at the update before, from the origin
	vec4 launch; // xy: inclination, radians, zw: speed, m/s
	vec4 initial; // xy: size, zw: opacity
	vec4 sizechange; // x: per second, y: min, z: max
	vec4 opacitychange; // x: per second, y: min, z: max
	vec4 motion; // x: rise, y: damping (per second, of the vertical velocity), z: least vertical velocity
	vec4 color; // rgb: colour of the smoke, lit by the lights of the pass
	vec4 oldcolor; // rgb: colour lit as the cpu does it, without the lights (gfx.smoke.lighting off)
};

// what a draw of a particle reads, as attributes of an instance
struct particle_draw_s
{
	vec4 place; // xyz: metres from the camera (world axes), w: size (smoke) or width (rain, snow)
	vec4 shape; // x: rotation (smoke); xyz: from the head of the streak to its tail (rain, snow)
	vec4 color; // rgb: lit colour, a: opacity
};

// the indirect draw command of the particles; the compute passes count the instances
layout(std430, binding = PARTICLE_COMMAND_SSBO_BINDING) buffer particle_command_ssbo
{
	uint particle_draw_vertices;
	uint particle_draw_instances;
	uint particle_draw_first;
	uint particle_draw_base;
};

layout(std430, binding = PARTICLE_OUTPUT_SSBO_BINDING) writeonly buffer particle_output_ssbo
{
	particle_draw_s particle_draws[];
};

// pcg hash
uint particle_hash(uint value)
{
	uint state = value * 747796405U + 2891336453U;
	uint word = ((state >> ((state >> 28U) + 4U)) ^ state) * 277803737U;
	return (word >> 22U) ^ word;
}

// next of a sequence of random numbers 0-1
float particle_random(inout uint state)
{
	state = particle_hash(state);
	return float(state >> 8U) * (1.0 / 16777216.0);
}

// whether a sphere (centre from the camera along the world axes) is within the planes of the view
bool particle_visible(vec4 planes[5], vec3 centre, float radius)
{
	for (int i = 0; i < 5; i++)
		if (dot(planes[i].xyz, centre) + planes[i].w < -radius)
			return false;
	return true;
}

// adds a particle to the draw; Capacity: the places of the storage of the draws
void particle_emit(vec3 place, float size, vec3 shape, vec3 color, float opacity, uint capacity)
{
	uint index = atomicAdd(particle_draw_instances, 1U);
	if (index >= capacity)
	{
		atomicAdd(particle_draw_instances, 0xFFFFFFFFU);
		return;
	}
	particle_draws[index].place = vec4(place, size);
	particle_draws[index].shape = vec4(shape, 0.0);
	particle_draws[index].color = vec4(color, opacity);
}

#endif
