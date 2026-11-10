// soft particles (gfx::gpu_particles): a particle fades where it nears what is behind it, instead of being cut by it.
// the resolved depth of the scene drawn so far (SCENE_DEPTH_TEX of the pass set), reversed with the far plane at infinity: the distance
// along the axis of the view is the near plane over the depth, and the sky, which writes no depth, stays at 0

#ifndef PARTICLE_SOFT_GLSL
#define PARTICLE_SOFT_GLSL

layout(set = PASS_SET, binding = SCENE_DEPTH_TEX) uniform sampler2D particle_scene_depth;

// share of the particle left at the fragment, 0-1: Range (metres) is how far in front of the scene it fades out over, 0 for none.
// Fragdistance: of the fragment along the axis of the view
float particle_soft_fade(float range, float fragdistance)
{
	if (range <= 0.0)
		return 1.0;
	float depth = texelFetch(particle_scene_depth, ivec2(gl_FragCoord.xy), 0).r;
	if (depth <= 0.0)
		return 1.0;
	float scene = projection[3][2] / depth;
	float share = clamp((scene - fragdistance) / range, 0.0, 1.0);
	return share * share * (3.0 - 2.0 * share);
}

#endif
