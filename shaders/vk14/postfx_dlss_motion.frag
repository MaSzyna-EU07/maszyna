#version 460 core
#define STAGE_FRAGMENT 1
#define LAYOUT_POST 1
#include "common.glsl"
// the velocities dlss takes (rendering/vulkan14/vulkan14dlss.h): those the colour pass drew, and for the sky, which has none drawn
// (the depth of the reverse z at its infinity), the motion of the camera, as the temporal anti-aliasing takes it (taa_resolve.glsl).
// param is pc.data of the push constants: param[2..5] the columns of the matrix of the clip space of this frame to the one of the
// last frame, for the world standing still (gfx::temporal_aa::reprojection())
layout(location = 14) in vec2 f_coords;
layout(location = 0) out vec4 out_motion;

#pragma texture(velocity_tex, 0, RG)
layout(set = POST_SET, binding = POST_TEXTURE_FIRST + 0) uniform sampler2D velocity_tex;

#pragma texture(depth_tex, 1, R)
layout(set = POST_SET, binding = POST_TEXTURE_FIRST + 1) uniform sampler2D depth_tex;

void main()
{
	ivec2 pixel = ivec2(gl_FragCoord.xy);
	vec2 velocity = texelFetch(velocity_tex, pixel, 0).xy;
	float depth = texelFetch(depth_tex, pixel, 0).r;
	if (depth <= 0.0)
	{
		vec4 world = mat4(param[2], param[3], param[4], param[5]) * vec4(f_coords * 2.0 - 1.0, depth, 1.0);
		velocity = (world.w > 0.0) ? (world.xy / world.w * 0.5 + 0.5) - f_coords : vec2(0.0);
	}
	out_motion = vec4(velocity, 0.0, 0.0);
}
