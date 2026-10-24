// the stars of the catalogue (environment/starcatalog.h) as points: turned from the equatorial frame to the sky of the observer
// by the sidereal time and the latitude, dimmed through the air towards the horizon and twinkling there

layout(location = 0) in vec4 v_star; // xyz: direction in the equatorial frame (z to the north pole), w: visual magnitude
layout(location = 1) in vec4 v_color; // rgb: colour of the star, w: a random phase of the twinkling

#include <common>

// equatorial frame to world axes (x west, y up, z north), and world axes to the view
layout(location = 0) uniform mat4 equatorial_to_world;
layout(location = 1) uniform mat4 world_to_view;
// x: brightness of a star of magnitude 0, y: seconds, z: size of the points in pixels
layout(location = 2) uniform vec4 params;

out vec3 f_color;

void main()
{
	vec3 world = mat3(equatorial_to_world) * v_star.xyz;
	float elevation = world.y;
	if (elevation < -0.01)
	{
		// below the horizon: out of the view
		gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
		f_color = vec3(0.0);
		return;
	}
	// air mass after Kasten and Young, 0.25 magnitude of extinction per air mass
	float altitude = max(degrees(asin(clamp(elevation, 0.0, 1.0))), 0.0);
	float airmass = 1.0 / (max(elevation, 0.0) + 0.50572 * pow(altitude + 6.07995, -1.6364));
	float magnitude = v_star.w + 0.25 * airmass;
	// low stars twinkle
	float twinkle = 1.0 + 0.35 * clamp((airmass - 1.0) * 0.25, 0.0, 1.0) * sin(params.y * (7.0 + 5.0 * v_color.w) + v_color.w * 40.0);
	float flux = pow(10.0, -0.4 * magnitude) * params.x * twinkle;
	f_color = v_color.rgb * flux;
	vec3 view = mat3(world_to_view) * world;
	gl_Position = projection * vec4(view, 1.0);
	// at the far end of the depth (reverse z)
	gl_Position.z = 0.0;
	gl_PointSize = params.z * clamp(1.0 + 0.4 * log(1.0 + flux / max(params.x * 0.01, 1e-6)), 1.0, 2.5);
}
