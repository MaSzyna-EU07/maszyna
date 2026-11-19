// the wind blowing a span of the catenary aside (TTraction::sway(), weather_physics::wire_sway(): the same the pantographs meet the
// wire at): param[1] of the draw of a span holds the world x and z of the horizontal across it (xy) and how far its middle goes along
// it (z, m); the texture x of a vertex its place along the span, 0..1. the wires, their solids and their shadows go by it alike
#ifndef WIRE_SWAY_GLSL
#define WIRE_SWAY_GLSL

vec3 wire_sway(vec3 vertex, vec2 coord)
{
	float u = clamp(coord.x, 0.0, 1.0);
	return vertex + vec3(param[1].x, 0.0, param[1].y) * (param[1].z * 4.0 * u * (1.0 - u));
}

#endif
