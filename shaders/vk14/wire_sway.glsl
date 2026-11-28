// the wind blowing a span of the catenary aside (TTraction::sway(), weather_physics::wire_sway(): the same the pantographs meet the
// wire at): param[1] of the draw of a span holds the world x and z of the horizontal across it (xy) and how far its middle goes along
// it (z, m); the texture x of a vertex its place along the span, 0..1. and the ice on the wires (param[1].w, the radial thickness,
// m): the solids of the wires up close grow by it along their normals (the lines have none). the wires, their solids and their
// shadows go by it alike. below zero param[1].w is the pass drawing the lines of iced wires again a pixel aside (wire_pixel())
#ifndef WIRE_SWAY_GLSL
#define WIRE_SWAY_GLSL

vec3 wire_sway(vec3 vertex, vec3 normal, vec2 coord)
{
	float u = clamp(coord.x, 0.0, 1.0);
	return vertex + vec3(param[1].x, 0.0, param[1].y) * (param[1].z * 4.0 * u * (1.0 - u)) + normal * max(param[1].w, 0.0);
}

// the lines of the iced wires drawn again a pixel aside, so they show two pixels wide rather than one: param[1].w below zero, minus
// the size of a pixel at a unit of the depth (2 / (projection[1][1] * the height of the target)). View: the vertex in the space of
// the view; the shift goes across the wire on the screen, across it and across the line of sight. elsewhere nothing
vec3 wire_pixel(vec3 view)
{
	if (param[1].w >= 0.0)
		return vec3(0.0);
	// along the span, horizontal: across the horizontal param[1].xy goes across it
	vec3 along = mat3(modelview) * vec3(param[1].y, 0.0, -param[1].x);
	bool perspective = (projection[3][3] == 0.0);
	vec3 across = cross(along, perspective ? view : vec3(0.0, 0.0, -1.0));
	float size = length(across);
	if (size < 1e-6)
		return vec3(0.0);
	float depth = perspective ? max(-view.z, 0.0) : 1.0;
	return across / size * (-param[1].w * depth);
}

#endif
