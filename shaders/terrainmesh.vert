// Terrain mesh, vertex stage.
//
// A vertex is a position and the ground material it belongs to, nothing else. The position is
// measured from the middle of the terrain rather than from the scenery's origin, so that the whole
// of a map fits in floats and every resident tile can live in one vertex buffer - which is what
// lets a frame of terrain be one draw call. terrainorigin is where that middle is relative to the
// camera, terrainworld where it is in the world.
//
// There are no texture coordinates: ground textures tile in world space, so the coordinate follows
// from where the vertex is and from how many metres one repeat of its material covers, which the
// cook measured from the scenery and left in the material table.

#include <common>

layout(location = 0) in vec3 in_position;
layout(location = 1) in uint in_material;

uniform vec3 terrainorigin;   // the terrain's middle, relative to the camera
uniform vec2 terrainworld;    // the terrain's middle, in the world

out vec4 f_pos;
out vec2 f_ground;            // world position on the ground, for texturing
flat out uint f_material;

void main()
{
	vec3 local = in_position + terrainorigin;
	f_ground = terrainworld + in_position.xz;
	f_material = in_material;
	f_pos = modelview * vec4( local, 1.0 );
	gl_Position = projection * f_pos;
}
