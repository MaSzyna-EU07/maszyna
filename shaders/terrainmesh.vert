// Terrain mesh, vertex stage.
//
// A vertex is a position, the ground material it belongs to, and the ground's normal there, folded onto
// an octahedron and kept in two bytes. The position is measured from the middle of the terrain rather
// than from the scenery's origin, so that the whole of a map fits in floats and every resident tile can
// live in one vertex buffer - which is what lets a frame of terrain be one draw call. terrainorigin is
// where that middle is relative to the camera, terrainworld where it is in the world.
//
// There are no texture coordinates: ground textures tile in world space, so the coordinate follows from
// where the vertex is and from how many metres one repeat of its material covers, which the cook
// measured from the scenery and left in the material table.
//
// Everything arrives as a float rather than as an integer. The engine's vertex array wrapper sets
// attributes up that way, and going around it would leave what the engine thinks is bound out of step
// with what is.

#include <common>

layout(location = 0) in vec3 in_position;
layout(location = 1) in float in_material;
layout(location = 2) in vec2 in_normal;     // octahedron-encoded, 0..255 each

uniform vec3 terrainorigin;   // the terrain's middle, relative to the camera
uniform vec2 terrainworld;    // the terrain's middle, in the world

out vec4 v_pos;
out vec3 v_normal;
out vec2 v_ground;            // world position on the ground, for texturing
out uint v_material;

// the square unfolded back onto the sphere. y is up, so it is y whose sign the fold carries
vec3 unfold_normal( vec2 Encoded )
{
	vec2 folded = Encoded / 255.0 * 2.0 - 1.0;
	vec3 normal = vec3( folded.x, 1.0 - abs( folded.x ) - abs( folded.y ), folded.y );
	if( normal.y < 0.0 ) {
		float wasx = normal.x;
		normal.x = ( 1.0 - abs( normal.z ) ) * ( wasx >= 0.0 ? 1.0 : -1.0 );
		normal.z = ( 1.0 - abs( wasx ) ) * ( normal.z >= 0.0 ? 1.0 : -1.0 );
	}
	return normalize( normal );
}

void main()
{
	vec3 local = in_position + terrainorigin;
	v_ground = terrainworld + in_position.xz;
	v_material = uint( in_material + 0.5 );
	v_normal = normalize( modelviewnormal * unfold_normal( in_normal ) );
	v_pos = modelview * vec4( local, 1.0 );
	gl_Position = projection * v_pos;
}
