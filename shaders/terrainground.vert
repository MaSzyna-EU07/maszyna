// Terrain ground textures, copy pass: a triangle covering the whole target, with no vertex
// buffer. Draws one source texture into one layer of the ground array.

out vec2 f_coord;

void main()
{
	vec2 corner = vec2( float( ( gl_VertexID & 1 ) << 2 ), float( ( gl_VertexID & 2 ) << 1 ) );
	f_coord = corner * 0.5;
	gl_Position = vec4( corner - 1.0, 0.0, 1.0 );
}
