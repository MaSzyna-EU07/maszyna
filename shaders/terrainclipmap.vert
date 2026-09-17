// Terrain heightfield, vertex stage.
//
// There is no vertex buffer: the grid exists only as an index buffer, and a sample's row and
// column come from gl_VertexID. A whole level is drawn in one instanced call; what differs
// between its tiles - where each stands and which layer of the level's texture arrays holds
// it - comes from a buffer texture, two texels per instance. Heights are read as unsigned
// integers so that the cooker's no-data value survives unchanged instead of being normalised
// into a float.
//
// Levels meet without cracks and change without popping because each vertex morphs into the
// next coarser level as its distance nears the end of its level's range (CDLOD). The mip chain
// is point sampled, so the next level's grid is this level's even samples: an odd vertex
// slides onto its even neighbour and takes its height, and by the end of the range the tile
// is, vertex for vertex, the coarser tile that lies beyond it.

#include <common>

uniform usampler2DArray heights;
uniform samplerBuffer instances;

uniform float samplestep;   // metres between samples at this level
uniform float heightbias;   // world height of stored value 0
uniform float heightscale;  // metres per stored unit
uniform uint nodata;
uniform int side;           // samples along a tile side at this level
uniform vec2 morph;         // distance at which morphing into the next level starts, and where it is complete

out vec4 f_pos;
out vec3 f_normal;
out vec2 f_ground;          // world position on the ground, for texturing and material lookup
flat out vec2 f_tileworld;  // world position of the tile's first sample
flat out int f_layer;       // the tile's layer in the level's arrays

int layer;

// height of the first neighbouring sample that carries terrain, or the lowest height on the
// map when the sample is surrounded by emptiness. a sample outside the cooked ground has no
// height of its own, and decoding the no-data value as one would put the vertex at the top of
// the map's range
float borrowed_height( ivec2 Texel )
{
	const ivec2 offsets[ 4 ] = ivec2[ 4 ]( ivec2( -1, 0 ), ivec2( 1, 0 ), ivec2( 0, -1 ), ivec2( 0, 1 ) );
	for( int index = 0; index < 4; ++index ) {
		ivec2 neighbour = clamp( Texel + offsets[ index ], ivec2( 0 ), ivec2( side - 1 ) );
		uint raw = texelFetch( heights, ivec3( neighbour, layer ), 0 ).r;
		if( raw != nodata ) { return heightbias + float( raw ) * heightscale; }
	}
	return heightbias;
}

float height_at( ivec2 Texel )
{
	uint raw = texelFetch( heights, ivec3( Texel, layer ), 0 ).r;
	return ( raw == nodata ) ? borrowed_height( Texel ) : heightbias + float( raw ) * heightscale;
}

// height of a neighbouring sample, for the central difference the normal is built from. a
// neighbour carrying no terrain gives Fallback, so the edge of a hole comes out flat rather
// than as a wall to a surface a hundred metres up
float sample_height( ivec2 Texel, float Fallback )
{
	ivec2 clamped = clamp( Texel, ivec2( 0 ), ivec2( side - 1 ) );
	uint raw = texelFetch( heights, ivec3( clamped, layer ), 0 ).r;
	if( raw == nodata ) { return Fallback; }
	return heightbias + float( raw ) * heightscale;
}

void main()
{
	vec4 placement = texelFetch( instances, gl_InstanceID * 2 );
	vec4 world = texelFetch( instances, gl_InstanceID * 2 + 1 );
	vec3 tileorigin = placement.xyz;
	layer = int( placement.w );

	ivec2 texel = ivec2( gl_VertexID % side, gl_VertexID / side );
	float height = height_at( texel );
	vec3 local = tileorigin + vec3( vec2( texel ).x * samplestep, height, vec2( texel ).y * samplestep );

	// the viewpoint is at the horizontal origin, so the distance is the length of local.xz,
	// measured as the renderer measures it when choosing the level
	float morphing = clamp( ( length( local.xz ) - morph.x ) / ( morph.y - morph.x ), 0.0, 1.0 );
	if( morphing > 0.0 ) {
		ivec2 coarse = texel - ( texel % 2 );
		vec2 grid = mix( vec2( texel ), vec2( coarse ), morphing );
		local = tileorigin + vec3( grid.x * samplestep, mix( height, height_at( coarse ), morphing ), grid.y * samplestep );
	}

	// central differences over the neighbouring samples; at a tile edge the clamp repeats the
	// border sample, which is what the neighbouring tile holds there as well
	float west = sample_height( texel - ivec2( 1, 0 ), height );
	float east = sample_height( texel + ivec2( 1, 0 ), height );
	float north = sample_height( texel - ivec2( 0, 1 ), height );
	float south = sample_height( texel + ivec2( 0, 1 ), height );
	vec3 normal = normalize( vec3( west - east, 2.0 * samplestep, north - south ) );

	f_tileworld = world.xy;
	f_layer = layer;
	// world coordinates, so the ground texture stays put as the camera moves and as the tile
	// changes level
	f_ground = f_tileworld + ( local.xz - tileorigin.xz );

	f_pos = modelview * vec4( local, 1.0 );
	f_normal = normalize( modelviewnormal * normal );

	gl_Position = projection * f_pos;
}
