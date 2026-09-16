// Terrain heightfield, vertex stage.
//
// There is no vertex buffer: the grid exists only as an index buffer, and a sample's
// row and column come from gl_VertexID. Heights are read as unsigned integers so that
// the cooker's no-data value survives unchanged instead of being normalised into a float.

#include <common>

uniform usampler2D heights;
uniform usampler2D materials;

// tile origin relative to the viewpoint, with the viewpoint height already folded into y
uniform vec3 tileorigin;
uniform float samplestep;   // metres between samples at the level this tile is drawn at
uniform float heightbias;   // world height of stored value 0
uniform float heightscale;  // metres per stored unit
uniform uint nodata;
uniform int side;           // samples along a tile side at this level
uniform vec2 tileworld;     // tile origin in world space, for texture coordinates

out vec4 f_pos;
out vec3 f_normal;
out vec2 f_ground;
out float f_valid;
flat out uint f_material;

// height of a neighbouring sample, for the central difference the normal is built from.
// a neighbour that carries no terrain would otherwise decode as the no-data value and
// read as a surface a hundred metres up, wrecking the normal and leaving the ground
// unlit; Fallback keeps the difference at zero there, so the edge of a hole comes out
// flat rather than black
// height of the first neighbouring sample that carries terrain, or the lowest height on
// the map when the sample is surrounded by emptiness
float borrowed_height( ivec2 Texel )
{
	const ivec2 offsets[ 4 ] = ivec2[ 4 ]( ivec2( -1, 0 ), ivec2( 1, 0 ), ivec2( 0, -1 ), ivec2( 0, 1 ) );
	for( int index = 0; index < 4; ++index ) {
		ivec2 neighbour = clamp( Texel + offsets[ index ], ivec2( 0 ), ivec2( side - 1 ) );
		uint raw = texelFetch( heights, neighbour, 0 ).r;
		if( raw != nodata ) { return heightbias + float( raw ) * heightscale; }
	}
	return heightbias;
}

float sample_height( ivec2 Texel, float Fallback )
{
	ivec2 clamped = clamp( Texel, ivec2( 0 ), ivec2( side - 1 ) );
	uint raw = texelFetch( heights, clamped, 0 ).r;
	if( raw == nodata ) { return Fallback; }
	return heightbias + float( raw ) * heightscale;
}

void main()
{
	int column = gl_VertexID % side;
	int row = gl_VertexID / side;
	ivec2 texel = ivec2( column, row );

	uint raw = texelFetch( heights, texel, 0 ).r;
	f_valid = ( raw == nodata ) ? 0.0 : 1.0;
	f_material = texelFetch( materials, texel, 0 ).r;

	// a sample outside the cooked ground has no height. decoding the no-data value as
	// one puts the vertex at the top of the map's range, and since f_valid is
	// interpolated, the slivers of triangle nearest the valid corners survive the
	// fragment test and stretch up there. so an empty sample borrows a neighbour's
	// height: the fragments are still dropped, but the geometry stays on the ground
	float height = heightbias + float( raw ) * heightscale;
	if( raw == nodata ) {
		height = borrowed_height( texel );
	}
	vec3 local = tileorigin + vec3( float( column ) * samplestep, height, float( row ) * samplestep );

	// central differences over the neighbouring samples; at a tile edge the clamp repeats
	// the border sample, which is what the neighbouring tile holds there as well
	float west = sample_height( texel - ivec2( 1, 0 ), height );
	float east = sample_height( texel + ivec2( 1, 0 ), height );
	float north = sample_height( texel - ivec2( 0, 1 ), height );
	float south = sample_height( texel + ivec2( 0, 1 ), height );
	vec3 normal = normalize( vec3( west - east, 2.0 * samplestep, north - south ) );

	// world coordinates, so the ground texture stays put as the camera moves and as the
	// tile changes mip level
	f_ground = tileworld + vec2( float( column ), float( row ) ) * samplestep;

	f_pos = modelview * vec4( local, 1.0 );
	f_normal = normalize( modelviewnormal * normal );

	gl_Position = projection * f_pos;
}
