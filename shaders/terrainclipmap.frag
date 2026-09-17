// Terrain heightfield, fragment stage.
//
// Every material of a tile is drawn in the same pass. The fragment finds the four samples
// around it in the tile's layer, drops itself if any of them carries no terrain - the
// heightfield only has ground where a whole cell does - and blends the ground textures of their
// materials by distance, so that material boundaries follow the samples smoothly rather than
// stepping. Texture coordinates are world space divided by the tiling the cooker measured from
// the source geometry, so the ground keeps the scale the scenery was authored with.

in vec4 f_pos;
in vec3 f_normal;
in vec2 f_ground;
flat in vec2 f_tileworld;
flat in int f_layer;

#include <common>
#include <apply_fog.glsl>
#include <tonemapping.glsl>

uniform usampler2DArray heights;
uniform usampler2DArray materials;
uniform sampler2DArray ground;
// per material: rgb the colour to show while it has no texture, a the metres one texture
// repeat covers - negative while there is no texture layer for it yet
uniform sampler2D materialtable;
uniform float samplestep;
uniform int side;
uniform uint nodata;

layout(location = 0) out vec4 out_color;
#if MOTIONBLUR_ENABLED
layout(location = 1) out vec4 out_motion;
#endif

vec3 material_colour( uint Material )
{
	vec4 entry = texelFetch( materialtable, ivec2( int( Material ), 0 ), 0 );
	if( entry.a < 0.0 ) { return entry.rgb; }
	return texture( ground, vec3( f_ground / max( entry.a, 0.01 ), float( Material ) ) ).rgb;
}

void main()
{
	vec2 cell = ( f_ground - f_tileworld ) / samplestep;
	ivec2 base = clamp( ivec2( floor( cell ) ), ivec2( 0 ), ivec2( side - 2 ) );
	vec2 weight = clamp( cell - vec2( base ), 0.0, 1.0 );

	ivec2 corners[ 4 ] = ivec2[ 4 ]( base, base + ivec2( 1, 0 ), base + ivec2( 0, 1 ), base + ivec2( 1, 1 ) );
	float weights[ 4 ] = float[ 4 ](
		( 1.0 - weight.x ) * ( 1.0 - weight.y ),
		weight.x * ( 1.0 - weight.y ),
		( 1.0 - weight.x ) * weight.y,
		weight.x * weight.y );

	uint kinds[ 4 ];
	for( int index = 0; index < 4; ++index ) {
		if( texelFetch( heights, ivec3( corners[ index ], f_layer ), 0 ).r == nodata ) { discard; }
		kinds[ index ] = texelFetch( materials, ivec3( corners[ index ], f_layer ), 0 ).r;
	}

	vec3 albedo;
	if( ( kinds[ 0 ] == kinds[ 1 ] ) && ( kinds[ 0 ] == kinds[ 2 ] ) && ( kinds[ 0 ] == kinds[ 3 ] ) ) {
		// the common case: one material across the whole cell, one texture sample
		albedo = material_colour( kinds[ 0 ] );
	}
	else {
		albedo = vec3( 0.0 );
		for( int index = 0; index < 4; ++index ) {
			albedo += weights[ index ] * material_colour( kinds[ index ] );
		}
	}

	// a plain lambert term against the first directional light, so that relief reads
	float lambert = 0.35;
	if( lights_count >= 1U && lights[ 0 ].type == LIGHT_DIR ) {
		lambert = 0.35 + 0.65 * max( dot( normalize( f_normal ), normalize( -lights[ 0 ].dir ) ), 0.0 );
	}
	vec3 col = albedo * lambert;

#if POSTFX_ENABLED
	out_color = vec4( apply_fog( col ), 1.0 );
#else
	out_color = tonemap( vec4( apply_fog( col ), 1.0 ) );
#endif
#if MOTIONBLUR_ENABLED
	out_motion = vec4( 0.0 );
#endif
}
