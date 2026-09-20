// Terrain mesh, fragment stage.
//
// The texture coordinate is the world position over the material's tiling, so the ground keeps the
// scale it was authored at whatever level the tile is drawn at. Where a triangle has one material at
// every corner - which is nearly all of them - that is one texture sample; where it crosses a boundary
// the three are blended by how near the fragment is to each corner, so the ground changes over a
// triangle rather than along its edge.
//
// The normal comes from the vertex, not from how the view position changes across the triangle. A
// normal worked out here would be the triangle's own, so the ground would read as facets, and a coarser
// level's facets face differently from the ones they stand for - which shows as the light over a
// hillside changing when the level does.

in vec4 f_pos;
in vec3 f_normal;
in vec2 f_ground;
in vec3 f_weights;
flat in uvec3 f_materials;

#include <common>
#include <apply_fog.glsl>
#include <tonemapping.glsl>

uniform sampler2DArray ground;
// per material: rgb the colour to show while it has no texture, a the metres one texture repeat
// covers - negative while there is no texture layer for it yet
uniform sampler2D materialtable;

layout(location = 0) out vec4 out_color;
#if MOTIONBLUR_ENABLED
layout(location = 1) out vec4 out_motion;
#endif

// Value noise from the world position: the same everywhere the ground is, at any level, and costing
// nothing to store. Two octaves are enough for what it is for.
float ground_hash( vec2 Cell )
{
	return fract( sin( dot( Cell, vec2( 127.1, 311.7 ) ) ) * 43758.5453 );
}

float ground_noise( vec2 Where )
{
	vec2 cell = floor( Where );
	vec2 part = fract( Where );
	part = part * part * ( 3.0 - 2.0 * part );
	return mix(
		mix( ground_hash( cell ), ground_hash( cell + vec2( 1.0, 0.0 ) ), part.x ),
		mix( ground_hash( cell + vec2( 0.0, 1.0 ) ), ground_hash( cell + vec2( 1.0, 1.0 ) ), part.x ),
		part.y );
}

vec3 material_colour( uint Material )
{
	vec4 entry = texelFetch( materialtable, ivec2( int( Material ), 0 ), 0 );
	if( entry.a < 0.0 ) { return entry.rgb; }
	return texture( ground, vec3( f_ground / max( entry.a, 0.01 ), float( Material ) ) ).rgb;
}

void main()
{
	vec3 albedo;
	if( ( f_materials.x == f_materials.y ) && ( f_materials.x == f_materials.z ) ) {
		// the common case: one kind of ground across the whole triangle, one texture sample
		albedo = material_colour( f_materials.x );
	}
	else {
		// The weights say how near the fragment is to each corner, so left alone the ground would fade
		// along a straight line across the triangle and the mesh would be visible in the texturing.
		// Disturbing them with noise taken from the world position makes the two grounds interlock
		// instead, which is what a boundary between them looks like - and since the noise is a function
		// of where the fragment is, it does not change with the level the tile is drawn at.
		float grain = ground_noise( f_ground * 0.6 ) * 0.62
		            + ground_noise( f_ground * 2.9 ) * 0.28
		            + ground_noise( f_ground * 11.0 ) * 0.10;
		vec3 weights = max( f_weights + ( grain - 0.5 ) * 0.55, vec3( 0.0 ) );
		// and sharpened a little, so the interlocking reads as one ground over the other rather than as
		// a wash of both
		weights = weights * weights;
		weights /= max( weights.x + weights.y + weights.z, 1e-4 );
		albedo = weights.x * material_colour( f_materials.x )
		       + weights.y * material_colour( f_materials.y )
		       + weights.z * material_colour( f_materials.z );
	}

	vec3 normal = normalize( f_normal );
	// a wall is drawn from either side, so the normal follows whichever way it is seen from
	if( dot( normal, -f_pos.xyz ) < 0.0 ) { normal = -normal; }

	// a plain lambert term against the first directional light, so that relief reads
	float lambert = 0.35;
	if( lights_count >= 1U && lights[ 0 ].type == LIGHT_DIR ) {
		lambert = 0.35 + 0.65 * max( dot( normal, normalize( -lights[ 0 ].dir ) ), 0.0 );
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
