// Terrain mesh, fragment stage.
//
// One triangle carries one material: the cook only welds vertices that share it, so nothing has to
// be blended here and a material boundary follows the edges the scenery drew it with. The texture
// coordinate is the world position over the material's tiling, so the ground keeps the scale it was
// authored at whatever level the tile is drawn at.
//
// The mesh carries no normals - that would be four more bytes on every vertex - so the surface
// normal comes from how the view position changes across the triangle, which is its face normal.

in vec4 f_pos;
in vec2 f_ground;
flat in uint f_material;

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

void main()
{
	vec4 entry = texelFetch( materialtable, ivec2( int( f_material ), 0 ), 0 );
	vec3 albedo =
		( entry.a < 0.0 )
			? entry.rgb
			: texture( ground, vec3( f_ground / max( entry.a, 0.01 ), float( f_material ) ) ).rgb;

	vec3 normal = normalize( cross( dFdx( f_pos.xyz ), dFdy( f_pos.xyz ) ) );
	// towards the camera, whichever way the derivatives came out
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
