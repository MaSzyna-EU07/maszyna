// Terrain heightfield, fragment stage.
//
// A tile is drawn once per material it contains, with that material's texture bound, and
// every fragment belonging to a different material is dropped. Texture coordinates are
// world space divided by the tiling the cooker measured from the source geometry, so the
// ground keeps the scale the scenery was authored with.

in vec4 f_pos;
in vec3 f_normal;
in vec2 f_ground;
in float f_valid;
flat in uint f_material;

#include <common>
#include <apply_fog.glsl>
#include <tonemapping.glsl>

uniform sampler2D ground;
uniform float groundscale;   // metres of ground covered by one texture repeat
uniform uint drawnmaterial;  // the material this pass is drawing
uniform bool hastexture;     // false when the material has no texture to sample
uniform vec3 fallback;       // colour used in that case

layout(location = 0) out vec4 out_color;
#if MOTIONBLUR_ENABLED
layout(location = 1) out vec4 out_motion;
#endif

void main()
{
	// f_valid is 1.0 only where every vertex of this triangle carried terrain, so a
	// triangle reaching into a hole disappears. it erodes the boundary by up to one
	// sample, which is the price of holes being a property of samples, not of triangles
	if( f_valid < 0.999 ) { discard; }
	// this pass belongs to one material; everything else is drawn by another pass
	if( f_material != drawnmaterial ) { discard; }

	vec3 albedo = (
		hastexture ?
			texture( ground, f_ground / max( groundscale, 0.01 ) ).rgb :
			fallback );
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
