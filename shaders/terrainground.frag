// Terrain ground textures, copy pass. The source is sampled with its own mip chain, so a
// texture larger than the layer is filtered down rather than point sampled.

in vec2 f_coord;

uniform sampler2D source;

layout(location = 0) out vec4 out_color;

void main()
{
	out_color = vec4( texture( source, f_coord ).rgb, 1.0 );
}
