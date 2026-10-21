// the rails of a track of a normal mapped material faded into those of the tracks joined at its ends, see trackblend.glsl
#define TRACKBLEND_NORMALMAP 1
#texture (normalmap, 4, RGBA)
uniform sampler2D normalmap;
#texture (normalmap1, 5, RGBA)
uniform sampler2D normalmap1;
#texture (normalmap2, 6, RGBA)
uniform sampler2D normalmap2;
#texture (normalmap3, 7, RGBA)
uniform sampler2D normalmap3;
#include <trackblend.glsl>
