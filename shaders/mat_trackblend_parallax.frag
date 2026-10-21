// the rails of a track of a parallax material faded into those of the tracks joined at its ends, see trackblend.glsl
#define TRACKBLEND_NORMALMAP 1
#define TRACKBLEND_PARALLAX 1
#param (height_scale, 2, 1, 1, zero)
#param (height_offset, 2, 2, 1, zero)
#texture (diffuse, 0, sRGB_A)
uniform sampler2D diffuse;
#texture (diffuse1, 1, sRGB_A)
uniform sampler2D diffuse1;
#texture (diffuse2, 2, sRGB_A)
uniform sampler2D diffuse2;
#texture (diffuse3, 3, sRGB_A)
uniform sampler2D diffuse3;
#texture (normalmap, 4, RGBA)
uniform sampler2D normalmap;
#texture (normalmap1, 5, RGBA)
uniform sampler2D normalmap1;
#texture (normalmap2, 6, RGBA)
uniform sampler2D normalmap2;
#texture (normalmap3, 7, RGBA)
uniform sampler2D normalmap3;
#include <trackblend.glsl>
