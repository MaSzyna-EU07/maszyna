// the ballast of a track of a material with normal and specgloss maps mixed in grains into that of the tracks joined at its ends, see trackblend.glsl
#define TRACKBLEND_GRAIN 1
#define TRACKBLEND_NORMALMAP 1
#define TRACKBLEND_SPECGLOSS 1
#texture (diffuse, 0, sRGB_A)
uniform sampler2D diffuse;
#texture (diffuse1, 1, sRGB_A)
uniform sampler2D diffuse1;
#texture (diffuse2, 2, sRGB_A)
uniform sampler2D diffuse2;
#texture (normalmap, 3, RGBA)
uniform sampler2D normalmap;
#texture (normalmap1, 4, RGBA)
uniform sampler2D normalmap1;
#texture (normalmap2, 5, RGBA)
uniform sampler2D normalmap2;
#texture (specgloss, 6, RGBA)
uniform sampler2D specgloss;
#include <trackblend.glsl>
