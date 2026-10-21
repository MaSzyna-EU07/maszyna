// the rails of a track faded into those of the tracks joined at its ends, see trackblend.glsl
#texture (diffuse, 0, sRGB_A)
uniform sampler2D diffuse;
#texture (diffuse1, 1, sRGB_A)
uniform sampler2D diffuse1;
#texture (diffuse2, 2, sRGB_A)
uniform sampler2D diffuse2;
#texture (diffuse3, 3, sRGB_A)
uniform sampler2D diffuse3;
#include <trackblend.glsl>
