// the settings of the game the shaders depend on (gfx.* in eu07.ini). in the SPIR-V binaries (shaders/ogl46bin) they are
// specialization constants the game sets when it loads a binary (gl::shader_setting in gl/shader.h has the ids); compiled
// from the source, constants of the values the game puts before the source (the defaults below otherwise)
#ifndef SETTINGS_GLSL
#define SETTINGS_GLSL

#ifdef GL_SPIRV
layout(constant_id = 0) const bool SHADOWMAP_ENABLED = true;
layout(constant_id = 1) const bool ENVMAP_ENABLED = true;
layout(constant_id = 2) const bool MOTIONBLUR_ENABLED = true;
// the colour goes on to the post-processing; tonemapped by the shaders of the scene otherwise
layout(constant_id = 3) const bool POSTFX_ENABLED = true;
layout(constant_id = 4) const bool EXTRAEFFECTS_ENABLED = true;
// whether the colour written to the window is encoded by the shader (FBOUT) or left to the sRGB framebuffer (dither.glsl)
layout(constant_id = 5) const bool OUTPUT_GAMMA = false;
#else
#ifndef SHADOWMAP_ENABLED_VALUE
#define SHADOWMAP_ENABLED_VALUE true
#endif
#ifndef ENVMAP_ENABLED_VALUE
#define ENVMAP_ENABLED_VALUE true
#endif
#ifndef MOTIONBLUR_ENABLED_VALUE
#define MOTIONBLUR_ENABLED_VALUE true
#endif
#ifndef POSTFX_ENABLED_VALUE
#define POSTFX_ENABLED_VALUE true
#endif
#ifndef EXTRAEFFECTS_ENABLED_VALUE
#define EXTRAEFFECTS_ENABLED_VALUE true
#endif
#ifndef OUTPUT_GAMMA_VALUE
#define OUTPUT_GAMMA_VALUE false
#endif
const bool SHADOWMAP_ENABLED = SHADOWMAP_ENABLED_VALUE;
const bool ENVMAP_ENABLED = ENVMAP_ENABLED_VALUE;
const bool MOTIONBLUR_ENABLED = MOTIONBLUR_ENABLED_VALUE;
const bool POSTFX_ENABLED = POSTFX_ENABLED_VALUE;
const bool EXTRAEFFECTS_ENABLED = EXTRAEFFECTS_ENABLED_VALUE;
const bool OUTPUT_GAMMA = OUTPUT_GAMMA_VALUE;
#endif

// the colour written to the window
vec4 FBOUT(vec4 x)
{
	return (OUTPUT_GAMMA ? vec4(pow(x.rgb, vec3(1.0 / 2.2)), x.a) : x);
}

#endif
