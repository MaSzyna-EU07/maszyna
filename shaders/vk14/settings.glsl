// the settings of the game the shaders depend on (gfx.* in eu07.ini): specialization constants the renderer sets when it makes a
// pipeline, the ids are the SPEC_* of rendering/vulkan14/vulkan14bindings.h. included by common.glsl after the constants
#ifndef SETTINGS_GLSL
#define SETTINGS_GLSL

#ifndef SPEC_OUTPUT_GAMMA
#error "settings.glsl takes the ids of the specialization constants from common.glsl: include common.glsl instead"
#endif

layout(constant_id = SPEC_SHADOWMAP_ENABLED) const bool SHADOWMAP_ENABLED = true;
layout(constant_id = SPEC_ENVMAP_ENABLED) const bool ENVMAP_ENABLED = true;
layout(constant_id = SPEC_MOTIONBLUR_ENABLED) const bool MOTIONBLUR_ENABLED = true;
// the colour goes on to the post-processing; tonemapped by the shaders of the scene otherwise
layout(constant_id = SPEC_POSTFX_ENABLED) const bool POSTFX_ENABLED = true;
layout(constant_id = SPEC_EXTRAEFFECTS_ENABLED) const bool EXTRAEFFECTS_ENABLED = true;
// whether the colour written to the window is encoded by the shader (FBOUT): the window has no srgb format (gfx.shadergamma yes)
layout(constant_id = SPEC_OUTPUT_GAMMA) const bool OUTPUT_GAMMA = false;

// the colour written to the window
vec4 FBOUT(vec4 x)
{
	return (OUTPUT_GAMMA ? vec4(pow(x.rgb, vec3(1.0 / 2.2)), x.a) : x);
}

#endif
