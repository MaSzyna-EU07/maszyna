// what the shaders of the vulkan 1.4 renderer share: the bindings of rendering/vulkan14/vulkan14bindings.h and the settings.
// compiled by the game, the bindings come from the renderer (vk14::binding_preamble()) and are taken instead of these; the
// SPIR-V binaries (shaders/vk14bin) are compiled with the values written here, so they have to stay the same as in the game,
// which checks them when it starts (and doesn't load the binaries when they differ)
#ifndef COMMON_GLSL
#define COMMON_GLSL

#if !defined(STAGE_VERTEX) && !defined(STAGE_FRAGMENT) && !defined(STAGE_GEOMETRY) && !defined(STAGE_COMPUTE)
#error "the stage of the shader isn't defined: #define STAGE_VERTEX 1 (STAGE_FRAGMENT, STAGE_GEOMETRY, STAGE_COMPUTE) before the include of common.glsl"
#endif

#ifndef MASZYNA_CONSTANTS
#define MASZYNA_CONSTANTS 1
#define UI_SET 0
#define UI_TEXTURE 0
#define PASS_SET 0
#define MATERIAL_SET 1
#define MATERIAL_TEXTURE_FIRST 0
#define SPEC_OUTPUT_GAMMA 0
#endif

// whether the colour written to the window is encoded by the shader (FBOUT): the window has no srgb format (gfx.shadergamma yes).
// a specialization constant, set by the renderer when it makes the pipeline
layout(constant_id = SPEC_OUTPUT_GAMMA) const bool OUTPUT_GAMMA = false;

// the colour written to the window
vec4 FBOUT(vec4 x)
{
	return (OUTPUT_GAMMA ? vec4(pow(x.rgb, vec3(1.0 / 2.2)), x.a) : x);
}

#endif
