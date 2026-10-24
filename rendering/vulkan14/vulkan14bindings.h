/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the bindings of the shaders of the vulkan renderer, in one place for the c++ code and the shaders: the renderer puts them
// before the sources of shaders/vk14 as #define lines (vk14::binding_preamble()), and shaders/vk14/common.glsl writes the
// same values for the SPIR-V binaries, which glslc compiles without the renderer (the renderer checks they are the same).
//
// the descriptor sets (decision K6 of the plan): set 0 holds the resources of a pass, made when the renderer starts and bound
// with a dynamic offset in the ring of the frame; set 1 is pushed with the draws (vkCmdPushDescriptorSet2): the textures of
// the material and what changes between the draws. a layout can have only one set pushed, so the passes without a set of
// their own (the user interface, the compute passes) push set 0

#pragma once

#include <cstdint>
#include <string>

namespace vk14::bindings
{

// the user interface (imgui): one pushed set with the texture, the transform in the push constants
constexpr std::uint32_t UI_SET = 0;
constexpr std::uint32_t UI_TEXTURE = 0;

// the sets of the passes of the scene (from the stage D1 of the plan)
constexpr std::uint32_t PASS_SET = 0;
constexpr std::uint32_t MATERIAL_SET = 1;
// the textures of a material take the bindings of their units (#pragma texture(name, unit, ...)), 0 .. MAX_TEXTURES - 1
constexpr std::uint32_t MATERIAL_TEXTURE_FIRST = 0;

// the specialization constants of the shaders
// 1: the colour written to the window is encoded by the shader (a window without an srgb format, gfx.shadergamma yes)
constexpr std::uint32_t SPEC_OUTPUT_GAMMA = 0;

} // namespace vk14::bindings

namespace vk14
{

// the bindings above as #define lines, with MASZYNA_CONSTANTS, which tells shaders/vk14/common.glsl not to write its own
inline std::string binding_preamble()
{
	auto const define = [](char const *Name, std::uint32_t const Value) { return std::string("#define ") + Name + " " + std::to_string(Value) + "\n"; };
	return define("MASZYNA_CONSTANTS", 1) + define("UI_SET", bindings::UI_SET) + define("UI_TEXTURE", bindings::UI_TEXTURE) + define("PASS_SET", bindings::PASS_SET) + define("MATERIAL_SET", bindings::MATERIAL_SET) +
	       define("MATERIAL_TEXTURE_FIRST", bindings::MATERIAL_TEXTURE_FIRST) + define("SPEC_OUTPUT_GAMMA", bindings::SPEC_OUTPUT_GAMMA);
}

} // namespace vk14
