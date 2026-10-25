/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the shaders and the pipelines of the vulkan renderer.
//
// a shader of shaders/vk14 is taken from the SPIR-V binary of shaders/vk14bin when there's an up to date one
// (gfx.shaders.binaries), from the cache of the shaders compiled before (cache/vulkan14/shaders in the directory of the user,
// keyed by the source with its includes, the variant and the version of glslang), or compiled now by glslang and put in the
// cache. the pipelines are made from a description of what isn't dynamic (the shaders, the layout of the vertices, the class
// of the primitives, the blending, the formats of the attachments, the samples); the rest of the state is dynamic. the
// VkPipelineCache is read from and written to the directory of the user, and thrown away when it comes from another device
// or driver. no VkShaderModule: the code goes to the stages of the pipelines (maintenance5)

#pragma once

#include "rendering/vulkan14/vulkan14common.h"
#include "rendering/common/shadersource.h"

#include <memory>
#include <unordered_map>

namespace vk14
{

class device;

// the SPIR-V of a shader, and what its source says (the material directives)
struct shader_code
{
	gfx::shader_source source;
	VkShaderStageFlagBits stage{VK_SHADER_STAGE_VERTEX_BIT};
	std::vector<std::uint32_t> spirv;
	// where it came from: "binary", "cache", "compiled"
	char const *origin{""};
};

class shader_library
{
  public:
	shader_library() = default;
	shader_library(shader_library const &) = delete;
	shader_library &operator=(shader_library const &) = delete;
	~shader_library();

	// Preamble: what the renderer puts after #version (the constants of vulkan14bindings.h)
	bool init(std::string const &Preamble, std::string &Reason);
	void shutdown();
	// the shader of shaders/vk14 (Defines: the variant). throws gfx::shader_exception
	std::shared_ptr<shader_code const> get(std::string const &Name, std::string const &Defines = {});
	// what was loaded from where, for the debug panel
	std::string info() const;

  private:
	std::vector<std::uint32_t> compile(gfx::shader_source const &Source) const;

	std::string m_preamble;
	std::string m_cachedir;
	std::string m_compilerversion;
	bool m_glslang{false};
	bool m_binaries{false};
	std::unordered_map<std::string, std::shared_ptr<shader_code const>> m_shaders;
	int m_frombinary{0};
	int m_fromcache{0};
	int m_compiled{0};
};

// how the colour of a pipeline is mixed with what is there
enum class blend_mode
{
	none,
	alpha, // the colour over what is there by its alpha, the alpha kept for what's behind (the user interface)
	premultiplied,
	additive,
	// the blending of the scene, as glBlendFunc() of the opengl renderer gives it to the colour and the alpha alike
	scene_alpha, // GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA
	scene_additive, // GL_SRC_ALPHA, GL_ONE (the glare of the lamps)
	under_transmittance, // GL_ONE, GL_SRC_ALPHA: the colour added, what is there kept by the alpha (the clouds and the fog over the sky)
	multiply, // GL_ZERO, GL_SRC_COLOR: what is there times the colour (the ambient occlusion)
	no_colour // the colour isn't written, only the depth (the depth prepass of the instances inside the colour pass)
};

struct vertex_layout
{
	std::vector<VkVertexInputBindingDescription> bindings;
	std::vector<VkVertexInputAttributeDescription> attributes;
};

// what a pipeline is made of; the rest of the state is set while drawing (vk14::set_draw_state())
struct pipeline_desc
{
	std::vector<std::shared_ptr<shader_code const>> stages;
	VkPipelineLayout layout{VK_NULL_HANDLE};
	vertex_layout vertices;
	// the class of the primitives; the primitive itself is dynamic
	VkPrimitiveTopology topology{VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST};
	blend_mode blending{blend_mode::none};
	std::vector<VkFormat> colour_formats;
	VkFormat depth_format{VK_FORMAT_UNDEFINED};
	VkSampleCountFlagBits samples{VK_SAMPLE_COUNT_1_BIT};
	// the views drawn at once (multiview, the cube shadow maps of the lamps), 0: none
	std::uint32_t view_mask{0};
	// the values of the specialization constants 0, 1, 2 ... of all the stages
	std::vector<std::uint32_t> specialization;
	std::string name;
};

class pipeline_library
{
  public:
	pipeline_library() = default;
	pipeline_library(pipeline_library const &) = delete;
	pipeline_library &operator=(pipeline_library const &) = delete;
	~pipeline_library();

	bool init(device const &Device, std::string &Reason);
	// writes the pipeline cache to the disk and destroys the pipelines
	void shutdown();
	// the pipeline of the description, made at the first call (from the cache on the disk, when it has it). null when it can't be made
	VkPipeline get(pipeline_desc const &Desc);
	VkPipeline compute(std::shared_ptr<shader_code const> const &Shader, VkPipelineLayout Layout, std::vector<std::uint32_t> const &Specialization = {}, std::string const &Name = {});
	std::string info() const;

  private:
	std::uint64_t key_of(pipeline_desc const &Desc) const;

	device const *m_device{nullptr};
	VkPipelineCache m_cache{VK_NULL_HANDLE};
	std::string m_cachepath;
	std::unordered_map<std::uint64_t, VkPipeline> m_pipelines;
	double m_creationms{0.0};
};

// the dynamic state of a pipeline in its plain form: the whole of Extent, no culling, no depth, triangles
void set_draw_state(command_stream *Commands, VkExtent2D Extent);

} // namespace vk14
