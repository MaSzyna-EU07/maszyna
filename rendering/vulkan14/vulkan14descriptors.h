/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the descriptors of the vulkan renderer without pools: the samplers every pass shares, the layouts of the pushed sets, and the
// templates the descriptors are pushed by with the draws (vkCmdPushDescriptorSetWithTemplate2). no descriptor is allocated or
// updated while a frame is made

#pragma once

#include "rendering/vulkan14/vulkan14common.h"

namespace vk14
{

class device;

// the samplers of the renderer, made once
struct samplers
{
	VkSampler repeat_aniso{VK_NULL_HANDLE}; // the textures of the scene: repeated, mipmapped, anisotropic (anisotropicfiltering)
	VkSampler clamp_aniso{VK_NULL_HANDLE}; // the same clamped (textures with the s and t traits)
	VkSampler clamp_linear{VK_NULL_HANDLE}; // images of the user interface, post-processing
	VkSampler clamp_nearest{VK_NULL_HANDLE};
	VkSampler shadow{VK_NULL_HANDLE}; // depth compared (shadow maps, reverse z)

	bool create(device const &Device, float Anisotropy, std::string &Reason);
	void destroy(VkDevice Device);
};

// a layout of a pushed set and its template
struct push_set
{
	VkDescriptorSetLayout layout{VK_NULL_HANDLE};
	VkDescriptorUpdateTemplate update{VK_NULL_HANDLE};
	std::uint32_t set{0};
	VkPipelineBindPoint bind_point{VK_PIPELINE_BIND_POINT_GRAPHICS};

	// Bindings: one descriptor each, the data of the template is an array of VkDescriptorImageInfo / VkDescriptorBufferInfo in
	// the order of the bindings (Stride bytes per binding)
	bool create(VkDevice Device, std::vector<VkDescriptorSetLayoutBinding> const &Bindings, std::uint32_t Set, VkPipelineBindPoint BindPoint, std::size_t Stride, std::string const &Name);
	// the template needs the pipeline layout the set is in
	bool create_template(VkDevice Device, VkPipelineLayout Layout, std::string const &Name);
	void destroy(VkDevice Device);
	// pushes Data (laid out as create() was told)
	void push(VkCommandBuffer Commands, VkPipelineLayout Layout, void const *Data) const;

  private:
	std::vector<VkDescriptorSetLayoutBinding> m_bindings;
	std::size_t m_stride{0};
};

// a pipeline layout of the given sets and push constants
VkPipelineLayout make_pipeline_layout(VkDevice Device, std::vector<VkDescriptorSetLayout> const &Sets, std::vector<VkPushConstantRange> const &PushConstants, std::string const &Name);

} // namespace vk14
