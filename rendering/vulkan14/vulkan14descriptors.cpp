/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14descriptors.h"
#include "rendering/vulkan14/vulkan14device.h"

namespace vk14
{

// samplers

bool samplers::create(device const &Device, float Anisotropy, std::string &Reason)
{
	auto const handle{Device.handle()};
	auto const maxanisotropy{Device.caps().limits.maxSamplerAnisotropy};
	auto const make = [&](VkSampler &Sampler, VkFilter Filter, VkSamplerAddressMode Address, bool Mipmaps, float Aniso, bool Compare, char const *Name) {
		VkSamplerCreateInfo info{};
		info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
		info.magFilter = Filter;
		info.minFilter = Filter;
		info.mipmapMode = (Filter == VK_FILTER_LINEAR ? VK_SAMPLER_MIPMAP_MODE_LINEAR : VK_SAMPLER_MIPMAP_MODE_NEAREST);
		info.addressModeU = Address;
		info.addressModeV = Address;
		info.addressModeW = Address;
		info.maxLod = (Mipmaps ? VK_LOD_CLAMP_NONE : 0.f);
		if (Aniso > 1.f)
		{
			info.anisotropyEnable = VK_TRUE;
			info.maxAnisotropy = std::min(Aniso, maxanisotropy);
		}
		if (Compare)
		{
			// as GL_LEQUAL of the opengl renderer: 1 (in the shadow) where the reference isn't nearer the light than the map (reverse
			// z: its depth isn't greater), the shaders take the result as the amount of the shadow
			info.compareEnable = VK_TRUE;
			info.compareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
			info.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK;
		}
		if (!VK14_VERIFY(vkCreateSampler(handle, &info, nullptr, &Sampler)))
		{
			Sampler = VK_NULL_HANDLE;
			Reason = std::string("the sampler ") + Name + " can't be made";
			return false;
		}
		name_object(handle, VK_OBJECT_TYPE_SAMPLER, handle_value(Sampler), Name);
		return true;
	};
	return make(repeat_aniso, VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_REPEAT, true, Anisotropy, false, "repeat aniso") && make(clamp_aniso, VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, true, Anisotropy, false, "clamp aniso") &&
	       make(clamp_linear, VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, true, 1.f, false, "clamp linear") && make(clamp_nearest, VK_FILTER_NEAREST, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, false, 1.f, false, "clamp nearest") &&
	       make(shadow, VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, false, 1.f, true, "shadow");
}

void samplers::destroy(VkDevice Device)
{
	for (auto *sampler : {&repeat_aniso, &clamp_aniso, &clamp_linear, &clamp_nearest, &shadow})
	{
		if (*sampler != VK_NULL_HANDLE)
		{
			vkDestroySampler(Device, *sampler, nullptr);
			*sampler = VK_NULL_HANDLE;
		}
	}
}

// pushed sets

bool push_set::create(VkDevice Device, std::vector<VkDescriptorSetLayoutBinding> const &Bindings, std::uint32_t Set, VkPipelineBindPoint BindPoint, std::size_t Stride, std::string const &Name)
{
	m_bindings = Bindings;
	m_stride = Stride;
	set = Set;
	bind_point = BindPoint;

	VkDescriptorSetLayoutCreateInfo info{};
	info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	info.flags = VK_DESCRIPTOR_SET_LAYOUT_CREATE_PUSH_DESCRIPTOR_BIT;
	info.bindingCount = static_cast<std::uint32_t>(Bindings.size());
	info.pBindings = Bindings.data();
	if (!VK14_VERIFY(vkCreateDescriptorSetLayout(Device, &info, nullptr, &layout)))
	{
		layout = VK_NULL_HANDLE;
		return false;
	}
	name_object(Device, VK_OBJECT_TYPE_DESCRIPTOR_SET_LAYOUT, handle_value(layout), Name);
	return true;
}

bool push_set::create_template(VkDevice Device, VkPipelineLayout Layout, std::string const &Name)
{
	std::vector<VkDescriptorUpdateTemplateEntry> entries;
	for (std::size_t i = 0; i < m_bindings.size(); ++i)
	{
		VkDescriptorUpdateTemplateEntry entry{};
		entry.dstBinding = m_bindings[i].binding;
		entry.dstArrayElement = 0;
		entry.descriptorCount = m_bindings[i].descriptorCount;
		entry.descriptorType = m_bindings[i].descriptorType;
		entry.offset = i * m_stride;
		entry.stride = m_stride;
		entries.push_back(entry);
	}
	VkDescriptorUpdateTemplateCreateInfo info{};
	info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_UPDATE_TEMPLATE_CREATE_INFO;
	info.descriptorUpdateEntryCount = static_cast<std::uint32_t>(entries.size());
	info.pDescriptorUpdateEntries = entries.data();
	info.templateType = VK_DESCRIPTOR_UPDATE_TEMPLATE_TYPE_PUSH_DESCRIPTORS;
	info.descriptorSetLayout = layout;
	info.pipelineBindPoint = bind_point;
	info.pipelineLayout = Layout;
	info.set = set;
	if (!VK14_VERIFY(vkCreateDescriptorUpdateTemplate(Device, &info, nullptr, &update)))
	{
		update = VK_NULL_HANDLE;
		return false;
	}
	name_object(Device, VK_OBJECT_TYPE_DESCRIPTOR_UPDATE_TEMPLATE, handle_value(update), Name);
	return true;
}

void push_set::destroy(VkDevice Device)
{
	if (update != VK_NULL_HANDLE)
	{
		vkDestroyDescriptorUpdateTemplate(Device, update, nullptr);
		update = VK_NULL_HANDLE;
	}
	if (layout != VK_NULL_HANDLE)
	{
		vkDestroyDescriptorSetLayout(Device, layout, nullptr);
		layout = VK_NULL_HANDLE;
	}
}

void push_set::push(VkCommandBuffer Commands, VkPipelineLayout Layout, void const *Data) const
{
	VkPushDescriptorSetWithTemplateInfo info{};
	info.sType = VK_STRUCTURE_TYPE_PUSH_DESCRIPTOR_SET_WITH_TEMPLATE_INFO;
	info.descriptorUpdateTemplate = update;
	info.layout = Layout;
	info.set = set;
	info.pData = Data;
	vkCmdPushDescriptorSetWithTemplate2(Commands, &info);
}

VkPipelineLayout make_pipeline_layout(VkDevice Device, std::vector<VkDescriptorSetLayout> const &Sets, std::vector<VkPushConstantRange> const &PushConstants, std::string const &Name)
{
	VkPipelineLayoutCreateInfo info{};
	info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	info.setLayoutCount = static_cast<std::uint32_t>(Sets.size());
	info.pSetLayouts = Sets.data();
	info.pushConstantRangeCount = static_cast<std::uint32_t>(PushConstants.size());
	info.pPushConstantRanges = PushConstants.data();
	VkPipelineLayout layout{VK_NULL_HANDLE};
	if (!VK14_VERIFY(vkCreatePipelineLayout(Device, &info, nullptr, &layout)))
	{
		return VK_NULL_HANDLE;
	}
	name_object(Device, VK_OBJECT_TYPE_PIPELINE_LAYOUT, handle_value(layout), Name);
	return layout;
}

} // namespace vk14
