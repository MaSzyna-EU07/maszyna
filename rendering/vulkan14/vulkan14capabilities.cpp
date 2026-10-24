/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14capabilities.h"

#include <cstring>

namespace vk14
{

namespace
{

// one VkBool32 in the chain; without captures, so it turns into a plain function pointer and the table stays constant
#define VK14_FIELD(member, field) [](feature_chain &chain) -> VkBool32 & { return chain.member.field; }
#define VK14_CORE(field) VK14_FIELD(features2.features, field)

// columns: structure, name, required, guaranteed by vulkan 1.4, purpose.
//
// beyond what vulkan 1.4 requires of every device the renderer needs multiDrawIndirect, shaderDrawParameters,
// textureCompressionBC and geometryShader, so a gpu without one of them is turned down by its name rather than failing later.
// "guaranteed" follows the minimum requirements of vulkan 1.2-1.4 (VP_LUNARG_minimum_requirements_1_4 of the vulkan sdk)
std::vector<feature_entry> const g_feature_table = {
    // VkPhysicalDeviceFeatures (1.0)
    {"VkPhysicalDeviceFeatures", "geometryShader", true, false, "map icons and cube shadow maps of the lamps until multiview takes them over", VK14_CORE(geometryShader)},
    {"VkPhysicalDeviceFeatures", "multiDrawIndirect", true, false, "multi draw indirect of the gpu culling", VK14_CORE(multiDrawIndirect)},
    {"VkPhysicalDeviceFeatures", "textureCompressionBC", true, false, "bc1-bc7 compressed dds textures", VK14_CORE(textureCompressionBC)},
    {"VkPhysicalDeviceFeatures", "drawIndirectFirstInstance", true, true, "instanced indirect draws", VK14_CORE(drawIndirectFirstInstance)},
    {"VkPhysicalDeviceFeatures", "fullDrawIndexUint32", true, true, "32 bit index buffers", VK14_CORE(fullDrawIndexUint32)},
    {"VkPhysicalDeviceFeatures", "imageCubeArray", true, true, "environment map and cube shadow maps", VK14_CORE(imageCubeArray)},
    {"VkPhysicalDeviceFeatures", "independentBlend", true, true, "blending per attachment", VK14_CORE(independentBlend)},
    {"VkPhysicalDeviceFeatures", "sampleRateShading", true, true, "multisampling", VK14_CORE(sampleRateShading)},
    {"VkPhysicalDeviceFeatures", "depthClamp", true, true, "shadow maps", VK14_CORE(depthClamp)},
    {"VkPhysicalDeviceFeatures", "depthBiasClamp", true, true, "shadow maps", VK14_CORE(depthBiasClamp)},
    {"VkPhysicalDeviceFeatures", "samplerAnisotropy", true, true, "anisotropic texture filtering", VK14_CORE(samplerAnisotropy)},
    {"VkPhysicalDeviceFeatures", "largePoints", true, true, "points larger than a pixel", VK14_CORE(largePoints)},
    {"VkPhysicalDeviceFeatures", "fragmentStoresAndAtomics", true, true, "clustered lights and post-processing", VK14_CORE(fragmentStoresAndAtomics)},
    {"VkPhysicalDeviceFeatures", "shaderImageGatherExtended", true, true, "shadow filtering", VK14_CORE(shaderImageGatherExtended)},
    {"VkPhysicalDeviceFeatures", "shaderStorageImageExtendedFormats", true, true, "compute passes", VK14_CORE(shaderStorageImageExtendedFormats)},
    {"VkPhysicalDeviceFeatures", "fillModeNonSolid", false, false, "wireframe", VK14_CORE(fillModeNonSolid)},
    {"VkPhysicalDeviceFeatures", "pipelineStatisticsQuery", false, false, "draw statistics", VK14_CORE(pipelineStatisticsQuery)},

    // VkPhysicalDeviceVulkan11Features
    {"VkPhysicalDeviceVulkan11Features", "shaderDrawParameters", true, false, "gl_DrawID and gl_BaseInstance of multi draws", VK14_FIELD(v11, shaderDrawParameters)},
    {"VkPhysicalDeviceVulkan11Features", "multiview", true, true, "cube shadow maps of the lamps in one pass", VK14_FIELD(v11, multiview)},

    // VkPhysicalDeviceVulkan12Features
    {"VkPhysicalDeviceVulkan12Features", "timelineSemaphore", true, true, "synchronization of the frames", VK14_FIELD(v12, timelineSemaphore)},
    {"VkPhysicalDeviceVulkan12Features", "scalarBlockLayout", true, true, "buffer layouts shared with the c++ code", VK14_FIELD(v12, scalarBlockLayout)},
    {"VkPhysicalDeviceVulkan12Features", "uniformBufferStandardLayout", true, true, "std430 uniform buffers", VK14_FIELD(v12, uniformBufferStandardLayout)},
    {"VkPhysicalDeviceVulkan12Features", "hostQueryReset", true, true, "gpu times of the passes", VK14_FIELD(v12, hostQueryReset)},
    {"VkPhysicalDeviceVulkan12Features", "separateDepthStencilLayouts", true, true, "depth read while the stencil is written", VK14_FIELD(v12, separateDepthStencilLayouts)},
    {"VkPhysicalDeviceVulkan12Features", "drawIndirectCount", false, false, "draw counts made on the gpu", VK14_FIELD(v12, drawIndirectCount)},
    {"VkPhysicalDeviceVulkan12Features", "samplerFilterMinmax", false, false, "min/max reduction of the depth pyramid", VK14_FIELD(v12, samplerFilterMinmax)},
    {"VkPhysicalDeviceVulkan12Features", "shaderOutputLayer", false, false, "layered rendering without a geometry shader", VK14_FIELD(v12, shaderOutputLayer)},

    // VkPhysicalDeviceVulkan13Features
    {"VkPhysicalDeviceVulkan13Features", "dynamicRendering", true, true, "passes without VkRenderPass and VkFramebuffer", VK14_FIELD(v13, dynamicRendering)},
    {"VkPhysicalDeviceVulkan13Features", "synchronization2", true, true, "barriers and submits", VK14_FIELD(v13, synchronization2)},
    {"VkPhysicalDeviceVulkan13Features", "maintenance4", true, true, "pipeline layouts and buffer sizes", VK14_FIELD(v13, maintenance4)},
    {"VkPhysicalDeviceVulkan13Features", "shaderDemoteToHelperInvocation", true, true, "alpha tested geometry", VK14_FIELD(v13, shaderDemoteToHelperInvocation)},
    {"VkPhysicalDeviceVulkan13Features", "pipelineCreationCacheControl", true, true, "pipelines made on worker threads", VK14_FIELD(v13, pipelineCreationCacheControl)},

    // VkPhysicalDeviceVulkan14Features
    {"VkPhysicalDeviceVulkan14Features", "pushDescriptor", true, true, "descriptors of materials and compute passes without pools", VK14_FIELD(v14, pushDescriptor)},
    {"VkPhysicalDeviceVulkan14Features", "maintenance5", true, true, "pipelines without shader modules", VK14_FIELD(v14, maintenance5)},
    {"VkPhysicalDeviceVulkan14Features", "maintenance6", true, true, "descriptor and push constant calls of 1.4", VK14_FIELD(v14, maintenance6)},
    {"VkPhysicalDeviceVulkan14Features", "hostImageCopy", false, false, "texture uploads without command buffers", VK14_FIELD(v14, hostImageCopy)},
};

#undef VK14_CORE
#undef VK14_FIELD

} // namespace

char const *device_type_string(VkPhysicalDeviceType Type)
{
	switch (Type)
	{
	case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU:
		return "integrated gpu";
	case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU:
		return "discrete gpu";
	case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU:
		return "virtual gpu";
	case VK_PHYSICAL_DEVICE_TYPE_CPU:
		return "cpu";
	default:
		break;
	}
	return "other";
}

std::string vendor_string(std::uint32_t VendorId)
{
	switch (VendorId)
	{
	case 0x1002:
		return "AMD";
	case 0x106B:
		return "Apple";
	case 0x10DE:
		return "NVIDIA";
	case 0x13B5:
		return "ARM";
	case 0x1414:
		return "Microsoft";
	case 0x5143:
		return "Qualcomm";
	case 0x8086:
		return "Intel";
	case 0x10005:
		return "Mesa";
	default:
		break;
	}
	char buffer[16] = {};
	std::snprintf(buffer, sizeof(buffer), "0x%04X", VendorId);
	return std::string(buffer);
}

feature_chain::feature_chain()
{
	reset();
}

void feature_chain::reset()
{
	features2 = {};
	features2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
	v11 = {};
	v11.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_FEATURES;
	v12 = {};
	v12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
	v13 = {};
	v13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
	v14 = {};
	v14.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_4_FEATURES;

	features2.pNext = &v11;
	v11.pNext = &v12;
	v12.pNext = &v13;
	v13.pNext = &v14;
	v14.pNext = nullptr;
}

std::vector<feature_entry> const &feature_table()
{
	return g_feature_table;
}

void query_capabilities(VkPhysicalDevice Physical, feature_chain &Queried, capabilities &Caps)
{
	Caps = capabilities();

	// extensions
	std::uint32_t extensioncount{0};
	std::vector<VkExtensionProperties> extensions;
	if (VK14_VERIFY(vkEnumerateDeviceExtensionProperties(Physical, nullptr, &extensioncount, nullptr)) && (extensioncount > 0))
	{
		extensions.resize(extensioncount);
		if (!VK14_VERIFY(vkEnumerateDeviceExtensionProperties(Physical, nullptr, &extensioncount, extensions.data())))
		{
			extensions.clear();
		}
	}
	auto const hasextension = [&extensions](char const *Name) {
		return std::any_of(extensions.begin(), extensions.end(), [Name](VkExtensionProperties const &Entry) { return std::strcmp(Entry.extensionName, Name) == 0; });
	};

	Caps.ext_swapchain = hasextension(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
	Caps.ext_memory_budget = hasextension(VK_EXT_MEMORY_BUDGET_EXTENSION_NAME);
	Caps.ext_swapchain_maintenance1 = hasextension(VK_KHR_SWAPCHAIN_MAINTENANCE_1_EXTENSION_NAME) || hasextension(VK_EXT_SWAPCHAIN_MAINTENANCE_1_EXTENSION_NAME);
	Caps.ext_extended_dynamic_state3 = hasextension(VK_EXT_EXTENDED_DYNAMIC_STATE_3_EXTENSION_NAME);

	// properties
	VkPhysicalDeviceVulkan12Properties properties12{};
	properties12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_PROPERTIES;
	VkPhysicalDeviceProperties2 properties{};
	properties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
	properties.pNext = &properties12;
	vkGetPhysicalDeviceProperties2(Physical, &properties);

	Caps.device_name = properties.properties.deviceName;
	Caps.device_type = properties.properties.deviceType;
	Caps.api_version = properties.properties.apiVersion;
	Caps.driver_version = properties.properties.driverVersion;
	Caps.vendor_id = properties.properties.vendorID;
	Caps.device_id = properties.properties.deviceID;
	Caps.driver_name = properties12.driverName;
	Caps.driver_info = properties12.driverInfo;
	Caps.timestamp_period = properties.properties.limits.timestampPeriod;
	Caps.limits = properties.properties.limits;
	std::copy(std::begin(properties.properties.pipelineCacheUUID), std::end(properties.properties.pipelineCacheUUID), Caps.pipeline_cache_uuid.begin());

	VkPhysicalDeviceMemoryProperties memory{};
	vkGetPhysicalDeviceMemoryProperties(Physical, &memory);
	for (std::uint32_t i = 0; i < memory.memoryHeapCount; ++i)
	{
		if ((memory.memoryHeaps[i].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) != 0)
		{
			Caps.device_local_memory += memory.memoryHeaps[i].size;
		}
	}

	// the version first: the structures of 1.4 mean nothing to an older driver
	if (Caps.api_version < VK_API_VERSION_1_4)
	{
		Caps.missing.push_back("the driver gives vulkan " + version_string(Caps.api_version) + ", the renderer needs 1.4");
		return;
	}

	// the properties of 1.4, asked for once the driver is known to have them
	{
		VkPhysicalDeviceVulkan14Properties properties14{};
		properties14.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_4_PROPERTIES;
		VkPhysicalDeviceProperties2 properties2{};
		properties2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
		properties2.pNext = &properties14;
		vkGetPhysicalDeviceProperties2(Physical, &properties2);
		Caps.max_push_descriptors = properties14.maxPushDescriptors;
	}

	// features
	Queried.reset();
	vkGetPhysicalDeviceFeatures2(Physical, Queried.head());

	Caps.draw_indirect_count = (Queried.v12.drawIndirectCount == VK_TRUE);
	Caps.host_image_copy = (Queried.v14.hostImageCopy == VK_TRUE);
	Caps.fill_mode_non_solid = (Queried.features2.features.fillModeNonSolid == VK_TRUE);
	Caps.pipeline_statistics_query = (Queried.features2.features.pipelineStatisticsQuery == VK_TRUE);
	Caps.sampler_filter_minmax = (Queried.v12.samplerFilterMinmax == VK_TRUE);
	Caps.shader_output_layer = (Queried.v12.shaderOutputLayer == VK_TRUE);

	// can the renderer run on it
	if (!Caps.ext_swapchain)
	{
		Caps.missing.push_back(std::string(VK_KHR_SWAPCHAIN_EXTENSION_NAME) + " isn't supported, the device can't show images in a window");
	}
	if (properties.properties.limits.timestampComputeAndGraphics == VK_FALSE)
	{
		Caps.missing.push_back("timestampComputeAndGraphics isn't supported (needed for the gpu times of the passes)");
	}
	for (auto const &entry : g_feature_table)
	{
		if ((false == entry.required) || (entry.field(Queried) == VK_TRUE))
		{
			continue;
		}
		auto message{std::string(entry.structure) + "::" + entry.name + " isn't supported (needed for " + entry.purpose + ")"};
		if (entry.guaranteed)
		{
			message += "; vulkan 1.4 requires it, the driver doesn't conform";
		}
		Caps.missing.push_back(message);
	}
}

void build_enabled_features(feature_chain &Queried, feature_chain &Enabled)
{
	Enabled.reset();
	for (auto const &entry : g_feature_table)
	{
		if (entry.field(Queried) == VK_TRUE)
		{
			entry.field(Enabled) = VK_TRUE;
		}
	}
}

std::string capability_report(capabilities const &Caps)
{
	std::string report{"gpu capabilities\n"};
	auto const line = [&report](std::string const &Text) { report += "  " + Text + "\n"; };
	auto const flag = [](bool const Value) { return Value ? "yes" : "no"; };

	line("device            " + Caps.device_name + " (" + vendor_string(Caps.vendor_id) + ", " + device_type_string(Caps.device_type) + ")");
	line("api               " + version_string(Caps.api_version));
	line("driver            " + Caps.driver_name + " " + Caps.driver_info);
	line("device memory     " + memory_size_string(Caps.device_local_memory));
	line(std::string("draw indirect count     ") + flag(Caps.draw_indirect_count));
	line(std::string("extended dynamic state3 ") + flag(Caps.ext_extended_dynamic_state3));
	line(std::string("memory budget           ") + flag(Caps.ext_memory_budget));
	line(std::string("swapchain maintenance1  ") + flag(Caps.ext_swapchain_maintenance1));
	line(std::string("host image copy         ") + flag(Caps.host_image_copy));
	line(std::string("sampler filter minmax   ") + flag(Caps.sampler_filter_minmax));
	line(std::string("shader output layer     ") + flag(Caps.shader_output_layer));
	line(std::string("fill mode non solid     ") + flag(Caps.fill_mode_non_solid));
	line(std::string("pipeline statistics     ") + flag(Caps.pipeline_statistics_query));

	return report;
}

} // namespace vk14
