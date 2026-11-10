/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// what a gpu can do, and whether the renderer can run on it.
//
// one table of features drives three things: what is asked of the driver, what is switched on when the device is made, and
// what a gpu is turned down for (by the name of the missing feature). the code of the renderer goes by the flags of
// vk14::capabilities, each optional one with a way without it

#pragma once

#include "rendering/vulkan14/vulkan14common.h"

#include <array>

namespace vk14
{

// the structures of the features the renderer cares about, in one object so a single pNext chain can go over them.
// not copyable: the chain points into the object itself
struct feature_chain
{
	VkPhysicalDeviceFeatures2 features2{};
	VkPhysicalDeviceVulkan11Features v11{};
	VkPhysicalDeviceVulkan12Features v12{};
	VkPhysicalDeviceVulkan13Features v13{};
	VkPhysicalDeviceVulkan14Features v14{};

	feature_chain();
	feature_chain(feature_chain const &) = delete;
	feature_chain &operator=(feature_chain const &) = delete;

	// every structure with its sType and no features, linked into the chain
	void reset();

	// head of the chain, for vkGetPhysicalDeviceFeatures2 or VkDeviceCreateInfo::pNext
	VkPhysicalDeviceFeatures2 *head()
	{
		return &features2;
	}
};

struct feature_entry
{
	char const *structure;
	char const *name;
	// the renderer doesn't run on a gpu without it
	bool required;
	// vulkan 1.4 requires it of every device, so a 1.4 device without it is a driver which doesn't conform
	bool guaranteed;
	// what it is for, for the log
	char const *purpose;
	VkBool32 &(*field)(feature_chain &);
};

std::vector<feature_entry> const &feature_table();

struct capabilities
{
	// identification
	std::string device_name;
	std::string driver_name;
	std::string driver_info;
	VkPhysicalDeviceType device_type{VK_PHYSICAL_DEVICE_TYPE_OTHER};
	std::uint32_t api_version{0};
	std::uint32_t driver_version{0};
	std::uint32_t vendor_id{0};
	std::uint32_t device_id{0};
	VkDeviceSize device_local_memory{0};
	float timestamp_period{0.f}; // nanoseconds per tick
	// the limits of the device, the identity of its pipeline cache (VkPipelineCache data of another device or driver is
	// thrown away) and the bindings a push descriptor set can have
	VkPhysicalDeviceLimits limits{};
	std::array<std::uint8_t, VK_UUID_SIZE> pipeline_cache_uuid{};
	std::uint32_t max_push_descriptors{0};

	// device extensions
	bool ext_swapchain{false};
	bool ext_memory_budget{false};
	bool ext_swapchain_maintenance1{false}; // the khr one, or the ext one it came from
	bool ext_extended_dynamic_state3{false};

	// optional features, each with a way without it
	bool draw_indirect_count{false};
	bool host_image_copy{false};
	// the host fills images straight to VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL (one of the copy layouts of 1.4)
	bool host_copy_sampled{false};
	bool fill_mode_non_solid{false};
	bool pipeline_statistics_query{false};
	bool sampler_filter_minmax{false};
	bool shader_output_layer{false};
	// ray queries in the shaders with what they need (VK_KHR_acceleration_structure, VK_KHR_ray_query, VK_KHR_deferred_host_operations,
	// the device addresses of buffers and arrays of textures indexed in the shaders), for the ray traced reflections (gfx.raytracing).
	// the alignment of the scratch memory of the builds of acceleration structures, and the most textures the shading may index
	bool ray_query{false};
	std::uint32_t scratch_alignment{256};
	std::uint32_t max_indexed_textures{0};

	// empty when the renderer can run on the device, otherwise one line per missing thing
	std::vector<std::string> missing;
};

// the features, properties and extensions of a gpu. Queried gets what the device reports; the device is made with the
// features switched on from it
void query_capabilities(VkPhysicalDevice Physical, feature_chain &Queried, capabilities &Caps);

// the required features and the optional ones the device has, switched on
void build_enabled_features(feature_chain &Queried, feature_chain &Enabled);

// the report for the log, several lines
std::string capability_report(capabilities const &Caps);

// the kind of the gpu and its maker, for the log
char const *device_type_string(VkPhysicalDeviceType Type);
std::string vendor_string(std::uint32_t VendorId);

} // namespace vk14
