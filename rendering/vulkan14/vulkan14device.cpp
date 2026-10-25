/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14device.h"
#include "rendering/vulkan14/vulkan14instance.h"
#include "utilities/Globals.h"

#include <cstring>

namespace vk14
{

namespace
{

// "1" -> 1; anything which isn't a plain number -> -1
int index_of(std::string const &Text)
{
	if (Text.empty() || (Text.size() > 3) || (false == std::all_of(Text.begin(), Text.end(), [](char const Char) { return std::isdigit(static_cast<unsigned char>(Char)) != 0; })))
	{
		return -1;
	}
	return std::stoi(Text);
}

std::string lowercase(std::string Text)
{
	std::transform(Text.begin(), Text.end(), Text.begin(), [](unsigned char const Char) { return static_cast<char>(std::tolower(Char)); });
	return Text;
}

} // namespace

device::~device()
{
	destroy();
}

queue_families device::find_queue_families(VkInstance Instance, VkPhysicalDevice Physical, VkSurfaceKHR Surface, std::uint32_t &TimestampBits)
{
	queue_families families;
	TimestampBits = 0;

	std::uint32_t count{0};
	vkGetPhysicalDeviceQueueFamilyProperties(Physical, &count, nullptr);
	std::vector<VkQueueFamilyProperties> properties(count);
	vkGetPhysicalDeviceQueueFamilyProperties(Physical, &count, properties.data());

	for (std::uint32_t i = 0; i < count; ++i)
	{
		if ((properties[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) == 0)
		{
			continue;
		}
		VkBool32 present{VK_FALSE};
		if (Surface != VK_NULL_HANDLE)
		{
			if (!VK14_VERIFY(vkGetPhysicalDeviceSurfaceSupportKHR(Physical, i, Surface, &present)))
			{
				present = VK_FALSE;
			}
		}
		else
		{
			present = (glfwGetPhysicalDevicePresentationSupport(Instance, Physical, i) == GLFW_TRUE ? VK_TRUE : VK_FALSE);
		}
		if (present == VK_TRUE)
		{
			families.graphics = i;
			TimestampBits = properties[i].timestampValidBits;
			break;
		}
	}
	if (families.graphics == queue_families::invalid)
	{
		return families;
	}

	families.compute = families.graphics;
	families.transfer = families.graphics;
	// a compute family without graphics can work alongside the frame
	for (std::uint32_t i = 0; i < count; ++i)
	{
		auto const flags{properties[i].queueFlags};
		if (((flags & VK_QUEUE_COMPUTE_BIT) != 0) && ((flags & VK_QUEUE_GRAPHICS_BIT) == 0))
		{
			families.compute = i;
			families.dedicated_compute = true;
			break;
		}
	}
	// a family of transfers only is usually the copy engine
	for (std::uint32_t i = 0; i < count; ++i)
	{
		auto const flags{properties[i].queueFlags};
		if (((flags & VK_QUEUE_TRANSFER_BIT) != 0) && ((flags & (VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT)) == 0))
		{
			families.transfer = i;
			families.dedicated_transfer = true;
			break;
		}
	}
	return families;
}

std::uint64_t device::score(capabilities const &Caps)
{
	std::uint64_t score{0};
	switch (Caps.device_type)
	{
	case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU:
		score = 3;
		break;
	case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU:
		score = 2;
		break;
	case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU:
		score = 1;
		break;
	default:
		break;
	}
	// then the memory, in mebibytes
	return (score << 40) + (Caps.device_local_memory >> 20);
}

bool device::select(VkInstance Instance, VkSurfaceKHR Surface, std::string const &Wanted, bool const Verbose, std::string &Reason)
{
	m_physical = VK_NULL_HANDLE;

	std::uint32_t count{0};
	if (!VK14_VERIFY(vkEnumeratePhysicalDevices(Instance, &count, nullptr)) || (count == 0))
	{
		Reason = "no vulkan device";
		return false;
	}
	std::vector<VkPhysicalDevice> devices(count);
	if (!VK14_VERIFY(vkEnumeratePhysicalDevices(Instance, &count, devices.data())))
	{
		Reason = "the vulkan devices can't be listed";
		return false;
	}

	auto const wantedindex{index_of(Wanted)};
	auto const wantedname{lowercase(Wanted)};
	if (Verbose)
	{
		log_info(std::to_string(count) + " device(s)");
	}

	VkPhysicalDevice best{VK_NULL_HANDLE};
	VkPhysicalDevice chosen{VK_NULL_HANDLE};
	std::uint64_t bestscore{0};
	bool wantedfound{false};
	std::string firstmissing;

	for (std::uint32_t i = 0; i < count; ++i)
	{
		feature_chain queried;
		capabilities caps;
		query_capabilities(devices[i], queried, caps);
		std::uint32_t timestampbits{0};
		auto const families{find_queue_families(Instance, devices[i], Surface, timestampbits)};
		if (families.graphics == queue_families::invalid)
		{
			caps.missing.push_back("no queue family both draws and shows images in a window");
		}
		else if (timestampbits == 0)
		{
			caps.missing.push_back("the graphics queue has no timestamps (needed for the gpu times of the passes)");
		}

		auto const label{"device " + std::to_string(i) + " \"" + caps.device_name + "\" (" + device_type_string(caps.device_type) + ", vulkan " + version_string(caps.api_version) + ")"};
		auto const iswanted{(false == Wanted.empty()) && ((wantedindex >= 0) ? (wantedindex == static_cast<int>(i)) : (lowercase(caps.device_name).find(wantedname) != std::string::npos))};
		wantedfound |= iswanted;

		if (false == caps.missing.empty())
		{
			if (Verbose || iswanted)
			{
				log_warning(label + " can't be used:");
				for (auto const &reason : caps.missing)
				{
					log_warning("  " + reason);
				}
			}
			if (firstmissing.empty())
			{
				firstmissing = caps.device_name + ": " + caps.missing.front();
			}
			continue;
		}
		if (Verbose)
		{
			log_info(label + " can be used");
		}
		if (iswanted && (chosen == VK_NULL_HANDLE))
		{
			chosen = devices[i];
		}
		auto const devicescore{score(caps)};
		if ((best == VK_NULL_HANDLE) || (devicescore > bestscore))
		{
			best = devices[i];
			bestscore = devicescore;
		}
	}

	if (Verbose && (false == Wanted.empty()) && (chosen == VK_NULL_HANDLE))
	{
		log_warning("gfx.vulkan.device " + Wanted + (wantedfound ? " can't be used" : " matches no device") + ", picking the device by itself");
	}
	if (chosen == VK_NULL_HANDLE)
	{
		chosen = best;
	}
	if (chosen == VK_NULL_HANDLE)
	{
		Reason = "no gpu has what the vulkan renderer needs (" + firstmissing + ")";
		return false;
	}

	m_physical = chosen;
	query_capabilities(m_physical, m_queried, m_capabilities);
	std::uint32_t timestampbits{0};
	m_families = find_queue_families(Instance, m_physical, Surface, timestampbits);
	return true;
}

bool device::create(instance const &Instance, std::string &Reason)
{
	if (m_device != VK_NULL_HANDLE)
	{
		return true;
	}
	if (m_physical == VK_NULL_HANDLE)
	{
		Reason = "no gpu was picked";
		return false;
	}

	feature_chain enabled;
	build_enabled_features(m_queried, enabled);

	std::vector<std::uint32_t> families{m_families.graphics};
	for (auto const family : {m_families.compute, m_families.transfer})
	{
		if (std::find(families.begin(), families.end(), family) == families.end())
		{
			families.push_back(family);
		}
	}
	float const priority{1.f};
	std::vector<VkDeviceQueueCreateInfo> queues;
	for (auto const family : families)
	{
		VkDeviceQueueCreateInfo info{};
		info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
		info.queueFamilyIndex = family;
		info.queueCount = 1;
		info.pQueuePriorities = &priority;
		queues.push_back(info);
	}

	// everything else the renderer uses is in the core of vulkan 1.4
	std::vector<char const *> extensions{VK_KHR_SWAPCHAIN_EXTENSION_NAME};
	{
		// a device of the portability subset (vulkan on metal) has to be told the program knows it
		std::uint32_t count{0};
		vkEnumerateDeviceExtensionProperties(m_physical, nullptr, &count, nullptr);
		std::vector<VkExtensionProperties> available(count);
		vkEnumerateDeviceExtensionProperties(m_physical, nullptr, &count, available.data());
		m_portability_subset = std::any_of(available.begin(), available.end(), [](VkExtensionProperties const &Entry) { return std::strcmp(Entry.extensionName, "VK_KHR_portability_subset") == 0; });
		if (m_portability_subset)
		{
			extensions.push_back("VK_KHR_portability_subset");
		}
		// the allocator keeps to the budget of the memory heaps where the driver tells it
		m_memory_budget = m_capabilities.ext_memory_budget;
		if (m_memory_budget)
		{
			extensions.push_back(VK_EXT_MEMORY_BUDGET_EXTENSION_NAME);
		}
	}

	// the ray traced reflections, asked for in eu07.ini (gfx.raytracing), where the device has what they need
	VkPhysicalDeviceAccelerationStructureFeaturesKHR acceleration{};
	acceleration.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;
	VkPhysicalDeviceRayQueryFeaturesKHR rayquery{};
	rayquery.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR;
	m_raytracing = Global.gfx_raytracing && m_capabilities.ray_query;
	if (m_raytracing)
	{
		extensions.push_back(VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME);
		extensions.push_back(VK_KHR_RAY_QUERY_EXTENSION_NAME);
		extensions.push_back(VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME);
		enabled.v12.bufferDeviceAddress = VK_TRUE;
		enabled.v12.runtimeDescriptorArray = VK_TRUE;
		enabled.v12.descriptorBindingPartiallyBound = VK_TRUE;
		enabled.v12.descriptorBindingVariableDescriptorCount = VK_TRUE;
		enabled.v12.shaderSampledImageArrayNonUniformIndexing = VK_TRUE;
		acceleration.accelerationStructure = VK_TRUE;
		rayquery.rayQuery = VK_TRUE;
		acceleration.pNext = &rayquery;
		enabled.v14.pNext = &acceleration;
	}
	else if (Global.gfx_raytracing)
	{
		log_info("gfx.raytracing: " + m_capabilities.device_name + " has no ray queries (or not what they need), the reflections keep to the screen and the environment map");
	}

	VkDeviceCreateInfo create{};
	create.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
	create.pNext = enabled.head();
	create.queueCreateInfoCount = static_cast<std::uint32_t>(queues.size());
	create.pQueueCreateInfos = queues.data();
	create.enabledExtensionCount = static_cast<std::uint32_t>(extensions.size());
	create.ppEnabledExtensionNames = extensions.data();

	auto const result{vkCreateDevice(m_physical, &create, nullptr, &m_device)};
	if (result != VK_SUCCESS)
	{
		m_device = VK_NULL_HANDLE;
		Reason = std::string("vkCreateDevice failed with ") + result_string(result);
		return false;
	}
	// the functions of the device straight from the driver, without the dispatch of the loader
	volkLoadDevice(m_device);

	vkGetDeviceQueue(m_device, m_families.graphics, 0, &m_graphics_queue);
	vkGetDeviceQueue(m_device, m_families.compute, 0, &m_compute_queue);
	vkGetDeviceQueue(m_device, m_families.transfer, 0, &m_transfer_queue);

	Instance.set_object_name(m_device, VK_OBJECT_TYPE_DEVICE, reinterpret_cast<std::uint64_t>(m_device), "eu07 device");
	Instance.set_object_name(m_device, VK_OBJECT_TYPE_QUEUE, reinterpret_cast<std::uint64_t>(m_graphics_queue), "graphics queue");

	log_info("using " + m_capabilities.device_name + ", queue families: graphics " + std::to_string(m_families.graphics) + ", compute " + std::to_string(m_families.compute) + (m_families.dedicated_compute ? " (own)" : " (shared)") + ", transfer " +
	         std::to_string(m_families.transfer) + (m_families.dedicated_transfer ? " (own)" : " (shared)"));
	if (m_raytracing)
	{
		log_info("ray queries on, for the ray traced reflections (gfx.raytracing)");
	}
	return true;
}

void device::destroy()
{
	if (m_device != VK_NULL_HANDLE)
	{
		vkDestroyDevice(m_device, nullptr);
		m_device = VK_NULL_HANDLE;
	}
	m_graphics_queue = VK_NULL_HANDLE;
	m_compute_queue = VK_NULL_HANDLE;
	m_transfer_queue = VK_NULL_HANDLE;
}

} // namespace vk14
