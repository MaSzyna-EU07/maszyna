/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the gpu the renderer runs on, and the VkDevice with its queues.
//
// the gpu is the one named by gfx.vulkan.device (its index in the list of the log, or a part of its name), or else the best
// usable one: discrete before integrated, then the one with more memory. a gpu missing something the renderer needs is
// turned down with the name of what it lacks

#pragma once

#include "rendering/vulkan14/vulkan14common.h"
#include "rendering/vulkan14/vulkan14capabilities.h"

namespace vk14
{

class instance;

struct queue_families
{
	static constexpr std::uint32_t invalid{VK_QUEUE_FAMILY_IGNORED};

	// graphics and presentation, from one family (every desktop driver has one)
	std::uint32_t graphics{invalid};
	// asynchronous compute and transfers; the graphics family where there's no family of their own
	std::uint32_t compute{invalid};
	std::uint32_t transfer{invalid};
	bool dedicated_compute{false};
	bool dedicated_transfer{false};
};

class device
{
  public:
	device() = default;
	device(device const &) = delete;
	device &operator=(device const &) = delete;
	~device();

	// picks the gpu. Surface: the window to show images in, null before there's one (the presentation is then asked of
	// glfw). Verbose: every gpu and why it can't be used goes to the log. false: no usable gpu, Reason tells why
	bool select(VkInstance Instance, VkSurfaceKHR Surface, std::string const &Wanted, bool const Verbose, std::string &Reason);
	// makes the VkDevice of the gpu picked by select()
	// Wanted: the extensions a library asks for (dlss), taken where the device has them, with the features of vulkan 1.2 it uses
	bool create(instance const &Instance, std::string &Reason, std::vector<std::string> const &Wanted = {});
	void destroy();

	VkPhysicalDevice physical() const
	{
		return m_physical;
	}
	VkDevice handle() const
	{
		return m_device;
	}
	capabilities const &caps() const
	{
		return m_capabilities;
	}
	queue_families const &families() const
	{
		return m_families;
	}
	VkQueue graphics_queue() const
	{
		return m_graphics_queue;
	}
	// the queue of the compute family; the graphics queue where the family is the same
	VkQueue compute_queue() const
	{
		return m_compute_queue;
	}
	// VK_EXT_memory_budget is on (the allocator keeps to the budget of the heaps)
	bool memory_budget_enabled() const
	{
		return m_memory_budget;
	}
	// the ray queries are on with what they need (gfx.raytracing on a device which has them): the device addresses of buffers, the
	// acceleration structures, the arrays of textures indexed in the shaders
	bool raytracing_enabled() const
	{
		return m_raytracing;
	}
	// VK_NV_low_latency2 with VK_KHR_present_id is on (reflex, gfx.reflex)
	bool low_latency_enabled() const
	{
		return m_low_latency;
	}
	// VK_NV_device_diagnostic_checkpoints is on (gfx.vulkan.checkpoints): the passes mark where the gpu got to, for the report of a lost
	// device
	bool checkpoints_enabled() const
	{
		return m_checkpoints;
	}
	// what the driver tells of the fault which lost the device (VK_EXT_device_fault, where the device has it): its description, the
	// addresses the gpu went wrong at and what the vendor adds. empty without the extension
	std::string fault_report() const;
	// VK_EXT_device_fault is on
	bool fault_enabled() const
	{
		return m_device_fault;
	}

  private:
	static queue_families find_queue_families(VkInstance Instance, VkPhysicalDevice Physical, VkSurfaceKHR Surface, std::uint32_t &TimestampBits);
	static std::uint64_t score(capabilities const &Caps);

	VkPhysicalDevice m_physical{VK_NULL_HANDLE};
	VkDevice m_device{VK_NULL_HANDLE};
	capabilities m_capabilities;
	queue_families m_families;
	feature_chain m_queried;
	bool m_portability_subset{false};
	bool m_memory_budget{false};
	bool m_raytracing{false};
	bool m_low_latency{false};
	bool m_checkpoints{false};
	bool m_device_fault{false};

	VkQueue m_graphics_queue{VK_NULL_HANDLE};
	VkQueue m_compute_queue{VK_NULL_HANDLE};
	VkQueue m_transfer_queue{VK_NULL_HANDLE};
};

} // namespace vk14
