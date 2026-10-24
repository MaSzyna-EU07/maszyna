/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the vulkan loader, loaded when the program runs, and the VkInstance: vulkan 1.4, the extensions glfw needs to show images
// in a window and, with gfx.vulkan.validation, the khronos validation layer with its messages passed to the log

#pragma once

#include "rendering/vulkan14/vulkan14common.h"

namespace vk14
{

// loads the vulkan loader (once). false: there's no vulkan on the machine, Reason tells what is missing
bool load_vulkan(std::string &Reason);

struct instance_config
{
	// 0: none, 1: VK_LAYER_KHRONOS_validation, 2: also its gpu-assisted validation
	int validation{0};
	// VK_EXT_debug_utils: the messages of the validation layer in the log, names of objects
	bool debug_utils{false};
};

class instance
{
  public:
	instance() = default;
	instance(instance const &) = delete;
	instance &operator=(instance const &) = delete;
	~instance();

	// false: Reason tells why
	bool create(instance_config const &Config, std::string &Reason);
	void destroy();

	VkInstance handle() const
	{
		return m_instance;
	}
	bool validation_enabled() const
	{
		return m_validation_enabled;
	}
	// a name of a vulkan object, for the messages of the validation layer and for captures. nothing without debug utils
	void set_object_name(VkDevice Device, VkObjectType Type, std::uint64_t Handle, std::string const &Name) const;

  private:
	bool collect_layers(std::vector<char const *> &Layers, instance_config const &Config);
	bool collect_extensions(std::vector<char const *> &Extensions, instance_config const &Config, std::string &Reason);

	VkInstance m_instance{VK_NULL_HANDLE};
	VkDebugUtilsMessengerEXT m_messenger{VK_NULL_HANDLE};
	bool m_validation_enabled{false};
	bool m_debug_utils_enabled{false};
};

} // namespace vk14
