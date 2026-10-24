/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the vulkan api of the vulkan 1.4 renderer, for the files of rendering/vulkan14 only: the rest of the program doesn't see
// the vulkan headers. the functions come from volk, which loads the vulkan loader when the program runs, so the program starts
// on a machine without vulkan too (and runs with opengl there)

#pragma once

#include <volk.h>

// glfw declares its vulkan functions only when the vulkan headers come before its own, and stdafx.h brings glfw in first
extern "C" GLFWAPI VkResult glfwCreateWindowSurface(VkInstance instance, GLFWwindow *window, VkAllocationCallbacks const *allocator, VkSurfaceKHR *surface);
extern "C" GLFWAPI int glfwGetPhysicalDevicePresentationSupport(VkInstance instance, VkPhysicalDevice device, std::uint32_t queuefamily);

namespace vk14
{

// human readable VkResult, for the log
char const *result_string(VkResult Result);
// "1.4.309"
std::string version_string(std::uint32_t Version);
// bytes as MiB or GiB
std::string memory_size_string(VkDeviceSize Bytes);
// logs a failed call and gives back false; true for VK_SUCCESS
bool verify_result(VkResult Result, char const *Call, char const *File, int Line);

// the log of the renderer, with the "vulkan: " prefix
void log_info(std::string const &Message);
void log_warning(std::string const &Message);
void log_error(std::string const &Message);

} // namespace vk14

// true when the call succeeded; a failure is logged with the call and the place
#define VK14_VERIFY(expr) (::vk14::verify_result((expr), #expr, __FILE__, __LINE__))
