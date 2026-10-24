/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14common.h"
#include "utilities/Logs.h"

namespace vk14
{

char const *result_string(VkResult Result)
{
	switch (Result)
	{
	case VK_SUCCESS:
		return "VK_SUCCESS";
	case VK_NOT_READY:
		return "VK_NOT_READY";
	case VK_TIMEOUT:
		return "VK_TIMEOUT";
	case VK_EVENT_SET:
		return "VK_EVENT_SET";
	case VK_EVENT_RESET:
		return "VK_EVENT_RESET";
	case VK_INCOMPLETE:
		return "VK_INCOMPLETE";
	case VK_ERROR_OUT_OF_HOST_MEMORY:
		return "VK_ERROR_OUT_OF_HOST_MEMORY";
	case VK_ERROR_OUT_OF_DEVICE_MEMORY:
		return "VK_ERROR_OUT_OF_DEVICE_MEMORY";
	case VK_ERROR_INITIALIZATION_FAILED:
		return "VK_ERROR_INITIALIZATION_FAILED";
	case VK_ERROR_DEVICE_LOST:
		return "VK_ERROR_DEVICE_LOST";
	case VK_ERROR_MEMORY_MAP_FAILED:
		return "VK_ERROR_MEMORY_MAP_FAILED";
	case VK_ERROR_LAYER_NOT_PRESENT:
		return "VK_ERROR_LAYER_NOT_PRESENT";
	case VK_ERROR_EXTENSION_NOT_PRESENT:
		return "VK_ERROR_EXTENSION_NOT_PRESENT";
	case VK_ERROR_FEATURE_NOT_PRESENT:
		return "VK_ERROR_FEATURE_NOT_PRESENT";
	case VK_ERROR_INCOMPATIBLE_DRIVER:
		return "VK_ERROR_INCOMPATIBLE_DRIVER";
	case VK_ERROR_TOO_MANY_OBJECTS:
		return "VK_ERROR_TOO_MANY_OBJECTS";
	case VK_ERROR_FORMAT_NOT_SUPPORTED:
		return "VK_ERROR_FORMAT_NOT_SUPPORTED";
	case VK_ERROR_FRAGMENTED_POOL:
		return "VK_ERROR_FRAGMENTED_POOL";
	case VK_ERROR_UNKNOWN:
		return "VK_ERROR_UNKNOWN";
	case VK_ERROR_OUT_OF_POOL_MEMORY:
		return "VK_ERROR_OUT_OF_POOL_MEMORY";
	case VK_ERROR_INVALID_EXTERNAL_HANDLE:
		return "VK_ERROR_INVALID_EXTERNAL_HANDLE";
	case VK_ERROR_FRAGMENTATION:
		return "VK_ERROR_FRAGMENTATION";
	case VK_ERROR_INVALID_OPAQUE_CAPTURE_ADDRESS:
		return "VK_ERROR_INVALID_OPAQUE_CAPTURE_ADDRESS";
	case VK_ERROR_SURFACE_LOST_KHR:
		return "VK_ERROR_SURFACE_LOST_KHR";
	case VK_ERROR_NATIVE_WINDOW_IN_USE_KHR:
		return "VK_ERROR_NATIVE_WINDOW_IN_USE_KHR";
	case VK_SUBOPTIMAL_KHR:
		return "VK_SUBOPTIMAL_KHR";
	case VK_ERROR_OUT_OF_DATE_KHR:
		return "VK_ERROR_OUT_OF_DATE_KHR";
	case VK_ERROR_INCOMPATIBLE_DISPLAY_KHR:
		return "VK_ERROR_INCOMPATIBLE_DISPLAY_KHR";
	case VK_ERROR_VALIDATION_FAILED_EXT:
		return "VK_ERROR_VALIDATION_FAILED_EXT";
	case VK_ERROR_FULL_SCREEN_EXCLUSIVE_MODE_LOST_EXT:
		return "VK_ERROR_FULL_SCREEN_EXCLUSIVE_MODE_LOST_EXT";
	default:
		break;
	}
	return "VK_ERROR_<unrecognized>";
}

std::string version_string(std::uint32_t Version)
{
	return std::to_string(VK_API_VERSION_MAJOR(Version)) + "." + std::to_string(VK_API_VERSION_MINOR(Version)) + "." + std::to_string(VK_API_VERSION_PATCH(Version));
}

std::string memory_size_string(VkDeviceSize Bytes)
{
	auto const mib{static_cast<double>(Bytes) / (1024.0 * 1024.0)};

	char buffer[64] = {};
	if (mib >= 1024.0)
	{
		std::snprintf(buffer, sizeof(buffer), "%.2f GiB", mib / 1024.0);
	}
	else
	{
		std::snprintf(buffer, sizeof(buffer), "%.0f MiB", mib);
	}
	return std::string(buffer);
}

bool verify_result(VkResult Result, char const *Call, char const *File, int Line)
{
	if (Result == VK_SUCCESS)
	{
		return true;
	}

	// only the name of the file, the path of the build is noise in the log
	char const *name{File};
	for (char const *c = File; *c != '\0'; ++c)
	{
		if ((*c == '/') || (*c == '\\'))
		{
			name = c + 1;
		}
	}

	log_error(std::string(Call) + " failed with " + result_string(Result) + " (" + name + ":" + std::to_string(Line) + ")");

	return false;
}

void log_info(std::string const &Message)
{
	WriteLog("vulkan: " + Message);
}

void log_warning(std::string const &Message)
{
	WriteLog("vulkan: warning: " + Message);
}

void log_error(std::string const &Message)
{
	ErrorLog("vulkan: " + Message);
}

} // namespace vk14
