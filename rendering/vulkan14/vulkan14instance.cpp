/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14instance.h"

#include <cstring>

namespace vk14
{

namespace
{

char const *const validation_layer_name = "VK_LAYER_KHRONOS_validation";

bool contains(std::vector<char const *> const &List, char const *Name)
{
	return std::any_of(List.begin(), List.end(), [Name](char const *Entry) { return std::strcmp(Entry, Name) == 0; });
}

std::vector<VkExtensionProperties> instance_extensions(char const *Layer)
{
	std::uint32_t count{0};
	std::vector<VkExtensionProperties> extensions;
	if (VK14_VERIFY(vkEnumerateInstanceExtensionProperties(Layer, &count, nullptr)) && (count > 0))
	{
		extensions.resize(count);
		if (!VK14_VERIFY(vkEnumerateInstanceExtensionProperties(Layer, &count, extensions.data())))
		{
			extensions.clear();
		}
	}
	return extensions;
}

bool has_extension(std::vector<VkExtensionProperties> const &Extensions, char const *Name)
{
	return std::any_of(Extensions.begin(), Extensions.end(), [Name](VkExtensionProperties const &Entry) { return std::strcmp(Entry.extensionName, Name) == 0; });
}

VKAPI_ATTR VkBool32 VKAPI_CALL debug_callback(VkDebugUtilsMessageSeverityFlagBitsEXT Severity, VkDebugUtilsMessageTypeFlagsEXT Types, VkDebugUtilsMessengerCallbackDataEXT const *Data, void * /*UserData*/)
{
	std::string message{(Types & VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT) != 0 ? "validation" : (Types & VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT) != 0 ? "performance" : "general"};
	if ((Data != nullptr) && (Data->pMessageIdName != nullptr))
	{
		message += " [";
		message += Data->pMessageIdName;
		message += "]";
	}
	message += ": ";
	message += ((Data != nullptr) && (Data->pMessage != nullptr)) ? Data->pMessage : "<no message>";

	if ((Severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) != 0)
	{
		log_error(message);
	}
	else if ((Severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) != 0)
	{
		log_warning(message);
	}
	else
	{
		log_info(message);
	}
	// anything else is for the development of layers
	return VK_FALSE;
}

void fill_messenger_info(VkDebugUtilsMessengerCreateInfoEXT &Info)
{
	Info = {};
	Info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
	Info.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
	Info.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
	Info.pfnUserCallback = &debug_callback;
}

} // namespace

bool load_vulkan(std::string &Reason)
{
	// the loader is loaded once for the run; volk keeps it
	static VkResult const loaded{volkInitialize()};
	if (loaded != VK_SUCCESS)
	{
		Reason = "no vulkan loader (vulkan-1.dll / libvulkan.so.1) on the machine; the graphics driver has no vulkan";
		return false;
	}
	if (glfwVulkanSupported() == GLFW_FALSE)
	{
		Reason = "glfw finds no vulkan loader with the extensions to show images in a window";
		return false;
	}

	auto const loaderversion{volkGetInstanceVersion()};
	// an instance of vulkan 1.4 needs a loader of 1.1 or newer; the device has to give 1.4 itself
	if (loaderversion < VK_API_VERSION_1_1)
	{
		Reason = "the vulkan loader is of version " + version_string(loaderversion) + ", update the graphics driver";
		return false;
	}
	return true;
}

instance::~instance()
{
	destroy();
}

bool instance::create(instance_config const &Config, std::string &Reason)
{
	if (m_instance != VK_NULL_HANDLE)
	{
		return true;
	}
	if (false == load_vulkan(Reason))
	{
		return false;
	}

	std::vector<char const *> layers;
	std::vector<char const *> extensions;
	if (false == collect_layers(layers, Config))
	{
		Reason = "the validation layer can't be listed";
		return false;
	}
	if (false == collect_extensions(extensions, Config, Reason))
	{
		return false;
	}

	VkApplicationInfo application{};
	application.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
	application.pApplicationName = "MaSzyna";
	application.applicationVersion = VK_MAKE_API_VERSION(0, 1, 0, 0);
	application.pEngineName = "MaSzyna vulkan14";
	application.engineVersion = VK_MAKE_API_VERSION(0, 1, 0, 0);
	application.apiVersion = VK_API_VERSION_1_4;

	VkInstanceCreateInfo create{};
	create.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
	create.pApplicationInfo = &application;
#ifdef __APPLE__
	// vulkan on metal (moltenvk, kosmickrisp) is listed only to the programs which ask for the devices of the portability subset
	if (contains(extensions, VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME))
	{
		create.flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
	}
#endif
	create.enabledLayerCount = static_cast<std::uint32_t>(layers.size());
	create.ppEnabledLayerNames = layers.empty() ? nullptr : layers.data();
	create.enabledExtensionCount = static_cast<std::uint32_t>(extensions.size());
	create.ppEnabledExtensionNames = extensions.empty() ? nullptr : extensions.data();

	void const **tail{&create.pNext};
	// in the chain the messenger sees the creation and the destruction of the instance too
	VkDebugUtilsMessengerCreateInfoEXT messenger{};
	if (m_debug_utils_enabled)
	{
		fill_messenger_info(messenger);
		*tail = &messenger;
		tail = &messenger.pNext;
	}
	// synchronization validation checks the barriers; the gpu-assisted one the accesses of the shaders
	std::vector<VkValidationFeatureEnableEXT> validationfeatures;
	VkValidationFeaturesEXT validation{};
	if (m_validation_enabled && contains(extensions, VK_EXT_VALIDATION_FEATURES_EXTENSION_NAME))
	{
		validationfeatures.push_back(VK_VALIDATION_FEATURE_ENABLE_SYNCHRONIZATION_VALIDATION_EXT);
		if (Config.validation >= 2)
		{
			validationfeatures.push_back(VK_VALIDATION_FEATURE_ENABLE_GPU_ASSISTED_EXT);
		}
		validation.sType = VK_STRUCTURE_TYPE_VALIDATION_FEATURES_EXT;
		validation.enabledValidationFeatureCount = static_cast<std::uint32_t>(validationfeatures.size());
		validation.pEnabledValidationFeatures = validationfeatures.data();
		*tail = &validation;
		tail = &validation.pNext;
	}

	auto const result{vkCreateInstance(&create, nullptr, &m_instance)};
	if (result != VK_SUCCESS)
	{
		m_instance = VK_NULL_HANDLE;
		Reason = (result == VK_ERROR_INCOMPATIBLE_DRIVER ? std::string("no installed driver gives vulkan 1.4 (vkCreateInstance: VK_ERROR_INCOMPATIBLE_DRIVER)") : std::string("vkCreateInstance failed with ") + result_string(result));
		return false;
	}
	// the functions of the instance; those of the device come from it directly, once it's made
	volkLoadInstanceOnly(m_instance);

	if (m_debug_utils_enabled)
	{
		VkDebugUtilsMessengerCreateInfoEXT info{};
		fill_messenger_info(info);
		if ((vkCreateDebugUtilsMessengerEXT == nullptr) || !VK14_VERIFY(vkCreateDebugUtilsMessengerEXT(m_instance, &info, nullptr, &m_messenger)))
		{
			// the renderer works, only the messages of the validation layer don't reach the log
			m_messenger = VK_NULL_HANDLE;
			log_warning("the messages of the validation layer won't reach the log");
		}
	}

	return true;
}

bool instance::collect_layers(std::vector<char const *> &Layers, instance_config const &Config)
{
	Layers.clear();
	m_validation_enabled = false;
	if (Config.validation <= 0)
	{
		return true;
	}

	std::uint32_t count{0};
	if (!VK14_VERIFY(vkEnumerateInstanceLayerProperties(&count, nullptr)))
	{
		return false;
	}
	std::vector<VkLayerProperties> available(count);
	if ((count > 0) && !VK14_VERIFY(vkEnumerateInstanceLayerProperties(&count, available.data())))
	{
		return false;
	}

	if (std::any_of(available.begin(), available.end(), [](VkLayerProperties const &Layer) { return std::strcmp(Layer.layerName, validation_layer_name) == 0; }))
	{
		Layers.push_back(validation_layer_name);
		m_validation_enabled = true;
		log_info(std::string("validation layer on") + (Config.validation >= 2 ? " (with gpu-assisted validation)" : ""));
	}
	else
	{
		// gfx.vulkan.validation doesn't stop the renderer on a machine without the vulkan sdk
		log_warning(std::string(validation_layer_name) + " isn't installed (vulkan sdk), running without validation");
	}
	return true;
}

bool instance::collect_extensions(std::vector<char const *> &Extensions, instance_config const &Config, std::string &Reason)
{
	Extensions.clear();
	m_debug_utils_enabled = false;

	std::uint32_t glfwcount{0};
	auto const **glfwextensions{glfwGetRequiredInstanceExtensions(&glfwcount)};
	if ((glfwextensions == nullptr) || (glfwcount == 0))
	{
		Reason = "no vulkan driver gives the extensions to show images in a window";
		return false;
	}
	for (std::uint32_t i = 0; i < glfwcount; ++i)
	{
		Extensions.push_back(glfwextensions[i]);
	}

	auto const available{instance_extensions(nullptr)};
	for (auto const *name : Extensions)
	{
		if (false == has_extension(available, name))
		{
			Reason = std::string("the instance extension ") + name + " to show images in a window isn't available";
			return false;
		}
	}

	if (Config.debug_utils || m_validation_enabled)
	{
		if (has_extension(available, VK_EXT_DEBUG_UTILS_EXTENSION_NAME))
		{
			if (false == contains(Extensions, VK_EXT_DEBUG_UTILS_EXTENSION_NAME))
			{
				Extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
			}
			m_debug_utils_enabled = true;
		}
		else
		{
			log_warning(std::string(VK_EXT_DEBUG_UTILS_EXTENSION_NAME) + " isn't available, no names of objects and no messages of the validation layer");
		}
	}
#ifdef __APPLE__
	if (has_extension(available, VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME))
	{
		Extensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
	}
#endif
	if (m_validation_enabled && has_extension(instance_extensions(validation_layer_name), VK_EXT_VALIDATION_FEATURES_EXTENSION_NAME))
	{
		Extensions.push_back(VK_EXT_VALIDATION_FEATURES_EXTENSION_NAME);
	}

	std::string list;
	for (auto const *name : Extensions)
	{
		list += (list.empty() ? "" : ", ");
		list += name;
	}
	log_info("instance extensions: " + list);

	return true;
}

void instance::set_object_name(VkDevice Device, VkObjectType Type, std::uint64_t Handle, std::string const &Name) const
{
	if ((false == m_debug_utils_enabled) || (vkSetDebugUtilsObjectNameEXT == nullptr) || (Device == VK_NULL_HANDLE) || (Handle == 0))
	{
		return;
	}
	VkDebugUtilsObjectNameInfoEXT info{};
	info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_OBJECT_NAME_INFO_EXT;
	info.objectType = Type;
	info.objectHandle = Handle;
	info.pObjectName = Name.c_str();
	vkSetDebugUtilsObjectNameEXT(Device, &info);
}

void instance::destroy()
{
	if ((m_messenger != VK_NULL_HANDLE) && (vkDestroyDebugUtilsMessengerEXT != nullptr))
	{
		vkDestroyDebugUtilsMessengerEXT(m_instance, m_messenger, nullptr);
	}
	m_messenger = VK_NULL_HANDLE;
	if (m_instance != VK_NULL_HANDLE)
	{
		vkDestroyInstance(m_instance, nullptr);
		m_instance = VK_NULL_HANDLE;
	}
	m_validation_enabled = false;
	m_debug_utils_enabled = false;
}

} // namespace vk14
