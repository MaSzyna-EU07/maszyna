/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14swapchain.h"
#include "rendering/vulkan14/vulkan14commands.h"
#include "rendering/vulkan14/vulkan14instance.h"
#include "rendering/vulkan14/vulkan14device.h"
#include "utilities/Globals.h"

namespace vk14
{

namespace
{

char const *present_mode_string(VkPresentModeKHR const Mode)
{
	switch (Mode)
	{
	case VK_PRESENT_MODE_IMMEDIATE_KHR:
		return "immediate";
	case VK_PRESENT_MODE_MAILBOX_KHR:
		return "mailbox";
	case VK_PRESENT_MODE_FIFO_KHR:
		return "fifo";
	case VK_PRESENT_MODE_FIFO_RELAXED_KHR:
		return "fifo relaxed";
	default:
		break;
	}
	return "other";
}

bool is_srgb(VkFormat const Format)
{
	switch (Format)
	{
	case VK_FORMAT_B8G8R8A8_SRGB:
	case VK_FORMAT_R8G8B8A8_SRGB:
	case VK_FORMAT_A8B8G8R8_SRGB_PACK32:
		return true;
	default:
		break;
	}
	return false;
}

// the settings the swapchain follows: vsync, and whether the shaders write gamma themselves (as GLFW_SRGB_CAPABLE in opengl)
bool wanted_vsync()
{
	return Global.VSync;
}

bool wanted_srgb()
{
	return (false == Global.gfx_shadergamma);
}

} // namespace

swapchain::~swapchain()
{
	destroy();
}

bool swapchain::create(instance const &Instance, device const &Device, GLFWwindow *Window, std::string const &Name, std::uint32_t const Frames, std::string &Reason)
{
	m_instance = &Instance;
	m_device = &Device;
	m_window = Window;
	m_name = Name;

	if (!VK14_VERIFY(glfwCreateWindowSurface(Instance.handle(), Window, nullptr, &m_surface)))
	{
		m_surface = VK_NULL_HANDLE;
		Reason = "the window of " + Name + " gives no vulkan surface";
		return false;
	}
	Instance.set_object_name(Device.handle(), VK_OBJECT_TYPE_SURFACE_KHR, reinterpret_cast<std::uint64_t>(m_surface), Name + " surface");

	VkBool32 present{VK_FALSE};
	if (!VK14_VERIFY(vkGetPhysicalDeviceSurfaceSupportKHR(Device.physical(), Device.families().graphics, m_surface, &present)) || (present == VK_FALSE))
	{
		Reason = "the gpu can't show images in the window of " + Name;
		return false;
	}

	VkSemaphoreCreateInfo semaphoreinfo{};
	semaphoreinfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
	m_acquiresemaphores.assign(Frames * (1 + m_generatedcapacity), VK_NULL_HANDLE);
	for (std::uint32_t i = 0; i < m_acquiresemaphores.size(); ++i)
	{
		if (!VK14_VERIFY(vkCreateSemaphore(Device.handle(), &semaphoreinfo, nullptr, &m_acquiresemaphores[i])))
		{
			Reason = "no semaphores for " + Name;
			return false;
		}
		Instance.set_object_name(Device.handle(), VK_OBJECT_TYPE_SEMAPHORE, reinterpret_cast<std::uint64_t>(m_acquiresemaphores[i]), Name + " acquire " + std::to_string(i));
	}

	return build(Reason);
}

bool swapchain::outdated() const
{
	if (m_rebuild || (m_swapchain == VK_NULL_HANDLE) || (m_vsync != wanted_vsync()) || (m_srgb != wanted_srgb()))
	{
		return true;
	}
	int width{0}, height{0};
	glfwGetFramebufferSize(m_window, &width, &height);
	return (width != m_framebuffer.x) || (height != m_framebuffer.y);
}

bool swapchain::build(std::string &Reason)
{
	auto const physical{m_device->physical()};
	auto const device{m_device->handle()};

	int width{0}, height{0};
	glfwGetFramebufferSize(m_window, &width, &height);
	if ((width <= 0) || (height <= 0))
	{
		// minimized: no swapchain of 0x0, the window is skipped until it comes back
		m_rebuild = true;
		return true;
	}

	VkSurfaceCapabilitiesKHR capabilities{};
	auto const result{vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physical, m_surface, &capabilities)};
	if (result != VK_SUCCESS)
	{
		Reason = std::string("the surface of ") + m_name + " can't be queried (" + result_string(result) + ")";
		return false;
	}

	VkExtent2D extent{static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height)};
	if (capabilities.currentExtent.width != 0xFFFFFFFFu)
	{
		extent = capabilities.currentExtent;
	}
	extent.width = std::clamp(extent.width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
	extent.height = std::clamp(extent.height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height);
	if ((extent.width == 0) || (extent.height == 0))
	{
		m_rebuild = true;
		return true;
	}

	// the format: srgb when the shaders write linear colours (gfx.shadergamma no), as GLFW_SRGB_CAPABLE in opengl
	auto const srgb{wanted_srgb()};
	std::uint32_t formatcount{0};
	vkGetPhysicalDeviceSurfaceFormatsKHR(physical, m_surface, &formatcount, nullptr);
	std::vector<VkSurfaceFormatKHR> formats(formatcount);
	vkGetPhysicalDeviceSurfaceFormatsKHR(physical, m_surface, &formatcount, formats.data());
	if (formats.empty())
	{
		Reason = "the surface of " + m_name + " has no formats";
		return false;
	}
	std::vector<VkFormat> const wanted{srgb ? std::vector<VkFormat>{VK_FORMAT_B8G8R8A8_SRGB, VK_FORMAT_R8G8B8A8_SRGB, VK_FORMAT_A8B8G8R8_SRGB_PACK32} : std::vector<VkFormat>{VK_FORMAT_B8G8R8A8_UNORM, VK_FORMAT_R8G8B8A8_UNORM, VK_FORMAT_A8B8G8R8_UNORM_PACK32}};
	VkSurfaceFormatKHR chosen{formats.front()};
	if ((formats.size() == 1) && (formats.front().format == VK_FORMAT_UNDEFINED))
	{
		// any format will do
		chosen = {wanted.front(), VK_COLOR_SPACE_SRGB_NONLINEAR_KHR};
	}
	else
	{
		auto found{false};
		for (auto const format : wanted)
		{
			auto const lookup{std::find_if(formats.begin(), formats.end(), [format](VkSurfaceFormatKHR const &Entry) { return (Entry.format == format) && (Entry.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR); })};
			if (lookup != formats.end())
			{
				chosen = *lookup;
				found = true;
				break;
			}
		}
		if (false == found)
		{
			log_warning(m_name + ": no " + (srgb ? "srgb" : "unorm") + " 8 bit format of the surface, using format " + std::to_string(chosen.format));
		}
	}

	// the presentation: vsync waits for the screen (fifo, always there), without vsync mailbox, or else immediate
	auto const vsync{wanted_vsync()};
	std::uint32_t modecount{0};
	vkGetPhysicalDeviceSurfacePresentModesKHR(physical, m_surface, &modecount, nullptr);
	std::vector<VkPresentModeKHR> modes(modecount);
	vkGetPhysicalDeviceSurfacePresentModesKHR(physical, m_surface, &modecount, modes.data());
	auto presentmode{VK_PRESENT_MODE_FIFO_KHR};
	if (false == vsync)
	{
		for (auto const mode : {VK_PRESENT_MODE_MAILBOX_KHR, VK_PRESENT_MODE_IMMEDIATE_KHR})
		{
			if (std::find(modes.begin(), modes.end(), mode) != modes.end())
			{
				presentmode = mode;
				break;
			}
		}
	}

	// with the generated frames one image more for each of them: all the images of a frame are acquired at once
	auto imagecount{capabilities.minImageCount + 1 + m_generatedcapacity};
	if (capabilities.maxImageCount > 0)
	{
		imagecount = std::min(imagecount, capabilities.maxImageCount);
	}

	auto compositealpha{VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR};
	if ((capabilities.supportedCompositeAlpha & VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR) == 0)
	{
		for (auto const alpha : {VK_COMPOSITE_ALPHA_INHERIT_BIT_KHR, VK_COMPOSITE_ALPHA_PRE_MULTIPLIED_BIT_KHR, VK_COMPOSITE_ALPHA_POST_MULTIPLIED_BIT_KHR})
		{
			if ((capabilities.supportedCompositeAlpha & alpha) != 0)
			{
				compositealpha = alpha;
				break;
			}
		}
	}

	// the images still in use by the frames in flight and the presentation have to be done with before they go
	auto const oldswapchain{m_swapchain};
	if (oldswapchain != VK_NULL_HANDLE)
	{
		vkQueueWaitIdle(m_device->graphics_queue());
		release_images();
	}

	VkSwapchainCreateInfoKHR create{};
	create.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
	create.surface = m_surface;
	create.minImageCount = imagecount;
	create.imageFormat = chosen.format;
	create.imageColorSpace = chosen.colorSpace;
	create.imageExtent = extent;
	create.imageArrayLayers = 1;
	// drawn into as a colour attachment; the frame is copied into it at the end (flipped, see K8 of the plan)
	create.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
	create.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
	create.preTransform = capabilities.currentTransform;
	create.compositeAlpha = compositealpha;
	create.presentMode = presentmode;
	create.clipped = VK_TRUE;
	create.oldSwapchain = oldswapchain;
	// reflex (VK_NV_low_latency2) is told about the swapchain when it's made
	VkSwapchainLatencyCreateInfoNV latency{};
	latency.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_LATENCY_CREATE_INFO_NV;
	latency.latencyModeEnable = VK_TRUE;
	auto const lowlatency{(m_lowlatency > 0) && m_device->low_latency_enabled()};
	if (lowlatency)
	{
		create.pNext = &latency;
	}

	VkSwapchainKHR swapchain{VK_NULL_HANDLE};
	auto const created{vkCreateSwapchainKHR(device, &create, nullptr, &swapchain)};
	if (oldswapchain != VK_NULL_HANDLE)
	{
		vkDestroySwapchainKHR(device, oldswapchain, nullptr);
	}
	m_swapchain = VK_NULL_HANDLE;
	if (created != VK_SUCCESS)
	{
		Reason = "the swapchain of " + m_name + " can't be made (" + result_string(created) + ")";
		return false;
	}
	m_swapchain = swapchain;
	m_lowlatencyon = false;
	if (lowlatency && (vkSetLatencySleepModeNV != nullptr))
	{
		VkLatencySleepModeInfoNV mode{};
		mode.sType = VK_STRUCTURE_TYPE_LATENCY_SLEEP_MODE_INFO_NV;
		mode.lowLatencyMode = VK_TRUE;
		mode.lowLatencyBoost = (m_lowlatency > 1 ? VK_TRUE : VK_FALSE);
		mode.minimumIntervalUs = 0;
		auto const result{vkSetLatencySleepModeNV(device, m_swapchain, &mode)};
		m_lowlatencyon = (result == VK_SUCCESS);
		if (false == m_lowlatencyon)
		{
			log_warning(m_name + ": reflex can't be turned on (" + result_string(result) + ")");
		}
	}
	m_format = chosen.format;
	m_colorspace = chosen.colorSpace;
	m_presentmode = presentmode;
	m_extent = extent;
	m_framebuffer = {width, height};
	m_vsync = vsync;
	m_srgb = srgb;
	m_rebuild = false;
	m_instance->set_object_name(device, VK_OBJECT_TYPE_SWAPCHAIN_KHR, reinterpret_cast<std::uint64_t>(m_swapchain), m_name + " swapchain");

	std::uint32_t count{0};
	vkGetSwapchainImagesKHR(device, m_swapchain, &count, nullptr);
	m_images.resize(count);
	vkGetSwapchainImagesKHR(device, m_swapchain, &count, m_images.data());
	// the images the program may hold acquired at once (an acquire which waits without end may be done only within them)
	m_acquirable = (count > capabilities.minImageCount ? count - capabilities.minImageCount : 1);
	m_states.assign(count, image_state{});

	VkSemaphoreCreateInfo semaphoreinfo{};
	semaphoreinfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
	m_views.assign(count, VK_NULL_HANDLE);
	m_presentsemaphores.assign(count, VK_NULL_HANDLE);
	for (std::uint32_t i = 0; i < count; ++i)
	{
		VkImageViewCreateInfo view{};
		view.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
		view.image = m_images[i];
		view.viewType = VK_IMAGE_VIEW_TYPE_2D;
		view.format = m_format;
		view.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
		if (!VK14_VERIFY(vkCreateImageView(device, &view, nullptr, &m_views[i])) || !VK14_VERIFY(vkCreateSemaphore(device, &semaphoreinfo, nullptr, &m_presentsemaphores[i])))
		{
			Reason = "the images of the swapchain of " + m_name + " can't be used";
			return false;
		}
		m_instance->set_object_name(device, VK_OBJECT_TYPE_IMAGE, reinterpret_cast<std::uint64_t>(m_images[i]), m_name + " image " + std::to_string(i));
		m_instance->set_object_name(device, VK_OBJECT_TYPE_SEMAPHORE, reinterpret_cast<std::uint64_t>(m_presentsemaphores[i]), m_name + " present " + std::to_string(i));
	}

	log_info("swapchain of " + m_name + ": " + std::to_string(m_extent.width) + "x" + std::to_string(m_extent.height) + ", " + std::to_string(count) + " images, " + (is_srgb(m_format) ? "srgb" : "unorm") + " format " + std::to_string(m_format) + ", present mode " +
	         present_mode_string(m_presentmode) + (m_vsync ? " (vsync)" : ""));
	return true;
}

swapchain::acquire_result swapchain::acquire(std::uint32_t const Frame, std::uint32_t const Generated)
{
	m_generated.clear();
	if (outdated())
	{
		std::string reason;
		if (false == build(reason))
		{
			log_error(reason);
			return acquire_result::failed;
		}
	}
	if ((m_swapchain == VK_NULL_HANDLE) || m_rebuild)
	{
		return acquire_result::skipped;
	}

	auto const result{vkAcquireNextImageKHR(m_device->handle(), m_swapchain, UINT64_MAX, acquire_semaphore(Frame), VK_NULL_HANDLE, &m_image)};
	if ((result == VK_SUCCESS) || (result == VK_SUBOPTIMAL_KHR))
	{
		// what the image held is of no use; its use waits for the acquire semaphore, which the submit waits for at the colour output
		if (m_states.size() < m_images.size())
		{
			m_states.resize(m_images.size());
		}
		m_states[m_image] = image_state{VK_IMAGE_LAYOUT_UNDEFINED, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_NONE};
	}
	// the images of the generated frames, shown before the one drawn; the swapchain has one image more for each of them, so the
	// images of a frame can all be acquired. one which can't be had leaves fewer of them
	if ((result == VK_SUCCESS) || (result == VK_SUBOPTIMAL_KHR))
	{
		auto const room{std::min(std::min(Generated, m_generatedcapacity), m_acquirable - 1)};
		for (std::uint32_t i = 0; i < room; ++i)
		{
			std::uint32_t image{0};
			auto const extra{vkAcquireNextImageKHR(m_device->handle(), m_swapchain, UINT64_MAX, generated_acquire_semaphore(Frame, i), VK_NULL_HANDLE, &image)};
			if ((extra != VK_SUCCESS) && (extra != VK_SUBOPTIMAL_KHR))
			{
				m_rebuild = true;
				break;
			}
			if (extra == VK_SUBOPTIMAL_KHR)
			{
				m_rebuild = true;
			}
			m_states[image] = image_state{VK_IMAGE_LAYOUT_UNDEFINED, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_NONE};
			m_generated.push_back(image);
		}
	}
	switch (result)
	{
	case VK_SUCCESS:
		return acquire_result::ready;
	case VK_SUBOPTIMAL_KHR:
		// the image is acquired and its semaphore will be signalled, so it's drawn and shown; the swapchain is made anew after
		m_rebuild = true;
		return acquire_result::ready;
	case VK_ERROR_OUT_OF_DATE_KHR:
		m_rebuild = true;
		return acquire_result::skipped;
	default:
		break;
	}
	log_error(std::string("vkAcquireNextImageKHR of ") + m_name + " failed with " + result_string(result));
	return acquire_result::failed;
}

bool swapchain::latency_sleep(VkSemaphore const Semaphore, std::uint64_t const Value) const
{
	if ((false == m_lowlatencyon) || (m_swapchain == VK_NULL_HANDLE) || (vkLatencySleepNV == nullptr))
	{
		return false;
	}
	// minimized: nothing is presented, the driver has nothing to measure (the main thread, as glfw wants)
	int width{0}, height{0};
	glfwGetFramebufferSize(m_window, &width, &height);
	if ((width <= 0) || (height <= 0))
	{
		return false;
	}
	VkLatencySleepInfoNV info{};
	info.sType = VK_STRUCTURE_TYPE_LATENCY_SLEEP_INFO_NV;
	info.signalSemaphore = Semaphore;
	info.value = Value;
	return (vkLatencySleepNV(m_device->handle(), m_swapchain, &info) == VK_SUCCESS);
}

void swapchain::latency_marker(std::uint64_t const PresentId, VkLatencyMarkerNV const Marker) const
{
	if ((false == m_lowlatencyon) || (m_swapchain == VK_NULL_HANDLE) || (vkSetLatencyMarkerNV == nullptr))
	{
		return;
	}
	VkSetLatencyMarkerInfoNV info{};
	info.sType = VK_STRUCTURE_TYPE_SET_LATENCY_MARKER_INFO_NV;
	info.presentID = PresentId;
	info.marker = Marker;
	vkSetLatencyMarkerNV(m_device->handle(), m_swapchain, &info);
}

void swapchain::presented(VkResult const Result)
{
	if ((Result == VK_SUBOPTIMAL_KHR) || (Result == VK_ERROR_OUT_OF_DATE_KHR))
	{
		m_rebuild = true;
	}
}

void swapchain::record_clear(command_stream *Commands, glm::vec4 const &Colour) const
{
	VkImageMemoryBarrier2 barrier{};
	barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
	// the acquire semaphore is waited for at the colour output, so the change of layout waits for it too
	barrier.srcStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
	barrier.srcAccessMask = VK_ACCESS_2_NONE;
	barrier.dstStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
	barrier.dstAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
	barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	barrier.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
	barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	barrier.image = m_images[m_image];
	barrier.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};

	VkDependencyInfo dependency{};
	dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
	dependency.imageMemoryBarrierCount = 1;
	dependency.pImageMemoryBarriers = &barrier;
	Commands->pipeline_barrier2(&dependency);

	VkRenderingAttachmentInfo colour{};
	colour.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
	colour.imageView = m_views[m_image];
	colour.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
	colour.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	colour.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	colour.clearValue.color = {{Colour.r, Colour.g, Colour.b, Colour.a}};

	VkRenderingInfo rendering{};
	rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
	rendering.renderArea = {{0, 0}, m_extent};
	rendering.layerCount = 1;
	rendering.colorAttachmentCount = 1;
	rendering.pColorAttachments = &colour;
	Commands->begin_rendering(&rendering);
	Commands->end_rendering();

	barrier.srcStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
	barrier.srcAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
	barrier.dstStageMask = VK_PIPELINE_STAGE_2_NONE;
	barrier.dstAccessMask = VK_ACCESS_2_NONE;
	barrier.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
	barrier.newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
	Commands->pipeline_barrier2(&dependency);
}

void swapchain::release_images()
{
	auto const device{m_device != nullptr ? m_device->handle() : VK_NULL_HANDLE};
	if (device == VK_NULL_HANDLE)
	{
		return;
	}
	for (auto view : m_views)
	{
		if (view != VK_NULL_HANDLE)
		{
			vkDestroyImageView(device, view, nullptr);
		}
	}
	for (auto semaphore : m_presentsemaphores)
	{
		if (semaphore != VK_NULL_HANDLE)
		{
			vkDestroySemaphore(device, semaphore, nullptr);
		}
	}
	m_views.clear();
	m_presentsemaphores.clear();
	m_images.clear();
	m_states.clear();
}

void swapchain::destroy()
{
	auto const device{m_device != nullptr ? m_device->handle() : VK_NULL_HANDLE};
	if (device != VK_NULL_HANDLE)
	{
		release_images();
		if (m_swapchain != VK_NULL_HANDLE)
		{
			vkDestroySwapchainKHR(device, m_swapchain, nullptr);
		}
		for (auto semaphore : m_acquiresemaphores)
		{
			if (semaphore != VK_NULL_HANDLE)
			{
				vkDestroySemaphore(device, semaphore, nullptr);
			}
		}
	}
	m_swapchain = VK_NULL_HANDLE;
	m_acquiresemaphores.clear();
	m_generated.clear();
	if ((m_surface != VK_NULL_HANDLE) && (m_instance != nullptr) && (m_instance->handle() != VK_NULL_HANDLE))
	{
		vkDestroySurfaceKHR(m_instance->handle(), m_surface, nullptr);
	}
	m_surface = VK_NULL_HANDLE;
	m_window = nullptr;
}

} // namespace vk14
