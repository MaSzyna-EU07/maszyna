/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// a window the renderer shows images in: its surface and swapchain, made anew when the window changes size, when the driver
// says the swapchain is out of date, and when vsync or gfx.shadergamma change. a minimized window (0x0) is skipped until
// it comes back

#pragma once

#include "rendering/vulkan14/vulkan14common.h"
#include "rendering/vulkan14/vulkan14passes.h"

namespace vk14
{

class instance;
class device;

class swapchain
{
  public:
	enum class acquire_result
	{
		ready, // an image to draw into, its acquire semaphore will be signalled
		skipped, // nothing to draw this frame (minimized, or made anew)
		failed // the device is gone
	};

	swapchain() = default;
	swapchain(swapchain const &) = delete;
	swapchain &operator=(swapchain const &) = delete;
	~swapchain();

	// the surface of the window and, when the window isn't minimized, the swapchain. Frames: frames in flight
	bool create(instance const &Instance, device const &Device, GLFWwindow *Window, std::string const &Name, std::uint32_t const Frames, std::string &Reason);
	void destroy();

	// the image of the frame Frame (index of the frame in flight)
	acquire_result acquire(std::uint32_t const Frame);
	// the image acquired, from undefined to cleared with Colour (linear) and ready to be shown
	void record_clear(command_stream *Commands, glm::vec4 const &Colour) const;
	// the image acquired and its view
	VkImage image() const
	{
		return m_images[m_image];
	}
	VkImageView view() const
	{
		return m_views[m_image];
	}
	// a use of the image acquired by a pass (vk14::pass_list); the image starts the frame undefined, after the acquire semaphore
	image_use use(VkImageLayout Layout, VkPipelineStageFlags2 Stage, VkAccessFlags2 Access)
	{
		return image_use{m_images[m_image], VK_IMAGE_ASPECT_COLOR_BIT, &m_states[m_image], Layout, Stage, Access};
	}
	// the semaphore signalled when the image of the frame Frame is acquired
	VkSemaphore acquire_semaphore(std::uint32_t const Frame) const
	{
		return m_acquiresemaphores[Frame];
	}
	// the semaphore the presentation of the acquired image waits for
	VkSemaphore present_semaphore() const
	{
		return m_presentsemaphores[m_image];
	}
	VkSwapchainKHR handle() const
	{
		return m_swapchain;
	}
	std::uint32_t image_index() const
	{
		return m_image;
	}
	// the result of the presentation of the image: out of date or suboptimal makes the swapchain anew before the next image
	void presented(VkResult const Result);

	GLFWwindow *window() const
	{
		return m_window;
	}
	VkExtent2D extent() const
	{
		return m_extent;
	}
	VkFormat format() const
	{
		return m_format;
	}
	VkPresentModeKHR present_mode() const
	{
		return m_presentmode;
	}
	bool srgb() const
	{
		return m_srgb;
	}

  private:
	// false only when the device is gone or the surface is lost; a minimized window gives true with no swapchain
	bool build(std::string &Reason);
	void release_images();
	// whether the swapchain has to be made anew before the next image
	bool outdated() const;

	instance const *m_instance{nullptr};
	device const *m_device{nullptr};
	GLFWwindow *m_window{nullptr};
	std::string m_name;
	VkSurfaceKHR m_surface{VK_NULL_HANDLE};
	VkSwapchainKHR m_swapchain{VK_NULL_HANDLE};
	VkFormat m_format{VK_FORMAT_UNDEFINED};
	VkColorSpaceKHR m_colorspace{VK_COLOR_SPACE_SRGB_NONLINEAR_KHR};
	VkPresentModeKHR m_presentmode{VK_PRESENT_MODE_FIFO_KHR};
	VkExtent2D m_extent{0, 0};
	// the size of the window the swapchain was made for; the swapchain itself may be clamped to the limits of the surface
	glm::ivec2 m_framebuffer{0, 0};
	// the settings the swapchain was made for
	bool m_vsync{false};
	bool m_srgb{false};
	bool m_rebuild{false};

	std::vector<VkImage> m_images;
	// the states of the images for the barriers of the passes
	std::vector<image_state> m_states;
	std::vector<VkImageView> m_views;
	// one per image: a semaphore given to the presentation can be reused only once the image comes back
	std::vector<VkSemaphore> m_presentsemaphores;
	// one per frame in flight: free again once the frame which waited for it is done
	std::vector<VkSemaphore> m_acquiresemaphores;
	std::uint32_t m_image{0};
};

} // namespace vk14
