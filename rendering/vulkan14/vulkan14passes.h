/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the passes of a frame (decision K12 of the plan): a list in the fixed order the frame makes them, each pass saying which images
// it uses and how. the barriers between the passes (sync2) are worked out from the state each image was left in, there's no
// graph. each pass is a label for the captures and the validation layer (debug utils), and is timed on the gpu: the times
// of the passes of the frames done go to the debug panel under the same names as the parts of the gpu frame of the opengl
// renderer. when the device is lost, the log tells which passes of which frame were on the gpu

#pragma once

#include "rendering/vulkan14/vulkan14common.h"
#include "rendering/vulkan14/vulkan14memory.h"

namespace vk14
{

class device;

// an image a pass uses, and how
struct image_use
{
	VkImage image{VK_NULL_HANDLE};
	VkImageAspectFlags aspect{VK_IMAGE_ASPECT_COLOR_BIT};
	// where the state of the image is kept: by the image itself, or by the swapchain for its images
	image_state *state{nullptr};
	VkImageLayout layout{VK_IMAGE_LAYOUT_UNDEFINED};
	VkPipelineStageFlags2 stage{VK_PIPELINE_STAGE_2_NONE};
	VkAccessFlags2 access{VK_ACCESS_2_NONE};
};

// the use of a vk14::image
image_use use_of(image &Image, VkImageLayout Layout, VkPipelineStageFlags2 Stage, VkAccessFlags2 Access);

// the barriers which bring the images from their states to the uses, the states updated
void barriers(command_stream *Commands, std::vector<image_use> const &Uses);

class pass_list
{
  public:
	pass_list() = default;
	pass_list(pass_list const &) = delete;
	pass_list &operator=(pass_list const &) = delete;
	~pass_list();

	bool create(device const &Device, std::uint32_t Frames, std::string &Reason);
	void destroy();

	// the frame Frame (index of the frame in flight) is recorded to Commands from now on; Number counts the frames
	void begin_frame(command_stream *Commands, std::uint32_t Frame, std::uint64_t Number);
	// a pass: the barriers of its images, its label and the timestamp of its start
	void begin_pass(char const *Name, std::vector<image_use> const &Uses = {});
	void end_pass();
	void end_frame();
	// the gpu times of the frame Frame, once the gpu is done with it (after the wait for its value of the timeline)
	void collect(std::uint32_t Frame);

	// the passes recorded in the frame done last, and the ones of the frame Number, for the report of a lost device
	std::string passes_of(std::uint64_t Number) const;
	// the gpu times (ms) of the parts of the frame, averaged, as the opengl renderer gives them: "name: x.xx ms" lines
	std::string info() const;
	double frame_ms() const
	{
		return m_framems;
	}

  private:
	struct frame_queries
	{
		VkQueryPool pool{VK_NULL_HANDLE};
		std::vector<std::string> names;
		std::uint32_t used{0};
		std::uint64_t number{0};
		bool recorded{false};
	};
	static constexpr std::uint32_t max_passes{32};

	VkDevice m_device{VK_NULL_HANDLE};
	float m_period{1.f}; // ns per tick
	std::uint64_t m_mask{~0ull}; // the valid bits of the timestamps
	std::vector<frame_queries> m_frames;
	command_stream *m_commands{nullptr};
	std::uint32_t m_frame{0};
	int m_open{-1};
	// averaged times of the parts, ms
	std::vector<std::pair<std::string, double>> m_times;
	double m_framems{0.0};
};

} // namespace vk14
