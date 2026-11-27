/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the shadow maps of the lamps in the vulkan renderer (the stage P4 of the plan, opengl46_local_shadows): the maps of gfx::local_shadows,
// which tells what they hold. the six faces of a cube map are drawn at once by multiview (shadowcube.vert reads the face from
// gl_ViewIndex) instead of the geometry shader of the opengl renderer. the renderer draws to the target the class sets up
// (target()), the copies and the clears of the cubes of the lamps are recorded into the commands of the frame

#pragma once

#include "rendering/common/localshadows.h"
#include "rendering/vulkan14/vulkan14memory.h"

#include <memory>
#include <vector>

namespace vk14
{

class local_shadow_maps : public gfx::local_shadows
{
  public:
	// what the drawing begun last draws to
	struct draw_target
	{
		image *map{nullptr};
		VkImageView view{VK_NULL_HANDLE}; // the layers of the cube (6, multiview) or of the headlights (1)
		std::uint32_t size{0};
		std::uint32_t view_mask{0}; // 0x3f for a cube
		bool clear{true}; // the cube or the layer starts cleared, the working copy of a lamp from the copy of its scenery part
		bool cube{true};
	};

	local_shadow_maps() = default;
	~local_shadow_maps() override;

	// Device, Memory, Deletions: what the maps are made and let go with; called once
	void init(VkDevice Device, allocator &Memory, deletion_queue &Deletions);
	// the maps go (the device is about to)
	void destroy();
	// the commands of the frame being recorded and the value of the timeline the gpu reaches when it's done with it, before the drawing
	void begin_frame(command_stream *Commands, std::uint64_t Value)
	{
		m_commands = Commands;
		m_value = Value;
	}
	// the frame was recorded: no commands go to it any more
	void end_frame()
	{
		m_commands = nullptr;
	}
	draw_target const &target() const
	{
		return m_target;
	}
	// the maps, for the pass sets (null: none, the stand-ins are read)
	image *cab_map() const
	{
		return m_cab.get();
	}
	image *lamp_maps() const
	{
		return m_lamps.get();
	}
	image *headlight_maps() const
	{
		return m_headlights.get();
	}

  protected:
	bool gpu_allocate(int const Count, int const Size) override;
	void gpu_release() override;
	bool gpu_allocate_lamps(int const Count, int const Size) override;
	void gpu_release_lamps() override;
	bool gpu_allocate_headlights(int const Count, int const Size) override;
	void gpu_release_headlights() override;
	void gpu_begin_cab(std::size_t const Slot) override;
	void gpu_end_cab() override {}
	void gpu_begin_lamp(int const Scenerycube, int const Workingcube, bool const Vehicles) override;
	void gpu_end_lamp() override {}
	void gpu_begin_headlight(std::size_t const Slot) override;
	void gpu_end_headlight() override {}
	// the gpu times are those of the pass list
	void gpu_lamps_timer(bool const Begin) override {}
	void gpu_headlights_timer(bool const Begin) override {}

  private:
	// a depth map array of Layers layers of Size x Size, its default view a cube array (Cube) or a 2d array, and a view of each
	// cube (6 layers) or of each layer
	bool make(std::shared_ptr<image> &Map, std::vector<VkImageView> &Views, int const Size, int const Layers, bool const Cube, char const *Name);
	void let_go(std::shared_ptr<image> &Map, std::vector<VkImageView> &Views);

	VkDevice m_device{VK_NULL_HANDLE};
	allocator *m_memory{nullptr};
	deletion_queue *m_deletions{nullptr};
	command_stream *m_commands{nullptr};
	std::uint64_t m_value{0};
	std::shared_ptr<image> m_cab;
	std::vector<VkImageView> m_cabviews;
	std::shared_ptr<image> m_lamps;
	std::vector<VkImageView> m_lampviews;
	std::shared_ptr<image> m_headlights;
	std::vector<VkImageView> m_headlightviews;
	draw_target m_target;
};

} // namespace vk14
