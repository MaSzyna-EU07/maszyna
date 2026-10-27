/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the probe of the occupied cab in the vulkan renderer (the stage A4 of the plan, opengl46_cab_probe): the maps of gfx::cab_probe,
// which tells when they're drawn and what the shading reads. the six faces are drawn at once by multiview (cabprobe.vert reads the
// face from gl_ViewIndex) instead of the geometry shader of the opengl renderer; the renderer draws to the target the class keeps
// (target()), makes the mipmaps and the spherical harmonics (vulkan14cabprobe.cpp)

#pragma once

#include "rendering/common/cabprobe.h"
#include "rendering/vulkan14/vulkan14memory.h"

#include <memory>

namespace vk14
{

class cab_probe_maps : public gfx::cab_probe
{
  public:
	// what the drawing draws to: the views of the six layers of the first level of the maps and of the depth
	struct draw_target
	{
		image *light{nullptr};
		image *albedo{nullptr};
		image *depth{nullptr};
		VkImageView light_view{VK_NULL_HANDLE};
		VkImageView albedo_view{VK_NULL_HANDLE};
		std::uint32_t size{0};
	};

	cab_probe_maps() = default;
	~cab_probe_maps() override;

	// Device, Memory, Deletions: what the maps are made and let go with; called once
	void init(VkDevice Device, allocator &Memory, deletion_queue &Deletions);
	// the maps go (the device is about to)
	void destroy();
	// the value of the timeline the gpu reaches when it's done with the frame being recorded, before the drawing
	void begin_frame(std::uint64_t const Value)
	{
		m_value = Value;
	}
	draw_target const &target() const
	{
		return m_target;
	}
	// the maps, once drawn (in the layout the shading reads them in), for the pass sets; null: the stand-in is read
	image *light_map() const
	{
		return m_drawn ? m_light.get() : nullptr;
	}
	image *albedo_map() const
	{
		return m_drawn ? m_albedo.get() : nullptr;
	}
	// the maps were drawn and are in the layout the shading reads them in
	void drawn()
	{
		m_drawn = true;
	}

  protected:
	bool gpu_allocate(int const Size) override;
	void gpu_release() override;
	// the drawing is recorded by the renderer
	void gpu_begin() override {}
	void gpu_end() override {}

  private:
	void let_go();

	VkDevice m_device{VK_NULL_HANDLE};
	allocator *m_memory{nullptr};
	deletion_queue *m_deletions{nullptr};
	std::uint64_t m_value{0};
	std::shared_ptr<image> m_light;
	std::shared_ptr<image> m_albedo;
	std::shared_ptr<image> m_depth;
	VkImageView m_lightview{VK_NULL_HANDLE};
	VkImageView m_albedoview{VK_NULL_HANDLE};
	draw_target m_target;
	bool m_drawn{false};
};

} // namespace vk14
