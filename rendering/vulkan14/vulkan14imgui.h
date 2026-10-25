/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the user interface (imgui) drawn by the vulkan renderer (decision K10 of the plan): a backend of its own instead of
// imgui_impl_vulkan, which would need a pool of descriptors and a set per texture. the texture of a draw is pushed with it
// (push descriptors), the vertices and the indices go through the ring of the frame, and ImTextureID is the id of the image in
// the table of the renderer, the same for the textures (ITexture::get_id()), the images of the user interface (Create_Image())
// and the font. the colours are mixed as the user interface of the opengl renderer mixes them: in linear light, encoded by the
// srgb image of the window or, with gfx.shadergamma, by the shader

#pragma once

#include "rendering/renderer.h"
#include "rendering/vulkan14/vulkan14descriptors.h"
#include "rendering/vulkan14/vulkan14shaders.h"
#include "rendering/vulkan14/vulkan14texture.h"

struct ImDrawData;

namespace vk14
{

class imgui_backend : public imgui_renderer
{
  public:
	// what the backend draws with, from the renderer
	bool create(resources const &Resources, shader_library &Shaders, pipeline_library &Pipelines, frame_ring &Ring, std::string &Reason);
	void destroy();

	// imgui_renderer
	bool Init() override;
	void Shutdown() override;
	void BeginFrame() override;
	void Render() override;
	using imgui_renderer::Create_Image;
	std::uint64_t Create_Image(std::uint8_t const *Rgba, int const Width, int const Height, image_options const &Options) override;
	std::uint64_t Update_Image(std::uint64_t const Image, std::uint8_t const *Rgba, int const Width, int const Height, image_options const &Options) override;
	void Release_Image(std::uint64_t const Image) override;

	// whether imgui made a frame to draw since the last record()
	bool has_frame() const
	{
		return m_drawdata != nullptr;
	}
	// records the frame of imgui into Commands, in a rendering to an image of Extent and Format begun by the renderer. Srgb: the
	// image encodes the colour itself
	void record(command_stream *Commands, VkExtent2D Extent, VkFormat Format, bool Srgb);
	// the frame of imgui is dropped (nothing to draw it to)
	void skip()
	{
		m_drawdata = nullptr;
	}

  private:
	resources const *m_resources{nullptr};
	shader_library *m_shaders{nullptr};
	pipeline_library *m_pipelines{nullptr};
	frame_ring *m_ring{nullptr};
	push_set m_set;
	VkPipelineLayout m_layout{VK_NULL_HANDLE};
	std::shared_ptr<shader_code const> m_vertex;
	std::shared_ptr<shader_code const> m_fragment;
	std::uint64_t m_font{0};
	ImDrawData *m_drawdata{nullptr};
	// the images made by Create_Image()
	std::vector<std::uint64_t> m_images;
};

} // namespace vk14
