/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14imgui.h"
#include "rendering/vulkan14/vulkan14commands.h"
#include "rendering/vulkan14/vulkan14bindings.h"
#include "rendering/vulkan14/vulkan14device.h"
#include "imgui/imgui.h"

#include <cstring>

namespace vk14
{

bool imgui_backend::create(resources const &Resources, shader_library &Shaders, pipeline_library &Pipelines, frame_ring &Ring, std::string &Reason)
{
	m_resources = &Resources;
	m_shaders = &Shaders;
	m_pipelines = &Pipelines;
	m_ring = &Ring;
	auto const device{Resources.gpu->handle()};

	VkDescriptorSetLayoutBinding texture{};
	texture.binding = bindings::UI_TEXTURE;
	texture.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
	texture.descriptorCount = 1;
	texture.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
	if (false == m_set.create(device, {texture}, bindings::UI_SET, VK_PIPELINE_BIND_POINT_GRAPHICS, sizeof(VkDescriptorImageInfo), "ui texture"))
	{
		Reason = "no descriptor set layout for the user interface";
		return false;
	}
	m_layout = make_pipeline_layout(device, {m_set.layout}, {VkPushConstantRange{VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(float) * 4}}, "ui");
	if ((m_layout == VK_NULL_HANDLE) || (false == m_set.create_template(device, m_layout, "ui texture")))
	{
		Reason = "no pipeline layout for the user interface";
		return false;
	}
	try
	{
		m_vertex = Shaders.get("imgui.vert");
		m_fragment = Shaders.get("imgui.frag");
	}
	catch (gfx::shader_exception const &e)
	{
		Reason = std::string("the shaders of the user interface: ") + e.what();
		return false;
	}
	return true;
}

void imgui_backend::destroy()
{
	if (m_resources == nullptr)
	{
		return;
	}
	auto const device{m_resources->gpu->handle()};
	for (auto const image : m_images)
	{
		m_resources->images->remove(image, *m_resources->deletions, m_resources->uploads->current_value());
	}
	m_images.clear();
	m_font = 0;
	if (ImGui::GetCurrentContext() != nullptr)
	{
		// imgui outlives the renderer (ui_layer::shutdown() comes after it)
		auto &io{ImGui::GetIO()};
		io.Fonts->SetTexID(0);
		io.BackendRendererName = nullptr;
		io.BackendFlags &= ~ImGuiBackendFlags_RendererHasVtxOffset;
	}
	m_set.destroy(device);
	if (m_layout != VK_NULL_HANDLE)
	{
		vkDestroyPipelineLayout(device, m_layout, nullptr);
		m_layout = VK_NULL_HANDLE;
	}
	m_vertex.reset();
	m_fragment.reset();
	m_drawdata = nullptr;
	m_resources = nullptr;
}

bool imgui_backend::Init()
{
	auto &io{ImGui::GetIO()};
	io.BackendRendererName = "maszyna vulkan14";
	io.BackendFlags |= ImGuiBackendFlags_RendererHasVtxOffset;
	BeginFrame();
	return (m_resources != nullptr);
}

void imgui_backend::Shutdown()
{
	auto &io{ImGui::GetIO()};
	if ((m_font != 0) && (m_resources != nullptr))
	{
		Release_Image(m_font);
		m_font = 0;
	}
	io.Fonts->SetTexID(0);
	io.BackendRendererName = nullptr;
	io.BackendFlags &= ~ImGuiBackendFlags_RendererHasVtxOffset;
	m_drawdata = nullptr;
}

void imgui_backend::BeginFrame()
{
	if (m_resources == nullptr)
	{
		return;
	}
	// the font, made again when imgui built it anew
	auto &io{ImGui::GetIO()};
	if ((m_font != 0) && io.Fonts->IsBuilt() && (io.Fonts->TexID == m_font))
	{
		return;
	}
	unsigned char *pixels{nullptr};
	int width{0}, height{0};
	io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
	image_options options;
	// the alpha of the glyphs as it is, as the opengl renderer has it
	options.srgb = false;
	m_font = Update_Image(m_font, pixels, width, height, options);
	io.Fonts->SetTexID(static_cast<ImTextureID>(m_font));
}

void imgui_backend::Render()
{
	m_drawdata = ImGui::GetDrawData();
}

std::uint64_t imgui_backend::Create_Image(std::uint8_t const *Rgba, int const Width, int const Height, image_options const &Options)
{
	return Update_Image(0, Rgba, Width, Height, Options);
}

std::uint64_t imgui_backend::Update_Image(std::uint64_t const Id, std::uint8_t const *Rgba, int const Width, int const Height, image_options const &Options)
{
	if ((m_resources == nullptr) || (Rgba == nullptr) || (Width <= 0) || (Height <= 0))
	{
		return Id;
	}
	// only the images made here are given new content; another id (released, or of a texture) gets a new image
	auto const *current{(std::find(m_images.begin(), m_images.end(), Id) != m_images.end()) ? m_resources->images->find(Id) : nullptr};
	auto const Image{current != nullptr ? Id : 0};
	auto const format{Options.srgb ? VK_FORMAT_R8G8B8A8_SRGB : VK_FORMAT_R8G8B8A8_UNORM};
	auto const width{static_cast<std::uint32_t>(Width)}, height{static_cast<std::uint32_t>(Height)};
	auto const mipmaps{Options.mipmaps && m_resources->uploads->can_blit(format)};
	auto const sampler{Options.nearest ? m_resources->sampling->clamp_nearest : m_resources->sampling->clamp_linear};

	std::shared_ptr<image> target;
	if (current != nullptr)
	{
		auto const &desc{current->target->desc()};
		if ((desc.extent.width == width) && (desc.extent.height == height) && (desc.format == format) && ((desc.mips > 1) == mipmaps))
		{
			// the same image given new content; the frames which drew it come first in the queue
			target = current->target;
		}
	}
	if (target == nullptr)
	{
		image_desc desc;
		desc.format = format;
		desc.extent = {width, height, 1};
		desc.mips = (mipmaps ? static_cast<std::uint32_t>(std::floor(std::log2(std::max(width, height)))) + 1 : 1);
		desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | (mipmaps ? VK_IMAGE_USAGE_TRANSFER_SRC_BIT : 0);
		target = std::make_shared<image>();
		if (false == target->create(*m_resources->memory, desc, "ui image"))
		{
			return Id;
		}
	}
	if (false == m_resources->uploads->upload(target, Rgba, static_cast<VkDeviceSize>(width) * height * 4, {{0, {width, height, 1}}}, mipmaps))
	{
		return Id;
	}

	if (Image != 0)
	{
		if (current->target != target)
		{
			// the old image goes once the frames which drew it are done
			m_resources->deletions->push(m_resources->uploads->current_value(), [old = current->target]() {});
		}
		m_resources->images->replace(Image, target, sampler);
		return Image;
	}
	auto const id{m_resources->images->add(target, sampler)};
	m_images.push_back(id);
	return id;
}

void imgui_backend::Release_Image(std::uint64_t const Image)
{
	if ((m_resources == nullptr) || (Image == 0))
	{
		return;
	}
	auto const lookup{std::find(m_images.begin(), m_images.end(), Image)};
	if (lookup == m_images.end())
	{
		// not one of the images of the user interface
		return;
	}
	m_images.erase(lookup);
	m_resources->images->remove(Image, *m_resources->deletions, m_resources->uploads->current_value());
}

void imgui_backend::record(command_stream *Commands, VkExtent2D Extent, VkFormat Format, bool Srgb, bool const Keep)
{
	auto const *drawdata{m_drawdata};
	if (false == Keep)
	{
		m_drawdata = nullptr;
	}
	if ((drawdata == nullptr) || (drawdata->CmdListsCount == 0) || (drawdata->TotalVtxCount == 0) || (m_resources == nullptr))
	{
		return;
	}
	auto const scale{drawdata->FramebufferScale};
	if ((drawdata->DisplaySize.x <= 0.f) || (drawdata->DisplaySize.y <= 0.f))
	{
		return;
	}

	pipeline_desc desc;
	desc.stages = {m_vertex, m_fragment};
	desc.layout = m_layout;
	desc.vertices.bindings = {VkVertexInputBindingDescription{0, sizeof(ImDrawVert), VK_VERTEX_INPUT_RATE_VERTEX}};
	desc.vertices.attributes = {VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32_SFLOAT, static_cast<std::uint32_t>(offsetof(ImDrawVert, pos))},
	                            VkVertexInputAttributeDescription{1, 0, VK_FORMAT_R32G32_SFLOAT, static_cast<std::uint32_t>(offsetof(ImDrawVert, uv))},
	                            VkVertexInputAttributeDescription{2, 0, VK_FORMAT_R8G8B8A8_UNORM, static_cast<std::uint32_t>(offsetof(ImDrawVert, col))}};
	desc.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
	desc.blending = blend_mode::alpha;
	desc.colour_formats = {Format};
	// the colour encoded by the shader where the image of the window doesn't do it
	desc.specialization = {Srgb ? 0u : 1u};
	desc.name = "ui";
	auto const pipeline{m_pipelines->get(desc)};
	if (pipeline == VK_NULL_HANDLE)
	{
		return;
	}

	// the vertices and the indices of the frame, through its ring
	auto const vertexbytes{static_cast<VkDeviceSize>(drawdata->TotalVtxCount) * sizeof(ImDrawVert)};
	auto const indexbytes{static_cast<VkDeviceSize>(drawdata->TotalIdxCount) * sizeof(ImDrawIdx)};
	auto const vertices{m_ring->allocate(vertexbytes, sizeof(ImDrawVert))};
	auto const indices{m_ring->allocate(indexbytes, 4)};
	if (!vertices || !indices)
	{
		log_warning("the ring of the frame is too small for the user interface");
		return;
	}
	auto *vertexdata{vertices.data};
	auto *indexdata{indices.data};
	for (int list = 0; list < drawdata->CmdListsCount; ++list)
	{
		auto const *commands{drawdata->CmdLists[list]};
		std::memcpy(vertexdata, commands->VtxBuffer.Data, commands->VtxBuffer.Size * sizeof(ImDrawVert));
		std::memcpy(indexdata, commands->IdxBuffer.Data, commands->IdxBuffer.Size * sizeof(ImDrawIdx));
		vertexdata += commands->VtxBuffer.Size * sizeof(ImDrawVert);
		indexdata += commands->IdxBuffer.Size * sizeof(ImDrawIdx);
	}

	Commands->bind_pipeline(VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
	set_draw_state(Commands, Extent);
	Commands->bind_vertex_buffers(0, 1, &vertices.buffer, &vertices.offset);
	Commands->bind_index_buffer(indices.buffer, indices.offset, sizeof(ImDrawIdx) == 2 ? VK_INDEX_TYPE_UINT16 : VK_INDEX_TYPE_UINT32);

	// the pixels of the display (top left origin, y down) to the clip space of vulkan (y down too)
	float transform[4];
	transform[0] = 2.f / drawdata->DisplaySize.x;
	transform[1] = 2.f / drawdata->DisplaySize.y;
	transform[2] = -1.f - drawdata->DisplayPos.x * transform[0];
	transform[3] = -1.f - drawdata->DisplayPos.y * transform[1];
	VkPushConstantsInfo push{};
	push.sType = VK_STRUCTURE_TYPE_PUSH_CONSTANTS_INFO;
	push.layout = m_layout;
	push.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
	push.offset = 0;
	push.size = sizeof(transform);
	push.pValues = transform;
	Commands->push_constants2(&push);

	auto const clipoffset{drawdata->DisplayPos};
	std::uint32_t vertexoffset{0}, indexoffset{0};
	std::uint64_t pushed{~0ull};
	for (int list = 0; list < drawdata->CmdListsCount; ++list)
	{
		auto const *commands{drawdata->CmdLists[list]};
		for (int index = 0; index < commands->CmdBuffer.Size; ++index)
		{
			auto const &command{commands->CmdBuffer[index]};
			if (command.UserCallback != nullptr)
			{
				// no callbacks in the user interface of the simulator; the one which resets the state is all there is to do
				if (command.UserCallback != ImDrawCallback_ResetRenderState)
				{
					command.UserCallback(commands, &command);
				}
				continue;
			}
			// the clip rectangle in the pixels of the image
			auto const minx{std::max((command.ClipRect.x - clipoffset.x) * scale.x, 0.f)};
			auto const miny{std::max((command.ClipRect.y - clipoffset.y) * scale.y, 0.f)};
			auto const maxx{std::min((command.ClipRect.z - clipoffset.x) * scale.x, static_cast<float>(Extent.width))};
			auto const maxy{std::min((command.ClipRect.w - clipoffset.y) * scale.y, static_cast<float>(Extent.height))};
			if ((maxx <= minx) || (maxy <= miny))
			{
				continue;
			}
			auto const *entry{m_resources->images->find(command.GetTexID())};
			if (entry == nullptr)
			{
				// no image of that id (not made, or gone)
				continue;
			}
			VkRect2D scissor{{static_cast<std::int32_t>(minx), static_cast<std::int32_t>(miny)}, {static_cast<std::uint32_t>(maxx - minx), static_cast<std::uint32_t>(maxy - miny)}};
			Commands->set_scissor(0, 1, &scissor);
			if (pushed != command.GetTexID())
			{
				VkDescriptorImageInfo info{entry->sampler, entry->target->view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
				m_set.push(Commands, m_layout, &info);
				pushed = command.GetTexID();
			}
			Commands->draw_indexed(command.ElemCount, 1, command.IdxOffset + indexoffset, static_cast<std::int32_t>(command.VtxOffset + vertexoffset), 0);
		}
		vertexoffset += static_cast<std::uint32_t>(commands->VtxBuffer.Size);
		indexoffset += static_cast<std::uint32_t>(commands->IdxBuffer.Size);
	}
}

} // namespace vk14
