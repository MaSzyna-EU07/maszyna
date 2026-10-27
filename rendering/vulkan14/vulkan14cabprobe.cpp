/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the probe of the occupied cab in the vulkan renderer (the stage A4 of the plan; opengl46_cab_probe and
// opengl46_renderer::Render_cab_probe()): the maps of vk14::cab_probe_maps, the cab drawn to them by the colour mode of the shared walk
// of the scene with the programs of the probe (the six faces at once by multiview), their mipmaps and the spherical harmonics of
// the light the cab reflects (cabprobe_sh.comp)

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14context.h"
#include "rendering/vulkan14/vulkan14commands.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "vehicle/DynObj.h"
#include "vehicle/Train.h"

namespace
{

VkPipelineStageFlags2 const colour_stage{VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT};
VkAccessFlags2 const colour_access{VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT};
VkPipelineStageFlags2 const depth_stage{VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT};
VkAccessFlags2 const depth_access{VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT};
VkPipelineStageFlags2 const sampled_stage{VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT};
VkAccessFlags2 const sampled_access{VK_ACCESS_2_SHADER_SAMPLED_READ_BIT};

// the formats of the maps: the light reflected (hdr) and the albedo
VkFormat const LIGHT_FORMAT{VK_FORMAT_R16G16B16A16_SFLOAT};
VkFormat const ALBEDO_FORMAT{VK_FORMAT_R8G8B8A8_UNORM};

// the mipmaps of a cube map whose first level was drawn: each level from the one before it, all of them left in the layout the
// shading reads them in (the environment map makes its own the same way, render_reflections())
void make_cube_mipmaps(vk14::command_stream *Commands, vk14::image &Image)
{
	auto const mips{Image.desc().mips};
	auto const size{static_cast<std::int32_t>(Image.desc().extent.width)};
	auto barrier = [&](std::uint32_t Level, std::uint32_t Count, VkImageLayout From, VkImageLayout To, VkPipelineStageFlags2 SrcStage, VkAccessFlags2 SrcAccess, VkPipelineStageFlags2 DstStage, VkAccessFlags2 DstAccess) {
		VkImageMemoryBarrier2 entry{};
		entry.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
		entry.srcStageMask = SrcStage;
		entry.srcAccessMask = SrcAccess;
		entry.dstStageMask = DstStage;
		entry.dstAccessMask = DstAccess;
		entry.oldLayout = From;
		entry.newLayout = To;
		entry.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		entry.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		entry.image = Image.handle();
		entry.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, Level, Count, 0, 6};
		VkDependencyInfo dependency{};
		dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
		dependency.imageMemoryBarrierCount = 1;
		dependency.pImageMemoryBarriers = &entry;
		Commands->pipeline_barrier2(&dependency);
	};
	// the first level was drawn, the others are undefined
	barrier(0, 1, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, colour_stage, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
	if (mips > 1)
	{
		barrier(1, mips - 1, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_PIPELINE_STAGE_2_NONE, VK_ACCESS_2_NONE, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
	}
	for (std::uint32_t level = 1; level < mips; ++level)
	{
		VkImageBlit2 blit{};
		blit.sType = VK_STRUCTURE_TYPE_IMAGE_BLIT_2;
		blit.srcSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, level - 1, 0, 6};
		blit.srcOffsets[1] = {std::max(size >> (level - 1), 1), std::max(size >> (level - 1), 1), 1};
		blit.dstSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, level, 0, 6};
		blit.dstOffsets[1] = {std::max(size >> level, 1), std::max(size >> level, 1), 1};
		VkBlitImageInfo2 info{};
		info.sType = VK_STRUCTURE_TYPE_BLIT_IMAGE_INFO_2;
		info.srcImage = Image.handle();
		info.srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
		info.dstImage = Image.handle();
		info.dstImageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		info.regionCount = 1;
		info.pRegions = &blit;
		info.filter = VK_FILTER_LINEAR;
		Commands->blit_image2(&info);
		barrier(level, 1, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
	}
	barrier(0, mips, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_READ_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT, sampled_stage, sampled_access);
	Image.state = vk14::image_state{VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access};
}

VkRenderingAttachmentInfo cleared_attachment(VkImageView const View, VkImageLayout const Layout, VkClearValue const &Value)
{
	VkRenderingAttachmentInfo info{};
	info.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
	info.imageView = View;
	info.imageLayout = Layout;
	info.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	info.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	info.clearValue = Value;
	return info;
}

} // namespace

namespace vk14
{

cab_probe_maps::~cab_probe_maps()
{
	destroy();
}

void cab_probe_maps::init(VkDevice const Device, allocator &Memory, deletion_queue &Deletions)
{
	m_device = Device;
	m_memory = &Memory;
	m_deletions = &Deletions;
}

void cab_probe_maps::destroy()
{
	// the device is idle: the maps go at once
	m_deletions = nullptr;
	let_go();
}

bool cab_probe_maps::gpu_allocate(int const Size)
{
	if ((m_memory == nullptr) || (Size <= 0))
	{
		return false;
	}
	auto const size{static_cast<std::uint32_t>(Size)};
	// the maps: cube maps with their mipmaps, read by the shading, drawn to through a view of the six layers of the first level
	auto make_map = [&](std::shared_ptr<image> &Map, VkImageView &View, VkFormat const Format, char const *Name) {
		image_desc desc;
		desc.format = Format;
		desc.extent = {size, size, 1};
		desc.layers = 6;
		desc.mips = static_cast<std::uint32_t>(std::floor(std::log2(static_cast<float>(size)))) + 1;
		desc.view_type = VK_IMAGE_VIEW_TYPE_CUBE;
		desc.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
		desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
		Map = std::make_shared<image>();
		if (false == Map->create(*m_memory, desc, Name))
		{
			Map.reset();
			return false;
		}
		VkImageViewCreateInfo info{};
		info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
		info.image = Map->handle();
		info.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
		info.format = Format;
		info.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 6};
		if (vkCreateImageView(m_device, &info, nullptr, &View) != VK_SUCCESS)
		{
			View = VK_NULL_HANDLE;
			return false;
		}
		return true;
	};
	if ((false == make_map(m_light, m_lightview, LIGHT_FORMAT, "cab probe light")) || (false == make_map(m_albedo, m_albedoview, ALBEDO_FORMAT, "cab probe albedo")))
	{
		let_go();
		return false;
	}
	image_desc depth;
	depth.format = VK_FORMAT_D32_SFLOAT;
	depth.extent = {size, size, 1};
	depth.layers = 6;
	depth.view_type = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
	depth.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
	m_depth = std::make_shared<image>();
	if (false == m_depth->create(*m_memory, depth, "cab probe depth"))
	{
		let_go();
		return false;
	}
	m_target = draw_target{m_light.get(), m_albedo.get(), m_depth.get(), m_lightview, m_albedoview, size};
	m_drawn = false;
	return true;
}

void cab_probe_maps::gpu_release()
{
	let_go();
}

void cab_probe_maps::let_go()
{
	m_target = draw_target{};
	m_drawn = false;
	std::vector<VkImageView> views;
	for (auto *view : {&m_lightview, &m_albedoview})
	{
		if (*view != VK_NULL_HANDLE)
		{
			views.push_back(*view);
		}
		*view = VK_NULL_HANDLE;
	}
	if (m_deletions != nullptr)
	{
		// the frames in flight may still read them
		m_deletions->push(m_value, [device = m_device, views = std::move(views), light = std::move(m_light), albedo = std::move(m_albedo), depth = std::move(m_depth)]() {
			for (auto *view : views)
			{
				vkDestroyImageView(device, view, nullptr);
			}
		});
	}
	else
	{
		for (auto *view : views)
		{
			vkDestroyImageView(m_device, view, nullptr);
		}
	}
	m_light.reset();
	m_albedo.reset();
	m_depth.reset();
}

} // namespace vk14

bool vulkan14_renderer::init_cab_probe(std::string &Reason)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto const device{ctx.device.handle()};
	try
	{
		auto program = [&](char const *Fragment) {
			auto result{std::make_shared<vk14::scene_program>()};
			result->vertex = ctx.shaders.get("cabprobe.vert");
			result->fragment = ctx.shaders.get(Fragment);
			result->fragment_name = Fragment;
			result->vertex_name = "cabprobe.vert";
			result->vertices = vk14::vertex_kind::scene;
			result->name = std::string("cabprobe.vert + ") + Fragment;
			return result;
		};
		scene.cabprobe_program = program("cabprobe.frag");
		scene.cabprobe_alpha_program = program("cabprobe_alpha.frag");
		if (false == scene.cabprobe_sh.create(device, ctx.pipelines, ctx.shaders.get("cabprobe_sh.comp"), scene.pass_layout, {}, "cab probe spherical harmonics"))
		{
			Reason = "no compute pipeline of the cab probe";
			return false;
		}
	}
	catch (gfx::shader_exception const &e)
	{
		Reason = std::string("invalid shader: ") + e.what();
		return false;
	}
	// the light reflected and the albedo, the six faces drawn at once
	scene.cabprobe_formats.colour = {LIGHT_FORMAT, ALBEDO_FORMAT};
	scene.cabprobe_formats.colours = 2;
	scene.cabprobe_formats.depth = VK_FORMAT_D32_SFLOAT;
	scene.cabprobe_formats.view_mask = 0x3f;
	scene.cabprobe_formats.id = ++scene.formats_count;
	// the light the cab lights itself with again, cleared with the stand-ins in the first frame
	if (false == scene.cabprobe_bounce.create(ctx.memory, sizeof(gl::cab_probe_data), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, vk14::memory_kind::device, "cab probe bounce"))
	{
		Reason = "no buffer of the cab probe";
		return false;
	}
	// the stand-in of the maps: 1x1 faces of nothing, cleared with the other stand-ins in the first frame
	vk14::image_desc desc;
	desc.format = VK_FORMAT_R8G8B8A8_UNORM;
	desc.view_type = VK_IMAGE_VIEW_TYPE_CUBE;
	desc.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
	desc.layers = 6;
	desc.extent = {1, 1, 1};
	desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
	scene.cabprobe_empty = std::make_shared<vk14::image>();
	if (false == scene.cabprobe_empty->create(ctx.memory, desc, "empty cab probe"))
	{
		scene.cabprobe_empty.reset();
		Reason = "no stand-in of the cab probe";
		return false;
	}
	scene.cabprobe.init(device, ctx.memory, ctx.deletions);
	return true;
}

void vulkan14_renderer::destroy_cab_probe()
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	scene.cabprobe.destroy();
	scene.cabprobe_sh.destroy(ctx.device.handle());
	scene.cabprobe_program.reset();
	scene.cabprobe_alpha_program.reset();
	scene.cabprobe_bounce.destroy();
	scene.cabprobe_empty.reset();
}

TDynamicObject *vulkan14_renderer::cab_probe_vehicle() const
{
	// the conditions of Render_cab()
	if (FreeFlyModeFlag || (false == Global.render_cab) || (simulation::Train == nullptr))
	{
		return nullptr;
	}
	auto *vehicle{simulation::Train->Dynamic()};
	if ((vehicle == nullptr) || (false == vehicle->bDisplayCab) || (vehicle->mdKabina == nullptr) || (vehicle->mdKabina == vehicle->mdModel))
	{
		return nullptr;
	}
	return vehicle;
}

void vulkan14_renderer::render_cab_probe()
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &probe{scene.cabprobe};
	auto const commands{m_commands};
	auto *vehicle{cab_probe_vehicle()};
	glm::vec3 center{0.f};
	glm::vec3 extent{0.f};
	if ((vehicle != nullptr) && (false == m_locallights.cab_bounds(vehicle->mdKabina, m_geometry, center, extent)))
	{
		vehicle = nullptr;
	}
	// the cab lamp lights the probe when it's a light of the frame
	auto const *lamp{(vehicle != nullptr) && (m_locallights.cab_lamp_vehicle() == vehicle) ? m_locallights.cab_lamp() : nullptr};
	if (false == probe.update_wanted(vehicle, (vehicle != nullptr ? vehicle->mdKabina : nullptr), center, extent, lamp, m_framestamp))
	{
		return;
	}
	auto const &target{probe.target()};
	if ((target.light == nullptr) || (target.albedo == nullptr) || (target.depth == nullptr) || (false == bool(scene.cabprobe_program)))
	{
		return;
	}
	ctx.passes.begin_pass("cab probe");
	// the lamp with its shadow map, when the map holds the shadows of the lamp where it is
	auto const shadows{(lamp != nullptr) && scene.lamps.ready(vehicle) && (scene.lamps.lamp() == m_locallights.cab_lamp_local())};
	probe.setup_drawing(scene_ubs, lamp, m_locallights.cab_lamp_local(), shadows, scene.lamps.weights());
	// the cab drawn by the colour mode with the programs of the probe. the camera is put at the vehicle, which Render_cab() places the cab
	// relative to, and the modelview takes the cab model to the space of the probe
	m_renderpass.draw_mode = rendermode::color;
	m_renderpass.pass_camera.position() = vehicle->GetPosition();
	m_renderpass.draw_stats = {};
	OpenGLMatrices.mode(GL_MODELVIEW);
	OpenGLMatrices.load_matrix(glm::mat4{probe.model_to_probe() * glm::inverse(vehicle->mMatrix)});
	probe.begin();
	std::vector<vk14::image_use> const uses{vk14::use_of(*target.light, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, colour_stage, colour_access),
	                                        vk14::use_of(*target.albedo, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, colour_stage, colour_access),
	                                        vk14::use_of(*target.depth, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, depth_stage, depth_access)};
	vk14::barriers(commands, uses);
	VkClearValue nothing{};
	nothing.color = {{0.f, 0.f, 0.f, 0.f}};
	VkClearValue farthest{};
	farthest.depthStencil = {0.f, 0};
	std::array<VkRenderingAttachmentInfo, 2> colours{cleared_attachment(target.light_view, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, nothing),
	                                                 cleared_attachment(target.albedo_view, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, nothing)};
	auto depth{cleared_attachment(target.depth->view(), VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, farthest)};
	VkRenderingInfo rendering{};
	rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
	rendering.renderArea = {{0, 0}, {target.size, target.size}};
	rendering.layerCount = 1;
	rendering.viewMask = scene.cabprobe_formats.view_mask;
	rendering.colorAttachmentCount = static_cast<std::uint32_t>(colours.size());
	rendering.pColorAttachments = colours.data();
	rendering.pDepthAttachment = &depth;
	commands->begin_rendering(&rendering);
	remember_rendering(rendering, uses);
	// one sided walls face the probe from either side
	start_draws(scene.cabprobe_formats, static_cast<int>(target.size), static_cast<int>(target.size), vk14::scene_state::set_plain, false);
	m_draw->cullface = false;
	walk_blending(false);
	upload_scene_block();
	upload_light_header();
	m_draw->probepass = true;
	m_draw->depthskip = true;
	Render_cab(vehicle, 0.0f, false);
	m_draw->probepass = false;
	m_draw->depthskip = true;
	if (m_draw->rendering)
	{
		commands->end_rendering();
	}
	end_draws();
	// the mipmaps, and the light the cab lights itself with again from a small level of the light reflected
	make_cube_mipmaps(commands, *target.light);
	make_cube_mipmaps(commands, *target.albedo);
	struct
	{
		glm::ivec2 level;
	} constants{glm::ivec2{probe.sh_level(), probe.sh_size()}};
	scene.cabprobe_sh.dispatch(commands,
	                           {vk14::image_descriptor(ctx.sampling.clamp_linear, target.light->view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL),
	                            vk14::buffer_descriptor(scene.cabprobe_bounce.handle(), 0, sizeof(gl::cab_probe_data))},
	                           constants, 1);
	vk14::memory_barrier(commands, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT, VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_STORAGE_READ_BIT);
	probe.drawn();
	probe.end(static_cast<std::uint32_t>(m_renderpass.draw_stats.drawcalls));
	m_renderpass = m_colorpass;
	ctx.passes.end_pass();
}
