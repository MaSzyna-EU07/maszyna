/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the scene of the vulkan renderer, part two (the stages D3-D7 of the plan): the frame and its passes, in the order of
// opengl46_renderer::Render_pass(): the cascades of the sun shadows, the faces of the environment map, the light clusters, the
// colour of the scene (the sky, the opaque scene with the cab, the translucent parts, the particles, the precipitation, the cab),
// the post-processing (the motion blur, the tonemapping, the chromatic aberration), the output to the windows with the user
// interface; the picking, the position of the cursor of the editor, the screenshots, the previews of the models

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14context.h"
#include "rendering/screenshot.h"
#include "rendering/editoroverlay.h"
#include "application/application.h"
#include "simulation/simulation.h"
#include "simulation/simulationenvironment.h"
#include "simulation/simulationtime.h"
#include "vehicle/Train.h"
#include "vehicle/DynObj.h"
#include "model/Model3d.h"
#include "model/AnimModel.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/Timer.h"
#include "utilities/utilities.h"
#include "utilities/color.h"

namespace
{

// the size of the (square) images of the picking passes
int constexpr pick_buffer_size{1024};

// the projections of the opengl renderer (opengl46_renderer::perspective_projection() and the others): reverse z with the
// range 1..0, an infinite far plane for the perspective ones
glm::mat4 perspective_projection(viewport_proj_config const &c, float n, float f, glm::mat4 &frustum)
{
	glm::vec3 vr, vu, vn;
	vr = glm::normalize(c.pb - c.pa);
	vu = glm::normalize(c.pc - c.pa);
	vn = glm::normalize(glm::cross(vr, vu));

	glm::mat4 M;
	M[0] = glm::vec4(vr, 0.0f);
	M[1] = glm::vec4(vu, 0.0f);
	M[2] = glm::vec4(vn, 0.0f);
	M = glm::transpose(M);

	glm::vec3 va, vb, vc;
	va = c.pa - c.pe;
	vb = c.pb - c.pe;
	vc = c.pc - c.pe;

	float l, r, b, t, d;
	d = -glm::dot(va, vn);

	l = glm::dot(vr, va) * n / d;
	r = glm::dot(vr, vb) * n / d;
	b = glm::dot(vu, va) * n / d;
	t = glm::dot(vu, vc) * n / d;

	frustum = glm::frustum(l, r, b, t, n, f);
	glm::mat4 R = frustum;
	frustum = glm::translate(frustum * M, -c.pe);

	R[2][2] = -1.0f;
	R[3][2] = -2.0f * n;
	R = glm::mat4(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.5f, 0.0f, 0.0f, 0.0f, 0.5f, 1.0f) * R;

	return glm::translate(R * M, -c.pe);
}

glm::mat4 perspective_projection_raw(float fovy, float aspect, float znear, float zfar)
{
	const float f = 1.0f / std::tan(fovy / 2.0f);
	return glm::mat4(f / aspect, 0.0f, 0.0f, 0.0f, 0.0f, f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, znear, 0.0f);
}

glm::mat4 ortho_projection(float l, float r, float b, float t, float znear, float zfar)
{
	glm::mat4 proj = glm::ortho(l, r, b, t, znear, zfar);
	return glm::mat4(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.5f, 0.0f, 0.0f, 0.0f, 0.5f, 1.0f) * proj;
}

glm::mat4 ortho_frustumtest_projection(float l, float r, float b, float t, float znear, float zfar)
{
	return glm::ortho(l, r, b, t, znear, zfar);
}

// transform 1..-1 NDC xy coordinates to 1..0 (the z is 1..0 already)
glm::mat4 const coordmove{0.5, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.5, 0.5, 0.0, 1.0};

// the vertices of a particle of the smoke (opengl46_particles)
struct particle_vertex
{
	glm::vec3 position;
	glm::vec4 color;
	glm::vec2 texture;
};
static_assert(sizeof(particle_vertex) == 36, "the layout of the vertices of the particles");

std::vector<std::pair<glm::vec3, glm::vec2>> const billboard_vertices{
    {{-0.5f, -0.5f, 0.f}, {0.f, 0.f}}, {{0.5f, -0.5f, 0.f}, {1.f, 0.f}}, {{0.5f, 0.5f, 0.f}, {1.f, 1.f}}, {{-0.5f, -0.5f, 0.f}, {0.f, 0.f}}, {{0.5f, 0.5f, 0.f}, {1.f, 1.f}}, {{-0.5f, 0.5f, 0.f}, {0.f, 1.f}},
};

// the cone of the precipitation around the camera (opengl46_precipitation::create())
struct precipitation_mesh
{
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec2> uvs;
	std::vector<std::uint16_t> indices;

	void create(int const Tesselation)
	{
		auto const heightfactor{10.f};
		auto const verticaltexturestretchfactor{1.5f};
		auto const latitudes{3};
		auto const longitudes{Tesselation};
		auto const longitudehalfstep{0.5f * static_cast<float>(2.0 * M_PI * 1.f / longitudes)};
		std::uint16_t index = 0;
		std::vector radii{25.f, 10.f, 5.f, 1.f};
		for (auto radius : radii)
		{
			for (int i = 0; i <= latitudes; ++i)
			{
				auto const latitude{static_cast<float>(M_PI * (-0.5f + (float)(i) / latitudes))};
				auto const z{std::sin(latitude)};
				auto const zr{std::cos(latitude)};
				for (int j = 0; j <= longitudes; ++j)
				{
					auto const longitude{static_cast<float>(2.0 * M_PI * (float)(j) / longitudes)};
					auto const x{std::cos(longitude)};
					auto const y{std::sin(longitude)};
					vertices.emplace_back(glm::vec3(-x * zr, -z * heightfactor, -y * zr) * radius);
					auto u = (i == 0 ? longitude + longitudehalfstep : i == latitudes ? longitude - longitudehalfstep : longitude);
					uvs.emplace_back(u / (2.0 * M_PI) * radius, 1.f - (float)(i) / latitudes * radius * heightfactor * 0.5f / verticaltexturestretchfactor);
					if ((i == 0) || (j == 0))
					{
						++index;
					}
					else
					{
						if (i < latitudes)
						{
							indices.emplace_back(index - 1 - (longitudes + 1));
							indices.emplace_back(index - 1);
							indices.emplace_back(index);
						}
						if (i > 1)
						{
							indices.emplace_back(index);
							indices.emplace_back(index - (longitudes + 1));
							indices.emplace_back(index - 1 - (longitudes + 1));
						}
						++index;
					}
				}
			}
		}
	}
};

precipitation_mesh &precipitation()
{
	static precipitation_mesh mesh;
	if (mesh.vertices.empty())
	{
		mesh.create(18);
	}
	return mesh;
}

VkPipelineStageFlags2 const colour_stage{VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT};
VkAccessFlags2 const colour_access{VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT};
VkPipelineStageFlags2 const depth_stage{VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT};
VkAccessFlags2 const depth_access{VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT};
// a depth resolved by a rendering: written in the stage of the colour output
VkPipelineStageFlags2 const resolve_depth_stage{VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT};
VkAccessFlags2 const resolve_depth_access{VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT};
VkPipelineStageFlags2 const sampled_stage{VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT};
VkAccessFlags2 const sampled_access{VK_ACCESS_2_SHADER_SAMPLED_READ_BIT};

// the copies to the buffers the cpu reads back, made seen by the host once the gpu is done with the frame
void host_read_barrier(VkCommandBuffer Commands)
{
	VkMemoryBarrier2 barrier{};
	barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
	barrier.srcStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
	barrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
	barrier.dstStageMask = VK_PIPELINE_STAGE_2_HOST_BIT;
	barrier.dstAccessMask = VK_ACCESS_2_HOST_READ_BIT;
	VkDependencyInfo dependency{};
	dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
	dependency.memoryBarrierCount = 1;
	dependency.pMemoryBarriers = &barrier;
	vkCmdPipelineBarrier2(Commands, &dependency);
}

// the images the colour of a viewport is drawn to, as its rendering uses them
std::vector<vk14::image_use> colour_target_uses(vk14::viewport_targets &targets)
{
	auto const msaa{targets.samples != VK_SAMPLE_COUNT_1_BIT};
	std::vector<vk14::image_use> uses;
	auto &colourimage{msaa ? *targets.msaa_colour : *targets.colour};
	auto &depthimage{msaa ? *targets.msaa_depth : *targets.depth};
	uses.push_back(vk14::use_of(colourimage, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, colour_stage, colour_access));
	uses.push_back(vk14::use_of(depthimage, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, depth_stage, depth_access));
	if (msaa)
	{
		uses.push_back(vk14::use_of(*targets.colour, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, colour_stage, colour_access));
		uses.push_back(vk14::use_of(*targets.depth, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, resolve_depth_stage, resolve_depth_access));
	}
	if (targets.motion)
	{
		uses.push_back(vk14::use_of(msaa ? *targets.msaa_motion : *targets.motion_image, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, colour_stage, colour_access));
		if (msaa)
		{
			uses.push_back(vk14::use_of(*targets.motion_image, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, colour_stage, colour_access));
		}
	}
	return uses;
}

VkRenderingAttachmentInfo attachment_info(VkImageView View, VkImageLayout Layout, bool const Clear, VkClearValue const &Value)
{
	VkRenderingAttachmentInfo info{};
	info.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
	info.imageView = View;
	info.imageLayout = Layout;
	info.loadOp = (Clear ? VK_ATTACHMENT_LOAD_OP_CLEAR : VK_ATTACHMENT_LOAD_OP_LOAD);
	info.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	info.clearValue = Value;
	return info;
}

} // namespace

// the data of the frames, prepared once the renderer is there

void vulkan14_renderer::init_frame_data()
{
	auto &ctx{*m_context};
	// rgb value for 5780 kelvin
	Global.DayLight.diffuse[0] = 1.0f;
	Global.DayLight.diffuse[1] = 0.9490196f;
	Global.DayLight.diffuse[2] = 0.9058824f;
	Global.DayLight.is_directional = true;

	auto const texturesize{static_cast<int>(ctx.device.caps().limits.maxImageDimension2D)};
	Global.iMaxTextureSize = std::min(Global.iMaxTextureSize, texturesize);
	Global.iMaxCabTextureSize = std::min(Global.iMaxCabTextureSize, texturesize);
	Global.CurrentMaxTextureSize = Global.iMaxTextureSize;
	WriteLog("texture sizes capped at " + std::to_string(Global.iMaxTextureSize) + "p (" + std::to_string(Global.iMaxCabTextureSize) + "p for cab textures)");
	if (Global.iMultisampling)
	{
		WriteLog("using multisampling x" + std::to_string(1 << Global.iMultisampling));
	}
	if (Global.gfx_framebuffer_width == -1)
	{
		Global.gfx_framebuffer_width = Global.fb_size.x;
	}
	if (Global.gfx_framebuffer_height == -1)
	{
		Global.gfx_framebuffer_height = Global.fb_size.y;
	}
	WriteLog("main window size: " + std::to_string(Global.gfx_framebuffer_width) + "x" + std::to_string(Global.gfx_framebuffer_height));

	m_viewports.clear();
	auto &main{m_viewports.emplace_back()};
	main.width = Global.gfx_framebuffer_width;
	main.height = Global.gfx_framebuffer_height;
	main.main = true;
	main.shadow = true;
	main.window = 0;
	main.draw_range = 1.0f;

	// preload some common textures
	WriteLog("Loading common gfx data...");
	m_glaretexture = Fetch_Texture("fx/lightglare");
	m_suntexture = Fetch_Texture("fx/sun");
	m_moontexture = Fetch_Texture("fx/moon");
	m_smoketexture = Fetch_Texture("fx/smoke");
	m_headlightstexture = Fetch_Texture("fx/headlights:st");
	WriteLog("...gfx data pre-loading done");

	// prepare basic geometry chunks
	float const size = 2.5f / 2.0f;
	auto const geometrybank = Create_Bank();
	gfx::vertex_array billboard_array{
	    {{-size, size, 0.f}, glm::vec3(), {1.f, 1.f}}, {{size, size, 0.f}, glm::vec3(), {0.f, 1.f}}, {{-size, -size, 0.f}, glm::vec3(), {1.f, 0.f}}, {{size, -size, 0.f}, glm::vec3(), {0.f, 0.f}}};
	gfx::userdata_array userdata{};
	m_billboardgeometry = m_geometry.create_chunk(billboard_array, userdata, geometrybank, GL_TRIANGLE_STRIP);

	m_invalid_material = Fetch_Material("invalid");
	m_fogrange = 2000.f;
}

// the images of a viewport for its size and the settings
namespace
{

bool make_targets(vk14::allocator &Memory, vk14::deletion_queue &Deletions, std::uint64_t const Value, std::uint32_t &Formatscount, VkSampleCountFlags const Samplecounts, vk14::viewport_targets &Targets, int const Width, int const Height, bool const Srgb)
{
	// as many samples as asked for, as far as the device has them for the colour and the depth
	auto samples{static_cast<VkSampleCountFlagBits>(std::clamp(1 << Global.iMultisampling, 1, 8))};
	while ((samples > VK_SAMPLE_COUNT_1_BIT) && (0 == (Samplecounts & samples)))
	{
		samples = static_cast<VkSampleCountFlagBits>(samples >> 1);
	}
	auto const motion{Global.gfx_postfx_motionblur_enabled};
	auto const aberration{Global.gfx_postfx_chromaticaberration_enabled};
	VkExtent2D const extent{static_cast<std::uint32_t>(std::max(Width, 1)), static_cast<std::uint32_t>(std::max(Height, 1))};
	if ((Targets.extent.width == extent.width) && (Targets.extent.height == extent.height) && (Targets.samples == samples) && (Targets.motion == motion) && (Targets.aberration == aberration) && (Targets.srgb == Srgb) &&
	    Targets.output)
	{
		return true;
	}
	Deletions.push(Value, [old = std::move(Targets)]() {});
	auto const oldformats{Targets.formats};
	auto const oldoutput{Targets.output_formats};
	Targets = vk14::viewport_targets{};
	Targets.extent = extent;
	Targets.samples = samples;
	Targets.motion = motion;
	Targets.aberration = aberration;
	Targets.srgb = Srgb;

	auto make = [&](std::shared_ptr<vk14::image> &Image, VkFormat Format, VkImageUsageFlags Usage, VkSampleCountFlagBits Samples, char const *Name) {
		vk14::image_desc desc;
		desc.format = Format;
		desc.extent = {extent.width, extent.height, 1};
		desc.samples = Samples;
		desc.usage = Usage;
		Image = std::make_shared<vk14::image>();
		if (false == Image->create(Memory, desc, Name))
		{
			Image.reset();
			return false;
		}
		return true;
	};
	auto const hdr{VK_FORMAT_R16G16B16A16_SFLOAT};
	auto const velocity{VK_FORMAT_R16G16_SFLOAT};
	auto const depth{VK_FORMAT_D32_SFLOAT};
	auto const sampledcolour{VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT};
	auto ok{true};
	if (samples != VK_SAMPLE_COUNT_1_BIT)
	{
		ok = ok && make(Targets.msaa_colour, hdr, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT, samples, "msaa colour") && make(Targets.msaa_depth, depth, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, samples, "msaa depth");
		if (motion)
		{
			ok = ok && make(Targets.msaa_motion, velocity, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT, samples, "msaa velocity");
		}
	}
	ok = ok && make(Targets.colour, hdr, sampledcolour, VK_SAMPLE_COUNT_1_BIT, "scene colour") &&
	     make(Targets.depth, depth, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT, VK_SAMPLE_COUNT_1_BIT, "scene depth");
	if (motion)
	{
		ok = ok && make(Targets.motion_image, velocity, sampledcolour, VK_SAMPLE_COUNT_1_BIT, "scene velocity") && make(Targets.blurred, hdr, sampledcolour, VK_SAMPLE_COUNT_1_BIT, "motion blurred");
	}
	auto const output{Srgb ? VK_FORMAT_R8G8B8A8_SRGB : VK_FORMAT_R8G8B8A8_UNORM};
	if (aberration)
	{
		ok = ok && make(Targets.tonemapped, output, sampledcolour, VK_SAMPLE_COUNT_1_BIT, "tonemapped");
	}
	ok = ok && make(Targets.output, output, sampledcolour | VK_IMAGE_USAGE_TRANSFER_SRC_BIT, VK_SAMPLE_COUNT_1_BIT, "output");
	if (false == ok)
	{
		vk14::log_error("the images of a viewport of " + std::to_string(extent.width) + "x" + std::to_string(extent.height) + " can't be made");
		Targets.output.reset();
		return false;
	}
	Targets.formats.colour = {hdr, motion ? velocity : VK_FORMAT_UNDEFINED};
	Targets.formats.colours = (motion ? 2 : 1);
	Targets.formats.depth = depth;
	Targets.formats.samples = samples;
	Targets.output_formats.colour = {output, VK_FORMAT_UNDEFINED};
	Targets.output_formats.colours = 1;
	// the pipelines are made for the number of the formats: the same formats keep theirs
	auto const same = [](vk14::target_formats const &Left, vk14::target_formats const &Right) {
		return (Left.colour == Right.colour) && (Left.colours == Right.colours) && (Left.depth == Right.depth) && (Left.samples == Right.samples);
	};
	Targets.formats.id = ((oldformats.id != 0) && same(oldformats, Targets.formats) ? oldformats.id : ++Formatscount);
	Targets.output_formats.id = ((oldoutput.id != 0) && same(oldoutput, Targets.output_formats) ? oldoutput.id : ++Formatscount);
	return true;
}

} // namespace

// the frame

bool vulkan14_renderer::Render()
{
	if (!m_context || m_context->devicelost)
	{
		return false;
	}
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto const device{ctx.device.handle()};
	auto const slot{static_cast<std::uint32_t>(ctx.framecount % ctx.frames.size())};
	auto &frame{ctx.frames[slot]};

	Timer::subsystem.gfx_total.start();

	// the gpu has to be done with the frame which used the slot before
	if (frame.done > 0)
	{
		if (false == ctx.wait(frame.done))
		{
			report_device_lost("waiting for a frame");
			return false;
		}
		ctx.passes.collect(slot);
	}
	// what the frames done used last goes
	ctx.deletions.collect(std::max(ctx.completed(), frame.done));
	VK14_VERIFY(vkResetCommandPool(device, frame.pool, 0));
	ctx.ring.begin_frame(slot);
	scene.draws.ring.begin_frame(slot);
	scene.instances.ring.begin_frame(slot);
	scene.draws.open = false;
	scene.instances.open = false;
	// the draws which aren't instanced read the identity at the start of the window from the first draw of the frame on
	open_instance_window();
	// the occlusion queries of the lamps the slot made the last time
	{
		auto &submodels{scene.occlusion_submodels[slot]};
		if (false == submodels.empty())
		{
			std::vector<std::uint32_t> results(submodels.size(), 1);
			if (vkGetQueryPoolResults(device, scene.occlusion_pools[slot], 0, static_cast<std::uint32_t>(submodels.size()), results.size() * sizeof(std::uint32_t), results.data(), sizeof(std::uint32_t), 0) == VK_SUCCESS)
			{
				for (std::size_t i = 0; i < submodels.size(); ++i)
				{
					scene.glare_visible[submodels[i]] = (results[i] > 0);
				}
			}
			vkResetQueryPool(device, scene.occlusion_pools[slot], 0, static_cast<std::uint32_t>(submodels.size()));
			submodels.clear();
		}
	}

	VkCommandBufferBeginInfo begin{};
	begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	VK14_VERIFY(vkBeginCommandBuffer(frame.uploads, &begin));
	VK14_VERIFY(vkBeginCommandBuffer(frame.commands, &begin));
	auto const value{ctx.timelinevalue + 1};
	ctx.uploads.begin_frame(frame.uploads, value);
	ctx.slot = slot;
	ctx.commands = frame.commands;
	// the copies of the light buffer were in the part of the ring of another frame
	m_lightheaders.clear();
	// the content other threads gave the textures (python)
	{
		std::vector<vk14::vulkan14_texture *> pending;
		{
			std::lock_guard<std::mutex> lock(ctx.pending.mutex);
			pending.swap(ctx.pending.textures);
		}
		for (auto *texture : pending)
		{
			texture->apply_pending();
		}
	}

	ctx.acquired.clear();
	for (auto &window : ctx.windows)
	{
		switch (window->acquire(slot))
		{
		case vk14::swapchain::acquire_result::ready:
			ctx.acquired.push_back(window.get());
			break;
		case vk14::swapchain::acquire_result::skipped:
			break;
		default:
			if (window != ctx.windows.front())
			{
				// a window of an extra viewport which can't be shown doesn't stop the others
				break;
			}
			ctx.acquired.clear();
			report_device_lost("acquiring an image");
			return false;
		}
	}

	frame.number = ctx.framecount;
	ctx.passes.begin_frame(frame.commands, slot, ctx.framecount);

	// the stand-ins put in place the first time: the empty maps cleared
	if (false == scene.images_ready)
	{
		scene.images_ready = true;
		ctx.passes.begin_pass("frame setup", {vk14::use_of(*scene.shadow_empty, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_PIPELINE_STAGE_2_CLEAR_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT),
		                                      vk14::use_of(*scene.env_empty, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_PIPELINE_STAGE_2_CLEAR_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT)});
		VkClearDepthStencilValue const farthest{0.f, 0};
		VkImageSubresourceRange const depthrange{VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, gl::MAX_CASCADES};
		vkCmdClearDepthStencilImage(frame.commands, scene.shadow_empty->handle(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &farthest, 1, &depthrange);
		VkClearColorValue const black{{0.f, 0.f, 0.f, 0.f}};
		VkImageSubresourceRange const colourrange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 6};
		vkCmdClearColorImage(frame.commands, scene.env_empty->handle(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &black, 1, &colourrange);
		std::vector<vk14::image_use> uses{vk14::use_of(*scene.shadow_empty, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access),
		                                  vk14::use_of(*scene.env_empty, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access)};
		// the maps which aren't drawn yet are read before they are (the pass sets hold them)
		if (scene.shadow_map)
		{
			uses.push_back(vk14::use_of(*scene.shadow_map, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access));
		}
		if (scene.env_map)
		{
			uses.push_back(vk14::use_of(*scene.env_map, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access));
		}
		vk14::barriers(frame.commands, uses);
		// no light is assigned to any cluster until the culling pass runs
		vkCmdFillBuffer(frame.commands, scene.clusters.ranges.handle(), 0, VK_WHOLE_SIZE, 0);
		VkMemoryBarrier2 barrier{};
		barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
		barrier.srcStageMask = VK_PIPELINE_STAGE_2_CLEAR_BIT;
		barrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
		barrier.dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
		barrier.dstAccessMask = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
		VkDependencyInfo dependency{};
		dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
		dependency.memoryBarrierCount = 1;
		dependency.pMemoryBarriers = &barrier;
		vkCmdPipelineBarrier2(frame.commands, &dependency);
		ctx.passes.end_pass();
	}

	// fetch simulation data
	if (simulation::is_ready)
	{
		static_cast<basic_light &>(m_sunlight) = Global.DayLight;
		// quantize sun angle to reduce shadow crawl
		auto const quantizationstep{0.004f};
		m_sunlight.direction = glm::normalize(quantizationstep * glm::roundEven(m_sunlight.direction * (1.f / quantizationstep)));
		m_fogcolor = Global.FogColor;
		// lamps of models and vehicles around the camera and the cab lamp, for the views of the frame to take into their light buffers
		if ((false == Global.gfx_skiprendering) && (simulation::Region != nullptr))
		{
			m_locallights.gather(Global.pCamera.Pos, *simulation::Region, m_geometry, m_framestamp);
		}
	}
	else
	{
		// a scenery is being loaded: the cells, instances and models the lights were taken from may be gone
		m_locallights.reset();
		scene.glare_visible.clear();
	}
	if ((EditorModeFlag) || (m_nodestateversion != scene::node_state_version))
	{
		// the editor moves and resizes model instances in place, events show and hide them: the instance records are out of date
		++m_instancerecordsversion;
		m_nodestateversion = scene::node_state_version;
	}
	m_renderpass.draw_mode = rendermode::none;
	// the vehicles which moved in the last frame (motion blur)
	m_vehiclesmoved = m_vehiclesmoving;
	m_vehiclesmoving = false;
	m_renderpass.draw_stats = debug_stats();
	m_draw->draws = 0;

	auto const drawscene{simulation::is_ready && (false == Global.gfx_skiprendering)};
	if (drawscene)
	{
		render_scene();
		if (m_pickcontrolswanted && simulation::Train)
		{
			render_picking(rendermode::pickcontrols);
		}
		if (m_picknodewanted)
		{
			render_picking(rendermode::pickscenery);
		}
	}
	m_pickcontrolswanted = false;
	m_picknodewanted = false;

	// the user interface after the scene, as in the opengl renderer: imgui makes its frame, the images it asks for are uploaded in
	// this frame
	Timer::subsystem.gfx_gui.start();
	Application.render_ui();
	Timer::subsystem.gfx_gui.stop();

	// the windows: the image of their viewport, the user interface over the main one
	bool uidrawn{false};
	for (auto *window : ctx.acquired)
	{
		std::size_t viewport{m_viewports.size()};
		for (std::size_t i = 0; i < m_viewports.size(); ++i)
		{
			if (ctx.windows[m_viewports[i].window].get() == window)
			{
				viewport = i;
				break;
			}
		}
		auto const ismain{window == ctx.windows.front().get()};
		ctx.passes.begin_pass(ismain ? "ui" : "other viewports", {window->use(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, colour_stage, colour_access)});
		VkClearValue clear{};
		clear.color = {{0.f, 0.f, 0.f, 1.f}};
		auto attachment{attachment_info(window->view(), VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, true, clear)};
		VkRenderingInfo rendering{};
		rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
		rendering.renderArea = {{0, 0}, window->extent()};
		rendering.layerCount = 1;
		rendering.colorAttachmentCount = 1;
		rendering.pColorAttachments = &attachment;
		vkCmdBeginRendering(frame.commands, &rendering);
		if (drawscene && (viewport < scene.targets.size()) && scene.targets[viewport].output)
		{
			present_viewport(viewport);
		}
		if (ismain)
		{
			ctx.imgui.record(frame.commands, window->extent(), window->format(), window->srgb());
			uidrawn = true;
		}
		vkCmdEndRendering(frame.commands);
		ctx.passes.end_pass();
		vk14::barriers(frame.commands, {window->use(VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, VK_PIPELINE_STAGE_2_NONE, VK_ACCESS_2_NONE)});
	}
	if (false == uidrawn)
	{
		// the main window is minimized: the frame of imgui has nowhere to go
		ctx.imgui.skip();
	}
	// the screenshot asked for: the output of the main viewport, read back once the gpu is done with the frame
	if (scene.screenshot_wanted && drawscene && (false == scene.targets.empty()) && scene.targets.front().output && (false == scene.screenshot_pending))
	{
		scene.screenshot_wanted = false;
		auto &output{*scene.targets.front().output};
		auto const extent{scene.targets.front().extent};
		scene.screenshot = std::make_shared<vk14::buffer>();
		if (scene.screenshot->create(ctx.memory, static_cast<VkDeviceSize>(extent.width) * extent.height * 4, VK_BUFFER_USAGE_TRANSFER_DST_BIT, vk14::memory_kind::readback, "screenshot"))
		{
			vk14::barriers(frame.commands, {vk14::use_of(output, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_READ_BIT)});
			VkBufferImageCopy2 region{};
			region.sType = VK_STRUCTURE_TYPE_BUFFER_IMAGE_COPY_2;
			region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
			region.imageExtent = {extent.width, extent.height, 1};
			VkCopyImageToBufferInfo2 copy{};
			copy.sType = VK_STRUCTURE_TYPE_COPY_IMAGE_TO_BUFFER_INFO_2;
			copy.srcImage = output.handle();
			copy.srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
			copy.dstBuffer = scene.screenshot->handle();
			copy.regionCount = 1;
			copy.pRegions = &region;
			vkCmdCopyImageToBuffer2(frame.commands, &copy);
			host_read_barrier(frame.commands);
			scene.screenshot_pending = true;
			scene.screenshot_value = value;
			scene.screenshot_extent = extent;
		}
	}
	ctx.passes.end_frame();
	auto const uploaded{ctx.uploads.end_frame()};
	VK14_VERIFY(vkEndCommandBuffer(frame.uploads));
	VK14_VERIFY(vkEndCommandBuffer(frame.commands));
	ctx.ring.flush();
	scene.draws.ring.flush();
	scene.instances.ring.flush();
	ctx.commands = VK_NULL_HANDLE;

	std::vector<VkSemaphoreSubmitInfo> waits;
	std::vector<VkSemaphoreSubmitInfo> signals;
	for (auto *window : ctx.acquired)
	{
		VkSemaphoreSubmitInfo wait{};
		wait.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
		wait.semaphore = window->acquire_semaphore(slot);
		wait.stageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
		waits.push_back(wait);

		VkSemaphoreSubmitInfo signal{};
		signal.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
		signal.semaphore = window->present_semaphore();
		signal.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
		signals.push_back(signal);
	}
	VkSemaphoreSubmitInfo timelinesignal{};
	timelinesignal.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
	timelinesignal.semaphore = ctx.timeline;
	timelinesignal.value = value;
	timelinesignal.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
	signals.push_back(timelinesignal);

	// the uploads first, in the same submit: their barriers make them done before the frame uses what they copied
	std::vector<VkCommandBufferSubmitInfo> commands;
	if (uploaded)
	{
		VkCommandBufferSubmitInfo info{};
		info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
		info.commandBuffer = frame.uploads;
		commands.push_back(info);
	}
	{
		VkCommandBufferSubmitInfo info{};
		info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
		info.commandBuffer = frame.commands;
		commands.push_back(info);
	}

	VkSubmitInfo2 submit{};
	submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
	submit.waitSemaphoreInfoCount = static_cast<std::uint32_t>(waits.size());
	submit.pWaitSemaphoreInfos = waits.data();
	submit.commandBufferInfoCount = static_cast<std::uint32_t>(commands.size());
	submit.pCommandBufferInfos = commands.data();
	submit.signalSemaphoreInfoCount = static_cast<std::uint32_t>(signals.size());
	submit.pSignalSemaphoreInfos = signals.data();
	if (!VK14_VERIFY(vkQueueSubmit2(ctx.device.graphics_queue(), 1, &submit, VK_NULL_HANDLE)))
	{
		report_device_lost("submitting a frame");
		return false;
	}
	frame.done = ctx.timelinevalue = value;

	m_laststats = m_colorpass.draw_stats;
	++m_framestamp;
	Timer::subsystem.gfx_total.stop();
	return true;
}

void vulkan14_renderer::render_scene()
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	if (scene.targets.size() < m_viewports.size())
	{
		scene.targets.resize(m_viewports.size());
	}
	for (std::size_t i = 0; i < m_viewports.size(); ++i)
	{
		auto const &viewport{m_viewports[i]};
		auto const srgb{false == Global.gfx_shadergamma};
		if (false == make_targets(ctx.memory, ctx.deletions, ctx.uploads.current_value(), scene.formats_count, ctx.device.caps().limits.framebufferColorSampleCounts & ctx.device.caps().limits.framebufferDepthSampleCounts, scene.targets[i], viewport.width, viewport.height, srgb))
		{
			continue;
		}
		render_viewport(i);
	}
	m_current_viewport = (m_viewports.empty() ? nullptr : &m_viewports.front());
}

// the passes of the walk

void vulkan14_renderer::start_draws(vk14::target_formats const &Targets, int const Width, int const Height, int const Set, bool const Shadows)
{
	auto &draw{*m_draw};
	draw.commands = m_context->commands;
	draw.targets = &Targets;
	draw.extent = {static_cast<std::uint32_t>(Width), static_cast<std::uint32_t>(Height)};
	draw.set = static_cast<vk14::scene_state::pass_set_kind>(Set);
	draw.shadows = Shadows;
	draw.rendering = true;
	draw.set_dirty = true;
	draw.textures_dirty = true;
	draw.pipeline = VK_NULL_HANDLE;
	draw.vertices = VK_NULL_HANDLE;
	draw.userdata = VK_NULL_HANDLE;
	draw.indices = VK_NULL_HANDLE;
	draw.topology = VK_PRIMITIVE_TOPOLOGY_MAX_ENUM;
	draw.cull = VK_CULL_MODE_FLAG_BITS_MAX_ENUM;
	draw.depthtest = -1;
	draw.depthwrite = -1;
	draw.pushed_draw = ~0u;
	draw.block = ~0u;
	draw.cullface = true;
	draw.glare = false;
	draw.depthmask_off = false;
	vk14::set_draw_state(draw.commands, draw.extent);
	// the faces counter-clockwise in the window of opengl are clockwise in the framebuffer of vulkan, which counts its rows the
	// other way (with the same memory of the images, decision K8)
	vkCmdSetFrontFace(draw.commands, VK_FRONT_FACE_CLOCKWISE);
}

void vulkan14_renderer::end_draws()
{
	auto &draw{*m_draw};
	draw.rendering = false;
	draw.program = nullptr;
}

void vulkan14_renderer::upload_scene_block()
{
	auto &ctx{*m_context};
	auto const slice{ctx.ring.allocate(sizeof(gl::scene_ubs), ctx.device.caps().limits.minUniformBufferOffsetAlignment)};
	if (false == bool(slice))
	{
		return;
	}
	std::memcpy(slice.data, &scene_ubs, sizeof(gl::scene_ubs));
	m_draw->scene_offset = static_cast<std::uint32_t>(slice.offset);
	m_draw->set_dirty = true;
}

void vulkan14_renderer::upload_light_header()
{
	// the header with the lights in use: what changes between the draws (the sun in tunnels, the ambient light of the cab) goes
	// with a copy of the lights, the shaders read the buffer whole
	auto &ctx{*m_context};
	// the lights stay the same between the calls of update_lights(): a header with the sun given already goes again
	auto const *header{reinterpret_cast<std::uint8_t const *>(&light_data)};
	for (auto const &[copy, offset] : m_lightheaders)
	{
		if (0 == std::memcmp(copy.data(), header, copy.size()))
		{
			if (m_draw && (m_draw->light_offset != offset))
			{
				m_draw->light_offset = offset;
				m_draw->set_dirty = true;
			}
			return;
		}
	}
	auto const size{offsetof(gl::light_data, lights) + sizeof(gl::light_element) * std::max<std::uint32_t>(m_lightcount, 1)};
	auto const slice{ctx.ring.allocate(size, ctx.device.caps().limits.minStorageBufferOffsetAlignment)};
	if (false == bool(slice))
	{
		return;
	}
	std::memcpy(slice.data, &light_data, size);
	if (m_lightheaders.size() < 32)
	{
		auto &entry{m_lightheaders.emplace_back()};
		std::memcpy(entry.first.data(), header, entry.first.size());
		entry.second = static_cast<std::uint32_t>(slice.offset);
	}
	if (m_draw)
	{
		m_draw->light_offset = static_cast<std::uint32_t>(slice.offset);
		m_draw->set_dirty = true;
	}
}

void vulkan14_renderer::upload_lights()
{
	upload_light_header();
}

void vulkan14_renderer::setup_matrices()
{
	OpenGLMatrices.mode(GL_PROJECTION);
	OpenGLMatrices.load_matrix(m_renderpass.pass_camera.projection());
	// trim modelview matrix just to rotation, since rendering is done in camera-centric world space
	OpenGLMatrices.mode(GL_MODELVIEW);
	OpenGLMatrices.load_matrix(glm::mat4{glm::mat3{m_renderpass.pass_camera.modelview()}});
}

void vulkan14_renderer::setup_shadow_matrices()
{
	if (false == Global.gfx_shadowmap_enabled)
	{
		return;
	}
	for (std::size_t idx = 0; idx < m_shadowpass.size(); ++idx)
	{
		glm::mat4 depthproj = m_shadowpass[idx].pass_camera.projection();
		// NOTE: we strip transformations from camera projections to remove jitter that occurs
		// with large (and unneded as we only need the offset) transformations back and forth
		auto const depthcam{glm::mat3{m_shadowpass[idx].pass_camera.modelview()}};
		auto const worldcam{glm::mat3{m_renderpass.pass_camera.modelview()}};
		scene_ubs.lightview[idx] = coordmove * depthproj * glm::translate(glm::mat4{depthcam}, glm::vec3{m_renderpass.pass_camera.position() - m_shadowpass[idx].pass_camera.position()}) * glm::mat4{glm::inverse(worldcam)};
	}
}

// the camera of the pass, as opengl46_renderer::setup_pass() sets it up (without the vr and the cascades fitted to the depth)
void vulkan14_renderer::setup_pass(viewport_config &Viewport, renderpass_config &Config, rendermode const Mode, float const Znear, float const Zfar, bool const Ignoredebug)
{
	Config.draw_mode = Mode;

	if (false == simulation::is_ready)
	{
		return;
	}
	switch (Mode)
	{
	case rendermode::color:
		Config.draw_range = Global.BaseDrawRange * Global.fDistanceFactor * Viewport.draw_range;
		break;
	case rendermode::shadows:
		Config.draw_range = Global.shadowtune.range;
		break;
	case rendermode::reflections:
		Config.draw_range = Global.BaseDrawRange * Viewport.draw_range;
		break;
	case rendermode::pickcontrols:
		Config.draw_range = 50.f;
		break;
	case rendermode::pickscenery:
		Config.draw_range = Global.BaseDrawRange * Viewport.draw_range * 0.5f;
		break;
	default:
		Config.draw_range = 0.f;
		break;
	}

	auto &camera = Config.pass_camera;
	glm::dmat4 viewmatrix(1.0);
	glm::mat4 frustumtest_proj;
	Config.viewport_camera.position() = Global.pCamera.Pos;

	auto const editor_ortho_projection = [](viewport_config const &Viewport, float const Zfar, glm::mat4 &Projection, glm::mat4 &Frustum) {
		if (false == (EditorModeFlag && Global.EditorOrtho && Viewport.main))
		{
			return;
		}
		auto const height{Global.EditorOrthoExtent};
		auto const width{height * Global.window_size.x / std::max(1.f, static_cast<float>(Global.window_size.y))};
		Projection = ortho_projection(-width, width, -height, height, -Zfar, Zfar);
		Frustum = ortho_frustumtest_projection(-width, width, -height, height, -Zfar, Zfar);
	};

	switch (Mode)
	{
	case rendermode::color:
	{
		if ((Viewport.proj_type == viewport_config::normal) && Viewport.main)
		{
			float const fovy = glm::radians(Global.FieldOfView / Global.ZoomFactor);
			// setup virtual screen
			glm::vec2 screen_h = glm::vec2(Global.window_size) / 2.0f;
			float const dist = screen_h.y / glm::tan(fovy / 2.0f);
			Viewport.projection.pa = glm::vec3(-screen_h.x, -screen_h.y, -dist);
			Viewport.projection.pb = glm::vec3(screen_h.x, -screen_h.y, -dist);
			Viewport.projection.pc = glm::vec3(-screen_h.x, screen_h.y, -dist);
			Viewport.projection.pe = glm::vec3(0.0f, 0.0f, 0.0f);
		}
		if (Global.headtrack_conf.magic_window)
		{
			Viewport.projection.pe = Global.viewport_move;
		}
		TCamera *cam = ((false == DebugCameraFlag) || (true == Ignoredebug)) ? &Global.pCamera : &Global.pDebugCamera;
		camera.position() = cam->Pos;
		cam->SetMatrix(viewmatrix);
		if (!Global.headtrack_conf.magic_window)
		{
			camera.position() += Global.viewport_move * glm::mat3(viewmatrix);
			viewmatrix = glm::dmat4(glm::inverse(Global.viewport_rotate)) * viewmatrix;
		}
		// optimization: don't bother drawing things completely blended with the background
		// but ensure at least 2 km range to account for signals and lights
		Config.draw_range = std::min(Config.draw_range, m_fogrange * 2);
		Config.draw_range = std::max(2000.f, Config.draw_range);
		auto zfar = (Zfar > 1.f ? Zfar : Config.draw_range * Zfar);
		auto const znear = (Znear > 1.f ? Znear : Znear > 0.f ? Znear * zfar : 0.1f * Global.ZoomFactor);
		if (EditorModeFlag && Viewport.main)
		{
			zfar = editor_track_view_range(zfar, true);
		}
		Config.znear = znear;
		Config.zfar = zfar;
		camera.projection() = perspective_projection(Viewport.projection, znear, zfar, frustumtest_proj);
		editor_ortho_projection(Viewport, zfar, camera.projection(), frustumtest_proj);
		break;
	}
	case rendermode::shadows:
	{
		auto const zfar = (Zfar > 1.f ? Zfar : Zfar * Global.shadowtune.range * std::max(Zfar, Global.ZoomFactor * 0.5f));
		std::size_t bounding_viewports_count = 0;
		std::vector<glm::vec4> frustumchunkshapepoints;
		glm::dvec3 pass_camera{0.0};
		for (auto &conf : m_viewports)
		{
			if (!conf.shadow)
			{
				continue;
			}
			renderpass_config worldview;
			setup_pass(conf, worldview, rendermode::color, Znear, zfar, true);
			auto &points = worldview.pass_camera.frustum_points();
			frustumchunkshapepoints.insert(frustumchunkshapepoints.end(), points.begin(), points.end());
			pass_camera += worldview.pass_camera.position();
			bounding_viewports_count++;
		}
		pass_camera /= std::max<std::size_t>(bounding_viewports_count, 1);
		// ...modelview matrix: determine the centre of frustum chunk in world space...
		glm::vec3 frustumchunkmin, frustumchunkmax;
		bounding_box(frustumchunkmin, frustumchunkmax, std::begin(frustumchunkshapepoints), std::end(frustumchunkshapepoints));
		auto const frustumchunkcentre = (frustumchunkmin + frustumchunkmax) * 0.5f;
		// ...cap the vertical angle to keep shadows from getting too long...
		auto const lightvector{glm::normalize(glm::vec3{m_sunlight.direction.x, std::min(m_sunlight.direction.y, Global.gfx_shadow_angle_min), m_sunlight.direction.z})};
		// ...place the light source at the calculated centre and setup world space light view matrix...
		camera.position() = pass_camera + glm::dvec3{frustumchunkcentre};
		viewmatrix *= glm::lookAt(camera.position(), camera.position() + glm::dvec3{lightvector}, glm::dvec3{0.f, 1.f, 0.f});
		// ...projection matrix: calculate boundaries of the frustum chunk in light space...
		auto const lightviewmatrix = glm::translate(glm::mat4{glm::mat3{viewmatrix}}, -frustumchunkcentre);
		for (auto &point : frustumchunkshapepoints)
		{
			point = lightviewmatrix * point;
		}
		bounding_box(frustumchunkmin, frustumchunkmax, std::begin(frustumchunkshapepoints), std::end(frustumchunkshapepoints));
		// quantize the frustum points and add some padding, to reduce shadow shimmer on scale changes
		auto const frustumchunkdepth{zfar - (Znear > 1.f ? Znear : 0.f)};
		auto const quantizationstep{std::min(frustumchunkdepth, 50.f)};
		frustumchunkmin = quantizationstep * glm::floor(frustumchunkmin * (1.f / quantizationstep));
		frustumchunkmax = quantizationstep * glm::ceil(frustumchunkmax * (1.f / quantizationstep));
		// ...use the dimensions to set up light projection boundaries...
		camera.projection() = ortho_projection(frustumchunkmin.x, frustumchunkmax.x, frustumchunkmin.y, frustumchunkmax.y, frustumchunkmin.z - 500.f, frustumchunkmax.z + 500.f);
		frustumtest_proj = ortho_frustumtest_projection(frustumchunkmin.x, frustumchunkmax.x, frustumchunkmin.y, frustumchunkmax.y, frustumchunkmin.z - 500.f, frustumchunkmax.z + 500.f);
		// ... and adjust the projection to sample complete shadow map texels
		auto const shadowbuffersize{static_cast<float>(m_context->scene.shadow_size)};
		auto shadowmaptexel = glm::vec2{camera.projection() * glm::mat4{viewmatrix} * glm::vec4{0.f, 0.f, 0.f, 1.f}};
		shadowmaptexel *= shadowbuffersize * 0.5f;
		auto shadowmapadjustment = glm::round(shadowmaptexel) - shadowmaptexel;
		shadowmapadjustment /= shadowbuffersize * 0.5f;
		camera.projection() = glm::translate(glm::mat4{1.f}, glm::vec3{shadowmapadjustment, 0.f}) * camera.projection();
		break;
	}
	case rendermode::pickcontrols:
	case rendermode::pickscenery:
	{
		camera.position() = Global.pCamera.Pos;
		Global.pCamera.SetMatrix(viewmatrix);
		viewport_proj_config proj = Viewport.projection;
		float znear = 0.1f * Global.ZoomFactor;
		float zfar = Config.draw_range * Global.fDistanceFactor;
		if (EditorModeFlag && Viewport.main && Mode == rendermode::pickscenery)
		{
			zfar = editor_track_view_range(zfar, true);
		}
		camera.projection() = perspective_projection(proj, znear, zfar, frustumtest_proj);
		editor_ortho_projection(Viewport, zfar, camera.projection(), frustumtest_proj);
		break;
	}
	case rendermode::reflections:
	{
		// the faces of the map are drawn over a few frames. each is drawn from the place the camera was at
		// when the update began, so they fit together along the edges when the camera moves in the meantime
		camera.position() = m_environmentupdatelocation;
		glm::dvec3 const cubefacetargetvectors[6] = {{1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, -1.0}};
		glm::dvec3 const cubefaceupvectors[6] = {{0.0, -1.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, -1.0}, {0.0, -1.0, 0.0}, {0.0, -1.0, 0.0}};
		auto const cubefaceindex = m_environmentcubetextureface;
		viewmatrix *= glm::lookAt(camera.position(), camera.position() + cubefacetargetvectors[cubefaceindex], cubefaceupvectors[cubefaceindex]);
		float znear = 0.1f * Global.ZoomFactor;
		float zfar = Config.draw_range * Global.fDistanceFactor;
		viewport_proj_config proj({{-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f}, {0.0f, 0.0f, 0.0f}});
		camera.projection() = perspective_projection(proj, znear, zfar, frustumtest_proj);
		break;
	}
	default:
		break;
	}
	camera.modelview() = viewmatrix;
	camera.update_frustum(frustumtest_proj);
}

// the colour pass of a viewport (opengl46_renderer::Render_pass(), rendermode::color)
void vulkan14_renderer::render_viewport(std::size_t const Index)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &vp{m_viewports[Index]};
	auto &targets{scene.targets[Index]};
	auto const commands{ctx.commands};
	m_current_viewport = &vp;

	setup_pass(vp, m_renderpass, rendermode::color);
	m_colorpass = m_renderpass;

	scene_ubs.time = Timer::GetTime();
	{
		float percipitation_intensity = glm::saturate(Global.Overcast - 1.);
		scene_ubs.rain_params.x = percipitation_intensity;
		scene_ubs.rain_params.y = glm::mix(15., 1., percipitation_intensity);
		if (TDynamicObject const *owner = Global.pCamera.m_owner; owner && !!owner->MoverParameters->CabActive)
		{
			for (int i = 0; i < 4; ++i)
			{
				if (i < owner->dWiperPos.size())
				{
					int index = owner->MoverParameters->CabActive > 0 ? i : static_cast<int>(owner->dWiperPos.size() - 1) - i;
					scene_ubs.wiper_pos[i] = owner->dWiperPos[index];
					if (owner->dWiperPos[index] > 0. && owner->wiperDirection[index])
					{
						scene_ubs.wiper_pos[i] += 1.;
					}
					if (owner->dWiperPos[index] < .025)
					{
						scene_ubs.wiper_timer_out[i] = scene_ubs.time;
					}
					if (owner->dWiperPos[index] > .975)
					{
						scene_ubs.wiper_timer_return[i] = scene_ubs.time;
					}
				}
				else
				{
					scene_ubs.wiper_pos[i] = 0.;
					scene_ubs.wiper_timer_out[i] = -1000.;
					scene_ubs.wiper_timer_return[i] = -1000.;
				}
			}
		}
		else
		{
			scene_ubs.wiper_pos = glm::vec4{0.};
			scene_ubs.wiper_timer_out = glm::vec4{-1000.};
			scene_ubs.wiper_timer_return = glm::vec4{-1000.};
		}
	}
	// the stages after D: no contact shadows, no cascades fitted to the depth, no shadow maps of headlights
	scene_ubs.contact_shadow_params = glm::vec4{0.f};
	scene_ubs.contact_shadow_lamps = glm::vec4{0.f};
	scene_ubs.shadow_fit = glm::vec4{0.f};

	if (Global.gfx_shadowmap_enabled && vp.main && scene.shadow_map)
	{
		Timer::subsystem.gfx_shadows.start();
		m_renderpass.draw_stats = {};
		render_shadows(vp);
		m_renderpass = m_colorpass;
		Timer::subsystem.gfx_shadows.stop();
	}
	if (Global.gfx_envmap_enabled && vp.main && scene.env_map)
	{
		m_renderpass.draw_stats = {};
		Timer::subsystem.gfx_reflections.start();
		if (render_reflections(vp))
		{
			m_renderpass = m_colorpass;
		}
		Timer::subsystem.gfx_reflections.stop();
	}

	// the motion blur only when something in the view moves (see opengl46_renderer::Render_pass())
	auto const motionblur{targets.motion && (m_vehiclesmoved || ((Global.pCamera.m_owner != nullptr) && (Global.pCamera.m_owner->get_future_movement() != glm::dvec3{0.0})))};

	setup_matrices();
	m_renderpass.draw_stats = {};
	model_ubs.future = glm::mat4();
	glm::mat4 future;
	if (Global.pCamera.m_owner != nullptr)
	{
		auto const *vehicle = Global.pCamera.m_owner;
		glm::mat4 mv = OpenGLMatrices.data(GL_MODELVIEW);
		future = glm::translate(mv, -glm::vec3(vehicle->get_future_movement())) * glm::inverse(mv);
	}

	// the light clusters of the viewport (the buffers of the grid are taken by each viewport in turn), and the lights of the pass
	auto const clusters{Global.gfx_lights_clustered || Global.gfx_lights_clusterheatmap};
	ctx.passes.begin_pass("clear, clusters, lights");
	if (clusters)
	{
		update_clusters(glm::ivec2{vp.width, vp.height});
	}
	auto const clustered{clusters && Global.gfx_lights_clustered && (scene.clusters.params.valid != 0)};
	update_lights(simulation::Lights, true, clustered);

	// the colour of the scene: drawn to the images with the samples, resolved to the single ones
	auto const uses{colour_target_uses(targets)};
	ctx.passes.begin_pass("sky", uses);
	begin_colour_rendering(Index, true);
	// as in the opengl renderer, the shadow map and the environment map are those of the main view only
	start_draws(targets.formats, static_cast<int>(targets.extent.width), static_cast<int>(targets.extent.height), vp.main ? vk14::scene_state::set_colour : vk14::scene_state::set_plain, false);
	upload_light_header();
	if (clustered)
	{
		m_draw->cluster_offset = static_cast<std::uint32_t>(scene.clusters.params_offset);
	}

	// the sky
	walk_blending(true);
	scene_ubs.projection = OpenGLMatrices.data(GL_PROJECTION);
	scene_ubs.inv_view = glm::inverse(glm::mat4{glm::mat3{m_colorpass.pass_camera.modelview()}});
	upload_scene_block();
	render_environment(&simulation::Environment);

	setup_shadow_matrices();
	upload_scene_block();

	// opaque parts...
	ctx.passes.begin_pass("colour, opaque");
	walk_blending(false);
	// without rain/snow we can render the cab early to limit the overdraw
	if (!FreeFlyModeFlag && Global.Overcast <= 1.0f && Global.render_cab && simulation::Train)
	{
		model_ubs.future = glm::mat4();
		auto const *vehicle{simulation::Train->Dynamic()};
		// the lamp lighting the shadows of the sun, unless it is a light of its own
		if ((vehicle->InteriorLightLevel > 0.f) && (false == m_cablamplit))
		{
			setup_shadow_color(glm::min(colors::white, m_shadowcolor + glm::vec4(vehicle->InteriorLight * vehicle->InteriorLightLevel, 1.f)));
		}
		Render_cab(vehicle, vehicle->InteriorLightLevel, false);
	}
	setup_shadow_color(m_shadowcolor);
	model_ubs.future = future;
	Render(simulation::Region);

	// ...translucent parts
	ctx.passes.begin_pass("colour, translucent and cab");
	walk_blending(true);
	Render_Alpha(simulation::Region);
	render_editor_overlay();
	// particles
	render_particles();
	// precipitation; done at end, only before cab render
	render_precipitation();
	// cab render
	if ((false == FreeFlyModeFlag) && Global.render_cab && simulation::Train)
	{
		model_ubs.future = glm::mat4();
		auto *vehicle{simulation::Train->Dynamic()};
		if ((vehicle->InteriorLightLevel > 0.f) && (false == m_cablamplit))
		{
			setup_shadow_color(glm::min(colors::white, m_shadowcolor + glm::vec4(vehicle->InteriorLight * vehicle->InteriorLightLevel, 1.f)));
		}
		if (Global.Overcast > 1.0f)
		{
			// with active precipitation draw the opaque cab parts here to mask rain/snow placed 'inside' the cab
			walk_blending(false);
			Render_cab(vehicle, vehicle->InteriorLightLevel, false);
			Render_interior(false);
			walk_blending(true);
			Render_interior(true);
		}
		Render_cab(vehicle, vehicle->InteriorLightLevel, true);
		setup_shadow_color(m_shadowcolor);
		if (Global.Overcast > 1.0f)
		{
			// with the cab in place we can (finally) safely draw translucent part of the occupied vehicle
			Render_Alpha(vehicle);
		}
	}
	vkCmdEndRendering(commands);
	end_draws();
	// the images resolved and drawn are read by the post-processing
	m_colorpass.draw_stats = m_renderpass.draw_stats;

	render_post(Index, motionblur);
}

// the rendering of the colour of a viewport: begun with the images cleared, or again after the depth under the cursor was read
void vulkan14_renderer::begin_colour_rendering(std::size_t const Index, bool const Clear)
{
	auto &ctx{*m_context};
	auto &targets{ctx.scene.targets[Index]};
	auto const msaa{targets.samples != VK_SAMPLE_COUNT_1_BIT};
	VkClearValue colourclear{};
	colourclear.color = {{0.f, 0.f, 0.f, 0.f}};
	VkClearValue depthclear{};
	depthclear.depthStencil = {0.f, 0};
	std::array<VkRenderingAttachmentInfo, 2> colours{};
	colours[0] = attachment_info((msaa ? targets.msaa_colour : targets.colour)->view(), VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, Clear, colourclear);
	if (msaa)
	{
		colours[0].resolveMode = VK_RESOLVE_MODE_AVERAGE_BIT;
		colours[0].resolveImageView = targets.colour->view();
		colours[0].resolveImageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
	}
	if (targets.motion)
	{
		colours[1] = attachment_info((msaa ? targets.msaa_motion : targets.motion_image)->view(), VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, Clear, colourclear);
		if (msaa)
		{
			colours[1].resolveMode = VK_RESOLVE_MODE_AVERAGE_BIT;
			colours[1].resolveImageView = targets.motion_image->view();
			colours[1].resolveImageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		}
	}
	auto depth{attachment_info((msaa ? targets.msaa_depth : targets.depth)->view(), VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, Clear, depthclear)};
	if (msaa)
	{
		// the depth of the first sample, for the motion blur and the cursor of the editor
		depth.resolveMode = VK_RESOLVE_MODE_SAMPLE_ZERO_BIT;
		depth.resolveImageView = targets.depth->view();
		depth.resolveImageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
	}
	VkRenderingInfo rendering{};
	rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
	rendering.renderArea = {{0, 0}, targets.extent};
	rendering.layerCount = 1;
	rendering.colorAttachmentCount = targets.formats.colours;
	rendering.pColorAttachments = colours.data();
	rendering.pDepthAttachment = &depth;
	vkCmdBeginRendering(ctx.commands, &rendering);
}

// the depth of the ground under the cursor of the editor: the rendering is ended (which resolves the depth), the depth under the
// cursor copied, and the rendering begun again where it was
void vulkan14_renderer::walk_editor_cursor()
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &draw{*m_draw};
	if ((m_current_viewport == nullptr) || (false == m_current_viewport->main) || (false == draw.rendering) || scene.cursor.pending || scene.targets.empty())
	{
		return;
	}
	auto &targets{scene.targets.front()};
	auto const commands{ctx.commands};
	glm::vec2 mousepos = Global.cursor_pos;
	// cursor coordinates are flipped compared to opengl, whose rows the images keep
	mousepos.y = Global.window_size.y - mousepos.y;
	auto bufferpos{glm::ivec2(mousepos * glm::vec2(targets.extent.width, targets.extent.height) / glm::max(glm::vec2(Global.window_size), glm::vec2(1.f)))};
	bufferpos = glm::clamp(bufferpos, glm::ivec2(0, 0), glm::ivec2(targets.extent.width - 1, targets.extent.height - 1));

	vkCmdEndRendering(commands);
	auto readback{std::make_shared<vk14::buffer>()};
	if (readback->create(ctx.memory, sizeof(float), VK_BUFFER_USAGE_TRANSFER_DST_BIT, vk14::memory_kind::readback, "cursor depth"))
	{
		vk14::barriers(commands, {vk14::use_of(*targets.depth, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_READ_BIT)});
		VkBufferImageCopy2 region{};
		region.sType = VK_STRUCTURE_TYPE_BUFFER_IMAGE_COPY_2;
		region.imageSubresource = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 0, 1};
		region.imageOffset = {bufferpos.x, bufferpos.y, 0};
		region.imageExtent = {1, 1, 1};
		VkCopyImageToBufferInfo2 copy{};
		copy.sType = VK_STRUCTURE_TYPE_COPY_IMAGE_TO_BUFFER_INFO_2;
		copy.srcImage = targets.depth->handle();
		copy.srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
		copy.dstBuffer = readback->handle();
		copy.regionCount = 1;
		copy.pRegions = &region;
		vkCmdCopyImageToBuffer2(commands, &copy);
		host_read_barrier(commands);
		scene.cursor.data = readback;
		scene.cursor.value = ctx.timelinevalue + 1;
		scene.cursor.pending = true;
		scene.cursor.position = bufferpos;
		vk14::barriers(commands, {vk14::use_of(*targets.depth, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, resolve_depth_stage, resolve_depth_access)});
	}
	// what the rendering stored before is loaded again
	vk14::barriers(commands, colour_target_uses(targets));
	begin_colour_rendering(0, false);
	// the dynamic state starts anew with the rendering
	auto const set{draw.set};
	auto const scene_offset{draw.scene_offset};
	auto const light_offset{draw.light_offset};
	auto const cluster_offset{draw.cluster_offset};
	auto const blending{draw.blending};
	auto *program{draw.program};
	auto const textures{draw.textures};
	start_draws(targets.formats, static_cast<int>(targets.extent.width), static_cast<int>(targets.extent.height), set, false);
	draw.scene_offset = scene_offset;
	draw.light_offset = light_offset;
	draw.cluster_offset = cluster_offset;
	draw.blending = blending;
	draw.program = program;
	draw.textures = textures;
}

// the cascades of the sun shadows
void vulkan14_renderer::render_shadows(viewport_config &Viewport)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto const commands{ctx.commands};
	setup_pass(Viewport, m_renderpass, rendermode::shadows);
	static_assert(gl::MAX_CASCADES == 3, "the fixed cascades are made for 3 of them");
	std::array<float, gl::MAX_CASCADES + 1> csmstageboundaries{0.0f, Global.shadowtune.range / 32, Global.shadowtune.range / 8, Global.shadowtune.range};
	if (Global.shadowtune.map_size > 2048)
	{
		// increase coverage if our shadow map is large enough to produce decent results
		for (std::size_t idx = 1; idx < csmstageboundaries.size(); ++idx)
		{
			csmstageboundaries[idx] *= Global.shadowtune.map_size / 2048;
		}
	}
	static char const *const parts[] = {"sun shadows 1", "sun shadows 2", "sun shadows 3"};
	for (std::size_t idx = 0; idx < m_shadowpass.size(); ++idx)
	{
		ctx.passes.begin_pass(parts[idx], {vk14::use_of(*scene.shadow_map, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, depth_stage, depth_access)});
		setup_pass(Viewport, m_renderpass, rendermode::shadows, csmstageboundaries[idx], csmstageboundaries[idx + 1]);
		m_shadowpass[idx] = m_renderpass;
		setup_matrices();
		scene_ubs.projection = OpenGLMatrices.data(GL_PROJECTION);
		auto const csmstagezfar{(csmstageboundaries[idx + 1]) + (m_shadowpass.size() - idx)};
		scene_ubs.cascade_end[idx] = csmstagezfar * csmstagezfar;

		VkClearValue clear{};
		clear.depthStencil = {0.f, 0};
		auto depth{attachment_info(scene.shadow_layers[idx], VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, true, clear)};
		VkRenderingInfo rendering{};
		rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
		rendering.renderArea = {{0, 0}, {scene.shadow_size, scene.shadow_size}};
		rendering.layerCount = 1;
		rendering.pDepthAttachment = &depth;
		vkCmdBeginRendering(commands, &rendering);
		start_draws(scene.shadow_formats, static_cast<int>(scene.shadow_size), static_cast<int>(scene.shadow_size), vk14::scene_state::set_plain, true);
		walk_blending(false);
		upload_scene_block();
		if (m_shadowcolor != colors::white)
		{
			Render(simulation::Region);
			// render cab only in the closest csm stage
			if ((idx == 0) && !FreeFlyModeFlag && Global.render_cab && simulation::Train)
			{
				Render_cab(simulation::Train->Dynamic(), 0.0f, false);
				Render_cab(simulation::Train->Dynamic(), 0.0f, true);
			}
		}
		vkCmdEndRendering(commands);
		end_draws();
		ctx.passes.end_pass();
	}
	vk14::barriers(commands, {vk14::use_of(*scene.shadow_map, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access)});
}

// the environment map, a face of it per frame (opengl46_renderer::Render_reflections())
bool vulkan14_renderer::render_reflections(viewport_config &Viewport)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	if (Global.reflectiontune.update_interval == 0)
	{
		return false;
	}
	if (m_environmentupdateface >= 6)
	{
		// no update in progress, check whether it's time for another one
		auto const timestamp{Timer::GetRenderTime()};
		if ((timestamp - m_environmentupdatetime < Global.reflectiontune.update_interval) && (glm::length2(m_renderpass.pass_camera.position() - m_environmentupdatelocation) < sq(1000.0)))
		{
			return false;
		}
		m_environmentupdatetime = timestamp;
		m_environmentupdatelocation = m_renderpass.pass_camera.position();
		m_environmentupdateface = 0;
	}
	// the first update is done in one go, as there's nothing to show in the meantime
	auto const lastface{m_environmentready ? m_environmentupdateface : 5};
	ctx.passes.begin_pass("reflections", {vk14::use_of(*scene.env_map, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, colour_stage, colour_access),
	                                      vk14::use_of(*scene.env_depth, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, depth_stage, depth_access)});
	for (; m_environmentupdateface <= lastface; ++m_environmentupdateface)
	{
		// the depth of the faces is the same image: the clear of a face waits for the draws of the one before
		vk14::barriers(ctx.commands, {vk14::use_of(*scene.env_depth, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, depth_stage, depth_access)});
		m_environmentcubetextureface = m_environmentupdateface;
		render_reflection_face(Viewport);
	}
	if (m_environmentupdateface >= 6)
	{
		// the mipmaps, each level from the one before it
		auto const commands{ctx.commands};
		auto &image{*scene.env_map};
		auto const mips{image.desc().mips};
		auto const size{static_cast<std::int32_t>(image.desc().extent.width)};
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
			entry.image = image.handle();
			entry.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, Level, Count, 0, 6};
			VkDependencyInfo dependency{};
			dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
			dependency.imageMemoryBarrierCount = 1;
			dependency.pImageMemoryBarriers = &entry;
			vkCmdPipelineBarrier2(commands, &dependency);
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
			info.srcImage = image.handle();
			info.srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
			info.dstImage = image.handle();
			info.dstImageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
			info.regionCount = 1;
			info.pRegions = &blit;
			info.filter = VK_FILTER_LINEAR;
			vkCmdBlitImage2(commands, &info);
			barrier(level, 1, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
		}
		barrier(0, mips, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_READ_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT, sampled_stage, sampled_access);
		image.state = vk14::image_state{VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access};
		m_environmentready = true;
	}
	else
	{
		vk14::barriers(ctx.commands, {vk14::use_of(*scene.env_map, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access)});
	}
	return true;
}

void vulkan14_renderer::render_reflection_face(viewport_config &Viewport)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto const commands{ctx.commands};
	auto const size{scene.env_map->desc().extent.width};
	setup_pass(Viewport, m_renderpass, rendermode::reflections);

	VkClearValue colourclear{};
	colourclear.color = {{0.f, 0.f, 0.f, 1.f}};
	VkClearValue depthclear{};
	depthclear.depthStencil = {0.f, 0};
	auto colour{attachment_info(scene.env_faces[m_environmentcubetextureface], VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, true, colourclear)};
	auto depth{attachment_info(scene.env_depth->view(), VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, true, depthclear)};
	VkRenderingInfo rendering{};
	rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
	rendering.renderArea = {{0, 0}, {size, size}};
	rendering.layerCount = 1;
	rendering.colorAttachmentCount = 1;
	rendering.pColorAttachments = &colour;
	rendering.pDepthAttachment = &depth;
	vkCmdBeginRendering(commands, &rendering);
	start_draws(scene.reflection_formats, static_cast<int>(size), static_cast<int>(size), vk14::scene_state::set_reflections, false);

	setup_matrices();
	// the light buffer holds what the last color pass left in it; the lights go in anew, without the headlights
	update_lights(simulation::Lights, false);
	upload_light_header();
	// environment...
	walk_blending(true);
	scene_ubs.projection = OpenGLMatrices.data(GL_PROJECTION);
	scene_ubs.inv_view = glm::inverse(glm::mat4{glm::mat3{m_renderpass.pass_camera.modelview()}});
	upload_scene_block();
	render_environment(&simulation::Environment);
	// opaque parts...
	walk_blending(false);
	setup_shadow_matrices();
	upload_scene_block();
	Render(simulation::Region);

	vkCmdEndRendering(commands);
	end_draws();
}

// the sky dome, the sun and the moon (opengl46_renderer::Render(world_environment *) without the atmosphere, the stars and the
// weather of the later stages of the plan)
void vulkan14_renderer::render_environment(world_environment *Environment)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &draw{*m_draw};
	m_shadowcolor = colors::white; // prevent shadow from affecting sky
	setup_shadow_color(m_shadowcolor);
	if (Global.bWireFrame)
	{
		return;
	}
	walk_material(null_handle);
	::glPushMatrix();
	auto const fogdensity{model_ubs.fog_density};
	model_ubs.fog_density = 0.f;

	// skydome, drawn with 500m radius to blend in if the fog range is low
	auto &skydome{simulation::Environment.skydome()};
	skydome.is_dirty() = false;
	if (false == skydome.indices().empty())
	{
		::glPushMatrix();
		::glScalef(500.0f, 500.0f, 500.0f);
		model_ubs.set_modelview(OpenGLMatrices.data(GL_MODELVIEW));
		auto const &vertices{skydome.vertices()};
		auto const &colours{skydome.colors()};
		auto const &indices{skydome.indices()};
		auto const vertexslice{ctx.ring.allocate(vertices.size() * sizeof(glm::vec3), 16)};
		auto const colourslice{ctx.ring.allocate(colours.size() * sizeof(glm::vec3), 16)};
		auto const indexslice{ctx.ring.allocate(indices.size() * sizeof(std::uint16_t), 16)};
		if (vertexslice && colourslice && indexslice && bind_scene_pipeline(*scene.skydome, *draw.targets, draw.blending ? vk14::blend_mode::scene_alpha : vk14::blend_mode::none, VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST))
		{
			std::memcpy(vertexslice.data, vertices.data(), vertexslice.size);
			std::memcpy(colourslice.data, colours.data(), colourslice.size);
			std::memcpy(indexslice.data, indices.data(), indexslice.size);
			apply_draw_state();
			draw.block = push_draw_block();
			if (draw.block != ~0u)
			{
				if (draw.set_dirty)
				{
					bind_pass_set();
				}
				if (draw.textures_dirty)
				{
					push_textures();
				}
				vk14::draw_constants constants;
				constants.draw = draw.block;
				push_constants(&constants);
				VkBuffer const buffers[2]{vertexslice.buffer, colourslice.buffer};
				VkDeviceSize const offsets[2]{vertexslice.offset, colourslice.offset};
				vkCmdBindVertexBuffers(draw.commands, 0, 2, buffers, offsets);
				vkCmdBindIndexBuffer(draw.commands, indexslice.buffer, indexslice.offset, VK_INDEX_TYPE_UINT16);
				vkCmdDrawIndexed(draw.commands, static_cast<std::uint32_t>(indices.size()), 1, 0, 0, 0);
				draw.vertices = VK_NULL_HANDLE;
				draw.userdata = VK_NULL_HANDLE;
				draw.indices = VK_NULL_HANDLE;
			}
		}
		::glPopMatrix();
	}

	auto const &modelview = OpenGLMatrices.data(GL_MODELVIEW);
	// the bodies give their colour premultiplied, and as alpha how much of the sky behind them they hide (celestial.frag)
	auto const &sun{Environment->sun()};
	auto const &moon{Environment->moon()};
	// without the clouds the overcast fades the sun and the moon
	auto const overcastfactor{std::clamp(1.5f - Global.Overcast, 0.f, 1.f)};
	float const duskfactor = 1.0f - std::clamp(std::abs(sun.getAngle()), 0.0f, 12.0f) / 12.0f;
	glm::vec3 suncolor = glm::mix(glm::vec3(1.0f, 242.0f / 255.0f, 231.0f / 255.0f), glm::vec3(235.0f / 255.0f, 140.0f / 255.0f, 36.0f / 255.0f), duskfactor);
	glm::vec2 zenith{glm::mat3{modelview} * glm::vec3{0.f, 1.f, 0.f}};
	zenith = (glm::length(zenith) > 1e-3f ? glm::normalize(zenith) : glm::vec2{0.f, 1.f});
	auto const flattening = [](float const Elevation, float const Radius) {
		auto const refraction = [](float const Height) {
			auto const height{std::max(Height, -2.f)};
			return 1.f / std::tan(glm::radians(height + 7.31f / (height + 4.4f)));
		};
		auto const radius{glm::degrees(Radius)};
		return std::clamp(1.f - (refraction(Elevation - radius) - refraction(Elevation + radius)) / (2.f * radius * 60.f), 0.5f, 1.f);
	};
	auto const mainview{(m_renderpass.draw_mode == rendermode::color) && (m_current_viewport != nullptr) && m_current_viewport->main};
	auto const draw_body = [&](std::array<glm::vec4, 3> const &Body) {
		if (false == bind_scene_pipeline(*scene.celestial, *draw.targets, vk14::blend_mode::premultiplied, VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP))
		{
			return;
		}
		apply_draw_state();
		draw.block = push_draw_block();
		if (draw.block == ~0u)
		{
			return;
		}
		if (draw.set_dirty)
		{
			bind_pass_set();
		}
		if (draw.textures_dirty)
		{
			push_textures();
		}
		vk14::draw_constants constants;
		constants.draw = draw.block;
		for (std::size_t i = 0; i < Body.size(); ++i)
		{
			constants.data[i] = Body[i];
		}
		push_constants(&constants);
		vkCmdDraw(draw.commands, 4, 1, 0, 0);
	};
	std::array<glm::vec4, 3> body{};
	// sun
	{
		float const sunradius{glm::radians(0.2666f)};
		float const quadmargin{1.25f};
		glm::vec3 const limbdarkening{0.50f, 0.60f, 0.75f};
		float const limbpeak{1.f / (1.f - 0.75f / 3.f)};
		float const radiancemax{60000.f};
		auto const sunvector = sun.getDirection();
		glm::vec3 radiance{suncolor * overcastfactor};
		if (mainview)
		{
			auto const solidangle{2.f * glm::pi<float>() * (1.f - std::cos(sunradius))};
			auto const hidden{glm::smoothstep(0.5f, 1.f, Global.Overcast)};
			auto const physical{glm::vec3{m_sunlight.diffuse} * (1.5f * glm::pi<float>() / solidangle * Global.gfx_sun_disc)};
			radiance = glm::mix(physical, radiance, hidden);
			auto const peak{std::max({radiance.r, radiance.g, radiance.b}) * limbpeak};
			if (peak > radiancemax)
			{
				radiance *= radiancemax / peak;
			}
		}
		body[0] = glm::vec4{zenith, flattening(sun.getAngle(), sunradius), 1.f / quadmargin};
		body[1] = glm::vec4{0.f};
		body[2] = glm::vec4{limbdarkening, 0.f};
		model_ubs.param[0] = glm::vec4{radiance, 1.f};
		model_ubs.param[1] = glm::vec4(glm::vec3(modelview * glm::vec4(sunvector, 1.0f)), std::tan(sunradius) * quadmargin);
		model_ubs.param[2] = glm::vec4(0.f, 0.f, 0.f, 1.f);
		draw_body(body);
	}
	// moon
	{
		glm::vec2 const discposition{0.5f, 0.5f};
		float const discradius{0.1635f};
		float const quadmargin{1.1f};
		float const earthshine{0.005f};
		glm::vec3 const extinction{0.10f, 0.17f, 0.30f};
		walk_texture(0, m_moontexture);
		glm::vec3 mooncolor(1.0f, 0.9490196f, 0.9058824f);
		float visibility = std::max<float>(0.f, 1.0 - 0.5 * Global.fLuminance - 0.65 * std::max(0.f, glm::dot(sun.getDirection(), moon.getDirection()))) * overcastfactor;
		if (mainview)
		{
			visibility = 1.4f * overcastfactor;
		}
		auto const moonvector = moon.getDirection();
		auto const moonradius{moon.getAngularRadius()};
		auto const moonelevation{moon.getAngle()};
		auto const moonview{glm::vec3(modelview * glm::vec4(moonvector, 1.0f))};
		if (moon.getPhase() == 50)
		{
			model_ubs.param[0] = glm::vec4(mooncolor, visibility);
			model_ubs.param[1] = glm::vec4(moonview, std::tan(moonradius) * (0.1665f / discradius));
			model_ubs.param[2] = glm::vec4(0.66f, 1.0f - 0.667f, 0.333f, 0.f);
		}
		else
		{
			auto const zenithangle{90.f - std::max(moonelevation, -1.f)};
			auto const airmass{1.f / (std::cos(glm::radians(zenithangle)) + 0.50572f * std::pow(std::max(96.07995f - zenithangle, 0.1f), -1.6364f))};
			auto const transmittance{glm::exp(-extinction * (airmass - 1.f))};
			auto const night{1.f - glm::smoothstep(-15.f, -9.f, sun.getAngle())};
			auto const earthphase{0.5f * (1.f - std::cos(glm::radians(moon.getPhaseAngle())))};
			body[0] = glm::vec4{zenith, flattening(moonelevation, moonradius), 1.f / quadmargin};
			body[1] = glm::vec4{glm::normalize(glm::mat3{modelview} * sun.getDirection()), earthshine * earthphase};
			body[2] = glm::vec4{0.f};
			model_ubs.param[0] = glm::vec4(mooncolor * transmittance * (visibility * Global.gfx_moon_disc), night * overcastfactor);
			model_ubs.param[1] = glm::vec4(moonview, std::tan(moonradius) * quadmargin);
			model_ubs.param[2] = glm::vec4(discposition, discradius, 2.f);
		}
		draw_body(body);
	}
	::glPopMatrix();
	model_ubs.fog_density = fogdensity;

	// calculate shadow tone, based on positions of celestial bodies
	m_shadowcolor = glm::mix(glm::vec4{colors::shadow}, glm::vec4{colors::white}, std::clamp(-sun.getAngle(), 0.f, 6.f) / 6.f);
	if ((sun.getAngle() < -18.f) && (moon.getAngle() > 0.f))
	{
		// turn on moon shadows after astronomical twilight, if the moon is actually up
		auto const moonshadow{std::clamp(4.f * moon.getBrightness() * Global.gfx_moonlight, 0.f, 1.f) * std::clamp((-18.f - sun.getAngle()) / 2.f, 0.f, 1.f) * std::clamp(moon.getAngle() / 5.f, 0.f, 1.f)};
		m_shadowcolor = glm::mix(glm::vec4{colors::white}, glm::vec4{colors::shadow}, moonshadow);
	}
	// soften shadows depending on sky overcast factor (the shadows of the clouds come with the stage P2)
	m_shadowcolor = glm::min(colors::white, m_shadowcolor + ((colors::white - colors::shadow) * Global.Overcast));
	setup_shadow_color(m_shadowcolor);
}

void vulkan14_renderer::push_constants(void const *Constants)
{
	auto &draw{*m_draw};
	VkPushConstantsInfo info{};
	info.sType = VK_STRUCTURE_TYPE_PUSH_CONSTANTS_INFO;
	info.layout = m_context->scene.scene_layout;
	info.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
	info.offset = 0;
	info.size = sizeof(vk14::draw_constants);
	info.pValues = Constants;
	vkCmdPushConstants2(draw.commands, &info);
	draw.pushed_draw = ~0u;
}

// the smoke of the cpu (opengl46_particles)
void vulkan14_renderer::render_particles()
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &draw{*m_draw};
	model_ubs.set_modelview(OpenGLMatrices.data(GL_MODELVIEW));
	walk_texture(0, m_smoketexture);
	// the particles of the gpu come with the stage P6
	simulation::Particles.gpu(false);
	if (false == Global.Smoke)
	{
		return;
	}
	auto const &camera{m_renderpass.pass_camera};
	std::multimap<float, smoke_source const &> sources;
	for (auto const &source : simulation::Particles.sequence())
	{
		if (false == camera.visible(source.area()))
		{
			continue;
		}
		sources.emplace(static_cast<float>(glm::length(camera.position() - source.area().center) - source.area().radius), source);
	}
	if (sources.empty())
	{
		return;
	}
	std::vector<particle_vertex> vertices;
	auto const camerarotation{glm::mat3(camera.modelview())};
	particle_vertex vertex;
	for (auto const &[sourcekey, sourcedata] : sources)
	{
		auto const particlecolor{glm::clamp(sourcedata.color() * (glm::vec3{Global.DayLight.ambient} + 0.35f * glm::vec3{Global.DayLight.diffuse}) * simulation::Environment.light_intensity(), glm::vec3{0.f}, glm::vec3{1.f})};
		for (auto const &particle : sourcedata.sequence())
		{
			vertex.color[0] = particlecolor.r;
			vertex.color[1] = particlecolor.g;
			vertex.color[2] = particlecolor.b;
			vertex.color.a = std::clamp(particle.opacity, 0.0f, 1.0f) * sourcedata.fade_in(particle.age);
			auto const offset{glm::vec3{particle.position - camera.position()}};
			auto const rotation{glm::angleAxis(particle.rotation, glm::vec3{0.f, 0.f, 1.f})};
			for (auto const &[billboardoffset, billboardtexture] : billboard_vertices)
			{
				vertex.position = offset + (rotation * billboardoffset * particle.size) * camerarotation;
				vertex.texture = billboardtexture;
				vertices.emplace_back(vertex);
			}
		}
	}
	m_renderpass.draw_stats.particles = static_cast<int>(vertices.size() / 6);
	auto const slice{ctx.ring.allocate(vertices.size() * sizeof(particle_vertex), 16)};
	if ((false == bool(slice)) || (false == bind_scene_pipeline(*scene.smoke, *draw.targets, vk14::blend_mode::scene_alpha, VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)))
	{
		return;
	}
	std::memcpy(slice.data, vertices.data(), slice.size);
	apply_draw_state();
	draw.block = push_draw_block();
	if (draw.block == ~0u)
	{
		return;
	}
	if (draw.set_dirty)
	{
		bind_pass_set();
	}
	if (draw.textures_dirty)
	{
		push_textures();
	}
	vk14::draw_constants constants;
	constants.draw = draw.block;
	push_constants(&constants);
	VkDeviceSize const offset{slice.offset};
	vkCmdBindVertexBuffers(draw.commands, 0, 1, &slice.buffer, &offset);
	vkCmdDraw(draw.commands, static_cast<std::uint32_t>(vertices.size()), 1, 0, 0);
	draw.vertices = VK_NULL_HANDLE;
	draw.userdata = VK_NULL_HANDLE;
}

// the rain and the snow of the cpu (opengl46_renderer::Render_precipitation(), opengl46_precipitation)
void vulkan14_renderer::render_precipitation()
{
	if (Global.Overcast <= 1.f)
	{
		return;
	}
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &draw{*m_draw};
	if (m_precipitationovercast != Global.Overcast)
	{
		m_precipitationovercast = Global.Overcast;
		std::string const densitysuffix{(m_precipitationovercast < 1.35 ? "_light" : "_medium")};
		if (Global.Weather == "rain:")
		{
			m_precipitationtexture = Fetch_Texture("fx/rain" + densitysuffix);
		}
		else if (Global.Weather == "snow:")
		{
			m_precipitationtexture = Fetch_Texture("fx/snow" + densitysuffix);
		}
	}
	::glPushMatrix();
	// tilt the precipitation cone against the velocity vector for crude motion blur, include current wind vector while at it
	auto const velocity{simulation::Environment.precipitation().m_cameramove * -1.0 + glm::dvec3{simulation::Environment.wind()} * 0.5};
	if (glm::length2(velocity) > 0.0)
	{
		auto const forward{glm::normalize(velocity)};
		auto left{glm::cross(forward, {0.0, 1.0, 0.0})};
		auto const rotationangle{std::min(45.0, ((FreeFlyModeFlag || simulation::Train == nullptr) ? 5 * glm::length(velocity) : simulation::Train->Dynamic()->GetVelocity() * 0.2))};
		::glRotated(rotationangle, left.x, 0.0, left.z);
	}
	if ((false == FreeFlyModeFlag) && (simulation::Train != nullptr))
	{
		// counter potential vehicle roll
		auto const roll{0.5 * glm::degrees(simulation::Train->Dynamic()->Roll())};
		if (roll != 0.0)
		{
			auto const forward{simulation::Train->Dynamic()->VectorFront()};
			::glRotated(roll, forward.x, 0.0, forward.z);
		}
	}
	if (!Global.iPause)
	{
		// oddly enough random streaks produce more natural looking rain than ones the eye can follow
		m_precipitationrotation = (Global.Weather == "rain:" ? LocalRandom() * 360 : 0.0);
	}
	::glRotated(m_precipitationrotation, 0.0, 1.0, 0.0);
	model_ubs.set_modelview(OpenGLMatrices.data(GL_MODELVIEW));
	model_ubs.param[0] = glm::mix(0.5f * (Global.DayLight.diffuse + Global.DayLight.ambient), colors::white, 0.5f * std::clamp((float)Global.fLuminance, 0.f, 1.f));
	model_ubs.param[1].x = simulation::Environment.precipitation().get_textureoffset();
	walk_texture(0, m_precipitationtexture);

	auto const &mesh{precipitation()};
	auto const vertexslice{ctx.ring.allocate(mesh.vertices.size() * sizeof(glm::vec3), 16)};
	auto const uvslice{ctx.ring.allocate(mesh.uvs.size() * sizeof(glm::vec2), 16)};
	auto const indexslice{ctx.ring.allocate(mesh.indices.size() * sizeof(std::uint16_t), 16)};
	if (vertexslice && uvslice && indexslice && bind_scene_pipeline(*scene.precipitation, *draw.targets, draw.blending ? vk14::blend_mode::scene_alpha : vk14::blend_mode::none, VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST))
	{
		std::memcpy(vertexslice.data, mesh.vertices.data(), vertexslice.size);
		std::memcpy(uvslice.data, mesh.uvs.data(), uvslice.size);
		std::memcpy(indexslice.data, mesh.indices.data(), indexslice.size);
		apply_draw_state();
		draw.block = push_draw_block();
		if (draw.block != ~0u)
		{
			if (draw.set_dirty)
			{
				bind_pass_set();
			}
			if (draw.textures_dirty)
			{
				push_textures();
			}
			vk14::draw_constants constants;
			constants.draw = draw.block;
			push_constants(&constants);
			VkBuffer const buffers[2]{vertexslice.buffer, uvslice.buffer};
			VkDeviceSize const offsets[2]{vertexslice.offset, uvslice.offset};
			vkCmdBindVertexBuffers(draw.commands, 0, 2, buffers, offsets);
			vkCmdBindIndexBuffer(draw.commands, indexslice.buffer, indexslice.offset, VK_INDEX_TYPE_UINT16);
			vkCmdDrawIndexed(draw.commands, static_cast<std::uint32_t>(mesh.indices.size()), 1, 0, 0, 0);
			draw.vertices = VK_NULL_HANDLE;
			draw.userdata = VK_NULL_HANDLE;
			draw.indices = VK_NULL_HANDLE;
		}
	}
	::glPopMatrix();
}

// the lines the editor draws over the scene (opengl46_renderer::Render_editor_overlay())
void vulkan14_renderer::render_editor_overlay()
{
	if (false == EditorModeFlag || EditorOverlay.batches.empty())
	{
		return;
	}
	if (m_editor_overlay_revision != EditorOverlay.revision)
	{
		m_editor_overlay_revision = EditorOverlay.revision;
		if (m_editor_overlay_bank == null_handle)
		{
			m_editor_overlay_bank = Create_Bank();
		}
		auto const &batches{EditorOverlay.batches};
		for (std::size_t i = 0; i < batches.size(); ++i)
		{
			gfx::vertex_array vertices;
			vertices.reserve(batches[i].points.size());
			for (auto const &point : batches[i].points)
			{
				vertices.emplace_back(point, glm::vec3(0.f, 1.f, 0.f), glm::vec2());
			}
			gfx::userdata_array userdata;
			if (i >= m_editor_overlay_geometry.size())
			{
				m_editor_overlay_geometry.emplace_back();
			}
			auto &geometry{m_editor_overlay_geometry[i]};
			if (geometry != null_handle)
			{
				m_geometry.replace(vertices, userdata, geometry);
			}
			else if (false == vertices.empty())
			{
				geometry = m_geometry.create_chunk(vertices, userdata, m_editor_overlay_bank, batches[i].type);
			}
		}
	}
	auto &draw{*m_draw};
	::glPushMatrix();
	auto const origin{EditorOverlay.origin - m_renderpass.pass_camera.position()};
	::glTranslated(origin.x, origin.y, origin.z);
	draw.cullface = false;
	draw.depthmask_off = true;
	walk_program(walk_shader::line);
	auto const &batches{EditorOverlay.batches};
	for (std::size_t i = 0; i < batches.size() && i < m_editor_overlay_geometry.size(); ++i)
	{
		if (batches[i].points.empty() || m_editor_overlay_geometry[i] == null_handle)
		{
			continue;
		}
		model_ubs.param[0] = batches[i].color;
		walk_draw(m_editor_overlay_geometry[i]);
		++m_renderpass.draw_stats.drawcalls;
	}
	draw.depthmask_off = false;
	draw.cullface = true;
	::glPopMatrix();
}

// the post-processing of a viewport: from the colour of the scene to its output, which the window shows
void vulkan14_renderer::render_post(std::size_t const Index, bool const Motionblur)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &targets{scene.targets[Index]};
	auto const commands{ctx.commands};
	auto const extent{targets.extent};

	auto const post = [&](char const *Fragment, vk14::image &Target, std::vector<vk14::image *> const &Inputs, vk14::draw_constants const &Constants, char const *Pass) {
		std::vector<vk14::image_use> uses{vk14::use_of(Target, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, colour_stage, colour_access)};
		for (auto *input : Inputs)
		{
			uses.push_back(vk14::use_of(*input, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, sampled_access));
		}
		if (Pass != nullptr)
		{
			ctx.passes.begin_pass(Pass, uses);
		}
		else
		{
			vk14::barriers(commands, uses);
		}
		auto const pipeline{post_pipeline(Fragment, static_cast<int>(Target.desc().format))};
		if (pipeline == nullptr)
		{
			return;
		}
		VkClearValue clear{};
		auto attachment{attachment_info(Target.view(), VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, false, clear)};
		attachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
		VkRenderingInfo rendering{};
		rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
		rendering.renderArea = {{0, 0}, extent};
		rendering.layerCount = 1;
		rendering.colorAttachmentCount = 1;
		rendering.pColorAttachments = &attachment;
		vkCmdBeginRendering(commands, &rendering);
		vk14::set_draw_state(commands, extent);
		vkCmdBindPipeline(commands, VK_PIPELINE_BIND_POINT_GRAPHICS, reinterpret_cast<VkPipeline>(pipeline));
		vkCmdSetPrimitiveTopology(commands, VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP);
		std::array<VkDescriptorImageInfo, 4> images;
		for (std::size_t i = 0; i < images.size(); ++i)
		{
			auto *input{i < Inputs.size() ? Inputs[i] : Inputs.front()};
			// a depth isn't filtered (linear filtering of the depth formats isn't required of the devices)
			auto const depth{input->desc().format == VK_FORMAT_D32_SFLOAT};
			images[i] = VkDescriptorImageInfo{depth ? ctx.sampling.clamp_nearest : ctx.sampling.clamp_linear, input->view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
		}
		scene.post_set.push(commands, scene.post_layout, images.data());
		VkPushConstantsInfo info{};
		info.sType = VK_STRUCTURE_TYPE_PUSH_CONSTANTS_INFO;
		info.layout = scene.post_layout;
		info.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
		info.size = sizeof(Constants);
		info.pValues = &Constants;
		vkCmdPushConstants2(commands, &info);
		vkCmdDraw(commands, 4, 1, 0, 0);
		vkCmdEndRendering(commands);
	};

	auto *hdr{targets.colour.get()};
	if (Motionblur && targets.blurred)
	{
		vk14::draw_constants constants;
		constants.data[0].x = m_framerate / (1.0 / Global.gfx_postfx_motionblur_shutter);
		post("postfx_motionblur.frag", *targets.blurred, {targets.colour.get(), targets.motion_image.get(), targets.depth.get()}, constants, "resolve, ambient occlusion, motion blur");
		hdr = targets.blurred.get();
	}
	// the exposure and the bloom come with the stage P1: the fixed exposure
	vk14::draw_constants tonemap;
	tonemap.data[0] = glm::vec4{0.f, 0.f, std::exp2(Global.gfx_postfx_autoexposure_compensation), 0.f};
	tonemap.data[1] = glm::vec4{targets.aberration ? 0.f : 1.f, 0.f, 0.f, 0.f};
	if (targets.aberration && targets.tonemapped)
	{
		post("postfx_tonemapping.frag", *targets.tonemapped, {hdr, hdr}, tonemap, "tonemapping, output");
		post("postfx_chromaticaberration.frag", *targets.output, {targets.tonemapped.get()}, vk14::draw_constants{}, nullptr);
	}
	else
	{
		post("postfx_tonemapping.frag", *targets.output, {hdr, hdr}, tonemap, "tonemapping, output");
	}
	vk14::barriers(commands, {vk14::use_of(*targets.output, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, sampled_access)});
}

// the output of a viewport drawn to its window turned upside down (decision K8), inside the rendering of the window
void vulkan14_renderer::present_viewport(std::size_t const Index)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &targets{scene.targets[Index]};
	auto const &window{*ctx.windows[m_viewports[Index].window]};
	auto const commands{ctx.commands};
	// the output encoded already goes as it is; an srgb output to a window which isn't (no srgb format of the surface) is encoded
	auto const pipeline{post_pipeline("present.frag", static_cast<int>(window.format()), (targets.srgb && (false == window.srgb())) ? 1 : 0)};
	if (pipeline == nullptr)
	{
		return;
	}
	vk14::set_draw_state(commands, window.extent());
	vkCmdBindPipeline(commands, VK_PIPELINE_BIND_POINT_GRAPHICS, reinterpret_cast<VkPipeline>(pipeline));
	vkCmdSetPrimitiveTopology(commands, VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP);
	std::array<VkDescriptorImageInfo, 4> images;
	images.fill(VkDescriptorImageInfo{ctx.sampling.clamp_linear, targets.output->view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL});
	scene.post_set.push(commands, scene.post_layout, images.data());
	vk14::draw_constants constants;
	VkPushConstantsInfo info{};
	info.sType = VK_STRUCTURE_TYPE_PUSH_CONSTANTS_INFO;
	info.layout = scene.post_layout;
	info.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
	info.size = sizeof(constants);
	info.pValues = &constants;
	vkCmdPushConstants2(commands, &info);
	vkCmdDraw(commands, 4, 1, 0, 0);
}

// a pipeline of the post-processing: quad.vert with the fragment shader, to an image of the format. the copy to a window has the
// colour encoded already (the output of the viewport is srgb, or encoded by the tonemapping with gfx.shadergamma)
void *vulkan14_renderer::post_pipeline(char const *Fragment, int const Format, int const Encode)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto const lookup{scene.post_fragments.find(Fragment)};
	if (lookup == scene.post_fragments.end())
	{
		return nullptr;
	}
	vk14::pipeline_desc desc;
	desc.stages = {scene.quad, lookup->second};
	desc.layout = scene.post_layout;
	desc.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
	desc.colour_formats = {static_cast<VkFormat>(Format)};
	auto const gamma{Encode >= 0 ? (Encode > 0) : Global.gfx_shadergamma};
	desc.specialization = {gamma ? 1u : 0u, Global.gfx_shadowmap_enabled ? 1u : 0u, Global.gfx_envmap_enabled ? 1u : 0u, Global.gfx_postfx_motionblur_enabled ? 1u : 0u, 1u, Global.gfx_extraeffects ? 1u : 0u};
	desc.name = Fragment;
	return reinterpret_cast<void *>(ctx.pipelines.get(desc));
}

// the lights of the pass (opengl46_renderer::Update_Lights())
void vulkan14_renderer::update_lights(light_array &Lights, bool const Headlights, bool const Clustered)
{
	// arrange the light array from closest to farthest from current position of the camera
	auto const camera = m_colorpass.pass_camera.position();
	if (Headlights)
	{
		std::sort(std::begin(Lights.data), std::end(Lights.data), [&camera](light_array::light_record const &Left, light_array::light_record const &Right) {
			if (Left.intensity == 0.f)
			{
				return false;
			}
			if (Right.intensity == 0.f)
			{
				return true;
			}
			return (glm::length2(camera - Left.position) / Left.intensity) < (glm::length2(camera - Right.position) / Right.intensity);
		});
	}
	glm::mat4 mv = OpenGLMatrices.data(GL_MODELVIEW);
	// the headlights of opengl46_renderer::headlight_config
	float const in_cutoff{1.005f};
	float const out_cutoff{0.993f};
	float const falloff_linear{0.15f};
	float const falloff_quadratic{0.15f};
	float const intensity{1.0f};
	float const ambient{0.0f};

	sun_light renderlight;
	renderlight.is_directional = false;
	std::size_t light_i = 1;
	m_lights_dropped = 0;

	// the lights other than the headlights are only taken from the lists of the clusters. the lamp of the occupied cab goes first
	auto const locallights{Headlights && Clustered};
	m_cablamplit = false;
	if (locallights)
	{
		if (auto const *lamp{m_locallights.cab_lamp()})
		{
			auto &light{light_data.lights[light_i++]};
			write_local_light(light, *lamp, camera, mv);
			m_cablamplit = true;
		}
	}
	auto const headlightsstart{light_i};
	for (auto const &scenelight : Lights.data)
	{
		if (!Headlights || !Global.gfx_lights_headlights || scenelight.intensity == 0.f)
		{
			break;
		}
		auto const lightoffset = glm::vec3{scenelight.position - camera};
		if (glm::length2(lightoffset) > sq(1000.f))
		{
			continue;
		}
		if (light_i >= gl::MAX_LIGHTS)
		{
			++m_lights_dropped;
			continue;
		}
		renderlight.position = lightoffset;
		renderlight.direction = scenelight.direction;
		auto luminance = static_cast<float>(Global.fLuminance);
		auto const environment = scenelight.owner->fShade;
		if (environment > 0.f)
		{
			luminance *= environment;
		}
		renderlight.diffuse = glm::vec4{glm::max(glm::vec3{colors::none}, scenelight.color - glm::vec3{luminance}), renderlight.diffuse[3]};
		renderlight.ambient = glm::vec4{glm::max(glm::vec3{colors::none}, scenelight.color * glm::vec3{scenelight.intensity} - glm::vec3{luminance}), renderlight.ambient[3]};
		renderlight.factor = (scenelight.count * 0.5) * (scenelight.owner->DimHeadlights ? 0.5 : 1.0);

		gl::light_element *light = &light_data.lights[light_i];
		light->pos = mv * glm::vec4(renderlight.position, 1.0f);
		light->dir = mv * glm::vec4(renderlight.direction, 0.0f);
		light->type = gl::light_element::HEADLIGHTS;
		light->in_cutoff = in_cutoff;
		light->out_cutoff = out_cutoff;
		light->color = renderlight.diffuse * renderlight.factor;
		light->linear = falloff_linear / 10.0f;
		light->quadratic = falloff_quadratic / 100.0f;
		light->ambient = ambient;
		light->intensity = intensity;
		auto const worldcam{glm::mat3{m_renderpass.pass_camera.modelview()}};
		{
			glm::dvec3 projector;
			glm::mat3 lightcam;
			glm::mat4 depthproj;
			float nearplane, fieldofview;
			headlight_projector(scenelight, projector, lightcam, depthproj, nearplane, fieldofview);
			light->headlight_projection = coordmove * depthproj * glm::translate(glm::mat4{lightcam}, glm::vec3{m_renderpass.pass_camera.position() - projector}) * glm::mat4{glm::inverse(worldcam)};
		}
		// headlights weights; no shadow maps of the headlights before the stage P4
		light->headlight_weights = {scenelight.state, 0.f};
		++light_i;
	}
	m_lights_lit = static_cast<uint32_t>(light_i - headlightsstart);
	// lamps of models and vehicles, the most important first, those which may be seen lighting something in this view
	if (locallights)
	{
		uint32_t placed{0};
		uint32_t dropped{0};
		for (auto const &local : m_locallights.lights())
		{
			if (false == m_renderpass.pass_camera.visible(scene::bounding_area{local.position, local.range}))
			{
				continue;
			}
			if (light_i >= gl::MAX_LIGHTS)
			{
				++dropped;
				continue;
			}
			auto &light{light_data.lights[light_i++]};
			write_local_light(light, local, camera, mv);
			++placed;
		}
		m_locallights.placed(placed, dropped);
	}
	light_data.lights_total = static_cast<uint32_t>(light_i);
	light_data.lights_count = 1 + std::min<uint32_t>(m_lights_lit, static_cast<uint32_t>(std::max(Global.DynamicLightCount, 0)));
	light_data.lights_clustered = Clustered ? 1 : 0;
	light_data.env_ggx = 0;
	// fill sunlight data
	light_data.ambient = m_sunlight.ambient * m_sunlight.factor;
	light_data.sky_ambient_factor = 0.f;
	light_data.lights[0].type = gl::light_element::DIR;
	light_data.lights[0].dir = mv * glm::vec4(m_sunlight.direction, 0.0f);
	light_data.lights[0].color = m_sunlight.diffuse * m_sunlight.factor * simulation::Environment.light_intensity();
	light_data.lights[0].ambient = 0.0f;
	light_data.lights[0].intensity = 1.0f;
	// fill fog data; the fog of the weather comes with the stage P2
	light_data.fog_color = m_fogcolor;
	if (Global.fFogEnd > 0)
	{
		m_fogrange = std::max(Global.fFogEnd / 1.5f, 5.f);
		model_ubs.fog_density = 1.f / m_fogrange;
	}
	else
	{
		model_ubs.fog_density = 0.0f;
	}
	m_lightcount = static_cast<std::uint32_t>(light_i);
	m_lightheaders.clear();
	if (m_draw && m_draw->rendering)
	{
		upload_light_header();
	}

	// the lights of the clusters: the culling into the lists, for the colour pass which follows
	if (Clustered)
	{
		auto &ctx{*m_context};
		auto &scene{ctx.scene};
		auto &clusters{scene.clusters};
		auto const commands{ctx.commands};
		auto const lightsize{offsetof(gl::light_data, lights) + sizeof(gl::light_element) * m_lightcount};
		auto const lights{ctx.ring.allocate(sizeof(gl::light_data), ctx.device.caps().limits.minStorageBufferOffsetAlignment)};
		if (false == bool(lights))
		{
			return;
		}
		std::memcpy(lights.data, &light_data, lightsize);
		m_draw->light_offset = static_cast<std::uint32_t>(lights.offset);
		vkCmdFillBuffer(commands, clusters.counters.handle(), 0, VK_WHOLE_SIZE, 0);
		VkMemoryBarrier2 barrier{};
		barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
		barrier.srcStageMask = VK_PIPELINE_STAGE_2_CLEAR_BIT | VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
		barrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT | VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
		barrier.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
		barrier.dstAccessMask = VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
		VkDependencyInfo dependency{};
		dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
		dependency.memoryBarrierCount = 1;
		dependency.pMemoryBarriers = &barrier;
		vkCmdPipelineBarrier2(commands, &dependency);
		std::array<VkDescriptorBufferInfo, 6> buffers{{{ctx.ring.handle(), clusters.params_offset, sizeof(gl::cluster_ubs)},
		                                               {clusters.aabbs.handle(), 0, VK_WHOLE_SIZE},
		                                               {clusters.ranges.handle(), 0, VK_WHOLE_SIZE},
		                                               {clusters.indices.handle(), 0, VK_WHOLE_SIZE},
		                                               {clusters.counters.handle(), 0, VK_WHOLE_SIZE},
		                                               {lights.buffer, lights.offset, sizeof(gl::light_data)}}};
		vkCmdBindPipeline(commands, VK_PIPELINE_BIND_POINT_COMPUTE, scene.cluster_cull);
		scene.cluster_set.push(commands, scene.cluster_layout, buffers.data());
		vkCmdDispatch(commands, (gl::CLUSTER_COUNT + gl::CLUSTER_CULL_GROUP_SIZE - 1) / gl::CLUSTER_CULL_GROUP_SIZE, 1, 1);
		barrier.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
		barrier.srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
		barrier.dstStageMask = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_CLEAR_BIT;
		barrier.dstAccessMask = VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT;
		vkCmdPipelineBarrier2(commands, &dependency);
		clusters.lights_total = light_data.lights_total;
	}
}

void vulkan14_renderer::write_local_light(gl::light_element &Light, gfx::local_lights::light const &Source, glm::dvec3 const &Camera, glm::mat4 const &View)
{
	// a cone wider than a half of the space is no limit for the culling, but the shading still fades the light out of it
	Light.type = gl::light_element::SPOT;
	Light.pos = View * glm::vec4(glm::vec3{Source.position - Camera}, 1.0f);
	Light.dir = glm::normalize(glm::vec3{View * glm::vec4(Source.direction, 0.0f)});
	Light.in_cutoff = Source.cos_inner;
	Light.out_cutoff = Source.cos_outer;
	if (Source.cos_outer <= -1.f)
	{
		Light.type = gl::light_element::POINT;
	}
	Light.color = Source.color;
	Light.intensity = 1.0f;
	Light.ambient = 0.0f;
	// the shading takes the light to end at 1 / sqrt(quadratic)
	Light.linear = 0.0f;
	Light.quadratic = 1.0f / sq(std::max(Source.range, 0.01f));
	Light.headlight_projection = glm::mat4{1.0f};
	// no shadow map
	Light.headlight_weights = glm::vec4{-1.0f, 0.0f, 0.0f, 0.0f};
}

void vulkan14_renderer::headlight_projector(light_array::light_record const &Light, glm::dvec3 &Position, glm::mat3 &Rotation, glm::mat4 &Projection, float &Near, float &Fieldofview)
{
	auto const &ownerdimensions{Light.owner->MoverParameters->Dim};
	auto const up{static_cast<glm::dvec3>(Light.owner->VectorUp())};
	auto const size{static_cast<float>(std::max(ownerdimensions.W, ownerdimensions.H) * 1.0)};
	auto const cone{5.f};
	auto const offset{75.f * (5.f / cone)};
	Position = Light.owner->GetPosition() - glm::dvec3(Light.direction * offset) + up * (size * 0.5);
	Near = static_cast<float>(ownerdimensions.L * 0.5 + offset - 0.25);
	Fieldofview = glm::radians(cone);
	Projection = perspective_projection_raw(Fieldofview, 1.0, Near, 1000.0f);
	glm::dmat4 viewmatrix{1.0};
	viewmatrix *= glm::lookAt(Position, Position - up * 1.0 + glm::dvec3{Light.direction * 1000.f}, glm::dvec3{0.f, 1.f, 0.f});
	Rotation = glm::mat3{glm::mat4{viewmatrix}};
}

// the light cluster grid of a viewport for the projection of its colour pass (gl::cluster_grid::update() and build()); the
// viewports take the buffers of the grid in turn, its boxes are made anew when the projection changes
void vulkan14_renderer::update_clusters(glm::ivec2 const &Targetsize)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &clusters{scene.clusters};
	auto const commands{ctx.commands};
	auto const projection{OpenGLMatrices.data(GL_PROJECTION)};
	auto const znear{m_renderpass.znear};
	auto const zfar{m_renderpass.zfar};
	auto params{clusters.params};
	params.viewport_width = static_cast<float>(Targetsize.x);
	params.viewport_height = static_cast<float>(Targetsize.y);
	// clip space w is a function of the position in a perspective projection, and a constant in an orthographic one
	bool const perspective{glm::length(glm::vec3{projection[0][3], projection[1][3], projection[2][3]}) > 1e-4f};
	bool const usable{perspective && znear > 0.f && zfar > znear};
	params.valid = usable ? 1 : 0;
	if (usable && (projection != clusters.params.projection || znear != clusters.params.znear || zfar != clusters.params.zfar))
	{
		params.projection = projection;
		params.znear = znear;
		params.zfar = zfar;
		params.slice_scale = static_cast<float>(gl::CLUSTER_GRID_Z) / std::log(zfar / znear);
		params.slice_bias = -std::log(znear) * params.slice_scale;
		clusters.stale = true;
	}
	clusters.params = params;
	auto const block{ctx.ring.allocate(sizeof(gl::cluster_ubs), ctx.device.caps().limits.minUniformBufferOffsetAlignment)};
	if (false == bool(block))
	{
		clusters.params.valid = 0;
		return;
	}
	std::memcpy(block.data, &clusters.params, sizeof(gl::cluster_ubs));
	clusters.params_offset = block.offset;
	if (usable && clusters.stale)
	{
		// the boxes of the clusters depend only on the projection and the depth range
		std::array<VkDescriptorBufferInfo, 6> buffers{{{block.buffer, block.offset, sizeof(gl::cluster_ubs)},
		                                               {clusters.aabbs.handle(), 0, VK_WHOLE_SIZE},
		                                               {clusters.ranges.handle(), 0, VK_WHOLE_SIZE},
		                                               {clusters.indices.handle(), 0, VK_WHOLE_SIZE},
		                                               {clusters.counters.handle(), 0, VK_WHOLE_SIZE},
		                                               {ctx.ring.handle(), 0, sizeof(gl::light_data)}}};
		VkMemoryBarrier2 barrier{};
		barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
		barrier.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
		barrier.srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_READ_BIT;
		barrier.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
		barrier.dstAccessMask = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
		VkDependencyInfo dependency{};
		dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
		dependency.memoryBarrierCount = 1;
		dependency.pMemoryBarriers = &barrier;
		vkCmdPipelineBarrier2(commands, &dependency);
		vkCmdBindPipeline(commands, VK_PIPELINE_BIND_POINT_COMPUTE, scene.cluster_build);
		scene.cluster_set.push(commands, scene.cluster_layout, buffers.data());
		vkCmdDispatch(commands, (gl::CLUSTER_COUNT + gl::CLUSTER_BUILD_GROUP_SIZE - 1) / gl::CLUSTER_BUILD_GROUP_SIZE, 1, 1);
		barrier.srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
		barrier.dstAccessMask = VK_ACCESS_2_SHADER_STORAGE_READ_BIT;
		vkCmdPipelineBarrier2(commands, &dependency);
		clusters.stale = false;
	}
}

// picking: the colours of the controls of the cab or of the nodes of the scenery, and the one under the cursor read back

void vulkan14_renderer::render_picking(rendermode const Mode)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto const commands{ctx.commands};
	auto &readback{Mode == rendermode::pickcontrols ? scene.pick_controls : scene.pick_scenery};
	if (readback.pending || m_viewports.empty())
	{
		return;
	}
	auto &vp{m_viewports.front()};
	setup_pass(vp, m_renderpass, Mode);
	ctx.passes.begin_pass(Mode == rendermode::pickcontrols ? "pick controls" : "pick scenery", {vk14::use_of(*scene.pick_colour, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, colour_stage, colour_access),
	                                                                                             vk14::use_of(*scene.pick_depth, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, depth_stage, depth_access)});
	VkClearValue colourclear{};
	colourclear.color = {{0.f, 0.f, 0.f, 0.f}};
	VkClearValue depthclear{};
	depthclear.depthStencil = {0.f, 0};
	auto colour{attachment_info(scene.pick_colour->view(), VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, true, colourclear)};
	auto depth{attachment_info(scene.pick_depth->view(), VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, true, depthclear)};
	VkRenderingInfo rendering{};
	rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
	rendering.renderArea = {{0, 0}, {static_cast<std::uint32_t>(pick_buffer_size), static_cast<std::uint32_t>(pick_buffer_size)}};
	rendering.layerCount = 1;
	rendering.colorAttachmentCount = 1;
	rendering.pColorAttachments = &colour;
	rendering.pDepthAttachment = &depth;
	vkCmdBeginRendering(commands, &rendering);
	start_draws(scene.pick_formats, pick_buffer_size, pick_buffer_size, vk14::scene_state::set_plain, false);
	setup_matrices();
	walk_blending(false);
	scene_ubs.projection = OpenGLMatrices.data(GL_PROJECTION);
	upload_scene_block();
	upload_light_header();
	if (Mode == rendermode::pickcontrols)
	{
		m_picksurfaceitems.clear();
		m_pickcontrolsitems.clear();
		Render_cab(simulation::Train->Dynamic(), 0.0f);
		Render(simulation::Train->Dynamic());
	}
	else
	{
		m_picksceneryitems.clear();
		Render(simulation::Region);
	}
	vkCmdEndRendering(commands);
	end_draws();

	auto buffer{std::make_shared<vk14::buffer>()};
	if (buffer->create(ctx.memory, 4, VK_BUFFER_USAGE_TRANSFER_DST_BIT, vk14::memory_kind::readback, "pick colour"))
	{
		vk14::barriers(commands, {vk14::use_of(*scene.pick_colour, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_READ_BIT)});
		VkBufferImageCopy2 region{};
		region.sType = VK_STRUCTURE_TYPE_BUFFER_IMAGE_COPY_2;
		region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
		region.imageOffset = {m_pickposition.x, m_pickposition.y, 0};
		region.imageExtent = {1, 1, 1};
		VkCopyImageToBufferInfo2 copy{};
		copy.sType = VK_STRUCTURE_TYPE_COPY_IMAGE_TO_BUFFER_INFO_2;
		copy.srcImage = scene.pick_colour->handle();
		copy.srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
		copy.dstBuffer = buffer->handle();
		copy.regionCount = 1;
		copy.pRegions = &region;
		vkCmdCopyImageToBuffer2(commands, &copy);
		host_read_barrier(commands);
		readback.data = buffer;
		readback.value = ctx.timelinevalue + 1;
		readback.pending = true;
	}
	ctx.passes.end_pass();
	m_renderpass = m_colorpass;
}

namespace
{
// the place in the image of the picking under the cursor, in the rows of opengl
glm::ivec2 pick_position()
{
	glm::vec2 mousepos = Global.cursor_pos;
	mousepos.y = Global.window_size.y - mousepos.y;
	auto pickbufferpos{glm::ivec2(mousepos * glm::vec2(pick_buffer_size) / glm::max(glm::vec2(Global.window_size), glm::vec2(1.f)))};
	return glm::clamp(pickbufferpos, glm::ivec2(0, 0), glm::ivec2(pick_buffer_size - 1, pick_buffer_size - 1));
}
} // namespace

void vulkan14_renderer::Update_Pick_Control()
{
	if (!m_context)
	{
		return;
	}
	auto &ctx{*m_context};
	auto &readback{ctx.scene.pick_controls};
	if (readback.pending)
	{
		if (ctx.completed() < readback.value)
		{
			return;
		}
		readback.pending = false;
		readback.data->invalidate(0, 4);
		auto const *data{readback.data->mapped()};
		auto const controlindex = pick_index(glm::ivec3{data[0], data[1], data[2]});
		TSubModel const *control{nullptr};
		glm::vec2 position(0.0f);
		if ((controlindex > 0) && (controlindex <= m_pickcontrolsitems.size()))
		{
			control = m_pickcontrolsitems[controlindex - 1];
		}
		else if (255 - data[0] < m_picksurfaceitems.size())
		{
			control = m_picksurfaceitems[255 - data[0]];
			position.x = data[1] / 255.0f;
			position.y = data[2] / 255.0f;
		}
		m_pickcontrolitem = control;
		for (auto const &callback : m_control_pick_requests)
		{
			callback(m_pickcontrolitem, position);
		}
		m_control_pick_requests.clear();
		readback.data.reset();
	}
	if (false == m_control_pick_requests.empty())
	{
		m_pickposition = pick_position();
		m_pickcontrolswanted = true;
	}
}

void vulkan14_renderer::Update_Pick_Node()
{
	if (!m_context)
	{
		return;
	}
	auto &ctx{*m_context};
	auto &readback{ctx.scene.pick_scenery};
	if (readback.pending)
	{
		if (ctx.completed() < readback.value)
		{
			return;
		}
		readback.pending = false;
		readback.data->invalidate(0, 4);
		auto const *data{readback.data->mapped()};
		auto const nodeindex = pick_index(glm::ivec3{data[0], data[1], data[2]});
		scene::basic_node *node{nullptr};
		if ((nodeindex > 0) && (nodeindex <= m_picksceneryitems.size()))
		{
			node = m_picksceneryitems[nodeindex - 1];
		}
		m_picksceneryitem = node;
		for (auto const &callback : m_node_pick_requests)
		{
			callback(m_picksceneryitem);
		}
		m_node_pick_requests.clear();
		readback.data.reset();
	}
	if (false == m_node_pick_requests.empty())
	{
		m_pickposition = pick_position();
		m_picknodewanted = true;
	}
}

void vulkan14_renderer::Pick_Control_Callback(std::function<void(TSubModel const *, glm::vec2 const)> Callback)
{
	if (!Global.render_cab)
	{
		Callback(nullptr, glm::vec2());
		return;
	}
	m_control_pick_requests.push_back(Callback);
}

void vulkan14_renderer::Pick_Node_Callback(std::function<void(scene::basic_node *)> Callback)
{
	m_node_pick_requests.push_back(Callback);
}

TSubModel const *vulkan14_renderer::Pick_Control() const
{
	return m_pickcontrolitem;
}

scene::basic_node const *vulkan14_renderer::Pick_Node() const
{
	return m_picksceneryitem;
}

glm::dvec3 vulkan14_renderer::Mouse_Position() const
{
	return m_worldmousecoordinates;
}

bool vulkan14_renderer::Mouse_Hit() const
{
	return m_mousehit;
}

glm::mat4 vulkan14_renderer::Camera_View_Matrix() const
{
	return glm::mat4(glm::mat3(m_colorpass.pass_camera.modelview()));
}

glm::mat4 vulkan14_renderer::Camera_Projection_Matrix() const
{
	return m_colorpass.pass_camera.projection();
}

glm::dvec3 vulkan14_renderer::Camera_Position() const
{
	return m_colorpass.pass_camera.position();
}

// the position of the cursor of the editor in the scene: the depth under it read back from an earlier frame
glm::dvec3 vulkan14_renderer::Update_Mouse_Position()
{
	if (m_context)
	{
		auto &ctx{*m_context};
		auto &cursor{ctx.scene.cursor};
		if (cursor.pending && (ctx.completed() >= cursor.value) && (false == ctx.scene.targets.empty()))
		{
			cursor.pending = false;
			float pointdepth{0.f};
			cursor.data->invalidate(0, sizeof(float));
			std::memcpy(&pointdepth, cursor.data->mapped(), sizeof(float));
			cursor.data.reset();
			auto const extent{ctx.scene.targets.front().extent};
			m_mousehit = pointdepth > 0.0f;
			if (pointdepth > 0.0f)
			{
				m_worldmousecoordinates = glm::unProjectZO(glm::vec3(cursor.position, pointdepth), glm::mat4(glm::mat3(m_colorpass.pass_camera.modelview())), m_colorpass.pass_camera.projection(),
				                                           glm::vec4(0, 0, extent.width, extent.height));
			}
		}
	}
	return m_colorpass.pass_camera.position() + glm::dvec3{m_worldmousecoordinates};
}

// screenshots: the output of the main viewport copied with the frame, saved once the gpu is done with it
void vulkan14_renderer::MakeScreenshot()
{
	if (m_context)
	{
		m_context->scene.screenshot_wanted = true;
	}
}

void vulkan14_renderer::update_screenshot()
{
	if (!m_context)
	{
		return;
	}
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	if ((false == scene.screenshot_pending) || (ctx.completed() < scene.screenshot_value))
	{
		return;
	}
	scene.screenshot_pending = false;
	auto const width{static_cast<int>(scene.screenshot_extent.width)};
	auto const height{static_cast<int>(scene.screenshot_extent.height)};
	scene.screenshot->invalidate(0, VK_WHOLE_SIZE);
	auto const *rgba{scene.screenshot->mapped()};
	// rgb rows as glReadPixels() of the opengl renderer gives them: the rows of the images are those of opengl (decision K8)
	auto *image = new char[static_cast<std::size_t>(width) * height * 3];
	for (std::size_t pixel = 0; pixel < static_cast<std::size_t>(width) * height; ++pixel)
	{
		image[pixel * 3 + 0] = static_cast<char>(rgba[pixel * 4 + 0]);
		image[pixel * 3 + 1] = static_cast<char>(rgba[pixel * 4 + 1]);
		image[pixel * 3 + 2] = static_cast<char>(rgba[pixel * 4 + 2]);
	}
	scene.screenshot.reset();
	screenshot_manager::save(image, width, height);
}

// textures used only by vehicles farther than gfx.textures.releasedistance times the draw range give their images back
// (opengl46_renderer::Update_Texture_Release())
void vulkan14_renderer::update_texture_release()
{
	if (!m_context || !m_context->textures || (m_colorpass.draw_range <= 0.f))
	{
		return;
	}
	auto &textures{*m_context->textures};
	auto const range{static_cast<double>(Global.gfx_texture_releasedistance) * m_colorpass.draw_range};
	auto const camera{m_colorpass.pass_camera.position()};
	textures.begin_release_scan();
	for (auto *vehicle : simulation::Vehicles.sequence())
	{
		if (vehicle == nullptr)
		{
			continue;
		}
		bool const isnear{glm::length(vehicle->vPosition - camera) - vehicle->radius() <= range};
		auto const mark_model = [&](TModel3d *Model) {
			if (Model == nullptr)
			{
				return;
			}
			for (auto const texture : Model_Textures(Model))
			{
				textures.mark_as_needed(texture, isnear);
			}
		};
		mark_model(vehicle->mdModel);
		mark_model(vehicle->mdLowPolyInt);
		mark_model(vehicle->mdLoad);
		for (auto *attachment : vehicle->mdAttachments)
		{
			mark_model(attachment);
		}
		for (auto const skin : vehicle->Material()->replacable_skins)
		{
			if (skin <= 0)
			{
				continue;
			}
			auto const &material{m_materials.material(skin)};
			for (auto const texture : material.textures)
			{
				textures.mark_as_needed(texture, isnear);
			}
			for (auto const &variant : material.texture_variants)
			{
				for (auto const texture : variant)
				{
					textures.mark_as_needed(texture, isnear);
				}
			}
		}
	}
	textures.release_unneeded();
}

std::vector<texture_handle> const &vulkan14_renderer::Model_Textures(TModel3d *Model)
{
	auto const lookup{m_modeltextures.find(Model)};
	if (lookup != m_modeltextures.end())
	{
		return lookup->second;
	}
	auto &textures{m_modeltextures[Model]};
	std::vector<TSubModel *> submodels;
	if (Model->GetSMRoot() != nullptr)
	{
		submodels.emplace_back(Model->GetSMRoot());
	}
	while (false == submodels.empty())
	{
		auto *submodel{submodels.back()};
		submodels.pop_back();
		if (submodel->NextGet() != nullptr)
		{
			submodels.emplace_back(submodel->NextGet());
		}
		if (submodel->ChildGet() != nullptr)
		{
			submodels.emplace_back(submodel->ChildGet());
		}
		// negative material is one of replaceable skins of the vehicle, these are handled with the vehicle
		if (submodel->GetMaterial() <= 0)
		{
			continue;
		}
		auto const &material{m_materials.material(submodel->GetMaterial())};
		for (auto const texture : material.textures)
		{
			if (texture > 0)
			{
				textures.emplace_back(texture);
			}
		}
		for (auto const &variant : material.texture_variants)
		{
			for (auto const texture : variant)
			{
				if (texture > 0)
				{
					textures.emplace_back(texture);
				}
			}
		}
	}
	std::sort(std::begin(textures), std::end(textures));
	textures.erase(std::unique(std::begin(textures), std::end(textures)), std::end(textures));
	return textures;
}

// a preview of a model (opengl46_renderer::Render_Preview()) isn't made by the vulkan renderer yet: the caller falls back to no
// preview
bool vulkan14_renderer::Render_Preview(TAnimModel *Instance, int const Size, int const Margin, bool const Shadows, std::vector<std::uint8_t> &Image)
{
	return false;
}
