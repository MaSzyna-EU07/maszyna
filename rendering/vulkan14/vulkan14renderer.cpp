/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14renderer.h"
#include "rendering/vulkan14/vulkan14common.h"
#include "rendering/vulkan14/vulkan14instance.h"
#include "rendering/vulkan14/vulkan14device.h"
#include "rendering/vulkan14/vulkan14swapchain.h"
#include "rendering/vulkan14/vulkan14memory.h"
#include "rendering/vulkan14/vulkan14shaders.h"
#include "rendering/vulkan14/vulkan14descriptors.h"
#include "rendering/vulkan14/vulkan14passes.h"
#include "rendering/vulkan14/vulkan14texture.h"
#include "rendering/vulkan14/vulkan14imgui.h"
#include "rendering/vulkan14/vulkan14bindings.h"
#include "rendering/nullrenderer.h"
#include "application/application.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/Timer.h"
#include "utilities/crashreporter.h"
#include "utilities/utilities.h"
#include "imgui/imgui.h"

namespace
{

// the colour the windows are cleared to once a scenery is loaded, until the scenery is drawn (stage D3 of the plan): the initial
// background of the opengl renderer (srgb). before that the windows are black, as with the opengl renderer
glm::vec3 const clear_colour{51.f / 255.f, 102.f / 255.f, 85.f / 255.f};

// the ring each frame in flight streams its data through (the user interface for now)
VkDeviceSize const ring_size{8 * 1024 * 1024};

float srgb_to_linear(float const Value)
{
	return (Value <= 0.04045f ? Value / 12.92f : std::pow((Value + 0.055f) / 1.055f, 2.4f));
}

} // namespace

// the objects of the vulkan api. destroyed in the reverse order: the swapchains and the frames, then the device, then the instance
struct vulkan14_renderer::context
{
	struct frame
	{
		VkCommandPool pool{VK_NULL_HANDLE};
		// the uploads of the frame go to the queue first, then the commands of the frame
		VkCommandBuffer uploads{VK_NULL_HANDLE};
		VkCommandBuffer commands{VK_NULL_HANDLE};
		// the value of the timeline semaphore the gpu reaches when it's done with the frame; 0: never submitted
		std::uint64_t done{0};
		// the number of the frame, for the report of a lost device
		std::uint64_t number{0};
	};

	vk14::instance instance;
	vk14::device device;
	vk14::allocator memory;
	vk14::deletion_queue deletions;
	vk14::uploader uploads;
	vk14::image_table images;
	vk14::samplers sampling;
	vk14::frame_ring ring;
	vk14::shader_library shaders;
	vk14::pipeline_library pipelines;
	vk14::pass_list passes;
	vk14::pending_textures pending;
	vk14::resources resources;
	vk14::imgui_backend imgui;
	// the textures of the files; made after the objects above and gone before them
	std::unique_ptr<texture_bank> textures;
	// [0]: the main window, then the windows of the extra viewports
	std::vector<std::unique_ptr<vk14::swapchain>> windows;
	std::vector<frame> frames;
	// one semaphore counts the frames done by the gpu; the binary ones are only between the swapchains and the queue
	VkSemaphore timeline{VK_NULL_HANDLE};
	std::uint64_t timelinevalue{0};
	// frames recorded since the start
	std::uint64_t framecount{0};
	// the windows with an image acquired in the frame being made
	std::vector<vk14::swapchain *> acquired;
	bool devicelost{false};

	~context()
	{
		destroy();
	}

	bool create_frames(std::uint32_t const Count, std::string &Reason)
	{
		auto const handle{device.handle()};

		VkSemaphoreTypeCreateInfo timelineinfo{};
		timelineinfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO;
		timelineinfo.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE;
		timelineinfo.initialValue = 0;
		VkSemaphoreCreateInfo semaphoreinfo{};
		semaphoreinfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
		semaphoreinfo.pNext = &timelineinfo;
		if (!VK14_VERIFY(vkCreateSemaphore(handle, &semaphoreinfo, nullptr, &timeline)))
		{
			Reason = "no timeline semaphore";
			return false;
		}
		instance.set_object_name(handle, VK_OBJECT_TYPE_SEMAPHORE, reinterpret_cast<std::uint64_t>(timeline), "frame timeline");

		frames.resize(Count);
		for (std::uint32_t i = 0; i < Count; ++i)
		{
			VkCommandPoolCreateInfo poolinfo{};
			poolinfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
			poolinfo.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
			poolinfo.queueFamilyIndex = device.families().graphics;
			if (!VK14_VERIFY(vkCreateCommandPool(handle, &poolinfo, nullptr, &frames[i].pool)))
			{
				Reason = "no command pool";
				return false;
			}
			VkCommandBuffer buffers[2] = {VK_NULL_HANDLE, VK_NULL_HANDLE};
			VkCommandBufferAllocateInfo bufferinfo{};
			bufferinfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
			bufferinfo.commandPool = frames[i].pool;
			bufferinfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
			bufferinfo.commandBufferCount = 2;
			if (!VK14_VERIFY(vkAllocateCommandBuffers(handle, &bufferinfo, buffers)))
			{
				Reason = "no command buffer";
				return false;
			}
			frames[i].uploads = buffers[0];
			frames[i].commands = buffers[1];
			instance.set_object_name(handle, VK_OBJECT_TYPE_COMMAND_BUFFER, reinterpret_cast<std::uint64_t>(frames[i].uploads), "uploads " + std::to_string(i));
			instance.set_object_name(handle, VK_OBJECT_TYPE_COMMAND_BUFFER, reinterpret_cast<std::uint64_t>(frames[i].commands), "frame " + std::to_string(i));
		}
		return true;
	}

	// the parts of the renderer made once the device is there
	bool create_core(std::uint32_t const Frames, std::string &Reason)
	{
		if (false == memory.create(instance, device, Reason))
		{
			return false;
		}
		uploads.init(memory, deletions, device.physical());
		if (false == sampling.create(device, static_cast<float>(Global.AnisotropicFiltering), Reason))
		{
			return false;
		}
		auto const &limits{device.caps().limits};
		(void)limits;
		if (false == ring.create(memory, ring_size, Frames, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, "frame ring"))
		{
			Reason = "no ring for the data of the frames";
			return false;
		}
		if ((false == shaders.init(vk14::binding_preamble(), Reason)) || (false == pipelines.init(device, Reason)) || (false == passes.create(device, Frames, Reason)))
		{
			return false;
		}
		resources.gpu = &device;
		resources.memory = &memory;
		resources.uploads = &uploads;
		resources.deletions = &deletions;
		resources.sampling = &sampling;
		resources.images = &images;
		resources.thread = std::this_thread::get_id();
		resources.pending = &pending;
		textures = std::make_unique<texture_bank>([this]() -> texture_source * { return new vk14::vulkan14_texture(resources); });
		return imgui.create(resources, shaders, pipelines, ring, Reason);
	}

	// the value of the timeline the gpu reached, 0 when it can't be read
	std::uint64_t completed() const
	{
		std::uint64_t value{0};
		if ((timeline == VK_NULL_HANDLE) || (vkGetSemaphoreCounterValue(device.handle(), timeline, &value) != VK_SUCCESS))
		{
			return 0;
		}
		return value;
	}

	void destroy()
	{
		auto const handle{device.handle()};
		if (handle != VK_NULL_HANDLE)
		{
			// the end of the renderer, not a frame: everything on the gpu has to be done
			vkDeviceWaitIdle(handle);
		}
		acquired.clear();
		windows.clear();
		imgui.destroy();
		// the textures give their images to the deletion queue, which goes now that the gpu is idle
		textures.reset();
		uploads.end_frame();
		uploads = vk14::uploader{};
		images.clear();
		deletions.flush();
		pipelines.shutdown();
		shaders.shutdown();
		passes.destroy();
		ring.destroy();
		if (handle != VK_NULL_HANDLE)
		{
			sampling.destroy(handle);
			for (auto &frame : frames)
			{
				if (frame.pool != VK_NULL_HANDLE)
				{
					vkDestroyCommandPool(handle, frame.pool, nullptr);
				}
			}
			if (timeline != VK_NULL_HANDLE)
			{
				vkDestroySemaphore(handle, timeline, nullptr);
			}
		}
		memory.destroy();
		frames.clear();
		timeline = VK_NULL_HANDLE;
		device.destroy();
		instance.destroy();
	}
};

namespace
{

// whether the renderer can run on the machine, before there is a window: the loader, an instance of vulkan 1.4 and a gpu
// with everything the renderer needs
bool probe(std::string &Reason)
{
	vk14::instance instance;
	if (false == instance.create(vk14::instance_config{}, Reason))
	{
		return false;
	}
	vk14::device device;
	if (false == device.select(instance.handle(), VK_NULL_HANDLE, Global.gfx_vulkan_device, true, Reason))
	{
		return false;
	}
	WriteLog(vk14::capability_report(device.caps()));
	return true;
}

void window_hints()
{
	// the swapchain shows the images; a window with an opengl context couldn't be given to vulkan
	glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
}

void window_failed(std::string const &Reason)
{
	vk14::log_error("the window couldn't be made (" + Reason + ")");
}

} // namespace

vulkan14_renderer::vulkan14_renderer() = default;

vulkan14_renderer::~vulkan14_renderer()
{
	Shutdown();
}

std::unique_ptr<gfx_renderer> vulkan14_renderer::create_func()
{
	return std::make_unique<vulkan14_renderer>();
}

gfx_backend_desc vulkan14_renderer::backend_desc()
{
	gfx_backend_desc desc;
	desc.name = "vulkan 1.4";
	desc.create = &vulkan14_renderer::create_func;
	desc.probe = &probe;
	desc.window_hints = &window_hints;
	desc.window_failed = &window_failed;
	desc.gl_context = false;
	return desc;
}

bool vulkan14_renderer::renderer_register = gfx_renderer_factory::get_instance()->register_backend("vulkan", vulkan14_renderer::backend_desc());

bool vulkan14_renderer::Init(GLFWwindow *Window)
{
	m_context = std::make_unique<context>();
	auto &ctx{*m_context};

	std::string reason;
	vk14::instance_config config;
	config.validation = Global.gfx_vulkan_validation;
	config.debug_utils = (Global.gfx_vulkan_validation > 0);

	auto const framesinflight{static_cast<std::uint32_t>(std::clamp(Global.gfx_vulkan_framesinflight, 2, 3))};
	auto ok{ctx.instance.create(config, reason) && ctx.device.select(ctx.instance.handle(), VK_NULL_HANDLE, Global.gfx_vulkan_device, false, reason) && ctx.device.create(ctx.instance, reason) && ctx.create_frames(framesinflight, reason) &&
	        ctx.create_core(framesinflight, reason)};
	if (ok)
	{
		auto mainwindow{std::make_unique<vk14::swapchain>()};
		ok = mainwindow->create(ctx.instance, ctx.device, Window, "main window", framesinflight, reason);
		ctx.windows.emplace_back(std::move(mainwindow));
	}
	if (false == ok)
	{
		vk14::log_error("the renderer can't start: " + reason);
		m_context.reset();
		return false;
	}

	vk14::log_info(std::to_string(framesinflight) + " frames in flight");
	// until the stages of the plan bring them, these parts of the simulator aren't there with vulkan
	vk14::log_info("not available with vulkan yet: the scenery and the vehicles (D1-D7), picking, mouse position in the scene and screenshots (D6), the map panel, "
	               "python screens in their own windows (N5), panels in their own windows (R5b), VR (X4)");
	WriteLog("Gfx Renderer: setup complete");
	update_info();
	return true;
}

bool vulkan14_renderer::AddViewport(global_settings::extraviewport_config const &conf)
{
	if (!m_context)
	{
		return false;
	}
	if (conf.monitor == "MAIN")
	{
		// the projection of the main window is changed only, which the drawing of the scenery takes care of (D6)
		vk14::log_info("extra viewport on the main window: drawn from the stage D6 of the plan");
		return true;
	}

	// a window of its own, without an opengl context
	auto *window{Application.window(-1, true, conf.width, conf.height, Application.find_monitor(conf.monitor), true, false)};
	if (window == nullptr)
	{
		vk14::log_error("the window of the extra viewport on " + conf.monitor + " couldn't be made");
		return false;
	}
	auto &ctx{*m_context};
	auto viewport{std::make_unique<vk14::swapchain>()};
	std::string reason;
	if (false == viewport->create(ctx.instance, ctx.device, window, "viewport " + conf.monitor, static_cast<std::uint32_t>(ctx.frames.size()), reason))
	{
		vk14::log_error(reason);
		return false;
	}
	ctx.windows.emplace_back(std::move(viewport));
	return true;
}

void vulkan14_renderer::Shutdown()
{
	m_context.reset();
}

void vulkan14_renderer::report_device_lost(char const *Where)
{
	auto &ctx{*m_context};
	if (ctx.devicelost)
	{
		return;
	}
	ctx.devicelost = true;
	auto const done{ctx.completed()};
	std::string report{std::string("the device was lost (") + Where + "); the gpu was done with the timeline value " + std::to_string(done) + " of " + std::to_string(ctx.timelinevalue)};
	for (auto const &frame : ctx.frames)
	{
		if ((frame.done > done) && (frame.done <= ctx.timelinevalue))
		{
			report += "; frame " + std::to_string(frame.number) + " (value " + std::to_string(frame.done) + ") wasn't done, its passes: " + ctx.passes.passes_of(frame.number);
		}
	}
	vk14::log_error(report);
	crashreport_add_info("vulkan_device_lost", report);
}

bool vulkan14_renderer::Render()
{
	if (!m_context || m_context->devicelost)
	{
		return false;
	}
	auto &ctx{*m_context};
	auto const device{ctx.device.handle()};
	auto const slot{static_cast<std::uint32_t>(ctx.framecount % ctx.frames.size())};
	auto &frame{ctx.frames[slot]};

	Timer::subsystem.gfx_total.start();

	// the gpu has to be done with the frame which used the slot before
	if (frame.done > 0)
	{
		VkSemaphoreWaitInfo wait{};
		wait.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
		wait.semaphoreCount = 1;
		wait.pSemaphores = &ctx.timeline;
		wait.pValues = &frame.done;
		if (!VK14_VERIFY(vkWaitSemaphores(device, &wait, UINT64_MAX)))
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

	VkCommandBufferBeginInfo begin{};
	begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	VK14_VERIFY(vkBeginCommandBuffer(frame.uploads, &begin));
	VK14_VERIFY(vkBeginCommandBuffer(frame.commands, &begin));
	auto const value{ctx.timelinevalue + 1};
	ctx.uploads.begin_frame(frame.uploads, value);
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
			report_device_lost("acquiring an image");
			return false;
		}
	}

	// the user interface: imgui makes its frame, the images it asks for are uploaded in this frame
	Timer::subsystem.gfx_gui.start();
	Application.render_ui();
	Timer::subsystem.gfx_gui.stop();

	frame.number = ctx.framecount;
	ctx.passes.begin_frame(frame.commands, slot, ctx.framecount);
	auto const background{simulation::is_ready ? clear_colour : glm::vec3{0.f}};
	bool uidrawn{false};
	for (auto *window : ctx.acquired)
	{
		auto const colour{window->srgb() ? glm::vec3{srgb_to_linear(background.r), srgb_to_linear(background.g), srgb_to_linear(background.b)} : background};
		if (window != ctx.windows.front().get())
		{
			// the extra viewports: their colour only, until the scenery is drawn into them (D6)
			window->record_clear(frame.commands, glm::vec4{colour, 1.f});
			continue;
		}
		// the main window: cleared, the user interface over it, shown
		ctx.passes.begin_pass("ui", {window->use(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT)});
		VkRenderingAttachmentInfo attachment{};
		attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
		attachment.imageView = window->view();
		attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
		attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
		attachment.clearValue.color = {{colour.r, colour.g, colour.b, 1.f}};
		VkRenderingInfo rendering{};
		rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
		rendering.renderArea = {{0, 0}, window->extent()};
		rendering.layerCount = 1;
		rendering.colorAttachmentCount = 1;
		rendering.pColorAttachments = &attachment;
		vkCmdBeginRendering(frame.commands, &rendering);
		ctx.imgui.record(frame.commands, window->extent(), window->format(), window->srgb());
		vkCmdEndRendering(frame.commands);
		ctx.passes.end_pass();
		vk14::barriers(frame.commands, {window->use(VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, VK_PIPELINE_STAGE_2_NONE, VK_ACCESS_2_NONE)});
		uidrawn = true;
	}
	if (false == uidrawn)
	{
		// the main window is minimized: the frame of imgui has nowhere to go
		ctx.imgui.skip();
	}
	ctx.passes.end_frame();
	auto const uploaded{ctx.uploads.end_frame()};
	VK14_VERIFY(vkEndCommandBuffer(frame.uploads));
	VK14_VERIFY(vkEndCommandBuffer(frame.commands));
	ctx.ring.flush();

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

	// the uploads first, in the same submit: their barriers make them done before the frame samples the images
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

	Timer::subsystem.gfx_total.stop();
	return true;
}

void vulkan14_renderer::SwapBuffers()
{
	if (!m_context)
	{
		return;
	}
	auto &ctx{*m_context};
	Timer::subsystem.gfx_swap.start();

	if (false == ctx.acquired.empty())
	{
		std::vector<VkSemaphore> waits;
		std::vector<VkSwapchainKHR> swapchains;
		std::vector<std::uint32_t> images;
		std::vector<VkResult> results(ctx.acquired.size(), VK_SUCCESS);
		for (auto *window : ctx.acquired)
		{
			waits.push_back(window->present_semaphore());
			swapchains.push_back(window->handle());
			images.push_back(window->image_index());
		}
		VkPresentInfoKHR present{};
		present.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
		present.waitSemaphoreCount = static_cast<std::uint32_t>(waits.size());
		present.pWaitSemaphores = waits.data();
		present.swapchainCount = static_cast<std::uint32_t>(swapchains.size());
		present.pSwapchains = swapchains.data();
		present.pImageIndices = images.data();
		present.pResults = results.data();
		auto const result{vkQueuePresentKHR(ctx.device.graphics_queue(), &present)};
		for (std::size_t i = 0; i < ctx.acquired.size(); ++i)
		{
			ctx.acquired[i]->presented(results[i]);
		}
		if ((result == VK_ERROR_DEVICE_LOST) || (result == VK_ERROR_SURFACE_LOST_KHR))
		{
			vk14::log_error(std::string("vkQueuePresentKHR failed with ") + vk14::result_string(result));
			report_device_lost("presenting");
		}
		ctx.acquired.clear();
	}
	++ctx.framecount;

	Timer::subsystem.gfx_swap.stop();
}

float vulkan14_renderer::Framerate()
{
	return m_framerate;
}

// geometry: kept in memory for the simulation, nothing on the gpu until D1

gfx::geometrybank_handle vulkan14_renderer::Create_Bank()
{
	return m_geometry.register_bank(std::make_unique<null_geometrybank>());
}

gfx::geometry_handle vulkan14_renderer::Insert(gfx::index_array &Indices, gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometrybank_handle const &Geometry, int const Type)
{
	// NOTE: we expect indexed geometry to come with calculated tangents
	return m_geometry.create_chunk(Indices, Vertices, Userdata, Geometry, Type);
}

gfx::geometry_handle vulkan14_renderer::Insert(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometrybank_handle const &Geometry, int const Type)
{
	gfx::calculate_tangents(Vertices, gfx::index_array(), Type);
	return m_geometry.create_chunk(Vertices, Userdata, Geometry, Type);
}

bool vulkan14_renderer::Replace(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometry_handle const &Geometry, int const Type, std::size_t const Offset)
{
	gfx::calculate_tangents(Vertices, gfx::index_array(), Type);
	return m_geometry.replace(Vertices, Userdata, Geometry, Offset);
}

bool vulkan14_renderer::Append(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometry_handle const &Geometry, int const Type)
{
	gfx::calculate_tangents(Vertices, gfx::index_array(), Type);
	return m_geometry.append(Vertices, Userdata, Geometry);
}

gfx::index_array const &vulkan14_renderer::Indices(gfx::geometry_handle const &Geometry) const
{
	return m_geometry.indices(Geometry);
}

gfx::vertex_array const &vulkan14_renderer::Vertices(gfx::geometry_handle const &Geometry) const
{
	return m_geometry.vertices(Geometry);
}

gfx::userdata_array const &vulkan14_renderer::UserData(gfx::geometry_handle const &Geometry) const
{
	return m_geometry.userdata(Geometry);
}

// materials and their shaders: none until D1

material_handle vulkan14_renderer::Fetch_Material(std::string const &Filename, bool const Loadnow)
{
	return null_handle;
}

void vulkan14_renderer::Bind_Material(material_handle const Material, TSubModel const *sm, lighting_data const *lighting) {}

IMaterial const *vulkan14_renderer::Material(material_handle const Material) const
{
	return IMaterial::null_material();
}

std::shared_ptr<gfx::shader_program> vulkan14_renderer::Fetch_Shader(std::string const &name)
{
	return nullptr;
}

// textures: the files read and decoded as for the opengl renderer, made into images when they are first used (create(), e.g. by the
// launcher); the materials of the scene take them from D1

texture_handle vulkan14_renderer::Fetch_Texture(std::string const &Filename, bool const Loadnow, GLint format_hint)
{
	if (!m_context || !m_context->textures)
	{
		return null_handle;
	}
	return m_context->textures->create(Filename, Loadnow, format_hint);
}

// the shaders take the textures by the pushed descriptors (D1), there's nothing to bind
void vulkan14_renderer::Bind_Texture(texture_handle const Texture) {}

void vulkan14_renderer::Bind_Texture(std::size_t const Unit, texture_handle const Texture) {}

ITexture &vulkan14_renderer::Texture(texture_handle const Texture)
{
	if ((Texture == null_handle) || !m_context || !m_context->textures)
	{
		return *ITexture::null_texture();
	}
	return m_context->textures->mark_as_used(Texture);
}

ITexture const &vulkan14_renderer::Texture(texture_handle const Texture) const
{
	if ((Texture == null_handle) || !m_context || !m_context->textures)
	{
		return *ITexture::null_texture();
	}
	return m_context->textures->texture(Texture);
}

// picking: D6

void vulkan14_renderer::Pick_Control_Callback(std::function<void(TSubModel const *, glm::vec2 const)> Callback) {}

void vulkan14_renderer::Pick_Node_Callback(std::function<void(scene::basic_node *)> Callback) {}

TSubModel const *vulkan14_renderer::Pick_Control() const
{
	return nullptr;
}

scene::basic_node const *vulkan14_renderer::Pick_Node() const
{
	return nullptr;
}

glm::dvec3 vulkan14_renderer::Mouse_Position() const
{
	return glm::dvec3(0.0);
}

// maintenance

void vulkan14_renderer::Update(double const Deltatime)
{
	if (m_context && m_context->textures && (true == Global.ResourceSweep) && (true == simulation::is_ready))
	{
		// textures nothing used for a while give their images back
		m_context->textures->update();
	}
	m_updateaccumulator += Deltatime;
	if (m_updateaccumulator < 1.0)
	{
		return;
	}
	m_updateaccumulator = 0.0;
	m_framerate = 1000.f / Timer::subsystem.mainloop_total.average();
	update_info();
}

void vulkan14_renderer::update_info()
{
	m_infotimes.clear();
	m_infostats.clear();
	if (!m_context)
	{
		return;
	}
	auto const &ctx{*m_context};
	auto const &caps{ctx.device.caps()};
	m_infotimes = "vulkan " + vk14::version_string(caps.api_version) + ": " + caps.device_name + " (" + caps.driver_name + ")\n" + "frames: " + std::to_string(ctx.framecount) + ", in flight " + std::to_string(ctx.frames.size()) + "\n" +
	              "mainloop total: " + to_string(Timer::subsystem.mainloop_total.average(), 2) + " ms, frame cpu: " + to_string(Timer::subsystem.gfx_total.average(), 2) + " ms, ui: " + to_string(Timer::subsystem.gfx_gui.average(), 2) +
	              " ms, present: " + to_string(Timer::subsystem.gfx_swap.average(), 2) + " ms\n" + "gpu frame parts:\n" + ctx.passes.info();
	VkDeviceSize usage{0}, budget{0};
	ctx.memory.budget(usage, budget);
	m_infostats = "device memory: " + vk14::memory_size_string(usage) + " of " + vk14::memory_size_string(budget) + (ctx.device.memory_budget_enabled() ? "" : " (no budget from the driver)") + "\n" + "images: " + std::to_string(ctx.images.size()) +
	              ", uploaded " + vk14::memory_size_string(ctx.uploads.uploaded()) + ", waiting for the gpu to be deleted: " + std::to_string(ctx.deletions.size()) + "\n" + (ctx.textures ? ctx.textures->info() + "\n" : std::string{}) + ctx.shaders.info() +
	              "\n" + ctx.pipelines.info() + "\n" + "frame ring: " + vk14::memory_size_string(ctx.ring.used()) + " of " + vk14::memory_size_string(ctx.ring.capacity()) + "\n";
	for (auto const &window : ctx.windows)
	{
		auto const extent{window->extent()};
		m_infostats += "window: " + std::to_string(extent.width) + "x" + std::to_string(extent.height) + (window->srgb() ? ", srgb" : ", unorm") + (window->present_mode() == VK_PRESENT_MODE_FIFO_KHR ? ", vsync" : ", no vsync") + "\n";
	}
}

void vulkan14_renderer::Update_Pick_Control() {}

void vulkan14_renderer::Update_Pick_Node() {}

glm::dvec3 vulkan14_renderer::Update_Mouse_Position()
{
	return glm::dvec3(0.0);
}

bool vulkan14_renderer::Debug_Ui_State(std::optional<bool> State)
{
	if (State)
	{
		m_debug_ui_active = *State;
	}
	return m_debug_ui_active;
}

std::string const &vulkan14_renderer::info_times() const
{
	return m_infotimes;
}

std::string const &vulkan14_renderer::info_stats() const
{
	return m_infostats;
}

imgui_renderer *vulkan14_renderer::GetImguiRenderer()
{
	return (m_context ? &m_context->imgui : nullptr);
}

void vulkan14_renderer::MakeScreenshot()
{
	vk14::log_warning("screenshots aren't made by the vulkan renderer yet (D6)");
}
