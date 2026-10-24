/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14context.h"
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

// the ring each frame in flight streams its data through: the user interface, the blocks of the scene and of the light clusters,
// the light buffers of the passes, the vertices of the particles and the sky; the descriptors read the light buffers with the range
// of the whole gl::light_data, which the tail past the last frame leaves room for
VkDeviceSize const ring_size{16 * 1024 * 1024};
VkDeviceSize const ring_tail{(sizeof(gl::light_data) + 65535) / 65536 * 65536};

float srgb_to_linear(float const Value)
{
	return (Value <= 0.04045f ? Value / 12.92f : std::pow((Value + 0.055f) / 1.055f, 2.4f));
}

} // namespace

bool vulkan14_renderer::context::create_frames(std::uint32_t const Count, std::string &Reason)
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
bool vulkan14_renderer::context::create_core(std::uint32_t const Frames, std::string &Reason)
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
	if (false == ring.create(memory, ring_size, Frames, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, "frame ring", ring_tail))
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
std::uint64_t vulkan14_renderer::context::completed() const
{
	std::uint64_t value{0};
	if ((timeline == VK_NULL_HANDLE) || (vkGetSemaphoreCounterValue(device.handle(), timeline, &value) != VK_SUCCESS))
	{
		return 0;
	}
	return value;
}

bool vulkan14_renderer::context::wait(std::uint64_t const Value) const
{
	if ((Value == 0) || (completed() >= Value))
	{
		return true;
	}
	VkSemaphoreWaitInfo wait{};
	wait.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
	wait.semaphoreCount = 1;
	wait.pSemaphores = &timeline;
	wait.pValues = &Value;
	return VK14_VERIFY(vkWaitSemaphores(device.handle(), &wait, UINT64_MAX));
}

void vulkan14_renderer::context::destroy()
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

	if (false == init_scene(reason))
	{
		vk14::log_error("the renderer can't start: " + reason);
		destroy_scene();
		m_context.reset();
		return false;
	}
	init_frame_data();

	vk14::log_info(std::to_string(framesinflight) + " frames in flight");
	// until the stages of the plan bring them, these parts of the simulator aren't there with vulkan
	vk14::log_info("not available with vulkan yet: the atmosphere, the sky ambient light, automatic exposure and bloom, stars (P1), the weather fog and clouds (P2), "
	               "ambient occlusion and contact shadows (P3), shadows of lamps and headlights (P4), the culling on the gpu (P5), the heightmap terrain, grass and the particles "
	               "of the gpu (P6), the map panel, python screens in their own windows (N5), panels in their own windows (R5b), VR (X4)");
	WriteLog("Gfx Renderer: setup complete");
	update_info();
	return true;
}

bool vulkan14_renderer::AddViewport(global_settings::extraviewport_config const &conf)
{
	if (!m_context || m_viewports.empty())
	{
		return false;
	}
	viewport_config *viewport{nullptr};
	if (conf.monitor == "MAIN")
	{
		// the main window drawn with the projection of the viewport
		viewport = &m_viewports.front();
	}
	else
	{
		// a window of its own, without an opengl context, with a swapchain of its own
		auto *window{Application.window(-1, true, conf.width, conf.height, Application.find_monitor(conf.monitor), true, false)};
		if (window == nullptr)
		{
			vk14::log_error("the window of the extra viewport on " + conf.monitor + " couldn't be made");
			return false;
		}
		auto &ctx{*m_context};
		auto swapchain{std::make_unique<vk14::swapchain>()};
		std::string reason;
		if (false == swapchain->create(ctx.instance, ctx.device, window, "viewport " + conf.monitor, static_cast<std::uint32_t>(ctx.frames.size()), reason))
		{
			vk14::log_error(reason);
			return false;
		}
		ctx.windows.emplace_back(std::move(swapchain));
		m_viewports.emplace_back();
		viewport = &m_viewports.back();
		viewport->window = ctx.windows.size() - 1;
	}
	viewport->width = conf.width;
	viewport->height = conf.height;
	viewport->projection = conf.projection;
	viewport->proj_type = viewport_config::custom;
	viewport->draw_range = conf.draw_range;
	vk14::log_info("viewport " + conf.monitor + ": " + std::to_string(conf.width) + "x" + std::to_string(conf.height));
	return true;
}

void vulkan14_renderer::Shutdown()
{
	if (m_context && (m_context->device.handle() != VK_NULL_HANDLE))
	{
		// the end of the renderer, not a frame: everything on the gpu has to be done
		vkDeviceWaitIdle(m_context->device.handle());
	}
	// the geometry on the gpu goes with the context; the data of the banks stays for the simulation
	for (auto *bank : m_banks)
	{
		if (bank != nullptr)
		{
			bank->release();
		}
	}
	destroy_scene();
	m_draw.reset();
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

// maintenance

void vulkan14_renderer::Update(double const Deltatime)
{
	Update_Pick_Control();
	Update_Pick_Node();
	update_screenshot();

	m_updateaccumulator += Deltatime;
	if (m_updateaccumulator < 1.0)
	{
		// too early for any work
		return;
	}
	m_updateaccumulator = 0.0;
	m_framerate = 1000.f / Timer::subsystem.mainloop_total.average();

	// adjust draw ranges etc, based on recent performance (opengl46_renderer::Update())
	if (Global.targetfps != 0.0f)
	{
		float fps_diff = Global.targetfps - m_framerate;
		if (fps_diff > 0.0f)
			Global.fDistanceFactor = std::max(0.5f, Global.fDistanceFactor - 0.05f);
		else
			Global.fDistanceFactor = std::min(Global.gfx_distance_factor_max, Global.fDistanceFactor + 0.05f);
	}
	// legacy framerate parameters
	else if (Global.fFpsAverage != 0.f)
	{
		auto const fps_diff = m_framerate - Global.fFpsAverage;
		if (fps_diff < -Global.fFpsDeviation)
		{
			Global.fDistanceFactor = std::min(std::max(1.0f, Global.fDistanceFactor - 0.05f), Global.gfx_distance_factor_max);
		}
		else if (fps_diff > Global.fFpsDeviation)
		{
			Global.fDistanceFactor = std::min(std::min(3.0f, Global.fDistanceFactor + 0.05f), Global.gfx_distance_factor_max);
		}
	}

	if (Global.UpdateMaterials)
	{
		// update resources if there was environmental change
		simulation_state simulationstate{Global.Weather, Global.Season};
		std::swap(m_simulationstate, simulationstate);
		if ((m_simulationstate.season != simulationstate.season) && (false == simulationstate.season.empty()))
		{
			m_materials.on_season_change();
		}
		if ((m_simulationstate.weather != simulationstate.weather) && (false == simulationstate.weather.empty()))
		{
			m_materials.on_weather_change();
		}
	}

	if (m_context && m_context->textures && (true == Global.ResourceSweep) && (true == simulation::is_ready))
	{
		// garbage collection
		m_geometry.update();
		m_context->textures->update();
	}
	if ((Global.gfx_texture_releasedistance > 0.f) && (true == simulation::is_ready))
	{
		update_texture_release();
	}
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
