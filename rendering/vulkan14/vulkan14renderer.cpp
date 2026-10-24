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
#include "rendering/nullrenderer.h"
#include "application/application.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/Timer.h"
#include "utilities/utilities.h"
#include "imgui/imgui.h"

namespace
{

// the colour the windows are cleared to, the initial background of the opengl renderer (srgb)
glm::vec3 const clear_colour{51.f / 255.f, 102.f / 255.f, 85.f / 255.f};

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
		VkCommandBuffer commands{VK_NULL_HANDLE};
		// the value of the timeline semaphore the gpu reaches when it's done with the frame; 0: never submitted
		std::uint64_t done{0};
	};

	vk14::instance instance;
	vk14::device device;
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
			VkCommandBufferAllocateInfo bufferinfo{};
			bufferinfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
			bufferinfo.commandPool = frames[i].pool;
			bufferinfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
			bufferinfo.commandBufferCount = 1;
			if (!VK14_VERIFY(vkAllocateCommandBuffers(handle, &bufferinfo, &frames[i].commands)))
			{
				Reason = "no command buffer";
				return false;
			}
			instance.set_object_name(handle, VK_OBJECT_TYPE_COMMAND_BUFFER, reinterpret_cast<std::uint64_t>(frames[i].commands), "frame " + std::to_string(i));
		}
		return true;
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
		if (handle != VK_NULL_HANDLE)
		{
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
	auto ok{ctx.instance.create(config, reason) && ctx.device.select(ctx.instance.handle(), VK_NULL_HANDLE, Global.gfx_vulkan_device, false, reason) && ctx.device.create(ctx.instance, reason) && ctx.create_frames(framesinflight, reason)};
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
	vk14::log_info("not available with vulkan yet: the scenery and the vehicles (D1-D7), the user interface (R5), picking, mouse position in the scene and screenshots (D6), "
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
			ctx.devicelost = true;
			return false;
		}
	}
	VK14_VERIFY(vkResetCommandPool(device, frame.pool, 0));

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
			ctx.devicelost = true;
			return false;
		}
	}

	// the frames of the user interface go on (its input works too), though nothing of it is drawn yet
	Timer::subsystem.gfx_gui.start();
	Application.render_ui();
	Timer::subsystem.gfx_gui.stop();

	VkCommandBufferBeginInfo begin{};
	begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	VK14_VERIFY(vkBeginCommandBuffer(frame.commands, &begin));
	for (auto *window : ctx.acquired)
	{
		auto const colour{window->srgb() ? glm::vec3{srgb_to_linear(clear_colour.r), srgb_to_linear(clear_colour.g), srgb_to_linear(clear_colour.b)} : clear_colour};
		window->record_clear(frame.commands, glm::vec4{colour, 1.f});
	}
	VK14_VERIFY(vkEndCommandBuffer(frame.commands));

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
	timelinesignal.value = ctx.timelinevalue + 1;
	timelinesignal.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
	signals.push_back(timelinesignal);

	VkCommandBufferSubmitInfo commands{};
	commands.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
	commands.commandBuffer = frame.commands;

	VkSubmitInfo2 submit{};
	submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
	submit.waitSemaphoreInfoCount = static_cast<std::uint32_t>(waits.size());
	submit.pWaitSemaphoreInfos = waits.data();
	submit.commandBufferInfoCount = 1;
	submit.pCommandBufferInfos = &commands;
	submit.signalSemaphoreInfoCount = static_cast<std::uint32_t>(signals.size());
	submit.pSignalSemaphoreInfos = signals.data();
	if (!VK14_VERIFY(vkQueueSubmit2(ctx.device.graphics_queue(), 1, &submit, VK_NULL_HANDLE)))
	{
		ctx.devicelost = true;
		return false;
	}
	frame.done = ++ctx.timelinevalue;

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
			ctx.devicelost = true;
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

// materials, shaders, textures: none until D1 and D2

material_handle vulkan14_renderer::Fetch_Material(std::string const &Filename, bool const Loadnow)
{
	return null_handle;
}

void vulkan14_renderer::Bind_Material(material_handle const Material, TSubModel const *sm, lighting_data const *lighting) {}

IMaterial const *vulkan14_renderer::Material(material_handle const Material) const
{
	return IMaterial::null_material();
}

std::shared_ptr<gl::program> vulkan14_renderer::Fetch_Shader(std::string const &name)
{
	return nullptr;
}

texture_handle vulkan14_renderer::Fetch_Texture(std::string const &Filename, bool const Loadnow, GLint format_hint)
{
	return null_handle;
}

void vulkan14_renderer::Bind_Texture(texture_handle const Texture) {}

void vulkan14_renderer::Bind_Texture(std::size_t const Unit, texture_handle const Texture) {}

ITexture &vulkan14_renderer::Texture(texture_handle const Texture)
{
	return *ITexture::null_texture();
}

ITexture const &vulkan14_renderer::Texture(texture_handle const Texture) const
{
	return *ITexture::null_texture();
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
	              "mainloop total: " + to_string(Timer::subsystem.mainloop_total.average(), 2) + " ms, frame cpu: " + to_string(Timer::subsystem.gfx_total.average(), 2) + " ms, present: " + to_string(Timer::subsystem.gfx_swap.average(), 2) + " ms\n";
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
	return &m_imgui;
}

void vulkan14_renderer::MakeScreenshot()
{
	vk14::log_warning("screenshots aren't made by the vulkan renderer yet (D6)");
}

// imgui: the font atlas is built so its frames go on; nothing is drawn until R5

bool vulkan14_renderer::imgui_stub::Init()
{
	auto &io{ImGui::GetIO()};
	io.BackendRendererName = "vulkan14 (no drawing yet)";
	unsigned char *pixels{nullptr};
	int width{0}, height{0};
	io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
	io.Fonts->SetTexID(static_cast<ImTextureID>(0));
	return true;
}

void vulkan14_renderer::imgui_stub::Shutdown()
{
	ImGui::GetIO().BackendRendererName = nullptr;
}

void vulkan14_renderer::imgui_stub::BeginFrame() {}

void vulkan14_renderer::imgui_stub::Render() {}
