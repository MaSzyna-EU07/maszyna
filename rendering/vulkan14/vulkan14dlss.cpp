/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14dlss.h"

#include "rendering/vulkan14/vulkan14commands.h"
#include "rendering/vulkan14/vulkan14memory.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"
#include "imgui/imgui.h"

#if defined(WITH_DLSS)
#include <nvsdk_ngx_helpers.h>
#include <nvsdk_ngx_helpers_vk.h>
#endif

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <system_error>

#if defined(WITH_DLSS) && !defined(_WIN32)
#include <unistd.h>
#endif

namespace vk14
{

#if defined(WITH_DLSS)

namespace
{

// the identifier of the program for ngx (a project of an engine of its own): anything shaped like a guid
char const *const project_id{"5b3e2f0a-8c4d-4e6b-9a71-2d0f6c8e1a37"};
char const *const engine_version{"1.0"};

// the modes upscaling from less than the full size, from the largest part of the window to the smallest
NVSDK_NGX_PerfQuality_Value const upscaling_modes[]{NVSDK_NGX_PerfQuality_Value_MaxQuality, NVSDK_NGX_PerfQuality_Value_Balanced, NVSDK_NGX_PerfQuality_Value_MaxPerf,
                                                     NVSDK_NGX_PerfQuality_Value_UltraPerformance};

char const *mode_name(int const Mode)
{
	switch (Mode)
	{
	case NVSDK_NGX_PerfQuality_Value_MaxPerf:
		return "performance";
	case NVSDK_NGX_PerfQuality_Value_Balanced:
		return "balanced";
	case NVSDK_NGX_PerfQuality_Value_MaxQuality:
		return "quality";
	case NVSDK_NGX_PerfQuality_Value_UltraPerformance:
		return "ultra performance";
	case NVSDK_NGX_PerfQuality_Value_UltraQuality:
		return "ultra quality";
	case NVSDK_NGX_PerfQuality_Value_DLAA:
		return "dlaa";
	default:
		return "none";
	}
}

char const *preset_name(int const Preset)
{
	switch (Preset)
	{
	case NVSDK_NGX_DLSS_Hint_Render_Preset_J:
		return "j";
	case NVSDK_NGX_DLSS_Hint_Render_Preset_K:
		return "k";
	case NVSDK_NGX_DLSS_Hint_Render_Preset_L:
		return "l";
	case NVSDK_NGX_DLSS_Hint_Render_Preset_M:
		return "m";
	default:
		return "default";
	}
}

// the code of a result of ngx with its text (the text is ascii)
std::string result_text(NVSDK_NGX_Result const Result)
{
	char code[16];
	std::snprintf(code, sizeof(code), "0x%08x", static_cast<unsigned int>(Result));
	std::string text{code};
	if (auto const *wide{GetNGXResultAsString(Result)})
	{
		text += " (";
		for (auto const *c{wide}; *c != 0; ++c)
		{
			text += (*c < 128 ? static_cast<char>(*c) : '?');
		}
		text += ")";
	}
	return text;
}

// the directory of the program, where the libraries of the features are looked for first
std::filesystem::path program_directory()
{
#if defined(_WIN32)
	wchar_t path[MAX_PATH];
	auto const length{::GetModuleFileNameW(nullptr, path, MAX_PATH)};
	if ((length == 0) || (length >= MAX_PATH))
	{
		return {};
	}
	return std::filesystem::path(path).parent_path();
#else
	char path[4096];
	auto const length{::readlink("/proc/self/exe", path, sizeof(path) - 1)};
	if (length <= 0)
	{
		return {};
	}
	path[length] = '\0';
	return std::filesystem::path(path).parent_path();
#endif
}

} // namespace

struct dlss::state
{
	// ngx: where it writes its logs, where it looks for the libraries besides the directory of the program
	std::wstring data_path;
	std::vector<std::wstring> paths;
	std::vector<wchar_t *> path_list;
	NVSDK_NGX_FeatureCommonInfo common{};

	VkInstance instance{VK_NULL_HANDLE};
	VkPhysicalDevice physical{VK_NULL_HANDLE};
	VkDevice device{VK_NULL_HANDLE};
	bool initialized{false};
	bool available{false};
	// the parameters given to the feature, with the capabilities of the driver in them
	NVSDK_NGX_Parameter *parameters{nullptr};

	// the feature: the one the frames ask for and the one made
	struct feature
	{
		glm::ivec2 render{0};
		glm::ivec2 create{0}; // the size the feature is made for, at least the size it's given
		glm::ivec2 output{0};
		int mode{-1};
		int flags{0};
		int preset{0};
		bool operator==(feature const &Other) const
		{
			return (render == Other.render) && (create == Other.create) && (output == Other.output) && (mode == Other.mode) && (flags == Other.flags) && (preset == Other.preset);
		}
	};
	feature wanted;
	feature made;
	NVSDK_NGX_Handle *handle{nullptr};

	// the sizes planned at the start
	glm::ivec2 plan_render{0};
	glm::ivec2 plan_create{0};
	glm::ivec2 plan_output{0};
	int plan_mode{-1};

	// what the replay of the frame evaluates. written by the main thread while it records the frame, read by the replay: the
	// main thread waits for the replay of the frame before it records the next one
	struct resource
	{
		VkImage image{VK_NULL_HANDLE};
		VkImageView view{VK_NULL_HANDLE};
		VkFormat format{VK_FORMAT_UNDEFINED};
		VkImageAspectFlags aspect{VK_IMAGE_ASPECT_COLOR_BIT};
		std::uint32_t width{0};
		std::uint32_t height{0};
		bool valid{false};
	};
	resource colour, depth, motion, exposure, output;
	glm::vec2 jitter{0.f};
	bool reset{false};
	float frame_time{0.f};

	// set by the replay, read by the main thread once it waited for it (sync())
	bool failed{false};
	std::string error;
	unsigned long long memory{0};
	unsigned int evaluations{0};
	// what the main thread shows: taken from the above by sync(), as the statistics are made while the next frame is replayed
	bool seen_failed{false};
	bool error_told{false};
	std::string seen_error;
	feature seen_feature;
	unsigned long long seen_memory{0};

	// the debug window: the offsets of the jitter turned the other way, the history dropped
	bool flip_x{false};
	bool flip_y{false};
	bool reset_wanted{false};

	NVSDK_NGX_FeatureDiscoveryInfo discovery() const
	{
		NVSDK_NGX_FeatureDiscoveryInfo info{};
		info.SDKVersion = NVSDK_NGX_Version_API;
		info.FeatureID = NVSDK_NGX_Feature_SuperSampling;
		info.Identifier.IdentifierType = NVSDK_NGX_Application_Identifier_Type_Project_Id;
		info.Identifier.v.ProjectDesc.ProjectId = project_id;
		info.Identifier.v.ProjectDesc.EngineType = NVSDK_NGX_ENGINE_TYPE_CUSTOM;
		info.Identifier.v.ProjectDesc.EngineVersion = engine_version;
		info.ApplicationDataPath = data_path.c_str();
		info.FeatureInfo = &common;
		return info;
	}

	void prepare_paths()
	{
		auto logs{user_config_path("ngx")};
		std::error_code error;
		if (false == logs.empty())
		{
			std::filesystem::create_directories(logs, error);
		}
		data_path = ((logs.empty() || error) ? std::filesystem::path(".") : logs).wstring();
		paths.clear();
		path_list.clear();
		if (Global.gfx_dlss_developer)
		{
			// the development library, with the indicator and the keys of the debug views
			auto const directory{program_directory()};
			paths.push_back(((directory.empty() ? std::filesystem::path(".") : directory) / "ngx_dev").wstring());
		}
		for (auto &path : paths)
		{
			path_list.push_back(path.data());
		}
		common = NVSDK_NGX_FeatureCommonInfo{};
		common.PathListInfo.Path = path_list.empty() ? nullptr : path_list.data();
		common.PathListInfo.Length = static_cast<unsigned int>(path_list.size());
	}

	void fail(std::string const &Error)
	{
		if (error.empty())
		{
			error = Error;
		}
		failed = true;
	}

	// a failed frame: the output black instead of what was in its memory, which the post-processing (the exposure too) reads
	void clear_output(VkCommandBuffer const Buffer) const
	{
		if (false == output.valid)
		{
			return;
		}
		VkClearColorValue const black{{0.f, 0.f, 0.f, 0.f}};
		VkImageSubresourceRange const range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
		vkCmdClearColorImage(Buffer, output.image, VK_IMAGE_LAYOUT_GENERAL, &black, 1, &range);
		VkMemoryBarrier2 barrier{};
		barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
		barrier.srcStageMask = VK_PIPELINE_STAGE_2_CLEAR_BIT;
		barrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
		barrier.dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
		barrier.dstAccessMask = VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_TRANSFER_READ_BIT;
		VkDependencyInfo dependency{};
		dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
		dependency.memoryBarrierCount = 1;
		dependency.pMemoryBarriers = &barrier;
		vkCmdPipelineBarrier2(Buffer, &dependency);
	}

	static NVSDK_NGX_Resource_VK wrap(resource const &Resource, bool const Write)
	{
		VkImageSubresourceRange const range{Resource.aspect, 0, 1, 0, 1};
		return NVSDK_NGX_Create_ImageView_Resource_VK(Resource.view, Resource.image, range, Resource.format, Resource.width, Resource.height, Write);
	}

	// the replay of the frame: the feature made when it isn't there, then evaluated
	static void record(VkCommandBuffer const Buffer, void *const Data)
	{
		auto &self{*static_cast<state *>(Data)};
		if (self.failed || (self.parameters == nullptr))
		{
			self.clear_output(Buffer);
			return;
		}
		if (self.handle == nullptr)
		{
			for (auto const *hint : {NVSDK_NGX_Parameter_DLSS_Hint_Render_Preset_DLAA, NVSDK_NGX_Parameter_DLSS_Hint_Render_Preset_Quality, NVSDK_NGX_Parameter_DLSS_Hint_Render_Preset_Balanced,
			                         NVSDK_NGX_Parameter_DLSS_Hint_Render_Preset_Performance, NVSDK_NGX_Parameter_DLSS_Hint_Render_Preset_UltraPerformance})
			{
				NVSDK_NGX_Parameter_SetUI(self.parameters, hint, static_cast<unsigned int>(self.wanted.preset));
			}
			NVSDK_NGX_DLSS_Create_Params create{};
			create.Feature.InWidth = static_cast<unsigned int>(self.wanted.create.x);
			create.Feature.InHeight = static_cast<unsigned int>(self.wanted.create.y);
			create.Feature.InTargetWidth = static_cast<unsigned int>(self.wanted.output.x);
			create.Feature.InTargetHeight = static_cast<unsigned int>(self.wanted.output.y);
			create.Feature.InPerfQualityValue = static_cast<NVSDK_NGX_PerfQuality_Value>(self.wanted.mode);
			create.InFeatureCreateFlags = self.wanted.flags;
			create.InEnableOutputSubrects = false;
			auto const result{NGX_VULKAN_CREATE_DLSS_EXT1(self.device, Buffer, 1, 1, &self.handle, self.parameters, &create)};
			if (NVSDK_NGX_FAILED(result) || (self.handle == nullptr))
			{
				self.handle = nullptr;
				self.fail("the feature of dlss can't be made: " + result_text(result));
				self.clear_output(Buffer);
				return;
			}
			self.made = self.wanted;
		}
		auto colour{wrap(self.colour, false)};
		auto depth{wrap(self.depth, false)};
		auto motion{wrap(self.motion, false)};
		auto exposure{wrap(self.exposure, false)};
		auto output{wrap(self.output, true)};
		NVSDK_NGX_VK_DLSS_Eval_Params evaluation{};
		evaluation.Feature.pInColor = &colour;
		evaluation.Feature.pInOutput = &output;
		evaluation.pInDepth = &depth;
		evaluation.pInMotionVectors = &motion;
		evaluation.pInExposureTexture = (self.exposure.valid ? &exposure : nullptr);
		evaluation.InJitterOffsetX = self.jitter.x;
		evaluation.InJitterOffsetY = self.jitter.y;
		evaluation.InRenderSubrectDimensions = {self.colour.width, self.colour.height};
		evaluation.InReset = (self.reset ? 1 : 0);
		// the velocities are parts of the image, dlss takes pixels of the colour
		evaluation.InMVScaleX = static_cast<float>(self.colour.width);
		evaluation.InMVScaleY = static_cast<float>(self.colour.height);
		evaluation.InPreExposure = 1.f;
		evaluation.InExposureScale = 1.f;
		// the rows of the images go from the bottom of the picture up (decision K8): the indicator of the library the right way up
		evaluation.InIndicatorInvertYAxis = 1;
		evaluation.InFrameTimeDeltaInMsec = self.frame_time;
		auto const result{NGX_VULKAN_EVALUATE_DLSS_EXT(Buffer, self.handle, self.parameters, &evaluation)};
		if (NVSDK_NGX_FAILED(result))
		{
			self.fail("dlss can't be evaluated: " + result_text(result));
			self.clear_output(Buffer);
			return;
		}
		// the memory of the features now and then, for the statistics (ngx isn't to be called from two threads at once)
		if ((self.evaluations++ % 120) == 0)
		{
			unsigned long long bytes{0};
			if (NVSDK_NGX_SUCCEED(NGX_DLSS_GET_STATS(self.parameters, &bytes)))
			{
				self.memory = bytes;
			}
		}
	}

	void release(deletion_queue *Deletions, std::uint64_t const Value)
	{
		if (handle == nullptr)
		{
			return;
		}
		auto *old{handle};
		handle = nullptr;
		made = feature{};
		if (Deletions != nullptr)
		{
			Deletions->push(Value, [old]() { NVSDK_NGX_VULKAN_ReleaseFeature(old); });
		}
		else
		{
			NVSDK_NGX_VULKAN_ReleaseFeature(old);
		}
	}
};

dlss::dlss() : m_state{std::make_unique<state>()} {}

dlss::~dlss()
{
	shutdown();
}

bool dlss::compiled()
{
	return true;
}

void dlss::instance_extensions(std::vector<std::string> &Extensions)
{
	state temporary;
	temporary.prepare_paths();
	auto const info{temporary.discovery()};
	std::uint32_t count{0};
	VkExtensionProperties *properties{nullptr};
	auto const result{NVSDK_NGX_VULKAN_GetFeatureInstanceExtensionRequirements(&info, &count, &properties)};
	if (NVSDK_NGX_FAILED(result))
	{
		log_warning("dlss: the instance extensions it needs can't be had from ngx: " + result_text(result));
		return;
	}
	for (std::uint32_t i = 0; i < count; ++i)
	{
		Extensions.emplace_back(properties[i].extensionName);
	}
}

void dlss::device_extensions(VkInstance const Instance, VkPhysicalDevice const Physical, std::vector<std::string> &Extensions)
{
	state temporary;
	temporary.prepare_paths();
	auto const info{temporary.discovery()};
	std::uint32_t count{0};
	VkExtensionProperties *properties{nullptr};
	auto const result{NVSDK_NGX_VULKAN_GetFeatureDeviceExtensionRequirements(Instance, Physical, &info, &count, &properties)};
	if (NVSDK_NGX_FAILED(result))
	{
		log_warning("dlss: the device extensions it needs can't be had from ngx: " + result_text(result));
		return;
	}
	for (std::uint32_t i = 0; i < count; ++i)
	{
		Extensions.emplace_back(properties[i].extensionName);
	}
}

bool dlss::init(VkInstance const Instance, VkPhysicalDevice const Physical, VkDevice const Device, std::string &Reason)
{
	auto &s{*m_state};
	if (s.initialized)
	{
		return s.available;
	}
	s.prepare_paths();
	s.instance = Instance;
	s.physical = Physical;
	s.device = Device;
	auto const result{NVSDK_NGX_VULKAN_Init_with_ProjectID(project_id, NVSDK_NGX_ENGINE_TYPE_CUSTOM, engine_version, s.data_path.c_str(), Instance, Physical, Device, vkGetInstanceProcAddr, vkGetDeviceProcAddr, &s.common,
	                                                        NVSDK_NGX_Version_API)};
	if (NVSDK_NGX_FAILED(result))
	{
		Reason = "ngx can't start: " + result_text(result);
		return false;
	}
	s.initialized = true;
	// the parameters with the capabilities in them (the drivers before 450 don't give them this way, and are too old anyway)
	auto const capabilities{NVSDK_NGX_VULKAN_GetCapabilityParameters(&s.parameters)};
	if (NVSDK_NGX_FAILED(capabilities) || (s.parameters == nullptr))
	{
		s.parameters = nullptr;
		Reason = "ngx gives no parameters, the graphics driver is probably too old: " + result_text(capabilities);
		shutdown();
		return false;
	}
	int updatedriver{0};
	if (NVSDK_NGX_SUCCEED(s.parameters->Get(NVSDK_NGX_Parameter_SuperSampling_NeedsUpdatedDriver, &updatedriver)) && (updatedriver != 0))
	{
		unsigned int major{0}, minor{0};
		s.parameters->Get(NVSDK_NGX_Parameter_SuperSampling_MinDriverVersionMajor, &major);
		s.parameters->Get(NVSDK_NGX_Parameter_SuperSampling_MinDriverVersionMinor, &minor);
		Reason = "the graphics driver is too old for dlss, it needs " + std::to_string(major) + "." + std::to_string(minor) + " or newer";
		shutdown();
		return false;
	}
	int supported{0};
	if (NVSDK_NGX_FAILED(s.parameters->Get(NVSDK_NGX_Parameter_SuperSampling_Available, &supported)) || (supported == 0))
	{
		Reason = "dlss isn't available on this gpu (it needs a geforce rtx) or with this driver";
		shutdown();
		return false;
	}
	supported = 0;
	if (NVSDK_NGX_FAILED(s.parameters->Get(NVSDK_NGX_Parameter_SuperSampling_FeatureInitResult, &supported)) || (supported == 0))
	{
		Reason = "dlss is denied to the program by the driver";
		shutdown();
		return false;
	}
	s.available = true;
	return true;
}

void dlss::shutdown()
{
	auto &s{*m_state};
	if (false == s.initialized)
	{
		return;
	}
	// the gpu is idle: the feature goes at once
	s.release(nullptr, 0);
	if (s.parameters != nullptr)
	{
		NVSDK_NGX_VULKAN_DestroyParameters(s.parameters);
	}
	s.parameters = nullptr;
	NVSDK_NGX_VULKAN_Shutdown1(s.device);
	s.initialized = false;
	s.available = false;
}

bool dlss::available() const
{
	return m_state->available;
}

bool dlss::plan(glm::ivec2 const &Output, float const Scale, glm::ivec2 &Render)
{
	auto &s{*m_state};
	// the smallest window dlss upscales to
	if ((false == s.available) || (Output.x < 64) || (Output.y < 32))
	{
		return false;
	}
	if (Scale >= 0.999f)
	{
		s.plan_render = Output;
		s.plan_create = Output;
		s.plan_output = Output;
		s.plan_mode = NVSDK_NGX_PerfQuality_Value_DLAA;
		Render = Output;
		log_info("dlss: dlaa, the scene drawn at the size of the window, " + std::to_string(Output.x) + "x" + std::to_string(Output.y));
		return true;
	}
	glm::ivec2 const wanted{glm::max(glm::ivec2(glm::round(glm::vec2(Output) * Scale)), glm::ivec2(1))};
	auto bestmode{-1};
	glm::ivec2 best{0};
	glm::ivec2 bestoptimal{0};
	auto bestinside{false};
	auto bestdistance{1e9f};
	for (auto const mode : upscaling_modes)
	{
		unsigned int optimalwidth{0}, optimalheight{0}, maxwidth{0}, maxheight{0}, minwidth{0}, minheight{0};
		float sharpness{0.f};
		auto const result{NGX_DLSS_GET_OPTIMAL_SETTINGS(s.parameters, static_cast<unsigned int>(Output.x), static_cast<unsigned int>(Output.y), mode, &optimalwidth, &optimalheight, &maxwidth, &maxheight, &minwidth,
		                                                &minheight, &sharpness)};
		if (NVSDK_NGX_FAILED(result) || (optimalwidth == 0) || (optimalheight == 0))
		{
			// a mode not there for this size
			continue;
		}
		glm::ivec2 const low{static_cast<int>(minwidth), static_cast<int>(minheight)};
		glm::ivec2 const high{static_cast<int>(std::max(maxwidth, minwidth)), static_cast<int>(std::max(maxheight, minheight))};
		auto const inside{(wanted.x >= low.x) && (wanted.y >= low.y) && (wanted.x <= high.x) && (wanted.y <= high.y)};
		auto const distance{std::abs(static_cast<float>(optimalwidth) / static_cast<float>(Output.x) - Scale)};
		// a mode the size fits in goes before one it has to be brought into, then the one whose own size is nearest
		if ((bestmode < 0) || (inside && (false == bestinside)) || ((inside == bestinside) && (distance < bestdistance)))
		{
			bestmode = mode;
			// brought into the range of the mode by one scale for both sides, which keeps the shape of the window
			auto const lowscale{std::max(static_cast<float>(low.x) / static_cast<float>(Output.x), static_cast<float>(low.y) / static_cast<float>(Output.y))};
			auto const highscale{std::min(static_cast<float>(high.x) / static_cast<float>(Output.x), static_cast<float>(high.y) / static_cast<float>(Output.y))};
			auto const scale{std::clamp(Scale, std::min(lowscale, highscale), highscale)};
			best = inside ? wanted : glm::clamp(glm::ivec2(glm::round(glm::vec2(Output) * scale)), low, high);
			bestoptimal = {static_cast<int>(optimalwidth), static_cast<int>(optimalheight)};
			bestinside = inside;
			bestdistance = distance;
		}
	}
	if (bestmode < 0)
	{
		log_warning("dlss: no mode of it upscales to " + std::to_string(Output.x) + "x" + std::to_string(Output.y));
		return false;
	}
	s.plan_render = best;
	// the feature made for the size the guide gives for the mode (3.2.2), or for the size drawn when that's larger: the presets l and
	// m make their buffers for the size given at the creation, and make them anew for a larger image (3.2.2.1)
	s.plan_create = glm::max(bestoptimal, best);
	s.plan_output = Output;
	s.plan_mode = bestmode;
	Render = best;
	log_info("dlss: the scene drawn at " + std::to_string(best.x) + "x" + std::to_string(best.y) + " for the window of " + std::to_string(Output.x) + "x" + std::to_string(Output.y) + ", mode " + mode_name(bestmode) +
	         (bestinside ? std::string{} : ", gfx.buffer.percent brought into the range of the mode"));
	return true;
}

float dlss::lod_bias() const
{
	auto const &s{*m_state};
	if ((s.plan_output.x <= 0) || (s.plan_render.x <= 0))
	{
		return 0.f;
	}
	// the textures sampled as at the size of the window (the dlss programming guide, 3.5)
	return std::log2(static_cast<float>(s.plan_render.x) / static_cast<float>(s.plan_output.x)) - 1.f + Global.gfx_dlss_lodbias;
}

std::uint32_t dlss::jitter_phases() const
{
	auto const &s{*m_state};
	if ((s.plan_output.x <= 0) || (s.plan_render.x <= 0))
	{
		return 8;
	}
	// 8 offsets for each pixel of the window (the dlss programming guide, 3.7.1.1)
	auto const ratio{static_cast<float>(s.plan_output.x) / static_cast<float>(s.plan_render.x)};
	return static_cast<std::uint32_t>(std::clamp(std::ceil(8.f * ratio * ratio), 8.f, 128.f));
}

void dlss::evaluate(command_stream &Commands, frame_inputs const &Inputs, deletion_queue &Deletions, std::uint64_t const Value)
{
	auto &s{*m_state};
	if ((false == s.available) || s.failed || (Inputs.colour == nullptr) || (Inputs.depth == nullptr) || (Inputs.motion == nullptr) || (Inputs.output == nullptr))
	{
		return;
	}
	state::feature feature;
	feature.render = {static_cast<int>(Inputs.colour->desc().extent.width), static_cast<int>(Inputs.colour->desc().extent.height)};
	feature.output = {static_cast<int>(Inputs.output->desc().extent.width), static_cast<int>(Inputs.output->desc().extent.height)};
	auto const planned{(feature.render == s.plan_render) && (feature.output == s.plan_output)};
	feature.mode = (feature.render == feature.output) ? static_cast<int>(NVSDK_NGX_PerfQuality_Value_DLAA) : planned ? s.plan_mode : static_cast<int>(NVSDK_NGX_PerfQuality_Value_MaxQuality);
	feature.create = planned ? s.plan_create : feature.render;
	// the colour is linear (hdr), the velocities have the size of the colour, the depth is reverse z
	feature.flags = NVSDK_NGX_DLSS_Feature_Flags_IsHDR | NVSDK_NGX_DLSS_Feature_Flags_MVLowRes | NVSDK_NGX_DLSS_Feature_Flags_DepthInverted | (Inputs.exposure == nullptr ? NVSDK_NGX_DLSS_Feature_Flags_AutoExposure : 0);
	feature.preset = Global.gfx_dlss_preset;
	if ((s.handle != nullptr) && (false == (feature == s.made)))
	{
		s.release(&Deletions, Value);
	}
	s.wanted = feature;
	auto const describe = [](image const *Image, state::resource &Resource) {
		Resource.valid = (Image != nullptr);
		if (Image == nullptr)
		{
			return;
		}
		Resource.image = Image->handle();
		Resource.view = Image->view();
		Resource.format = Image->desc().format;
		Resource.aspect = Image->aspect();
		Resource.width = Image->desc().extent.width;
		Resource.height = Image->desc().extent.height;
	};
	describe(Inputs.colour, s.colour);
	describe(Inputs.depth, s.depth);
	describe(Inputs.motion, s.motion);
	describe(Inputs.exposure, s.exposure);
	describe(Inputs.output, s.output);
	s.jitter = Inputs.jitter * glm::vec2{s.flip_x ? -1.f : 1.f, s.flip_y ? -1.f : 1.f};
	s.reset = Inputs.reset || s.reset_wanted;
	s.reset_wanted = false;
	s.frame_time = Inputs.frame_time;
	Commands.external(&state::record, &s);
}

std::string dlss::sync()
{
	auto &s{*m_state};
	s.seen_failed = s.failed;
	s.seen_error = s.error;
	s.seen_feature = ((s.handle != nullptr) ? s.made : s.wanted);
	s.seen_memory = s.memory;
	if ((false == s.failed) || s.error_told)
	{
		return {};
	}
	s.error_told = true;
	return s.error;
}

bool dlss::failed() const
{
	return m_state->seen_failed;
}

std::string dlss::info() const
{
	auto const &s{*m_state};
	if (false == s.available)
	{
		return "dlss: not available";
	}
	if (s.seen_failed)
	{
		return "dlss: off after an error (" + s.seen_error + ")";
	}
	auto const &feature{s.seen_feature};
	if (feature.mode < 0)
	{
		return "dlss: ready, not used yet";
	}
	// the presets l and m take no exposure, they work it out themselves (the dlss programming guide, 3.9)
	return std::string("dlss: ") + mode_name(feature.mode) + ", preset " + preset_name(feature.preset) + ", " + std::to_string(feature.render.x) + "x" + std::to_string(feature.render.y) + " to " + std::to_string(feature.output.x) + "x" +
	       std::to_string(feature.output.y) + ((feature.flags & NVSDK_NGX_DLSS_Feature_Flags_AutoExposure) != 0 ? ", its own exposure" : ", given the exposure of the tonemapping") + ", " +
	       std::to_string(s.seen_memory / (1024 * 1024)) + " MiB";
}

void dlss::draw_debug_ui()
{
	auto &s{*m_state};
	ImGui::TextUnformatted(info().c_str());
	if (false == s.available)
	{
		return;
	}
	ImGui::Checkbox("dlss: jitter offset turned the other way along the rows", &s.flip_x);
	ImGui::SameLine();
	ImGui::Checkbox("along the columns", &s.flip_y);
	if (ImGui::Button("dlss: drop its history"))
	{
		s.reset_wanted = true;
	}
}

#else // without the sdk: dlss is never available

struct dlss::state
{
};

dlss::dlss() : m_state{std::make_unique<state>()} {}

dlss::~dlss() = default;

bool dlss::compiled()
{
	return false;
}

void dlss::instance_extensions(std::vector<std::string> &) {}

void dlss::device_extensions(VkInstance, VkPhysicalDevice, std::vector<std::string> &) {}

bool dlss::init(VkInstance, VkPhysicalDevice, VkDevice, std::string &Reason)
{
	Reason = "the program was built without the dlss sdk (WITH_DLSS)";
	return false;
}

void dlss::shutdown() {}

bool dlss::available() const
{
	return false;
}

bool dlss::plan(glm::ivec2 const &, float, glm::ivec2 &)
{
	return false;
}

float dlss::lod_bias() const
{
	return 0.f;
}

std::uint32_t dlss::jitter_phases() const
{
	return 8;
}

void dlss::evaluate(command_stream &, frame_inputs const &, deletion_queue &, std::uint64_t) {}

std::string dlss::sync()
{
	return {};
}

bool dlss::failed() const
{
	return false;
}

std::string dlss::info() const
{
	return "dlss: not in this build (WITH_DLSS)";
}

void dlss::draw_debug_ui()
{
	ImGui::TextUnformatted(info().c_str());
}

#endif

} // namespace vk14
