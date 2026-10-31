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
#include <nvsdk_ngx_helpers_dlssg_vk.h>
#endif

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>
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

	// the frame generation: the most frames it makes between two, its own parameters (one map for each feature, as the guide of
	// the frame generation asks), the feature the frames ask for and the one made
	std::uint32_t generation_max{0};
	NVSDK_NGX_Parameter *generation_parameters{nullptr};
	struct generation_feature
	{
		glm::ivec2 output{0};
		glm::ivec2 render{0};
		VkFormat format{VK_FORMAT_UNDEFINED};
		bool operator==(generation_feature const &Other) const
		{
			return (output == Other.output) && (render == Other.render) && (format == Other.format);
		}
	};
	generation_feature generation_wanted;
	generation_feature generation_made;
	NVSDK_NGX_Handle *generation_handle{nullptr};
	// what the replay of the frame evaluates, as the resources above
	resource backbuffer, hudless, generation_depth, generation_motion;
	std::vector<resource> generated;
	NVSDK_NGX_DLSSG_Opt_Eval_Params generation_options{};
	// set by the replay, read by the main thread once it waited for it
	bool generation_failed{false};
	std::string generation_error;
	bool generation_error_told{false};
	bool seen_generation_failed{false};
	std::uint64_t generations{0};
	std::uint64_t seen_generations{0};

	// the debug window: the offsets of the jitter turned the other way, the history dropped
	bool flip_x{false};
	bool flip_y{false};
	bool reset_wanted{false};
	// the frame generation: the clip space of its matrices turned over (as the images it's given), its history dropped
	bool generation_clip_flip{false};
	bool generation_reset_wanted{false};

	NVSDK_NGX_FeatureDiscoveryInfo discovery(NVSDK_NGX_Feature const Feature) const
	{
		NVSDK_NGX_FeatureDiscoveryInfo info{};
		info.SDKVersion = NVSDK_NGX_Version_API;
		info.FeatureID = Feature;
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

	// a failed frame generation: the frames it was to make are the backbuffer, shown again
	void copy_backbuffer(VkCommandBuffer const Buffer) const
	{
		if (false == backbuffer.valid)
		{
			return;
		}
		for (auto const &output : generated)
		{
			if (false == output.valid)
			{
				continue;
			}
			VkImageCopy2 region{};
			region.sType = VK_STRUCTURE_TYPE_IMAGE_COPY_2;
			region.srcSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
			region.dstSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
			region.extent = {std::min(backbuffer.width, output.width), std::min(backbuffer.height, output.height), 1};
			VkCopyImageInfo2 copy{};
			copy.sType = VK_STRUCTURE_TYPE_COPY_IMAGE_INFO_2;
			copy.srcImage = backbuffer.image;
			copy.srcImageLayout = VK_IMAGE_LAYOUT_GENERAL;
			copy.dstImage = output.image;
			copy.dstImageLayout = VK_IMAGE_LAYOUT_GENERAL;
			copy.regionCount = 1;
			copy.pRegions = &region;
			vkCmdCopyImage2(Buffer, &copy);
		}
		VkMemoryBarrier2 barrier{};
		barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
		barrier.srcStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
		barrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
		barrier.dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
		barrier.dstAccessMask = VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_TRANSFER_READ_BIT;
		VkDependencyInfo dependency{};
		dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
		dependency.memoryBarrierCount = 1;
		dependency.pMemoryBarriers = &barrier;
		vkCmdPipelineBarrier2(Buffer, &dependency);
	}

	void fail_generation(std::string const &Error)
	{
		if (generation_error.empty())
		{
			generation_error = Error;
		}
		generation_failed = true;
	}

	// the replay of the frame: the feature of the frame generation made when it isn't there, then evaluated once for each frame it
	// makes, from the first one shown on
	static void record_generation(VkCommandBuffer const Buffer, void *const Data)
	{
		auto &self{*static_cast<state *>(Data)};
		if (self.generation_failed || (self.generation_parameters == nullptr) || self.generated.empty())
		{
			self.copy_backbuffer(Buffer);
			return;
		}
		auto *parameters{self.generation_parameters};
		if (self.generation_handle == nullptr)
		{
			NVSDK_NGX_DLSSG_Create_Params create{};
			create.Width = static_cast<unsigned int>(self.generation_wanted.output.x);
			create.Height = static_cast<unsigned int>(self.generation_wanted.output.y);
			create.NativeBackbufferFormat = static_cast<unsigned int>(self.generation_wanted.format);
			create.RenderWidth = static_cast<unsigned int>(self.generation_wanted.render.x);
			create.RenderHeight = static_cast<unsigned int>(self.generation_wanted.render.y);
			create.DynamicResolutionScaling = false;
			// the helper of vulkan leaves out the size of the depth and the velocities, which the one of d3d12 gives
			NVSDK_NGX_Parameter_SetUI(parameters, NVSDK_NGX_DLSSG_Parameter_Width, create.Width);
			NVSDK_NGX_Parameter_SetUI(parameters, NVSDK_NGX_DLSSG_Parameter_Height, create.Height);
			NVSDK_NGX_Parameter_SetUI(parameters, NVSDK_NGX_DLSSG_Parameter_InternalWidth, create.RenderWidth);
			NVSDK_NGX_Parameter_SetUI(parameters, NVSDK_NGX_DLSSG_Parameter_InternalHeight, create.RenderHeight);
			NVSDK_NGX_Parameter_SetUI(parameters, NVSDK_NGX_DLSSG_Parameter_DynamicResolution, 0);
			auto const result{NGX_VK_CREATE_DLSSG(Buffer, 1, 1, &self.generation_handle, parameters, &create)};
			if (NVSDK_NGX_FAILED(result) || (self.generation_handle == nullptr))
			{
				self.generation_handle = nullptr;
				self.fail_generation("the feature of the frame generation of dlss can't be made: " + result_text(result));
				self.copy_backbuffer(Buffer);
				return;
			}
			self.generation_made = self.generation_wanted;
		}
		auto backbuffer{wrap(self.backbuffer, true)};
		auto hudless{wrap(self.hudless, true)};
		auto depth{wrap(self.generation_depth, true)};
		auto motion{wrap(self.generation_motion, true)};
		auto options{self.generation_options};
		options.multiFrameCount = static_cast<unsigned int>(self.generated.size());
		for (std::size_t i = 0; i < self.generated.size(); ++i)
		{
			auto output{wrap(self.generated[i], true)};
			NVSDK_NGX_VK_DLSSG_Eval_Params evaluation{};
			evaluation.pBackbuffer = &backbuffer;
			evaluation.pDepth = &depth;
			evaluation.pMVecs = &motion;
			evaluation.pHudless = (self.hudless.valid ? &hudless : nullptr);
			evaluation.pOutputInterpFrame = &output;
			options.multiFrameIndex = static_cast<unsigned int>(i + 1);
			auto const result{NGX_VK_EVALUATE_DLSSG(Buffer, self.generation_handle, parameters, &evaluation, &options)};
			if (NVSDK_NGX_FAILED(result))
			{
				self.fail_generation("the frame generation of dlss can't be evaluated: " + result_text(result));
				self.copy_backbuffer(Buffer);
				return;
			}
		}
		++self.generations;
	}

	void release_generation(deletion_queue *Deletions, std::uint64_t const Value)
	{
		if (generation_handle == nullptr)
		{
			return;
		}
		auto *old{generation_handle};
		generation_handle = nullptr;
		generation_made = generation_feature{};
		if (Deletions != nullptr)
		{
			Deletions->push(Value, [old]() { NVSDK_NGX_VULKAN_ReleaseFeature(old); });
		}
		else
		{
			NVSDK_NGX_VULKAN_ReleaseFeature(old);
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

namespace
{

// the features of ngx the renderer uses: the super resolution, and the frame generation where it's asked for
std::vector<NVSDK_NGX_Feature> wanted_features()
{
	std::vector<NVSDK_NGX_Feature> features{NVSDK_NGX_Feature_SuperSampling};
	if (Global.gfx_framegeneration > 1)
	{
		features.push_back(NVSDK_NGX_Feature_FrameGeneration);
	}
	return features;
}

void add_extensions(VkExtensionProperties const *Properties, std::uint32_t const Count, std::vector<std::string> &Extensions)
{
	for (std::uint32_t i = 0; i < Count; ++i)
	{
		std::string const name{Properties[i].extensionName};
		if (std::find(Extensions.begin(), Extensions.end(), name) == Extensions.end())
		{
			Extensions.push_back(name);
		}
	}
}

} // namespace

void dlss::instance_extensions(std::vector<std::string> &Extensions)
{
	state temporary;
	temporary.prepare_paths();
	for (auto const feature : wanted_features())
	{
		auto const info{temporary.discovery(feature)};
		std::uint32_t count{0};
		VkExtensionProperties *properties{nullptr};
		auto const result{NVSDK_NGX_VULKAN_GetFeatureInstanceExtensionRequirements(&info, &count, &properties)};
		if (NVSDK_NGX_FAILED(result))
		{
			log_warning(std::string("dlss: the instance extensions ") + (feature == NVSDK_NGX_Feature_FrameGeneration ? "its frame generation" : "it") + " needs can't be had from ngx: " + result_text(result));
			continue;
		}
		add_extensions(properties, count, Extensions);
	}
}

void dlss::device_extensions(VkInstance const Instance, VkPhysicalDevice const Physical, std::vector<std::string> &Extensions)
{
	state temporary;
	temporary.prepare_paths();
	for (auto const feature : wanted_features())
	{
		auto const info{temporary.discovery(feature)};
		std::uint32_t count{0};
		VkExtensionProperties *properties{nullptr};
		auto const result{NVSDK_NGX_VULKAN_GetFeatureDeviceExtensionRequirements(Instance, Physical, &info, &count, &properties)};
		if (NVSDK_NGX_FAILED(result))
		{
			log_warning(std::string("dlss: the device extensions ") + (feature == NVSDK_NGX_Feature_FrameGeneration ? "its frame generation" : "it") + " needs can't be had from ngx: " + result_text(result));
			continue;
		}
		add_extensions(properties, count, Extensions);
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
	// the frame generation, where it's asked for; without it dlss goes on alone
	if (Global.gfx_framegeneration > 1)
	{
		std::string reason;
		int fg{0};
		int fgupdate{0};
		unsigned int framesmax{0};
		if (NVSDK_NGX_SUCCEED(s.parameters->Get(NVSDK_NGX_Parameter_FrameGeneration_NeedsUpdatedDriver, &fgupdate)) && (fgupdate != 0))
		{
			unsigned int major{0}, minor{0};
			s.parameters->Get(NVSDK_NGX_Parameter_FrameGeneration_MinDriverVersionMajor, &major);
			s.parameters->Get(NVSDK_NGX_Parameter_FrameGeneration_MinDriverVersionMinor, &minor);
			reason = "the graphics driver is too old for it, it needs " + std::to_string(major) + "." + std::to_string(minor) + " or newer";
		}
		else if (NVSDK_NGX_FAILED(s.parameters->Get(NVSDK_NGX_Parameter_FrameGeneration_Available, &fg)) || (fg == 0))
		{
			reason = "it isn't available on this gpu (it needs a geforce rtx of the 40 series or newer) or with this driver";
		}
		else if (NVSDK_NGX_FAILED(NVSDK_NGX_VULKAN_AllocateParameters(&s.generation_parameters)) || (s.generation_parameters == nullptr))
		{
			s.generation_parameters = nullptr;
			reason = "ngx gives no parameters for it";
		}
		else
		{
			// more than one frame between two (x3, x4) from the rtx 50 series on
			if (NVSDK_NGX_FAILED(s.parameters->Get(NVSDK_NGX_DLSSG_Parameter_MultiFrameCountMax, &framesmax)) || (framesmax < 1))
			{
				framesmax = 1;
			}
			s.generation_max = std::min(framesmax, 3u);
		}
		if (s.generation_max == 0)
		{
			log_info("gfx.framegeneration: " + reason + ", the frames aren't generated");
		}
		else
		{
			log_info("dlss: the frame generation can make up to " + std::to_string(s.generation_max) + (s.generation_max == 1 ? " frame" : " frames") + " between two (x" + std::to_string(s.generation_max + 1) + ")");
		}
	}
	return true;
}

void dlss::shutdown()
{
	auto &s{*m_state};
	if (false == s.initialized)
	{
		return;
	}
	// the gpu is idle: the features go at once
	s.release(nullptr, 0);
	s.release_generation(nullptr, 0);
	if (s.generation_parameters != nullptr)
	{
		NVSDK_NGX_VULKAN_DestroyParameters(s.generation_parameters);
	}
	s.generation_parameters = nullptr;
	s.generation_max = 0;
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

std::uint32_t dlss::generation_max() const
{
	auto const &s{*m_state};
	return (s.available ? s.generation_max : 0);
}

bool dlss::generation_failed() const
{
	return m_state->seen_generation_failed;
}

namespace
{

// a matrix of glm (columns of column vectors) as ngx takes it: rows, for row vectors multiplied by it from the left, which is the
// same memory
void ngx_matrix(glm::mat4 const &Matrix, float (&Target)[4][4])
{
	std::memcpy(Target, &Matrix[0][0], sizeof(Target));
}

} // namespace

void dlss::generate(command_stream &Commands, generation_inputs const &Inputs, deletion_queue &Deletions, std::uint64_t const Value)
{
	auto &s{*m_state};
	if ((false == s.available) || (s.generation_max == 0) || (Inputs.backbuffer == nullptr) || (Inputs.depth == nullptr) || (Inputs.motion == nullptr) || Inputs.outputs.empty())
	{
		return;
	}
	state::generation_feature feature;
	feature.output = {static_cast<int>(Inputs.backbuffer->desc().extent.width), static_cast<int>(Inputs.backbuffer->desc().extent.height)};
	feature.render = {static_cast<int>(Inputs.depth->desc().extent.width), static_cast<int>(Inputs.depth->desc().extent.height)};
	feature.format = Inputs.backbuffer->desc().format;
	if ((s.generation_handle != nullptr) && (false == (feature == s.generation_made)))
	{
		s.release_generation(&Deletions, Value);
	}
	s.generation_wanted = feature;
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
	describe(Inputs.backbuffer, s.backbuffer);
	describe(Inputs.hudless, s.hudless);
	describe(Inputs.depth, s.generation_depth);
	describe(Inputs.motion, s.generation_motion);
	s.generated.resize(std::min<std::size_t>(Inputs.outputs.size(), s.generation_max));
	for (std::size_t i = 0; i < s.generated.size(); ++i)
	{
		describe(Inputs.outputs[i], s.generated[i]);
	}

	auto &options{s.generation_options};
	options = NVSDK_NGX_DLSSG_Opt_Eval_Params{};
	// the clip space of the renderer has its rows from the bottom of the picture up, as the images; the images the frame generation
	// is given are turned over, and its clip space (that of d3d12, y up the picture) is the one of the renderer as it is. the debug
	// window turns it over for the other way
	glm::mat4 flip{1.f};
	if (s.generation_clip_flip)
	{
		flip[1][1] = -1.f;
	}
	auto const viewtoclip{flip * Inputs.view_to_clip};
	auto const cliptoprevious{flip * Inputs.clip_to_previous_clip * flip};
	ngx_matrix(viewtoclip, options.cameraViewToClip);
	ngx_matrix(glm::inverse(viewtoclip), options.clipToCameraView);
	ngx_matrix(glm::mat4{1.f}, options.clipToLensClip);
	ngx_matrix(cliptoprevious, options.clipToPrevClip);
	ngx_matrix(glm::inverse(cliptoprevious), options.prevClipToClip);
	// the offset as dlss takes it, along the rows turned over
	auto const jitter{Inputs.jitter * glm::vec2{s.flip_x ? -1.f : 1.f, s.flip_y ? 1.f : -1.f}};
	options.jitterOffset[0] = jitter.x;
	options.jitterOffset[1] = jitter.y;
	// the velocities are parts of the image along the rows of the renderer: pixels of the image turned over
	options.mvecScale[0] = static_cast<float>(s.generation_depth.width);
	options.mvecScale[1] = -static_cast<float>(s.generation_depth.height);
	options.cameraPinholeOffset[0] = 0.f;
	options.cameraPinholeOffset[1] = 0.f;
	for (int i = 0; i < 3; ++i)
	{
		options.cameraPos[i] = Inputs.position[i];
		options.cameraUp[i] = Inputs.up[i];
		options.cameraRight[i] = Inputs.right[i];
		options.cameraFwd[i] = Inputs.forward[i];
	}
	options.cameraNear = Inputs.znear;
	options.cameraFar = Inputs.zfar;
	options.cameraFOV = Inputs.fov;
	options.cameraAspectRatio = Inputs.aspect;
	options.colorBuffersHDR = false;
	options.depthInverted = true;
	options.cameraMotionIncluded = true;
	options.reset = Inputs.reset || s.generation_reset_wanted;
	s.generation_reset_wanted = false;
	options.automodeOverrideReset = false;
	options.notRenderingGameFrames = false;
	options.orthoProjection = false;
	// no value the velocities have marks them as missing
	options.motionVectorsInvalidValue = FLT_MAX;
	options.motionVectorsDilated = false;
	options.menuDetectionEnabled = false;
	options.mvecsSubrectSize = {s.generation_motion.width, s.generation_motion.height};
	options.depthSubrectSize = {s.generation_depth.width, s.generation_depth.height};
	options.hudLessSubrectSize = {s.hudless.width, s.hudless.height};
	options.backbufferSubrectSize = {s.backbuffer.width, s.backbuffer.height};
	options.outputInterpSubrectSize = {s.backbuffer.width, s.backbuffer.height};
	Commands.external(&state::record_generation, &s);
}

std::string dlss::sync()
{
	auto &s{*m_state};
	s.seen_failed = s.failed;
	s.seen_error = s.error;
	s.seen_feature = ((s.handle != nullptr) ? s.made : s.wanted);
	s.seen_memory = s.memory;
	s.seen_generation_failed = s.generation_failed;
	s.seen_generations = s.generations;
	if (s.failed && (false == s.error_told))
	{
		s.error_told = true;
		return s.error;
	}
	if (s.generation_failed && (false == s.generation_error_told))
	{
		s.generation_error_told = true;
		return s.generation_error;
	}
	return {};
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
	       std::to_string(s.seen_memory / (1024 * 1024)) + " MiB" +
	       (s.generation_max == 0       ? std::string{}
	        : s.seen_generation_failed ? ", the frame generation off after an error"
	                                   : ", the frame generation up to x" + std::to_string(s.generation_max + 1) + " (" + std::to_string(s.seen_generations) + " frames generated from)");
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
	if (s.generation_max > 0)
	{
		ImGui::Checkbox("frame generation: the clip space of its matrices turned over", &s.generation_clip_flip);
		ImGui::SameLine();
		if (ImGui::Button("drop its history"))
		{
			s.generation_reset_wanted = true;
		}
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

std::uint32_t dlss::generation_max() const
{
	return 0;
}

bool dlss::generation_failed() const
{
	return false;
}

void dlss::generate(command_stream &, generation_inputs const &, deletion_queue &, std::uint64_t) {}

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
