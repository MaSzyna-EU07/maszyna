/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14shaders.h"
#include "rendering/vulkan14/vulkan14commands.h"
#include "rendering/vulkan14/vulkan14device.h"
#include "rendering/vulkan14/vulkan14bindings.h"
#include "rendering/common/localshadows.h"
#include "gl/ubo.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"

#include <glslang/Public/ResourceLimits.h>
#include <glslang/Public/ShaderLang.h>
#include <SPIRV/GlslangToSpv.h>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>

namespace vk14
{

namespace
{

EShLanguage language_of(gfx::shader_stage const Stage)
{
	switch (Stage)
	{
	case gfx::shader_stage::vertex:
		return EShLangVertex;
	case gfx::shader_stage::fragment:
		return EShLangFragment;
	case gfx::shader_stage::geometry:
		return EShLangGeometry;
	case gfx::shader_stage::compute:
		return EShLangCompute;
	}
	return EShLangVertex;
}

VkShaderStageFlagBits stage_bit(gfx::shader_stage const Stage)
{
	switch (Stage)
	{
	case gfx::shader_stage::vertex:
		return VK_SHADER_STAGE_VERTEX_BIT;
	case gfx::shader_stage::fragment:
		return VK_SHADER_STAGE_FRAGMENT_BIT;
	case gfx::shader_stage::geometry:
		return VK_SHADER_STAGE_GEOMETRY_BIT;
	case gfx::shader_stage::compute:
		return VK_SHADER_STAGE_COMPUTE_BIT;
	}
	return VK_SHADER_STAGE_VERTEX_BIT;
}

std::string files_of(gfx::shader_source const &Source)
{
	std::string result;
	for (std::size_t i = 0; i < Source.files.size(); ++i)
	{
		result += (i > 0 ? ", " : "") + std::to_string(i) + ": " + Source.files[i];
	}
	return result;
}

std::vector<std::uint32_t> read_spirv(std::string const &Path)
{
	std::ifstream file(Path, std::ios::binary | std::ios::ate);
	if (false == file.is_open())
	{
		return {};
	}
	auto const size{static_cast<std::size_t>(file.tellg())};
	if ((size < 20) || (size % 4 != 0))
	{
		return {};
	}
	std::vector<std::uint32_t> words(size / 4);
	file.seekg(0);
	file.read(reinterpret_cast<char *>(words.data()), static_cast<std::streamsize>(size));
	if ((false == static_cast<bool>(file)) || (words[0] != 0x07230203u))
	{
		return {};
	}
	return words;
}

// written beside and moved into place, so a program reading the cache at the same time doesn't get half of the file
void write_file(std::string const &Path, void const *Data, std::size_t const Size)
{
	std::error_code error;
	std::filesystem::create_directories(std::filesystem::path(Path).parent_path(), error);
	// a temporary file of each thread: two of them may write the same file of the cache at once
	auto const temporary{Path + ".tmp" + std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id()))};
	{
		std::ofstream file(temporary, std::ios::binary | std::ios::trunc);
		if (false == file.is_open())
		{
			return;
		}
		file.write(static_cast<char const *>(Data), static_cast<std::streamsize>(Size));
		if (false == static_cast<bool>(file))
		{
			return;
		}
	}
	std::filesystem::rename(temporary, Path, error);
	if (error)
	{
		std::filesystem::remove(temporary, error);
	}
}

std::string hex(std::uint64_t const Value)
{
	char buffer[17] = {};
	std::snprintf(buffer, sizeof(buffer), "%016llx", static_cast<unsigned long long>(Value));
	return buffer;
}

// fnv-1a 64 over more data
void hash_bytes(std::uint64_t &Hash, void const *Data, std::size_t const Size)
{
	auto const *bytes{static_cast<std::uint8_t const *>(Data)};
	for (std::size_t i = 0; i < Size; ++i)
	{
		Hash ^= bytes[i];
		Hash *= 1099511628211ull;
	}
}

template <typename Type_> void hash_value(std::uint64_t &Hash, Type_ const &Value)
{
	hash_bytes(Hash, &Value, sizeof(Value));
}

std::string user_cache_dir(std::string const &Name)
{
	// with a separator at the end
	auto const path{user_config_path("cache")};
	auto const name{Name.empty() ? std::string{} : Name + "/"};
	if (path.empty())
	{
		return "cache/vulkan14/" + name;
	}
	return (path / "vulkan14").string() + "/" + name;
}

} // namespace

// shader library

shader_library::~shader_library()
{
	shutdown();
}

bool shader_library::init(std::string const &Preamble, std::string &Reason)
{
	m_preamble = Preamble;
	if (false == glslang::InitializeProcess())
	{
		Reason = "the shader compiler (glslang) can't start";
		return false;
	}
	m_glslang = true;
	auto const version{glslang::GetVersion()};
	m_compilerversion = "glslang " + std::to_string(version.major) + "." + std::to_string(version.minor) + "." + std::to_string(version.patch) + " vulkan1.4 spirv1.6";
	m_cachedir = user_cache_dir("shaders");

	m_binaries = Global.gfx_shader_binaries;
	if (m_binaries && (Preamble.find("#define REFLECTION_RAYS") != std::string::npos))
	{
		// the binaries are made without the ray traced reflections
		log_info("shaders: the ray traced reflections are on (gfx.raytracing), the SPIR-V binaries aren't used");
		m_binaries = false;
	}
	if (m_binaries)
	{
		// the binaries are compiled with the constants written in the files, the sources get those of the renderer
		std::vector<std::string> differences;
		if (false == gfx::shader_constants_match(gfx::shader_api::vulkan14, "common.glsl", Preamble, {}, differences))
		{
			std::string text;
			for (auto const &difference : differences)
			{
				text += "\n  " + difference;
			}
			ErrorLog("vulkan: the constants in " + gfx::shader_source_dir(gfx::shader_api::vulkan14) + "common.glsl aren't those of the renderer, so the SPIR-V binaries of " +
			             gfx::shader_binary_dir(gfx::shader_api::vulkan14) + " can't be used; the shaders are compiled from the sources:" + text,
			         logtype::shader);
			m_binaries = false;
		}
	}
	log_info("shaders: " + std::string(m_binaries ? "the SPIR-V binaries of " + gfx::shader_binary_dir(gfx::shader_api::vulkan14) + " first, then " : "") + "the cache in " + m_cachedir + ", then " + m_compilerversion);
	return true;
}

void shader_library::shutdown()
{
	m_shaders.clear();
	if (m_glslang)
	{
		glslang::FinalizeProcess();
		m_glslang = false;
	}
}

std::shared_ptr<shader_code const> shader_library::find(std::string const &Name, std::string const &Defines) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	auto const lookup{m_shaders.find(Name + "|" + Defines)};
	return (lookup != m_shaders.end() ? lookup->second : nullptr);
}

std::shared_ptr<shader_code const> shader_library::get(std::string const &Name, std::string const &Defines)
{
	auto const key{Name + "|" + Defines};
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		auto const lookup{m_shaders.find(key)};
		if (lookup != m_shaders.end())
		{
			return lookup->second;
		}
	}
	// read or compiled outside of the lock: two threads asking for the same shader at once both make it, the first one kept

	auto code{std::make_shared<shader_code>()};
	code->name = Name;
	code->defines = Defines;
	code->source = gfx::preprocess_shader(gfx::shader_api::vulkan14, Name, Defines, m_preamble, true);
	code->stage = stage_bit(code->source.stage);
	auto const label{Name + (code->source.variant.empty() ? "" : " (" + code->source.variant + ")")};

	if (m_binaries)
	{
		std::string reason;
		code->spirv = gfx::load_shader_binary(code->source, reason);
		if (false == reason.empty())
		{
			log_info("shader " + label + ": " + reason + ", compiled from the source");
		}
		if (false == code->spirv.empty())
		{
			code->origin = "binary";
		}
	}
	if (code->spirv.empty())
	{
		if (false == code->source.has_source)
		{
			throw gfx::shader_exception("cannot read shader: " + gfx::shader_source_dir(gfx::shader_api::vulkan14) + Name);
		}
		std::uint64_t hash{14695981039346656037ull};
		hash_bytes(hash, code->source.text.data(), code->source.text.size());
		hash_bytes(hash, m_compilerversion.data(), m_compilerversion.size());
		auto const cachepath{m_cachedir + Name + (code->source.variant.empty() ? "" : "." + code->source.variant) + "." + hex(hash) + ".spv"};
		code->spirv = read_spirv(cachepath);
		if (false == code->spirv.empty())
		{
			code->origin = "cache";
		}
		else
		{
			code->spirv = compile(code->source);
			code->origin = "compiled";
			write_file(cachepath, code->spirv.data(), code->spirv.size() * sizeof(std::uint32_t));
		}
	}
	// the text isn't needed any more
	code->source.text = std::string();
	std::lock_guard<std::mutex> lock(m_mutex);
	auto const [entry, added]{m_shaders.emplace(key, code)};
	if (added)
	{
		auto const origin{std::string(code->origin)};
		++(origin == "binary" ? m_frombinary : origin == "cache" ? m_fromcache : m_compiled);
	}
	return entry->second;
}

std::vector<std::uint32_t> shader_library::compile(gfx::shader_source const &Source) const
{
	auto const language{language_of(Source.stage)};
	glslang::TShader shader(language);
	char const *text{Source.text.c_str()};
	char const *name{Source.name.c_str()};
	shader.setStringsWithLengthsAndNames(&text, nullptr, &name, 1);
	shader.setEnvInput(glslang::EShSourceGlsl, language, glslang::EShClientVulkan, 100);
	shader.setEnvClient(glslang::EShClientVulkan, glslang::EShTargetVulkan_1_4);
	shader.setEnvTarget(glslang::EShTargetSpv, glslang::EShTargetSpv_1_6);
	auto const messages{static_cast<EShMessages>(EShMsgSpvRules | EShMsgVulkanRules)};
	if (false == shader.parse(GetDefaultResources(), 460, false, messages))
	{
		auto const info{std::string(shader.getInfoLog())};
		ErrorLog("bad shader: " + Source.name + ": " + info + "(source strings: " + files_of(Source) + ")", logtype::shader);
		throw gfx::shader_exception("failed to compile " + Source.name + ": " + info);
	}
	glslang::TProgram program;
	program.addShader(&shader);
	if (false == program.link(messages))
	{
		auto const info{std::string(program.getInfoLog())};
		ErrorLog("bad shader: " + Source.name + ": " + info, logtype::shader);
		throw gfx::shader_exception("failed to link " + Source.name + ": " + info);
	}
	std::vector<std::uint32_t> spirv;
	glslang::SpvOptions options;
	options.validate = false;
	glslang::GlslangToSpv(*program.getIntermediate(language), spirv, &options);
	if (spirv.empty())
	{
		throw gfx::shader_exception("no SPIR-V made of " + Source.name);
	}
	return spirv;
}

std::string shader_library::info() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return "shaders: " + std::to_string(m_shaders.size()) + " (" + std::to_string(m_frombinary) + " binaries, " + std::to_string(m_fromcache) + " from the cache, " + std::to_string(m_compiled) + " compiled)";
}

// pipeline library

pipeline_library::~pipeline_library()
{
	shutdown();
}

bool pipeline_library::init(device const &Device, std::string &Reason)
{
	m_device = &Device;
	auto const cachedir{user_cache_dir("")};
	m_cachepath = cachedir + "pipelines.bin";

	// the cache of another device or driver is no good; vulkan would ignore it, but it would be written over anyway
	std::vector<char> data;
	{
		std::ifstream file(m_cachepath, std::ios::binary | std::ios::ate);
		if (file.is_open())
		{
			data.resize(static_cast<std::size_t>(file.tellg()));
			file.seekg(0);
			file.read(data.data(), static_cast<std::streamsize>(data.size()));
			if (false == static_cast<bool>(file))
			{
				data.clear();
			}
		}
	}
	if (false == data.empty())
	{
		VkPipelineCacheHeaderVersionOne header{};
		auto const &caps{Device.caps()};
		bool valid{data.size() >= sizeof(header)};
		if (valid)
		{
			std::memcpy(&header, data.data(), sizeof(header));
			valid = (header.headerSize >= sizeof(header)) && (header.headerVersion == VK_PIPELINE_CACHE_HEADER_VERSION_ONE) && (header.vendorID == caps.vendor_id) && (header.deviceID == caps.device_id) &&
			        (std::memcmp(header.pipelineCacheUUID, caps.pipeline_cache_uuid.data(), VK_UUID_SIZE) == 0);
		}
		if (false == valid)
		{
			log_info("the pipeline cache " + m_cachepath + " comes from another device or driver, made anew");
			data.clear();
		}
	}

	VkPipelineCacheCreateInfo info{};
	info.sType = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO;
	info.initialDataSize = data.size();
	info.pInitialData = (data.empty() ? nullptr : data.data());
	if (!VK14_VERIFY(vkCreatePipelineCache(Device.handle(), &info, nullptr, &m_cache)))
	{
		// without the old data, then
		info.initialDataSize = 0;
		info.pInitialData = nullptr;
		if (!VK14_VERIFY(vkCreatePipelineCache(Device.handle(), &info, nullptr, &m_cache)))
		{
			m_cache = VK_NULL_HANDLE;
			Reason = "no pipeline cache";
			return false;
		}
	}
	log_info("pipeline cache " + m_cachepath + (data.empty() ? " (empty)" : " (" + std::to_string(data.size() / 1024) + " KiB)"));
	return true;
}

void pipeline_library::shutdown()
{
	if (m_device == nullptr)
	{
		return;
	}
	auto const device{m_device->handle()};
	if (device != VK_NULL_HANDLE)
	{
		for (auto const &[key, pipeline] : m_pipelines)
		{
			if (pipeline != VK_NULL_HANDLE)
			{
				vkDestroyPipeline(device, pipeline, nullptr);
			}
		}
		if (m_cache != VK_NULL_HANDLE)
		{
			std::size_t size{0};
			if (VK14_VERIFY(vkGetPipelineCacheData(device, m_cache, &size, nullptr)) && (size > 0))
			{
				std::vector<char> data(size);
				if (VK14_VERIFY(vkGetPipelineCacheData(device, m_cache, &size, data.data())))
				{
					write_file(m_cachepath, data.data(), size);
				}
			}
			vkDestroyPipelineCache(device, m_cache, nullptr);
		}
	}
	m_pipelines.clear();
	m_cache = VK_NULL_HANDLE;
	m_device = nullptr;
}

std::uint64_t pipeline_library::key_of(pipeline_desc const &Desc) const
{
	std::uint64_t hash{14695981039346656037ull};
	for (auto const &stage : Desc.stages)
	{
		hash_value(hash, stage.get());
	}
	hash_value(hash, Desc.layout);
	for (auto const &binding : Desc.vertices.bindings)
	{
		hash_value(hash, binding.binding);
		hash_value(hash, binding.stride);
		hash_value(hash, binding.inputRate);
	}
	for (auto const &attribute : Desc.vertices.attributes)
	{
		hash_value(hash, attribute.location);
		hash_value(hash, attribute.binding);
		hash_value(hash, attribute.format);
		hash_value(hash, attribute.offset);
	}
	hash_value(hash, Desc.topology);
	hash_value(hash, Desc.blending);
	for (auto const format : Desc.colour_formats)
	{
		hash_value(hash, format);
	}
	hash_value(hash, Desc.depth_format);
	hash_value(hash, Desc.samples);
	hash_value(hash, Desc.view_mask);
	for (auto const value : Desc.specialization)
	{
		hash_value(hash, value);
	}
	return hash;
}

namespace
{

VkPrimitiveTopology topology_class_head(VkPrimitiveTopology const Topology)
{
	return Topology;
}

std::vector<VkDynamicState> const dynamic_states{VK_DYNAMIC_STATE_VIEWPORT,
                                                 VK_DYNAMIC_STATE_SCISSOR,
                                                 VK_DYNAMIC_STATE_CULL_MODE,
                                                 VK_DYNAMIC_STATE_FRONT_FACE,
                                                 VK_DYNAMIC_STATE_PRIMITIVE_TOPOLOGY,
                                                 VK_DYNAMIC_STATE_DEPTH_TEST_ENABLE,
                                                 VK_DYNAMIC_STATE_DEPTH_WRITE_ENABLE,
                                                 VK_DYNAMIC_STATE_DEPTH_COMPARE_OP,
                                                 VK_DYNAMIC_STATE_DEPTH_BIAS_ENABLE,
                                                 VK_DYNAMIC_STATE_DEPTH_BIAS,
                                                 VK_DYNAMIC_STATE_PRIMITIVE_RESTART_ENABLE,
                                                 VK_DYNAMIC_STATE_STENCIL_TEST_ENABLE};

} // namespace

bool pipeline_library::find(pipeline_desc const &Desc, VkPipeline &Pipeline) const
{
	auto const key{key_of(Desc)};
	std::lock_guard<std::mutex> lock(m_mutex);
	auto const lookup{m_pipelines.find(key)};
	if (lookup == m_pipelines.end())
	{
		return false;
	}
	Pipeline = lookup->second;
	return true;
}

VkPipeline pipeline_library::keep(std::uint64_t const Key, VkPipeline const Pipeline, double const Milliseconds)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_creationms += Milliseconds;
	auto const [entry, added]{m_pipelines.emplace(Key, Pipeline)};
	if ((false == added) && (Pipeline != VK_NULL_HANDLE) && (entry->second != Pipeline))
	{
		// made by another thread meanwhile
		vkDestroyPipeline(m_device->handle(), Pipeline, nullptr);
	}
	return entry->second;
}

VkPipeline pipeline_library::get(pipeline_desc const &Desc)
{
	auto const key{key_of(Desc)};
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		auto const lookup{m_pipelines.find(key)};
		if (lookup != m_pipelines.end())
		{
			return lookup->second;
		}
	}
	auto const start{std::chrono::steady_clock::now()};

	// the code goes straight to the stages (maintenance5), with the specialization of all of them
	std::vector<VkSpecializationMapEntry> entries(Desc.specialization.size());
	for (std::size_t i = 0; i < entries.size(); ++i)
	{
		entries[i] = {static_cast<std::uint32_t>(i), static_cast<std::uint32_t>(i * sizeof(std::uint32_t)), sizeof(std::uint32_t)};
	}
	VkSpecializationInfo specialization{};
	specialization.mapEntryCount = static_cast<std::uint32_t>(entries.size());
	specialization.pMapEntries = entries.data();
	specialization.dataSize = Desc.specialization.size() * sizeof(std::uint32_t);
	specialization.pData = Desc.specialization.data();

	std::vector<VkShaderModuleCreateInfo> modules(Desc.stages.size());
	std::vector<VkPipelineShaderStageCreateInfo> stages(Desc.stages.size());
	for (std::size_t i = 0; i < Desc.stages.size(); ++i)
	{
		modules[i].sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
		modules[i].codeSize = Desc.stages[i]->spirv.size() * sizeof(std::uint32_t);
		modules[i].pCode = Desc.stages[i]->spirv.data();
		stages[i].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		stages[i].pNext = &modules[i];
		stages[i].stage = Desc.stages[i]->stage;
		stages[i].module = VK_NULL_HANDLE;
		stages[i].pName = "main";
		stages[i].pSpecializationInfo = (entries.empty() ? nullptr : &specialization);
	}

	VkPipelineVertexInputStateCreateInfo vertexinput{};
	vertexinput.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
	vertexinput.vertexBindingDescriptionCount = static_cast<std::uint32_t>(Desc.vertices.bindings.size());
	vertexinput.pVertexBindingDescriptions = Desc.vertices.bindings.data();
	vertexinput.vertexAttributeDescriptionCount = static_cast<std::uint32_t>(Desc.vertices.attributes.size());
	vertexinput.pVertexAttributeDescriptions = Desc.vertices.attributes.data();

	VkPipelineInputAssemblyStateCreateInfo inputassembly{};
	inputassembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
	inputassembly.topology = topology_class_head(Desc.topology);

	VkPipelineViewportStateCreateInfo viewport{};
	viewport.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
	viewport.viewportCount = 1;
	viewport.scissorCount = 1;

	VkPipelineRasterizationStateCreateInfo rasterization{};
	rasterization.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
	rasterization.polygonMode = VK_POLYGON_MODE_FILL;
	rasterization.cullMode = VK_CULL_MODE_NONE;
	rasterization.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
	rasterization.lineWidth = 1.f;

	VkPipelineMultisampleStateCreateInfo multisample{};
	multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
	multisample.rasterizationSamples = Desc.samples;

	VkPipelineDepthStencilStateCreateInfo depthstencil{};
	depthstencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
	depthstencil.depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL;
	depthstencil.maxDepthBounds = 1.f;

	std::vector<VkPipelineColorBlendAttachmentState> attachments(Desc.colour_formats.size());
	for (auto &attachment : attachments)
	{
		attachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
		switch (Desc.blending)
		{
		case blend_mode::none:
			break;
		case blend_mode::no_colour:
			attachment.colorWriteMask = 0;
			break;
		case blend_mode::alpha:
			attachment.blendEnable = VK_TRUE;
			attachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
			attachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
			attachment.colorBlendOp = VK_BLEND_OP_ADD;
			attachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
			attachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
			attachment.alphaBlendOp = VK_BLEND_OP_ADD;
			break;
		case blend_mode::premultiplied:
			attachment.blendEnable = VK_TRUE;
			attachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
			attachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
			attachment.colorBlendOp = VK_BLEND_OP_ADD;
			attachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
			attachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
			attachment.alphaBlendOp = VK_BLEND_OP_ADD;
			break;
		case blend_mode::scene_alpha:
			attachment.blendEnable = VK_TRUE;
			attachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
			attachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
			attachment.colorBlendOp = VK_BLEND_OP_ADD;
			attachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
			attachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
			attachment.alphaBlendOp = VK_BLEND_OP_ADD;
			break;
		case blend_mode::scene_additive:
			attachment.blendEnable = VK_TRUE;
			attachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
			attachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
			attachment.colorBlendOp = VK_BLEND_OP_ADD;
			attachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
			attachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
			attachment.alphaBlendOp = VK_BLEND_OP_ADD;
			break;
		case blend_mode::under_transmittance:
			attachment.blendEnable = VK_TRUE;
			attachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
			attachment.dstColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
			attachment.colorBlendOp = VK_BLEND_OP_ADD;
			attachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
			attachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
			attachment.alphaBlendOp = VK_BLEND_OP_ADD;
			break;
		case blend_mode::multiply:
			attachment.blendEnable = VK_TRUE;
			attachment.srcColorBlendFactor = VK_BLEND_FACTOR_ZERO;
			attachment.dstColorBlendFactor = VK_BLEND_FACTOR_SRC_COLOR;
			attachment.colorBlendOp = VK_BLEND_OP_ADD;
			attachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
			attachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
			attachment.alphaBlendOp = VK_BLEND_OP_ADD;
			break;
		case blend_mode::additive:
			attachment.blendEnable = VK_TRUE;
			attachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
			attachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
			attachment.colorBlendOp = VK_BLEND_OP_ADD;
			attachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
			attachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
			attachment.alphaBlendOp = VK_BLEND_OP_ADD;
			break;
		}
	}
	VkPipelineColorBlendStateCreateInfo blend{};
	blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
	blend.attachmentCount = static_cast<std::uint32_t>(attachments.size());
	blend.pAttachments = attachments.data();

	VkPipelineDynamicStateCreateInfo dynamic{};
	dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
	dynamic.dynamicStateCount = static_cast<std::uint32_t>(dynamic_states.size());
	dynamic.pDynamicStates = dynamic_states.data();

	// no render pass: the formats of the attachments only (dynamic rendering)
	VkPipelineRenderingCreateInfo rendering{};
	rendering.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
	rendering.colorAttachmentCount = static_cast<std::uint32_t>(Desc.colour_formats.size());
	rendering.pColorAttachmentFormats = Desc.colour_formats.data();
	rendering.depthAttachmentFormat = Desc.depth_format;
	rendering.viewMask = Desc.view_mask;

	VkGraphicsPipelineCreateInfo info{};
	info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
	info.pNext = &rendering;
	info.stageCount = static_cast<std::uint32_t>(stages.size());
	info.pStages = stages.data();
	info.pVertexInputState = &vertexinput;
	info.pInputAssemblyState = &inputassembly;
	info.pViewportState = &viewport;
	info.pRasterizationState = &rasterization;
	info.pMultisampleState = &multisample;
	info.pDepthStencilState = &depthstencil;
	info.pColorBlendState = &blend;
	info.pDynamicState = &dynamic;
	info.layout = Desc.layout;

	VkPipeline pipeline{VK_NULL_HANDLE};
	if (!VK14_VERIFY(vkCreateGraphicsPipelines(m_device->handle(), m_cache, 1, &info, nullptr, &pipeline)))
	{
		log_error("the pipeline " + Desc.name + " can't be made");
		pipeline = VK_NULL_HANDLE;
	}
	else
	{
		name_object(m_device->handle(), VK_OBJECT_TYPE_PIPELINE, handle_value(pipeline), Desc.name);
	}
	return keep(key, pipeline, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
}

VkPipeline pipeline_library::compute(std::shared_ptr<shader_code const> const &Shader, VkPipelineLayout Layout, std::vector<std::uint32_t> const &Specialization, std::string const &Name)
{
	std::uint64_t key{14695981039346656037ull};
	hash_value(key, Shader.get());
	hash_value(key, Layout);
	for (auto const value : Specialization)
	{
		hash_value(key, value);
	}
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		auto const lookup{m_pipelines.find(key)};
		if (lookup != m_pipelines.end())
		{
			return lookup->second;
		}
	}
	auto const start{std::chrono::steady_clock::now()};

	std::vector<VkSpecializationMapEntry> entries(Specialization.size());
	for (std::size_t i = 0; i < entries.size(); ++i)
	{
		entries[i] = {static_cast<std::uint32_t>(i), static_cast<std::uint32_t>(i * sizeof(std::uint32_t)), sizeof(std::uint32_t)};
	}
	VkSpecializationInfo specialization{};
	specialization.mapEntryCount = static_cast<std::uint32_t>(entries.size());
	specialization.pMapEntries = entries.data();
	specialization.dataSize = Specialization.size() * sizeof(std::uint32_t);
	specialization.pData = Specialization.data();

	VkShaderModuleCreateInfo module{};
	module.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
	module.codeSize = Shader->spirv.size() * sizeof(std::uint32_t);
	module.pCode = Shader->spirv.data();

	VkComputePipelineCreateInfo info{};
	info.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
	info.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	info.stage.pNext = &module;
	info.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
	info.stage.pName = "main";
	info.stage.pSpecializationInfo = (entries.empty() ? nullptr : &specialization);
	info.layout = Layout;

	VkPipeline pipeline{VK_NULL_HANDLE};
	if (!VK14_VERIFY(vkCreateComputePipelines(m_device->handle(), m_cache, 1, &info, nullptr, &pipeline)))
	{
		log_error("the compute pipeline " + Name + " can't be made");
		pipeline = VK_NULL_HANDLE;
	}
	else
	{
		name_object(m_device->handle(), VK_OBJECT_TYPE_PIPELINE, handle_value(pipeline), Name);
	}
	return keep(key, pipeline, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
}

std::string pipeline_library::info() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	char buffer[32] = {};
	std::snprintf(buffer, sizeof(buffer), "%.1f", m_creationms);
	return "pipelines: " + std::to_string(m_pipelines.size()) + " (made in " + buffer + " ms)";
}

void set_draw_state(command_stream *Commands, VkExtent2D Extent)
{
	VkViewport viewport{0.f, 0.f, static_cast<float>(Extent.width), static_cast<float>(Extent.height), 0.f, 1.f};
	Commands->set_viewport(0, 1, &viewport);
	VkRect2D scissor{{0, 0}, Extent};
	Commands->set_scissor(0, 1, &scissor);
	Commands->set_cull_mode(VK_CULL_MODE_NONE);
	Commands->set_front_face(VK_FRONT_FACE_COUNTER_CLOCKWISE);
	Commands->set_primitive_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
	Commands->set_depth_test_enable(VK_FALSE);
	Commands->set_depth_write_enable(VK_FALSE);
	// reverse z, as the opengl renderer
	Commands->set_depth_compare_op(VK_COMPARE_OP_GREATER_OR_EQUAL);
	Commands->set_depth_bias_enable(VK_FALSE);
	Commands->set_depth_bias(0.f, 0.f, 0.f);
	Commands->set_primitive_restart_enable(VK_FALSE);
	Commands->set_stencil_test_enable(VK_FALSE);
}

} // namespace vk14

namespace vk14
{

std::string binding_preamble(bool const Raytracing)
{
	std::string preamble{"#define MASZYNA_CONSTANTS 1\n"};
	if (Raytracing)
	{
		// the extensions go before anything else of the source; shaders/vk14/rt_reflections.glsl says what they're for
		preamble += "#extension GL_EXT_ray_query : require\n"
		            "#extension GL_EXT_buffer_reference2 : require\n"
		            "#extension GL_EXT_buffer_reference_uvec2 : require\n"
		            "#extension GL_EXT_nonuniform_qualifier : require\n"
		            "#define REFLECTION_RAYS 1\n";
	}
	auto const define = [&preamble](char const *Name, auto const Value, char const *Suffix = "") { preamble += std::string("#define ") + Name + " " + std::to_string(Value) + Suffix + "\n"; };
	using namespace vk14::bindings;
	define("UI_SET", UI_SET);
	define("UI_TEXTURE", UI_TEXTURE);
	define("PASS_SET", PASS_SET);
	define("MATERIAL_SET", MATERIAL_SET);
	define("SCENE_UBO", SCENE_UBO);
	define("DRAW_SSBO", DRAW_SSBO);
	define("INSTANCE_SSBO", INSTANCE_SSBO);
	define("LIGHT_SSBO", LIGHT_SSBO);
	define("CLUSTER_UBO", CLUSTER_UBO);
	define("CLUSTER_RANGE_SSBO", CLUSTER_RANGE_SSBO);
	define("CLUSTER_INDEX_SSBO", CLUSTER_INDEX_SSBO);
	define("SHADOW_TEX", SHADOW_TEX);
	define("ENV_TEX", ENV_TEX);
	define("HEADLIGHT_TEX", HEADLIGHT_TEX);
	define("SKY_SSBO", SKY_SSBO);
	define("ATMOSPHERE_UBO", ATMOSPHERE_UBO);
	define("SKY_VIEW_TEX", SKY_VIEW_TEX);
	define("ENV_GGX_TEX", ENV_GGX_TEX);
	define("WEATHER_UBO", WEATHER_UBO);
	define("WEATHER_HORIZON_UBO", WEATHER_HORIZON_UBO);
	define("FOG_VOLUME_TEX", FOG_VOLUME_TEX);
	define("CLOUD_SHADOW_TEX", CLOUD_SHADOW_TEX);
	define("CLOUD_SHAPE_TEX", CLOUD_SHAPE_TEX);
	define("CLOUD_DETAIL_TEX", CLOUD_DETAIL_TEX);
	define("CLOUD_COVERAGE_TEX", CLOUD_COVERAGE_TEX);
	define("LOCAL_SHADOW_TEX", LOCAL_SHADOW_TEX);
	define("LOCAL_SHADOW_LAMPS_TEX", LOCAL_SHADOW_LAMPS_TEX);
	define("HEADLIGHT_SHADOW_TEX", HEADLIGHT_SHADOW_TEX);
	define("SCENE_DEPTH_TEX", SCENE_DEPTH_TEX);
	define("SSR_HISTORY_TEX", SSR_HISTORY_TEX);
	define("WATER_SSBO", WATER_SSBO);
	define("SCENE_AO_TEX", SCENE_AO_TEX);
	define("CAB_PROBE_TEX", CAB_PROBE_TEX);
	define("CAB_PROBE_ALBEDO_TEX", CAB_PROBE_ALBEDO_TEX);
	define("CAB_PROBE_SSBO", CAB_PROBE_SSBO);
	define("SKY_OCCLUSION_HEIGHT_TEX", SKY_OCCLUSION_HEIGHT_TEX);
	define("SKY_OCCLUSION_TEX", SKY_OCCLUSION_TEX);
	define("CAB_PROBE_MASK_TEX", CAB_PROBE_MASK_TEX);
	define("PASS_BINDING_COUNT", PASS_BINDING_COUNT);
	define("MATERIAL_TEXTURE_FIRST", MATERIAL_TEXTURE_FIRST);
	define("CL_CLUSTER_UBO", CL_CLUSTER_UBO);
	define("CL_AABB_SSBO", CL_AABB_SSBO);
	define("CL_RANGE_SSBO", CL_RANGE_SSBO);
	define("CL_INDEX_SSBO", CL_INDEX_SSBO);
	define("CL_COUNTER_SSBO", CL_COUNTER_SSBO);
	define("CL_LIGHT_SSBO", CL_LIGHT_SSBO);
	define("POST_SET", POST_SET);
	define("POST_TEXTURE_FIRST", POST_TEXTURE_FIRST);
	define("POST_TEXTURES", POST_TEXTURES);
	define("POST_EXPOSURE_SSBO", POST_EXPOSURE_SSBO);
	define("COMPUTE_SET", COMPUTE_SET);
	define("RT_SET", RT_SET);
	define("RT_SCENE", RT_SCENE);
	define("RT_RECORDS", RT_RECORDS);
	define("RT_TERRAIN", RT_TERRAIN);
	define("RT_TEXTURES", RT_TEXTURES);
	define("PUSH_CONSTANTS_SIZE", PUSH_CONSTANTS_SIZE);
	define("SPEC_OUTPUT_GAMMA", SPEC_OUTPUT_GAMMA);
	define("SPEC_SHADOWMAP_ENABLED", SPEC_SHADOWMAP_ENABLED);
	define("SPEC_ENVMAP_ENABLED", SPEC_ENVMAP_ENABLED);
	define("SPEC_MOTIONBLUR_ENABLED", SPEC_MOTIONBLUR_ENABLED);
	define("SPEC_POSTFX_ENABLED", SPEC_POSTFX_ENABLED);
	define("SPEC_EXTRAEFFECTS_ENABLED", SPEC_EXTRAEFFECTS_ENABLED);
	define("SPEC_COUNT", SPEC_COUNT);
	// the constants of the game the blocks of the shaders are sized by, the same as for the opengl renderer (gl/ubo.h)
	define("MAX_LIGHTS", static_cast<unsigned int>(gl::MAX_LIGHTS), "U");
	define("MAX_CASCADES", static_cast<unsigned int>(gl::MAX_CASCADES), "U");
	define("MAX_PARAMS", static_cast<unsigned int>(gl::MAX_PARAMS), "U");
	define("MAX_HEADLIGHT_SHADOWS", static_cast<unsigned int>(gl::MAX_HEADLIGHT_SHADOWS), "U");
	define("CLUSTER_GRID_X", gl::CLUSTER_GRID_X);
	define("CLUSTER_GRID_Y", gl::CLUSTER_GRID_Y);
	define("CLUSTER_GRID_Z", gl::CLUSTER_GRID_Z);
	define("CLUSTER_COUNT", gl::CLUSTER_COUNT);
	define("MAX_LIGHTS_PER_CLUSTER", gl::MAX_LIGHTS_PER_CLUSTER);
	define("CLUSTER_BUILD_GROUP_SIZE", gl::CLUSTER_BUILD_GROUP_SIZE);
	define("CLUSTER_CULL_GROUP_SIZE", gl::CLUSTER_CULL_GROUP_SIZE);
	// the sizes of the stages P: the light of the sky, the exposure, the weather, the culling on the gpu, the shadow fit, the particles
	define("ENVMAP_GGX_LEVELS", static_cast<unsigned int>(gl::ENVMAP_GGX_LEVELS), "U");
	preamble += "#define ENVMAP_GGX_ROUGHNESS_MIN " + std::to_string(gl::ENVMAP_GGX_ROUGHNESS_MIN) + "\n";
	define("ENVMAP_GGX_GROUP_SIZE", static_cast<unsigned int>(gl::ENVMAP_GGX_GROUP_SIZE));
	define("EXPOSURE_HISTOGRAM_BINS", gl::EXPOSURE_HISTOGRAM_BINS);
	define("EXPOSURE_HISTOGRAM_GROUP_SIZE", gl::EXPOSURE_HISTOGRAM_GROUP_SIZE);
	define("ATMOSPHERE_TRANSMITTANCE_WIDTH", gl::ATMOSPHERE_TRANSMITTANCE_WIDTH);
	define("ATMOSPHERE_TRANSMITTANCE_HEIGHT", gl::ATMOSPHERE_TRANSMITTANCE_HEIGHT);
	define("ATMOSPHERE_MULTISCATTERING_SIZE", gl::ATMOSPHERE_MULTISCATTERING_SIZE);
	define("ATMOSPHERE_SKYVIEW_WIDTH", gl::ATMOSPHERE_SKYVIEW_WIDTH);
	define("ATMOSPHERE_SKYVIEW_HEIGHT", gl::ATMOSPHERE_SKYVIEW_HEIGHT);
	define("ATMOSPHERE_AMBIENT_GROUP_SIZE", gl::ATMOSPHERE_AMBIENT_GROUP_SIZE);
	define("MAX_CLOUD_LAYERS", static_cast<unsigned int>(gl::MAX_CLOUD_LAYERS));
	define("FOG_HORIZON_SAMPLES", static_cast<unsigned int>(gl::FOG_HORIZON_SAMPLES));
	define("WEATHER_GROUP_SIZE", gl::WEATHER_GROUP_SIZE);
	preamble += "#define FOG_VOLUME_BIAS " + std::to_string(gl::FOG_VOLUME_BIAS) + "\n";
	define("INSTANCE_CULL_GROUP_SIZE", gl::INSTANCE_CULL_GROUP_SIZE);
	define("INSTANCE_OFFSETS_GROUP_SIZE", gl::INSTANCE_OFFSETS_GROUP_SIZE);
	define("INSTANCE_COMMAND_GROUP_SIZE", gl::INSTANCE_COMMAND_GROUP_SIZE);
	define("INSTANCE_COUNTER_HEADER", gl::INSTANCE_COUNTER_HEADER);
	define("INSTANCE_JOB_SLEEPERS", static_cast<unsigned int>(gl::INSTANCE_JOB_SLEEPERS), "U");
	define("HIZ_GROUP_SIZE", gl::HIZ_GROUP_SIZE);
	define("SHADOW_FIT_GROUP_SIZE", gl::SHADOW_FIT_GROUP_SIZE);
	preamble += "#define LOCAL_SHADOW_NEAR " + std::to_string(gfx::local_shadows::shadow_near) + "\n";
	define("PARTICLE_GROUP_SIZE", gl::PARTICLE_GROUP_SIZE);
	define("WATER_SLOTS", gl::WATER_SLOTS);
	define("WATER_DROPS", gl::WATER_DROPS);
	define("WATER_GRID", gl::WATER_GRID);
	define("WATER_CELL_DROPS", gl::WATER_CELL_DROPS);
	define("WATER_TRAIL", gl::WATER_TRAIL);
	define("WATER_MAP", gl::WATER_MAP);
	define("WATER_GROUP_SIZE", gl::WATER_GROUP_SIZE);
	define("CAB_PROBE_SH_SIZE", static_cast<unsigned int>(gl::CAB_PROBE_SH_SIZE));
	define("CAB_PROBE_SH_GROUP_SIZE", static_cast<unsigned int>(gl::CAB_PROBE_SH_GROUP_SIZE));
	return preamble;
}

} // namespace vk14
