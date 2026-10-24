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
#include "rendering/vulkan14/vulkan14device.h"
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
	auto const temporary{Path + ".tmp"};
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

std::shared_ptr<shader_code const> shader_library::get(std::string const &Name, std::string const &Defines)
{
	auto const key{Name + "|" + Defines};
	auto const lookup{m_shaders.find(key)};
	if (lookup != m_shaders.end())
	{
		return lookup->second;
	}

	auto code{std::make_shared<shader_code>()};
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
			++m_frombinary;
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
			++m_fromcache;
		}
		else
		{
			code->spirv = compile(code->source);
			code->origin = "compiled";
			++m_compiled;
			write_file(cachepath, code->spirv.data(), code->spirv.size() * sizeof(std::uint32_t));
		}
	}
	// the text isn't needed any more
	code->source.text = std::string();
	m_shaders.emplace(key, code);
	return code;
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

VkPipeline pipeline_library::get(pipeline_desc const &Desc)
{
	auto const key{key_of(Desc)};
	auto const lookup{m_pipelines.find(key)};
	if (lookup != m_pipelines.end())
	{
		return lookup->second;
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
	m_creationms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
	m_pipelines.emplace(key, pipeline);
	return pipeline;
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
	auto const lookup{m_pipelines.find(key)};
	if (lookup != m_pipelines.end())
	{
		return lookup->second;
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
	m_creationms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
	m_pipelines.emplace(key, pipeline);
	return pipeline;
}

std::string pipeline_library::info() const
{
	char buffer[32] = {};
	std::snprintf(buffer, sizeof(buffer), "%.1f", m_creationms);
	return "pipelines: " + std::to_string(m_pipelines.size()) + " (made in " + buffer + " ms)";
}

void set_draw_state(VkCommandBuffer Commands, VkExtent2D Extent)
{
	VkViewport viewport{0.f, 0.f, static_cast<float>(Extent.width), static_cast<float>(Extent.height), 0.f, 1.f};
	vkCmdSetViewport(Commands, 0, 1, &viewport);
	VkRect2D scissor{{0, 0}, Extent};
	vkCmdSetScissor(Commands, 0, 1, &scissor);
	vkCmdSetCullMode(Commands, VK_CULL_MODE_NONE);
	vkCmdSetFrontFace(Commands, VK_FRONT_FACE_COUNTER_CLOCKWISE);
	vkCmdSetPrimitiveTopology(Commands, VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
	vkCmdSetDepthTestEnable(Commands, VK_FALSE);
	vkCmdSetDepthWriteEnable(Commands, VK_FALSE);
	// reverse z, as the opengl renderer
	vkCmdSetDepthCompareOp(Commands, VK_COMPARE_OP_GREATER_OR_EQUAL);
	vkCmdSetDepthBiasEnable(Commands, VK_FALSE);
	vkCmdSetDepthBias(Commands, 0.f, 0.f, 0.f);
	vkCmdSetPrimitiveRestartEnable(Commands, VK_FALSE);
	vkCmdSetStencilTestEnable(Commands, VK_FALSE);
}

} // namespace vk14
