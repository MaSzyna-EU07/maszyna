/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the scene of the vulkan renderer, part one (the stages D1 and D2 of the plan and the draws of the walk): the resources the
// passes of the scene share (the layouts, the pass sets, the rings of the draws, the stand-in images), the programs and their
// pipelines, the geometry, the materials and the textures, and the draws the shared walk of the scene (gfx::scene_walk) makes,
// recorded into the pass under way. the passes are in vulkan14frame.cpp

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14context.h"
#include "rendering/vulkan14/vulkan14commands.h"
#include "rendering/nullrenderer.h"
#include "model/Model3d.h"
#include "model/AnimModel.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/color.h"

#include <unordered_set>

namespace
{

// the rings of the blocks of the draws and of the matrices of the instances, per frame in flight, and the windows of them the
// descriptors of the pass sets read
VkDeviceSize const draw_ring_size{8 * 1024 * 1024};
VkDeviceSize const draw_window{4 * 1024 * 1024};
VkDeviceSize const instance_ring_size{8 * 1024 * 1024};
VkDeviceSize const instance_window{4 * 1024 * 1024};

// the topology the pipelines of the class of the primitive are made with (the primitive itself is dynamic state)
VkPrimitiveTopology topology_class(VkPrimitiveTopology const Topology)
{
	switch (Topology)
	{
	case VK_PRIMITIVE_TOPOLOGY_POINT_LIST:
		return VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
	case VK_PRIMITIVE_TOPOLOGY_LINE_LIST:
	case VK_PRIMITIVE_TOPOLOGY_LINE_STRIP:
		return VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
	default:
		return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
	}
}

vk14::vertex_layout layout_of(vk14::vertex_kind const Kind)
{
	vk14::vertex_layout layout;
	switch (Kind)
	{
	case vk14::vertex_kind::scene:
		layout.bindings = {{0, sizeof(gfx::basic_vertex), VK_VERTEX_INPUT_RATE_VERTEX}, {1, sizeof(gfx::vertex_userdata), VK_VERTEX_INPUT_RATE_VERTEX}};
		layout.attributes = {{0, 0, VK_FORMAT_R32G32B32_SFLOAT, static_cast<std::uint32_t>(offsetof(gfx::basic_vertex, position))},
		                     {1, 0, VK_FORMAT_R32G32B32_SFLOAT, static_cast<std::uint32_t>(offsetof(gfx::basic_vertex, normal))},
		                     {2, 0, VK_FORMAT_R32G32_SFLOAT, static_cast<std::uint32_t>(offsetof(gfx::basic_vertex, texture))},
		                     {3, 0, VK_FORMAT_R32G32B32A32_SFLOAT, static_cast<std::uint32_t>(offsetof(gfx::basic_vertex, tangent))},
		                     {4, 1, VK_FORMAT_R32G32B32A32_SFLOAT, static_cast<std::uint32_t>(offsetof(gfx::vertex_userdata, data))}};
		break;
	case vk14::vertex_kind::colour:
		layout.bindings = {{0, sizeof(glm::vec3), VK_VERTEX_INPUT_RATE_VERTEX}, {1, sizeof(glm::vec3), VK_VERTEX_INPUT_RATE_VERTEX}};
		layout.attributes = {{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0}, {1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0}};
		break;
	case vk14::vertex_kind::star:
		// gfx::weather_state::star_vertices(): the direction and the magnitude, then the colour and the phase, of each star
		layout.bindings = {{0, 2 * sizeof(glm::vec4), VK_VERTEX_INPUT_RATE_VERTEX}};
		layout.attributes = {{0, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 0}, {1, 0, VK_FORMAT_R32G32B32A32_SFLOAT, sizeof(glm::vec4)}};
		break;
	case vk14::vertex_kind::particle:
		// position, colour, texture coordinates (opengl46_particles::particle_vertex)
		layout.bindings = {{0, 36, VK_VERTEX_INPUT_RATE_VERTEX}};
		layout.attributes = {{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0}, {1, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 12}, {2, 0, VK_FORMAT_R32G32_SFLOAT, 28}};
		break;
	case vk14::vertex_kind::position_uv:
		layout.bindings = {{0, sizeof(glm::vec3), VK_VERTEX_INPUT_RATE_VERTEX}, {1, sizeof(glm::vec2), VK_VERTEX_INPUT_RATE_VERTEX}};
		layout.attributes = {{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0}, {1, 1, VK_FORMAT_R32G32_SFLOAT, 0}};
		break;
	case vk14::vertex_kind::particle_instance:
		// the instances drawn from the storage the compute passes of the particles fill (gl::particle_draw)
		layout.bindings = {{0, sizeof(gl::particle_draw), VK_VERTEX_INPUT_RATE_INSTANCE}};
		layout.attributes = {{0, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 0}, {1, 0, VK_FORMAT_R32G32B32A32_SFLOAT, sizeof(glm::vec4)}, {2, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 2 * sizeof(glm::vec4)}};
		break;
	case vk14::vertex_kind::none:
	default:
		break;
	}
	return layout;
}

// the locations of the inputs a vertex shader reads (the decorations of its variables of the input class): the attributes it
// doesn't read are left out of its pipelines, which the validation layer reports otherwise
std::vector<std::uint32_t> vertex_input_locations(std::vector<std::uint32_t> const &Spirv)
{
	std::uint32_t constexpr op_decorate{71};
	std::uint32_t constexpr op_variable{59};
	std::uint32_t constexpr decoration_location{30};
	std::uint32_t constexpr storage_input{1};
	std::unordered_map<std::uint32_t, std::uint32_t> locations;
	std::vector<std::uint32_t> inputs;
	for (std::size_t word = 5; word < Spirv.size();)
	{
		auto const count{Spirv[word] >> 16};
		auto const opcode{Spirv[word] & 0xffff};
		if ((count == 0) || (word + count > Spirv.size()))
		{
			break;
		}
		if ((opcode == op_decorate) && (count >= 4) && (Spirv[word + 2] == decoration_location))
		{
			locations[Spirv[word + 1]] = Spirv[word + 3];
		}
		else if ((opcode == op_variable) && (count >= 4) && (Spirv[word + 3] == storage_input))
		{
			inputs.push_back(Spirv[word + 2]);
		}
		word += count;
	}
	std::vector<std::uint32_t> result;
	for (auto const input : inputs)
	{
		auto const lookup{locations.find(input)};
		if (lookup != locations.end())
		{
			result.push_back(lookup->second);
		}
	}
	return result;
}

// the type of a binding of the pass set
VkDescriptorType pass_binding_type(std::uint32_t const Binding)
{
	using namespace vk14::bindings;
	switch (Binding)
	{
	case SCENE_UBO:
	case CLUSTER_UBO:
	case WEATHER_UBO:
		return VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC;
	case DRAW_SSBO:
	case INSTANCE_SSBO:
	case LIGHT_SSBO:
		return VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC;
	case CLUSTER_RANGE_SSBO:
	case CLUSTER_INDEX_SSBO:
	case SKY_SSBO:
		return VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	case ATMOSPHERE_UBO:
	case WEATHER_HORIZON_UBO:
		return VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	default:
		return VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
	}
}

// the views of the images each pass set holds, by the binding (the buffers and the beams of the headlights aside): the real
// images where the set reads them, the stand-ins elsewhere
std::array<std::array<VkImageView, vk14::bindings::PASS_BINDING_COUNT>, 3> pass_set_images(vk14::scene_state const &scene)
{
	using namespace vk14::bindings;
	std::array<std::array<VkImageView, PASS_BINDING_COUNT>, 3> result{};
	static_assert(vk14::scene_state::set_count == 3, "the pass sets");
	for (std::size_t set = 0; set < result.size(); ++set)
	{
		auto &views{result[set]};
		auto const colour{set == vk14::scene_state::set_colour};
		auto const plain{set == vk14::scene_state::set_plain};
		views[SHADOW_TEX] = ((false == plain) && scene.shadow_map ? scene.shadow_map->view() : scene.shadow_empty->view());
		views[ENV_TEX] = (colour && scene.env_map ? scene.env_map->view() : scene.env_empty->view());
		views[SKY_VIEW_TEX] = scene.sky.skyview->view();
		views[ENV_GGX_TEX] = (colour && scene.sky.ggx ? scene.sky.ggx->view() : scene.env_empty->view());
		auto const volume{scene.volume_empty->view()};
		views[FOG_VOLUME_TEX] = (colour && scene.weather.fog_integrated ? scene.weather.fog_integrated->view() : volume);
		views[CLOUD_SHADOW_TEX] = (scene.weather.shadow_map ? scene.weather.shadow_map->view() : scene.white->view());
		views[CLOUD_SHAPE_TEX] = (scene.weather.shape_noise ? scene.weather.shape_noise->view() : volume);
		views[CLOUD_DETAIL_TEX] = (scene.weather.detail_noise ? scene.weather.detail_noise->view() : volume);
		views[CLOUD_COVERAGE_TEX] = (scene.weather.coverage_noise ? scene.weather.coverage_noise->view() : scene.white->view());
		// the shadow maps of the lamps and the headlights, which the light buffers of the colour passes read
		auto const *cab{scene.lamps.cab_map()};
		auto const *lamps{scene.lamps.lamp_maps()};
		auto const *headlights{scene.lamps.headlight_maps()};
		views[LOCAL_SHADOW_TEX] = (cab != nullptr ? cab->view() : scene.cube_shadow_empty->view());
		views[LOCAL_SHADOW_LAMPS_TEX] = (lamps != nullptr ? lamps->view() : scene.cube_shadow_empty->view());
		views[HEADLIGHT_SHADOW_TEX] = (headlights != nullptr ? headlights->view() : scene.headlight_shadow_empty->view());
		// the depth of the scene of the main view drawn before its colour, which the contact shadows read
		views[SCENE_DEPTH_TEX] = (colour && (false == scene.targets.empty()) && scene.targets.front().scene_depth ? scene.targets.front().scene_depth->view() : scene.depth_empty->view());
		// the image of the last frame of the main view, which the screen space reflections take their colour from
		views[SSR_HISTORY_TEX] = (colour && (false == scene.targets.empty()) && scene.targets.front().ssr_history ? scene.targets.front().ssr_history->view() : scene.white->view());
	}
	return result;
}

// a pool for Count pass sets, and the sets
VkDescriptorPool make_pass_pool(VkDevice const Device, VkDescriptorSetLayout const Layout, std::uint32_t const Count, VkDescriptorSet *Sets)
{
	using namespace vk14::bindings;
	std::array<std::uint32_t, 4> perset{};
	for (std::uint32_t binding = 0; binding < PASS_BINDING_COUNT; ++binding)
	{
		switch (pass_binding_type(binding))
		{
		case VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC: ++perset[0]; break;
		case VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC: ++perset[1]; break;
		case VK_DESCRIPTOR_TYPE_STORAGE_BUFFER: ++perset[2]; break;
		case VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER: ++perset[3]; break;
		default: break;
		}
	}
	std::array<VkDescriptorPoolSize, 5> sizes{{{VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, perset[0] * Count},
	                                           {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, perset[1] * Count},
	                                           {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, perset[2] * Count},
	                                           {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, perset[3] * Count},
	                                           {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, (PASS_BINDING_COUNT - perset[0] - perset[1] - perset[2] - perset[3]) * Count}}};
	VkDescriptorPoolCreateInfo poolinfo{};
	poolinfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
	poolinfo.maxSets = Count;
	poolinfo.poolSizeCount = static_cast<std::uint32_t>(sizes.size());
	poolinfo.pPoolSizes = sizes.data();
	VkDescriptorPool pool{VK_NULL_HANDLE};
	if (!VK14_VERIFY(vkCreateDescriptorPool(Device, &poolinfo, nullptr, &pool)))
	{
		return VK_NULL_HANDLE;
	}
	std::vector<VkDescriptorSetLayout> layouts(Count, Layout);
	VkDescriptorSetAllocateInfo allocation{};
	allocation.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	allocation.descriptorPool = pool;
	allocation.descriptorSetCount = Count;
	allocation.pSetLayouts = layouts.data();
	if (!VK14_VERIFY(vkAllocateDescriptorSets(Device, &allocation, Sets)))
	{
		vkDestroyDescriptorPool(Device, pool, nullptr);
		return VK_NULL_HANDLE;
	}
	return pool;
}

// a view of one layer (and the first level) of an image
VkImageView layer_view(VkDevice Device, vk14::image const &Image, std::uint32_t const Layer, std::string const &Name)
{
	VkImageViewCreateInfo info{};
	info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
	info.image = Image.handle();
	info.viewType = VK_IMAGE_VIEW_TYPE_2D;
	info.format = Image.desc().format;
	info.subresourceRange = {Image.aspect(), 0, 1, Layer, 1};
	VkImageView view{VK_NULL_HANDLE};
	if (!VK14_VERIFY(vkCreateImageView(Device, &info, nullptr, &view)))
	{
		return VK_NULL_HANDLE;
	}
	vk14::name_object(Device, VK_OBJECT_TYPE_IMAGE_VIEW, vk14::handle_value(view), Name);
	return view;
}

} // namespace

// the resources of the scene

bool vulkan14_renderer::init_scene(std::string &Reason)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto const device{ctx.device.handle()};
	auto const frames{static_cast<std::uint32_t>(ctx.frames.size())};
	using namespace vk14::bindings;

	// the pass set: the blocks of the frame with dynamic offsets, the clusters, the maps, the light of the sky, the weather, the
	// shadows of the lamps and the depth of the scene. the compute kernels read it too
	{
		auto const stages{VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT};
		std::vector<VkDescriptorSetLayoutBinding> bindings;
		for (std::uint32_t binding = 0; binding < PASS_BINDING_COUNT; ++binding)
		{
			bindings.push_back({binding, pass_binding_type(binding), 1, stages, nullptr});
		}
		VkDescriptorSetLayoutCreateInfo info{};
		info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
		info.bindingCount = static_cast<std::uint32_t>(bindings.size());
		info.pBindings = bindings.data();
		if (!VK14_VERIFY(vkCreateDescriptorSetLayout(device, &info, nullptr, &scene.pass_layout)))
		{
			Reason = "no layout of the pass set";
			return false;
		}
		vk14::name_object(device, VK_OBJECT_TYPE_DESCRIPTOR_SET_LAYOUT, vk14::handle_value(scene.pass_layout), "pass set");
	}
	// the textures of the materials, pushed with the draws
	{
		std::vector<VkDescriptorSetLayoutBinding> bindings;
		for (std::uint32_t unit = 0; unit < gl::MAX_TEXTURES; ++unit)
		{
			bindings.push_back({MATERIAL_TEXTURE_FIRST + unit, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, nullptr});
		}
		if (false == scene.material_set.create(device, bindings, MATERIAL_SET, VK_PIPELINE_BIND_POINT_GRAPHICS, sizeof(VkDescriptorImageInfo), "material textures"))
		{
			Reason = "no layout of the textures of the materials";
			return false;
		}
	}
	VkPushConstantRange const graphicsconstants{VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, PUSH_CONSTANTS_SIZE};
	// the ray traced reflections: the scenery around the camera, set RT_SET of the scene, which the shaders of the scene read when they
	// were made with REFLECTION_RAYS (vk14::binding_preamble())
	std::vector<VkDescriptorSetLayout> scenesets{scene.pass_layout, scene.material_set.layout};
	if (ctx.device.raytracing_enabled())
	{
		scene.raytracing = std::make_unique<vk14::ray_tracing>();
		static_assert(RT_SET == 2, "the set of the ray tracing follows the pass set and the material set");
		if (false == scene.raytracing->init(ctx.device, ctx.memory, ctx.deletions, frames, Reason))
		{
			return false;
		}
		scenesets.push_back(scene.raytracing->layout());
	}
	scene.scene_layout = vk14::make_pipeline_layout(device, scenesets, {graphicsconstants}, "scene");
	if ((scene.scene_layout == VK_NULL_HANDLE) || (false == scene.material_set.create_template(device, scene.scene_layout, "material textures")))
	{
		Reason = "no layout of the pipelines of the scene";
		return false;
	}
	// the post-processing: the images it reads and the exposure, pushed
	{
		std::vector<VkDescriptorSetLayoutBinding> bindings;
		for (std::uint32_t image = 0; image < POST_TEXTURES; ++image)
		{
			bindings.push_back({POST_TEXTURE_FIRST + image, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr});
		}
		static_assert(POST_EXPOSURE_SSBO == POST_TEXTURE_FIRST + POST_TEXTURES, "the exposure follows the images of the template");
		bindings.push_back({POST_EXPOSURE_SSBO, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr});
		if (false == scene.post_set.create(device, bindings, POST_SET, VK_PIPELINE_BIND_POINT_GRAPHICS, sizeof(VkDescriptorImageInfo), "post-processing images"))
		{
			Reason = "no layout of the post-processing";
			return false;
		}
		scene.post_layout = vk14::make_pipeline_layout(device, {scene.post_set.layout}, {graphicsconstants}, "post-processing");
		if ((scene.post_layout == VK_NULL_HANDLE) || (false == scene.post_set.create_template(device, scene.post_layout, "post-processing images")))
		{
			Reason = "no layout of the pipelines of the post-processing";
			return false;
		}
	}
	// the compute passes of the clusters: their buffers, pushed
	{
		auto const stage{VK_SHADER_STAGE_COMPUTE_BIT};
		std::vector<VkDescriptorSetLayoutBinding> bindings{{CL_CLUSTER_UBO, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, stage, nullptr},  {CL_AABB_SSBO, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, stage, nullptr},
		                                                   {CL_RANGE_SSBO, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, stage, nullptr},   {CL_INDEX_SSBO, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, stage, nullptr},
		                                                   {CL_COUNTER_SSBO, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, stage, nullptr}, {CL_LIGHT_SSBO, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, stage, nullptr}};
		static_assert(sizeof(VkDescriptorBufferInfo) == sizeof(VkDescriptorImageInfo), "the templates are laid out by one stride");
		if (false == scene.cluster_set.create(device, bindings, 0, VK_PIPELINE_BIND_POINT_COMPUTE, sizeof(VkDescriptorBufferInfo), "cluster buffers"))
		{
			Reason = "no layout of the light clusters";
			return false;
		}
		VkPushConstantRange const computeconstants{VK_SHADER_STAGE_COMPUTE_BIT, 0, PUSH_CONSTANTS_SIZE};
		scene.cluster_layout = vk14::make_pipeline_layout(device, {scene.cluster_set.layout}, {computeconstants}, "light clusters");
		if ((scene.cluster_layout == VK_NULL_HANDLE) || (false == scene.cluster_set.create_template(device, scene.cluster_layout, "cluster buffers")))
		{
			Reason = "no layout of the compute passes of the light clusters";
			return false;
		}
	}

	// the rings of the draws
	auto const storagealignment{ctx.device.caps().limits.minStorageBufferOffsetAlignment};
	scene.draws.window = draw_window;
	scene.draws.stride = sizeof(gl::model_ubs);
	scene.instances.window = instance_window;
	scene.instances.stride = sizeof(glm::mat4);
	if ((false == scene.draws.ring.create(ctx.memory, draw_ring_size, frames, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, "draw blocks", draw_window)) ||
	    (false == scene.instances.ring.create(ctx.memory, instance_ring_size, frames, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, "instance matrices", instance_window)))
	{
		Reason = "no rings for the draws";
		return false;
	}
	(void)storagealignment;

	// the stand-in images and the maps
	auto make_image = [&](std::shared_ptr<vk14::image> &Image, vk14::image_desc const &Desc, char const *Name) {
		Image = std::make_shared<vk14::image>();
		if (false == Image->create(ctx.memory, Desc, Name))
		{
			Image.reset();
			Reason = std::string("no image for ") + Name;
			return false;
		}
		return true;
	};
	{
		vk14::image_desc desc;
		desc.format = VK_FORMAT_R8G8B8A8_UNORM;
		desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
		if (false == make_image(scene.white, desc, "white"))
		{
			return false;
		}
		std::uint32_t const white{0xffffffff};
		ctx.uploads.upload(scene.white, &white, sizeof(white), {{0, {1, 1, 1}}}, false);
		if (scene.raytracing)
		{
			scene.raytracing->set_white(scene.white->view(), ctx.sampling.repeat_aniso);
		}
	}
	{
		vk14::image_desc desc;
		desc.format = VK_FORMAT_D32_SFLOAT;
		desc.view_type = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
		desc.layers = gl::MAX_CASCADES;
		desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
		if (false == make_image(scene.shadow_empty, desc, "empty shadow map"))
		{
			return false;
		}
		if (Global.gfx_shadowmap_enabled)
		{
			scene.shadow_size = static_cast<std::uint32_t>(std::clamp(Global.shadowtune.map_size, 64u, ctx.device.caps().limits.maxImageDimension2D));
			desc.extent = {scene.shadow_size, scene.shadow_size, 1};
			desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
			if (false == make_image(scene.shadow_map, desc, "shadow map"))
			{
				return false;
			}
			for (std::uint32_t layer = 0; layer < gl::MAX_CASCADES; ++layer)
			{
				scene.shadow_layers.push_back(layer_view(device, *scene.shadow_map, layer, "shadow cascade " + std::to_string(layer)));
			}
			vk14::log_info("shadow map " + std::to_string(scene.shadow_size) + "p");
		}
	}
	{
		vk14::image_desc desc;
		desc.format = VK_FORMAT_R16G16B16A16_SFLOAT;
		desc.view_type = VK_IMAGE_VIEW_TYPE_CUBE;
		desc.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
		desc.layers = 6;
		desc.extent = {16, 16, 1};
		desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
		if (false == make_image(scene.env_empty, desc, "empty environment map"))
		{
			return false;
		}
		if (Global.gfx_envmap_enabled)
		{
			auto const size{static_cast<std::uint32_t>(gl::ENVMAP_SIZE)};
			desc.extent = {size, size, 1};
			desc.mips = static_cast<std::uint32_t>(std::floor(std::log2(static_cast<float>(size)))) + 1;
			desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
			if (false == make_image(scene.env_map, desc, "environment map"))
			{
				return false;
			}
			for (std::uint32_t face = 0; face < 6; ++face)
			{
				scene.env_faces[face] = layer_view(device, *scene.env_map, face, "environment face " + std::to_string(face));
			}
			vk14::image_desc depthdesc;
			depthdesc.format = VK_FORMAT_D32_SFLOAT;
			depthdesc.extent = {size, size, 1};
			depthdesc.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
			if (false == make_image(scene.env_depth, depthdesc, "environment map depth"))
			{
				return false;
			}
		}
	}
	{
		// the picking of the controls and of the scenery
		vk14::image_desc desc;
		desc.format = VK_FORMAT_R8G8B8A8_UNORM;
		desc.extent = {1024, 1024, 1};
		desc.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
		if (false == make_image(scene.pick_colour, desc, "pick colour"))
		{
			return false;
		}
		desc.format = VK_FORMAT_D32_SFLOAT;
		desc.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
		if (false == make_image(scene.pick_depth, desc, "pick depth"))
		{
			return false;
		}
	}
	{
		// the stand-ins of the images of the stages P
		vk14::image_desc desc;
		desc.type = VK_IMAGE_TYPE_3D;
		desc.view_type = VK_IMAGE_VIEW_TYPE_3D;
		desc.format = VK_FORMAT_R8G8B8A8_UNORM;
		desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
		if (false == make_image(scene.volume_empty, desc, "empty volume"))
		{
			return false;
		}
		desc = vk14::image_desc{};
		desc.format = VK_FORMAT_D32_SFLOAT;
		desc.view_type = VK_IMAGE_VIEW_TYPE_CUBE_ARRAY;
		desc.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
		desc.layers = 6;
		desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
		if (false == make_image(scene.cube_shadow_empty, desc, "empty cube shadow maps"))
		{
			return false;
		}
		desc = vk14::image_desc{};
		desc.format = VK_FORMAT_D32_SFLOAT;
		desc.view_type = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
		desc.layers = static_cast<std::uint32_t>(gl::MAX_HEADLIGHT_SHADOWS);
		desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
		if (false == make_image(scene.headlight_shadow_empty, desc, "empty headlight shadow maps"))
		{
			return false;
		}
		desc = vk14::image_desc{};
		desc.format = VK_FORMAT_R32_SFLOAT;
		desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
		if (false == make_image(scene.depth_empty, desc, "empty scene depth"))
		{
			return false;
		}
		if (false == scene.weather_horizon.create(ctx.memory, sizeof(glm::vec4) * gl::FOG_HORIZON_SAMPLES, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		                                          vk14::memory_kind::device, "fog horizon"))
		{
			Reason = "no buffer of the colours of the fog";
			return false;
		}
	}
	{
		VkFormatProperties properties{};
		vkGetPhysicalDeviceFormatProperties(ctx.device.physical(), scene.bloom_format, &properties);
		auto const needed{VK_FORMAT_FEATURE_COLOR_ATTACHMENT_BIT | VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT | VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT};
		if ((properties.optimalTilingFeatures & needed) != needed)
		{
			scene.bloom_format = VK_FORMAT_R16G16B16A16_SFLOAT;
		}
		// the history of the screen space reflections is copied and filtered by blits
		vkGetPhysicalDeviceFormatProperties(ctx.device.physical(), scene.history_format, &properties);
		auto const blits{VK_FORMAT_FEATURE_BLIT_SRC_BIT | VK_FORMAT_FEATURE_BLIT_DST_BIT | VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT | VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT | VK_FORMAT_FEATURE_TRANSFER_DST_BIT};
		if ((properties.optimalTilingFeatures & blits) != blits)
		{
			scene.history_format = VK_FORMAT_R16G16B16A16_SFLOAT;
		}
	}
	// the formats of the passes which don't depend on the viewport
	scene.shadow_formats.depth = VK_FORMAT_D32_SFLOAT;
	scene.shadow_formats.id = ++scene.formats_count;
	scene.cube_formats.depth = VK_FORMAT_D32_SFLOAT;
	scene.cube_formats.view_mask = 0x3f;
	scene.cube_formats.id = ++scene.formats_count;
	scene.reflection_formats.colour[0] = VK_FORMAT_R16G16B16A16_SFLOAT;
	scene.reflection_formats.colours = 1;
	scene.reflection_formats.depth = VK_FORMAT_D32_SFLOAT;
	scene.reflection_formats.id = ++scene.formats_count;
	scene.pick_formats.colour[0] = VK_FORMAT_R8G8B8A8_UNORM;
	scene.pick_formats.colours = 1;
	scene.pick_formats.depth = VK_FORMAT_D32_SFLOAT;
	scene.pick_formats.id = ++scene.formats_count;

	// the light clusters of the main viewport
	{
		auto &clusters{scene.clusters};
		auto const usage{VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT};
		if ((false == clusters.aabbs.create(ctx.memory, sizeof(gl::cluster_aabb) * gl::CLUSTER_COUNT, usage, vk14::memory_kind::device, "cluster boxes")) ||
		    (false == clusters.ranges.create(ctx.memory, sizeof(gl::cluster_range) * gl::CLUSTER_COUNT, usage, vk14::memory_kind::device, "cluster ranges")) ||
		    (false == clusters.indices.create(ctx.memory, sizeof(std::uint32_t) * gl::CLUSTER_COUNT * gl::MAX_LIGHTS_PER_CLUSTER, usage, vk14::memory_kind::device, "cluster lights")) ||
		    (false == clusters.counters.create(ctx.memory, sizeof(gl::cluster_counters), usage | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, vk14::memory_kind::device, "cluster counters")) ||
		    (false == clusters.counters_readback.create(ctx.memory, sizeof(gl::cluster_counters), frames, "cluster counters read back")))
		{
			Reason = "no buffers of the light clusters";
			return false;
		}
	}

	// the occlusion queries of the lamps
	for (std::uint32_t frame = 0; frame < frames; ++frame)
	{
		VkQueryPoolCreateInfo info{};
		info.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
		info.queryType = VK_QUERY_TYPE_OCCLUSION;
		info.queryCount = vk14::scene_state::occlusion_queries;
		VkQueryPool pool{VK_NULL_HANDLE};
		if (!VK14_VERIFY(vkCreateQueryPool(device, &info, nullptr, &pool)))
		{
			Reason = "no occlusion queries";
			return false;
		}
		vkResetQueryPool(device, pool, 0, vk14::scene_state::occlusion_queries);
		scene.occlusion_pools.push_back(pool);
	}
	scene.occlusion_submodels.resize(frames);

	// the programs of the renderer
	try
	{
		auto program = [&](char const *Vertex, char const *Fragment, vk14::vertex_kind const Kind) {
			auto result{std::make_shared<vk14::scene_program>()};
			result->vertex = ctx.shaders.get(Vertex);
			result->fragment = ctx.shaders.get(Fragment);
			result->fragment_name = Fragment;
			result->vertex_name = Vertex;
			result->vertices = Kind;
			result->name = std::string(Vertex) + " + " + Fragment;
			return result;
		};
		scene.shadow = program("simpleuv.vert", "shadowmap.frag", vk14::vertex_kind::scene);
		scene.alpha_shadow = program("simpleuv.vert", "alphashadowmap.frag", vk14::vertex_kind::scene);
		scene.pick = program("vertexonly.vert", "pick.frag", vk14::vertex_kind::scene);
		scene.pick_surface = program("simpleuv.vert", "pick_surface.frag", vk14::vertex_kind::scene);
		scene.line = program("traction.vert", "traction.frag", vk14::vertex_kind::scene);
		scene.billboard = program("simpleuv.vert", "billboard.frag", vk14::vertex_kind::scene);
		scene.freespot = program("freespot.vert", "freespot.frag", vk14::vertex_kind::scene);
		scene.celestial = program("celestial.vert", "celestial.frag", vk14::vertex_kind::none);
		scene.skydome = program("vbocolor.vert", "color.frag", vk14::vertex_kind::colour);
		scene.smoke = program("smoke.vert", "smoke.frag", vk14::vertex_kind::particle);
		scene.precipitation = program("precipitation.vert", "precipitation.frag", vk14::vertex_kind::position_uv);
		scene.prepass = program("depthprepass.vert", "depthprepass.frag", vk14::vertex_kind::scene);
		scene.prepass_alpha = program("depthprepass.vert", "depthprepass_alpha.frag", vk14::vertex_kind::scene);
		scene.cube_shadow = program("shadowcube.vert", "shadowmap.frag", vk14::vertex_kind::scene);
		scene.cube_alpha_shadow = program("shadowcube.vert", "alphashadowmap.frag", vk14::vertex_kind::scene);
		scene.quad = ctx.shaders.get("quad.vert");
		for (auto const *name : {"copy.frag", "present.frag", "postfx_tonemapping.frag", "postfx_motionblur.frag", "postfx_chromaticaberration.frag"})
		{
			scene.post_fragments.emplace(name, ctx.shaders.get(name));
		}
		if (Global.gfx_taa())
		{
			scene.post_fragments.emplace("postfx_taa.frag", ctx.shaders.get("postfx_taa.frag"));
		}
		if (Global.gfx_dlss())
		{
			for (auto const *name : {"postfx_dlss_motion.frag", "postfx_dlss_exposure.frag", "postfx_fg_flip.frag"})
			{
				scene.post_fragments.emplace(name, ctx.shaders.get(name));
			}
		}
		auto const build{ctx.shaders.get("cluster_build.comp")};
		auto const cull{ctx.shaders.get("cluster_cull.comp")};
		scene.cluster_build = ctx.pipelines.compute(build, scene.cluster_layout, {}, "cluster build");
		scene.cluster_cull = ctx.pipelines.compute(cull, scene.cluster_layout, {}, "cluster cull");
	}
	catch (gfx::shader_exception const &e)
	{
		Reason = std::string("invalid shader: ") + e.what();
		return false;
	}

	// the light of the sky, the exposure, the weather
	if ((false == init_sky(Reason)) || (false == init_weather(Reason)) || (false == init_ao(Reason)))
	{
		return false;
	}

	scene.lamps.init(device, ctx.memory, ctx.deletions);
	// the layer maps of the heightmap terrain drawn into the layers of its arrays by the copy shader
	scene.terrain = std::make_unique<vk14::terrain_materials>(m_materials, *ctx.textures);
	scene.grass.init(ctx.resources);
	{
		vk14::gpu_particles::host host;
		host.compute = [this]() {
			suspend_rendering();
			return m_context->commands;
		};
		host.bind = [this](vk14::compute_kernel const &Kernel) { bind_compute_set(Kernel); };
		host.draw = [this](vk14::scene_program &Program, VkBuffer const Output, VkBuffer const Command, glm::vec4 const &Data0, glm::vec4 const &Data1) {
			// the rendering goes on where the compute passes ended it; the model block is the one the caller set up
			resume_rendering();
			auto &draw{*m_draw};
			if ((false == draw.rendering) || (draw.targets == nullptr) || (false == bind_scene_pipeline(Program, *draw.targets, vk14::blend_mode::scene_alpha, VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP)))
			{
				return;
			}
			auto const cullface{draw.cullface};
			draw.cullface = false;
			apply_draw_state();
			if (draw.set_dirty)
			{
				bind_pass_set();
			}
			auto const block{push_draw_block()};
			if (block != ~0u)
			{
				if (draw.textures_dirty)
				{
					push_textures();
				}
				vk14::draw_constants constants;
				constants.draw = block;
				constants.data[0] = Data0;
				constants.data[1] = Data1;
				push_constants(&constants);
				VkDeviceSize const offset{0};
				draw.commands->bind_vertex_buffers(0, 1, &Output, &offset);
				draw.commands->draw_indirect(Command, 0, 1, 4 * sizeof(std::uint32_t));
				draw.vertices = VK_NULL_HANDLE;
				draw.userdata = VK_NULL_HANDLE;
				++draw.draws;
			}
			draw.cullface = cullface;
			apply_draw_state();
		};
		host.slot = [this]() { return m_context->slot; };
		host.value = [this]() { return m_context->timelinevalue + 1; };
		host.completed = [this]() { return m_context->completed(); };
		scene.particles.init(device, ctx.pipelines, ctx.shaders, scene.pass_layout, ctx.memory, ctx.uploads, ctx.deletions, ctx.ring, ctx.device.caps().limits.minUniformBufferOffsetAlignment, frames,
		                     std::move(host));
	}
	scene.terrain->init(ctx.resources, [this](vk14::command_stream *const Commands, VkImageView const Target, VkFormat const Format, VkExtent2D const Extent, VkImageView const Source, VkSampler const Sampler) {
		auto &scene{m_context->scene};
		auto const pipeline{post_pipeline("copy.frag", static_cast<int>(Format))};
		if (pipeline == nullptr)
		{
			return;
		}
		VkRenderingAttachmentInfo attachment{};
		attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
		attachment.imageView = Target;
		attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		attachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
		attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
		VkRenderingInfo rendering{};
		rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
		rendering.renderArea = {{0, 0}, Extent};
		rendering.layerCount = 1;
		rendering.colorAttachmentCount = 1;
		rendering.pColorAttachments = &attachment;
		Commands->begin_rendering(&rendering);
		vk14::set_draw_state(Commands, Extent);
		Commands->bind_pipeline(VK_PIPELINE_BIND_POINT_GRAPHICS, reinterpret_cast<VkPipeline>(pipeline));
		Commands->set_primitive_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP);
		// the texture as its material samples it, as the copy of the opengl renderer draws it
		std::array<vk14::descriptor, vk14::bindings::POST_TEXTURES + 1> descriptors;
		for (std::size_t i = 0; i < vk14::bindings::POST_TEXTURES; ++i)
		{
			descriptors[i] = vk14::image_descriptor(Sampler, Source, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
		}
		descriptors[vk14::bindings::POST_EXPOSURE_SSBO] = vk14::buffer_descriptor(scene.exposure.data.handle(), 0, sizeof(gl::exposure_data));
		scene.post_set.push(Commands, scene.post_layout, descriptors.data());
		vk14::draw_constants const constants;
		VkPushConstantsInfo info{};
		info.sType = VK_STRUCTURE_TYPE_PUSH_CONSTANTS_INFO;
		info.layout = scene.post_layout;
		info.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
		info.size = sizeof(constants);
		info.pValues = &constants;
		Commands->push_constants2(&info);
		Commands->draw(4, 1, 0, 0);
		Commands->end_rendering();
	});

	m_draw = std::make_unique<draw_state>();
	m_draw->textures.fill(null_handle);
	return true;
}

void vulkan14_renderer::destroy_scene()
{
	if (!m_context)
	{
		return;
	}
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto const device{ctx.device.handle()};
	if (device == VK_NULL_HANDLE)
	{
		return;
	}
	destroy_cull();
	if (scene.raytracing)
	{
		scene.raytracing->destroy();
		scene.raytracing.reset();
	}
	scene.lamps.destroy();
	scene.terrain.reset();
	scene.grass.destroy();
	scene.particles.destroy();
	scene.grass_program.reset();
	scene.grass_tried = false;
	scene.cluster_heatmap.reset();
	scene.cluster_heatmap_tried = false;
	destroy_sky();
	destroy_weather();
	destroy_ao();
	for (auto *pool : scene.occlusion_pools)
	{
		vkDestroyQueryPool(device, pool, nullptr);
	}
	scene.occlusion_pools.clear();
	for (auto *view : scene.shadow_layers)
	{
		if (view != VK_NULL_HANDLE)
		{
			vkDestroyImageView(device, view, nullptr);
		}
	}
	scene.shadow_layers.clear();
	for (auto &view : scene.env_faces)
	{
		if (view != VK_NULL_HANDLE)
		{
			vkDestroyImageView(device, view, nullptr);
			view = VK_NULL_HANDLE;
		}
	}
	if (scene.pass_pool != VK_NULL_HANDLE)
	{
		vkDestroyDescriptorPool(device, scene.pass_pool, nullptr);
		scene.pass_pool = VK_NULL_HANDLE;
	}
	scene.material_set.destroy(device);
	scene.post_set.destroy(device);
	scene.cluster_set.destroy(device);
	for (auto *layout : {&scene.scene_layout, &scene.post_layout, &scene.cluster_layout})
	{
		if (*layout != VK_NULL_HANDLE)
		{
			vkDestroyPipelineLayout(device, *layout, nullptr);
			*layout = VK_NULL_HANDLE;
		}
	}
	if (scene.pass_layout != VK_NULL_HANDLE)
	{
		vkDestroyDescriptorSetLayout(device, scene.pass_layout, nullptr);
		scene.pass_layout = VK_NULL_HANDLE;
	}
	// the pipelines belong to the pipeline library
	scene.materials.clear();
	scene.targets.clear();
	scene.generation = vk14::generation_targets{};
	scene.white.reset();
	scene.shadow_empty.reset();
	scene.shadow_map.reset();
	scene.env_empty.reset();
	scene.env_map.reset();
	scene.env_depth.reset();
	scene.pick_colour.reset();
	scene.pick_depth.reset();
	scene.volume_empty.reset();
	scene.cube_shadow_empty.reset();
	scene.headlight_shadow_empty.reset();
	scene.depth_empty.reset();
	scene.weather_horizon.destroy();
	scene.pick_controls = {};
	scene.pick_scenery = {};
	scene.cursor = {};
	scene.screenshot.reset();
	scene.clusters.aabbs.destroy();
	scene.clusters.ranges.destroy();
	scene.clusters.indices.destroy();
	scene.clusters.counters.destroy();
	scene.clusters.counters_readback.destroy();
	scene.draws.ring.destroy();
	scene.instances.ring.destroy();
}

// the pass sets, made anew when the images they read changed: the old ones go once the gpu is done with them
void vulkan14_renderer::update_pass_sets()
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto const device{ctx.device.handle()};
	using namespace vk14::bindings;

	// the beams of the headlights, once their texture is there
	auto headlights{scene.white->view()};
	auto headlightssampler{ctx.sampling.clamp_aniso};
	if (m_headlightstexture != null_handle)
	{
		auto &texture{ctx.textures->mark_as_used(m_headlightstexture)};
		texture.create();
		if (auto const *entry{ctx.images.find(texture.get_id())})
		{
			headlights = entry->target->view();
			headlightssampler = entry->sampler;
		}
	}
	if (headlights != scene.headlights_view)
	{
		scene.pass_sets_stale = true;
	}
	auto const pass_images{pass_set_images(scene)};
	if (pass_images != scene.pass_images)
	{
		scene.pass_sets_stale = true;
	}
	if (false == scene.pass_sets_stale)
	{
		return;
	}
	scene.pass_sets_stale = false;
	scene.headlights_view = headlights;
	scene.pass_images = pass_images;
	if (scene.pass_pool != VK_NULL_HANDLE)
	{
		ctx.deletions.push(ctx.uploads.current_value(), [device, pool = scene.pass_pool]() { vkDestroyDescriptorPool(device, pool, nullptr); });
		scene.pass_pool = VK_NULL_HANDLE;
	}
	scene.headlights_sampler = headlightssampler;
	scene.pass_pool = make_pass_pool(device, scene.pass_layout, static_cast<std::uint32_t>(vk14::scene_state::set_count), scene.pass_sets.data());
	if (scene.pass_pool == VK_NULL_HANDLE)
	{
		return;
	}
	write_pass_sets(nullptr);
	++scene.pass_generation;
}

// the bindings of the pass sets, or of those of a culling: the same but for the instance buffer, its output
void vulkan14_renderer::write_pass_sets(vk14::instance_cull *Cull)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto const device{ctx.device.handle()};
	auto const count{static_cast<std::uint32_t>(vk14::scene_state::set_count)};
	using namespace vk14::bindings;
	auto const &pass_images{scene.pass_images};
	auto const *sets{Cull != nullptr ? Cull->sets.data() : scene.pass_sets.data()};
	VkDescriptorBufferInfo const sceneblock{ctx.ring.handle(), 0, sizeof(gl::scene_ubs)};
	VkDescriptorBufferInfo const draws{scene.draws.ring.handle(), 0, scene.draws.window};
	VkDescriptorBufferInfo const instances{Cull != nullptr ? VkDescriptorBufferInfo{Cull->output_buffer(), 0, Cull->output_size()} : VkDescriptorBufferInfo{scene.instances.ring.handle(), 0, scene.instances.window}};
	VkDescriptorBufferInfo const lights{ctx.ring.handle(), 0, sizeof(gl::light_data)};
	VkDescriptorBufferInfo const clusterblock{ctx.ring.handle(), 0, sizeof(gl::cluster_ubs)};
	VkDescriptorBufferInfo const ranges{scene.clusters.ranges.handle(), 0, VK_WHOLE_SIZE};
	VkDescriptorBufferInfo const indices{scene.clusters.indices.handle(), 0, VK_WHOLE_SIZE};
	VkDescriptorBufferInfo const skyblock{scene.sky.sky.handle(), 0, sizeof(gl::sky_data)};
	VkDescriptorBufferInfo const atmosphereblock{scene.sky.atmosphere.handle(), 0, sizeof(gl::atmosphere_ubs)};
	VkDescriptorBufferInfo const weatherblock{ctx.ring.handle(), 0, offsetof(gl::weather_ubs, fog_horizon)};
	VkDescriptorBufferInfo const horizonblock{scene.weather_horizon.handle(), 0, sizeof(glm::vec4) * gl::FOG_HORIZON_SAMPLES};
	VkDescriptorImageInfo const headlightsinfo{scene.headlights_sampler, scene.headlights_view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
	std::vector<VkWriteDescriptorSet> writes;
	std::vector<std::unique_ptr<VkDescriptorImageInfo>> images;
	auto write = [&](VkDescriptorSet Set, std::uint32_t Binding, VkDescriptorBufferInfo const *Buffer, VkDescriptorImageInfo const *Image) {
		VkWriteDescriptorSet entry{};
		entry.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		entry.dstSet = Set;
		entry.dstBinding = Binding;
		entry.descriptorCount = 1;
		entry.descriptorType = pass_binding_type(Binding);
		entry.pBufferInfo = Buffer;
		entry.pImageInfo = Image;
		writes.push_back(entry);
	};
	auto image = [&](VkSampler Sampler, VkImageView View) {
		images.push_back(std::make_unique<VkDescriptorImageInfo>(VkDescriptorImageInfo{Sampler, View, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}));
		return images.back().get();
	};
	for (std::uint32_t set = 0; set < count; ++set)
	{
		auto const handle{sets[set]};
		auto const &views{pass_images[set]};
		write(handle, SCENE_UBO, &sceneblock, nullptr);
		write(handle, DRAW_SSBO, &draws, nullptr);
		write(handle, INSTANCE_SSBO, &instances, nullptr);
		write(handle, LIGHT_SSBO, &lights, nullptr);
		write(handle, CLUSTER_UBO, &clusterblock, nullptr);
		write(handle, CLUSTER_RANGE_SSBO, &ranges, nullptr);
		write(handle, CLUSTER_INDEX_SSBO, &indices, nullptr);
		write(handle, SKY_SSBO, &skyblock, nullptr);
		write(handle, ATMOSPHERE_UBO, &atmosphereblock, nullptr);
		write(handle, WEATHER_UBO, &weatherblock, nullptr);
		write(handle, WEATHER_HORIZON_UBO, &horizonblock, nullptr);
		write(handle, HEADLIGHT_TEX, nullptr, &headlightsinfo);
		write(handle, SHADOW_TEX, nullptr, image(ctx.sampling.shadow, views[SHADOW_TEX]));
		write(handle, ENV_TEX, nullptr, image(ctx.sampling.clamp_linear, views[ENV_TEX]));
		write(handle, SKY_VIEW_TEX, nullptr, image(ctx.sampling.clamp_linear, views[SKY_VIEW_TEX]));
		write(handle, ENV_GGX_TEX, nullptr, image(ctx.sampling.clamp_linear, views[ENV_GGX_TEX]));
		write(handle, FOG_VOLUME_TEX, nullptr, image(ctx.sampling.clamp_linear, views[FOG_VOLUME_TEX]));
		write(handle, CLOUD_SHADOW_TEX, nullptr, image(ctx.sampling.clamp_linear, views[CLOUD_SHADOW_TEX]));
		write(handle, CLOUD_SHAPE_TEX, nullptr, image(ctx.sampling.repeat_linear, views[CLOUD_SHAPE_TEX]));
		write(handle, CLOUD_DETAIL_TEX, nullptr, image(ctx.sampling.repeat_linear, views[CLOUD_DETAIL_TEX]));
		write(handle, CLOUD_COVERAGE_TEX, nullptr, image(ctx.sampling.repeat_linear, views[CLOUD_COVERAGE_TEX]));
		write(handle, LOCAL_SHADOW_TEX, nullptr, image(ctx.sampling.shadow, views[LOCAL_SHADOW_TEX]));
		write(handle, LOCAL_SHADOW_LAMPS_TEX, nullptr, image(ctx.sampling.shadow, views[LOCAL_SHADOW_LAMPS_TEX]));
		write(handle, HEADLIGHT_SHADOW_TEX, nullptr, image(ctx.sampling.shadow, views[HEADLIGHT_SHADOW_TEX]));
		write(handle, SCENE_DEPTH_TEX, nullptr, image(ctx.sampling.clamp_nearest, views[SCENE_DEPTH_TEX]));
		write(handle, SSR_HISTORY_TEX, nullptr, image(ctx.sampling.clamp_linear, views[SSR_HISTORY_TEX]));
	}
	vkUpdateDescriptorSets(device, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void vulkan14_renderer::use_cull_pass_sets(vk14::instance_cull *Cull)
{
	auto &draw{*m_draw};
	if (Cull == nullptr)
	{
		if (draw.cull_sets != nullptr)
		{
			draw.cull_sets = nullptr;
			draw.set_dirty = true;
		}
		return;
	}
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	update_pass_sets();
	if ((Cull->set_pool == VK_NULL_HANDLE) || (Cull->sets_generation != scene.pass_generation) || (Cull->sets_output != Cull->output_buffer()))
	{
		auto const device{ctx.device.handle()};
		if (Cull->set_pool != VK_NULL_HANDLE)
		{
			ctx.deletions.push(ctx.uploads.current_value(), [device, pool = Cull->set_pool]() { vkDestroyDescriptorPool(device, pool, nullptr); });
		}
		Cull->set_pool = make_pass_pool(device, scene.pass_layout, static_cast<std::uint32_t>(vk14::scene_state::set_count), Cull->sets.data());
		if (Cull->set_pool == VK_NULL_HANDLE)
		{
			draw.cull_sets = nullptr;
			return;
		}
		write_pass_sets(Cull);
		Cull->sets_generation = scene.pass_generation;
		Cull->sets_output = Cull->output_buffer();
	}
	draw.cull_sets = Cull->sets.data();
	draw.set_dirty = true;
}

void vulkan14_renderer::bind_pass_set()
{
	auto &draw{*m_draw};
	update_pass_sets();
	if (false == draw.rendering)
	{
		return;
	}
	auto &scene{m_context->scene};
	// the dynamic offsets in the order of the bindings
	// the instances of the indirect draws come from the output of the culling, in the pass sets of the culling
	auto const instancebase{draw.cull_sets != nullptr ? 0u : static_cast<std::uint32_t>(scene.instances.base)};
	std::array<std::uint32_t, vk14::bindings::PASS_DYNAMIC_OFFSETS> const offsets{draw.scene_offset, static_cast<std::uint32_t>(scene.draws.base), instancebase, draw.light_offset, draw.cluster_offset, draw.weather_offset};
	VkBindDescriptorSetsInfo info{};
	info.sType = VK_STRUCTURE_TYPE_BIND_DESCRIPTOR_SETS_INFO;
	info.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
	info.layout = scene.scene_layout;
	info.firstSet = vk14::bindings::PASS_SET;
	info.descriptorSetCount = 1;
	info.pDescriptorSets = (draw.cull_sets != nullptr ? &draw.cull_sets[draw.set] : &scene.pass_sets[draw.set]);
	info.dynamicOffsetCount = static_cast<std::uint32_t>(offsets.size());
	info.pDynamicOffsets = offsets.data();
	draw.commands->bind_descriptor_sets2(&info);
	draw.set_dirty = false;
	// the set of the ray tracing of the frame, which the shaders of the scene made with REFLECTION_RAYS read
	if (scene.raytracing)
	{
		VkDescriptorSet const rtset{scene.raytracing->set(m_context->slot)};
		VkBindDescriptorSetsInfo rt{};
		rt.sType = VK_STRUCTURE_TYPE_BIND_DESCRIPTOR_SETS_INFO;
		rt.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
		rt.layout = scene.scene_layout;
		rt.firstSet = vk14::bindings::RT_SET;
		rt.descriptorSetCount = 1;
		rt.pDescriptorSets = &rtset;
		draw.commands->bind_descriptor_sets2(&rt);
	}
}

// the pass set of the colour of the main view bound for the compute kernels (their layouts share set 0 and the push constants),
// with the offsets of the blocks the frame wrote last
void vulkan14_renderer::bind_compute_set(vk14::compute_kernel const &Kernel)
{
	auto &scene{m_context->scene};
	auto &draw{*m_draw};
	update_pass_sets();
	std::array<std::uint32_t, vk14::bindings::PASS_DYNAMIC_OFFSETS> const offsets{draw.scene_offset, static_cast<std::uint32_t>(scene.draws.base), static_cast<std::uint32_t>(scene.instances.base), draw.light_offset, draw.cluster_offset, draw.weather_offset};
	VkBindDescriptorSetsInfo info{};
	info.sType = VK_STRUCTURE_TYPE_BIND_DESCRIPTOR_SETS_INFO;
	info.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
	info.layout = Kernel.layout();
	info.firstSet = vk14::bindings::PASS_SET;
	info.descriptorSetCount = 1;
	info.pDescriptorSets = &scene.pass_sets[vk14::scene_state::set_colour];
	info.dynamicOffsetCount = static_cast<std::uint32_t>(offsets.size());
	info.pDynamicOffsets = offsets.data();
	m_context->commands->bind_descriptor_sets2(&info);
}

// geometry

gfx::geometrybank_handle vulkan14_renderer::Create_Bank()
{
	if (!m_context)
	{
		return m_geometry.register_bank(std::make_unique<null_geometrybank>());
	}
	auto bank{std::make_unique<vk14::vulkan14_geometrybank>(m_context->resources)};
	auto *raw{bank.get()};
	auto const handle{m_geometry.register_bank(std::move(bank))};
	if (m_banks.size() < handle.bank)
	{
		m_banks.resize(handle.bank, nullptr);
	}
	m_banks[handle.bank - 1] = raw;
	return handle;
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

bool vulkan14_renderer::Replace_Indexed(gfx::index_array &Indices, gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometry_handle const &Geometry)
{
	// NOTE: we expect indexed geometry to come with calculated tangents
	return m_geometry.replace(Indices, Vertices, Userdata, Geometry);
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

int vulkan14_renderer::Primitive(gfx::geometry_handle const &Geometry) const
{
	return static_cast<int>(m_geometry.type(Geometry));
}

// materials and their programs: the materials are read as for the opengl renderer, their programs are made of vertex.vert and
// mat_<name>.frag of shaders/vk14

material_handle vulkan14_renderer::Fetch_Material(std::string const &Filename, bool const Loadnow)
{
	return m_materials.create(Filename, Loadnow);
}

material_handle vulkan14_renderer::Matte_Material(material_handle const Material)
{
	return m_materials.create_matte(Material);
}

material_handle vulkan14_renderer::Tinted_Material(material_handle const Material, glm::vec3 const &Colour)
{
	return m_materials.create_tinted(Material, Colour);
}

material_handle vulkan14_renderer::Blend_Material(material_handle const Material, std::array<material_handle, 3> const &Others)
{
	return m_materials.create_blend(Material, Others);
}

bool vulkan14_renderer::Blendable_Material(material_handle const Material)
{
	return m_materials.blendable(Material);
}

IMaterial const *vulkan14_renderer::Material(material_handle const Material) const
{
	return &m_materials.material(Material);
}

material_handle vulkan14_renderer::Terrain_Material(material_handle const Reuse, std::vector<gfx::terrain_layer> const &Layers, int const Samples, std::uint8_t const *Weights, glm::vec3 const &Placement,
                                                    int const HoleSamples, std::uint8_t const *Holes)
{
	if (m_context && m_context->scene.terrain)
	{
		auto const material{m_context->scene.terrain->make(Reuse, Layers, Samples, Weights, Placement, HoleSamples, Holes)};
		if (material != null_handle)
		{
			return material;
		}
	}
	// the terrain shader can't be used, the chunk makes do with a plain material
	return gfx_renderer::Terrain_Material(Reuse, Layers, Samples, Weights, Placement, HoleSamples, Holes);
}

void vulkan14_renderer::Terrain_Release(material_handle const Material)
{
	if (m_context && m_context->scene.terrain)
	{
		m_context->scene.terrain->release(Material);
	}
}

void vulkan14_renderer::Terrain_Grass(std::vector<gfx::grass_kind> const &Kinds, std::vector<gfx::grass_chunk> const &Chunks)
{
	if (m_context)
	{
		m_context->scene.grass.update(Kinds, Chunks);
	}
}

std::shared_ptr<gfx::shader_program> vulkan14_renderer::Fetch_Shader(std::string const &name)
{
	if (!m_context)
	{
		return nullptr;
	}
	auto &materials{m_context->scene.materials};
	auto const lookup{materials.find(name)};
	if (lookup != materials.end())
	{
		return lookup->second;
	}
	// throws gfx::shader_exception, which the materials catch
	auto program{std::make_shared<vk14::scene_program>()};
	program->vertex = m_context->shaders.get("vertex.vert");
	program->fragment = m_context->shaders.get("mat_" + name + ".frag");
	program->fragment_name = "mat_" + name + ".frag";
	program->vertex_name = "vertex.vert";
	program->vertices = vk14::vertex_kind::scene;
	program->name = "mat_" + name;
	// the directives of the material, of both stages as the opengl program has them
	for (auto const *stage : {program->vertex.get(), program->fragment.get()})
	{
		program->texture_conf.insert(stage->source.texture_conf.begin(), stage->source.texture_conf.end());
		program->param_conf.insert(stage->source.param_conf.begin(), stage->source.param_conf.end());
	}
	materials.emplace(name, program);
	return program;
}

std::string vulkan14_renderer::Shader_Name(std::shared_ptr<gfx::shader_program> const &Program) const
{
	if (m_context)
	{
		for (auto const &program : m_context->scene.materials)
		{
			if (program.second == Program)
			{
				return program.first;
			}
		}
	}
	return {};
}

// the params of the material for the shaders of the scene in model_ubs, as opengl46_renderer::Bind_Material() sets them
void vulkan14_renderer::Bind_Material(material_handle const Material, TSubModel const *sm, lighting_data const *lighting)
{
	walk_material(Material, sm, lighting);
}

void vulkan14_renderer::walk_material(material_handle const Material, TSubModel const *Submodel, lighting_data const *Lighting)
{
	if (!m_draw)
	{
		return;
	}
	if (Material == null_handle)
	{
		if (Material != m_invalid_material)
		{
			walk_material(m_invalid_material);
		}
		return;
	}
	auto const &material{m_materials.material(Material)};
	if (false == material.is_good)
	{
		// use failure indicator instead
		if (Material != m_invalid_material)
		{
			walk_material(m_invalid_material);
		}
		return;
	}

	std::memcpy(&model_ubs.param[0], &material.params[0], sizeof(model_ubs.param));
	for (auto const &entry : material.params_state)
	{
		glm::vec4 src(1.0f);
		// submodel-based parameters
		if (Submodel)
		{
			switch (entry.defaultparam)
			{
			case gfx::shader_param_default::ambient:
				src = Submodel->f4Ambient;
				break;
			case gfx::shader_param_default::diffuse:
				src = Submodel->f4Diffuse;
				break;
			case gfx::shader_param_default::specular:
				src = Submodel->f4Specular;
				break;
			default:
				break;
			}
		}
		if (Lighting)
		{
			switch (entry.defaultparam)
			{
			case gfx::shader_param_default::ambient:
				src = Lighting->ambient;
				break;
			case gfx::shader_param_default::diffuse:
				src = Lighting->diffuse;
				break;
			case gfx::shader_param_default::specular:
				src = Lighting->specular;
				break;
			default:
				break;
			}
		}
		// material-based parameters
		switch (entry.defaultparam)
		{
		case gfx::shader_param_default::glossiness:
			src = glm::vec4{material.glossiness};
			break;
		default:
			break;
		}
		if (entry.size == 1)
		{
			// HACK: convert color to luminosity, if it's passed as single value
			src = glm::vec4{colors::RGBtoHSV(glm::vec3{src}).z};
		}
		for (size_t j = 0; j < entry.size; j++)
		{
			model_ubs.param[entry.location][entry.offset + j] = src[j];
		}
	}
	if (material.opacity)
	{
		model_ubs.opacity = (m_draw->blending ? -material.opacity.value() : material.opacity.value());
	}
	else
	{
		model_ubs.opacity = (m_draw->blending ? 0.0f : 0.5f);
	}
	model_ubs.alpha_mult = (Submodel ? Submodel->fVisible : 1.0f);
	if (m_indirectrecording)
	{
		// the walk of the groups of the gpu culling only notes what would be bound, it's bound when the draw is made
		m_indirectmaterial = Material;
		m_indirectsubmodel = Submodel;
		return;
	}

	// materials which declare texture variants provide a different texture set per model instance
	auto const &materialtextures{material.texture_variants.empty() ? material.textures : material.GetTextures(TSubModel::VariantId(Submodel))};
	auto &draw{*m_draw};
	if (draw.depthpass)
	{
		// the depth of the whole scene: the programs of the depth prepass. what they can't draw (blended, materials of other programs)
		// is left to the colour pass
		draw.depthskip = true;
		if (draw.blending || (model_ubs.opacity < 0.f))
		{
			return;
		}
		auto const prepassclass{prepass_class(Material)};
		if (prepassclass == 0)
		{
			return;
		}
		auto &scene{m_context->scene};
		if (prepassclass == 2)
		{
			// a texture which isn't there (yet) would leave the one bound before to the alpha test
			if (materialtextures[0] == null_handle)
			{
				return;
			}
			auto &texture{m_context->textures->mark_as_used(materialtextures[0])};
			if ((false == texture.create()) || (m_context->images.find(texture.get_id()) == nullptr))
			{
				return;
			}
			walk_texture(0, materialtextures[0]);
			draw.program = scene.prepass_alpha.get();
		}
		else
		{
			draw.program = scene.prepass.get();
		}
		draw.depthskip = false;
		return;
	}
	for (std::size_t unit = 0; unit < gl::MAX_TEXTURES; ++unit)
	{
		if (draw.textures[unit] != materialtextures[unit])
		{
			draw.textures[unit] = materialtextures[unit];
			draw.textures_dirty = true;
		}
	}
	draw.program = static_cast<vk14::scene_program *>(material.shader.get());
}

void vulkan14_renderer::walk_shadow_material(material_handle const Material)
{
	if (m_indirectrecording)
	{
		// the walk of the groups of the gpu culling of a shadow pass only notes the material
		m_indirectmaterial = Material;
		m_indirectsubmodel = nullptr;
		return;
	}
	auto &draw{*m_draw};
	auto &scene{m_context->scene};
	// the cube maps of the lamps with programs of their own (the faces drawn by multiview)
	if ((Material != null_handle) && (m_materials.material(Material).textures[0] != null_handle))
	{
		walk_texture(0, m_materials.material(Material).textures[0]);
		draw.program = (m_cubeshadows ? scene.cube_alpha_shadow.get() : scene.alpha_shadow.get());
	}
	else
	{
		draw.program = (m_cubeshadows ? scene.cube_shadow.get() : scene.shadow.get());
	}
}

opengl_material const &vulkan14_renderer::walk_material_data(material_handle const Material) const
{
	return m_materials.material(Material);
}

opengl_material const &vulkan14_renderer::walk_material_data(TSubModel const *Submodel) const
{
	return m_materials.material(submodel_material(Submodel));
}

void vulkan14_renderer::walk_program(walk_shader const Program)
{
	auto &scene{m_context->scene};
	switch (Program)
	{
	case walk_shader::pick:
		m_draw->program = scene.pick.get();
		break;
	case walk_shader::pick_surface:
		m_draw->program = scene.pick_surface.get();
		break;
	case walk_shader::line:
		m_draw->program = scene.line.get();
		break;
	case walk_shader::billboard:
		m_draw->program = scene.billboard.get();
		break;
	case walk_shader::freespot:
		m_draw->program = scene.freespot.get();
		break;
	}
}

void vulkan14_renderer::walk_texture(std::size_t const Unit, texture_handle const Texture)
{
	if ((Unit < gl::MAX_TEXTURES) && (m_draw->textures[Unit] != Texture))
	{
		m_draw->textures[Unit] = Texture;
		m_draw->textures_dirty = true;
	}
}

void vulkan14_renderer::walk_blending(bool const Alpha)
{
	// setup_drawing() of the opengl renderer: the blending and the depth written; the faces culled by the pass
	m_draw->blending = Alpha;
}

gfx::geometrybank_manager &vulkan14_renderer::walk_geometry()
{
	return m_geometry;
}

bool vulkan14_renderer::walk_blending_enabled() const
{
	return (m_draw != nullptr) && m_draw->blending;
}

void vulkan14_renderer::walk_cull_face(bool const Enabled)
{
	m_draw->cullface = Enabled;
}

void vulkan14_renderer::walk_glare(bool const Enabled)
{
	// without the depth test, added to what is there
	m_draw->glare = Enabled;
}

void vulkan14_renderer::walk_glare_draw(TSubModel *Submodel, gfx::geometry_handle const &Geometry)
{
	// the point of the light wasn't seen by the occlusion query of the last frame done by the gpu
	auto const &visible{m_context->scene.glare_visible};
	auto const lookup{visible.find(Submodel)};
	if ((lookup != visible.end()) && (false == lookup->second))
	{
		return;
	}
	walk_draw(Geometry);
}

void vulkan14_renderer::walk_light_query(TSubModel *Submodel, bool const Begin)
{
	auto &draw{*m_draw};
	if (false == draw.rendering)
	{
		return;
	}
	auto &scene{m_context->scene};
	auto &submodels{scene.occlusion_submodels[m_context->slot]};
	if (Begin)
	{
		if (submodels.size() >= vk14::scene_state::occlusion_queries)
		{
			draw.query = ~0u;
			return;
		}
		draw.query = static_cast<std::uint32_t>(submodels.size());
		submodels.push_back(Submodel);
		draw.commands->begin_query(scene.occlusion_pools[m_context->slot], draw.query, 0);
	}
	else if (draw.query != ~0u)
	{
		draw.commands->end_query(scene.occlusion_pools[m_context->slot], draw.query);
		draw.query = ~0u;
	}
}

// the light of the sun and the ambient light, for the draws which follow

void vulkan14_renderer::walk_sunlight(float const Factor)
{
	setup_sunlight_intensity(Factor);
}

void vulkan14_renderer::walk_environment(TEnvironmentType const Environment)
{
	switch (Environment)
	{
	case e_flat:
		setup_sunlight_intensity();
		break;
	case e_canyon:
		setup_sunlight_intensity(0.4f);
		break;
	case e_tunnel:
		setup_sunlight_intensity(0.2f);
		break;
	default:
		break;
	}
}

void vulkan14_renderer::walk_ambient_save()
{
	m_cabambient = light_data.ambient;
}

void vulkan14_renderer::walk_ambient_add(glm::vec3 const &Addition)
{
	light_data.ambient += packed_vec3(Addition);
	upload_light_header();
}

void vulkan14_renderer::walk_ambient_restore()
{
	light_data.ambient = m_cabambient;
}

glm::vec4 vulkan14_renderer::walk_sun_ambient() const
{
	return m_sunlight.ambient;
}

void vulkan14_renderer::setup_sunlight_intensity(float const Factor)
{
	m_sunlight.factor = Factor;
	light_data.lights[0].intensity = m_sunlight.factor * (1.0f - Global.Overcast * 0.33f);
	light_data.ambient = m_sunlight.ambient * m_sunlight.factor;
	light_data.sky_ambient_factor = (Global.gfx_ambient_sh ? m_sunlight.factor : 0.f);
	upload_light_header();
}

void vulkan14_renderer::setup_shadow_color(glm::vec4 const &Shadowcolor)
{
	model_ubs.shadow_tone = glm::pow(Shadowcolor.x, 2.2f);
}

// the draws

std::uint32_t vulkan14_renderer::push_draw_block()
{
	auto &scene{m_context->scene};
	auto &draws{scene.draws};
	auto const slice{draws.ring.allocate(draws.stride, draws.stride)};
	if (false == bool(slice))
	{
		return ~0u;
	}
	if ((false == draws.open) || (slice.offset + draws.stride > draws.base + draws.window))
	{
		// a new window of the blocks, the pass set is bound with its offset
		draws.base = slice.offset;
		draws.open = true;
		m_draw->set_dirty = true;
	}
	std::memcpy(slice.data, &model_ubs, sizeof(gl::model_ubs));
	return static_cast<std::uint32_t>((slice.offset - draws.base) / draws.stride);
}

std::uint32_t vulkan14_renderer::walk_instances(glm::mat4 const *Modelviews, std::size_t const Count)
{
	if (m_rtgather)
	{
		// noted for the ray tracing: the base is the index of the first of them among those noted
		auto const base{static_cast<std::uint32_t>(m_rtinstances.size())};
		m_rtinstances.insert(m_rtinstances.end(), Modelviews, Modelviews + Count);
		return base;
	}
	auto &scene{m_context->scene};
	auto &instances{scene.instances};
	auto const capacity{static_cast<std::size_t>(instances.window / instances.stride) - 1};
	auto const count{std::min(Count, capacity)};
	auto const size{static_cast<VkDeviceSize>(count) * instances.stride};
	if ((false == instances.open) || (instances.ring.next_offset(instances.stride) + size > instances.base + instances.window))
	{
		open_instance_window();
	}
	auto const slice{instances.ring.allocate(size, instances.stride)};
	if ((false == instances.open) || (false == bool(slice)) || (slice.offset + size > instances.base + instances.window))
	{
		// the ring of the frame is full: the instances are drawn where the draws which aren't instanced are
		return 0;
	}
	std::memcpy(slice.data, Modelviews, static_cast<std::size_t>(size));
	return static_cast<std::uint32_t>((slice.offset - instances.base) / instances.stride);
}

// a new window of the instance ring, which begins with the identity of the draws which aren't instanced
void vulkan14_renderer::open_instance_window()
{
	auto &instances{m_context->scene.instances};
	auto const alignment{m_context->device.caps().limits.minStorageBufferOffsetAlignment};
	auto const identity{instances.ring.allocate(instances.stride, std::max<VkDeviceSize>(alignment, instances.stride))};
	if (false == bool(identity))
	{
		instances.open = false;
		return;
	}
	glm::mat4 const one{1.f};
	std::memcpy(identity.data, &one, sizeof(one));
	instances.base = identity.offset;
	instances.open = true;
	if (m_draw)
	{
		m_draw->set_dirty = true;
	}
}

void vulkan14_renderer::walk_draw(gfx::geometry_handle const &Geometry)
{
	model_ubs.set_modelview(OpenGLMatrices.data(GL_MODELVIEW));
	if (m_rtgather)
	{
		gather_ray_draw(Geometry);
		return;
	}
	if (m_indirectrecording)
	{
		// the draw is made later, from the gpu culling, with this state
		Record_Indirect_Draw(Geometry);
		return;
	}
	draw_chunk(Geometry, true);
}

void vulkan14_renderer::walk_draw(std::vector<gfx::geometrybank_handle>::iterator First, std::vector<gfx::geometrybank_handle>::iterator Last)
{
	model_ubs.set_modelview(OpenGLMatrices.data(GL_MODELVIEW));
	if (m_rtgather)
	{
		for (; First != Last; ++First)
		{
			gather_ray_draw(*First);
		}
		return;
	}
	// the chunks drawn together share the block of the draw
	auto newblock{true};
	for (; First != Last; ++First)
	{
		draw_chunk(*First, newblock);
		newblock = false;
	}
}

void vulkan14_renderer::draw_chunk(gfx::geometry_handle const &Geometry, bool const Newblock)
{
	auto &draw{*m_draw};
	if ((false == draw.rendering) || (Geometry == null_handle) || (draw.program == nullptr) || (draw.program->vertices != vk14::vertex_kind::scene))
	{
		return;
	}
	if ((Geometry.bank == 0) || (Geometry.bank > m_banks.size()) || (m_banks[Geometry.bank - 1] == nullptr))
	{
		return;
	}
	// marks the bank as used, and has its data on the gpu
	gfx::indirect_source source;
	if (false == m_geometry.indirect_source(Geometry, source))
	{
		return;
	}
	auto const &chunk{static_cast<vk14::vulkan14_geometrybank *>(m_banks[Geometry.bank - 1])->last()};

	if (draw.depthpass && (draw.depthskip || (topology_class(chunk.topology) != VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)))
	{
		// the depth of the scene: what the programs of the prepass don't draw, and no points or lines
		return;
	}
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto const commands{draw.commands};
	// the pipeline for the state of the walk
	auto const blend{draw.glare ? vk14::blend_mode::scene_additive : (draw.blending ? vk14::blend_mode::scene_alpha : vk14::blend_mode::none)};
	if (false == bind_scene_pipeline(*draw.program, *draw.targets, blend, static_cast<int>(chunk.topology)))
	{
		return;
	}
	apply_draw_state();
	if (Newblock || (draw.block == ~0u))
	{
		draw.block = push_draw_block();
		if (draw.block == ~0u)
		{
			return;
		}
	}
	if (draw.set_dirty)
	{
		bind_pass_set();
	}
	if (draw.textures_dirty)
	{
		push_textures();
	}
	if (draw.pushed_draw != draw.block)
	{
		vk14::draw_constants constants;
		constants.draw = draw.block;
		VkPushConstantsInfo info{};
		info.sType = VK_STRUCTURE_TYPE_PUSH_CONSTANTS_INFO;
		info.layout = scene.scene_layout;
		info.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
		info.offset = 0;
		info.size = sizeof(constants);
		info.pValues = &constants;
		commands->push_constants2(&info);
		draw.pushed_draw = draw.block;
	}
	if ((chunk.vertices != draw.vertices) || (chunk.userdata != draw.userdata))
	{
		VkBuffer const buffers[2]{chunk.vertices, chunk.userdata};
		VkDeviceSize const offsets[2]{0, 0};
		commands->bind_vertex_buffers(0, 2, buffers, offsets);
		draw.vertices = chunk.vertices;
		draw.userdata = chunk.userdata;
	}
	auto const instances{m_current_instance_count > 0 ? static_cast<std::uint32_t>(m_current_instance_count) : 1u};
	auto const firstinstance{m_current_instance_count > 0 ? m_current_instance_base : 0u};
	if (chunk.indices != VK_NULL_HANDLE)
	{
		if (chunk.indices != draw.indices)
		{
			commands->bind_index_buffer(chunk.indices, 0, VK_INDEX_TYPE_UINT32);
			draw.indices = chunk.indices;
		}
		commands->draw_indexed(chunk.count, instances, chunk.first, chunk.basevertex, firstinstance);
	}
	else
	{
		commands->draw(chunk.count, instances, chunk.first, firstinstance);
	}
	++draw.draws;
}

// the pipeline of the program for the targets of the pass, the blending and the class of the primitive, made the first time it's
// asked for; bound with the primitive as dynamic state. false: it can't be made
bool vulkan14_renderer::bind_scene_pipeline(vk14::scene_program &Program, vk14::target_formats const &Targets, vk14::blend_mode const Blend, int const Topology)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &draw{*m_draw};
	auto const topology{static_cast<VkPrimitiveTopology>(Topology)};
	auto const primitiveclass{topology_class(topology)};
	// the settings of the game the shaders take as specialization constants
	auto const motion{Targets.colours > 1};
	std::uint32_t const settings{(Global.gfx_shadergamma ? 1u : 0u) | (Global.gfx_shadowmap_enabled ? 2u : 0u) | (Global.gfx_envmap_enabled ? 4u : 0u) | (motion ? 8u : 0u) | 16u | (Global.gfx_extraeffects ? 32u : 0u)};
	auto const key{(static_cast<std::uint64_t>(Targets.id)) | (static_cast<std::uint64_t>(Blend) << 16) | (static_cast<std::uint64_t>(primitiveclass) << 24) | (static_cast<std::uint64_t>(settings) << 32)};
	VkPipeline pipeline{VK_NULL_HANDLE};
	auto const lookup{Program.pipelines.find(key)};
	if (lookup != Program.pipelines.end())
	{
		pipeline = lookup->second;
	}
	else
	{
		auto fragment{Program.fragment};
		if ((false == motion) && (Targets.colours > 0))
		{
			// a shader writing the velocity without its target is reported by the validation layer
			if (!Program.fragment_single)
			{
				try
				{
					Program.fragment_single = ctx.shaders.get(Program.fragment_name, Program.defines + "#define NO_MOTION_OUTPUT\n");
				}
				catch (gfx::shader_exception const &Exception)
				{
					vk14::log_error(Exception.what());
				}
			}
			if (Program.fragment_single)
			{
				fragment = Program.fragment_single;
			}
		}
		vk14::pipeline_desc desc;
		desc.stages = {Program.vertex, fragment};
		desc.layout = scene.scene_layout;
		desc.vertices = layout_of(Program.vertices);
		{
			auto const read{vertex_input_locations(Program.vertex->spirv)};
			auto &attributes{desc.vertices.attributes};
			attributes.erase(std::remove_if(attributes.begin(), attributes.end(), [&](VkVertexInputAttributeDescription const &Attribute) { return std::find(read.begin(), read.end(), Attribute.location) == read.end(); }),
			                 attributes.end());
		}
		desc.topology = primitiveclass;
		desc.blending = Blend;
		desc.colour_formats.assign(Targets.colour.begin(), Targets.colour.begin() + Targets.colours);
		desc.depth_format = Targets.depth;
		desc.samples = Targets.samples;
		desc.view_mask = Targets.view_mask;
		desc.specialization = {settings & 1u, (settings >> 1) & 1u, (settings >> 2) & 1u, (settings >> 3) & 1u, (settings >> 4) & 1u, (settings >> 5) & 1u};
		desc.name = Program.name;
		pipeline = ctx.pipelines.get(desc);
		Program.pipelines.emplace(key, pipeline);
	}
	if (pipeline == VK_NULL_HANDLE)
	{
		return false;
	}
	if (pipeline != draw.pipeline)
	{
		draw.commands->bind_pipeline(VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
		draw.pipeline = pipeline;
	}
	if (topology != draw.topology)
	{
		draw.commands->set_primitive_topology(topology);
		draw.topology = topology;
	}
	return true;
}

// the dynamic state of the draws for the state of the walk: the faces culled, the depth tested and written
void vulkan14_renderer::apply_draw_state()
{
	auto &draw{*m_draw};
	auto const cull{draw.cullface ? (draw.shadows ? VK_CULL_MODE_FRONT_BIT : VK_CULL_MODE_BACK_BIT) : VK_CULL_MODE_NONE};
	if (cull != draw.cull)
	{
		draw.commands->set_cull_mode(cull);
		draw.cull = cull;
	}
	auto const depthtest{draw.glare ? 0 : 1};
	if (depthtest != draw.depthtest)
	{
		draw.commands->set_depth_test_enable(depthtest ? VK_TRUE : VK_FALSE);
		draw.depthtest = depthtest;
	}
	auto const depthwrite{(draw.blending || draw.depthmask_off) ? 0 : 1};
	if (depthwrite != draw.depthwrite)
	{
		draw.commands->set_depth_write_enable(depthwrite ? VK_TRUE : VK_FALSE);
		draw.depthwrite = depthwrite;
	}
}

// the textures of the draw pushed: the material's, the white stand-in where there is none or it isn't there yet
void vulkan14_renderer::push_textures()
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &draw{*m_draw};
	std::array<VkDescriptorImageInfo, gl::MAX_TEXTURES> images;
	for (std::size_t unit = 0; unit < gl::MAX_TEXTURES; ++unit)
	{
		images[unit] = VkDescriptorImageInfo{ctx.sampling.repeat_aniso, scene.white->view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
		auto const texture{draw.textures[unit]};
		if ((texture == null_handle) || (false == bool(ctx.textures)))
		{
			continue;
		}
		auto &source{ctx.textures->mark_as_used(texture)};
		if (false == source.create())
		{
			continue;
		}
		if (auto const *entry{ctx.images.find(source.get_id())})
		{
			images[unit] = VkDescriptorImageInfo{entry->sampler, entry->target->view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
		}
	}
	scene.material_set.push(draw.commands, scene.scene_layout, images.data());
	draw.textures_dirty = false;
}

// which program of the depth prepass stands for the program of the material (opengl46_renderer::Prepass_Class()): 0 none, 1 the
// one without the alpha test, 2 the one with it
int vulkan14_renderer::prepass_class(material_handle const Material)
{
	if (Material == null_handle)
	{
		return 0;
	}
	auto const &material{m_materials.material(Material)};
	if ((false == material.is_good) || (material.shader == nullptr))
	{
		return 0;
	}
	auto &scene{m_context->scene};
	auto const lookup{scene.prepass_classes.find(material.shader.get())};
	if (lookup != scene.prepass_classes.end())
	{
		return lookup->second;
	}
	// names the programs are fetched by: mat_<name>.frag
	static std::unordered_set<std::string> const alphatested{"default",           "default_1",           "default_2",    "default_detail",        "default_specgloss",
	                                                         "detail_normalmap",  "detail_normalmap_specgloss", "normalmap", "normalmap_specgloss", "reflmap",
	                                                         "reflmap_specgloss", "shadowlessnormalmap", "shadowlessnormalmap_specgloss", "sunlessnormalmap", "sunlessnormalmap_specgloss",
	                                                         "trackblend",        "trackblend_normalmap", "trackblend_specgloss"};
	static std::unordered_set<std::string> const solid{"colored", "default_0", "terrain", "water", "water_specgloss", "invalid"};
	auto prepassclass{0};
	for (auto const &program : scene.materials)
	{
		if (program.second.get() != material.shader.get())
		{
			continue;
		}
		prepassclass = (alphatested.count(program.first) > 0 ? 2 : solid.count(program.first) > 0 ? 1 : 0);
		break;
	}
	scene.prepass_classes.emplace(material.shader.get(), prepassclass);
	return prepassclass;
}

// textures: the files read and decoded as for the opengl renderer, made into images when they are first used

texture_handle vulkan14_renderer::Fetch_Texture(std::string const &Filename, bool const Loadnow, GLint format_hint)
{
	if (!m_context || !m_context->textures)
	{
		return null_handle;
	}
	return m_context->textures->create(Filename, Loadnow, format_hint);
}

void vulkan14_renderer::Bind_Texture(texture_handle const Texture)
{
	Bind_Texture(0, Texture);
}

void vulkan14_renderer::Bind_Texture(std::size_t const Unit, texture_handle const Texture)
{
	if (m_draw)
	{
		walk_texture(Unit, Texture);
	}
}

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
