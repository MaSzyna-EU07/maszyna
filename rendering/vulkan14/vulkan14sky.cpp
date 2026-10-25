/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the light of the sky of the vulkan renderer (the stage P1 of the plan, opengl46_sky and opengl46_exposure of the opengl
// renderer): the look-up tables of the physical atmosphere made by compute kernels, the light of the frame worked out from
// them and the directional part of the ambient light (gfx::sky_light, shared with the opengl renderer), the environment map
// prefiltered with the GGX lobe, the stars of the catalogue; the automatic exposure and the bloom of the post-processing

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14context.h"
#include "rendering/vulkan14/vulkan14commands.h"
#include "rendering/common/exposure.h"
#include "simulation/simulation.h"
#include "simulation/simulationenvironment.h"
#include "utilities/Globals.h"
#include "utilities/Timer.h"

namespace
{

VkPipelineStageFlags2 const shader_stages{VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT};
VkPipelineStageFlags2 const sampled_stage{VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT};
VkAccessFlags2 const sampled_access{VK_ACCESS_2_SHADER_SAMPLED_READ_BIT};
VkPipelineStageFlags2 const compute_stage{VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT};
VkAccessFlags2 const storage_write{VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT};

std::uint32_t groups(std::uint32_t const Size, std::uint32_t const Group)
{
	return (Size + Group - 1) / Group;
}

// a view of one level of a cube image as a cube, for a kernel writing the level
VkImageView level_view(VkDevice Device, vk14::image const &Image, std::uint32_t const Level, std::string const &Name)
{
	VkImageViewCreateInfo info{};
	info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
	info.image = Image.handle();
	info.viewType = VK_IMAGE_VIEW_TYPE_CUBE;
	info.format = Image.desc().format;
	info.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, Level, 1, 0, 6};
	VkImageView view{VK_NULL_HANDLE};
	if (!VK14_VERIFY(vkCreateImageView(Device, &info, nullptr, &view)))
	{
		return VK_NULL_HANDLE;
	}
	vk14::name_object(Device, VK_OBJECT_TYPE_IMAGE_VIEW, vk14::handle_value(view), Name);
	return view;
}

// the push constants of the kernels, laid out as the blocks of the shaders (std430)
struct prefilter_constants
{
	float roughness;
	std::int32_t target_size;
	float source_size;
	std::int32_t sample_count;
};
struct histogram_constants
{
	glm::vec2 log_range;
};
struct average_constants
{
	glm::vec2 log_range;
	glm::vec2 pad;
	glm::vec4 adaptation;
	glm::vec2 exposure_range;
};

} // namespace


bool vulkan14_renderer::init_sky(std::string &Reason)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &sky{scene.sky};
	auto &exposure{scene.exposure};
	auto const device{ctx.device.handle()};
	auto const frames{static_cast<std::uint32_t>(ctx.frames.size())};

	// the buffers the shading reads
	if ((false == sky.sky.create(ctx.memory, sizeof(gl::sky_data), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, vk14::memory_kind::device, "sky ambient")) ||
	    (false == sky.atmosphere.create(ctx.memory, sizeof(gl::atmosphere_ubs), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, vk14::memory_kind::device, "atmosphere")) ||
	    (false == sky.result.create(ctx.memory, sizeof(gl::atmosphere_result), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, vk14::memory_kind::device, "atmosphere result")) ||
	    (false == sky.result_readback.create(ctx.memory, sizeof(gl::atmosphere_result), frames, "atmosphere result readback")) ||
	    (false == exposure.data.create(ctx.memory, sizeof(gl::exposure_data), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, vk14::memory_kind::device, "exposure")) ||
	    (false == exposure.readback.create(ctx.memory, 4 * sizeof(float), frames, "exposure readback")))
	{
		Reason = "no buffers of the light of the sky";
		return false;
	}
	// the tables of the atmosphere, black until they are first made
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
		desc.format = VK_FORMAT_R16G16B16A16_SFLOAT;
		desc.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
		desc.extent = {gl::ATMOSPHERE_TRANSMITTANCE_WIDTH, gl::ATMOSPHERE_TRANSMITTANCE_HEIGHT, 1};
		if (false == make_image(sky.transmittance, desc, "atmosphere transmittance"))
		{
			return false;
		}
		desc.extent = {gl::ATMOSPHERE_MULTISCATTERING_SIZE, gl::ATMOSPHERE_MULTISCATTERING_SIZE, 1};
		if (false == make_image(sky.multiscattering, desc, "atmosphere multiple scattering"))
		{
			return false;
		}
		desc.extent = {gl::ATMOSPHERE_SKYVIEW_WIDTH, gl::ATMOSPHERE_SKYVIEW_HEIGHT, 1};
		if (false == make_image(sky.skyview, desc, "atmosphere sky-view"))
		{
			return false;
		}
	}
	// the environment map prefiltered with the GGX lobe, made with the environment map
	if (Global.gfx_envmap_enabled)
	{
		vk14::image_desc desc;
		desc.format = VK_FORMAT_R16G16B16A16_SFLOAT;
		desc.view_type = VK_IMAGE_VIEW_TYPE_CUBE;
		desc.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
		desc.layers = 6;
		desc.extent = {static_cast<std::uint32_t>(gl::ENVMAP_GGX_SIZE), static_cast<std::uint32_t>(gl::ENVMAP_GGX_SIZE), 1};
		desc.mips = static_cast<std::uint32_t>(gl::ENVMAP_GGX_LEVELS);
		desc.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
		if (false == make_image(sky.ggx, desc, "environment map ggx"))
		{
			return false;
		}
		for (std::uint32_t level = 0; level < desc.mips; ++level)
		{
			sky.ggx_levels.push_back(level_view(device, *sky.ggx, level, "environment map ggx " + std::to_string(level)));
		}
	}
	// the stars of the catalogue, in the equatorial frame
	{
		auto const stars{gfx::weather_state::star_vertices()};
		sky.star_count = static_cast<std::uint32_t>(stars.size() / 2);
		if (sky.star_count > 0)
		{
			auto vertices{std::make_shared<vk14::buffer>()};
			auto const size{static_cast<VkDeviceSize>(stars.size() * sizeof(glm::vec4))};
			if ((false == vertices->create(ctx.memory, size, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, vk14::memory_kind::device, "stars")) || (false == ctx.uploads.upload_buffer(vertices, stars.data(), size)))
			{
				Reason = "no buffer of the stars";
				return false;
			}
			sky.star_vertices = vertices;
		}
	}
	// the kernels and the programs
	try
	{
		auto kernel = [&](vk14::compute_kernel &Kernel, char const *Shader, char const *Name) {
			if (false == Kernel.create(device, ctx.pipelines, ctx.shaders.get(Shader), scene.pass_layout, {}, Name))
			{
				Reason = std::string("no compute pipeline of ") + Name;
				return false;
			}
			return true;
		};
		if ((false == kernel(sky.transmittance_kernel, "atmosphere_transmittance.comp", "atmosphere transmittance")) || (false == kernel(sky.multiscattering_kernel, "atmosphere_multiscattering.comp", "atmosphere multiple scattering")) ||
		    (false == kernel(sky.skyview_kernel, "atmosphere_skyview.comp", "atmosphere sky-view")) || (false == kernel(sky.ambient_kernel, "atmosphere_ambient.comp", "atmosphere ambient")) ||
		    (false == kernel(sky.prefilter_kernel, "envmap_prefilter.comp", "environment map ggx")) || (false == kernel(exposure.histogram, "exposure_histogram.comp", "exposure histogram")) ||
		    (false == kernel(exposure.average, "exposure_average.comp", "exposure average")))
		{
			return false;
		}
		auto program = [&](char const *Vertex, char const *Fragment, vk14::vertex_kind const Kind) {
			auto result{std::make_shared<vk14::scene_program>()};
			result->vertex = ctx.shaders.get(Vertex);
			result->fragment = ctx.shaders.get(Fragment);
			result->fragment_name = Fragment;
			result->vertices = Kind;
			result->name = std::string(Vertex) + " + " + Fragment;
			return result;
		};
		sky.atmosphere_dome = program("vbocolor.vert", "sky_atmosphere.frag", vk14::vertex_kind::colour);
		sky.stars = program("stars.vert", "stars.frag", vk14::vertex_kind::star);
		for (auto const *name : {"postfx_bloom_down.frag", "postfx_bloom_up.frag"})
		{
			scene.post_fragments.emplace(name, ctx.shaders.get(name));
		}
	}
	catch (gfx::shader_exception const &e)
	{
		Reason = std::string("invalid shader: ") + e.what();
		return false;
	}
	return true;
}

void vulkan14_renderer::destroy_sky()
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &sky{scene.sky};
	auto &exposure{scene.exposure};
	auto const device{ctx.device.handle()};
	for (auto *kernel : {&sky.transmittance_kernel, &sky.multiscattering_kernel, &sky.skyview_kernel, &sky.ambient_kernel, &sky.prefilter_kernel, &exposure.histogram, &exposure.average})
	{
		kernel->destroy(device);
	}
	for (auto *view : sky.ggx_levels)
	{
		if (view != VK_NULL_HANDLE)
		{
			vkDestroyImageView(device, view, nullptr);
		}
	}
	sky.ggx_levels.clear();
	sky.ggx.reset();
	sky.transmittance.reset();
	sky.multiscattering.reset();
	sky.skyview.reset();
	sky.sky.destroy();
	sky.atmosphere.destroy();
	sky.result.destroy();
	sky.result_readback.destroy();
	sky.atmosphere_dome.reset();
	sky.stars.reset();
	sky.star_vertices.reset();
	exposure.data.destroy();
	exposure.readback.destroy();
}

// the images of the sky put in their layouts the first time: black, ready to be read
void vulkan14_renderer::clear_sky_images()
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &sky{scene.sky};
	auto const commands{ctx.commands};
	std::vector<vk14::image_use> uses;
	for (auto *image : {sky.transmittance.get(), sky.multiscattering.get(), sky.skyview.get(), sky.ggx.get()})
	{
		if (image != nullptr)
		{
			uses.push_back(vk14::use_of(*image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_PIPELINE_STAGE_2_CLEAR_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT));
		}
	}
	vk14::barriers(commands, uses);
	VkClearColorValue const black{{0.f, 0.f, 0.f, 0.f}};
	for (auto *image : {sky.transmittance.get(), sky.multiscattering.get(), sky.skyview.get(), sky.ggx.get()})
	{
		if (image != nullptr)
		{
			VkImageSubresourceRange const range{VK_IMAGE_ASPECT_COLOR_BIT, 0, image->desc().mips, 0, image->desc().layers};
			commands->clear_color_image(image->handle(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &black, 1, &range);
		}
	}
	uses.clear();
	for (auto *image : {sky.transmittance.get(), sky.multiscattering.get(), sky.skyview.get(), sky.ggx.get()})
	{
		if (image != nullptr)
		{
			uses.push_back(vk14::use_of(*image, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access));
		}
	}
	vk14::barriers(commands, uses);
	// the exposure starts from the key, not adapted yet
	gl::exposure_data data{};
	data.adapted_luminance = Global.gfx_postfx_autoexposure_key;
	data.exposure = 1.f;
	data.average_luminance = Global.gfx_postfx_autoexposure_key;
	data.valid = 0;
	vk14::update_buffer(commands, scene.exposure.data, &data, sizeof(data), 0);
	// the atmosphere valid from the start, and no directional ambient light
	vk14::update_buffer(commands, sky.atmosphere, &m_skylight.atmosphere(), sizeof(gl::atmosphere_ubs), 0);
	vk14::update_buffer(commands, sky.sky, &sky.uploaded, sizeof(gl::sky_data), 0);
	sky.sky_uploaded = true;
}

// the light of the sky of the frame (opengl46_sky::update()): the tables of the atmosphere made where their input changed, the
// result of an earlier frame read back, the sunlight, the ambient light and the fog colour of the frame worked out from it and
// the directional ambient light written for the shading. before the passes of the frame
void vulkan14_renderer::update_sky()
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &sky{scene.sky};
	auto const commands{ctx.commands};
	auto const &sun{simulation::Environment.sun()};
	auto const sundirection{sun.getDirection()};

	m_skylight.begin_update();
	if (Global.gfx_atmosphere)
	{
		bool medium{false};
		if (m_skylight.atmosphere_changed(glm::normalize(sundirection), medium))
		{
			ctx.passes.begin_pass("atmosphere");
			vk14::update_buffer(commands, sky.atmosphere, &m_skylight.atmosphere(), sizeof(gl::atmosphere_ubs), 0);
			bind_compute_set(sky.transmittance_kernel);
			auto const sampled = [&](vk14::image &Image) { return vk14::image_descriptor(ctx.sampling.clamp_linear, Image.view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL); };
			if (medium)
			{
				vk14::barriers(commands, {vk14::use_of(*sky.transmittance, VK_IMAGE_LAYOUT_GENERAL, compute_stage, storage_write)});
				sky.transmittance_kernel.run(commands, {vk14::storage_image(sky.transmittance->view())}, groups(gl::ATMOSPHERE_TRANSMITTANCE_WIDTH, 8), groups(gl::ATMOSPHERE_TRANSMITTANCE_HEIGHT, 8));
				vk14::barriers(commands, {vk14::use_of(*sky.transmittance, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access),
				                          vk14::use_of(*sky.multiscattering, VK_IMAGE_LAYOUT_GENERAL, compute_stage, storage_write)});
				sky.multiscattering_kernel.run(commands, {sampled(*sky.transmittance), vk14::storage_image(sky.multiscattering->view())}, gl::ATMOSPHERE_MULTISCATTERING_SIZE, gl::ATMOSPHERE_MULTISCATTERING_SIZE);
				vk14::barriers(commands, {vk14::use_of(*sky.multiscattering, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access)});
			}
			vk14::barriers(commands, {vk14::use_of(*sky.skyview, VK_IMAGE_LAYOUT_GENERAL, compute_stage, storage_write)});
			sky.skyview_kernel.run(commands, {sampled(*sky.transmittance), sampled(*sky.multiscattering), vk14::storage_image(sky.skyview->view())}, groups(gl::ATMOSPHERE_SKYVIEW_WIDTH, 8), groups(gl::ATMOSPHERE_SKYVIEW_HEIGHT, 8));
			vk14::barriers(commands, {vk14::use_of(*sky.skyview, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access)});
			vk14::memory_barrier(commands, VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_READ_BIT, compute_stage, storage_write);
			sky.ambient_kernel.run(commands, {sampled(*sky.transmittance), sampled(*sky.skyview), vk14::buffer_descriptor(sky.result.handle(), 0, sizeof(gl::atmosphere_result))}, 1);
			sky.result_readback.push(commands, ctx.slot, ctx.timelinevalue + 1, sky.result.handle(), 0, sizeof(gl::atmosphere_result));
			ctx.passes.end_pass();
		}
		gl::atmosphere_result result;
		if (sky.result_readback.pop(ctx.completed(), &result, sizeof(result)))
		{
			m_skylight.set_result(result);
		}
	}
	m_skylight.end_update(m_sunlight, m_fogcolor, simulation::Environment.skydome(), sundirection, sun.getAngle());

	// the directional part of the ambient light for the shading
	auto const &skydata{m_skylight.sky_data()};
	if ((false == sky.sky_uploaded) || (0 != std::memcmp(&skydata, &sky.uploaded, sizeof(skydata))))
	{
		sky.uploaded = skydata;
		vk14::update_buffer(commands, sky.sky, &sky.uploaded, sizeof(gl::sky_data), 0);
		sky.sky_uploaded = true;
	}
}

// the levels of the GGX map prefiltered from the environment map with its mipmaps (opengl46_sky::prefilter())
void vulkan14_renderer::prefilter_environment()
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &sky{scene.sky};
	if ((false == bool(sky.ggx)) || (false == bool(scene.env_map)))
	{
		return;
	}
	auto const commands{ctx.commands};
	ctx.passes.begin_pass("envmap ggx", {vk14::use_of(*scene.env_map, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, compute_stage, sampled_access),
	                                     vk14::use_of(*sky.ggx, VK_IMAGE_LAYOUT_GENERAL, compute_stage, storage_write)});
	for (std::size_t level = 0; level < gl::ENVMAP_GGX_LEVELS; ++level)
	{
		auto const size{static_cast<std::int32_t>(gl::ENVMAP_GGX_SIZE >> level)};
		prefilter_constants constants;
		constants.roughness = gl::ENVMAP_GGX_ROUGHNESS_MIN + (1.f - gl::ENVMAP_GGX_ROUGHNESS_MIN) * static_cast<float>(level) / static_cast<float>(gl::ENVMAP_GGX_LEVELS - 1);
		constants.target_size = size;
		constants.source_size = static_cast<float>(gl::ENVMAP_SIZE);
		constants.sample_count = 64;
		auto const count{groups(static_cast<std::uint32_t>(size), static_cast<std::uint32_t>(gl::ENVMAP_GGX_GROUP_SIZE))};
		sky.prefilter_kernel.dispatch(commands, {vk14::storage_image(sky.ggx_levels[level]), vk14::image_descriptor(ctx.sampling.clamp_linear, scene.env_map->view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)}, constants, count, count, 6);
	}
	vk14::barriers(commands, {vk14::use_of(*sky.ggx, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access), vk14::use_of(*scene.env_map, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access)});
	ctx.passes.end_pass();
	sky.ggx_ready = true;
}

// the stars, in the sky pass (opengl46_weather::draw_stars()). View: the rotation of the camera of the pass
void vulkan14_renderer::render_stars(glm::mat4 const &View)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &sky{scene.sky};
	auto &draw{*m_draw};
	if ((sky.star_count == 0) || (false == bool(sky.star_vertices)) || (false == bool(sky.stars)))
	{
		return;
	}
	auto const &sun{simulation::Environment.sun()};
	glm::mat4 toworld, toview;
	glm::vec4 params;
	if (false == m_weatherstate.stars(sun.getAngle(), sun.getSiderealTime(), sun.getLatitude(), View, toworld, toview, params))
	{
		return;
	}
	if (false == bind_scene_pipeline(*sky.stars, *draw.targets, vk14::blend_mode::additive, VK_PRIMITIVE_TOPOLOGY_POINT_LIST))
	{
		return;
	}
	apply_draw_state();
	model_ubs.param[0] = params;
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
	for (int column = 0; column < 3; ++column)
	{
		constants.data[column] = toworld[column];
		constants.data[3 + column] = toview[column];
	}
	push_constants(&constants);
	VkDeviceSize const offset{0};
	auto const buffer{sky.star_vertices->handle()};
	draw.commands->bind_vertex_buffers(0, 1, &buffer, &offset);
	draw.commands->draw(sky.star_count, 1, 0, 0);
	draw.vertices = VK_NULL_HANDLE;
	draw.userdata = VK_NULL_HANDLE;
}

// the luminance of the image measured and the exposure adapted to it (opengl46_exposure::measure()), outside a rendering
void vulkan14_renderer::measure_exposure(vk14::image &Image)
{
	auto &ctx{*m_context};
	auto &exposure{ctx.scene.exposure};
	auto const commands{ctx.commands};
	float data[4];
	if (exposure.readback.pop(ctx.completed(), data, sizeof(data)))
	{
		std::copy(std::begin(data), std::end(data), std::begin(exposure.values));
	}
	vk14::barriers(commands, {vk14::use_of(Image, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, compute_stage, sampled_access)});
	// the reads of the tonemapping of the frame before
	vk14::memory_barrier(commands, VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_TRANSFER_READ_BIT, compute_stage, VK_ACCESS_2_SHADER_STORAGE_READ_BIT | storage_write);
	auto const buffer{vk14::buffer_descriptor(exposure.data.handle(), 0, sizeof(gl::exposure_data))};
	histogram_constants histogram{gfx::exposure_settings::histogram_range()};
	auto const groupsize{2 * gl::EXPOSURE_HISTOGRAM_GROUP_SIZE};
	exposure.histogram.dispatch(commands, {vk14::image_descriptor(ctx.sampling.clamp_nearest, Image.view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL), buffer}, histogram, groups(Image.desc().extent.width, groupsize),
	                            groups(Image.desc().extent.height, groupsize));
	vk14::memory_barrier(commands, compute_stage, storage_write, compute_stage, VK_ACCESS_2_SHADER_STORAGE_READ_BIT | storage_write);
	average_constants average{};
	average.log_range = gfx::exposure_settings::average_range();
	average.adaptation = gfx::exposure_settings::adaptation(Timer::GetDeltaRenderTime());
	average.exposure_range = gfx::exposure_settings::exposure_range();
	exposure.average.dispatch(commands, {buffer}, average, 1);
	// read by the tonemapping, and copied for the debug window
	vk14::memory_barrier(commands, compute_stage, storage_write, VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_TRANSFER_READ_BIT);
	exposure.readback.push(commands, ctx.slot, ctx.timelinevalue + 1, exposure.data.handle(), offsetof(gl::exposure_data, adapted_luminance), 4 * sizeof(float));
}

// the image taken down and up the bloom chain of the viewport (opengl46_exposure::bloom()); the result, which the tonemapping
// blends in, or null without the chain
vk14::image *vulkan14_renderer::render_bloom(std::size_t const Index, vk14::image &Image)
{
	auto &ctx{*m_context};
	auto &targets{ctx.scene.targets[Index]};
	if (targets.bloom_sizes.empty())
	{
		return nullptr;
	}
	// down, the first step from the image itself
	for (std::size_t level = 0; level < targets.bloom_sizes.size(); ++level)
	{
		vk14::draw_constants constants;
		constants.data[0] = gfx::exposure_settings::bloom_params(level);
		post_pass("postfx_bloom_down.frag", *targets.bloom_down[level], {level == 0 ? &Image : targets.bloom_down[level - 1].get()}, constants);
	}
	// up, from the lowest level
	for (std::size_t level = targets.bloom_up.size(); level-- > 0;)
	{
		auto *lower{(level + 1 < targets.bloom_up.size()) ? targets.bloom_up[level + 1].get() : targets.bloom_down[level + 1].get()};
		post_pass("postfx_bloom_up.frag", *targets.bloom_up[level], {lower, targets.bloom_down[level].get()}, vk14::draw_constants{});
	}
	return targets.bloom_up.empty() ? targets.bloom_down.front().get() : targets.bloom_up.front().get();
}
