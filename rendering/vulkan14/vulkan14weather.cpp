/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the weather of the vulkan renderer (the stage P2 of the plan, opengl46_weather of the opengl renderer), drawn from the weather of
// the simulation by gfx::weather_state, which it shares with the opengl renderer: the block of the weather of every pass, the
// colours of the fog towards the horizon from the sky-view table of the atmosphere, the noises of the clouds, the shadows of the
// clouds, the clouds of the main view (blended over frames) and the panorama of the clouds for the other views, the fog volume
// of the main view, and the clouds and the fog laid over the sky

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14context.h"
#include "rendering/vulkan14/vulkan14commands.h"
#include "simulation/simulation.h"
#include "simulation/simulationenvironment.h"
#include "utilities/Globals.h"

namespace
{

// sizes of the noises (clouds_noise.comp)
int const SHAPE_NOISE_SIZE{128};
int const DETAIL_NOISE_SIZE{32};
int const COVERAGE_NOISE_SIZE{512};

VkPipelineStageFlags2 const compute_stage{VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT};
VkPipelineStageFlags2 const sampled_stage{VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT};
VkAccessFlags2 const sampled_access{VK_ACCESS_2_SHADER_SAMPLED_READ_BIT};
VkAccessFlags2 const storage_write{VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT};

std::uint32_t groups(int const Size)
{
	return static_cast<std::uint32_t>((Size + static_cast<int>(gl::WEATHER_GROUP_SIZE) - 1) / static_cast<int>(gl::WEATHER_GROUP_SIZE));
}

// a view of the first level of an image, for a kernel writing it
VkImageView first_level_view(VkDevice Device, vk14::image const &Image, std::string const &Name)
{
	VkImageViewCreateInfo info{};
	info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
	info.image = Image.handle();
	info.viewType = Image.desc().view_type;
	info.format = Image.desc().format;
	info.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, Image.desc().layers};
	VkImageView view{VK_NULL_HANDLE};
	if (!VK14_VERIFY(vkCreateImageView(Device, &info, nullptr, &view)))
	{
		return VK_NULL_HANDLE;
	}
	vk14::name_object(Device, VK_OBJECT_TYPE_IMAGE_VIEW, vk14::handle_value(view), Name);
	return view;
}

// the push constants and the blocks of the kernels, laid out as those of the shaders
struct noise_constants
{
	std::int32_t kind;
	std::int32_t size;
};
struct horizon_constants
{
	std::int32_t offset;
	std::int32_t pad[3];
	glm::vec4 params;
	glm::vec3 white_balance;
	float pad1;
	glm::vec3 old_color;
	float pad2;
};
struct shadow_constants
{
	std::int32_t size;
	std::int32_t pad[3];
	glm::vec3 placement;
	std::int32_t steps;
};
struct march_constants
{
	std::int32_t mode;
	std::int32_t pad;
	glm::ivec2 size;
	glm::mat4 inverse_projection;
	glm::mat4 view_to_world;
	std::int32_t frame;
	std::int32_t pad1[3];
	glm::vec4 quality;
	glm::ivec2 rows;
};
static_assert(offsetof(march_constants, quality) == 160, "the layout of the constants of clouds_march.comp");
struct temporal_constants
{
	glm::ivec2 size;
	glm::ivec2 pad;
	glm::mat4 inverse_projection;
	glm::mat4 view_to_world;
	glm::mat4 previous_projection_view;
	glm::vec3 previous_camera;
	float blend;
};
static_assert(sizeof(temporal_constants) == 224, "the layout of the constants of clouds_temporal.comp");
struct inject_block
{
	glm::ivec3 grid;
	std::int32_t pad;
	glm::mat4 inverse_projection;
	glm::mat4 view_to_world;
	glm::vec4 slicing;
	glm::mat4 previous_projection_view;
	glm::vec3 previous_camera;
	float pad1;
	glm::vec4 options;
	std::int32_t cablamp;
	std::int32_t pad2;
	glm::vec2 previous_slicing;
	glm::vec2 jitter;
};
static_assert(offsetof(inject_block, cablamp) == 256, "the layout of the block of fog_inject.comp");
struct integrate_constants
{
	glm::ivec3 grid;
	std::int32_t pad;
	glm::mat4 inverse_projection;
	glm::mat4 view_to_world;
	glm::vec4 slicing;
};

} // namespace

bool vulkan14_renderer::init_weather(std::string &Reason)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &weather{scene.weather};
	auto const device{ctx.device.handle()};
	// the noises, mipmapped and repeated
	auto mipmapped = [&](std::shared_ptr<vk14::image> &Image, bool const Volume, int const Size, char const *Name) {
		vk14::image_desc desc;
		desc.type = (Volume ? VK_IMAGE_TYPE_3D : VK_IMAGE_TYPE_2D);
		desc.view_type = (Volume ? VK_IMAGE_VIEW_TYPE_3D : VK_IMAGE_VIEW_TYPE_2D);
		desc.format = VK_FORMAT_R8G8B8A8_UNORM;
		desc.extent = {static_cast<std::uint32_t>(Size), static_cast<std::uint32_t>(Size), Volume ? static_cast<std::uint32_t>(Size) : 1u};
		desc.mips = static_cast<std::uint32_t>(std::floor(std::log2(static_cast<float>(Size)))) + 1;
		desc.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
		// read by the passes of the weather on the compute queue (the async compute of M6)
		desc.shared = true;
		Image = std::make_shared<vk14::image>();
		if (false == Image->create(ctx.memory, desc, Name))
		{
			Image.reset();
			Reason = std::string("no image for ") + Name;
			return false;
		}
		return true;
	};
	if ((false == mipmapped(weather.shape_noise, true, SHAPE_NOISE_SIZE, "cloud shape noise")) || (false == mipmapped(weather.detail_noise, true, DETAIL_NOISE_SIZE, "cloud detail noise")) ||
	    (false == mipmapped(weather.coverage_noise, false, COVERAGE_NOISE_SIZE, "cloud coverage noise")))
	{
		return false;
	}
	{
		vk14::image_desc desc;
		desc.format = VK_FORMAT_R32_SFLOAT;
		desc.usage = VK_IMAGE_USAGE_STORAGE_BIT;
		desc.shared = true;
		weather.depth_dummy = std::make_shared<vk14::image>();
		if (false == weather.depth_dummy->create(ctx.memory, desc, "clouds panorama depth"))
		{
			weather.depth_dummy.reset();
			Reason = "no image for the depth of the panorama";
			return false;
		}
	}
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
		if ((false == kernel(weather.noise, "clouds_noise.comp", "clouds noise")) || (false == kernel(weather.march, "clouds_march.comp", "clouds march")) ||
		    (false == kernel(weather.temporal, "clouds_temporal.comp", "clouds temporal")) || (false == kernel(weather.shadow, "clouds_shadow.comp", "clouds shadow")) ||
		    (false == kernel(weather.horizon, "fog_horizon.comp", "fog horizon")) || (false == kernel(weather.inject, "fog_inject.comp", "fog inject")) || (false == kernel(weather.integrate, "fog_integrate.comp", "fog integrate")))
		{
			return false;
		}
		weather.sky = std::make_shared<vk14::scene_program>();
		weather.sky->vertex = ctx.shaders.get("quad.vert", "#define QUAD_SCENE 1\n");
		weather.sky->fragment = ctx.shaders.get("sky_weather.frag");
		weather.sky->fragment_name = "sky_weather.frag";
		weather.sky->vertices = vk14::vertex_kind::none;
		weather.sky->name = "quad.vert + sky_weather.frag";
	}
	catch (gfx::shader_exception const &e)
	{
		Reason = std::string("invalid shader: ") + e.what();
		return false;
	}
	return true;
}

void vulkan14_renderer::destroy_weather()
{
	auto &ctx{*m_context};
	auto &weather{ctx.scene.weather};
	auto const device{ctx.device.handle()};
	for (auto *kernel : {&weather.noise, &weather.march, &weather.temporal, &weather.shadow, &weather.horizon, &weather.inject, &weather.integrate})
	{
		kernel->destroy(device);
	}
	for (auto *image : {&weather.shape_noise, &weather.detail_noise, &weather.coverage_noise, &weather.depth_dummy, &weather.shadow_map, &weather.clouds_color, &weather.clouds_depth, &weather.clouds_history[0], &weather.clouds_history[1], &weather.panorama,
	                    &weather.fog_scattering[0], &weather.fog_scattering[1], &weather.fog_integrated})
	{
		image->reset();
	}
	weather.sky.reset();
}

namespace
{

// an image of the weather made anew: the old one goes once the gpu is done with the frames which used it (Value). Shared: made by the
// passes of the weather on the compute queue (the async compute of M6)
std::shared_ptr<vk14::image> weather_image(vk14::allocator &Memory, vk14::deletion_queue &Deletions, std::uint64_t const Value, std::shared_ptr<vk14::image> &Image, VkImageType const Type, VkFormat const Format, glm::ivec3 const &Size, char const *Name,
                                           bool const Shared = false)
{
	if (Image)
	{
		Deletions.push(Value, [old = std::move(Image)]() {});
	}
	vk14::image_desc desc;
	desc.type = Type;
	desc.view_type = (Type == VK_IMAGE_TYPE_3D ? VK_IMAGE_VIEW_TYPE_3D : VK_IMAGE_VIEW_TYPE_2D);
	desc.format = Format;
	desc.extent = {static_cast<std::uint32_t>(std::max(Size.x, 1)), static_cast<std::uint32_t>(std::max(Size.y, 1)), static_cast<std::uint32_t>(std::max(Size.z, 1))};
	desc.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
	desc.shared = Shared;
	Image = std::make_shared<vk14::image>();
	if (false == Image->create(Memory, desc, Name))
	{
		Image.reset();
	}
	return Image;
}

} // namespace

// the three noises, once (opengl46_weather::make_noise())
void vulkan14_renderer::make_cloud_noise()
{
	auto &ctx{*m_context};
	auto &weather{ctx.scene.weather};
	auto const commands{m_commands};
	auto const device{ctx.device.handle()};
	ctx.passes.begin_pass("clouds noise", {vk14::use_of(*weather.shape_noise, VK_IMAGE_LAYOUT_GENERAL, compute_stage, storage_write), vk14::use_of(*weather.detail_noise, VK_IMAGE_LAYOUT_GENERAL, compute_stage, storage_write),
	                                       vk14::use_of(*weather.coverage_noise, VK_IMAGE_LAYOUT_GENERAL, compute_stage, storage_write)});
	// the kernel writes the first levels
	auto const shape{first_level_view(device, *weather.shape_noise, "cloud shape noise level 0")};
	auto const detail{first_level_view(device, *weather.detail_noise, "cloud detail noise level 0")};
	auto const coverage{first_level_view(device, *weather.coverage_noise, "cloud coverage noise level 0")};
	auto const make = [&](int const Kind, VkImageView Volume, int const Size) {
		weather.noise.dispatch(commands, {vk14::storage_image(Volume), vk14::storage_image(coverage)}, noise_constants{Kind, Size}, groups(Size), groups(Size), Kind == 2 ? 1u : static_cast<std::uint32_t>(Size));
		vk14::memory_barrier(commands, compute_stage, storage_write, compute_stage, storage_write);
	};
	make(0, shape, SHAPE_NOISE_SIZE);
	make(1, detail, DETAIL_NOISE_SIZE);
	make(2, shape, COVERAGE_NOISE_SIZE);
	ctx.deletions.push(ctx.uploads.current_value(), [device, shape, detail, coverage]() {
		vkDestroyImageView(device, shape, nullptr);
		vkDestroyImageView(device, detail, nullptr);
		vkDestroyImageView(device, coverage, nullptr);
	});
	// the mipmaps, each level from the one before it
	for (auto *image : {weather.shape_noise.get(), weather.detail_noise.get(), weather.coverage_noise.get()})
	{
		auto const &desc{image->desc()};
		auto barrier = [&](std::uint32_t Level, VkImageLayout From, VkImageLayout To, VkPipelineStageFlags2 SrcStage, VkAccessFlags2 SrcAccess, VkPipelineStageFlags2 DstStage, VkAccessFlags2 DstAccess) {
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
			entry.image = image->handle();
			entry.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, Level, 1, 0, 1};
			VkDependencyInfo dependency{};
			dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
			dependency.imageMemoryBarrierCount = 1;
			dependency.pImageMemoryBarriers = &entry;
			commands->pipeline_barrier2(&dependency);
		};
		barrier(0, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, compute_stage, storage_write, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
		for (std::uint32_t level = 1; level < desc.mips; ++level)
		{
			barrier(level, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_PIPELINE_STAGE_2_NONE, VK_ACCESS_2_NONE, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
			auto const extent = [&](std::uint32_t Level) {
				return VkOffset3D{std::max(static_cast<std::int32_t>(desc.extent.width >> Level), 1), std::max(static_cast<std::int32_t>(desc.extent.height >> Level), 1), std::max(static_cast<std::int32_t>(desc.extent.depth >> Level), 1)};
			};
			VkImageBlit2 blit{};
			blit.sType = VK_STRUCTURE_TYPE_IMAGE_BLIT_2;
			blit.srcSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, level - 1, 0, 1};
			blit.srcOffsets[1] = extent(level - 1);
			blit.dstSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, level, 0, 1};
			blit.dstOffsets[1] = extent(level);
			VkBlitImageInfo2 info{};
			info.sType = VK_STRUCTURE_TYPE_BLIT_IMAGE_INFO_2;
			info.srcImage = image->handle();
			info.srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
			info.dstImage = image->handle();
			info.dstImageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
			info.regionCount = 1;
			info.pRegions = &blit;
			info.filter = VK_FILTER_LINEAR;
			commands->blit_image2(&info);
			barrier(level, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
		}
		VkImageMemoryBarrier2 entry{};
		entry.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
		entry.srcStageMask = VK_PIPELINE_STAGE_2_BLIT_BIT;
		entry.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
		entry.dstStageMask = sampled_stage;
		entry.dstAccessMask = sampled_access;
		entry.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
		entry.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		entry.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		entry.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		entry.image = image->handle();
		entry.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, desc.mips, 0, 1};
		VkDependencyInfo dependency{};
		dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
		dependency.imageMemoryBarrierCount = 1;
		dependency.pImageMemoryBarriers = &entry;
		commands->pipeline_barrier2(&dependency);
		image->state = vk14::image_state{VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access};
	}
	ctx.passes.end_pass();
	weather.noise_made = true;
}

// the block of the weather of the pass to come, from gfx::weather_state (without the colours of the horizon, a block of their own)
void vulkan14_renderer::upload_weather_block()
{
	auto &ctx{*m_context};
	auto constexpr size{offsetof(gl::weather_ubs, fog_horizon)};
	auto const slice{ctx.ring.allocate(size, ctx.device.caps().limits.minUniformBufferOffsetAlignment)};
	if (false == bool(slice))
	{
		return;
	}
	std::memcpy(slice.data, &m_weatherstate.block(), size);
	m_draw->weather_offset = static_cast<std::uint32_t>(slice.offset);
	m_draw->set_dirty = true;
}

// what the weather of the simulation is for the frame (opengl46_weather::update()): the block, the colours of the horizon, the
// shadows of the clouds, a band of the panorama. before the passes of the frame
void vulkan14_renderer::update_weather()
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &weather{scene.weather};
	auto const commands{m_commands};
	if (false == weather.noise_made)
	{
		make_cloud_noise();
	}
	m_weatherstate.update(m_skylight.atmosphere_weight(), true, -m_sunlight.direction, glm::vec3{m_sunlight.diffuse} * simulation::Environment.light_intensity(), glm::vec3{m_sunlight.ambient}, m_fogcolor, Global.pCamera.Pos);
	auto const shadows{m_weatherstate.shadows_wanted()};
	if (shadows)
	{
		if (weather.shadow_size != Global.gfx_clouds_shadows_size)
		{
			weather.shadow_size = Global.gfx_clouds_shadows_size;
			weather_image(ctx.memory, ctx.deletions, ctx.uploads.current_value(), weather.shadow_map, VK_IMAGE_TYPE_2D, VK_FORMAT_R16_SFLOAT, glm::ivec3{weather.shadow_size, weather.shadow_size, 1}, "cloud shadow map", true);
			if (weather.shadow_map)
			{
				// read before it's first drawn (the pass sets hold it)
				vk14::barriers(commands, {vk14::use_of(*weather.shadow_map, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access)});
			}
		}
		m_weatherstate.place_shadows(weather.shadow_size, Global.pCamera.Pos);
	}
	upload_weather_block();

	ctx.passes.begin_pass("weather");
	bind_compute_set(weather.horizon);
	// the colours of the horizon: from the atmosphere on the gpu, the old fog colour otherwise
	if (m_weatherstate.horizon_on_gpu())
	{
		horizon_constants constants{};
		constants.params = glm::vec4{m_skylight.atmosphere_weight(), m_skylight.sky_scale(), 0.5f * m_skylight.overcast(), 64.f};
		constants.white_balance = m_skylight.white_balance();
		constants.old_color = m_fogcolor;
		vk14::memory_barrier(commands, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_UNIFORM_READ_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT, compute_stage, storage_write);
		weather.horizon.dispatch(commands, {vk14::buffer_descriptor(scene.weather_horizon.handle(), 0, sizeof(glm::vec4) * gl::FOG_HORIZON_SAMPLES)}, constants, 1);
		vk14::memory_barrier(commands, compute_stage, storage_write, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_UNIFORM_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_READ_BIT);
	}
	else
	{
		vk14::update_buffer(commands, scene.weather_horizon, &m_weatherstate.block().fog_horizon[0], sizeof(glm::vec4) * gl::FOG_HORIZON_SAMPLES, 0);
	}

	if (shadows && weather.shadow_map)
	{
		vk14::barriers(commands, {vk14::use_of(*weather.shadow_map, VK_IMAGE_LAYOUT_GENERAL, compute_stage, storage_write)});
		shadow_constants constants{};
		constants.size = weather.shadow_size;
		constants.placement = glm::vec3{m_weatherstate.block().cloud_shadow_params.x, m_weatherstate.block().cloud_shadow_params.y, Global.gfx_clouds_shadows_range};
		constants.steps = 8;
		weather.shadow.dispatch(commands, {vk14::storage_image(weather.shadow_map->view())}, constants, groups(weather.shadow_size), groups(weather.shadow_size), 1);
		vk14::barriers(commands, {vk14::use_of(*weather.shadow_map, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access)});
		m_weatherstate.shadows_drawn(true);
		upload_weather_block();
		bind_compute_set(weather.march);
	}
	else
	{
		m_weatherstate.shadows_drawn(false);
	}

	// a band of the panorama of the clouds, all of it the first time
	if (m_weatherstate.clouds_drawn())
	{
		auto const level{gfx::weather_state::quality(Global.gfx_clouds, Global.gfx_fog_quality)};
		if ((weather.panorama_size != level.panorama) || (false == bool(weather.panorama)))
		{
			weather.panorama_size = level.panorama;
			weather_image(ctx.memory, ctx.deletions, ctx.uploads.current_value(), weather.panorama, VK_IMAGE_TYPE_2D, VK_FORMAT_R16G16B16A16_SFLOAT, glm::ivec3{weather.panorama_size, 1}, "clouds panorama", true);
			m_weatherstate.panorama_resized();
		}
		if (weather.panorama)
		{
			int first, rows;
			m_weatherstate.panorama_band(weather.panorama_size, first, rows);
			vk14::barriers(commands, {vk14::use_of(*weather.panorama, VK_IMAGE_LAYOUT_GENERAL, compute_stage, storage_write), vk14::use_of(*weather.depth_dummy, VK_IMAGE_LAYOUT_GENERAL, compute_stage, storage_write)});
			march_constants constants{};
			constants.mode = 1;
			constants.size = weather.panorama_size;
			constants.frame = m_weatherstate.frame();
			// the panorama is seen blurred: fewer samples
			constants.quality = glm::vec4{static_cast<float>(std::max(level.steps / 2, 24)), static_cast<float>(std::max(level.light_steps - 1, 2)), Global.gfx_clouds_distance, 0.f};
			constants.rows = glm::ivec2{first, rows};
			// the depth isn't written in this mode, but something has to be bound
			weather.march.dispatch(commands, {vk14::storage_image(weather.panorama->view()), vk14::storage_image(weather.depth_dummy->view())}, constants, groups(weather.panorama_size.x), groups(rows), 1);
			vk14::barriers(commands, {vk14::use_of(*weather.panorama, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access)});
		}
	}
	ctx.passes.end_pass();
}

// the clouds and the fog volume of the colour pass of the main view (opengl46_weather::render_view()): Projection and View
// (rotation, world axes to the view) of its camera, Size of the target, Cablamp the light of the cab lamp or -1. the lights, the
// clusters and the block of the scene of the pass are written, the shadow map drawn. outside a rendering
void vulkan14_renderer::render_weather_view(glm::mat4 const &Projection, glm::mat4 const &View, glm::dvec3 const &Camera, glm::ivec2 const &Size, int const Cablamp)
{
	auto &ctx{*m_context};
	auto &weather{ctx.scene.weather};
	auto const commands{m_commands};
	auto const level{gfx::weather_state::quality(Global.gfx_clouds, Global.gfx_fog_quality)};
	// the images of the clouds of the main view and of the fog volume, for the size and the quality
	auto const cloudsize{glm::max((Size + level.divisor - 1) / level.divisor, glm::ivec2{1})};
	if (m_weatherstate.clouds_drawn() && ((cloudsize != weather.view_size) || (false == bool(weather.clouds_color))))
	{
		weather.view_size = cloudsize;
		weather_image(ctx.memory, ctx.deletions, ctx.uploads.current_value(), weather.clouds_color, VK_IMAGE_TYPE_2D, VK_FORMAT_R16G16B16A16_SFLOAT, glm::ivec3{cloudsize, 1}, "clouds colour");
		weather_image(ctx.memory, ctx.deletions, ctx.uploads.current_value(), weather.clouds_depth, VK_IMAGE_TYPE_2D, VK_FORMAT_R32_SFLOAT, glm::ivec3{cloudsize, 1}, "clouds depth");
		for (auto &history : weather.clouds_history)
		{
			weather_image(ctx.memory, ctx.deletions, ctx.uploads.current_value(), history, VK_IMAGE_TYPE_2D, VK_FORMAT_R16G16B16A16_SFLOAT, glm::ivec3{cloudsize, 1}, "clouds history");
		}
		m_weatherstate.previous_clouds.valid = false;
	}
	if ((Global.gfx_fog == 1) && ((level.fog_grid != weather.fog_grid) || (false == bool(weather.fog_integrated))))
	{
		weather.fog_grid = level.fog_grid;
		for (auto &scattering : weather.fog_scattering)
		{
			weather_image(ctx.memory, ctx.deletions, ctx.uploads.current_value(), scattering, VK_IMAGE_TYPE_3D, VK_FORMAT_R16G16B16A16_SFLOAT, weather.fog_grid, "fog scattering");
		}
		// one slice more: the fog through the whole volume without the clouds, which the sky takes (fog_integrate.comp)
		weather_image(ctx.memory, ctx.deletions, ctx.uploads.current_value(), weather.fog_integrated, VK_IMAGE_TYPE_3D, VK_FORMAT_R16G16B16A16_SFLOAT, weather.fog_grid + glm::ivec3{0, 0, 1}, "fog volume");
		if (weather.fog_integrated)
		{
			// the pass sets hold it
			vk14::barriers(commands, {vk14::use_of(*weather.fog_integrated, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access)});
		}
		m_weatherstate.previous_fog.valid = false;
	}
	auto const rotation{glm::mat4{glm::mat3{View}}};
	auto const inverseprojection{glm::inverse(Projection)};
	auto const viewtoworld{glm::transpose(rotation)};
	auto const projectionview{Projection * rotation};
	auto &previousclouds{m_weatherstate.previous_clouds};
	auto &previousfog{m_weatherstate.previous_fog};

	if (m_weatherstate.clouds_drawn() && weather.clouds_color && weather.clouds_depth && weather.clouds_history[0] && weather.clouds_history[1])
	{
		ctx.passes.begin_pass("clouds, fog volume", {vk14::use_of(*weather.clouds_color, VK_IMAGE_LAYOUT_GENERAL, compute_stage, storage_write), vk14::use_of(*weather.clouds_depth, VK_IMAGE_LAYOUT_GENERAL, compute_stage, storage_write)});
		bind_compute_set(weather.march);
		march_constants constants{};
		constants.mode = 0;
		constants.size = weather.view_size;
		constants.inverse_projection = inverseprojection;
		constants.view_to_world = viewtoworld;
		constants.frame = m_weatherstate.frame();
		constants.quality = glm::vec4{static_cast<float>(level.steps), static_cast<float>(level.light_steps), Global.gfx_clouds_distance, level.detail ? 1.f : 0.f};
		constants.rows = glm::ivec2{0, weather.view_size.y};
		weather.march.dispatch(commands, {vk14::storage_image(weather.clouds_color->view()), vk14::storage_image(weather.clouds_depth->view())}, constants, groups(weather.view_size.x), groups(weather.view_size.y), 1);

		auto const next{1 - weather.clouds_current};
		auto &history{*weather.clouds_history[weather.clouds_current]};
		auto &result{*weather.clouds_history[next]};
		vk14::barriers(commands, {vk14::use_of(*weather.clouds_color, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access),
		                          vk14::use_of(*weather.clouds_depth, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access), vk14::use_of(history, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access),
		                          vk14::use_of(result, VK_IMAGE_LAYOUT_GENERAL, compute_stage, storage_write)});
		temporal_constants blend{};
		blend.size = weather.view_size;
		blend.inverse_projection = inverseprojection;
		blend.view_to_world = viewtoworld;
		blend.previous_projection_view = previousclouds.projection_view;
		blend.previous_camera = glm::vec3{previousclouds.camera - Camera};
		blend.blend = gfx::weather_state::continuity(previousclouds, Camera) ? 0.15f : 1.f;
		auto const &linear{ctx.sampling.clamp_linear};
		auto const &nearest{ctx.sampling.clamp_nearest};
		weather.temporal.dispatch(commands,
		                          {vk14::image_descriptor(linear, weather.clouds_color->view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL), vk14::image_descriptor(nearest, weather.clouds_depth->view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL),
		                           vk14::image_descriptor(linear, history.view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL), vk14::storage_image(result.view())},
		                          blend, groups(weather.view_size.x), groups(weather.view_size.y), 1);
		vk14::barriers(commands, {vk14::use_of(result, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access)});
		weather.clouds_current = next;
		previousclouds = {projectionview, Camera, {}, true};
		m_weatherstate.clouds_made();
	}
	else
	{
		ctx.passes.begin_pass("clouds, fog volume");
		previousclouds.valid = false;
	}

	if ((Global.gfx_fog == 1) && weather.fog_integrated && weather.fog_scattering[0] && weather.fog_scattering[1])
	{
		bind_compute_set(weather.inject);
		auto const temporal{Global.gfx_fog_temporal && gfx::weather_state::continuity(previousfog, Camera)};
		auto const jitter{m_weatherstate.fog_jitter(temporal)};
		auto const slicing{gfx::weather_state::fog_slicing(weather.fog_grid.z, jitter)};
		auto const next{1 - weather.fog_current};
		auto &history{*weather.fog_scattering[weather.fog_current]};
		auto &result{*weather.fog_scattering[next]};
		vk14::barriers(commands, {vk14::use_of(result, VK_IMAGE_LAYOUT_GENERAL, compute_stage, storage_write), vk14::use_of(history, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access)});
		inject_block block{};
		block.grid = weather.fog_grid;
		block.inverse_projection = inverseprojection;
		block.view_to_world = viewtoworld;
		block.slicing = slicing;
		block.previous_projection_view = previousfog.projection_view;
		block.previous_camera = glm::vec3{previousfog.camera - Camera};
		auto const sunshadows{Global.gfx_fog_shadows && Global.gfx_shadowmap_enabled};
		auto const share{temporal ? std::clamp(Global.gfx_fog_temporal_share, 0.02f, 1.f) : 1.f};
		block.options = glm::vec4{share, Global.gfx_fog_lights, sunshadows ? 1.f : 0.f, Global.gfx_fog_shadows ? 1.f : 0.f};
		block.cablamp = Cablamp;
		block.previous_slicing = glm::vec2{previousfog.slicing.x, previousfog.slicing.z};
		block.jitter = glm::vec2{jitter.x, jitter.y};
		auto const slice{ctx.ring.allocate(sizeof(block), ctx.device.caps().limits.minUniformBufferOffsetAlignment)};
		if (slice)
		{
			std::memcpy(slice.data, &block, sizeof(block));
			weather.inject.run(commands,
			                   {vk14::storage_image(result.view()), vk14::image_descriptor(ctx.sampling.clamp_linear, history.view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL), vk14::buffer_descriptor(slice.buffer, slice.offset, sizeof(block))},
			                   groups(weather.fog_grid.x), groups(weather.fog_grid.y), static_cast<std::uint32_t>(weather.fog_grid.z));
		}
		vk14::barriers(commands, {vk14::use_of(result, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access), vk14::use_of(*weather.fog_integrated, VK_IMAGE_LAYOUT_GENERAL, compute_stage, storage_write)});
		integrate_constants integrate{};
		integrate.grid = weather.fog_grid;
		integrate.inverse_projection = inverseprojection;
		integrate.view_to_world = viewtoworld;
		integrate.slicing = slicing;
		weather.integrate.dispatch(commands, {vk14::image_descriptor(ctx.sampling.clamp_linear, result.view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL), vk14::storage_image(weather.fog_integrated->view())}, integrate,
		                           groups(weather.fog_grid.x), groups(weather.fog_grid.y), 1);
		vk14::barriers(commands, {vk14::use_of(*weather.fog_integrated, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access)});
		weather.fog_current = next;
		previousfog = {projectionview, Camera, slicing, true};
		m_weatherstate.fog_made(slicing, weather.fog_grid);
	}
	else
	{
		previousfog.valid = false;
	}
	ctx.passes.end_pass();
}

// the clouds and the fog over the sky, in the sky pass (opengl46_weather::draw_sky()). Mainview: the clouds of the main view, the
// panorama otherwise
void vulkan14_renderer::render_sky_weather(glm::mat4 const &Projection, bool const Mainview)
{
	auto &ctx{*m_context};
	auto &weather{ctx.scene.weather};
	auto &draw{*m_draw};
	if (false == bool(weather.sky))
	{
		return;
	}
	int mode{0};
	vk14::image *image{nullptr};
	if (m_weatherstate.clouds_drawn())
	{
		if (Mainview && m_weatherstate.clouds_view_valid() && weather.clouds_history[weather.clouds_current])
		{
			mode = 1;
			image = weather.clouds_history[weather.clouds_current].get();
		}
		else if (weather.panorama)
		{
			mode = 2;
			image = weather.panorama.get();
		}
	}
	// over the sky as it is: (1, source alpha), without the depth
	auto const glare{draw.glare};
	draw.glare = true;
	if (false == bind_scene_pipeline(*weather.sky, *draw.targets, vk14::blend_mode::under_transmittance, VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP))
	{
		draw.glare = glare;
		return;
	}
	auto const cull{draw.cullface};
	draw.cullface = false;
	apply_draw_state();
	draw.cullface = cull;
	draw.glare = glare;
	draw.block = push_draw_block();
	if (draw.block == ~0u)
	{
		return;
	}
	if (draw.set_dirty)
	{
		bind_pass_set();
	}
	// the clouds pushed as the first texture of the draw, the textures of the walk pushed again for the draws which follow
	std::array<VkDescriptorImageInfo, gl::MAX_TEXTURES> images;
	auto const &scene{ctx.scene};
	images.fill(VkDescriptorImageInfo{ctx.sampling.clamp_linear, scene.white->view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL});
	if (image != nullptr)
	{
		// the panorama repeats around the camera
		images[0] = VkDescriptorImageInfo{(mode == 2 ? ctx.sampling.panorama : ctx.sampling.clamp_linear), image->view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
	}
	scene.material_set.push(draw.commands, scene.scene_layout, images.data());
	draw.textures_dirty = true;
	vk14::draw_constants constants;
	constants.draw = draw.block;
	auto const inverseprojection{glm::inverse(Projection)};
	for (int column = 0; column < 4; ++column)
	{
		constants.data[column] = inverseprojection[column];
	}
	constants.data[4] = glm::vec4{static_cast<float>(mode), 0.f, 0.f, 0.f};
	push_constants(&constants);
	draw.commands->draw(4, 1, 0, 0);
}
