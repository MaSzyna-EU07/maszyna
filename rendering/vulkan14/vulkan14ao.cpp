/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the ambient occlusion of the vulkan renderer (the stage P3 of the plan, opengl46_ao and Render_ambient_occlusion() of the
// opengl renderer): ground truth ambient occlusion at half the resolution in two compute kernels (gtao_main.comp,
// gtao_denoise.comp) or the older ssao (postfx_ssao.frag and its blur), multiplied into the image of the scene
// (postfx_ssao_apply.frag) with the fog in front of the surfaces taken into account

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14context.h"
#include "rendering/vulkan14/vulkan14commands.h"
#include "utilities/Globals.h"

#include <random>

namespace
{

// work group of the compute passes, 8x8 (gtao_*.comp)
std::uint32_t const GROUP_SIZE{8};

struct gtao_constants
{
	glm::mat4 inv_projection;
	glm::vec2 params;
	glm::vec2 pad;
	glm::vec4 clipw_row;
	glm::ivec2 quality;
};
static_assert(offsetof(gtao_constants, clipw_row) == 80, "the layout of the constants of gtao_main.comp");

VkPipelineStageFlags2 const compute_stage{VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT};
VkPipelineStageFlags2 const sampled_stage{VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT};
VkAccessFlags2 const sampled_access{VK_ACCESS_2_SHADER_SAMPLED_READ_BIT};

} // namespace

bool vulkan14_renderer::init_ao(std::string &Reason)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto const device{ctx.device.handle()};
	// the noise of the older ssao: 4x4 random rotations in the plane of the surface, drawn after the hemisphere kernel the opengl
	// renderer makes with the same generator (the shader makes the kernel itself), so they are the same numbers
	{
		std::uniform_real_distribution<float> rnd(0.0f, 1.0f);
		std::default_random_engine gen(42); // NOSONAR fixed seed on purpose, the same as the opengl renderer
		for (int i = 0; i < 32; i++)
		{
			glm::vec3 s(rnd(gen) * 2.0f - 1.0f, rnd(gen) * 2.0f - 1.0f, rnd(gen));
			s = glm::normalize(s) * rnd(gen);
		}
		std::vector<glm::vec4> noise;
		for (int i = 0; i < 16; i++)
		{
			noise.emplace_back(glm::normalize(glm::vec3(rnd(gen) * 2.0f - 1.0f, rnd(gen) * 2.0f - 1.0f, 0.0f)), 1.f);
		}
		vk14::image_desc desc;
		desc.format = VK_FORMAT_R32G32B32A32_SFLOAT;
		desc.extent = {4, 4, 1};
		desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
		scene.ssao_noise = std::make_shared<vk14::image>();
		if ((false == scene.ssao_noise->create(ctx.memory, desc, "ssao noise")) || (false == ctx.uploads.upload(scene.ssao_noise, noise.data(), noise.size() * sizeof(glm::vec4), {{0, {4, 4, 1}}}, false)))
		{
			Reason = "no image of the noise of the ssao";
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
		if ((false == kernel(scene.gtao_main, "gtao_main.comp", "gtao")) || (false == kernel(scene.gtao_denoise, "gtao_denoise.comp", "gtao denoise")))
		{
			return false;
		}
		scene.ao_apply = std::make_shared<vk14::scene_program>();
		scene.ao_apply->vertex = ctx.shaders.get("quad.vert", "#define QUAD_SCENE 1\n");
		scene.ao_apply->fragment = ctx.shaders.get("postfx_ssao_apply.frag");
		scene.ao_apply->fragment_name = "postfx_ssao_apply.frag";
		scene.ao_apply->vertices = vk14::vertex_kind::none;
		scene.ao_apply->name = "quad.vert + postfx_ssao_apply.frag";
		for (auto const *name : {"postfx_ssao.frag", "postfx_ssao_blur.frag"})
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

void vulkan14_renderer::destroy_ao()
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto const device{ctx.device.handle()};
	scene.gtao_main.destroy(device);
	scene.gtao_denoise.destroy(device);
	scene.ao_apply.reset();
	scene.ssao_noise.reset();
}

// the ambient occlusion goes into the light of the surfaces of the colour pass of the main view (gfx.postfx.ao.lighting,
// opengl46_renderer::Ao_Lighting_Wanted()): the ground truth one, made from the depth of the whole scene drawn before the colour,
// which needs a perspective projection with the far plane at infinity (the distance of a surface worked out from its depth)
bool vulkan14_renderer::ao_lighting_wanted(viewport_config const &Viewport) const
{
	if ((Global.gfx_postfx_ao != 2) || (false == Global.gfx_postfx_ao_lighting) || (false == Viewport.main) || Global.bWireFrame || Global.gfx_skippipeline)
	{
		return false;
	}
	auto const &targets{m_context->scene.targets};
	if (targets.empty() || (false == bool(targets.front().ssao)) || (false == bool(targets.front().ssao_blur)))
	{
		return false;
	}
	auto const &projection{scene_ubs.projection};
	return (projection[2][3] != 0.f) && (projection[2][2] == 0.f) && (projection[3][2] > 0.f);
}

// the ground truth ambient occlusion of the depth of the scene drawn before the colour (render_scene_depth()), for the shading of the
// colour pass, left in the layout it reads it in. outside a rendering
bool vulkan14_renderer::compute_ao_lighting(std::size_t const Index)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &targets{scene.targets[Index]};
	auto const commands{ctx.commands};
	if ((false == bool(targets.ssao)) || (false == bool(targets.ssao_blur)) || (false == bool(targets.scene_depth)))
	{
		return false;
	}
	ctx.passes.begin_pass("ambient occlusion");
	// the projection the depth was drawn with
	auto const &projection{m_colorpass.pass_camera.projection()};
	auto const inverseprojection{glm::inverse(projection)};
	auto const width{targets.ssao->desc().extent.width};
	auto const height{targets.ssao->desc().extent.height};
	auto const groups_x{(width + GROUP_SIZE - 1) / GROUP_SIZE};
	auto const groups_y{(height + GROUP_SIZE - 1) / GROUP_SIZE};
	// horizon search
	vk14::barriers(commands, {vk14::use_of(*targets.scene_depth, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access),
	                          vk14::use_of(*targets.ssao, VK_IMAGE_LAYOUT_GENERAL, compute_stage, VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT)});
	gtao_constants constants{};
	constants.inv_projection = inverseprojection;
	constants.params = glm::vec2{Global.gfx_postfx_ao_radius, projection[0][0] * 0.5f * static_cast<float>(width)};
	constants.clipw_row = glm::vec4{projection[0][3], projection[1][3], projection[2][3], projection[3][3]};
	constants.quality = glm::ivec2{Global.gfx_postfx_ao_slices, Global.gfx_postfx_ao_steps};
	auto const depthimage{vk14::image_descriptor(ctx.sampling.clamp_nearest, targets.scene_depth->view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)};
	scene.gtao_main.dispatch(commands, {depthimage, vk14::storage_image(targets.ssao->view())}, constants, groups_x, groups_y);
	// smoothing
	vk14::barriers(commands, {vk14::use_of(*targets.ssao, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access),
	                          vk14::use_of(*targets.ssao_blur, VK_IMAGE_LAYOUT_GENERAL, compute_stage, VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT)});
	scene.gtao_denoise.dispatch(commands, {depthimage, vk14::image_descriptor(ctx.sampling.clamp_nearest, targets.ssao->view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL), vk14::storage_image(targets.ssao_blur->view())},
	                            inverseprojection, groups_x, groups_y);
	vk14::barriers(commands, {vk14::use_of(*targets.ssao_blur, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access)});
	return true;
}

// the block of the scene of the colour pass set up to read the occlusion (light_common.glsl, ambient_occlusion())
void vulkan14_renderer::setup_ao_lighting()
{
	scene_ubs.ao_params = glm::vec4{1.f, Global.gfx_postfx_ao_strength, 0.f, 0.f};
	upload_scene_block();
}

// the ambient occlusion of the viewport from the depth of its scene (resolved), multiplied into its colour
// (opengl46_renderer::Render_ambient_occlusion()). Mode: Global.gfx_postfx_ao. outside a rendering
void vulkan14_renderer::render_ao(std::size_t const Index, int const Mode)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &targets{scene.targets[Index]};
	auto const commands{ctx.commands};
	if ((false == bool(targets.ssao)) || (false == bool(targets.ssao_blur)))
	{
		return;
	}
	// the projection the depth was drawn with
	auto const &projection{m_colorpass.pass_camera.projection()};
	auto const inverseprojection{glm::inverse(projection)};
	auto const &depth{*targets.depth};
	if (Mode == 1)
	{
		// the older ssao, which reads the projection from its constants
		vk14::draw_constants constants;
		for (int column = 0; column < 4; ++column)
		{
			constants.data[column] = projection[column];
		}
		post_pass("postfx_ssao.frag", *targets.ssao, {targets.depth.get(), scene.ssao_noise.get()}, constants, nullptr, 2u);
		post_pass("postfx_ssao_blur.frag", *targets.ssao_blur, {targets.ssao.get()}, vk14::draw_constants{});
	}
	else
	{
		auto const width{targets.ssao->desc().extent.width};
		auto const height{targets.ssao->desc().extent.height};
		auto const groups_x{(width + GROUP_SIZE - 1) / GROUP_SIZE};
		auto const groups_y{(height + GROUP_SIZE - 1) / GROUP_SIZE};
		// horizon search
		vk14::barriers(commands, {vk14::use_of(*targets.depth, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access),
		                          vk14::use_of(*targets.ssao, VK_IMAGE_LAYOUT_GENERAL, compute_stage, VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT)});
		gtao_constants constants{};
		constants.inv_projection = inverseprojection;
		constants.params = glm::vec2{Global.gfx_postfx_ao_radius, projection[0][0] * 0.5f * static_cast<float>(width)};
		constants.clipw_row = glm::vec4{projection[0][3], projection[1][3], projection[2][3], projection[3][3]};
		constants.quality = glm::ivec2{Global.gfx_postfx_ao_slices, Global.gfx_postfx_ao_steps};
		auto const depthimage{vk14::image_descriptor(ctx.sampling.clamp_nearest, depth.view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)};
		scene.gtao_main.dispatch(commands, {depthimage, vk14::storage_image(targets.ssao->view())}, constants, groups_x, groups_y);
		// smoothing
		vk14::barriers(commands, {vk14::use_of(*targets.ssao, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access),
		                          vk14::use_of(*targets.ssao_blur, VK_IMAGE_LAYOUT_GENERAL, compute_stage, VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT)});
		scene.gtao_denoise.dispatch(commands, {depthimage, vk14::image_descriptor(ctx.sampling.clamp_nearest, targets.ssao->view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL), vk14::storage_image(targets.ssao_blur->view())},
		                            inverseprojection, groups_x, groups_y);
	}
	// result = image * occlusion, drawn over the colour of the scene with the pass set and the weather of its colour pass
	vk14::barriers(commands, {vk14::use_of(*targets.ssao_blur, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access), vk14::use_of(*targets.depth, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, sampled_access),
	                          vk14::use_of(*targets.colour, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT)});
	VkRenderingAttachmentInfo attachment{};
	attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
	attachment.imageView = targets.colour->view();
	attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
	attachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
	attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	VkRenderingInfo rendering{};
	rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
	rendering.renderArea = {{0, 0}, targets.extent};
	rendering.layerCount = 1;
	rendering.colorAttachmentCount = 1;
	rendering.pColorAttachments = &attachment;
	commands->begin_rendering(&rendering);
	auto &draw{*m_draw};
	auto const set{m_viewports[Index].main ? vk14::scene_state::set_colour : vk14::scene_state::set_plain};
	start_draws(targets.post_formats, static_cast<int>(targets.extent.width), static_cast<int>(targets.extent.height), set, false);
	draw.glare = true; // no depth
	draw.cullface = false;
	if (bind_scene_pipeline(*scene.ao_apply, targets.post_formats, vk14::blend_mode::multiply, VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP))
	{
		apply_draw_state();
		draw.block = push_draw_block();
		if (draw.block != ~0u)
		{
			bind_pass_set();
			std::array<VkDescriptorImageInfo, gl::MAX_TEXTURES> images;
			images.fill(VkDescriptorImageInfo{ctx.sampling.clamp_nearest, scene.white->view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL});
			images[0] = VkDescriptorImageInfo{ctx.sampling.clamp_nearest, depth.view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
			images[1] = VkDescriptorImageInfo{ctx.sampling.clamp_nearest, targets.ssao_blur->view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
			scene.material_set.push(commands, scene.scene_layout, images.data());
			vk14::draw_constants constants;
			constants.draw = draw.block;
			for (int column = 0; column < 4; ++column)
			{
				constants.data[column] = inverseprojection[column];
			}
			constants.data[4] = glm::vec4{(Mode == 1 ? 1.f : 0.f), Global.gfx_postfx_ao_strength, Global.gfx_postfx_ao_bounce, 0.f};
			push_constants(&constants);
			commands->draw(4, 1, 0, 0);
		}
	}
	commands->end_rendering();
	end_draws();
}
