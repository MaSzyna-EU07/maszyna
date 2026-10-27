/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the instances drawn from the gpu culling in the vulkan renderer (the stage P5 of the plan; opengl46_renderer::Render_Instances_Indirect()
// and the rest): the cullings of the passes picked as the opengl renderer picks them, the rendering of the pass ended for the compute
// passes and begun again where it was, the walks of the culled groups shared with the opengl renderer (gfx::scene_walk), and the indirect
// draws of the recorded draws, with multi draws where they follow one another with the same state

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14context.h"
#include "rendering/vulkan14/vulkan14commands.h"
#include "rendering/common/walksync.h"
#include "model/AnimModel.h"
#include "model/Model3d.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"

#include <chrono>

namespace
{

VkPipelineStageFlags2 const compute_stage{VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT};

} // namespace

// the cullings

bool vulkan14_renderer::init_cull()
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &cull{scene.cull};
	auto const device{ctx.device.handle()};
	vk14::cull_environment environment;
	environment.memory = &ctx.memory;
	environment.uploads = &ctx.uploads;
	environment.deletions = &ctx.deletions;
	environment.ring = &ctx.ring;
	environment.uniform_alignment = ctx.device.caps().limits.minUniformBufferOffsetAlignment;
	environment.storage_alignment = ctx.device.caps().limits.minStorageBufferOffsetAlignment;
	environment.frames = static_cast<std::uint32_t>(ctx.frames.size());
	environment.no_pyramid = vk14::image_descriptor(ctx.sampling.clamp_nearest, scene.depth_empty->view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
	if (false == cull.main.init(environment, device, ctx.pipelines, ctx.shaders, scene.pass_layout))
	{
		return false;
	}
	for (auto &cascade : cull.cascades)
	{
		if (false == cascade.init(environment, device, ctx.pipelines, ctx.shaders, scene.pass_layout, &cull.main))
		{
			return false;
		}
	}
	if (false == cull.hiz.init(device, ctx.pipelines, ctx.shaders, scene.pass_layout))
	{
		return false;
	}
	cull.main.begin_frame(ctx.slot, ctx.timelinevalue + 1, ctx.completed());
	cull.ready = true;
	return true;
}

void vulkan14_renderer::destroy_cull()
{
	if (!m_context)
	{
		return;
	}
	auto &cull{m_context->scene.cull};
	auto const device{m_context->device.handle()};
	// the ones which share the kernels of the main view first
	for (auto &cascade : cull.cascades)
	{
		cascade.destroy(device);
	}
	for (auto *culls : {&cull.headlights, &cull.lamps})
	{
		for (auto &culling : *culls)
		{
			if (culling)
			{
				culling->destroy(device);
			}
		}
		culls->clear();
	}
	cull.main.destroy(device);
	cull.hiz.destroy(device);
	cull.shadowfit.destroy(device);
	cull.ready = false;
}

vk14::instance_cull &vulkan14_renderer::pass_cull()
{
	return static_cast<vk14::instance_cull &>(*m_passcull);
}

// the gpu culling runs along the first color pass of the main viewport of a frame, and the passes of the cascades of the sun
// shadows and of the faces of the environment map drawn for it, each with a culling of its own (opengl46_renderer::walk_region_begin())
// the cullings made the first time they're wanted, by the main renderer (before it gives the passes to the walkers)
void vulkan14_renderer::ensure_cull()
{
	auto &cull{m_context->scene.cull};
	static bool tried{false};
	if ((m_main == nullptr) && (Global.gfx_gpucull != 0) && (false == cull.ready) && (false == tried))
	{
		tried = true;
		if (false == init_cull())
		{
			ErrorLog("vulkan: the gpu culling of the instances can't be used, the cpu culls them (gfx.gpucull)");
			destroy_cull();
		}
	}
}

void vulkan14_renderer::walk_region_begin()
{
	vk14::instance_cull *passcull{nullptr};
	auto &cull{m_context->scene.cull};
	if ((Global.gfx_gpucull != 0) && (m_current_viewport != nullptr) && (m_current_viewport->main) && m_draw->rendering)
	{
		ensure_cull();
		if (cull.ready)
		{
			if ((m_renderpass.draw_mode == rendermode::color) && (m_instancecullframe != m_framestamp))
			{
				m_instancecullframe = m_framestamp;
				passcull = &cull.main;
			}
			else if ((m_renderpass.draw_mode == rendermode::shadows) && (m_lightshadowcull != nullptr))
			{
				// the map of a set of headlights or the scenery part of the map of a lamp, which picked its culling
				passcull = m_lightshadowcull;
				m_shadowcasters.clear();
			}
			else if ((m_renderpass.draw_mode == rendermode::shadows) && Global.gfx_gpucull_shadows && (m_shadowcascade >= 0) && (m_shadowcascade < static_cast<int>(cull.cascades.size())) &&
			         (false == m_cubeshadows))
			{
				// the cascades of the sun
				passcull = &cull.cascades[m_shadowcascade];
				m_shadowcasters.clear();
			}
		}
	}
	Begin_Pass_Cull(passcull, (passcull == &cull.main));
}

bool vulkan14_renderer::walk_instances_begin()
{
	if (m_indirectcolourpending)
	{
		// the instances culled and recorded by the depth pass of the whole scene, the buckets weren't queued for the cpu
		render_instances_indirect_pending();
	}
	if (m_instancecullpass)
	{
		if (m_passcull->drawing())
		{
			// the buckets of the pass weren't queued for the cpu, which leaves the loop of the walk empty
			if (m_passcull == &m_context->scene.cull.main)
			{
				render_instances_indirect();
			}
			else
			{
				render_instances_indirect_pass();
			}
		}
		else
		{
			// the view of the pass is in place here, the one Render_Instanced() takes
			suspend_rendering();
			pass_cull().dispatch(m_commands, OpenGLMatrices.data(GL_MODELVIEW));
			resume_rendering();
			// what the cpu drew of the sleepers and the models drawn on their own
			Compare_Pass_Cull();
		}
	}
	return m_instancecullpass;
}

void vulkan14_renderer::walk_instances_end()
{
	End_Pass_Cull();
}

// the rendering ended and begun again around the compute passes

void vulkan14_renderer::remember_rendering(VkRenderingInfo const &Info, std::vector<vk14::image_use> const &Uses)
{
	auto &resume{m_draw->resume};
	resume.info = Info;
	resume.info.pNext = nullptr;
	resume.info.flags = 0;
	auto const colours{std::min<std::uint32_t>(Info.colorAttachmentCount, static_cast<std::uint32_t>(resume.colours.size()))};
	for (std::uint32_t i = 0; i < colours; ++i)
	{
		resume.colours[i] = Info.pColorAttachments[i];
		// what was drawn is drawn over
		resume.colours[i].loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
	}
	resume.info.colorAttachmentCount = colours;
	resume.info.pColorAttachments = (colours > 0 ? resume.colours.data() : nullptr);
	resume.has_depth = (Info.pDepthAttachment != nullptr);
	if (resume.has_depth)
	{
		resume.depth = *Info.pDepthAttachment;
		resume.depth.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
	}
	resume.info.pDepthAttachment = (resume.has_depth ? &resume.depth : nullptr);
	resume.info.pStencilAttachment = nullptr;
	resume.uses = Uses;
	resume.valid = true;
}

void vulkan14_renderer::suspend_rendering()
{
	auto &draw{*m_draw};
	if (false == draw.rendering)
	{
		return;
	}
	m_commands->end_rendering();
	draw.rendering = false;
}

void vulkan14_renderer::resume_rendering()
{
	auto &draw{*m_draw};
	auto &resume{draw.resume};
	if (draw.rendering || (false == resume.valid) || (draw.targets == nullptr))
	{
		return;
	}
	auto const commands{m_commands};
	// what the rendering stored before is loaded again. a walker leaves the states of the images the main thread tracks as they were:
	// the rendering left them as it found them
	if (m_main != nullptr)
	{
		vk14::barriers_again(commands, resume.uses);
	}
	else
	{
		vk14::barriers(commands, resume.uses);
	}
	commands->begin_rendering(&resume.info);
	// the dynamic state starts anew with the rendering, the state of the walk goes on
	auto const set{draw.set};
	auto const shadows{draw.shadows};
	auto const blending{draw.blending};
	auto const cullface{draw.cullface};
	auto const glare{draw.glare};
	auto const depthmask_off{draw.depthmask_off};
	auto *program{draw.program};
	auto const textures{draw.textures};
	start_draws(*draw.targets, static_cast<int>(draw.extent.width), static_cast<int>(draw.extent.height), set, shadows);
	draw.blending = blending;
	draw.cullface = cullface;
	draw.glare = glare;
	draw.depthmask_off = depthmask_off;
	draw.program = program;
	draw.textures = textures;
	if (draw.depthpass)
	{
		// the bias of the depth pass of the scene (render_scene_depth())
		commands->set_depth_bias_enable(VK_TRUE);
		commands->set_depth_bias(-1.f, 0.f, -1.f);
	}
}

// the draws

vk14::scene_program *vulkan14_renderer::multidraw_program(vk14::scene_program &Program)
{
	// made by one of the threads walking the passes (M2)
	std::lock_guard<std::mutex> lock(Program.mutex);
	if (Program.multidraw_tried)
	{
		return Program.multidraw.get();
	}
	Program.multidraw_tried = true;
	if (Program.vertex_name.empty() || Program.fragment_name.empty() || (Program.vertices != vk14::vertex_kind::scene))
	{
		return nullptr;
	}
	auto &ctx{*m_context};
	try
	{
		auto variant{std::make_shared<vk14::scene_program>()};
		variant->defines = Program.defines + "#define MODEL_DRAWS\n";
		variant->vertex = ctx.shaders.get(Program.vertex_name, variant->defines);
		variant->fragment = ctx.shaders.get(Program.fragment_name, variant->defines);
		variant->vertex_name = Program.vertex_name;
		variant->fragment_name = Program.fragment_name;
		variant->vertices = Program.vertices;
		variant->name = Program.name + " (multi draws)";
		variant->multidraw_tried = true;
		Program.multidraw = std::move(variant);
	}
	catch (gfx::shader_exception const &Exception)
	{
		vk14::log_error(Exception.what());
	}
	return Program.multidraw.get();
}

bool vulkan14_renderer::draw_indirect(vk14::instance_cull &Cull, command_set const Set, std::size_t const First, std::uint32_t const Count, vk14::scene_program &Program, vk14::blend_mode const Blend)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &draw{*m_draw};
	auto const &first{m_indirectdraws[First]};
	if ((false == draw.rendering) || (Count == 0) || (Program.vertices != vk14::vertex_kind::scene) || (Cull.command_buffer() == VK_NULL_HANDLE))
	{
		return false;
	}
	auto const &geometry{first.geometry};
	if ((geometry.bank == 0) || (geometry.bank > m_banks.size()) || (m_banks[geometry.bank - 1] == nullptr))
	{
		return false;
	}
	// the buffers of the bank of the chunks, with their data on the gpu (the chunks of a multi draw are of one bank)
	vk14::chunk_source chunk;
	if (false == chunk_of(geometry, chunk))
	{
		return false;
	}
	if (false == bind_scene_pipeline(Program, *draw.targets, Blend, static_cast<int>(chunk.topology)))
	{
		return false;
	}
	apply_draw_state();
	// the blocks of the draws, one after another in the window of the blocks: the multi draw picks them by the number of the draw
	std::uint32_t block{~0u};
	if (Count == 1)
	{
		// the block of the recorded draw; the walk goes on with its own model block. what a recorded draw leaves in it (the
		// emission of a lit lamp) would light the tracks and the shapes the walk draws next
		auto const walkubs{model_ubs};
		model_ubs = first.ubs;
		block = push_draw_block();
		model_ubs = walkubs;
	}
	else
	{
		auto &draws{scene.draws};
		auto &window{draw.draws_window};
		auto const size{static_cast<VkDeviceSize>(Count) * draws.stride};
		auto slice{(window.open && (size <= draws.window)) ? draw.draws_cache.allocate_before(size, draws.stride, window.base + draws.window) : vk14::ring_slice{}};
		if ((false == bool(slice)) && (size <= draws.window))
		{
			slice = draw.draws_cache.allocate(size, draws.stride);
			if (slice)
			{
				window.base = slice.offset;
				window.open = true;
				draw.set_dirty = true;
			}
		}
		if (slice)
		{
			for (std::uint32_t i = 0; i < Count; ++i)
			{
				std::memcpy(slice.data + i * draws.stride, &m_indirectdraws[First + i].ubs, sizeof(gl::model_ubs));
			}
			block = static_cast<std::uint32_t>((slice.offset - window.base) / draws.stride);
		}
	}
	// the next draw of the walk pushes a block of its own
	draw.block = ~0u;
	if (block == ~0u)
	{
		return false;
	}
	if (draw.set_dirty)
	{
		bind_pass_set();
	}
	if (draw.textures_dirty)
	{
		push_textures();
	}
	auto const commands{draw.commands};
	if (draw.pushed_draw != block)
	{
		vk14::draw_constants constants;
		constants.draw = block;
		VkPushConstantsInfo info{};
		info.sType = VK_STRUCTURE_TYPE_PUSH_CONSTANTS_INFO;
		info.layout = scene.scene_layout;
		info.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
		info.offset = 0;
		info.size = sizeof(constants);
		info.pValues = &constants;
		commands->push_constants2(&info);
		draw.pushed_draw = block;
	}
	if ((chunk.vertices != draw.vertices) || (chunk.userdata != draw.userdata))
	{
		VkBuffer const buffers[2]{chunk.vertices, chunk.userdata};
		VkDeviceSize const offsets[2]{0, 0};
		commands->bind_vertex_buffers(0, 2, buffers, offsets);
		draw.vertices = chunk.vertices;
		draw.userdata = chunk.userdata;
	}
	auto const offset{static_cast<VkDeviceSize>(Cull.draw_command_offset(Set, First))};
	if (chunk.indices != VK_NULL_HANDLE)
	{
		if (chunk.indices != draw.indices)
		{
			commands->bind_index_buffer(chunk.indices, 0, VK_INDEX_TYPE_UINT32);
			draw.indices = chunk.indices;
		}
		commands->draw_indexed_indirect(Cull.command_buffer(), offset, Count, sizeof(gl::instance_draw_command));
	}
	else
	{
		commands->draw_indirect(Cull.command_buffer(), offset, Count, sizeof(gl::instance_draw_command));
	}
	draw.draws += Count;
	return true;
}

vulkan14_renderer::indirect_counts vulkan14_renderer::draw_indirect_colour(vk14::instance_cull &Cull, command_set const Set)
{
	auto &draw{*m_draw};
	auto const &scene{m_context->scene};
	// the draws were recorded with the seed of scenery models and sleepers, 0 (TAnimModel::RaPrepare()); vehicles drawn since
	// leave their own, which would pick other texture variants than those the draws were recorded and ordered with
	TSubModel::iVariantSeed = 0;
	indirect_counts counts;
	auto const count{m_indirectdraws.size()};
	auto const longest{static_cast<std::size_t>(scene.draws.window / scene.draws.stride)};
	draw.replaying = true;
	for (std::size_t index = 0; index < count;)
	{
		auto const blending{m_indirectdraws[index].blending};
		if (blending)
		{
			walk_blending(true);
		}
		walk_material(m_indirectdraws[index].material, m_indirectdraws[index].submodel);
		auto *program{draw.program};
		auto *multidraw{(m_indirectmultidraw && (program != nullptr)) ? multidraw_program(*program) : nullptr};
		auto last{index + 1};
		if (multidraw != nullptr)
		{
			// the draws which follow with the same state, in one call
			while ((last < count) && (last - index < longest) && Indirect_Mergeable(index, last))
			{
				++last;
			}
		}
		if (program != nullptr)
		{
			auto const drawcount{static_cast<std::uint32_t>(last - index)};
			if (draw_indirect(Cull, Set, index, drawcount, (multidraw != nullptr ? *multidraw : *program), (draw.blending ? vk14::blend_mode::scene_alpha : vk14::blend_mode::none)))
			{
				counts.draws += drawcount;
			}
			++counts.calls;
		}
		index = last;
		if (blending)
		{
			walk_blending(false);
		}
	}
	draw.replaying = false;
	if (count > 0)
	{
		// the model block the draws made one by one leave behind
		model_ubs = m_indirectdraws.back().ubs;
	}
	model_ubs.emission = 0.0f;
	return counts;
}

vulkan14_renderer::indirect_counts vulkan14_renderer::draw_indirect_depth(vk14::instance_cull &Cull, command_set const Set)
{
	auto &draw{*m_draw};
	auto &scene{m_context->scene};
	auto const commands{m_commands};
	// as in draw_indirect_colour(): the seed the draws were recorded with
	TSubModel::iVariantSeed = 0;
	indirect_counts counts;
	auto const count{m_indirectdraws.size()};
	auto const longest{static_cast<std::size_t>(scene.draws.window / scene.draws.stride)};
	// whether the prepass of the class draws the draw, and with which diffuse texture (class 2)
	auto const prepass_texture = [this](std::size_t const Index, int const Prepassclass, texture_handle &Texture) -> bool {
		auto const &entry{m_indirectdraws[Index]};
		if ((entry.blending) || (entry.ubs.opacity < 0.f))
		{
			return false;
		}
		if ((entry.type != GL_TRIANGLES) && (entry.type != GL_TRIANGLE_STRIP) && (entry.type != GL_TRIANGLE_FAN))
		{
			return false;
		}
		if (prepass_class(entry.material) != Prepassclass)
		{
			return false;
		}
		Texture = null_handle;
		if (Prepassclass == 2)
		{
			// the diffuse texture of the colour pass, the same choice as walk_material() makes
			auto const &material{m_materials.material(entry.material)};
			auto const &textures{material.texture_variants.empty() ? material.textures : material.GetTextures(TSubModel::VariantId(entry.submodel))};
			if (textures[0] == null_handle)
			{
				return false;
			}
			Texture = textures[0];
		}
		return true;
	};
	// inside the colour pass the depth goes a little farther than the surface, without the colour (the depth pass of the scene has
	// its own state)
	auto const insidecolour{false == draw.depthpass};
	if (insidecolour)
	{
		commands->set_depth_bias_enable(VK_TRUE);
		commands->set_depth_bias(-1.f, 0.f, -1.f);
	}
	walk_blending(false);
	// the draws without the alpha test first, then the ones with it, one program each
	for (auto prepassclass = 1; prepassclass <= 2; ++prepassclass)
	{
		auto &program{prepassclass == 1 ? *scene.prepass : *scene.prepass_alpha};
		auto *multidraw{m_indirectmultidraw ? multidraw_program(program) : nullptr};
		for (std::size_t index = 0; index < count;)
		{
			texture_handle texture;
			if (false == prepass_texture(index, prepassclass, texture))
			{
				++index;
				continue;
			}
			if (prepassclass == 2)
			{
				walk_texture(0, texture);
			}
			auto const &entry{m_indirectdraws[index]};
			auto last{index + 1};
			if (multidraw != nullptr)
			{
				// the draws which follow in the same bank and of the same primitive, with the same texture for the alpha test. the
				// material doesn't matter to the depth
				texture_handle othertexture;
				while ((last < count) && (last - index < longest) && (m_indirectdraws[last].group == entry.group) && (m_indirectdraws[last].type == entry.type) &&
				       (m_indirectcommands[last].indexed == m_indirectcommands[index].indexed) && (prepass_texture(last, prepassclass, othertexture)) && (othertexture == texture))
				{
					++last;
				}
			}
			auto const drawcount{static_cast<std::uint32_t>(last - index)};
			if (draw_indirect(Cull, Set, index, drawcount, (multidraw != nullptr ? *multidraw : program), (insidecolour ? vk14::blend_mode::no_colour : vk14::blend_mode::none)))
			{
				counts.draws += drawcount;
			}
			++counts.calls;
			index = last;
		}
	}
	if (insidecolour)
	{
		commands->set_depth_bias_enable(VK_FALSE);
	}
	return counts;
}

vulkan14_renderer::indirect_counts vulkan14_renderer::draw_indirect_shadow(vk14::instance_cull &Cull, command_set const Set)
{
	auto &scene{m_context->scene};
	indirect_counts counts;
	auto const count{m_indirectdraws.size()};
	auto const longest{static_cast<std::size_t>(scene.draws.window / scene.draws.stride)};
	for (std::size_t index = 0; index < count;)
	{
		auto const &entry{m_indirectdraws[index]};
		auto const texture{Indirect_Shadow_Texture(entry.material)};
		auto &program{m_cubeshadows ? (texture != null_handle ? *scene.cube_alpha_shadow : *scene.cube_shadow) : (texture != null_handle ? *scene.alpha_shadow : *scene.shadow)};
		auto *multidraw{m_indirectmultidraw ? multidraw_program(program) : nullptr};
		auto last{index + 1};
		if (multidraw != nullptr)
		{
			// the draws which follow in the same bank and of the same primitive, with the same texture for the alpha test
			while ((last < count) && (last - index < longest) && (m_indirectdraws[last].group == entry.group) && (m_indirectdraws[last].type == entry.type) &&
			       (m_indirectcommands[last].indexed == m_indirectcommands[index].indexed) && (Indirect_Shadow_Texture(m_indirectdraws[last].material) == texture))
			{
				++last;
			}
		}
		if (texture != null_handle)
		{
			walk_texture(0, texture);
		}
		auto const drawcount{static_cast<std::uint32_t>(last - index)};
		if (draw_indirect(Cull, Set, index, drawcount, (multidraw != nullptr ? *multidraw : program), vk14::blend_mode::none))
		{
			counts.draws += drawcount;
		}
		++counts.calls;
		index = last;
	}
	return counts;
}

void vulkan14_renderer::record_indirect_draws(vk14::instance_cull &Cull, bool const Cascade)
{
	m_indirectcull = &Cull;
	Walk_Indirect_Draws(Cull, Cascade);
	auto const orderstart{std::chrono::steady_clock::now()};
	// with multi draws the draws with the same state follow one another; the order goes into the commands, so it comes first
	m_indirectmultidraw = Global.gfx_gpucull_multidraw;
	if (m_indirectmultidraw && (false == m_indirectdraws.empty()))
	{
		Order_Indirect_Draws();
	}
	Cull.fill_commands(m_commands, m_indirectcommands);
	m_walkcounts.ordertime += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - orderstart).count();
}

// the instances of the main view: the depth pyramid of what the pass drew so far, the culling, the walks, the depth first (the
// depth pass of the scene, gfx.depthprepass), the colour; in two phases the instances seen in the previous frame first
void vulkan14_renderer::render_instances_indirect()
{
	auto const cpustart{std::chrono::steady_clock::now()};
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &cull{scene.cull.main};
	auto &draw{*m_draw};
	auto const commands{m_commands};
	auto const viewport{static_cast<std::size_t>(m_current_viewport - m_viewports.data())};
	auto *targets{viewport < scene.targets.size() ? &scene.targets[viewport] : nullptr};
	auto const projection{OpenGLMatrices.data(GL_PROJECTION)};

	suspend_rendering();
	// the occlusion test against the depth drawn so far in the pass: cab, terrain, shapes, tracks, vehicles, other models
	vk14::cull_occlusion occlusion;
	auto const build = [&]() {
		vk14::barriers(commands, {vk14::use_of(*targets->depth, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, compute_stage, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT)});
		if (scene.cull.hiz.build(commands, ctx.device.handle(), ctx.memory, ctx.deletions, ctx.uploads.current_value(), targets->depth->view(), ctx.sampling.clamp_nearest, static_cast<int>(targets->extent.width),
		                         static_cast<int>(targets->extent.height)))
		{
			occlusion = scene.cull.hiz.occlusion(ctx.sampling.clamp_nearest, projection);
		}
	};
	auto const useocclusion{Global.gfx_gpucull_occlusion && (false == Global.gfx_skippipeline) && (targets != nullptr) && targets->depth};
	if (useocclusion)
	{
		build();
	}
	// in two phases the instances seen in the previous frame are drawn first, and the others are tested against a pyramid made
	// again with their depth in it (not in wireframe, which doesn't draw surfaces)
	cull.dispatch(commands, OpenGLMatrices.data(GL_MODELVIEW), (occlusion.pyramid != VK_NULL_HANDLE ? &occlusion : nullptr), (Global.gfx_gpucull_occlusion_instances && (false == Global.bWireFrame)));
	// the draws, recorded while the gpu culls
	record_indirect_draws(cull, false);

	auto const twophase{cull.two_phase()};
	auto const draws{false == m_indirectdraws.empty()};
	// the depth comes first in the depth pass of the whole scene, which leaves the colour to the colour pass, and with the depth
	// prepass of the instances
	auto const depthfirst{draw.depthpass || (Global.gfx_depthprepass && (false == Global.bWireFrame))};
	indirect_counts prepasscounts;
	indirect_counts colourcounts;
	auto colourset{command_set::first};
	resume_rendering();
	if (twophase)
	{
		if (draws)
		{
			// the instances seen in the previous frame, then the pyramid again, with them in it
			use_cull_pass_sets(&cull);
			if (depthfirst)
			{
				prepasscounts += draw_indirect_depth(cull, command_set::first);
			}
			else
			{
				colourcounts += draw_indirect_colour(cull, command_set::first);
			}
			use_cull_pass_sets(nullptr);
		}
		suspend_rendering();
		if (draws)
		{
			build();
		}
		// the others are tested against it; with no draws the instances are only marked for the next frame
		cull.dispatch_second(commands, occlusion);
		resume_rendering();
		colourset = (depthfirst ? command_set::all : command_set::second);
	}
	if (draws)
	{
		use_cull_pass_sets(&cull);
		if (depthfirst)
		{
			prepasscounts += draw_indirect_depth(cull, (twophase ? command_set::second : command_set::first));
		}
		if (false == draw.depthpass)
		{
			colourcounts += draw_indirect_colour(cull, colourset);
		}
		use_cull_pass_sets(nullptr);
	}
	if (draw.depthpass)
	{
		// the colour pass takes the draws from here, and counts them
		m_indirectcolourpending = true;
		m_indirectcolourset = colourset;
		m_indirectcoloursleepers = m_sleeperspass;
		m_indirectcolourmodels = m_modelspass;
		m_renderpass.draw_stats.drawcalls += static_cast<int>(prepasscounts.calls);
		colourcounts.draws = static_cast<std::uint32_t>(m_indirectdraws.size());
	}
	else
	{
		m_renderpass.draw_stats.instanced_drawcalls += static_cast<int>(colourcounts.calls);
		m_renderpass.draw_stats.drawcalls += static_cast<int>(prepasscounts.calls);
		// the count of the frame the gpu reported last
		m_renderpass.draw_stats.instances += static_cast<int>(cull.last_count());
		m_renderpass.draw_stats.models += static_cast<int>(cull.last_count());
	}
	cull.draw_result(colourcounts.draws, prepasscounts.draws, colourcounts.calls, prepasscounts.calls, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - cpustart).count());
}

// the instances of a pass other than the main view: the culling of the pass with its own camera, no occlusion test, the draws
// recorded the same way, then drawn once with the programs the pass draws with on the cpu
void vulkan14_renderer::render_instances_indirect_pass()
{
	auto const cpustart{std::chrono::steady_clock::now()};
	auto &cull{pass_cull()};
	auto &cascades{m_context->scene.cull.cascades};
	suspend_rendering();
	cull.dispatch(m_commands, OpenGLMatrices.data(GL_MODELVIEW));
	record_indirect_draws(cull, (&cull >= cascades.data()) && (&cull < cascades.data() + cascades.size()));
	resume_rendering();

	indirect_counts counts;
	if (false == m_indirectdraws.empty())
	{
		use_cull_pass_sets(&cull);
		counts = (m_renderpass.draw_mode == rendermode::shadows ? draw_indirect_shadow(cull, command_set::first) : draw_indirect_colour(cull, command_set::first));
		use_cull_pass_sets(nullptr);
	}
	m_renderpass.draw_stats.instanced_drawcalls += static_cast<int>(counts.calls);
	// the count of the frame the gpu reported last
	m_renderpass.draw_stats.instances += static_cast<int>(cull.last_count());
	m_renderpass.draw_stats.models += static_cast<int>(cull.last_count());
	cull.draw_result(counts.draws, 0, counts.calls, 0, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - cpustart).count());
}

// the colour of the instances culled and recorded by the depth pass of the whole scene
void vulkan14_renderer::render_instances_indirect_pending()
{
	m_indirectcolourpending = false;
	auto &cull{m_context->scene.cull.main};
	indirect_counts colourcounts;
	if (false == m_indirectdraws.empty())
	{
		auto const cpustart{std::chrono::steady_clock::now()};
		use_cull_pass_sets(&cull);
		colourcounts = draw_indirect_colour(cull, m_indirectcolourset);
		use_cull_pass_sets(nullptr);
		cull.draw_result(colourcounts.draws, cull.prepass_draws(), colourcounts.calls, cull.prepass_calls(), cull.cpu_time() + std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - cpustart).count());
	}
	m_renderpass.draw_stats.instanced_drawcalls += static_cast<int>(colourcounts.calls);
	m_renderpass.draw_stats.instances += static_cast<int>(cull.last_count());
	m_renderpass.draw_stats.models += static_cast<int>(cull.last_count());
}

// the cascades of the sun shadows fitted to the depth of the main view (opengl46_renderer::Shadow_Fit())

char const *vulkan14_renderer::shadow_fit_unavailable(viewport_config const &Viewport) const
{
	auto const shadowviews{std::count_if(m_viewports.begin(), m_viewports.end(), [](viewport_config const &View) { return View.shadow; })};
	return gfx::shadow_fit::unavailable(false == Global.gfx_skippipeline, Viewport.shadow, static_cast<std::size_t>(shadowviews));
}

bool vulkan14_renderer::shadow_fit(viewport_config const &Viewport, std::array<float, gl::MAX_CASCADES + 1> &Splits, gfx::shadow_fit::areas &Areas)
{
	auto &ctx{*m_context};
	auto &cull{ctx.scene.cull};
	auto const *unavailable{Global.gfx_shadow_sdsm ? shadow_fit_unavailable(Viewport) : nullptr};
	if (Global.gfx_shadow_sdsm && (unavailable == nullptr))
	{
		if (cull.shadowfit)
		{
			// takes in what came back
			cull.shadowfit.latest(ctx.completed());
		}
		else
		{
			unavailable = "no compute passes";
		}
	}
	return cull.shadowfit.fit(m_framestamp, m_colorpass.pass_camera, unavailable, Splits, Areas, m_shadowfitstate);
}

void vulkan14_renderer::measure_shadow_fit(std::size_t const Viewport)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &cull{scene.cull};
	auto const &vp{m_viewports[Viewport]};
	if ((false == vp.main) || (false == Global.gfx_shadow_sdsm) || (shadow_fit_unavailable(vp) != nullptr) || (Viewport >= scene.targets.size()) || (false == bool(scene.targets[Viewport].depth)))
	{
		return;
	}
	if ((false == bool(cull.shadowfit)) && (false == cull.shadowfit_tried))
	{
		cull.shadowfit_tried = true;
		if (false == cull.shadowfit.init(ctx.device.handle(), ctx.pipelines, ctx.shaders, scene.pass_layout, ctx.memory, static_cast<std::uint32_t>(ctx.frames.size())))
		{
			ErrorLog("vulkan: the cascades of the sun shadows can't be fitted to the depth of the view (gfx.shadow.sdsm)");
			cull.shadowfit.destroy(ctx.device.handle());
		}
	}
	if (false == bool(cull.shadowfit))
	{
		return;
	}
	auto &targets{scene.targets[Viewport]};
	auto const commands{m_commands};
	vk14::barriers(commands, {vk14::use_of(*targets.depth, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, compute_stage, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT)});
	gfx::shadow_fit::view const view{m_colorpass.pass_camera.position(), m_colorpass.pass_camera.projection(), glm::mat3{m_colorpass.pass_camera.modelview()},
	                                 gfx::shadow_fit::light_rotation(m_sunlight.direction), gfx::shadow_fit::fixed_splits().back()};
	ctx.passes.begin_pass("sun shadows fitted to the depth");
	cull.shadowfit.measure(commands, targets.depth->view(), ctx.sampling.clamp_nearest, static_cast<int>(targets.extent.width), static_cast<int>(targets.extent.height), view, m_framestamp, ctx.slot, ctx.timelinevalue + 1);
	ctx.passes.end_pass();
}
