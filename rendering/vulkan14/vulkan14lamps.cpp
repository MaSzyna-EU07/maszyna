/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the shadow maps of the lamps in the vulkan renderer (the stage P4 of the plan; opengl46_renderer::Render_local_shadows() and the
// rest): the lamp of the occupied cab, the lamps of the scenery and the headlights of the vehicles, which gfx::local_shadows picks,
// drawn by the shadow pass of the shared walk of the scene to the maps of vk14::local_shadow_maps

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14context.h"
#include "rendering/vulkan14/vulkan14commands.h"
#include "model/AnimModel.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "vehicle/DynObj.h"

namespace
{

VkPipelineStageFlags2 const depth_stage{VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT};
VkAccessFlags2 const depth_access{VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT};
VkPipelineStageFlags2 const sampled_stage{VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT};

} // namespace

// the rendering of the map the maps set up last (begin(), begin_lamp(), begin_headlight()), and the pass of the walk into it
void vulkan14_renderer::begin_local_shadow_rendering()
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto const &target{scene.lamps.target()};
	auto const commands{ctx.commands};
	if ((target.map == nullptr) || (target.view == VK_NULL_HANDLE))
	{
		return;
	}
	auto const use{vk14::use_of(*target.map, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, depth_stage, depth_access)};
	vk14::barriers(commands, {use});
	VkRenderingAttachmentInfo depth{};
	depth.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
	depth.imageView = target.view;
	depth.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
	depth.loadOp = (target.clear ? VK_ATTACHMENT_LOAD_OP_CLEAR : VK_ATTACHMENT_LOAD_OP_LOAD);
	depth.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	// the farthest depth
	depth.clearValue.depthStencil = {0.f, 0};
	VkRenderingInfo rendering{};
	rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
	rendering.renderArea = {{0, 0}, {target.size, target.size}};
	rendering.layerCount = 1;
	rendering.viewMask = target.view_mask;
	rendering.pDepthAttachment = &depth;
	commands->begin_rendering(&rendering);
	remember_rendering(rendering, {use});
	// the cube maps: one sided walls (of cabs, of buildings) face the lamp, they are to cast shadows as well. the maps of the
	// headlights: the back faces, as in the shadows of the sun
	start_draws(target.cube ? scene.cube_formats : scene.shadow_formats, static_cast<int>(target.size), static_cast<int>(target.size), vk14::scene_state::set_plain, (false == target.cube));
	m_draw->cullface = (false == target.cube);
	walk_blending(false);
	upload_scene_block();
	upload_light_header();
}

void vulkan14_renderer::end_local_shadow_rendering()
{
	if (m_draw->rendering)
	{
		m_context->commands->end_rendering();
	}
	end_draws();
}

// the maps in the layout the shading reads them in: the ones made or drawn since
void vulkan14_renderer::ready_lamp_maps()
{
	auto &lamps{m_context->scene.lamps};
	std::vector<vk14::image_use> uses;
	for (auto *map : {lamps.cab_map(), lamps.lamp_maps(), lamps.headlight_maps()})
	{
		if (map != nullptr)
		{
			uses.push_back(vk14::use_of(*map, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, sampled_stage, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT));
		}
	}
	if (false == uses.empty())
	{
		vk14::barriers(m_context->commands, uses);
	}
}

void vulkan14_renderer::render_local_shadows()
{
	auto &lamps{m_context->scene.lamps};
	// only the light buffers with the lamps read the maps
	auto const *vehicle{Global.gfx_lights_clustered ? m_locallights.cab_lamp_vehicle() : nullptr};
	if (false == lamps.update_wanted(vehicle, (vehicle != nullptr ? vehicle->mdKabina : nullptr), m_locallights.cab_lamp_local(), m_framestamp))
	{
		return;
	}
	m_context->passes.begin_pass("lamp shadows");
	// the cab drawn by the shadow pass, with the programs of the cube map. the camera is put at the vehicle, which Render_cab()
	// places the cab relative to, and the modelview takes the cab model to the space of the map
	m_renderpass.draw_mode = rendermode::shadows;
	m_renderpass.pass_camera.position() = vehicle->GetPosition();
	m_renderpass.draw_stats = {};
	OpenGLMatrices.mode(GL_MODELVIEW);
	OpenGLMatrices.load_matrix(glm::mat4{glm::translate(glm::dmat4{1.0}, -glm::dvec3{lamps.lamp()}) * glm::inverse(vehicle->mMatrix)});
	lamps.begin();
	begin_local_shadow_rendering();
	m_cubeshadows = true;
	Render_cab(vehicle, 0.0f, false);
	m_cubeshadows = false;
	end_local_shadow_rendering();
	lamps.end(static_cast<std::uint32_t>(m_renderpass.draw_stats.drawcalls));
	m_renderpass = m_colorpass;
	m_context->passes.end_pass();
}

void vulkan14_renderer::render_lamp_shadows()
{
	auto &lamps{m_context->scene.lamps};
	m_lampscenerydrawn = 0;
	auto const &slots{lamps.lamps()};
	if (slots.empty() || (false == Global.gfx_lights_clustered))
	{
		return;
	}
	// vehicles near the camera cast the shadows of the lamps, and the occupied cab
	lamps.lamp_vehicles(Global.pCamera.Pos);
	auto const vehiclesrange{static_cast<double>(Global.gfx_lights_shadows_vehicles_range)};
	auto *cab{vehiclesrange > 0.0 ? lamps.shadow_cab() : nullptr};

	auto timing{false};
	for (auto const idx : slots)
	{
		auto const &slot{lamps.slot(idx)};
		if (lamps.scenery_wanted(idx))
		{
			if (false == timing)
			{
				m_context->passes.begin_pass("scenery lamp shadows");
				lamps.lamps_timer_begin();
				timing = true;
			}
			render_lamp_shadow(idx, false, nullptr);
		}
		if (false == slot.scenery)
		{
			continue;
		}
		// the vehicles by the lamp, within its reach
		auto const &slotvehicles{lamps.slot_vehicles(slot)};
		auto *slotcab{(cab != nullptr) && (glm::length(cab->GetPosition() - slot.position) <= slot.range + cab->radius()) ? cab : nullptr};
		if (slotvehicles.empty() && (slotcab == nullptr))
		{
			lamps.no_vehicles(idx);
			continue;
		}
		if (false == lamps.vehicles_wanted(idx, m_framestamp))
		{
			continue;
		}
		if (false == timing)
		{
			m_context->passes.begin_pass("scenery lamp shadows");
			lamps.lamps_timer_begin();
			timing = true;
		}
		render_lamp_shadow(idx, true, slotcab);
	}
	if (timing)
	{
		lamps.lamps_timer_end();
		m_context->passes.end_pass();
	}
}

void vulkan14_renderer::render_lamp_shadow(std::size_t const Slot, bool const Vehicles, TDynamicObject *Cab)
{
	auto &lamps{m_context->scene.lamps};
	auto const &slot{lamps.slot(Slot)};
	// the shadow pass from the lamp, in world axes: the camera of the pass is the lamp, its frustum the box of its reach. the
	// distances which pick the details of the models are measured from the real camera, as in the shadows of the sun
	m_renderpass = m_colorpass;
	m_renderpass.draw_mode = rendermode::shadows;
	m_renderpass.draw_range = slot.range;
	m_renderpass.draw_stats = {};
	auto &camera{m_renderpass.pass_camera};
	camera.position() = slot.position;
	camera.modelview() = glm::mat4{glm::translate(glm::dmat4{1.0}, -slot.position)};
	auto const reach{slot.range};
	camera.projection() = glm::ortho(-reach, reach, -reach, reach, -reach, reach);
	camera.update_frustum();
	OpenGLMatrices.mode(GL_MODELVIEW);
	OpenGLMatrices.load_matrix(glm::mat4{1.f});

	lamps.begin_lamp(Slot, Vehicles, m_instancerecordsversion, m_framestamp);
	begin_local_shadow_rendering();
	m_cubeshadows = true;
	if (Vehicles)
	{
		for (auto *vehicle : lamps.slot_vehicles())
		{
			Render(vehicle);
		}
		if (Cab != nullptr)
		{
			Render_cab(Cab, 0.0f, false);
		}
	}
	else
	{
		m_cubescenery = true;
		m_cubeskipinstance = static_cast<TAnimModel const *>(slot.owner);
		// the instances culled on the gpu, each scenery part drawn in a frame with a culling of its own
		m_lightshadowcull = light_shadow_cull(m_context->scene.cull.lamps, m_lampscenerydrawn++, Global.gfx_gpucull_shadows_lamps);
		Render(simulation::Region);
		m_lightshadowcull = nullptr;
		m_cubeskipinstance = nullptr;
		m_cubescenery = false;
	}
	m_cubeshadows = false;
	end_local_shadow_rendering();
	lamps.end_lamp(static_cast<std::uint32_t>(m_renderpass.draw_stats.drawcalls));
	m_renderpass = m_colorpass;
}

void vulkan14_renderer::render_headlight_shadows()
{
	auto &lamps{m_context->scene.lamps};
	// the headlights which shine, the most important first (in the order of update_lights()), whose beam may be seen within the
	// reach of the shadows
	auto const &picked{lamps.pick_headlights(simulation::Lights, m_colorpass.pass_camera)};
	lamps.update_headlights(picked, m_framestamp);
	auto const &slots{lamps.headlights()};
	if (slots.empty())
	{
		return;
	}
	// the occupied cab, as in render_lamp_shadows()
	auto *cab{lamps.shadow_cab()};
	auto timing{false};
	for (auto const idx : slots)
	{
		auto const &slot{lamps.headlight(idx)};
		light_array::light_record const *light{nullptr};
		for (auto const *candidate : picked)
		{
			if ((candidate->owner == slot.owner) && (candidate->index == slot.index))
			{
				light = candidate;
				break;
			}
		}
		if (light == nullptr)
		{
			continue;
		}
		glm::dvec3 projector;
		glm::mat3 rotation;
		glm::mat4 projection;
		float nearplane, fieldofview;
		headlight_projector(*light, projector, rotation, projection, nearplane, fieldofview);
		// a vehicle on the move needs its map anew, the things around it and the vehicles passing by only now and then
		auto const moved{(glm::length(projector - slot.projector) > 0.01) || (glm::dot(light->direction, slot.direction) < 0.99999f)};
		if (false == lamps.headlight_wanted(idx, moved, m_framestamp))
		{
			continue;
		}
		if (false == timing)
		{
			m_context->passes.begin_pass("headlight shadows");
			lamps.headlights_timer_begin();
			timing = true;
		}
		render_headlight_shadow(idx, *light, cab);
	}
	if (timing)
	{
		lamps.headlights_timer_end();
		m_context->passes.end_pass();
	}
}

void vulkan14_renderer::render_headlight_shadow(std::size_t const Slot, light_array::light_record const &Light, TDynamicObject *Cab)
{
	auto &lamps{m_context->scene.lamps};
	glm::dvec3 projector;
	glm::mat3 rotation;
	glm::mat4 projection;
	float nearplane, fieldofview;
	headlight_projector(Light, projector, rotation, projection, nearplane, fieldofview);
	auto const reach{nearplane + Global.gfx_lights_shadows_headlights_range};
	// the shadow pass from the projector of the beam, with its projection. the distances which pick the details of the models
	// are measured from the real camera, as in the shadows of the sun
	m_renderpass = m_colorpass;
	m_renderpass.draw_mode = rendermode::shadows;
	m_renderpass.draw_range = reach;
	m_renderpass.draw_stats = {};
	auto &camera{m_renderpass.pass_camera};
	camera.position() = projector;
	camera.modelview() = glm::mat4{glm::dmat4{glm::dmat3{rotation}} * glm::translate(glm::dmat4{1.0}, -projector)};
	camera.projection() = projection;
	// the projection goes on to the infinity, what the shadows are cast by ends at their reach
	camera.update_frustum(glm::perspective(fieldofview, 1.f, nearplane, reach));
	setup_matrices();
	auto const colorprojection{scene_ubs.projection};
	scene_ubs.projection = projection;

	lamps.begin_headlight(Slot, projector, rotation, projection, Light.direction, m_framestamp);
	begin_local_shadow_rendering();
	m_shadowskipvehicle = Light.owner;
	// the instances culled on the gpu, each slot with a culling of its own
	m_lightshadowcull = light_shadow_cull(m_context->scene.cull.headlights, Slot, Global.gfx_gpucull_shadows_headlights);
	Render(simulation::Region);
	m_lightshadowcull = nullptr;
	if ((Cab != nullptr) && (Cab != Light.owner) && (glm::length(Cab->GetPosition() - projector) <= reach + Cab->radius()))
	{
		Render_cab(Cab, 0.0f, false);
	}
	m_shadowskipvehicle = nullptr;
	end_local_shadow_rendering();
	lamps.end_headlight(static_cast<std::uint32_t>(m_renderpass.draw_stats.drawcalls));
	scene_ubs.projection = colorprojection;
	m_renderpass = m_colorpass;
}

vk14::instance_cull *vulkan14_renderer::light_shadow_cull(std::vector<std::unique_ptr<vk14::instance_cull>> &Culls, std::size_t const Index, bool const Enabled)
{
	auto &ctx{*m_context};
	auto &cull{ctx.scene.cull};
	// the same conditions as those of the other passes in walk_region_begin()
	if ((Global.gfx_gpucull == 0) || (false == Enabled) || (m_current_viewport == nullptr) || (false == m_current_viewport->main) || (false == cull.ready))
	{
		return nullptr;
	}
	if (Culls.size() <= Index)
	{
		Culls.resize(Index + 1);
	}
	if (!Culls[Index])
	{
		// the buffers of its own, the records and the kernels of the main view
		auto culling{std::make_unique<vk14::instance_cull>()};
		vk14::cull_environment environment;
		if (false == culling->init(environment, ctx.device.handle(), ctx.pipelines, ctx.shaders, ctx.scene.pass_layout, &cull.main))
		{
			culling->destroy(ctx.device.handle());
			return nullptr;
		}
		Culls[Index] = std::move(culling);
	}
	return Culls[Index].get();
}
