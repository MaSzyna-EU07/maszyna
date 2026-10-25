/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the tools of the vulkan renderer (the stage P7 of the plan): the debug windows of opengl46_renderer::draw_debug_ui() over the
// settings and the state shared with the opengl renderer, the lines of the statistics of the scene and of the cullings for
// the debug panel (Shift+F12), and the heatmap of the light clusters (gfx.lights.clusterheatmap)

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14renderer.h"
#include "rendering/vulkan14/vulkan14commands.h"
#include "rendering/vulkan14/vulkan14context.h"

#include "rendering/common/exposure.h"
#include "simulation/simulationenvironment.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"
#include "imgui/imgui.h"

namespace
{

VkPipelineStageFlags2 constexpr heatmap_colour_stage{VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT};
VkAccessFlags2 constexpr heatmap_colour_access{VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT};

} // namespace

void vulkan14_renderer::collect_cluster_stats()
{
	if (!m_context)
	{
		return;
	}
	auto &clusters{m_context->scene.clusters};
	gl::cluster_counters counters{};
	if (false == clusters.counters_readback.pop(m_context->completed(), &counters, sizeof(counters)))
	{
		return;
	}
	auto &stats{clusters.stats};
	stats.index_count = counters.index_count;
	stats.overflow_count = counters.overflow_count;
	stats.overflow_peak = std::max(counters.overflow_count, stats.valid ? stats.overflow_peak : 0u);
	stats.valid = true;
}

// the lines of the cullings of the passes (the part of opengl46_renderer::Update_Debug_Info() after the scene statistics)
std::string vulkan14_renderer::cull_info()
{
	std::string text;
	if (!m_context || (false == m_context->scene.cull.ready))
	{
		return text;
	}
	auto &cull{m_context->scene.cull};
	text += cull.main.info();
	if (Global.gfx_gpucull_shadows)
	{
		for (std::size_t cascade = 0; cascade < cull.cascades.size(); ++cascade)
		{
			if (false == cull.cascades[cascade].ran())
			{
				continue;
			}
			cull.cascades[cascade].collect();
			text += cull.cascades[cascade].info("gpu cull shadows " + std::to_string(cascade + 1), false);
		}
	}
	if (Global.gfx_gpucull != 0)
	{
		// the maps are drawn now and then, the numbers are of the last drawing of each
		auto const add = [&](std::vector<std::unique_ptr<vk14::instance_cull>> &Culls, bool const Enabled, std::string const &Label) {
			if (false == Enabled)
			{
				return;
			}
			for (std::size_t idx = 0; idx < Culls.size(); ++idx)
			{
				if (!Culls[idx] || (false == Culls[idx]->ran()))
				{
					continue;
				}
				Culls[idx]->collect();
				text += Culls[idx]->info("gpu cull " + Label + " " + std::to_string(idx + 1), false);
			}
		};
		add(cull.headlights, Global.gfx_gpucull_shadows_headlights, "headlight shadows");
		add(cull.lamps, Global.gfx_gpucull_shadows_lamps, "lamp shadows, scenery");
	}
	if (Global.gfx_gpucull == 1)
	{
		// all passes of the last frame
		auto const &walks{m_lastwalkcounts};
		text += "\nwalks of the submodel trees in the frame: " + std::to_string(walks.walked) + " made (" + std::to_string(walks.changing) + " not kept), " + std::to_string(walks.reused) +
		        " kept from earlier frames (" + std::to_string(m_walkrecords.size()) + " groups), cpu " + to_string(walks.walktime, 3) + " ms; the draws put in order and sent, cpu " +
		        to_string(walks.ordertime, 3) + " ms";
	}
	return text;
}

// the statistics of the scene (the part "Scene statistics" of opengl46_renderer::Update_Debug_Info())
std::string vulkan14_renderer::scene_info()
{
	auto &scene{m_context->scene};
	debug_stats shadowstats;
	for (auto const &shadowpass : m_shadowpass)
	{
		shadowstats += shadowpass.draw_stats;
	}
	auto const &colour{m_laststats};
	auto const line = [](char const *Name, int const Colour, int const Shadows) {
		return std::string(Name) + to_string(Colour, 7) + " +" + to_string(Shadows, 7) + " =" + to_string(Colour + Shadows, 7) + "\n";
	};
	// the cascades of the sun shadows: where they end and the size of a texel of their maps, fixed or fitted to the depth
	std::string cascades;
	if (Global.gfx_shadowmap_enabled && (scene.shadow_size > 0))
	{
		auto const &fit{m_shadowfitstate};
		auto const size{static_cast<float>(scene.shadow_size)};
		cascades = " sun shadow cascades to";
		for (std::size_t cascade = 0; cascade < gl::MAX_CASCADES; ++cascade)
		{
			cascades += " " + to_string(fit.splits[cascade + 1], 1);
		}
		cascades += " m, texels";
		for (std::size_t cascade = 0; cascade < gl::MAX_CASCADES; ++cascade)
		{
			cascades += " " + to_string(100.f * fit.drawn[cascade] / size, 1);
		}
		cascades += " cm";
		if (Global.gfx_shadow_sdsm)
		{
			if (fit.fitted)
			{
				cascades += " (whole slices";
				for (std::size_t cascade = 0; cascade < gl::MAX_CASCADES; ++cascade)
				{
					cascades += " " + to_string(100.f * fit.slice[cascade] / size, 1);
				}
				cascades += "), fitted to pixels " + to_string(fit.depthmin, 2) + "-" + to_string(fit.depthmax, 1) + " m " + std::to_string(fit.age) + " frames ago, view turned " + to_string(fit.turn, 1) +
				            " deg since";
			}
			else
			{
				cascades += ", fixed: " + std::string{fit.reason != nullptr ? fit.reason : ""};
			}
		}
		cascades += "\n";
	}
	std::string text{"triangles: " + to_string(static_cast<int>(m_geometry.primitives_count()), 7) + "\n" + line("vehicles:  ", colour.dynamics, shadowstats.dynamics) +
	                 line("models:    ", colour.models, shadowstats.models) + line("drawcalls: ", colour.drawcalls, shadowstats.drawcalls) + " instanced:" + to_string(colour.instances, 7) + " +" +
	                 to_string(shadowstats.instances, 7) + " =" + to_string(colour.instances + shadowstats.instances, 7) + " (" +
	                 std::to_string(colour.instanced_drawcalls + shadowstats.instanced_drawcalls) + " batches)\n"};
	if (Global.gfx_contactshadows > 0)
	{
		text += " contact shadows: " + std::string{Global.gfx_contactshadows == 3 ? "high" : Global.gfx_contactshadows == 2 ? "medium" : "low"} + ", rays " +
		        to_string(Global.gfx_contactshadows_length, 2) + " m within " + to_string(Global.gfx_contactshadows_range, 0) + " m\n";
	}
	text += cascades + line(" submodels:", colour.submodels, shadowstats.submodels) + line(" paths:    ", colour.paths, shadowstats.paths) +
	        line(" shapes:   ", colour.shapes, shadowstats.shapes) + " traction: " + to_string(colour.traction, 7) + "\n" + " lines:    " + to_string(colour.lines, 7) + "\n" +
	        "particles: " + to_string(colour.particles, 7) + "\nlights:    " + to_string(static_cast<int>(m_lights_lit), 7) + " (" + std::to_string(m_lights_dropped) + " over capacity), " +
	        (Global.gfx_lights_clustered ? "clustered" : "first " + std::to_string(std::max(Global.DynamicLightCount, 0))) + "\n" + m_locallights.info() + scene.lamps.info() +
	        scene.particles.info();
	auto const &stats{scene.clusters.stats};
	if (stats.valid && (Global.gfx_lights_clustered || Global.gfx_lights_clusterheatmap))
	{
		text += "cluster lists: " + std::to_string(stats.index_count) + ", overflow " + std::to_string(stats.overflow_count) + " (peak " + std::to_string(stats.overflow_peak) + ")\n";
	}
	if (scene.raytracing && m_raytracingon)
	{
		text += scene.raytracing->info() + "\n";
	}
	return text;
}

void vulkan14_renderer::draw_debug_ui()
{
	if ((false == m_debug_ui_active) || !m_context)
	{
		return;
	}
	auto &scene{m_context->scene};
	collect_cluster_stats();

	if (ImGui::Begin("Headlight config", &m_debug_ui_active))
	{
		ImGui::SetWindowSize(ImVec2(0, 0));
		auto &conf{m_headlightconfig};
		ImGui::SliderFloat("in_cutoff", &conf.in_cutoff, 0.9f, 1.1f);
		ImGui::SliderFloat("out_cutoff", &conf.out_cutoff, 0.9f, 1.1f);
		ImGui::SliderFloat("falloff_linear", &conf.falloff_linear, 0.0f, 1.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
		ImGui::SliderFloat("falloff_quadratic", &conf.falloff_quadratic, 0.0f, 1.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
		ImGui::SliderFloat("ambient", &conf.ambient, 0.0f, 3.0f);
		ImGui::SliderFloat("intensity", &conf.intensity, 0.0f, 10.0f);
	}
	ImGui::End();

	if (ImGui::Begin("Light clusters"))
	{
		ImGui::Checkbox("clustered lights (off: the first dynamiclights headlights)", &Global.gfx_lights_clustered);
		ImGui::Checkbox("heatmap", &Global.gfx_lights_clusterheatmap);
		ImGui::Checkbox("beams of vehicle headlights (gfx.lights.headlights)", &Global.gfx_lights_headlights);
		auto const &params{scene.clusters.params};
		ImGui::Text("grid %u x %u x %u, %s", gl::CLUSTER_GRID_X, gl::CLUSTER_GRID_Y, gl::CLUSTER_GRID_Z, params.valid ? "built" : "none (orthographic view)");
		ImGui::Text("near %.3f m, far %.1f m", params.znear, params.zfar);
		ImGui::Text("headlights %u, %u over capacity of %u", m_lights_lit, m_lights_dropped, static_cast<unsigned>(gl::MAX_LIGHTS - 1));
		auto const &stats{scene.clusters.stats};
		if (stats.valid)
		{
			ImGui::Text("index list %u of %u", stats.index_count, gl::CLUSTER_COUNT * gl::MAX_LIGHTS_PER_CLUSTER);
			ImGui::Text("overflow: %u lights dropped from full clusters (limit %u each), peak %u", stats.overflow_count, gl::MAX_LIGHTS_PER_CLUSTER, stats.overflow_peak);
		}
		ImGui::Separator();
		m_locallights.draw_debug_ui();
		ImGui::Separator();
		scene.lamps.draw_debug_ui();
		ImGui::Separator();
		char const *contactlevels[]{"no", "low (the sun)", "medium (the sun and 2 lamps)", "high (the sun and 4 lamps)"};
		ImGui::Combo("contact shadows (gfx.contactshadows)", &Global.gfx_contactshadows, contactlevels, IM_ARRAYSIZE(contactlevels));
		ImGui::SliderFloat("length of their rays, m (gfx.contactshadows.length)", &Global.gfx_contactshadows_length, 0.05f, 5.f, "%.2f", ImGuiSliderFlags_Logarithmic);
		ImGui::SliderFloat("distance they reach, m (gfx.contactshadows.range)", &Global.gfx_contactshadows_range, 1.f, 1000.f, "%.0f", ImGuiSliderFlags_Logarithmic);
	}
	ImGui::End();

	if (ImGui::Begin("Sun shadows"))
	{
		ImGui::Checkbox("cascades fitted to the depth of the view (gfx.shadow.sdsm)", &Global.gfx_shadow_sdsm);
		ImGui::SliderFloat("splits, 0 equal lengths .. 1 equal ratios (gfx.shadow.sdsm.split)", &Global.gfx_shadow_sdsm_split, 0.f, 1.f, "%.2f");
		auto const &fit{m_shadowfitstate};
		if (Global.gfx_shadow_sdsm)
		{
			if (fit.fitted)
			{
				ImGui::Text("fitted to pixels %.2f - %.1f m, measured %u frames ago, view turned %.1f deg since", fit.depthmin, fit.depthmax, fit.age, fit.turn);
			}
			else
			{
				ImGui::Text("fixed cascades: %s", fit.reason != nullptr ? fit.reason : "");
			}
		}
		if (scene.shadow_size > 0)
		{
			for (std::size_t cascade = 0; cascade < gl::MAX_CASCADES; ++cascade)
			{
				ImGui::Text("cascade %d: to %.1f m, map %.1f m across (whole slice %.1f m), texel %.1f cm", static_cast<int>(cascade + 1), fit.splits[cascade + 1], fit.drawn[cascade], fit.slice[cascade],
				            100.f * fit.drawn[cascade] / static_cast<float>(scene.shadow_size));
			}
		}
	}
	ImGui::End();

	if (ImGui::Begin("Weather"))
	{
		m_weatherstate.draw_debug_ui();
	}
	ImGui::End();

	if (ImGui::Begin("Particles"))
	{
		ImGui::Checkbox("smoke simulated and drawn on the gpu (gfx.smoke.gpu)", &Global.gfx_smoke_gpu);
		if (Global.gfx_smoke_gpu)
		{
			ImGui::Checkbox("smoke lit by the sun, the sky and the lamps (gfx.smoke.lighting)", &Global.gfx_smoke_lighting);
		}
		ImGui::Checkbox("rain and snow as particles of the gpu (gfx.precipitation.gpu)", &Global.gfx_precipitation_gpu);
		if (Global.gfx_precipitation_gpu)
		{
			ImGui::SliderInt("particles of the heaviest rain or snow (gfx.precipitation.particles)", &Global.gfx_precipitation_particles, 0, 262144);
		}
		ImGui::SliderFloat("soft particles, share of their size, 0: off (gfx.particles.soft)", &Global.gfx_particles_soft, 0.f, 4.f, "%.2f");
		auto const info{scene.particles.info()};
		if (false == info.empty())
		{
			ImGui::TextUnformatted(info.c_str() + 1);
		}
	}
	ImGui::End();

	if (ImGui::Begin("Sky and exposure"))
	{
		m_skylight.draw_debug_ui();
		ImGui::SliderFloat("sun disc, 1 = physical (gfx.sun.disc)", &Global.gfx_sun_disc, 0.f, 2.f);
		ImGui::SliderFloat("moon disc (gfx.moon.disc)", &Global.gfx_moon_disc, 0.f, 8.f);
		ImGui::SliderFloat("moonlight on the scenery (gfx.moonlight)", &Global.gfx_moonlight, 0.f, 10.f);
		{
			// where the sun and the moon are: azimuth from the north to the east (the sky has x to the west, z to the north)
			auto const azimuth = [](glm::vec3 const &Direction) { return glm::degrees(std::atan2(-Direction.x, Direction.z)) + (Direction.x > 0.f ? 360.f : 0.f); };
			auto const &sun{simulation::Environment.sun()};
			auto const &moon{simulation::Environment.moon()};
			ImGui::Text("sun: elevation %.1f, azimuth %.1f", sun.getAngle(), azimuth(sun.getDirection()));
			ImGui::Text("moon: elevation %.1f, azimuth %.1f, lit %.0f%%, age %.1f d", moon.getAngle(), azimuth(moon.getDirection()), 100.f * moon.getIllumination(), moon.getPhase());
		}
		ImGui::Separator();
		gfx::exposure_settings::draw_debug_ui(scene.exposure.values);
		ImGui::Separator();
		// the ambient occlusion (opengl46_ao::draw_debug_ui()): its images are made with the targets of the viewports
		if (scene.targets.empty() || (scene.targets.front().ssao == nullptr))
		{
			ImGui::TextUnformatted("ambient occlusion: off at the start, set gfx.postfx.ao in eu07.ini");
		}
		else
		{
			char const *modes[]{"off", "older ssao", "ground truth (gtao)"};
			ImGui::Combo("ambient occlusion (gfx.postfx.ao)", &Global.gfx_postfx_ao, modes, IM_ARRAYSIZE(modes));
			if (Global.gfx_postfx_ao == 2)
			{
				ImGui::SliderFloat("ao radius (m)", &Global.gfx_postfx_ao_radius, 0.1f, 10.f, "%.2f", ImGuiSliderFlags_Logarithmic);
				ImGui::SliderFloat("ao strength", &Global.gfx_postfx_ao_strength, 0.f, 4.f);
				ImGui::SliderFloat("ao bounce albedo", &Global.gfx_postfx_ao_bounce, 0.f, 1.f);
				ImGui::SliderInt("ao slices", &Global.gfx_postfx_ao_slices, 1, 8);
				ImGui::SliderInt("ao steps per side", &Global.gfx_postfx_ao_steps, 1, 16);
			}
		}
		ImGui::Separator();
		std::string raytracing;
		if (m_context->scene.raytracing)
		{
			ImGui::Checkbox("ray traced reflections where the screen has none (gfx.raytracing)", &m_raytracingon);
			ImGui::SliderFloat("range of their rays and of the scenery, m (gfx.raytracing.range)", &Global.gfx_raytracing_range, 10.f, 5000.f, "%.0f", ImGuiSliderFlags_Logarithmic);
			raytracing = m_context->scene.raytracing->info();
		}
		else
		{
			raytracing = (Global.gfx_raytracing ? "ray traced reflections: the gpu has no ray queries" : "ray traced reflections: off (gfx.raytracing yes in eu07.ini, taken at the start)");
		}
		m_ssr.draw_debug_ui(raytracing.c_str());
	}
	ImGui::End();

	if (ImGui::Begin("Instance culling"))
	{
		char const *modes[]{"cpu", "gpu, drawn from its result", "cpu, compared with the gpu"};
		ImGui::Combo("instance culling (gfx.gpucull)", &Global.gfx_gpucull, modes, IM_ARRAYSIZE(modes));
		if (Global.gfx_gpucull == 1)
		{
			ImGui::Checkbox("leave out instances hidden behind the depth drawn before them (gfx.gpucull.occlusion)", &Global.gfx_gpucull_occlusion);
			if (Global.gfx_gpucull_occlusion)
			{
				ImGui::Checkbox("draw the instances seen in the last frame first, to hide the others (gfx.gpucull.occlusion.instances)", &Global.gfx_gpucull_occlusion_instances);
			}
			ImGui::Checkbox("draw the depth of the instances before their colour (gfx.depthprepass)", &Global.gfx_depthprepass);
			ImGui::Checkbox("draw the instances of the same material and model with one multi draw (gfx.gpucull.multidraw)", &Global.gfx_gpucull_multidraw);
		}
		if (Global.gfx_gpucull != 0)
		{
			ImGui::Checkbox("the same for the cascades of the sun shadows (gfx.gpucull.shadows)", &Global.gfx_gpucull_shadows);
			ImGui::Checkbox("the same for the shadow maps of headlights (gfx.gpucull.shadows.headlights)", &Global.gfx_gpucull_shadows_headlights);
			ImGui::Checkbox("the same for the scenery of the shadow maps of lamps, when it's drawn next (gfx.gpucull.shadows.lamps)", &Global.gfx_gpucull_shadows_lamps);
			ImGui::Checkbox("the same for the sleepers of the tracks of the view (gfx.gpucull.sleepers)", &Global.gfx_gpucull_sleepers);
			ImGui::Checkbox("the same for the models drawn on their own, with lights, animations or skins of their own (gfx.gpucull.models)", &Global.gfx_gpucull_models);
			ImGui::Checkbox("leave out the models of a cell lying wholly out of view, beyond their range or too small before the culling (gfx.gpucull.bucketrange)", &Global.gfx_gpucull_bucketrange);
			ImGui::Checkbox("keep the walks of the submodel trees of the instances from frame to frame (gfx.gpucull.walkcache)", &Global.gfx_gpucull_walkcache);
		}
		ImGui::Checkbox("draw the depth of the whole scene before its colour (gfx.depthprepass.scene)", &Global.gfx_depthprepass_scene);
		if (Global.gfx_contactshadows > 0)
		{
			ImGui::TextUnformatted("(drawn anyway for the contact shadows, gfx.contactshadows)");
		}
		ImGui::Checkbox("grass of the heightmap terrain (gfx.grass)", &Global.gfx_grass);
		if (Global.gfx_grass)
		{
			ImGui::SliderFloat("the grass drawn whole up to, m (gfx.grass.distance)", &Global.gfx_grass_distance, 10.f, 300.f, "%.0f");
			ImGui::SliderFloat("thinning out up to, m (gfx.grass.fadedistance)", &Global.gfx_grass_fadedistance, 10.f, 350.f, "%.0f");
			ImGui::SliderFloat("density of the grass (gfx.grass.density)", &Global.gfx_grass_density, 0.1f, 2.f, "%.2f");
		}
		auto const info{cull_info()};
		if (false == info.empty())
		{
			ImGui::TextUnformatted(info.c_str() + 1);
		}
	}
	ImGui::End();
}

// the number of the lights of the cluster of each pixel tinted over the output of a viewport, from the depth of its scene
// (opengl46_renderer::Render_cluster_heatmap()), after its post-processing. outside a rendering
void vulkan14_renderer::render_cluster_heatmap(std::size_t const Index)
{
	auto &ctx{*m_context};
	auto &scene{ctx.scene};
	auto &targets{scene.targets[Index]};
	auto &draw{*m_draw};
	auto const commands{ctx.commands};
	if ((scene.clusters.params.valid == 0) || (false == bool(targets.output)) || (false == bool(targets.depth)))
	{
		return;
	}
	if ((scene.cluster_heatmap == nullptr) && (false == scene.cluster_heatmap_tried))
	{
		scene.cluster_heatmap_tried = true;
		try
		{
			auto program{std::make_shared<vk14::scene_program>()};
			program->vertex = ctx.shaders.get("quad.vert", "#define QUAD_SCENE 1\n");
			program->fragment = ctx.shaders.get("cluster_heatmap.frag");
			program->fragment_name = "cluster_heatmap.frag";
			program->vertices = vk14::vertex_kind::none;
			program->name = "quad.vert + cluster_heatmap.frag";
			scene.cluster_heatmap = program;
		}
		catch (gfx::shader_exception const &e)
		{
			ErrorLog("light clusters: can't use the heatmap: " + std::string(e.what()));
		}
	}
	if (scene.cluster_heatmap == nullptr)
	{
		return;
	}
	// the depth of the scene as the colour pass resolved it
	vk14::barriers(commands, {vk14::use_of(*targets.output, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, heatmap_colour_stage, heatmap_colour_access),
	                          vk14::use_of(*targets.depth, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT)});
	VkRenderingAttachmentInfo attachment{};
	attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
	attachment.imageView = targets.output->view();
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
	// the clusters of the pass set of the colour pass, with the offsets it was drawn with
	auto const offsets{std::array<std::uint32_t, 4>{draw.scene_offset, draw.light_offset, draw.cluster_offset, draw.weather_offset}};
	start_draws(targets.output_formats, static_cast<int>(targets.extent.width), static_cast<int>(targets.extent.height), m_viewports[Index].main ? vk14::scene_state::set_colour : vk14::scene_state::set_plain, false);
	draw.scene_offset = offsets[0];
	draw.light_offset = offsets[1];
	draw.cluster_offset = static_cast<std::uint32_t>(scene.clusters.params_offset);
	draw.weather_offset = offsets[3];
	if (bind_scene_pipeline(*scene.cluster_heatmap, targets.output_formats, vk14::blend_mode::scene_alpha, VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP))
	{
		bind_pass_set();
		std::array<VkDescriptorImageInfo, gl::MAX_TEXTURES> images;
		images.fill(VkDescriptorImageInfo{ctx.sampling.clamp_nearest, targets.depth->view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL});
		scene.material_set.push(commands, scene.scene_layout, images.data());
		vk14::draw_constants constants;
		push_constants(&constants);
		commands->draw(4, 1, 0, 0);
	}
	commands->end_rendering();
	end_draws();
}
