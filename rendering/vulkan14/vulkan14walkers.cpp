/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the passes walked by the walkers (the stage M2 of claude/vulkan14-renderer-plan.md, gfx.vulkan.parallelpasses). the main thread sets
// a pass up as it would walk it itself (the camera, the culling, the map it draws to with its barrier, the timestamps around it), hands
// the state of the frame over to a walker, splices the stream the walker records into the commands of the frame there and goes on with
// the frame; the walkers walk the passes at the same time as the main thread walks the colour pass of the main view, and are joined once
// it's recorded. what the walks share is guarded by gfx::walk_sync; what a walker's draws used is marked by the main renderer after the
// join. with the switch off, or for the other viewports, the passes are walked by the main thread as before

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14context.h"
#include "rendering/vulkan14/vulkan14commands.h"
#include "rendering/common/walksync.h"
#include "model/Model3d.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>

std::size_t vk14::walk_threads()
{
	// a pass for each of them: the three cascades, a lamp or two
	auto threads{static_cast<std::size_t>(Global.gfx_threads)};
	if (threads == 0)
	{
		auto const cores{static_cast<std::size_t>(std::thread::hardware_concurrency())};
		threads = (cores > 3 ? cores - 2 : 1);
	}
	return std::clamp<std::size_t>(threads, 1, 4);
}

vulkan14_renderer::vulkan14_renderer(vulkan14_renderer &Main) : m_context{Main.m_context}, m_main{&Main}, m_geometry{Main.m_geometry}, m_banks{Main.m_banks}, m_materials{Main.m_materials}
{
	m_draw = std::make_unique<draw_state>();
	m_draw->textures.fill(null_handle);
}

bool vulkan14_renderer::walks_wanted() const
{
	return (m_main == nullptr) && Global.gfx_vulkan_parallelpasses && m_context && m_context->walks.running() && (m_current_viewport != nullptr) && m_current_viewport->main && (m_commands != nullptr);
}

void vulkan14_renderer::take_state(vulkan14_renderer const &Main)
{
	// the frame: its viewports, the passes set up for it, the light, the blocks of the scene and of the lights uploaded so far
	m_viewports = Main.m_viewports;
	m_current_viewport = (Main.m_current_viewport != nullptr) ? &m_viewports[static_cast<std::size_t>(Main.m_current_viewport - Main.m_viewports.data())] : nullptr;
	m_renderpass = Main.m_renderpass;
	m_colorpass = Main.m_colorpass;
	m_shadowpass = Main.m_shadowpass;
	m_sunlight = Main.m_sunlight;
	m_fogcolor = Main.m_fogcolor;
	m_shadowcolor = Main.m_shadowcolor;
	scene_ubs = Main.scene_ubs;
	light_data = Main.light_data;
	m_lightcount = Main.m_lightcount;
	m_lightheaders = Main.m_lightheaders;
	m_cabambient = Main.m_cabambient;
	m_envmap_bound = Main.m_envmap_bound;
	m_headlightconfig = Main.m_headlightconfig;
	m_raytracingon = Main.m_raytracingon;
	m_rtfoliage = Main.m_rtfoliage;
	m_rtgather = false;
	m_instancecullframe = Main.m_instancecullframe;
	m_indirectcolourpending = false;
	m_cubeshadows = false;
	m_shadowcascade = -1;
	m_lightshadowcull = nullptr;
	// the textures and the materials the renderer draws with
	m_invalid_material = Main.m_invalid_material;
	m_suntexture = Main.m_suntexture;
	m_moontexture = Main.m_moontexture;
	m_smoketexture = Main.m_smoketexture;
	m_headlightstexture = Main.m_headlightstexture;
	m_precipitationtexture = Main.m_precipitationtexture;
	// the walk
	model_ubs = Main.model_ubs;
	m_framestamp = Main.m_framestamp;
	m_instancerecordsversion = Main.m_instancerecordsversion;
	m_fogrange = Main.m_fogrange;
	m_billboardgeometry = Main.m_billboardgeometry;
	m_glaretexture = Main.m_glaretexture;
	m_motionhistory = Main.m_motionhistory;
	m_cabfuture = Main.m_cabfuture;
	m_cablamplit = Main.m_cablamplit;
	m_cabprobelit = Main.m_cabprobelit;
	m_previewing = false;
	m_cubescenery = false;
	m_cubeskipinstance = nullptr;
	m_shadowskipvehicle = nullptr;
	m_renderme.clear();
	// the state of the thread of the main renderer, taken by the thread of the walker when it begins
	m_forkmatrices = OpenGLMatrices;
	m_forksubmodels = submodel_state{TSubModel::iInstance, TSubModel::iVariantSeed, TSubModel::ReplacableSkinId, TSubModel::iAlpha, TSubModel::fSquareDist, TSubModel::pRoot, TSubModel::pasText};
	// the draws: the offsets of the blocks of the pass set as the main renderer has them, the pass sets of the frame, the parts of the
	// rings of its own
	auto &ctx{*m_context};
	auto &draw{*m_draw};
	auto const &maindraw{*Main.m_draw};
	draw.scene_offset = maindraw.scene_offset;
	draw.light_offset = maindraw.light_offset;
	draw.cluster_offset = maindraw.cluster_offset;
	draw.weather_offset = maindraw.weather_offset;
	draw.pass_sets = ctx.scene.pass_sets;
	draw.rendering = false;
	draw.program = nullptr;
	draw.textures.fill(null_handle);
	draw.textures_dirty = true;
	draw.cull_sets = nullptr;
	draw.resume.valid = false;
	draw.depthpass = false;
	draw.probepass = false;
	draw.async = true;
	draw.lastprogram = nullptr;
	draw.draws_cache.reset(&ctx.scene.draws.ring, vk14::draw_ring_part);
	draw.instances_cache.reset(&ctx.scene.instances.ring, vk14::instance_ring_part);
	draw.draws_window = {};
	draw.instances_window = {};
	draw.used_banks.clear();
	draw.used_textures.clear();
	m_walkfailed = false;
}

void vulkan14_renderer::fork_walk(std::function<void(vulkan14_renderer &)> Job, walk_kind const Kind)
{
	auto &ctx{*m_context};
	if (false == m_walkwindow)
	{
		// the pass sets of the frame as the walkers bind them (they don't make them anew), then the walk lock held by the main thread,
		// which walks as well from now on
		update_pass_sets();
		gfx::walk_sync::open();
		m_walkwindow = true;
	}
	auto const index{m_walkjobs++};
	while (m_walkers.size() <= index)
	{
		m_walkers.emplace_back(std::make_unique<vulkan14_renderer>(*this));
	}
	if (m_walkstreams.size() < ctx.frames.size())
	{
		m_walkstreams.resize(ctx.frames.size());
	}
	auto &streams{m_walkstreams[ctx.slot]};
	while (streams.size() <= index)
	{
		streams.emplace_back(std::make_unique<vk14::command_stream>());
	}
	auto &stream{*streams[index]};
	auto &walker{*m_walkers[index]};
	// the stream of the frame which used it last was recorded into its command buffers before the slot of the frame came back
	stream.clear();
	walker.take_state(*this);
	walker.m_walkkind = Kind;
	walker.m_commands = &stream;
	walker.m_draw->commands = &stream;
	// the pass goes to the gpu here, with the timestamps the main thread put around it
	m_commands->splice(stream);
	ctx.walks.add(
	    [&walker, &stream, job = std::move(Job)]() {
		    gfx::walk_sync::walk_scope scope;
		    // the state of the thread of the main renderer at the moment it handed the pass over
		    OpenGLMatrices = walker.m_forkmatrices;
		    auto const &submodels{walker.m_forksubmodels};
		    TSubModel::iInstance = submodels.instance;
		    TSubModel::iVariantSeed = submodels.variantseed;
		    TSubModel::ReplacableSkinId = submodels.skins;
		    TSubModel::iAlpha = submodels.alpha;
		    TSubModel::fSquareDist = submodels.squaredistance;
		    TSubModel::pRoot = submodels.root;
		    TSubModel::pasText = submodels.text;
		    try
		    {
			    job(walker);
		    }
		    catch (std::exception const &Exception)
		    {
			    ErrorLog(std::string("vulkan: a walker failed to walk its pass, which is left out: ") + Exception.what());
			    walker.m_walkfailed = true;
		    }
		    if (walker.m_walkfailed)
		    {
			    // whatever it left half done (a rendering under way) goes
			    stream.clear();
		    }
		    walker.m_commands = nullptr;
		    walker.m_draw->commands = nullptr;
		    stream.finish();
	    },
	    false);
}

void vulkan14_renderer::join_walks()
{
	if (false == m_walkwindow)
	{
		return;
	}
	auto &ctx{*m_context};
	auto const start{std::chrono::steady_clock::now()};
	// the main thread lets go of the walk lock before it waits, so the walkers can make what they need meanwhile
	gfx::walk_sync::leave();
	ctx.walks.wait_idle();
	gfx::walk_sync::close();
	m_walkwindow = false;
	auto const waited{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count()};
	m_walkwait = (m_walkwait == 0.0 ? waited : m_walkwait * 0.9 + waited * 0.1);
	// what the walkers' draws used marked as used, what they did counted
	if (m_walkcountsframe != m_framestamp)
	{
		m_walkcountsframe = m_framestamp;
		m_lastwalkcounts = m_walkcounts;
		m_walkcounts = {};
	}
	std::uint32_t lampdraws{0};
	std::uint32_t headlightdraws{0};
	for (std::size_t index = 0; index < m_walkjobs; ++index)
	{
		auto &walker{*m_walkers[index]};
		auto &draw{*walker.m_draw};
		for (auto const bank : draw.used_banks)
		{
			m_geometry.mark_used(gfx::geometry_handle{bank, 1});
		}
		if (ctx.textures)
		{
			for (auto const texture : draw.used_textures)
			{
				ctx.textures->mark_as_used(texture);
			}
		}
		draw.used_banks.clear();
		draw.used_textures.clear();
		if (walker.m_walkcountsframe == m_framestamp)
		{
			m_walkcounts.walked += walker.m_walkcounts.walked;
			m_walkcounts.changing += walker.m_walkcounts.changing;
			m_walkcounts.reused += walker.m_walkcounts.reused;
			m_walkcounts.walktime += walker.m_walkcounts.walktime;
			m_walkcounts.ordertime += walker.m_walkcounts.ordertime;
		}
		auto const drawcalls{static_cast<std::uint32_t>(walker.m_renderpass.draw_stats.drawcalls)};
		switch (walker.m_walkkind)
		{
		case walk_kind::lamp:
			lampdraws += drawcalls;
			break;
		case walk_kind::headlight:
			headlightdraws += drawcalls;
			break;
		default:
			break;
		}
		walker.Settle_Renderme(false);
	}
	ctx.scene.lamps.add_drawcalls(lampdraws, headlightdraws);
	// the vehicles drawn by the colour pass keep what it set, as when the passes come one after another; then what the walks put off
	Settle_Renderme(true);
	Release_Deferred();
	m_walkpasses = m_walkjobs;
	m_walkjobs = 0;
}

// the walks of the groups of the cullings split among the helpers (the stage M3, gfx.vulkan.parallelmodels): a few groups are walked by
// the walk itself
bool vulkan14_renderer::walk_groups_parallel(std::size_t const Groups)
{
	m_helpgroups = Groups;
	return Global.gfx_vulkan_parallelmodels && (m_context != nullptr) && m_context->models.running() && (Groups >= 16) && (false == m_previewing) && (false == m_rtgather);
}

void vulkan14_renderer::take_helper_state(vulkan14_renderer const &Walk)
{
	// what the walk of the groups reads: the pass, the model block and the blending the walk is at, its recording of the draws
	m_renderpass = Walk.m_renderpass;
	m_renderpass.draw_stats = {};
	model_ubs = Walk.model_ubs;
	m_framestamp = Walk.m_framestamp;
	m_instancerecordsversion = Walk.m_instancerecordsversion;
	m_fogrange = Walk.m_fogrange;
	m_previewing = Walk.m_previewing;
	m_motionhistory = Walk.m_motionhistory;
	m_cabfuture = Walk.m_cabfuture;
	m_invalid_material = Walk.m_invalid_material;
	m_rtgather = false;
	m_indirectrecording = true;
	m_indirectentry = Walk.m_indirectentry;
	m_indirectmaterial = Walk.m_indirectmaterial;
	m_indirectsubmodel = Walk.m_indirectsubmodel;
	m_indirectchanging = false;
	m_forkmatrices = OpenGLMatrices;
	m_forksubmodels = submodel_state{TSubModel::iInstance, TSubModel::iVariantSeed, TSubModel::ReplacableSkinId, TSubModel::iAlpha, TSubModel::fSquareDist, TSubModel::pRoot, TSubModel::pasText};
	auto &draw{*m_draw};
	draw.blending = Walk.m_draw->blending;
	draw.rendering = false;
	draw.depthpass = false;
	draw.probepass = false;
	draw.used_banks.clear();
	draw.used_textures.clear();
}

void vulkan14_renderer::walk_groups_run(std::size_t const Tasks, std::function<void(gfx::scene_walk &Helper, std::size_t const Task)> const &Work)
{
	auto &ctx{*m_context};
	auto *main{m_main != nullptr ? m_main : this};
	// the helpers hold the models they walk (gfx::walk_sync): the window of the walks is opened for them when the main thread walks
	// alone
	auto const opened{(m_main == nullptr) && (false == gfx::walk_sync::parallel())};
	if (opened)
	{
		gfx::walk_sync::open();
	}
	while (m_helpers.size() < Tasks)
	{
		m_helpers.emplace_back(std::make_unique<vulkan14_renderer>(*main));
	}
	for (std::size_t task = 0; task < Tasks; ++task)
	{
		m_helpers[task]->take_helper_state(*this);
	}
	std::mutex mutex;
	std::condition_variable finished;
	std::size_t left{Tasks};
	std::exception_ptr failure;
	for (std::size_t task = 0; task < Tasks; ++task)
	{
		ctx.models.add(
		    [&, task]() {
			    {
				    // covered by the hold of the walk waiting for it
				    gfx::walk_sync::borrowed_scope scope;
				    auto &helper{*m_helpers[task]};
				    OpenGLMatrices = helper.m_forkmatrices;
				    auto const &submodels{helper.m_forksubmodels};
				    TSubModel::iInstance = submodels.instance;
				    TSubModel::iVariantSeed = submodels.variantseed;
				    TSubModel::ReplacableSkinId = submodels.skins;
				    TSubModel::iAlpha = submodels.alpha;
				    TSubModel::fSquareDist = submodels.squaredistance;
				    TSubModel::pRoot = submodels.root;
				    TSubModel::pasText = submodels.text;
				    try
				    {
					    Work(helper, task);
				    }
				    catch (...)
				    {
					    std::lock_guard<std::mutex> lock(mutex);
					    failure = std::current_exception();
				    }
			    }
			    std::lock_guard<std::mutex> lock(mutex);
			    if (--left == 0)
			    {
				    finished.notify_all();
			    }
		    },
		    false);
	}
	{
		std::unique_lock<std::mutex> lock(mutex);
		finished.wait(lock, [&]() { return left == 0; });
	}
	// the banks the helpers' draws used, marked with those of the walk
	for (std::size_t task = 0; task < Tasks; ++task)
	{
		auto &used{m_helpers[task]->m_draw->used_banks};
		for (auto const bank : used)
		{
			if (m_main != nullptr)
			{
				m_draw->used_banks.push_back(bank);
			}
			else
			{
				m_geometry.mark_used(gfx::geometry_handle{bank, 1});
			}
		}
		used.clear();
	}
	if (opened)
	{
		gfx::walk_sync::close();
	}
	ctx.helped += m_helpgroups;
	if (failure)
	{
		std::rethrow_exception(failure);
	}
}
