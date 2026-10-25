/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the part of the shared walk of the scene which feeds the gpu culling of the instances and records the draws made from its
// result (stage N6c of claude/vulkan14-renderer-plan.md): the records of the sleepers and of the models drawn on their own, the
// queueing of buckets, the walks of the submodel trees of the culled groups and the order of the recorded draws

#include "stdafx.h"
#include "rendering/common/scenewalk.h"

#include "model/AnimModel.h"
#include "model/Model3d.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"

namespace gfx
{

namespace
{

// the group of the sleepers of a track in the gpu culling: their model, and the skin in the slot Render_Sleepers() sets it in
scene::basic_cell::instance_bucket_key sleeper_key(TTrack const &Track)
{
	scene::basic_cell::instance_bucket_key key;
	key.pModel = Track.m_sleeper_model;
	key.skins[1] = Track.m_sleeper_skin;
	return key;
}

} // namespace

void scene_walk::Begin_Pass_Cull(gfx::instance_cull *Cull, bool const Mainview)
{
	m_passcull = Cull;
	m_instancecullpass = false;
	if (m_passcull != nullptr)
	{
		m_instancecullpass = true;
		// the same choices as in Render_Instanced()
		auto const reflections{m_renderpass.draw_mode == rendermode::reflections};
		gfx::instance_cull::pass_setup setup;
		setup.frame = m_framestamp;
		setup.camera = m_renderpass.pass_camera.position();
		setup.distancecamera = (m_renderpass.draw_mode == rendermode::shadows ? m_renderpass.viewport_camera.position() : m_renderpass.pass_camera.position());
		setup.frustum = &m_renderpass.pass_camera.frustum();
		setup.distancescale = (reflections ? 1.f : static_cast<float>(1.0 / (static_cast<double>(Global.ZoomFactor) * static_cast<double>(Global.ZoomFactor) * static_cast<double>(Global.fDistanceFactor))));
		setup.distanceoffset = (reflections ? static_cast<float>(sq(EU07_REFLECTIONFIDELITYOFFSET)) : 0.f);
		setup.distancerange = (reflections ? static_cast<float>(sq(Global.reflectiontune.range_instances)) : std::numeric_limits<float>::max());
		setup.drawrange = static_cast<float>(sq(m_renderpass.draw_range + 250));
		setup.zoom = Global.ZoomFactor;
		setup.drawing = (Global.gfx_gpucull == 1);
		m_passcull->begin_pass(setup);
	}
	// the sleepers of tracks go with the instances of the main view
	m_sleeperspass = (Mainview && (m_passcull != nullptr) && Global.gfx_gpucull_sleepers);
	if (m_sleeperspass && (false == m_passcull->drawing()))
	{
		for (auto &counts : m_sleeperbandcounts)
		{
			counts.second.clear();
		}
	}
	// the models drawn on their own go with the instances of every pass culled on the gpu
	m_modelspass = (m_instancecullpass && Global.gfx_gpucull_models);
}

void scene_walk::Compare_Pass_Cull()
{
	// the sleepers the cpu drew along the tracks, before the culling ran
	if (m_sleeperspass)
	{
		for (auto const &counts : m_sleeperbandcounts)
		{
			if (false == counts.second.empty())
			{
				m_passcull->cpu_result(counts.first, counts.second, true);
			}
		}
	}
	// and the models drawn on their own
	for (auto const &model : m_modelcpudistances)
	{
		m_passcull->cpu_instance(model.first, model.second);
	}
}

void scene_walk::End_Pass_Cull()
{
	if (m_instancecullpass)
	{
		m_passcull->end_pass();
		m_instancecullpass = false;
		m_passcull = nullptr;
	}
	m_sleeperspass = false;
	m_modelspass = false;
}

bool scene_walk::walk_bucket(scene::basic_cell::instance_bucket_key const &Key, scene::basic_cell::instance_bucket const &Bucket, bool const Bucketrange, scene::bounding_area const &Area)
{
	if (m_instancecullpass)
	{
		// the gpu tests every instance of the bucket, the way Render_Instanced() does. models with nothing to draw in the
		// sun shadows are left out of them, as Render_Instanced() leaves them
		if ((m_renderpass.draw_mode != rendermode::shadows) || Instances_Cast_Shadow(Key, Bucket))
		{
			m_passcull->add_bucket(Key, Bucket, (Bucketrange ? Bucket_Area(Bucket) : Area));
		}
		// and when the instances are drawn from its result, the cpu has nothing more to do with them
		if (m_passcull->drawing())
		{
			return false;
		}
	}
	// the depth pass of the whole scene culled them already, the colour pass draws what it recorded
	if (m_indirectcolourpending)
	{
		return false;
	}
	return true;
}

void scene_walk::walk_bucket_culled(scene::basic_cell::instance_bucket const &Bucket)
{
	if (m_instancecullpass)
	{
		m_passcull->skip_bucket(Bucket);
	}
}

bool scene_walk::walk_queue_model(TAnimModel *Instance, bool const Inrange)
{
	return Queue_Model(Instance, Inrange);
}

bool scene_walk::walk_queue_sleepers(TTrack *Track)
{
	if (m_sleeperspass)
	{
		// culled with the instances of the pass and drawn from the result, or drawn by the cpu as well to compare
		Queue_Sleepers(Track);
		if (m_passcull->drawing())
		{
			return true;
		}
	}
	else if (m_indirectcolourpending && m_indirectcoloursleepers)
	{
		// the depth pass of the whole scene culled and recorded them, the colour pass draws what it recorded
		return true;
	}
	return false;
}

void scene_walk::walk_sleepers_drawn(TTrack const &Track, std::vector<float> const &Lodbounds, std::vector<std::pair<float, glm::mat4>> const &Drawn)
{
	if (m_sleeperspass && m_instancecullpass && (false == m_passcull->drawing()))
	{
		// the comparison with the gpu culling: the sleepers drawn by lod band, the bands as the culling makes them, split by
		// the bounds of the model past 0 and short of the largest float
		auto const counted = [](float const Bound) {
			return (Bound > 0.f) && (Bound < std::numeric_limits<float>::max());
		};
		auto &counts{m_sleeperbandcounts[sleeper_key(Track)]};
		auto const bands{static_cast<std::size_t>(std::count_if(Lodbounds.begin(), Lodbounds.end(), counted)) + 1};
		if (counts.size() < bands)
		{
			counts.resize(bands, 0);
		}
		for (auto const &survivor : Drawn)
		{
			std::size_t band{0};
			for (auto const bound : Lodbounds)
			{
				if (counted(bound) && (bound <= survivor.first))
				{
					++band;
				}
			}
			++counts[band];
		}
	}
}

void scene_walk::walk_instances_compare(scene::basic_cell::instance_bucket_key const &Key, std::vector<std::uint32_t> const &Bandcounts)
{
	m_passcull->cpu_result(Key, Bandcounts);
}

void scene_walk::Queue_Sleepers(TTrack *Track)
{
	// the gates of Render_Sleepers(); the range is tested for the whole track
	if ((Track->m_sleeper_model == nullptr) || Track->m_sleeper_local_transforms.empty() || (Global.SleeperDistance <= 0.f))
	{
		return;
	}
	auto const distsq{glm::length2(Track->location() - m_renderpass.pass_camera.position())};
	auto const cutoffsq{static_cast<double>(Global.SleeperDistance) * static_cast<double>(Global.SleeperDistance)};
	if (distsq > cutoffsq)
	{
		return;
	}

	auto &records{m_sleeperrecords[Track]};
	auto const &transforms{Track->m_sleeper_local_transforms};
	if ((records.bucket.recordsversion != m_instancerecordsversion) || (records.model != Track->m_sleeper_model) || (records.skin != Track->m_sleeper_skin) ||
	    (records.transforms != transforms.data()) || (records.count != transforms.size()))
	{
		// each sleeper is a record placed and turned by its transform, relative to the origin of the track (the centre of its
		// cell), with the sphere Render_Sleepers() tests against the frustum
		auto const radius{std::max(Track->m_sleeper_model->bounding_radius(), 2.0f)};
		auto &bucket{records.bucket};
		bucket.records.clear();
		bucket.records.reserve(transforms.size());
		glm::vec3 low{std::numeric_limits<float>::max()};
		glm::vec3 high{std::numeric_limits<float>::lowest()};
		for (auto const &local : transforms)
		{
			scene::basic_cell::instance_record record;
			glm::vec3 const position{local[3]};
			record.position = packed_vec3{position};
			record.radius = radius;
			record.rangesquaredmin = 0.f;
			record.rangesquaredmax = std::numeric_limits<float>::max();
			record.flags = scene::basic_cell::instance_visible | scene::basic_cell::instance_instanceable;
			for (int column = 0; column < 3; ++column)
			{
				record.rotationscale[column] = local[column];
			}
			bucket.records.emplace_back(record);
			low = glm::min(low, position);
			high = glm::max(high, position);
		}
		bucket.recordorigin = Track->m_origin;
		bucket.lodrangesquared = std::numeric_limits<float>::max();
		bucket.recordsversion = m_instancerecordsversion;
		bucket.gpustale = true;
		records.key = sleeper_key(*Track);
		records.area = scene::bounding_area{Track->m_origin + glm::dvec3{(low + high) * 0.5f}, 0.5f * glm::length(high - low)};
		records.model = Track->m_sleeper_model;
		records.skin = Track->m_sleeper_skin;
		records.transforms = transforms.data();
		records.count = transforms.size();
	}
	records.frame = m_framestamp;
	m_passcull->add_sleepers(records.key, records.bucket, records.area);
}

// queues a model instance drawn on its own in the gpu culling of the pass, as a bucket and a group of its own. the culling
// makes the tests of Render( TAnimModel * ): visibility, frustum, the range of the instance, the draw range, the size on the
// screen, and gives it the lod band of the bounds of its submodels (mindistance/maxdistance) its distance falls in, then
// Walk_Indirect_Draws() animates, prepares and walks its submodel tree from its origin, like it does for a group of
// instanced ones
bool scene_walk::Queue_Model(TAnimModel *Instance, bool const Inrange)
{
	// a billboard is turned to the camera from the modelview of the instance, which a walk from its origin doesn't have
	if ((false == Instance->m_cameraindependent) || (Instance->Model() == nullptr))
	{
		return false;
	}
	if (false == m_modelspass)
	{
		// the depth pass of the whole scene culled and recorded it, the colour pass draws what it recorded
		return (m_indirectcolourpending && m_indirectcolourmodels);
	}
	auto const drawing{m_passcull->drawing()};
	// a model hidden by an event, or in a cell wholly out of the range of the pass, would be left out by the culling
	if ((false == Instance->visible()) || (false == Inrange))
	{
		return drawing;
	}

	auto &records{m_modelrecords[Instance]};
	auto &bucket{records.bucket};
	if ((bucket.recordsversion != m_instancerecordsversion) || (bucket.records.size() != 1))
	{
		// the key, the instance and the record anew: the editor may have given the instance another model or skins, or
		// another instance may have taken the place of a deleted one
		auto &key{records.key};
		key = {};
		key.pModel = Instance->Model();
		if (auto const *material{Instance->Material()})
		{
			for (int skin = 0; skin < 5; ++skin)
			{
				key.skins[skin] = material->replacable_skins[skin];
			}
		}
		key.instance = Instance;
		bucket.instances.assign(1, Instance);
		// the location of the instance as the origin of the record, which keeps the precision
		Update_Instance_Records(key.pModel, bucket, Instance->location());
		// drawn by its bucket, though it isn't instanceable
		bucket.records.front().flags |= scene::basic_cell::instance_instanceable;
		records.area = scene::bounding_area{Instance->location(), 0.f};
	}
	records.frame = m_framestamp;
	if (m_renderpass.draw_mode == rendermode::shadows)
	{
		// a model with nothing to draw in the shadows is left out of them, like the instances. the answer depends on the
		// model and the skins only, and is shared by the instances with them
		auto modelkey{records.key};
		modelkey.instance = nullptr;
		if (false == Instances_Cast_Shadow(modelkey, bucket))
		{
			return drawing;
		}
	}
	// out of the frustum of the pass, beyond its range or too small on the screen by its record: the culling would leave it out,
	// and the walk of Walk_Indirect_Draws() isn't made for it
	if (Global.gfx_gpucull_bucketrange && (false == EditorModeFlag) && Bucket_Culled(bucket))
	{
		m_passcull->skip_bucket(bucket);
		return drawing;
	}
	m_passcull->add_bucket(records.key, bucket, records.area);
	if (false == drawing)
	{
		// the cpu draws it too, and tells what it drew for the comparison
		m_modelcomparekey = &records.key;
	}
	return drawing;
}

bool scene_walk::Instances_Cast_Shadow(scene::basic_cell::instance_bucket_key const &Key, scene::basic_cell::instance_bucket const &Bucket)
{
	auto const lookup{m_shadowcasters.find(Key)};
	if (lookup != m_shadowcasters.end())
	{
		return lookup->second;
	}

	// the skins of the instance Render_Instanced() would take them from
	TAnimModel const *instance{nullptr};
	for (auto const *candidate : Bucket.instances)
	{
		if (candidate != nullptr)
		{
			instance = candidate;
			break;
		}
	}
	auto const *skins{instance != nullptr && instance->Material() != nullptr ? instance->Material()->replacable_skins : nullptr};
	auto castsshadow{false};
	m_instance_lodpending.clear();
	m_instance_lodpending.push_back(Key.pModel != nullptr ? Key.pModel->Root : nullptr);
	while ((false == castsshadow) && (false == m_instance_lodpending.empty()))
	{
		auto const *submodel = m_instance_lodpending.back();
		m_instance_lodpending.pop_back();
		if (submodel == nullptr)
		{
			continue;
		}
		if (submodel->eType < TP_ROTATOR)
		{
			auto const material{submodel->m_material >= 0 ? submodel->m_material : (skins != nullptr ? skins[-submodel->m_material] : null_handle)};
			castsshadow = (walk_material_data(material).shadow_rank <= Global.gfx_shadow_rank_cutoff);
		}
		m_instance_lodpending.push_back(submodel->Child);
		m_instance_lodpending.push_back(submodel->Next);
	}
	m_shadowcasters.emplace(Key, castsshadow);
	return castsshadow;
}

void scene_walk::Walk_Indirect_Draws(gfx::instance_cull const &Cull, bool const Cascade)
{
	m_indirectdraws.clear();
	m_indirectcommands.clear();
	auto const &lodbounds{Cull.lod_bounds()};
	// the cascades of the sun shadows make the same walks: the mode, the lod band and the state of the models are the same in all
	// of them within a frame, only the instances differ, and those the gpu fills in. the walk of a group in a band is made by the
	// first cascade which draws it, the others copy its draws and commands with entries of their own
	auto const cascade{(m_renderpass.draw_mode == rendermode::shadows) && Cascade};
	auto constexpr nowalk{std::numeric_limits<std::uint32_t>::max()};
	if (cascade && (m_cascaderecordframe != m_framestamp))
	{
		m_cascaderecordframe = m_framestamp;
		m_cascadedraws.clear();
		m_cascadecommands.clear();
		if ((m_framestamp % 256) == 0)
		{
			// the groups not drawn for a while
			for (auto records{m_cascaderecords.begin()}; records != m_cascaderecords.end();)
			{
				if (m_framestamp - records->second.frame > 256)
				{
					records = m_cascaderecords.erase(records);
				}
				else
				{
					++records;
				}
			}
		}
	}
	if (m_walkcountsframe != m_framestamp)
	{
		m_walkcountsframe = m_framestamp;
		m_lastwalkcounts = m_walkcounts;
		m_walkcounts = {};
	}
	auto const walkstart{std::chrono::steady_clock::now()};
	// the walks of earlier frames. they go when the switch is off, as the models may change meanwhile without the walks noticing
	// it the next time (the season), and when the shadows take other submodels
	auto const walkcache{Global.gfx_gpucull_walkcache && (false == m_previewing)};
	if ((false == Global.gfx_gpucull_walkcache) || (m_walkrankcutoff != Global.gfx_shadow_rank_cutoff))
	{
		m_walkrecords.clear();
		m_walkrankcutoff = Global.gfx_shadow_rank_cutoff;
	}
	if (walkcache && ((m_framestamp % 256) == 0))
	{
		// the groups not drawn for a while
		for (auto records{m_walkrecords.begin()}; records != m_walkrecords.end();)
		{
			if (m_framestamp - records->second.frame > 256)
			{
				records = m_walkrecords.erase(records);
			}
			else
			{
				++records;
			}
		}
	}
	auto const shadows{m_renderpass.draw_mode == rendermode::shadows};
	m_indirectrecording = true;
	for (auto const &group : Cull.draw_groups())
	{
		auto *model{group.key->pModel};
		if ((model == nullptr) || (model->Root == nullptr) || ((group.instance == nullptr) && (false == group.sleepers)))
		{
			continue;
		}
		// a model drawn on its own is prepared with its lights and animations each time; the others are prepared only for a walk.
		// the version is taken before, so a walk prepared by something which changes the model isn't taken up again
		walk_record *kept{nullptr};
		if (walkcache && (group.key->instance == nullptr))
		{
			kept = &m_walkrecords[*group.key];
			kept->frame = m_framestamp;
			auto &bands{kept->bands[shadows ? 1 : 0]};
			if (bands.size() != group.lodcount + 1)
			{
				bands.clear();
				bands.resize(group.lodcount + 1);
			}
		}
		auto const modelversion{model->state_version()};
		material_handle const *skins{nullptr};
		auto alpha{0x30300030};
		auto prepared{false};
		auto const prepare = [&]() {
			prepared = true;
			if (group.sleepers)
			{
				// as Render_Sleepers() sets them up: the skin of the track, if any. like scenery models they pick no texture
				// variants of their own
				if (group.key->skins[1] != null_handle)
				{
					skins = group.key->skins.data();
				}
				TSubModel::iVariantSeed = 0;
			}
			else
			{
				if (group.key->instance != nullptr)
				{
					// a model drawn on its own: its lights and animations, the way Render( TAnimModel * ) sets them up
					group.instance->RaAnimate(m_framestamp);
				}
				group.instance->RaPrepare();
				auto const *material{group.instance->Material()};
				if (material != nullptr)
				{
					skins = material->replacable_skins;
					alpha = material->textures_alpha;
				}
			}
			alpha ^= 0x0F0F000F;
		};
		// the walks of the group the cascades made in this frame. a group whose bands were all walked already isn't prepared
		cascade_record *walks{nullptr};
		if (cascade)
		{
			walks = &m_cascaderecords[*group.key];
			if ((walks->frame != m_framestamp) || (walks->bands.size() != group.lodcount + 1))
			{
				walks->frame = m_framestamp;
				walks->bands.assign(group.lodcount + 1, cascade_band{nowalk, 0});
			}
		}
		else if (kept == nullptr)
		{
			prepare();
		}
		for (std::uint32_t band = 0; band <= group.lodcount; ++band)
		{
			// a band holds the squared distances from its lower bound up to the next one; any of them selects the same lod
			auto const bandmin{band == 0 ? 0.f : lodbounds[group.lodfirst + band - 1]};
			auto const bandmax{band == group.lodcount ? std::numeric_limits<float>::max() : lodbounds[group.lodfirst + band]};
			if ((bandmin > group.distancemax) || (bandmax <= group.distancemin))
			{
				continue;
			}
			m_indirectentry = group.entryfirst + band;
			if ((walks != nullptr) && (walks->bands[band].first != nowalk))
			{
				// walked by another cascade: its draws, for the instances of this entry
				auto const &walk{walks->bands[band]};
				m_indirectdraws.insert(m_indirectdraws.end(), m_cascadedraws.begin() + walk.first, m_cascadedraws.begin() + walk.first + walk.count);
				for (std::uint32_t draw = 0; draw < walk.count; ++draw)
				{
					auto &command{m_indirectcommands.emplace_back(m_cascadecommands[walk.first + draw])};
					command.entry = m_indirectentry;
				}
				// as the walk would count them
				m_renderpass.draw_stats.submodels += static_cast<int>(walk.count);
				m_renderpass.draw_stats.drawcalls += static_cast<int>(walk.count);
				continue;
			}
			auto const walkfirst{m_indirectdraws.size()};
			auto *keptwalk{kept != nullptr ? &kept->bands[shadows ? 1 : 0][band] : nullptr};
			if ((keptwalk != nullptr) && (keptwalk->valid) && (keptwalk->bandmin == bandmin) && (keptwalk->modelversion == modelversion) &&
			    (keptwalk->blending == walk_blending_enabled()))
			{
				// walked in an earlier frame, with the model as it is now
				Replay_Walk(*keptwalk, shadows);
				++m_walkcounts.reused;
			}
			else
			{
				if (false == prepared)
				{
					prepare();
				}
				auto const blending{walk_blending_enabled()};
				m_indirectchanging = false;
				::glPushMatrix();
				::glLoadIdentity();
				model->Root->fSquareDist = bandmin;
				model->Root->ReplacableSet(skins, alpha);
				model->Root->pRoot = model;
				Render(model->Root);
				::glPopMatrix();
				++m_walkcounts.walked;
				if (m_indirectchanging)
				{
					++m_walkcounts.changing;
				}
				if (keptwalk != nullptr)
				{
					keptwalk->valid = (false == m_indirectchanging);
					if (keptwalk->valid)
					{
						keptwalk->draws.assign(m_indirectdraws.begin() + walkfirst, m_indirectdraws.end());
						keptwalk->bandmin = bandmin;
						keptwalk->modelversion = modelversion;
						keptwalk->blending = blending;
					}
					else
					{
						keptwalk->draws.clear();
					}
				}
			}
			if (walks != nullptr)
			{
				walks->bands[band] = cascade_band{static_cast<std::uint32_t>(m_cascadedraws.size()), static_cast<std::uint32_t>(m_indirectdraws.size() - walkfirst)};
				m_cascadedraws.insert(m_cascadedraws.end(), m_indirectdraws.begin() + walkfirst, m_indirectdraws.end());
				m_cascadecommands.insert(m_cascadecommands.end(), m_indirectcommands.begin() + walkfirst, m_indirectcommands.end());
			}
		}
	}
	m_indirectrecording = false;
	m_walkcounts.walktime += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - walkstart).count();
}

void scene_walk::Record_Indirect_Draw(gfx::geometry_handle const &Geometry)
{
	// the walk of Walk_Indirect_Draws(): the draw is made later, from the gpu culling, with this state
	auto &geometry{walk_geometry()};
	gfx::indirect_source source;
	if (geometry.indirect_source(Geometry, source))
	{
		m_indirectcommands.push_back({source.count, 0, source.first, static_cast<std::uint32_t>(source.basevertex), 0, m_indirectentry, (source.indexed ? 1u : 0u), 0});
		m_indirectdraws.push_back({model_ubs, m_indirectmaterial, m_indirectsubmodel, Geometry, source.type, walk_blending_enabled(), geometry.draw_group(Geometry)});
	}
}

void scene_walk::Replay_Walk(walk_band const &Walk, bool const Shadows)
{
	auto &geometry{walk_geometry()};
	for (auto const &walkdraw : Walk.draws)
	{
		// the place of the chunk in its bank is asked for again: the bank may have been let go since and made anew, which the
		// question (as the walk) takes care of, uploading the chunk
		gfx::indirect_source source;
		if (false == geometry.indirect_source(walkdraw.geometry, source))
		{
			continue;
		}
		m_indirectcommands.push_back({source.count, 0, source.first, static_cast<std::uint32_t>(source.basevertex), 0, m_indirectentry, (source.indexed ? 1u : 0u), 0});
		auto &draw{m_indirectdraws.emplace_back(walkdraw)};
		draw.type = source.type;
		draw.group = geometry.draw_group(walkdraw.geometry);
		// the walk sets the placement of the submodel, and before each draw of the colour passes the parameters of its material
		// (the material bind); the rest of the block is what the pass has at the moment, as the walk would leave it
		auto const &walked{walkdraw.ubs};
		draw.ubs = model_ubs;
		draw.ubs.modelview = walked.modelview;
		draw.ubs.modelviewnormal = walked.modelviewnormal;
		if (false == Shadows)
		{
			std::memcpy(&draw.ubs.param[0], &walked.param[0], sizeof(draw.ubs.param));
			draw.ubs.opacity = walked.opacity;
			draw.ubs.alpha_mult = walked.alpha_mult;
		}
	}
	// as the walk would count them
	m_renderpass.draw_stats.submodels += static_cast<int>(Walk.draws.size());
	m_renderpass.draw_stats.drawcalls += static_cast<int>(Walk.draws.size());
}

texture_handle scene_walk::Indirect_Shadow_Texture(material_handle const Material) const
{
	if (Material == null_handle)
	{
		return null_handle;
	}
	return walk_material_data(Material).textures[0];
}

// the order of the recorded draws for multi draws. draws go in one multi draw when they follow one another with the same
// state (material and its texture variant, geometry bank, primitive, blending; Indirect_Mergeable()). the draws of one entry,
// the walk of one submodel tree in one lod band, keep their order: submodels of a model may lie on one another and rely on
// the later one being drawn over the earlier one. the draws of different entries are drawn for different instances, which
// the gpu draws in no particular order anyway, so they're free to move. the entries are queues of draws; the draws at the
// heads of the queues which have the state taken next all go, each queue as far as its draws keep that state, then the
// queues wait for the state of their next draw. the states go in the order in which the queues first reach them
void scene_walk::Order_Indirect_Draws()
{
	auto const count{m_indirectdraws.size()};
	if (count < 2)
	{
		return;
	}

	// a number for each state the draws have, by sorting the states (the buffers are kept between frames)
	m_indirectstatekeys.clear();
	// the sun shadows draw with the program and the texture of the alpha test, whatever the material
	auto const shadows{m_renderpass.draw_mode == rendermode::shadows};
	for (std::size_t index = 0; index < count; ++index)
	{
		auto const &draw{m_indirectdraws[index]};
		if (shadows)
		{
			m_indirectstatekeys.push_back({Indirect_Shadow_Texture(draw.material), 0u, draw.group, draw.type, m_indirectcommands[index].indexed, draw.blending, static_cast<std::uint32_t>(index)});
			continue;
		}
		auto variant{0u};
		if (draw.material != null_handle)
		{
			auto const &material{walk_material_data(draw.material)};
			if (false == material.texture_variants.empty())
			{
				variant = TSubModel::VariantId(draw.submodel) % static_cast<std::uint32_t>(material.texture_variants.size());
			}
		}
		m_indirectstatekeys.push_back({draw.material, variant, draw.group, draw.type, m_indirectcommands[index].indexed, draw.blending, static_cast<std::uint32_t>(index)});
	}
	std::sort(std::begin(m_indirectstatekeys), std::end(m_indirectstatekeys), [](indirect_state_key const &Left, indirect_state_key const &Right) { return Left.tie() < Right.tie(); });
	m_indirectdrawstates.resize(count);
	std::uint32_t statecount{0};
	for (std::size_t index = 0; index < count; ++index)
	{
		if ((index > 0) && (m_indirectstatekeys[index].tie() != m_indirectstatekeys[index - 1].tie()))
		{
			++statecount;
		}
		m_indirectdrawstates[m_indirectstatekeys[index].draw] = statecount;
	}
	++statecount;
	if (statecount == count)
	{
		// no two draws share a state, nothing would go together
		return;
	}

	auto constexpr none{std::numeric_limits<std::uint32_t>::max()};
	m_indirectstates.assign(statecount, indirect_state{none, none});
	m_indirectstateorder.clear();
	m_indirectqueues.clear();
	// the queue waits for the state of its next draw, behind the queues waiting for it already
	auto const enqueue = [this, none](std::uint32_t const Queue) {
		auto &queue{m_indirectqueues[Queue]};
		auto &state{m_indirectstates[m_indirectdrawstates[queue.next]]};
		queue.link = none;
		if (state.head == none)
		{
			state.head = Queue;
			m_indirectstateorder.push_back(m_indirectdrawstates[queue.next]);
		}
		else
		{
			m_indirectqueues[state.tail].link = Queue;
		}
		state.tail = Queue;
	};
	for (std::size_t index = 0; index < count;)
	{
		auto end{index + 1};
		while ((end < count) && (m_indirectcommands[end].entry == m_indirectcommands[index].entry))
		{
			++end;
		}
		m_indirectqueues.push_back({static_cast<std::uint32_t>(index), static_cast<std::uint32_t>(end), none});
		index = end;
	}
	for (std::uint32_t queue = 0; queue < m_indirectqueues.size(); ++queue)
	{
		enqueue(queue);
	}
	m_indirectorder.clear();
	for (std::size_t taken = 0; taken < m_indirectstateorder.size(); ++taken)
	{
		auto const stateindex{m_indirectstateorder[taken]};
		auto queueindex{m_indirectstates[stateindex].head};
		m_indirectstates[stateindex] = {none, none};
		while (queueindex != none)
		{
			auto &queue{m_indirectqueues[queueindex]};
			auto const nextqueue{queue.link};
			while ((queue.next < queue.end) && (m_indirectdrawstates[queue.next] == stateindex))
			{
				m_indirectorder.push_back(queue.next);
				++queue.next;
			}
			if (queue.next < queue.end)
			{
				enqueue(queueindex);
			}
			queueindex = nextqueue;
		}
	}

	m_indirectdrawsordered.clear();
	m_indirectcommandsordered.clear();
	for (auto const index : m_indirectorder)
	{
		m_indirectdrawsordered.push_back(m_indirectdraws[index]);
		m_indirectcommandsordered.push_back(m_indirectcommands[index]);
	}
	m_indirectdraws.swap(m_indirectdrawsordered);
	m_indirectcommands.swap(m_indirectcommandsordered);
}

bool scene_walk::Indirect_Mergeable(std::size_t const First, std::size_t const Other) const
{
	auto const &first{m_indirectdraws[First]};
	auto const &other{m_indirectdraws[Other]};
	if ((first.material != other.material) || (first.blending != other.blending) || (first.type != other.type) || (first.group != other.group) ||
	    (m_indirectcommands[First].indexed != m_indirectcommands[Other].indexed))
	{
		return false;
	}
	if (first.material == null_handle)
	{
		return true;
	}
	// the texture set the material bind binds
	auto const &material{walk_material_data(first.material)};
	if (material.texture_variants.empty())
	{
		return true;
	}
	return (&material.GetTextures(TSubModel::VariantId(first.submodel)) == &material.GetTextures(TSubModel::VariantId(other.submodel)));
}

void scene_walk::Expire_Indirect_Records()
{
	if ((m_framestamp % 256) == 0)
	{
		// the sleeper records of tracks out of sight for a while. their places in the record buffer of the gpu culling are
		// dropped when it starts over
		for (auto records{m_sleeperrecords.begin()}; records != m_sleeperrecords.end();)
		{
			if (m_framestamp - records->second.frame > 256)
			{
				records = m_sleeperrecords.erase(records);
			}
			else
			{
				++records;
			}
		}
		// and so are the records of the models drawn on their own
		for (auto records{m_modelrecords.begin()}; records != m_modelrecords.end();)
		{
			if (m_framestamp - records->second.frame > 256)
			{
				records = m_modelrecords.erase(records);
			}
			else
			{
				++records;
			}
		}
	}
}

void scene_walk::Clear_Indirect_Records()
{
	m_sleeperrecords.clear();
	m_modelrecords.clear();
	m_cascaderecords.clear();
	m_walkrecords.clear();
}

} // namespace gfx
