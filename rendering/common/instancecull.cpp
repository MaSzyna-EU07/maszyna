/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/common/instancecull.h"
#include "rendering/common/walksync.h"

#include "model/AnimModel.h"
#include "model/Model3d.h"
#include "rendering/frustum.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"

namespace gfx
{

namespace
{

constexpr std::int64_t RECORD_SIZE{sizeof(scene::basic_cell::instance_record)};

} // namespace

instance_cull::instance_cull() : m_records{std::make_shared<record_state>()}
{
	m_readbackdata.resize(gl::INSTANCE_COUNTER_HEADER + READBACK_ENTRIES);
}

void instance_cull::begin_pass(pass_setup const &Setup)
{
	collect();

	m_inpass = true;
	m_ran = true;
	m_dispatched = false;
	m_setup = Setup;
	m_buckets.clear();
	m_joblist.clear();
	m_grouplist.clear();
	m_drawgroups.clear();
	m_lodlist.clear();
	m_groupmap.clear();
	m_sleepergroupmap.clear();
	m_cpucounts.clear();
	m_candidates = 0;
	m_entries = 0;
	m_sleepertracks = 0;
	m_sleepers = 0;
	m_models = 0;
	m_counts = {};
}

void instance_cull::add_bucket(scene::basic_cell::instance_bucket_key const &Key, scene::basic_cell::instance_bucket const &Bucket, scene::bounding_area const &Area)
{
	add(Key, Bucket, Area, false);
}

void instance_cull::skip_bucket(scene::basic_cell::instance_bucket const &Bucket)
{
	if (!m_inpass)
		return;
	++m_counts.skippedbuckets;
	m_counts.skippedinstances += Bucket.records.size();
}

void instance_cull::add_sleepers(scene::basic_cell::instance_bucket_key const &Key, scene::basic_cell::instance_bucket const &Bucket, scene::bounding_area const &Area)
{
	add(Key, Bucket, Area, true);
}

void instance_cull::add(scene::basic_cell::instance_bucket_key const &Key, scene::basic_cell::instance_bucket const &Bucket, scene::bounding_area const &Area, bool const Sleepers)
{
	if (!m_inpass || Bucket.records.empty() || Key.pModel == nullptr)
		return;

	// range of the squared distances the instances of the bucket can be at, scaled as in the culling, with some slack for
	// its single precision: the draws are made only for the lod bands within it. sleepers go by the plain distance from the
	// camera of the pass
	auto const &distancecamera{Sleepers ? m_setup.camera : m_setup.distancecamera};
	auto const distancescale{Sleepers ? 1.0 : static_cast<double>(m_setup.distancescale)};
	auto const distanceoffset{Sleepers ? 0.0 : static_cast<double>(m_setup.distanceoffset)};
	auto const centerdistance{glm::length(Area.center - distancecamera)};
	auto const nearest{std::max(0.0, centerdistance - Area.radius)};
	auto const farthest{centerdistance + Area.radius};
	auto const distancemin{static_cast<float>(0.999 * (nearest * nearest * distancescale + distanceoffset))};
	auto const distancemax{static_cast<float>(1.001 * (farthest * farthest * distancescale + distanceoffset))};

	auto &groupmap{Sleepers ? m_sleepergroupmap : m_groupmap};
	auto lookup{groupmap.find(Key)};
	if (lookup == groupmap.end())
	{
		// lod bounds of the model, the same set Render_Instanced() splits the distance into bands with, found once for the model
		auto const lodfirst{static_cast<std::uint32_t>(m_lodlist.size())};
		std::uint32_t lodcount{0};
		{
			// found by the first pass which asks, of the passes walked at the same time (walk_sync)
			gfx::walk_sync::model_hold hold{Key.pModel};
			auto const &lodbounds{Key.pModel->lod_bounds()};
			m_lodlist.insert(m_lodlist.end(), lodbounds.begin(), lodbounds.end());
			lodcount = static_cast<std::uint32_t>(lodbounds.size());
		}

		group_data const group{static_cast<std::uint32_t>(m_grouplist.size()), m_entries, lodcount + 1, distancemin, distancemax};
		m_grouplist.push_back({lodfirst, lodcount, m_entries, 0});
		m_entries += group.entrycount;
		lookup = groupmap.emplace(Key, group).first;
		if (m_setup.drawing)
		{
			// any instance of the group serves; it's only prepared and asked for its material
			TAnimModel *instance{nullptr};
			for (auto *candidate : Bucket.instances)
			{
				if (candidate != nullptr)
				{
					instance = candidate;
					break;
				}
			}
			m_drawgroups.push_back({&lookup->first, instance, Sleepers, lodfirst, lodcount, group.entryfirst, distancemin, distancemax});
		}
	}
	else
	{
		lookup->second.distancemin = std::min(lookup->second.distancemin, distancemin);
		lookup->second.distancemax = std::max(lookup->second.distancemax, distancemax);
		if (m_setup.drawing)
		{
			auto &drawgroup{m_drawgroups[lookup->second.index]};
			drawgroup.distancemin = lookup->second.distancemin;
			drawgroup.distancemax = lookup->second.distancemax;
			if (drawgroup.instance == nullptr)
			{
				for (auto *candidate : Bucket.instances)
				{
					if (candidate != nullptr)
					{
						drawgroup.instance = candidate;
						break;
					}
				}
			}
		}
	}

	{
		// the places in the record buffer the passes share are given by one walk at a time (walk_sync)
		auto const &shared{*m_records};
		gfx::walk_sync::change change{(Bucket.gpugeneration != shared.recordgeneration) || Bucket.gpustale};
		place(Bucket);
	}

	gl::instance_cull_job job{};
	job.origin = packed_vec3{Bucket.recordorigin - m_setup.camera};
	job.first = Bucket.gpufirst;
	job.distanceorigin = packed_vec3{Bucket.recordorigin - m_setup.distancecamera};
	job.count = static_cast<std::uint32_t>(Bucket.records.size());
	job.group = lookup->second.index;
	job.candidatestart = m_candidates;
	job.flags = (Sleepers ? gl::INSTANCE_JOB_SLEEPERS : 0u);
	m_joblist.push_back(job);
	m_buckets.push_back(&Bucket);
	m_candidates += job.count;
	if (Sleepers)
	{
		++m_sleepertracks;
		m_sleepers += job.count;
	}
	else if (Key.instance != nullptr)
	{
		++m_models;
	}
}

void instance_cull::place(scene::basic_cell::instance_bucket const &Bucket)
{
	auto &shared{*m_records};
	auto const count{static_cast<std::uint32_t>(Bucket.records.size())};
	auto const current{Bucket.gpugeneration == shared.recordgeneration};
	if (current && !Bucket.gpustale)
		return;
	if (current && count <= Bucket.gpucapacity)
	{
		send_records(Bucket.gpufirst, Bucket.records.data(), count);
		m_counts.sentrecords += count;
		Bucket.gpustale = false;
		return;
	}

	// a new place, with some room for instances the editor adds
	auto const allocate = [this, &shared](scene::basic_cell::instance_bucket const &Target) {
		auto const records{static_cast<std::uint32_t>(Target.records.size())};
		Target.gpufirst = shared.recordfill;
		Target.gpucapacity = records + records / 4;
		Target.gpugeneration = shared.recordgeneration;
		Target.gpustale = false;
		shared.recordfill += Target.gpucapacity;
		send_records(Target.gpufirst, Target.records.data(), records);
		m_counts.sentrecords += records;
	};

	auto const capacity{static_cast<std::uint32_t>(record_buffer_size() / RECORD_SIZE)};
	if (shared.recordfill + count + count / 4 > capacity)
	{
		// the buffer is started over, and only the buckets of this pass get places again. the places left behind by
		// buckets which were made anew or are out of sight are dropped this way. the passes of a frame queue different
		// buckets: when those of all of them don't fit in the buffer together, some pass would start it over in every frame,
		// so a buffer started over again in the same or the next frame grows
		std::int64_t live{count + count / 4};
		for (auto const *bucket : m_buckets)
			live += bucket->records.size() + bucket->records.size() / 4;
		auto const again{shared.restarted && (m_setup.frame - shared.restartframe <= 1)};
		if ((2 * live * RECORD_SIZE > record_buffer_size()) || again)
			resize_record_buffer(std::max(2 * live * RECORD_SIZE, 2 * record_buffer_size()));
		else
			restart_records();
		shared.restarted = true;
		shared.restartframe = m_setup.frame;
		m_counts.restarted = true;
		++shared.recordgeneration;
		shared.recordfill = 0;
		for (std::size_t job = 0; job < m_buckets.size(); ++job)
		{
			allocate(*m_buckets[job]);
			m_joblist[job].first = m_buckets[job]->gpufirst;
		}
	}
	allocate(Bucket);
}

void instance_cull::refresh_places()
{
	auto const &shared{*m_records};
	auto const moved{std::any_of(m_buckets.begin(), m_buckets.end(), [&](scene::basic_cell::instance_bucket const *Bucket) { return (Bucket->gpugeneration != shared.recordgeneration) || Bucket->gpustale; })};
	if (moved)
	{
		for (std::size_t job = 0; job < m_buckets.size(); ++job)
		{
			place(*m_buckets[job]);
		}
	}
	// another pass which started the buffer over placed the buckets it shares with this one anew, current again
	for (std::size_t job = 0; job < m_buckets.size(); ++job)
	{
		m_joblist[job].first = m_buckets[job]->gpufirst;
	}
}

bool instance_cull::dispatch_begin()
{
	m_occlusion = false;
	m_twophase = false;
	return (m_inpass && !m_joblist.empty());
}

void instance_cull::frustum_planes(glm::vec4 (&Planes)[6]) const
{
	auto const &frustumplanes{m_setup.frustum->planes()};
	for (int plane = 0; plane < 6; ++plane)
	{
		auto const *source{frustumplanes[plane]};
		Planes[plane] = glm::vec4{source[0], source[1], source[2],
		                          static_cast<float>(source[0] * m_setup.camera.x + source[1] * m_setup.camera.y + source[2] * m_setup.camera.z + source[3])};
	}
}

void instance_cull::dispatch_end()
{
	m_cpucounts.assign(m_entries, 0);
	m_dispatched = true;
}

void instance_cull::draw_result(std::uint32_t const Draws, std::uint32_t const Prepassdraws, std::uint32_t const Calls, std::uint32_t const Prepasscalls, double const Cputime)
{
	m_draws = Draws;
	m_prepassdraws = Prepassdraws;
	m_calls = Calls;
	m_prepasscalls = Prepasscalls;
	m_cputime = Cputime;
}

void instance_cull::cpu_result(scene::basic_cell::instance_bucket_key const &Key, std::vector<std::uint32_t> const &Bandcounts, bool const Sleepers)
{
	if (!m_dispatched)
		return;
	auto const &groupmap{Sleepers ? m_sleepergroupmap : m_groupmap};
	auto const lookup{groupmap.find(Key)};
	if (lookup == groupmap.end())
		return;
	auto const &group{lookup->second};
	for (std::size_t band = 0; band < std::min<std::size_t>(Bandcounts.size(), group.entrycount); ++band)
		m_cpucounts[group.entryfirst + band] += Bandcounts[band];
}

void instance_cull::cpu_instance(scene::basic_cell::instance_bucket_key const &Key, float const Distancesquared)
{
	if (!m_dispatched)
		return;
	auto const lookup{m_groupmap.find(Key)};
	if (lookup == m_groupmap.end())
		return;
	auto const &group{lookup->second};
	auto const &lods{m_grouplist[group.index]};
	// the band the culling gives the distance: the number of bounds it reached
	auto const first{m_lodlist.begin() + lods.lodfirst};
	auto const band{static_cast<std::uint32_t>(std::upper_bound(first, first + lods.lodcount, Distancesquared) - first)};
	if (band < group.entrycount)
		++m_cpucounts[group.entryfirst + band];
}

void instance_cull::end_pass()
{
	// the counts of both phases, for the panel and the comparison
	if (m_dispatched)
		push_counts();
	// the cpu counts are there to compare with only when the cpu drew
	if (m_dispatched && !m_setup.drawing)
	{
		std::uint64_t total{0};
		for (auto const count : m_cpucounts)
			total += count;
		m_cpupasses.push_back({m_setup.frame, m_cpucounts, total});
		// the gpu is never this far behind
		while (m_cpupasses.size() > 8)
			m_cpupasses.pop_front();
	}
	m_lastsleepertracks = m_sleepertracks;
	m_lastsleepers = m_sleepers;
	m_lastmodels = m_models;
	m_counts.jobs = static_cast<std::uint32_t>(m_joblist.size());
	m_counts.candidates = m_candidates;
	m_lastcounts = m_counts;
	m_inpass = false;
	m_dispatched = false;
}

void instance_cull::collect()
{
	if (!pop_counts(m_readbackdata.data()))
		return;

	auto const frame{m_readbackdata[0]};
	comparison result;
	result.frame = frame;
	result.occluded = m_readbackdata[3];
	result.gputotal = m_readbackdata[1] - std::min(m_readbackdata[1], result.occluded);
	result.entries = m_readbackdata[2];
	result.first = m_readbackdata[4];
	while (!m_cpupasses.empty() && m_cpupasses.front().frame != frame)
		m_cpupasses.pop_front();
	if (m_cpupasses.empty())
	{
		// drawn from the gpu: nothing to compare with
		m_last = result;
		return;
	}

	auto const &cpu{m_cpupasses.front()};
	result.cputotal = cpu.total;
	result.compared = std::min<std::uint32_t>({result.entries, READBACK_ENTRIES, static_cast<std::uint32_t>(cpu.counts.size())});
	for (std::uint32_t entry = 0; entry < result.compared; ++entry)
	{
		auto const gpu{m_readbackdata[gl::INSTANCE_COUNTER_HEADER + entry]};
		auto const difference{gpu > cpu.counts[entry] ? gpu - cpu.counts[entry] : cpu.counts[entry] - gpu};
		if (difference > 0)
		{
			++result.mismatched;
			result.largestdifference = std::max(result.largestdifference, difference);
		}
	}
	m_cpupasses.pop_front();

	m_last = result;
	++m_comparedframes;
	if ((result.mismatched > 0) || (result.gputotal != result.cputotal) || (result.entries != cpu.counts.size()))
		++m_mismatchedframes;
}

std::string instance_cull::info(std::string const &Label, bool const Occlusion)
{
	if (Global.gfx_gpucull == 0)
		return {};
	// the sleepers are among the instances counted
	auto sleepers{m_lastsleepertracks > 0 ? "\n" + Label + ": " + std::to_string(m_lastsleepers) + " sleepers of " + std::to_string(m_lastsleepertracks) + " tracks queued" : std::string{}};
	// and so are the models drawn on their own
	if (m_lastmodels > 0)
		sleepers += "\n" + Label + ": " + std::to_string(m_lastmodels) + " models drawn on their own queued";
	// what the pass tested and sent to the gpu
	sleepers += "\n" + Label + ": " + std::to_string(m_lastcounts.candidates) + " instances tested in " + std::to_string(m_lastcounts.jobs) + " buckets";
	if (Global.gfx_gpucull_bucketrange)
		sleepers += ", " + std::to_string(m_lastcounts.skippedbuckets) + " buckets (" + std::to_string(m_lastcounts.skippedinstances) + " instances) left out before the culling";
	sleepers += ", " + std::to_string(m_lastcounts.sentrecords) + " records sent" + (m_lastcounts.restarted ? ", record buffer started over" : "");
	if (Global.gfx_gpucull == 1)
	{
		if (m_last.frame == 0)
			return "\n" + Label + ": no result yet" + sleepers;
		if (false == Occlusion)
			return "\n" + Label + ": " + std::to_string(m_last.gputotal) + " instances drawn, " + std::to_string(m_last.entries) + " bands; " + std::to_string(m_draws) +
			       " indirect draws in " + std::to_string(m_calls) + " calls, cpu " + to_string(m_cputime, 3) + " ms" + sleepers;
		return "\n" + Label + ": " + std::to_string(m_last.gputotal) + " instances drawn" +
		       (m_twophase ? " (" + std::to_string(m_last.first) + " of them first, seen in the last frame)" : std::string{}) + ", " + std::to_string(m_last.occluded) +
		       " hidden by occlusion" + (m_occlusion ? (m_twophase ? " (also by one another)" : "") : " (test off)") + ", " + std::to_string(m_last.entries) + " bands; " +
		       std::to_string(m_draws) + " indirect draws in " + std::to_string(m_calls) + " calls (+" + std::to_string(m_prepassdraws) + " depth prepass in " +
		       std::to_string(m_prepasscalls) + "), cpu " + to_string(m_cputime, 3) + " ms" + sleepers;
	}
	if (m_comparedframes == 0)
		return "\n" + Label + ": no result yet" + sleepers;
	return "\n" + Label + ": " + std::to_string(m_last.gputotal) + " instances (cpu " + std::to_string(m_last.cputotal) + "), " + std::to_string(m_last.entries) + " bands, " +
	       std::to_string(m_last.mismatched) + " differ (by up to " + std::to_string(m_last.largestdifference) + "); frames differing " + std::to_string(m_mismatchedframes) + " of " +
	       std::to_string(m_comparedframes) + sleepers;
}

} // namespace gfx
