/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46instancecull.h"

#include "model/AnimModel.h"
#include "model/Model3d.h"
#include "model/Texture.h"
#include "rendering/frustum.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"

namespace
{

constexpr GLsizeiptr RECORD_SIZE{sizeof(scene::basic_cell::instance_record)};

// grows a buffer to hold at least Size bytes, at least doubling it. a new buffer drops the contents and the binding
void reserve(gl::ssbo &Buffer, GLsizeiptr const Size)
{
	if (Size > Buffer.size())
		Buffer.resize(std::max(Size, 2 * Buffer.size()));
}

} // namespace

void opengl46_instance_cull::init(opengl46_instance_cull const *Shared)
{
	if (Shared != nullptr)
	{
		m_shared = Shared->m_shared;
	}
	else
	{
		auto const make_compute = [](std::string const &Name) {
			gl::shader shader(Name);
			return std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{shader});
		};
		m_shared = std::make_shared<shared_data>();
		m_shared->cull_program = make_compute("instance_cull.comp");
		m_shared->offsets_program = make_compute("instance_cull_offsets.comp");
		m_shared->commands_program = make_compute("instance_cull_commands.comp");
		m_shared->records = std::make_unique<gl::ssbo>(65536 * RECORD_SIZE, gl::INSTANCE_RECORD_SSBO_BINDING);
	}

	auto const readbacksize{static_cast<GLsizeiptr>((gl::INSTANCE_COUNTER_HEADER + READBACK_ENTRIES) * sizeof(std::uint32_t))};
	m_jobs = std::make_unique<gl::ssbo>(4096 * sizeof(gl::instance_cull_job), gl::INSTANCE_JOB_SSBO_BINDING);
	m_groups = std::make_unique<gl::ssbo>(1024 * sizeof(gl::instance_cull_group), gl::INSTANCE_GROUP_SSBO_BINDING);
	m_lods = std::make_unique<gl::ssbo>(4096 * sizeof(float), gl::INSTANCE_LOD_SSBO_BINDING);
	// the readback copies a fixed size from it
	m_counters = std::make_unique<gl::ssbo>(readbacksize, gl::INSTANCE_COUNTER_SSBO_BINDING);
	m_output = std::make_unique<gl::ssbo>(65536 * sizeof(glm::mat4), gl::INSTANCE_OUTPUT_SSBO_BINDING);
	m_commands = std::make_unique<gl::ssbo>(4096 * sizeof(gl::instance_draw_command), gl::INSTANCE_COMMAND_SSBO_BINDING);
	m_readback = std::make_unique<gl::readback>(readbacksize);
	m_readbackdata.resize(gl::INSTANCE_COUNTER_HEADER + READBACK_ENTRIES);
}

void opengl46_instance_cull::begin_pass(pass_setup const &Setup)
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
}

void opengl46_instance_cull::add_bucket(scene::basic_cell::instance_bucket_key const &Key, scene::basic_cell::instance_bucket const &Bucket, scene::bounding_area const &Area)
{
	add(Key, Bucket, Area, false);
}

void opengl46_instance_cull::add_sleepers(scene::basic_cell::instance_bucket_key const &Key, scene::basic_cell::instance_bucket const &Bucket, scene::bounding_area const &Area)
{
	add(Key, Bucket, Area, true);
}

void opengl46_instance_cull::add(scene::basic_cell::instance_bucket_key const &Key, scene::basic_cell::instance_bucket const &Bucket, scene::bounding_area const &Area, bool const Sleepers)
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
		// lod bounds of the model, the same set Render_Instanced() splits the distance into bands with
		auto const lodfirst{static_cast<std::uint32_t>(m_lodlist.size())};
		m_pending.clear();
		m_pending.push_back(Key.pModel->Root);
		while (!m_pending.empty())
		{
			auto const *submodel{m_pending.back()};
			m_pending.pop_back();
			if (submodel == nullptr)
				continue;
			if (submodel->fSquareMinDist > 0.f)
				m_lodlist.emplace_back(submodel->fSquareMinDist);
			if (submodel->fSquareMaxDist < std::numeric_limits<float>::max())
				m_lodlist.emplace_back(submodel->fSquareMaxDist);
			m_pending.push_back(submodel->Child);
			m_pending.push_back(submodel->Next);
		}
		std::sort(m_lodlist.begin() + lodfirst, m_lodlist.end());
		m_lodlist.erase(std::unique(m_lodlist.begin() + lodfirst, m_lodlist.end()), m_lodlist.end());
		auto const lodcount{static_cast<std::uint32_t>(m_lodlist.size()) - lodfirst};

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

	place(Bucket);

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

void opengl46_instance_cull::place(scene::basic_cell::instance_bucket const &Bucket)
{
	auto &shared{*m_shared};
	auto const count{static_cast<std::uint32_t>(Bucket.records.size())};
	auto const current{Bucket.gpugeneration == shared.recordgeneration};
	if (current && !Bucket.gpustale)
		return;
	if (current && count <= Bucket.gpucapacity)
	{
		shared.records->update(Bucket.records.data(), static_cast<GLintptr>(Bucket.gpufirst) * RECORD_SIZE, static_cast<GLsizeiptr>(count) * RECORD_SIZE);
		Bucket.gpustale = false;
		return;
	}

	// a new place, with some room for instances the editor adds
	auto const allocate = [&shared](scene::basic_cell::instance_bucket const &Target) {
		auto const records{static_cast<std::uint32_t>(Target.records.size())};
		Target.gpufirst = shared.recordfill;
		Target.gpucapacity = records + records / 4;
		Target.gpugeneration = shared.recordgeneration;
		Target.gpustale = false;
		shared.recordfill += Target.gpucapacity;
		shared.records->update(Target.records.data(), static_cast<GLintptr>(Target.gpufirst) * RECORD_SIZE, static_cast<GLsizeiptr>(records) * RECORD_SIZE);
	};

	auto const capacity{static_cast<std::uint32_t>(shared.records->size() / RECORD_SIZE)};
	if (shared.recordfill + count + count / 4 > capacity)
	{
		// the buffer is started over, and only the buckets of this pass get places again. the places left behind by
		// buckets which were made anew or are out of sight are dropped this way. the passes of a frame queue different
		// buckets: when those of all of them don't fit in the buffer together, some pass would start it over in every frame,
		// so a buffer started over again in the same or the next frame grows
		GLsizeiptr live{count + count / 4};
		for (auto const *bucket : m_buckets)
			live += bucket->records.size() + bucket->records.size() / 4;
		auto const again{shared.restarted && (m_setup.frame - shared.restartframe <= 1)};
		if ((2 * live * RECORD_SIZE > shared.records->size()) || again)
			shared.records->resize(std::max(2 * live * RECORD_SIZE, 2 * shared.records->size()));
		shared.restarted = true;
		shared.restartframe = m_setup.frame;
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

void opengl46_instance_cull::dispatch(glm::mat4 const &View, occlusion_setup const *Occlusion, bool const Twophase)
{
	m_occlusion = false;
	m_twophase = false;
	if (!m_inpass || m_joblist.empty())
		return;

	auto const countersize{static_cast<GLsizeiptr>((gl::INSTANCE_COUNTER_HEADER + 3 * m_entries) * sizeof(std::uint32_t))};
	reserve(*m_jobs, static_cast<GLsizeiptr>(m_joblist.size() * sizeof(gl::instance_cull_job)));
	reserve(*m_groups, static_cast<GLsizeiptr>(m_grouplist.size() * sizeof(gl::instance_cull_group)));
	reserve(*m_lods, static_cast<GLsizeiptr>(std::max<std::size_t>(m_lodlist.size(), 1) * sizeof(float)));
	reserve(*m_counters, countersize);
	reserve(*m_output, static_cast<GLsizeiptr>(m_candidates) * static_cast<GLsizeiptr>(sizeof(glm::mat4)));

	m_jobs->update(m_joblist.data(), 0, static_cast<GLsizeiptr>(m_joblist.size() * sizeof(gl::instance_cull_job)));
	m_groups->update(m_grouplist.data(), 0, static_cast<GLsizeiptr>(m_grouplist.size() * sizeof(gl::instance_cull_group)));
	if (!m_lodlist.empty())
		m_lods->update(m_lodlist.data(), 0, static_cast<GLsizeiptr>(m_lodlist.size() * sizeof(float)));
	// counts and cursors start at zero
	glClearNamedBufferSubData(*m_counters, GL_R32UI, 0, countersize, GL_RED_INTEGER, GL_UNSIGNED_INT, nullptr);
	std::uint32_t const header[gl::INSTANCE_COUNTER_HEADER]{m_setup.frame, 0, m_entries, 0, 0, 0, 0, 0};
	m_counters->update(header, 0, sizeof(header));

	bind_buffers();

	// planes moved to the camera of the pass, in double precision: the cpu tests world coordinates
	auto const &frustumplanes{m_setup.frustum->planes()};
	glm::vec4 planes[6];
	for (int plane = 0; plane < 6; ++plane)
	{
		auto const *source{frustumplanes[plane]};
		planes[plane] = glm::vec4{source[0], source[1], source[2],
		                          static_cast<float>(source[0] * m_setup.camera.x + source[1] * m_setup.camera.y + source[2] * m_setup.camera.z + source[3])};
	}
	glProgramUniformMatrix4fv(*m_shared->cull_program, 0, 1, GL_FALSE, glm::value_ptr(View));
	glProgramUniform4fv(*m_shared->cull_program, 1, 6, glm::value_ptr(planes[0]));
	glProgramUniform4f(*m_shared->cull_program, 7, m_setup.distancescale, m_setup.distanceoffset, m_setup.distancerange, m_setup.drawrange);
	glProgramUniform4f(*m_shared->cull_program, 8, m_setup.zoom, 0.f, 0.f, 0.f);
	glProgramUniform4ui(*m_shared->offsets_program, 0, m_entries, 0, 0, 0);
	// the occlusion test only when the instances are drawn from the result, the cpu doesn't make it
	m_occlusion = (m_setup.drawing && (Occlusion != nullptr) && (Occlusion->pyramid != 0));
	if (m_occlusion)
	{
		glProgramUniformMatrix4fv(*m_shared->cull_program, 10, 1, GL_FALSE, glm::value_ptr(Occlusion->projection));
		glProgramUniform4i(*m_shared->cull_program, 14, 1, Occlusion->levels, Occlusion->size.x, Occlusion->size.y);
		glProgramUniform4f(*m_shared->cull_program, 15, Occlusion->screen.x, Occlusion->screen.y, 0.f, 0.f);
		glBindTextureUnit(0, Occlusion->pyramid);
	}
	else
	{
		glProgramUniform4i(*m_shared->cull_program, 14, 0, 0, 0, 0);
	}
	m_twophase = (m_occlusion && Twophase);
	glProgramUniform4ui(*m_shared->cull_program, 16, (m_twophase ? 1u : 0u), 0, 0, 0);

	// one invocation per record; past the count limit of one dimension the groups go into rows
	auto const groups{(m_candidates + gl::INSTANCE_CULL_GROUP_SIZE - 1) / gl::INSTANCE_CULL_GROUP_SIZE};
	auto const groups_x{std::min<GLuint>(groups, 65535)};
	auto const groups_y{(groups + groups_x - 1) / groups_x};

	m_timer.begin();
	glProgramUniform4ui(*m_shared->cull_program, 9, static_cast<GLuint>(m_joblist.size()), m_candidates, m_entries, 0);
	m_shared->cull_program->dispatch(groups_x, groups_y);
	gl::memory_barrier(GL_SHADER_STORAGE_BARRIER_BIT);
	m_shared->offsets_program->dispatch(1);
	gl::memory_barrier(GL_SHADER_STORAGE_BARRIER_BIT);
	glProgramUniform4ui(*m_shared->cull_program, 9, static_cast<GLuint>(m_joblist.size()), m_candidates, m_entries, 1);
	m_shared->cull_program->dispatch(groups_x, groups_y);
	// the output and the counts go to draws, indirect draws and the copy for the cpu
	gl::memory_barrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT | GL_COMMAND_BARRIER_BIT);
	m_timer.end();
	if (m_occlusion)
	{
		glBindTextureUnit(0, 0);
		opengl_texture::reset_unit_cache();
	}

	m_cpucounts.assign(m_entries, 0);
	m_dispatched = true;
}

void opengl46_instance_cull::bind_buffers()
{
	m_shared->records->bind_base();
	m_jobs->bind_base();
	m_groups->bind_base();
	m_lods->bind_base();
	m_counters->bind_base();
	m_output->bind_base();
}

void opengl46_instance_cull::dispatch_second(occlusion_setup const &Occlusion)
{
	if (!m_dispatched || !m_twophase)
		return;

	// the draws of the first phase bound buffers of their own, and other passes share the binding of the records
	bind_buffers();
	glProgramUniform4i(*m_shared->cull_program, 14, 1, Occlusion.levels, Occlusion.size.x, Occlusion.size.y);
	glProgramUniform4f(*m_shared->cull_program, 15, Occlusion.screen.x, Occlusion.screen.y, 0.f, 0.f);
	glBindTextureUnit(0, Occlusion.pyramid);

	auto const groups{(m_candidates + gl::INSTANCE_CULL_GROUP_SIZE - 1) / gl::INSTANCE_CULL_GROUP_SIZE};
	auto const groups_x{std::min<GLuint>(groups, 65535)};
	auto const groups_y{(groups + groups_x - 1) / groups_x};

	m_secondtimer.begin();
	glProgramUniform4ui(*m_shared->cull_program, 9, static_cast<GLuint>(m_joblist.size()), m_candidates, m_entries, 2);
	m_shared->cull_program->dispatch(groups_x, groups_y);
	gl::memory_barrier(GL_SHADER_STORAGE_BARRIER_BIT);
	if (m_commandcount > 0)
	{
		m_commands->bind_base();
		glProgramUniform4ui(*m_shared->commands_program, 0, m_commandcount, m_entries, 1, 0);
		m_shared->commands_program->dispatch((m_commandcount + gl::INSTANCE_COMMAND_GROUP_SIZE - 1) / gl::INSTANCE_COMMAND_GROUP_SIZE);
	}
	// the commands go to indirect draws, the matrices to the vertex shaders, the counts to the copy for the cpu
	gl::memory_barrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT | GL_COMMAND_BARRIER_BIT);
	m_secondtimer.end();

	glBindTextureUnit(0, 0);
	opengl_texture::reset_unit_cache();
}

void opengl46_instance_cull::fill_commands(std::vector<gl::instance_draw_command> const &Commands)
{
	m_commandcount = static_cast<std::uint32_t>(Commands.size());
	if (!m_dispatched || Commands.empty())
		return;

	// the first copy is sent, the others are made from it on the gpu
	auto const size{static_cast<GLsizeiptr>(Commands.size() * sizeof(gl::instance_draw_command))};
	reserve(*m_commands, 3 * size);
	m_commands->update(Commands.data(), 0, size);
	m_counters->bind_base();
	m_commands->bind_base();
	glProgramUniform4ui(*m_shared->commands_program, 0, m_commandcount, m_entries, 0, 0);
	m_shared->commands_program->dispatch((m_commandcount + gl::INSTANCE_COMMAND_GROUP_SIZE - 1) / gl::INSTANCE_COMMAND_GROUP_SIZE);
	// the commands go to indirect draws, the matrices to the vertex shaders
	gl::memory_barrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_COMMAND_BARRIER_BIT);
}

void opengl46_instance_cull::bind_draw()
{
	glBindBuffer(GL_DRAW_INDIRECT_BUFFER, *m_commands);
	// the way gl::ssbo binds, leaving the general binding point alone
	GLuint const output{*m_output};
	glBindBuffersBase(GL_SHADER_STORAGE_BUFFER, gl::INSTANCE_SSBO_BINDING, 1, &output);
}

void opengl46_instance_cull::draw_result(std::uint32_t const Draws, std::uint32_t const Prepassdraws, std::uint32_t const Calls, std::uint32_t const Prepasscalls, double const Cputime)
{
	m_draws = Draws;
	m_prepassdraws = Prepassdraws;
	m_calls = Calls;
	m_prepasscalls = Prepasscalls;
	m_cputime = Cputime;
}

void opengl46_instance_cull::cpu_result(scene::basic_cell::instance_bucket_key const &Key, std::vector<std::uint32_t> const &Bandcounts, bool const Sleepers)
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

void opengl46_instance_cull::cpu_instance(scene::basic_cell::instance_bucket_key const &Key, float const Distancesquared)
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

void opengl46_instance_cull::end_pass()
{
	// the counts of both phases, for the panel and the comparison
	if (m_dispatched)
		m_readback->push(*m_counters);
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
	m_inpass = false;
	m_dispatched = false;
}

void opengl46_instance_cull::collect()
{
	if (!m_readback || !m_readback->pop(m_readbackdata.data()))
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

std::string opengl46_instance_cull::info(std::string const &Label, bool const Occlusion)
{
	if (Global.gfx_gpucull == 0)
		return {};
	// the sleepers are among the instances counted
	auto sleepers{m_lastsleepertracks > 0 ? "\n" + Label + ": " + std::to_string(m_lastsleepers) + " sleepers of " + std::to_string(m_lastsleepertracks) + " tracks queued" : std::string{}};
	// and so are the models drawn on their own
	if (m_lastmodels > 0)
		sleepers += "\n" + Label + ": " + std::to_string(m_lastmodels) + " models drawn on their own queued";
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

std::string opengl46_instance_cull::info_times(std::string const &Label)
{
	std::string text;
	if (Global.gfx_gpucull == 0)
		return text;
	if (auto const time = m_timer.result())
	{
		text += " " + Label + ": " + to_string(static_cast<double>(*time) / 1000000.0, 3) + " ms";
		auto const second{m_secondtimer.result()};
		if (m_twophase && second)
			text += " (+" + to_string(static_cast<double>(*second) / 1000000.0, 3) + " ms second phase)";
		text += "\n";
	}
	return text;
}
