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

#include "model/Model3d.h"
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

void opengl46_instance_cull::init()
{
	auto const make_compute = [](std::string const &Name) {
		gl::shader shader(Name);
		return std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{shader});
	};
	m_cull_program = make_compute("instance_cull.comp");
	m_offsets_program = make_compute("instance_cull_offsets.comp");

	auto const readbacksize{static_cast<GLsizeiptr>((gl::INSTANCE_COUNTER_HEADER + READBACK_ENTRIES) * sizeof(std::uint32_t))};
	m_records = std::make_unique<gl::ssbo>(65536 * RECORD_SIZE, gl::INSTANCE_RECORD_SSBO_BINDING);
	m_jobs = std::make_unique<gl::ssbo>(4096 * sizeof(gl::instance_cull_job), gl::INSTANCE_JOB_SSBO_BINDING);
	m_groups = std::make_unique<gl::ssbo>(1024 * sizeof(gl::instance_cull_group), gl::INSTANCE_GROUP_SSBO_BINDING);
	m_lods = std::make_unique<gl::ssbo>(4096 * sizeof(float), gl::INSTANCE_LOD_SSBO_BINDING);
	// the readback copies a fixed size from it
	m_counters = std::make_unique<gl::ssbo>(readbacksize, gl::INSTANCE_COUNTER_SSBO_BINDING);
	m_output = std::make_unique<gl::ssbo>(65536 * sizeof(glm::mat4), gl::INSTANCE_OUTPUT_SSBO_BINDING);
	m_readback = std::make_unique<gl::readback>(readbacksize);
	m_readbackdata.resize(gl::INSTANCE_COUNTER_HEADER + READBACK_ENTRIES);
}

void opengl46_instance_cull::begin_pass(pass_setup const &Setup)
{
	collect();

	m_inpass = true;
	m_dispatched = false;
	m_setup = Setup;
	m_buckets.clear();
	m_joblist.clear();
	m_grouplist.clear();
	m_lodlist.clear();
	m_groupmap.clear();
	m_cpucounts.clear();
	m_candidates = 0;
	m_entries = 0;
}

void opengl46_instance_cull::add_bucket(scene::basic_cell::instance_bucket_key const &Key, scene::basic_cell::instance_bucket const &Bucket)
{
	if (!m_inpass || Bucket.records.empty() || Key.pModel == nullptr)
		return;

	auto lookup{m_groupmap.find(Key)};
	if (lookup == m_groupmap.end())
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

		group_data const group{static_cast<std::uint32_t>(m_grouplist.size()), m_entries, lodcount + 1};
		m_grouplist.push_back({lodfirst, lodcount, m_entries, 0});
		m_entries += group.entrycount;
		lookup = m_groupmap.emplace(Key, group).first;
	}

	place(Bucket);

	gl::instance_cull_job job{};
	job.origin = packed_vec3{Bucket.recordorigin - m_setup.camera};
	job.first = Bucket.gpufirst;
	job.distanceorigin = packed_vec3{Bucket.recordorigin - m_setup.distancecamera};
	job.count = static_cast<std::uint32_t>(Bucket.records.size());
	job.group = lookup->second.index;
	job.candidatestart = m_candidates;
	m_joblist.push_back(job);
	m_buckets.push_back(&Bucket);
	m_candidates += job.count;
}

void opengl46_instance_cull::place(scene::basic_cell::instance_bucket const &Bucket)
{
	auto const count{static_cast<std::uint32_t>(Bucket.records.size())};
	auto const current{Bucket.gpugeneration == m_recordgeneration};
	if (current && !Bucket.gpustale)
		return;
	if (current && count <= Bucket.gpucapacity)
	{
		m_records->update(Bucket.records.data(), static_cast<GLintptr>(Bucket.gpufirst) * RECORD_SIZE, static_cast<GLsizeiptr>(count) * RECORD_SIZE);
		Bucket.gpustale = false;
		return;
	}

	// a new place, with some room for instances the editor adds
	auto const allocate = [this](scene::basic_cell::instance_bucket const &Target) {
		auto const records{static_cast<std::uint32_t>(Target.records.size())};
		Target.gpufirst = m_recordfill;
		Target.gpucapacity = records + records / 4;
		Target.gpugeneration = m_recordgeneration;
		Target.gpustale = false;
		m_recordfill += Target.gpucapacity;
		m_records->update(Target.records.data(), static_cast<GLintptr>(Target.gpufirst) * RECORD_SIZE, static_cast<GLsizeiptr>(records) * RECORD_SIZE);
	};

	auto const capacity{static_cast<std::uint32_t>(m_records->size() / RECORD_SIZE)};
	if (m_recordfill + count + count / 4 > capacity)
	{
		// the buffer is started over, and only the buckets of this pass get places again. the places left behind by
		// buckets which were made anew or are out of sight are dropped this way
		GLsizeiptr live{count + count / 4};
		for (auto const *bucket : m_buckets)
			live += bucket->records.size() + bucket->records.size() / 4;
		if (2 * live * RECORD_SIZE > m_records->size())
			m_records->resize(std::max(2 * live * RECORD_SIZE, 2 * m_records->size()));
		++m_recordgeneration;
		m_recordfill = 0;
		for (std::size_t job = 0; job < m_buckets.size(); ++job)
		{
			allocate(*m_buckets[job]);
			m_joblist[job].first = m_buckets[job]->gpufirst;
		}
	}
	allocate(Bucket);
}

void opengl46_instance_cull::dispatch(glm::mat4 const &View)
{
	if (!m_inpass || m_joblist.empty())
		return;

	auto const countersize{static_cast<GLsizeiptr>((gl::INSTANCE_COUNTER_HEADER + 2 * m_entries) * sizeof(std::uint32_t))};
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
	std::uint32_t const header[gl::INSTANCE_COUNTER_HEADER]{m_setup.frame, 0, m_entries, 0};
	m_counters->update(header, 0, sizeof(header));

	m_records->bind_base();
	m_jobs->bind_base();
	m_groups->bind_base();
	m_lods->bind_base();
	m_counters->bind_base();
	m_output->bind_base();

	// planes moved to the camera of the pass, in double precision: the cpu tests world coordinates
	auto const &frustumplanes{m_setup.frustum->planes()};
	glm::vec4 planes[6];
	for (int plane = 0; plane < 6; ++plane)
	{
		auto const *source{frustumplanes[plane]};
		planes[plane] = glm::vec4{source[0], source[1], source[2],
		                          static_cast<float>(source[0] * m_setup.camera.x + source[1] * m_setup.camera.y + source[2] * m_setup.camera.z + source[3])};
	}
	glProgramUniformMatrix4fv(*m_cull_program, 0, 1, GL_FALSE, glm::value_ptr(View));
	glProgramUniform4fv(*m_cull_program, 1, 6, glm::value_ptr(planes[0]));
	glProgramUniform4f(*m_cull_program, 7, m_setup.distancescale, m_setup.distanceoffset, m_setup.distancerange, m_setup.drawrange);
	glProgramUniform4f(*m_cull_program, 8, m_setup.zoom, 0.f, 0.f, 0.f);
	glProgramUniform4ui(*m_offsets_program, 0, m_entries, 0, 0, 0);

	// one invocation per record; past the count limit of one dimension the groups go into rows
	auto const groups{(m_candidates + gl::INSTANCE_CULL_GROUP_SIZE - 1) / gl::INSTANCE_CULL_GROUP_SIZE};
	auto const groups_x{std::min<GLuint>(groups, 65535)};
	auto const groups_y{(groups + groups_x - 1) / groups_x};

	m_timer.begin();
	glProgramUniform4ui(*m_cull_program, 9, static_cast<GLuint>(m_joblist.size()), m_candidates, m_entries, 0);
	m_cull_program->dispatch(groups_x, groups_y);
	gl::memory_barrier(GL_SHADER_STORAGE_BARRIER_BIT);
	m_offsets_program->dispatch(1);
	gl::memory_barrier(GL_SHADER_STORAGE_BARRIER_BIT);
	glProgramUniform4ui(*m_cull_program, 9, static_cast<GLuint>(m_joblist.size()), m_candidates, m_entries, 1);
	m_cull_program->dispatch(groups_x, groups_y);
	// the output and the counts go to draws, indirect draws and the copy for the cpu
	gl::memory_barrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT | GL_COMMAND_BARRIER_BIT);
	m_timer.end();

	m_readback->push(*m_counters);
	m_cpucounts.assign(m_entries, 0);
	m_dispatched = true;
}

void opengl46_instance_cull::cpu_result(scene::basic_cell::instance_bucket_key const &Key, std::vector<std::uint32_t> const &Bandcounts)
{
	if (!m_dispatched)
		return;
	auto const lookup{m_groupmap.find(Key)};
	if (lookup == m_groupmap.end())
		return;
	auto const &group{lookup->second};
	for (std::size_t band = 0; band < std::min<std::size_t>(Bandcounts.size(), group.entrycount); ++band)
		m_cpucounts[group.entryfirst + band] += Bandcounts[band];
}

void opengl46_instance_cull::end_pass()
{
	if (m_dispatched)
	{
		std::uint64_t total{0};
		for (auto const count : m_cpucounts)
			total += count;
		m_cpupasses.push_back({m_setup.frame, m_cpucounts, total});
		// the gpu is never this far behind
		while (m_cpupasses.size() > 8)
			m_cpupasses.pop_front();
	}
	m_inpass = false;
	m_dispatched = false;
}

void opengl46_instance_cull::collect()
{
	if (!m_readback || !m_readback->pop(m_readbackdata.data()))
		return;

	auto const frame{m_readbackdata[0]};
	while (!m_cpupasses.empty() && m_cpupasses.front().frame != frame)
		m_cpupasses.pop_front();
	if (m_cpupasses.empty())
		return;

	auto const &cpu{m_cpupasses.front()};
	comparison result;
	result.frame = frame;
	result.gputotal = m_readbackdata[1];
	result.cputotal = cpu.total;
	result.entries = m_readbackdata[2];
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

std::string opengl46_instance_cull::info()
{
	if (!Global.gfx_gpucull)
		return {};
	if (m_comparedframes == 0)
		return "\ngpu cull: no result yet";
	return "\ngpu cull: " + std::to_string(m_last.gputotal) + " instances (cpu " + std::to_string(m_last.cputotal) + "), " + std::to_string(m_last.entries) + " bands, " +
	       std::to_string(m_last.mismatched) + " differ (by up to " + std::to_string(m_last.largestdifference) + "); frames differing " + std::to_string(m_mismatchedframes) + " of " +
	       std::to_string(m_comparedframes);
}

std::string opengl46_instance_cull::info_times()
{
	std::string text;
	if (!Global.gfx_gpucull)
		return text;
	if (auto const time = m_timer.result())
		text += " instance cull: " + to_string(static_cast<double>(*time) / 1000000.0, 3) + " ms\n";
	return text;
}
