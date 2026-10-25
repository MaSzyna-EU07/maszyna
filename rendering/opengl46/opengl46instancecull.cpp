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

#include "model/Texture.h"
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
		share_records(*Shared);
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
}

void opengl46_instance_cull::dispatch(glm::mat4 const &View, occlusion_setup const *Occlusion, bool const Twophase)
{
	if (!dispatch_begin())
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
	glm::vec4 planes[6];
	frustum_planes(planes);
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

	dispatch_end();
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

std::int64_t opengl46_instance_cull::record_buffer_size() const
{
	return m_shared->records->size();
}

void opengl46_instance_cull::resize_record_buffer(std::int64_t const Size)
{
	m_shared->records->resize(static_cast<GLsizeiptr>(Size));
}

void opengl46_instance_cull::send_records(std::uint32_t const First, scene::basic_cell::instance_record const *Records, std::uint32_t const Count)
{
	m_shared->records->update(Records, static_cast<GLintptr>(First) * RECORD_SIZE, static_cast<GLsizeiptr>(Count) * RECORD_SIZE);
}

void opengl46_instance_cull::push_counts()
{
	m_readback->push(*m_counters);
}

bool opengl46_instance_cull::pop_counts(std::uint32_t *Counts)
{
	return (m_readback && m_readback->pop(Counts));
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
