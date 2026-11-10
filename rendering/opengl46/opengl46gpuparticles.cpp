/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46gpuparticles.h"

#include "utilities/Logs.h"
#include "utilities/utilities.h"
#include "gl/vao.h"

namespace
{

std::unique_ptr<gl::program> make_program(std::vector<std::string> const &Names)
{
	std::vector<std::unique_ptr<gl::shader>> shaders;
	std::vector<std::reference_wrapper<const gl::shader>> references;
	for (auto const &name : Names)
	{
		shaders.emplace_back(std::make_unique<gl::shader>(name));
		references.emplace_back(*shaders.back());
	}
	return std::make_unique<gl::program>(references);
}

std::uint32_t groups(std::uint32_t const Count)
{
	return (Count + gl::PARTICLE_GROUP_SIZE - 1) / gl::PARTICLE_GROUP_SIZE;
}

} // namespace

opengl46_gpu_particles::~opengl46_gpu_particles()
{
	// the vertex array goes with the context; deleted here only if it's still there
	if ((m_vao != 0) && (::glfwGetCurrentContext() != nullptr))
		::glDeleteVertexArrays(1, &m_vao);
}

bool opengl46_gpu_particles::gpu_init()
{
	try
	{
		m_smokesimulate = make_program({"particles_smoke_simulate.comp"});
		m_smokeprepare = make_program({"particles_smoke_prepare.comp"});
		m_smokedraw = make_program({"particles_smoke.vert", "particles_smoke.frag"});
		m_precipitationprepare = make_program({"particles_precipitation.comp"});
		m_precipitationdraw = make_program({"particles_precipitation.vert", "particles_precipitation.frag"});
	}
	catch (gl::shader_exception const &e)
	{
		ErrorLog("particles: can't use the programs of the particles of the gpu, the smoke stays on the cpu: " + std::string(e.what()));
		return false;
	}

	// the attributes of the instances: three vec4 of gl::particle_draw
	::glCreateVertexArrays(1, &m_vao);
	for (GLuint attribute = 0; attribute < 3; ++attribute)
	{
		::glEnableVertexArrayAttrib(m_vao, attribute);
		::glVertexArrayAttribFormat(m_vao, attribute, 4, GL_FLOAT, GL_FALSE, attribute * sizeof(glm::vec4));
		::glVertexArrayAttribBinding(m_vao, attribute, 0);
	}
	::glVertexArrayBindingDivisor(m_vao, 0, 1);

	m_smokecommand = std::make_unique<gl::ssbo>(4 * sizeof(GLuint), gl::PARTICLE_COMMAND_SSBO_BINDING);
	m_precipitationcommand = std::make_unique<gl::ssbo>(4 * sizeof(GLuint), gl::PARTICLE_COMMAND_SSBO_BINDING);
	m_smokereadback = std::make_unique<gl::readback>(4 * sizeof(GLuint));
	m_precipitationreadback = std::make_unique<gl::readback>(4 * sizeof(GLuint));
	return true;
}

void opengl46_gpu_particles::gpu_clear()
{
	m_particles.reset();
	m_sources.reset();
	m_owners.reset();
	m_smokeoutput.reset();
	m_precipitationoutput.reset();
}

void opengl46_gpu_particles::gpu_layout(std::uint32_t const Places, std::uint32_t const Kept, std::vector<std::uint32_t> const &Owners, std::size_t const Sources)
{
	auto const places{static_cast<GLsizeiptr>(Places)};
	// the particles of the places kept go over to the new storage, the new places are dead
	auto particles{std::make_unique<gl::ssbo>(places * static_cast<GLsizeiptr>(sizeof(gl::particle_smoke)), gl::PARTICLE_SSBO_BINDING)};
	::glClearNamedBufferData(*particles, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, nullptr);
	if (m_particles && (Kept > 0))
		::glCopyNamedBufferSubData(*m_particles, *particles, 0, 0, static_cast<GLsizeiptr>(Kept) * static_cast<GLsizeiptr>(sizeof(gl::particle_smoke)));
	m_particles = std::move(particles);

	auto const ensure = [](std::unique_ptr<gl::ssbo> &Buffer, GLsizeiptr const Size, GLuint const Binding) {
		if (!Buffer)
			Buffer = std::make_unique<gl::ssbo>(Size, Binding);
		else
			Buffer->resize(Size);
	};
	ensure(m_owners, places * static_cast<GLsizeiptr>(sizeof(std::uint32_t)), gl::PARTICLE_OWNER_SSBO_BINDING);
	ensure(m_smokeoutput, places * static_cast<GLsizeiptr>(sizeof(gl::particle_draw)), gl::PARTICLE_OUTPUT_SSBO_BINDING);
	ensure(m_sources, static_cast<GLsizeiptr>(std::max<std::size_t>(Sources, 1) * sizeof(gl::particle_smoke_source)), gl::PARTICLE_SOURCE_SSBO_BINDING);
	m_owners->update(Owners.data(), 0, static_cast<GLsizeiptr>(Owners.size() * sizeof(std::uint32_t)));
}

void opengl46_gpu_particles::gpu_simulate(smoke_simulation const &Simulation, std::vector<gl::particle_smoke_source> const &Records)
{
	if (false == Records.empty())
		m_sources->update(Records.data(), 0, static_cast<GLsizeiptr>(Records.size() * sizeof(gl::particle_smoke_source)));

	m_simulatetimer.begin();
	m_particles->bind_base();
	m_sources->bind_base();
	m_owners->bind_base();
	auto &program{*m_smokesimulate};
	::glProgramUniform1ui(program, 0, Simulation.places);
	::glProgramUniform4fv(program, 1, 1, glm::value_ptr(Simulation.step));
	::glProgramUniform4fv(program, 2, 1, glm::value_ptr(Simulation.wind));
	::glProgramUniform1ui(program, 3, static_cast<GLuint>(Simulation.seed));
	program.dispatch(groups(Simulation.places));
	::glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	m_simulatetimer.end();
}

void opengl46_gpu_particles::collect(gl::readback &Readback, std::size_t &Drawn)
{
	GLuint command[4];
	if (Readback.pop(command))
		Drawn = command[1];
}

void opengl46_gpu_particles::draw(GLuint const Output, GLuint const Command, gl::program &Program)
{
	auto const culling{::glIsEnabled(GL_CULL_FACE) == GL_TRUE};
	if (culling)
		::glDisable(GL_CULL_FACE);
	Program.bind();
	// the vertex array is bound behind the back of gl::vao, which is told nothing is bound afterwards
	gl::vao::unbind();
	::glVertexArrayVertexBuffer(m_vao, 0, Output, 0, sizeof(gl::particle_draw));
	::glBindVertexArray(m_vao);
	::glBindBuffer(GL_DRAW_INDIRECT_BUFFER, Command);
	::glDrawArraysIndirect(GL_TRIANGLE_STRIP, nullptr);
	::glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);
	gl::vao::unbind();
	gl::program::unbind();
	if (culling)
		::glEnable(GL_CULL_FACE);
}

void opengl46_gpu_particles::gpu_render_smoke(smoke_draw const &Draw, std::size_t &Drawn)
{
	collect(*m_smokereadback, Drawn);

	m_smoketimer.begin();
	GLuint const command[4]{4, 0, 0, 0};
	m_smokecommand->update(command, 0, sizeof(command));
	m_particles->bind_base();
	m_sources->bind_base();
	m_owners->bind_base();
	m_smokeoutput->bind_base();
	m_smokecommand->bind_base();
	auto &program{*m_smokeprepare};
	::glProgramUniform1ui(program, 0, Draw.places);
	::glProgramUniform3fv(program, 1, 1, glm::value_ptr(Draw.origin));
	::glProgramUniformMatrix4fv(program, 2, 1, GL_FALSE, glm::value_ptr(Draw.view));
	::glProgramUniform4fv(program, 3, 1, glm::value_ptr(Draw.options));
	::glProgramUniform4fv(program, 4, 5, glm::value_ptr(Draw.planes[0]));
	program.dispatch(groups(Draw.places));
	::glMemoryBarrier(GL_COMMAND_BARRIER_BIT | GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT);

	::glProgramUniform4fv(*m_smokedraw, 0, 1, glm::value_ptr(Draw.soft));
	draw(*m_smokeoutput, *m_smokecommand, *m_smokedraw);
	m_smoketimer.end();

	m_smokereadback->push(*m_smokecommand);
}

void opengl46_gpu_particles::gpu_collect_precipitation(std::size_t &Drawn)
{
	collect(*m_precipitationreadback, Drawn);
}

void opengl46_gpu_particles::gpu_precipitation_storage(std::uint32_t const Capacity)
{
	auto const size{static_cast<GLsizeiptr>(Capacity) * static_cast<GLsizeiptr>(sizeof(gl::particle_draw))};
	if (!m_precipitationoutput)
		m_precipitationoutput = std::make_unique<gl::ssbo>(size, gl::PARTICLE_OUTPUT_SSBO_BINDING);
	else
		m_precipitationoutput->resize(size);
}

void opengl46_gpu_particles::gpu_render_precipitation(precipitation_draw const &Draw)
{
	m_precipitationtimer.begin();
	GLuint const command[4]{4, 0, 0, 0};
	m_precipitationcommand->update(command, 0, sizeof(command));
	m_precipitationoutput->bind_base();
	m_precipitationcommand->bind_base();
	auto &program{*m_precipitationprepare};
	::glProgramUniform4uiv(program, 0, 1, glm::value_ptr(Draw.counts));
	::glProgramUniform4fv(program, 1, 3, glm::value_ptr(Draw.offsets[0]));
	::glProgramUniform4fv(program, 4, 1, glm::value_ptr(Draw.sizes));
	::glProgramUniform4fv(program, 5, 1, glm::value_ptr(Draw.velocity));
	::glProgramUniform4fv(program, 6, 1, glm::value_ptr(Draw.timing));
	::glProgramUniformMatrix4fv(program, 7, 1, GL_FALSE, glm::value_ptr(Draw.view));
	::glProgramUniform4fv(program, 11, 1, glm::value_ptr(Draw.options));
	::glProgramUniform4fv(program, 12, 5, glm::value_ptr(Draw.planes[0]));
	program.dispatch(groups(Draw.count));
	::glMemoryBarrier(GL_COMMAND_BARRIER_BIT | GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT);

	::glProgramUniform4fv(*m_precipitationdraw, 0, 1, glm::value_ptr(Draw.soft));
	::glProgramUniform4fv(*m_precipitationdraw, 1, 1, glm::value_ptr(Draw.pixel));
	draw(*m_precipitationoutput, *m_precipitationcommand, *m_precipitationdraw);
	m_precipitationtimer.end();

	m_precipitationreadback->push(*m_precipitationcommand);
}

std::string opengl46_gpu_particles::info_times()
{
	std::string text;
	auto const add = [&text](gl::gpu_timer &Timer, char const *Name) {
		if (auto const time = Timer.result())
			text += std::string(text.empty() ? " particles:" : ",") + " " + Name + " " + to_string(static_cast<double>(*time) / 1000000.0, 3) + " ms";
	};
	if (places() > 0)
	{
		add(m_simulatetimer, "smoke simulation");
		add(m_smoketimer, "smoke");
	}
	if (precipitation_count() > 0)
		add(m_precipitationtimer, precipitation_snow() ? "snow" : "rain");
	if (false == text.empty())
		text += "\n";
	return text;
}
