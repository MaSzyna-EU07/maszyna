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

#include "rendering/particles.h"
#include "simulation/simulationenvironment.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/Timer.h"
#include "utilities/utilities.h"
#include "gl/vao.h"

namespace
{

// places of the smoke at most, all sources together (48 bytes each, as many again for the draw); the sources past it get none
std::uint32_t constexpr MAX_SMOKE_PLACES{1u << 19};
// the origin of the smoke moves by whole steps when the camera gets this far from it
double constexpr ORIGIN_STEP{256.0};
double constexpr ORIGIN_REACH{1024.0};
// how much the smoke scatters the light onwards (Henyey-Greenstein)
float constexpr SMOKE_FORWARD{0.3f};

// the boxes of the rain and snow around the camera: their sizes (metres) and the shares of the particles in each
std::array<float, 3> constexpr BOX_SIZE{10.f, 24.f, 60.f};
std::array<float, 3> constexpr BOX_SHARE{0.2f, 0.35f, 0.45f};

// what the rain and the snow are like: the speed of the fall (m/s), how long a streak lasts (s, the eye's or the camera's
// exposure), opacity, how much a drop or a flake scatters the light onwards and how bright it is against a white surface
struct precipitation_kind
{
	float fall;
	float exposure;
	float opacity;
	float forward;
	float brightness;
};
precipitation_kind constexpr RAIN{7.f, 0.025f, 0.5f, 0.6f, 0.6f};
precipitation_kind constexpr SNOW{1.1f, 0.01f, 0.85f, 0.25f, 0.9f};
// the share of the wind of the environment the drops and flakes drift with (as the old cylinder leant with it)
double constexpr WIND_SHARE{0.5};
// the camera faster than this from a frame to the next was moved, not driven or flown
double constexpr CAMERA_JUMP{120.0};

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

bool opengl46_gpu_particles::init()
{
	if (m_state != 0)
		return m_state > 0;
	m_state = -1;
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
	m_state = 1;
	return true;
}

bool opengl46_gpu_particles::smoke_available()
{
	return init();
}

bool opengl46_gpu_particles::precipitation_available()
{
	return init();
}

void opengl46_gpu_particles::clear()
{
	m_pool.clear();
	m_requested.clear();
	m_poolfull = false;
	m_sourcerecords.clear();
	m_places = 0;
	m_particles.reset();
	m_sources.reset();
	m_owners.reset();
	m_smokeoutput.reset();
	m_precipitationoutput.reset();
	m_precipitationcapacity = 0;
	m_originset = false;
	m_lastupdate = 0;
}

void opengl46_gpu_particles::layout(particle_manager &Particles)
{
	auto const &sources{Particles.sequence()};
	// the sources already laid out which asked for the same places since
	std::size_t kept{0};
	while ((kept < m_requested.size()) && (kept < sources.size()) && (m_requested[kept] == sources[kept].max_particles()))
		++kept;
	if ((kept == m_requested.size()) && (kept == sources.size()) && m_particles)
		return;
	if (kept < m_requested.size())
	{
		// a source went away or changed: all anew
		kept = 0;
		m_pool.clear();
		m_requested.clear();
		m_places = 0;
		m_poolfull = false;
	}
	auto const oldplaces{m_places};

	auto first{m_places};
	m_pool.resize(sources.size());
	m_requested.resize(sources.size());
	for (std::size_t i = kept; i < sources.size(); ++i)
	{
		m_requested[i] = sources[i].max_particles();
		auto const size{static_cast<std::uint32_t>(std::min<std::size_t>(m_requested[i], MAX_SMOKE_PLACES))};
		m_pool[i] = pool_source{};
		if (first + size > MAX_SMOKE_PLACES)
		{
			if (false == m_poolfull)
				WriteLog("particles: the smoke of some sources left out, over " + std::to_string(MAX_SMOKE_PLACES) + " places of particles");
			m_poolfull = true;
			continue;
		}
		m_pool[i].first = first;
		m_pool[i].size = size;
		first += size;
	}
	m_places = first;

	std::vector<std::uint32_t> owners(std::max<std::uint32_t>(m_places, 1), 0);
	for (std::size_t i = 0; i < m_pool.size(); ++i)
		std::fill_n(owners.begin() + m_pool[i].first, m_pool[i].size, static_cast<std::uint32_t>(i));

	auto const places{static_cast<GLsizeiptr>(std::max<std::uint32_t>(m_places, 1))};
	// the particles of the places kept go over to the new storage, the new places are dead
	auto particles{std::make_unique<gl::ssbo>(places * static_cast<GLsizeiptr>(sizeof(gl::particle_smoke)), gl::PARTICLE_SSBO_BINDING)};
	::glClearNamedBufferData(*particles, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, nullptr);
	if (m_particles && (oldplaces > 0) && (kept > 0))
		::glCopyNamedBufferSubData(*m_particles, *particles, 0, 0, static_cast<GLsizeiptr>(oldplaces) * static_cast<GLsizeiptr>(sizeof(gl::particle_smoke)));
	m_particles = std::move(particles);

	auto const ensure = [](std::unique_ptr<gl::ssbo> &Buffer, GLsizeiptr const Size, GLuint const Binding) {
		if (!Buffer)
			Buffer = std::make_unique<gl::ssbo>(Size, Binding);
		else
			Buffer->resize(Size);
	};
	ensure(m_owners, places * static_cast<GLsizeiptr>(sizeof(std::uint32_t)), gl::PARTICLE_OWNER_SSBO_BINDING);
	ensure(m_smokeoutput, places * static_cast<GLsizeiptr>(sizeof(gl::particle_draw)), gl::PARTICLE_OUTPUT_SSBO_BINDING);
	ensure(m_sources, static_cast<GLsizeiptr>(std::max<std::size_t>(sources.size(), 1) * sizeof(gl::particle_smoke_source)), gl::PARTICLE_SOURCE_SSBO_BINDING);
	m_owners->update(owners.data(), 0, static_cast<GLsizeiptr>(owners.size() * sizeof(std::uint32_t)));
	m_sourcerecords.resize(sources.size(), gl::particle_smoke_source{});
}

void opengl46_gpu_particles::simulate_smoke(particle_manager &Particles, glm::dvec3 const &Camera)
{
	if (false == init())
		return;
	layout(Particles);
	if (Particles.updates() == m_lastupdate)
		return;
	m_lastupdate = Particles.updates();
	if (m_places == 0)
		return;

	// the origin of the positions near the camera; when it moves, the particles move by as much the other way
	glm::vec3 shift{0.f};
	auto const snapped{glm::floor(Camera / ORIGIN_STEP) * ORIGIN_STEP};
	if (false == m_originset)
	{
		m_origin = snapped;
		m_originset = true;
	}
	else if (glm::length(Camera - m_origin) > ORIGIN_REACH)
	{
		shift = glm::vec3{m_origin - snapped};
		m_origin = snapped;
	}

	// the light of the smoke as the cpu works it out, for gfx.smoke.lighting off
	auto const oldlight{(glm::vec3{Global.DayLight.ambient} + 0.35f * glm::vec3{Global.DayLight.diffuse}) * simulation::Environment.light_intensity()};
	auto const &sources{Particles.sequence()};
	m_smokealive = 0;
	for (std::size_t i = 0; i < sources.size(); ++i)
	{
		auto const &state{sources[i].gpu()};
		auto &pool{m_pool[i]};
		auto &record{m_sourcerecords[i]};
		auto const spawns{(pool.size > 0) ? std::min(state.spawns, pool.size) : 0u};
		record.pool = glm::uvec4{pool.first, pool.size, pool.head, spawns};
		record.location = glm::vec4{glm::vec3{state.location - m_origin}, 0.f};
		record.previous = glm::vec4{glm::vec3{state.previous - m_origin}, 0.f};
		record.launch = glm::vec4{state.inclination[0], state.inclination[1], state.velocity[0], state.velocity[1]};
		record.initial = glm::vec4{state.size[0], state.size[1], state.opacity[0], state.opacity[1]};
		record.sizechange = glm::vec4{state.sizechange[0], state.sizechange[1], state.sizechange[2], 0.f};
		record.opacitychange = glm::vec4{state.opacitychange[0], state.opacitychange[1], state.opacitychange[2], 0.f};
		record.motion = glm::vec4{state.rise, state.damping, state.risemin, 0.f};
		record.color = glm::vec4{glm::clamp(state.color, glm::vec3{0.f}, glm::vec3{1.f}), 0.f};
		record.oldcolor = glm::vec4{glm::clamp(state.color * oldlight, glm::vec3{0.f}, glm::vec3{1.f}), 0.f};
		if (pool.size > 0)
		{
			pool.head = (pool.head + spawns) % pool.size;
			if (spawns > 0)
				m_smokealive += pool.size;
		}
	}
	if (false == m_sourcerecords.empty())
		m_sources->update(m_sourcerecords.data(), 0, static_cast<GLsizeiptr>(m_sourcerecords.size() * sizeof(gl::particle_smoke_source)));

	m_simulatetimer.begin();
	m_particles->bind_base();
	m_sources->bind_base();
	m_owners->bind_base();
	auto const wind{glm::vec3{simulation::Environment.wind()}};
	auto &program{*m_smokesimulate};
	::glProgramUniform1ui(program, 0, m_places);
	::glProgramUniform4f(program, 1, static_cast<float>(Particles.timedelta()), shift.x, shift.y, shift.z);
	::glProgramUniform4f(program, 2, wind.x, wind.y, wind.z, 0.f);
	::glProgramUniform1ui(program, 3, static_cast<GLuint>(m_lastupdate));
	program.dispatch(groups(m_places));
	::glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	m_simulatetimer.end();
}

std::array<glm::vec4, 5> opengl46_gpu_particles::planes(view const &View)
{
	// the sides and the near side of the clip space, w >= 0 (the far plane is at infinity)
	auto const matrix{glm::transpose(View.projection * View.modelview)};
	std::array<glm::vec4, 5> result{matrix[3] + matrix[0], matrix[3] - matrix[0], matrix[3] + matrix[1], matrix[3] - matrix[1], matrix[3]};
	for (auto &plane : result)
	{
		auto const length{glm::length(glm::vec3{plane})};
		if (length > 0.f)
			plane /= length;
	}
	return result;
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

std::size_t opengl46_gpu_particles::render_smoke(view const &View)
{
	if ((false == init()) || (m_places == 0))
		return 0;
	collect(*m_smokereadback, m_smokedrawn);

	m_smoketimer.begin();
	GLuint const command[4]{4, 0, 0, 0};
	m_smokecommand->update(command, 0, sizeof(command));
	m_particles->bind_base();
	m_sources->bind_base();
	m_owners->bind_base();
	m_smokeoutput->bind_base();
	m_smokecommand->bind_base();
	auto &program{*m_smokeprepare};
	auto const origin{glm::vec3{m_origin - View.camera}};
	auto const viewplanes{planes(View)};
	::glProgramUniform1ui(program, 0, m_places);
	::glProgramUniform3f(program, 1, origin.x, origin.y, origin.z);
	::glProgramUniformMatrix4fv(program, 2, 1, GL_FALSE, glm::value_ptr(View.modelview));
	::glProgramUniform4f(program, 3, Global.gfx_smoke_lighting ? 1.f : 0.f, SMOKE_FORWARD, static_cast<float>(m_places), 0.f);
	::glProgramUniform4fv(program, 4, 5, glm::value_ptr(viewplanes[0]));
	program.dispatch(groups(m_places));
	::glMemoryBarrier(GL_COMMAND_BARRIER_BIT | GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT);

	::glProgramUniform4f(*m_smokedraw, 0, View.soft ? Global.gfx_particles_soft : 0.f, 0.f, 0.f, 0.f);
	draw(*m_smokeoutput, *m_smokecommand, *m_smokedraw);
	m_smoketimer.end();

	m_smokereadback->push(*m_smokecommand);
	return m_smokedrawn;
}

std::size_t opengl46_gpu_particles::render_precipitation(view const &View, bool const Snow, float const Intensity, unsigned int const Frame)
{
	if (false == init())
		return 0;
	collect(*m_precipitationreadback, m_precipitationdrawn);

	auto const &kind{Snow ? SNOW : RAIN};
	// once per frame: the time moves on, the particles drift with the wind and fall, the camera has its velocity
	if ((false == m_framestarted) || (Frame != m_frame))
	{
		m_framestarted = true;
		m_frame = Frame;
		auto const seconds{Timer::GetDeltaTime()};
		if (seconds > 0.0)
		{
			auto const wind{glm::dvec3{simulation::Environment.wind()} * WIND_SHARE};
			for (std::size_t box = 0; box < BOX_SIZE.size(); ++box)
			{
				auto const size{static_cast<double>(BOX_SIZE[box])};
				m_drift[box] = glm::mod(m_drift[box] + wind * seconds, size);
				m_fall[box] = std::fmod(m_fall[box] + kind.fall * seconds, 16.0 * size);
			}
			m_time = std::fmod(m_time + seconds, 3600.0);
			if (m_camerastarted)
			{
				auto const velocity{(View.camera - m_lastcamera) / seconds};
				if (glm::length(velocity) < CAMERA_JUMP)
					m_cameravelocity = glm::mix(m_cameravelocity, velocity, 1.0 - std::exp(-10.0 * seconds));
			}
		}
		m_lastcamera = View.camera;
		m_camerastarted = true;
	}

	// the particles, more of them in heavier rain or snow
	auto const most{static_cast<std::uint32_t>(std::max(Global.gfx_precipitation_particles, 0))};
	auto const count{static_cast<std::uint32_t>(most * std::clamp(0.15f + 0.85f * Intensity, 0.f, 1.f))};
	m_precipitationcount = count;
	m_precipitationsnow = Snow;
	if (count == 0)
		return 0;
	if (m_precipitationcapacity < count)
	{
		m_precipitationcapacity = std::max(count, most);
		auto const size{static_cast<GLsizeiptr>(m_precipitationcapacity) * static_cast<GLsizeiptr>(sizeof(gl::particle_draw))};
		if (!m_precipitationoutput)
			m_precipitationoutput = std::make_unique<gl::ssbo>(size, gl::PARTICLE_OUTPUT_SSBO_BINDING);
		else
			m_precipitationoutput->resize(size);
	}

	m_precipitationtimer.begin();
	GLuint const command[4]{4, 0, 0, 0};
	m_precipitationcommand->update(command, 0, sizeof(command));
	m_precipitationoutput->bind_base();
	m_precipitationcommand->bind_base();
	auto &program{*m_precipitationprepare};
	auto const second{static_cast<std::uint32_t>(count * BOX_SHARE[0])};
	auto const third{static_cast<std::uint32_t>(count * (BOX_SHARE[0] + BOX_SHARE[1]))};
	::glProgramUniform4ui(program, 0, second, third, count, Snow ? 1u : 0u);
	std::array<glm::vec4, 3> offsets;
	for (std::size_t box = 0; box < BOX_SIZE.size(); ++box)
	{
		auto const size{static_cast<double>(BOX_SIZE[box])};
		offsets[box] = glm::vec4{glm::vec3{glm::mod(m_drift[box] - glm::mod(View.camera, size), size)}, static_cast<float>(m_fall[box])};
	}
	::glProgramUniform4fv(program, 1, 3, glm::value_ptr(offsets[0]));
	::glProgramUniform4f(program, 4, BOX_SIZE[0], BOX_SIZE[1], BOX_SIZE[2], 0.f);
	auto const velocity{glm::vec3{glm::dvec3{simulation::Environment.wind()} * WIND_SHARE - m_cameravelocity}};
	::glProgramUniform4f(program, 5, velocity.x, velocity.y, velocity.z, kind.fall);
	::glProgramUniform4f(program, 6, static_cast<float>(m_time), kind.exposure, kind.opacity, static_cast<float>(m_precipitationcapacity));
	::glProgramUniformMatrix4fv(program, 7, 1, GL_FALSE, glm::value_ptr(View.modelview));
	::glProgramUniform4f(program, 11, 1.f, kind.forward, kind.brightness, 0.f);
	auto const viewplanes{planes(View)};
	::glProgramUniform4fv(program, 12, 5, glm::value_ptr(viewplanes[0]));
	program.dispatch(groups(count));
	::glMemoryBarrier(GL_COMMAND_BARRIER_BIT | GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT);

	// the width of a pixel a metre in front of the camera, for the least width of a streak
	auto const pixel{(View.height > 0) && (View.projection[1][1] != 0.f) ? 2.f / (std::abs(View.projection[1][1]) * static_cast<float>(View.height)) : 0.f};
	::glProgramUniform4f(*m_precipitationdraw, 0, View.soft ? 0.25f * Global.gfx_particles_soft : 0.f, 0.f, 0.f, 0.f);
	::glProgramUniform4f(*m_precipitationdraw, 1, pixel, 0.f, 0.f, 0.f);
	draw(*m_precipitationoutput, *m_precipitationcommand, *m_precipitationdraw);
	m_precipitationtimer.end();

	m_precipitationreadback->push(*m_precipitationcommand);
	return m_precipitationdrawn;
}

std::string opengl46_gpu_particles::info() const
{
	if (m_state <= 0)
		return {};
	std::string text;
	if (m_places > 0)
		text += " smoke of the gpu: " + std::to_string(m_smokedrawn) + " drawn, " + std::to_string(m_places) + " places of " + std::to_string(m_pool.size()) + " sources ("
		        + std::to_string(m_smokealive) + " of the ones which bore lately)" + (m_poolfull ? ", some sources left out" : "") + "\n";
	if (m_precipitationcount > 0)
		text += std::string(" ") + (m_precipitationsnow ? "snow" : "rain") + " of the gpu: " + std::to_string(m_precipitationdrawn) + " of " + std::to_string(m_precipitationcount)
		        + " drawn\n";
	return text;
}

std::string opengl46_gpu_particles::info_times()
{
	std::string text;
	auto const add = [&text](gl::gpu_timer &Timer, char const *Name) {
		if (auto const time = Timer.result())
			text += std::string(text.empty() ? " particles:" : ",") + " " + Name + " " + to_string(static_cast<double>(*time) / 1000000.0, 3) + " ms";
	};
	if (m_places > 0)
	{
		add(m_simulatetimer, "smoke simulation");
		add(m_smoketimer, "smoke");
	}
	if (m_precipitationcount > 0)
		add(m_precipitationtimer, m_precipitationsnow ? "snow" : "rain");
	if (false == text.empty())
		text += "\n";
	return text;
}
