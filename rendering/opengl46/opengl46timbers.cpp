/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46timbers.h"

#include "utilities/Globals.h"
#include "world/Track.h"

#include <cstring>

namespace {

// the buffer to start with, and how soon after being started over it's too small for the tracks in sight
std::uint32_t constexpr initial_capacity{65536};
unsigned int constexpr restart_frames{600};

// the instances of a track, in the order they lie in the buffer
std::uint32_t count_of(track_sleepers::laid const &Laid)
{
	std::size_t count{Laid.whole.size() + Laid.plain.size()};
	for (auto const &instances : Laid.instances)
		count += instances.size();
	return static_cast<std::uint32_t>(count);
}

} // namespace

opengl46_timbers::~opengl46_timbers()
{
	if (m_buffer != 0)
		glDeleteBuffers(1, &m_buffer);
}

void opengl46_timbers::init()
{
	gl::shader shader("timber_instances.comp");
	m_program = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{shader});
	// the modelviews of the origins and the runs, both placed at whole matrices
	m_ranges = std::make_unique<gl::frame_stream>(
		static_cast<GLsizeiptr>(4096 * sizeof(gl::timber_range)), static_cast<GLsizeiptr>(sizeof(glm::mat4)), Global.gfx_bufferstreaming);
	restart(0);
}

void opengl46_timbers::restart(std::uint32_t const Needed)
{
	auto capacity{std::max(m_capacity, initial_capacity)};
	if ((m_capacity > 0) && (m_frame - m_restartframe < restart_frames))
		capacity *= 2;
	while (capacity < 2 * Needed)
		capacity *= 2;
	// the draws and passes issued with the old buffer keep it until the gpu is done with them
	if (m_buffer != 0)
		glDeleteBuffers(1, &m_buffer);
	glCreateBuffers(1, &m_buffer);
	glNamedBufferStorage(m_buffer, static_cast<GLsizeiptr>(capacity) * static_cast<GLsizeiptr>(sizeof(glm::mat4)), nullptr, GL_DYNAMIC_STORAGE_BIT);
	m_capacity = capacity;
	m_fill = 0;
	m_restartframe = m_frame;
	m_tracks.clear();
}

bool opengl46_timbers::place(std::vector<TTrack *> const &Tracks, bool const Restart)
{
	auto const placed_now = [&](TTrack const &Track) {
		auto const found{m_tracks.find(&Track)};
		return (found != m_tracks.end()) && (found->second.made == Track.m_sleepers3d.made);
	};
	std::uint32_t needed{0};
	std::uint32_t all{0};
	for (auto const *track : Tracks)
	{
		auto const count{count_of(track->m_sleepers3d)};
		all += count;
		if (false == placed_now(*track))
			needed += count;
	}
	if (needed == 0)
		return true;
	if (m_fill + needed > m_capacity)
	{
		if (false == Restart)
			return false;
		restart(all);
	}
	for (auto const *track : Tracks)
	{
		if (placed_now(*track))
			continue;
		auto const &laid{track->m_sleepers3d};
		auto const first{m_fill};
		auto const send = [&](std::vector<glm::mat4> const &Instances) {
			if (false == Instances.empty())
				glNamedBufferSubData(m_buffer, static_cast<GLintptr>(m_fill) * static_cast<GLintptr>(sizeof(glm::mat4)),
				                     static_cast<GLsizeiptr>(Instances.size() * sizeof(glm::mat4)), Instances.data());
			m_fill += static_cast<std::uint32_t>(Instances.size());
		};
		for (auto const &instances : laid.instances)
			send(instances);
		send(laid.whole);
		send(laid.plain);
		m_tracks[track] = {laid.made, first};
	}
	return true;
}

std::uint32_t opengl46_timbers::first(TTrack const &Track) const
{
	auto const found{m_tracks.find(&Track)};
	return (found != m_tracks.end() ? found->second.first : 0);
}

void opengl46_timbers::expand(std::vector<glm::mat4> const &Origins, std::vector<gl::timber_range> const &Ranges, GLuint const Output)
{
	if (Ranges.empty())
		return;
	// sent together, in one buffer of the stream
	auto const originbytes{Origins.size() * sizeof(glm::mat4)};
	m_upload.resize(originbytes + Ranges.size() * sizeof(gl::timber_range));
	std::memcpy(m_upload.data(), Origins.data(), originbytes);
	std::memcpy(m_upload.data() + originbytes, Ranges.data(), Ranges.size() * sizeof(gl::timber_range));
	auto const origins{m_ranges->push(m_upload.data(), static_cast<GLsizeiptr>(m_upload.size()))};
	auto const ranges{origins + static_cast<GLintptr>(originbytes)};
	GLuint const buffers[]{m_buffer, m_ranges->buffer()};
	::glBindBuffersBase(GL_SHADER_STORAGE_BUFFER, gl::TIMBER_LOCAL_SSBO_BINDING, 1, &buffers[0]);
	::glBindBuffersBase(GL_SHADER_STORAGE_BUFFER, gl::TIMBER_RANGE_SSBO_BINDING, 1, &buffers[1]);
	::glBindBuffersBase(GL_SHADER_STORAGE_BUFFER, gl::TIMBER_OUTPUT_SSBO_BINDING, 1, &Output);
	::glBindBuffersBase(GL_SHADER_STORAGE_BUFFER, gl::TIMBER_ORIGIN_SSBO_BINDING, 1, &buffers[1]);
	// a dispatch takes at most 65535 work groups
	auto const first{static_cast<std::uint32_t>(ranges / static_cast<GLintptr>(sizeof(gl::timber_range)))};
	auto const origin{static_cast<std::uint32_t>(origins / static_cast<GLintptr>(sizeof(glm::mat4)))};
	for (std::size_t done = 0; done < Ranges.size();)
	{
		auto const count{std::min<std::size_t>(Ranges.size() - done, 65535)};
		::glProgramUniform4ui(*m_program, 0, first + static_cast<std::uint32_t>(done), origin, 0, 0);
		m_program->dispatch(static_cast<GLuint>(count));
		done += count;
	}
	::glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

void opengl46_timbers::next_frame()
{
	++m_frame;
	m_ranges->next_frame();
}
