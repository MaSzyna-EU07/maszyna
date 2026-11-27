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
#include "utilities/Logs.h"
#include "world/Track.h"

#include <algorithm>
#include <cstring>
#include <format>

namespace {

// the buffer to start with, and how soon after being started over it's too small for the tracks in sight
std::uint32_t constexpr initial_capacity{65536};
unsigned int constexpr restart_frames{600};
// how much bigger than the room for the instances to place a buffer grows by being started over soon
std::uint64_t constexpr restart_growth{8};

// the instances of a track, in the order they lie in the buffer
std::uint32_t count_of(track_sleepers::laid const &Laid)
{
	std::size_t count{Laid.whole.size() + Laid.plain.size()};
	for (auto const &instances : Laid.instances)
		count += instances.size();
	return static_cast<std::uint32_t>(count);
}

// a buffer for Capacity instances, 0 when there's no memory for it
GLuint make_buffer(std::uint64_t const Capacity)
{
	GLuint buffer{0};
	glCreateBuffers(1, &buffer);
	glNamedBufferStorage(buffer, static_cast<GLsizeiptr>(Capacity * sizeof(glm::mat4)), nullptr, GL_DYNAMIC_STORAGE_BIT);
	if (::glGetError() == GL_OUT_OF_MEMORY)
	{
		glDeleteBuffers(1, &buffer);
		return 0;
	}
	return buffer;
}

} // namespace

opengl46_timbers::~opengl46_timbers()
{
	if (m_buffer != 0)
		glDeleteBuffers(1, &m_buffer);
}

void opengl46_timbers::init()
{
	try
	{
		gl::shader shader("timber_instances.comp");
		m_program = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{shader});
	}
	catch (gl::shader_exception const &e)
	{
		// the walk turns the instances on the cpu
		ErrorLog(std::format("track sleepers on the gpu not available: {}", e.what()));
		m_gpu = false;
		return;
	}
	// the modelviews of the origins and the runs, both placed at whole matrices
	m_ranges = std::make_unique<gl::frame_stream>(
		static_cast<GLsizeiptr>(4096 * sizeof(gl::timber_range)), static_cast<GLsizeiptr>(sizeof(glm::mat4)), Global.gfx_bufferstreaming);
	restart(0);
	m_gpu = (false == m_failed);
}

bool opengl46_timbers::restart(std::uint32_t const Needed)
{
	// room for twice the instances to place, and when the buffer was started over not long before, the tracks in sight change
	// fast: twice the buffer before, but not more than restart_growth times that room. a buffer started over later goes back to it
	std::uint64_t minimum{initial_capacity};
	while (minimum < 2 * static_cast<std::uint64_t>(Needed))
		minimum *= 2;
	auto capacity{minimum};
	if ((m_capacity > 0) && (m_frame - m_restartframe < restart_frames))
		capacity = std::clamp<std::uint64_t>(2 * static_cast<std::uint64_t>(m_capacity), minimum, restart_growth * minimum);
	auto buffer{make_buffer(capacity)};
	if ((buffer == 0) && (capacity > Needed))
	{
		// no memory for the room to spare: only the instances to place
		capacity = std::max<std::uint64_t>(Needed, 1);
		buffer = make_buffer(capacity);
	}
	if (buffer == 0)
	{
		// the old buffer stays, the tracks are turned on the cpu from the next frame on
		ErrorLog("openGL error: out of memory; failed to create the buffer of the track sleepers, they're placed by the cpu");
		m_failed = true;
		return false;
	}
	// the draws and passes issued with the old buffer keep it until the gpu is done with them
	if (m_buffer != 0)
		glDeleteBuffers(1, &m_buffer);
	m_buffer = buffer;
	m_capacity = static_cast<std::uint32_t>(capacity);
	m_fill = 0;
	m_restartframe = m_frame;
	m_tracks.clear();
	return true;
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
		if ((false == Restart) || (false == restart(all)))
			return false;
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
	if (m_ranges)
		m_ranges->next_frame();
	// between the frames, so the walks of a frame all gather and draw the timbers the same way
	if (m_failed)
		m_gpu = false;
}
