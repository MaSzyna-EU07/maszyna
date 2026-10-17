#include "stdafx.h"
#include "streambuffer.h"

#include <algorithm>
#include <cstring>
#include <string>

#include "utilities/Logs.h"

namespace
{
// a single frame isn't given more than this; past it the frame waits for the gpu to finish everything and starts the buffer over
constexpr GLsizeiptr STREAM_SIZE_MAX = 256 * 1024 * 1024;
constexpr GLbitfield STREAM_MAP_FLAGS = GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT;

GLsizeiptr align_up(GLsizeiptr const Value, GLsizeiptr const Alignment)
{
	return (Value + Alignment - 1) / Alignment * Alignment;
}
} // namespace

gl::frame_stream::frame_stream(GLsizeiptr Size, GLsizeiptr Alignment, bool Mapped, void const *Header, GLsizeiptr Headersize)
    : m_alignment(std::max<GLsizeiptr>(Alignment, 1)), m_mapped(Mapped)
{
	if (Header != nullptr && Headersize > 0)
	{
		auto const *bytes{static_cast<std::uint8_t const *>(Header)};
		m_header.assign(bytes, bytes + Headersize);
	}
	m_start = align_up(static_cast<GLsizeiptr>(m_header.size()), m_alignment);
	m_fill = m_start;
	Size = std::max(Size, m_start + m_alignment);
	for (auto &slot : m_slots)
	{
		create(slot, Size);
	}
}

gl::frame_stream::~frame_stream()
{
	for (auto &slot : m_slots)
	{
		release(slot);
	}
}

void gl::frame_stream::create(slot &Slot, GLsizeiptr Size)
{
	++m_generation;
	glCreateBuffers(1, &Slot.buffer);
	Slot.size = Size;
	Slot.data = nullptr;
	if (m_mapped)
	{
		glNamedBufferStorage(Slot.buffer, Size, nullptr, STREAM_MAP_FLAGS);
		Slot.data = static_cast<std::uint8_t *>(glMapNamedBufferRange(Slot.buffer, 0, Size, STREAM_MAP_FLAGS));
		if (Slot.data == nullptr)
		{
			// the driver wouldn't map it: this buffer and the ones made later are written the plain way
			ErrorLog("Bad buffer: persistent mapping failed, buffer streaming falls back to glNamedBufferSubData");
			m_mapped = false;
			glDeleteBuffers(1, &Slot.buffer);
			glCreateBuffers(1, &Slot.buffer);
		}
	}
	if (!m_mapped)
	{
		glNamedBufferStorage(Slot.buffer, Size, nullptr, GL_DYNAMIC_STORAGE_BIT);
	}
	if (!m_header.empty())
	{
		if (Slot.data != nullptr)
			std::memcpy(Slot.data, m_header.data(), m_header.size());
		else
			glNamedBufferSubData(Slot.buffer, 0, static_cast<GLsizeiptr>(m_header.size()), m_header.data());
	}
}

void gl::frame_stream::release(slot &Slot)
{
	if (Slot.fence != nullptr)
	{
		glDeleteSync(Slot.fence);
		Slot.fence = nullptr;
	}
	if (Slot.buffer != 0)
	{
		// a mapped buffer is unmapped by its deletion. draws issued before which read it keep its storage until they are done
		glDeleteBuffers(1, &Slot.buffer);
		Slot.buffer = 0;
	}
	Slot.data = nullptr;
	Slot.size = 0;
}

void gl::frame_stream::wait(slot &Slot)
{
	if (Slot.fence == nullptr)
		return;

	auto result{glClientWaitSync(Slot.fence, 0, 0)};
	if (result == GL_TIMEOUT_EXPIRED)
	{
		++m_waits;
		do
		{
			result = glClientWaitSync(Slot.fence, GL_SYNC_FLUSH_COMMANDS_BIT, 1000000000);
		} while (result == GL_TIMEOUT_EXPIRED);
	}
	glDeleteSync(Slot.fence);
	Slot.fence = nullptr;
}

void gl::frame_stream::next_frame()
{
	auto &current{m_slots[m_slot]};
	if (current.data != nullptr)
	{
		// the cpu writes to a mapped buffer behind the back of the driver, the frames take turns by the fences
		if (current.fence != nullptr)
			glDeleteSync(current.fence);
		current.fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
		// the fence may be waited for in another context, whose flush wouldn't send it
		glFlush();
	}
	m_slot = (m_slot + 1) % FRAMES;
	wait(m_slots[m_slot]);
	m_fill = m_start;
}

GLintptr gl::frame_stream::push(void const *Data, GLsizeiptr Size)
{
	auto *current{&m_slots[m_slot]};
	auto offset{align_up(m_fill, m_alignment)};
	if (offset + Size > current->size)
	{
		auto const needed{m_start + align_up(Size, m_alignment)};
		auto const size{needed > STREAM_SIZE_MAX ? needed : std::min(std::max(needed, 2 * current->size), STREAM_SIZE_MAX)};
		if (size > current->size)
		{
			// a bigger buffer for the rest of the frame, starting after its header again. the fence of the slot (if any) was
			// waited for when the frame took the slot over, the old buffer is only read by the draws of this frame
			release(*current);
			create(*current, size);
			++m_grows;
		}
		else
		{
			// the frame wrote as much as a buffer may hold: start it over once the gpu read all of it
			ErrorLog("Bad buffer: a frame streamed more than " + std::to_string(STREAM_SIZE_MAX / (1024 * 1024)) + " MiB, waiting for the gpu");
			glFinish();
			++m_waits;
		}
		current = &m_slots[m_slot];
		offset = m_start;
	}
	if (current->data != nullptr)
		std::memcpy(current->data + offset, Data, static_cast<std::size_t>(Size));
	else
		glNamedBufferSubData(current->buffer, offset, Size, Data);
	m_fill = offset + Size;
	return offset;
}

gl::stream_ubo::stream_ubo(GLsizeiptr Blocksize, GLuint Index, bool Streamed) : m_index(Index), m_blocksize(Blocksize)
{
	if (!Streamed)
	{
		m_plain = std::make_unique<ubo>(static_cast<size_t>(Blocksize), static_cast<int>(Index), GL_STREAM_DRAW);
		return;
	}
	GLint alignment{256};
	glGetIntegerv(GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT, &alignment);
	// a frame draws some thousands of times; the buffers grow when it draws more
	m_stream = std::make_unique<frame_stream>(align_up(Blocksize, std::max<GLsizeiptr>(alignment, 1)) * 8192, std::max<GLint>(alignment, 1), true);
	m_last.resize(static_cast<std::size_t>(Blocksize));
}

void gl::stream_ubo::update(void const *Data, GLsizeiptr Size)
{
	++m_updates;
	if (m_plain)
	{
		m_plain->update(static_cast<uint8_t const *>(Data), 0, Size);
		return;
	}
	m_boundoffset = m_stream->push(Data, Size);
	m_boundbuffer = m_stream->buffer();
	m_lastsize = std::min(Size, m_blocksize);
	if (Data != m_last.data())
		std::memcpy(m_last.data(), Data, static_cast<std::size_t>(m_lastsize));
	// the plural form leaves the general GL_UNIFORM_BUFFER binding alone, which the binding tracker of gl::buffer relies on
	GLsizeiptr const size{Size};
	glBindBuffersRange(GL_UNIFORM_BUFFER, m_index, 1, &m_boundbuffer, &m_boundoffset, &size);
}

void gl::stream_ubo::bind_uniform()
{
	if (m_plain)
	{
		m_plain->bind_uniform();
		return;
	}
	if (m_boundbuffer == 0)
		return;
	GLsizeiptr const size{m_lastsize};
	glBindBuffersRange(GL_UNIFORM_BUFFER, m_index, 1, &m_boundbuffer, &m_boundoffset, &size);
}

void gl::stream_ubo::next_frame()
{
	m_updates = 0;
	if (m_plain)
		return;
	m_stream->next_frame();
	if (m_lastsize > 0)
	{
		update(m_last.data(), m_lastsize);
		m_updates = 0;
	}
}
