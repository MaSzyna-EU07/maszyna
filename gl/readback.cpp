#include "stdafx.h"
#include "readback.h"

gl::readback::readback(GLsizeiptr size) : m_size(size) {}

gl::readback::~readback()
{
	for (auto &slot : m_slots)
	{
		// fences first, they belong to the copies
		slot.copied.reset();
		if (slot.buffer != 0)
			glDeleteBuffers(1, &slot.buffer);
	}
}

void gl::readback::push(GLuint Source, GLintptr Offset)
{
	if (m_used >= SLOTS)
		return;

	auto &slot{m_slots[(m_first + m_used) % SLOTS]};
	if (slot.buffer == 0)
	{
		glCreateBuffers(1, &slot.buffer);
		glNamedBufferStorage(slot.buffer, m_size, nullptr, GL_CLIENT_STORAGE_BIT);
	}
	glCopyNamedBufferSubData(Source, slot.buffer, Offset, 0, m_size);
	slot.copied = std::make_unique<fence>();
	++m_used;
}

bool gl::readback::pop(void *Data)
{
	bool result{false};
	// oldest first, as long as the gpu has finished the copies; the newest finished one ends in Data
	while (m_used > 0)
	{
		auto &slot{m_slots[m_first]};
		if (!slot.copied->is_signalled())
			break;

		glGetNamedBufferSubData(slot.buffer, 0, m_size, Data);
		slot.copied.reset();
		result = true;

		m_first = (m_first + 1) % SLOTS;
		--m_used;
	}
	return result;
}
