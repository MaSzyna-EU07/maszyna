#include "stdafx.h"
#include "ssbo.h"

gl::ssbo::ssbo(GLsizeiptr size, GLuint index) : m_index(index)
{
	create(size);
}

gl::ssbo::~ssbo()
{
	glDeleteBuffers(1, *this);
}

void gl::ssbo::create(GLsizeiptr size)
{
	glCreateBuffers(1, *this);
	// written only through glNamedBufferSubData and by shaders
	glNamedBufferStorage(*this, size, nullptr, GL_DYNAMIC_STORAGE_BIT);
	m_size = size;
}

void gl::ssbo::bind_base() const
{
	glBindBuffersBase(GL_SHADER_STORAGE_BUFFER, m_index, 1, *this);
}

void gl::ssbo::resize(GLsizeiptr size)
{
	if (size == m_size)
		return;

	glDeleteBuffers(1, *this);
	create(size);
}

void gl::ssbo::update(const void *data, GLintptr offset, GLsizeiptr size)
{
	glNamedBufferSubData(*this, offset, size, data);
}

void gl::ssbo::download(void *data, GLintptr offset, GLsizeiptr size) const
{
	glGetNamedBufferSubData(*this, offset, size, data);
}

void gl::memory_barrier(GLbitfield barriers)
{
	glMemoryBarrier(barriers);
}
