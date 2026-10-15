#pragma once

#include <type_traits>

#include "object.h"

namespace gl
{
	// shader storage buffer (std430 data, compute shader input and output), on 4.6 core direct state access.
	// the storage is immutable: resize() to a different size replaces the buffer object, drops its contents and
	// leaves it unbound (call bind_base() again).
	// binding goes through glBindBuffersBase, which leaves the general GL_SHADER_STORAGE_BUFFER binding point
	// alone, so the binding tracker of gl::buffer stays valid
	class ssbo : public object
	{
	  public:
		// size in bytes, > 0. index is the binding point used by the shaders: layout(std430, binding = index)
		ssbo(GLsizeiptr size, GLuint index);
		~ssbo();

		void bind_base() const;
		void resize(GLsizeiptr size);
		GLsizeiptr size() const
		{
			return m_size;
		}

		void update(const void *data, GLintptr offset, GLsizeiptr size);
		template <typename T> void update(const T &data, GLintptr offset = 0)
		{
			static_assert(std::is_trivially_copyable_v<T>, "ssbo data must be trivially copyable");
			update(&data, offset, sizeof(data));
		}
		// synchronous read back, waits for the gpu; for debug data, not for per-frame use
		void download(void *data, GLintptr offset, GLsizeiptr size) const;

	  private:
		void create(GLsizeiptr size);

		GLuint m_index;
		GLsizeiptr m_size = 0;
	};

	// makes writes done by shaders (compute dispatches, storage buffer stores) visible to the operations named
	// by barriers, e.g. GL_SHADER_STORAGE_BARRIER_BIT before a draw that reads what a dispatch wrote
	void memory_barrier(GLbitfield barriers);
} // namespace gl
