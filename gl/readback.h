#pragma once

#include <array>
#include <memory>

#include "object.h"
#include "fence.h"

namespace gl
{
	// copies of a small range of a buffer the gpu writes, for the cpu to read a few frames later without waiting for the
	// gpu: a queue of buffers, each with the fence of the copy into it. if the gpu is so far behind that every buffer is
	// still waiting, the copy is skipped
	class readback
	{
	  public:
		explicit readback(GLsizeiptr size);
		~readback();
		readback(const readback &) = delete;
		readback &operator=(const readback &) = delete;

		// queues a copy of size bytes of Source, from Offset
		void push(GLuint Source, GLintptr Offset = 0);
		// writes the newest copy which the gpu has finished to Data and returns true; false if none was finished since the last call
		bool pop(void *Data);

	  private:
		static constexpr size_t SLOTS = 4;
		struct slot
		{
			GLuint buffer = 0;
			std::unique_ptr<fence> copied;
		};
		GLsizeiptr m_size;
		std::array<slot, SLOTS> m_slots;
		// oldest slot in use and the number of slots in use
		size_t m_first = 0;
		size_t m_used = 0;
	};
} // namespace gl
