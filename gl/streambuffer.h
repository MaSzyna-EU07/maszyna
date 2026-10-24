#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <type_traits>
#include <vector>

#include "object.h"
#include "ubo.h"

namespace gl
{
	// data the cpu writes for the draws of one frame (the uniform block of every draw, matrices of instances), appended to the
	// buffer of the frame. the frames use FRAMES buffers in turn. mapped: the buffers are persistently mapped and written with
	// memcpy, a fence marks the end of each frame, and a frame waits for the fence of the buffer it takes over (written
	// FRAMES - 1 frames before; normally the gpu is long done with it). not mapped: written with glNamedBufferSubData, the driver
	// keeps it in step with the gpu, as with any other buffer
	class frame_stream
	{
	  public:
		static constexpr std::size_t FRAMES = 3;

		// Size: initial size of the buffer of a frame, it grows when a frame needs more. Alignment: of the offsets push()
		// returns. Header: bytes placed at the start of every buffer, never overwritten (nullptr: none)
		frame_stream(GLsizeiptr Size, GLsizeiptr Alignment, bool Mapped, void const *Header = nullptr, GLsizeiptr Headersize = 0);
		~frame_stream();
		frame_stream(frame_stream const &) = delete;
		frame_stream &operator=(frame_stream const &) = delete;

		// ends the frame: what was written so far goes with its fence, the next frame writes to the next buffer
		void next_frame();
		// appends Size bytes to the buffer of the frame and returns their offset. may replace the buffer of the frame with a
		// bigger one (generation() changes, buffer() tells which to bind); the old one stays until the frame is done with, so
		// what was written to it can still be bound and drawn from later in the frame
		GLintptr push(void const *Data, GLsizeiptr Size);

		GLuint buffer() const
		{
			return m_slots[m_slot].buffer;
		}
		// changes whenever buffer() is replaced by a new buffer object. the name alone can't tell: gl hands the name of a
		// deleted buffer to the next one made
		std::uint64_t generation() const
		{
			return m_generation;
		}
		// size of the buffer of the frame, and bytes written to it in this frame (after the header)
		GLsizeiptr size() const
		{
			return m_slots[m_slot].size;
		}
		GLsizeiptr used() const
		{
			return m_fill - m_start;
		}
		// the buffer of the frame is persistently mapped
		bool mapped() const
		{
			return m_slots[m_slot].data != nullptr;
		}
		// frames which had to wait for the gpu before writing, buffers grown, since the start
		std::uint64_t waits() const
		{
			return m_waits;
		}
		std::uint64_t grows() const
		{
			return m_grows;
		}

	  private:
		struct slot
		{
			GLuint buffer = 0;
			GLsizeiptr size = 0;
			std::uint8_t *data = nullptr; // mapped
			GLsync fence = nullptr; // mapped: end of the last frame which wrote to the buffer
		};

		void create(slot &Slot, GLsizeiptr Size);
		void release(slot &Slot);
		void wait(slot &Slot);

		std::array<slot, FRAMES> m_slots;
		// buffers of the slots replaced by bigger ones in their frame, deleted when the slot is taken over again
		std::array<std::vector<GLuint>, FRAMES> m_retired;
		std::size_t m_slot = 0;
		GLsizeiptr m_alignment;
		bool m_mapped; // buffers made from now on are mapped
		std::vector<std::uint8_t> m_header;
		GLsizeiptr m_start; // the header, aligned
		GLsizeiptr m_fill;
		std::uint64_t m_waits = 0;
		std::uint64_t m_grows = 0;
		std::uint64_t m_generation = 0;
	};

	// uniform block written anew for (nearly) every draw, the model block of the renderer. streamed: every update() goes to
	// its own place in a frame_stream and is bound there with glBindBuffersRange. not streamed: one plain ubo, updated with
	// glBufferSubData, as before
	class stream_ubo
	{
	  public:
		stream_ubo(GLsizeiptr Blocksize, GLuint Index, bool Streamed);

		void update(void const *Data, GLsizeiptr Size);
		template <typename T> void update(T const &Data)
		{
			static_assert(std::is_trivially_copyable_v<T>, "ubo data must be trivially copyable");
			update(&Data, static_cast<GLsizeiptr>(sizeof(Data)));
		}
		// binds the block written last (to a new context)
		void bind_uniform();
		// call at the start of a frame: the last block is copied to the buffer of the new frame and bound, so nothing drawn in
		// it reads a buffer the frame doesn't fence
		void next_frame();

		bool streamed() const
		{
			return m_stream != nullptr;
		}
		frame_stream const *stream() const
		{
			return m_stream.get();
		}
		// blocks written in this frame
		std::uint64_t updates() const
		{
			return m_updates;
		}

	  private:
		GLuint m_index;
		GLsizeiptr m_blocksize;
		std::unique_ptr<ubo> m_plain;
		std::unique_ptr<frame_stream> m_stream;
		std::vector<std::uint8_t> m_last; // streamed: copy of the last block
		GLsizeiptr m_lastsize = 0;
		GLuint m_boundbuffer = 0;
		GLintptr m_boundoffset = 0;
		std::uint64_t m_updates = 0;
	};
} // namespace gl
