/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "gl/shader.h"
#include "gl/streambuffer.h"
#include "gl/ubo.h"

class TTrack;

// the instances of the timbers and sleepers of the tracks (track_sleepers::laid) kept on the gpu, those of a track sent once as
// it's laid, and the runs of them drawn made into the modelviews of the instance buffer of the frame by a compute pass
// (timber_instances.comp): for a run the cpu gives only where it lies and where it goes, not its matrices
class opengl46_timbers
{
  public:
	opengl46_timbers() = default;
	opengl46_timbers(opengl46_timbers const &) = delete;
	opengl46_timbers &operator=(opengl46_timbers const &) = delete;
	~opengl46_timbers();

	// throws gl::shader_exception
	void init();
	// the instances of the tracks on the gpu, sent again for a track laid anew since; when those to send don't fit in what's left
	// of the buffer, it's started over for all the tracks, a bigger one when it was started over not long before. without
	// Restart nothing is sent then, and false returned: the runs of the instances placed before are to be expanded first
	bool place(std::vector<TTrack *> const &Tracks, bool const Restart);
	// where the instances of a track placed lie in the buffer: the first of the parts in turn, then of the whole timbers, then of
	// the plain sleepers
	std::uint32_t first(TTrack const &Track) const;
	// the runs written to the instance buffer Output, each as the modelview of its origin times its transforms, for the draws after
	void expand(std::vector<glm::mat4> const &Origins, std::vector<gl::timber_range> const &Ranges, GLuint const Output);
	void next_frame();

  private:
	struct placed
	{
		std::uint64_t made; // the making of the instances of the track sent
		std::uint32_t first;
	};

	// a new buffer, room for Needed instances at least, the tracks to be sent again
	void restart(std::uint32_t const Needed);

	std::unique_ptr<gl::program> m_program;
	std::unique_ptr<gl::frame_stream> m_ranges;
	GLuint m_buffer{0};
	std::uint32_t m_capacity{0}; // in instances
	std::uint32_t m_fill{0};
	unsigned int m_frame{0};
	unsigned int m_restartframe{0};
	std::unordered_map<TTrack const *, placed> m_tracks;
	std::vector<std::uint8_t> m_upload; // the origins and the runs of an expand(), as they're sent
};
