#pragma once

#include <array>
#include <optional>
#include <vector>

#include "object.h"

namespace gl
{
class query : public object
{
public:
	enum targets
	{
		SAMPLES_PASSED = 0,
		ANY_SAMPLES_PASSED,
		ANY_SAMPLES_PASSED_CONSERVATIVE,
		PRIMITIVES_GENERATED,
		TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN,
		TIME_ELAPSED
	};

private:
	targets target;
	thread_local static query* active_queries[6];

protected:
	GLenum glenum_target(targets target) const;

public:
	explicit query(targets target);
	~query();

	void begin();
	void end();

	std::optional<int64_t> result();
};

// gpu time of a section of commands, from a pair of GL_TIMESTAMP queries. unlike GL_TIME_ELAPSED, timestamps
// can be taken while another time query runs (e.g. the whole-frame query of the renderer). results are read
// a few frames later without stalling; if the gpu falls further behind, the measurement is skipped
class gpu_timer
{
  public:
	gpu_timer() = default;
	~gpu_timer();
	gpu_timer(const gpu_timer &) = delete;
	gpu_timer &operator=(const gpu_timer &) = delete;

	void begin();
	void end();
	// duration of the most recent finished measurement, in nanoseconds
	std::optional<uint64_t> result();

  private:
	void collect();

	static constexpr size_t SLOTS = 4;
	// start and end timestamp per slot, created on first use
	std::array<GLuint, SLOTS * 2> m_queries{};
	std::array<bool, SLOTS> m_pending{};
	// slot written by the next begin(), also the oldest one still pending
	size_t m_slot = 0;
	bool m_active = false;
	std::optional<uint64_t> m_last;
};

// gpu time of the parts of a frame, from GL_TIMESTAMP queries put one after another: a part lasts from its mark to the next
// one, so the parts add up to the time from the first mark to the last, with nothing left out and nothing counted twice.
// the first mark of a gpu_timer is put while the gpu may still be busy with what came before, so its time can hold the tail
// of that work; in the run of marks that time goes to the part it belongs to. results are read a few frames later without
// stalling; if the gpu falls further behind, the frame is skipped
class gpu_timeline
{
  public:
	struct part
	{
		char const *name;
		uint64_t time; // nanoseconds
	};

	gpu_timeline() = default;
	~gpu_timeline();
	gpu_timeline(const gpu_timeline &) = delete;
	gpu_timeline &operator=(const gpu_timeline &) = delete;

	// starts the frame with its first part. the names are kept as pointers: string literals
	void begin(char const *Name);
	// ends the part under way and starts the next one
	void next(char const *Name);
	void end();
	bool active() const
	{
		return m_active;
	}
	// parts of the most recent finished frame in the order they came first, the time of parts of the same name added up
	std::vector<part> const &result();
	// from the first mark of that frame to the last
	uint64_t total() const
	{
		return m_total;
	}

  private:
	void collect();

	static constexpr size_t SLOTS = 4;
	static constexpr size_t MARKS = 32; // per frame, the last one included
	struct frame
	{
		std::array<GLuint, MARKS> queries{};
		std::array<char const *, MARKS> names{};
		size_t count = 0; // marks put
		bool pending = false;
	};
	std::array<frame, SLOTS> m_frames;
	// slot written by the next begin(), also the oldest one still pending
	size_t m_slot = 0;
	bool m_active = false;
	std::vector<part> m_last;
	uint64_t m_total = 0;
};
}
