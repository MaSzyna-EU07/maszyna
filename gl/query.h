#pragma once

#include <array>
#include <optional>

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
}
