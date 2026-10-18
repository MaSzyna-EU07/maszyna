#include "stdafx.h"
#include "query.h"
#include "utilities/Globals.h"

#include <cstring>

gl::query::query(targets target)
    : target(target)
{
	glGenQueries(1, *this);
}

gl::query::~query()
{
	end();
	glDeleteQueries(1, *this);
}

void gl::query::begin()
{
	if (active_queries[target])
		active_queries[target]->end();

	glBeginQuery(glenum_target(target), *this);
	active_queries[target] = this;
}

void gl::query::end()
{
	if (active_queries[target] != this)
		return;

	glEndQuery(glenum_target(target));
	active_queries[target] = nullptr;
}

std::optional<int64_t> gl::query::result()
{
    end(); // intercept potential error if the result check is called for still active object
    GLuint ready;
	glGetQueryObjectuiv(*this, GL_QUERY_RESULT_AVAILABLE, &ready);
	int64_t value = 0;
	if (ready) {
		glGetQueryObjecti64v(*this, GL_QUERY_RESULT, &value);

		return std::optional<int64_t>(value);
	}
	return std::nullopt;
}

GLenum gl::query::glenum_target(targets Target) const
{
	static std::array<GLenum, 6> const mapping =
	{
	    GL_SAMPLES_PASSED,
	    GL_ANY_SAMPLES_PASSED,
	    GL_ANY_SAMPLES_PASSED_CONSERVATIVE,
	    GL_PRIMITIVES_GENERATED,
	    GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN,
	    GL_TIME_ELAPSED
	};
	return mapping[Target];
}

thread_local gl::query* gl::query::active_queries[6];

gl::gpu_timer::~gpu_timer()
{
	if (m_queries[0] != 0)
		glDeleteQueries(static_cast<GLsizei>(m_queries.size()), m_queries.data());
}

void gl::gpu_timer::begin()
{
	if (m_queries[0] == 0)
		glCreateQueries(GL_TIMESTAMP, static_cast<GLsizei>(m_queries.size()), m_queries.data());

	collect();
	m_active = !m_pending[m_slot];
	if (m_active)
		glQueryCounter(m_queries[m_slot * 2], GL_TIMESTAMP);
}

void gl::gpu_timer::end()
{
	if (!m_active)
		return;

	glQueryCounter(m_queries[m_slot * 2 + 1], GL_TIMESTAMP);
	m_pending[m_slot] = true;
	m_slot = (m_slot + 1) % SLOTS;
	m_active = false;
}

std::optional<uint64_t> gl::gpu_timer::result()
{
	if (m_queries[0] != 0)
		collect();

	return m_last;
}

void gl::gpu_timer::collect()
{
	// slots finish in the order they were issued, oldest first
	for (size_t i = 0; i < SLOTS; ++i)
	{
		size_t const slot = (m_slot + i) % SLOTS;
		if (!m_pending[slot])
			continue;

		GLint startready = GL_FALSE;
		GLint endready = GL_FALSE;
		glGetQueryObjectiv(m_queries[slot * 2], GL_QUERY_RESULT_AVAILABLE, &startready);
		glGetQueryObjectiv(m_queries[slot * 2 + 1], GL_QUERY_RESULT_AVAILABLE, &endready);
		if (!startready || !endready)
			break;

		GLuint64 start = 0;
		GLuint64 end = 0;
		glGetQueryObjectui64v(m_queries[slot * 2], GL_QUERY_RESULT, &start);
		glGetQueryObjectui64v(m_queries[slot * 2 + 1], GL_QUERY_RESULT, &end);
		m_last = end >= start ? end - start : 0;
		m_pending[slot] = false;
	}
}

gl::gpu_timeline::~gpu_timeline()
{
	for (auto &frame : m_frames)
	{
		if (frame.queries[0] != 0)
			glDeleteQueries(static_cast<GLsizei>(frame.queries.size()), frame.queries.data());
	}
}

void gl::gpu_timeline::begin(char const *Name)
{
	if (m_active)
		end();

	collect();
	auto &frame = m_frames[m_slot];
	m_active = !frame.pending;
	if (!m_active)
		return;

	if (frame.queries[0] == 0)
		glCreateQueries(GL_TIMESTAMP, static_cast<GLsizei>(frame.queries.size()), frame.queries.data());
	frame.count = 0;
	next(Name);
}

void gl::gpu_timeline::next(char const *Name)
{
	if (!m_active)
		return;

	auto &frame = m_frames[m_slot];
	// the last mark is kept for end(); past the limit the part under way goes on
	if (frame.count + 1 >= MARKS)
		return;

	glQueryCounter(frame.queries[frame.count], GL_TIMESTAMP);
	frame.names[frame.count] = Name;
	++frame.count;
}

void gl::gpu_timeline::end()
{
	if (!m_active)
		return;

	auto &frame = m_frames[m_slot];
	glQueryCounter(frame.queries[frame.count], GL_TIMESTAMP);
	frame.names[frame.count] = nullptr;
	++frame.count;
	frame.pending = true;
	m_slot = (m_slot + 1) % SLOTS;
	m_active = false;
}

std::vector<gl::gpu_timeline::part> const &gl::gpu_timeline::result()
{
	collect();
	return m_last;
}

void gl::gpu_timeline::collect()
{
	// frames finish in the order they were issued, oldest first
	for (size_t i = 0; i < SLOTS; ++i)
	{
		auto &frame = m_frames[(m_slot + i) % SLOTS];
		if (!frame.pending)
			continue;

		auto ready = true;
		for (size_t mark = 0; (mark < frame.count) && ready; ++mark)
		{
			GLint available = GL_FALSE;
			glGetQueryObjectiv(frame.queries[mark], GL_QUERY_RESULT_AVAILABLE, &available);
			ready = (available != GL_FALSE);
		}
		if (!ready)
			break;

		m_last.clear();
		GLuint64 first = 0;
		GLuint64 previous = 0;
		for (size_t mark = 0; mark < frame.count; ++mark)
		{
			GLuint64 time = 0;
			glGetQueryObjectui64v(frame.queries[mark], GL_QUERY_RESULT, &time);
			if (mark == 0)
				first = time;
			else
			{
				auto const length = (time >= previous ? time - previous : 0);
				auto const *name = frame.names[mark - 1];
				auto lookup = m_last.begin();
				while ((lookup != m_last.end()) && (std::strcmp(lookup->name, name) != 0))
					++lookup;
				if (lookup != m_last.end())
					lookup->time += length;
				else
					m_last.push_back({name, length});
			}
			previous = time;
		}
		m_total = (previous >= first ? previous - first : 0);
		frame.pending = false;
	}
}
