/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "utilities/workerpool.h"

#include "utilities/Globals.h"
#include "utilities/utilities.h"

#include <random>

namespace
{

// threads by the setting: all the logical cores of the machine when it's 0 or less, otherwise as many as it says, up to a sane limit
std::size_t wanted_threads()
{
	auto const cores{static_cast<int>(std::max(1u, std::thread::hardware_concurrency()))};
	auto const setting{Global.performance_cpucores};
	return static_cast<std::size_t>(setting > 0 ? std::min(setting, 64) : cores);
}

} // namespace

worker_pool::~worker_pool()
{
	resize(0);
}

std::size_t worker_pool::size()
{
	auto const wanted{wanted_threads()};
	if (wanted != m_threads.size() + 1)
	{
		resize(wanted - 1);
	}
	return m_threads.size() + 1;
}

void worker_pool::resize(std::size_t const Workers)
{
	{
		std::scoped_lock lock(m_mutex);
		m_stop = true;
	}
	m_start.notify_all();
	for (auto &thread : m_threads)
	{
		thread.join();
	}
	m_threads.clear();
	m_stop = false;

	m_threads.reserve(Workers);
	for (std::size_t index = 0; index < Workers; ++index)
	{
		// the generation of the jobs is given from here: read by the thread, a job begun meanwhile would be taken for done
		m_threads.emplace_back(&worker_pool::worker, this, m_generation);
	}
}

void worker_pool::run(std::size_t const Count, std::function<void(std::size_t)> const &Work)
{
	if (Count == 0)
	{
		return;
	}
	if ((size() == 1) || (Count == 1))
	{
		for (std::size_t index = 0; index < Count; ++index)
		{
			Work(index);
		}
		return;
	}

	{
		std::scoped_lock lock(m_mutex);
		m_work = &Work;
		m_count = Count;
		m_next = 0;
		m_busy = m_threads.size();
		m_exception = nullptr;
		++m_generation;
	}
	m_start.notify_all();
	// the calling thread works too
	work(Work, Count);

	std::unique_lock lock(m_mutex);
	m_done.wait(lock, [this]() { return m_busy == 0; });
	m_work = nullptr;
	if (m_exception)
	{
		auto const exception{m_exception};
		m_exception = nullptr;
		lock.unlock();
		std::rethrow_exception(exception);
	}
}

void worker_pool::work(std::function<void(std::size_t)> const &Work, std::size_t const Count)
{
	try
	{
		for (auto index{m_next++}; index < Count; index = m_next++)
		{
			Work(index);
		}
	}
	catch (...)
	{
		std::scoped_lock lock(m_mutex);
		if (false == static_cast<bool>(m_exception))
		{
			m_exception = std::current_exception();
		}
		// the indices left are taken by nobody
		m_next = Count;
	}
}

void worker_pool::worker(unsigned int Generation)
{
	// draws of Random() in this thread go to an engine of its own: the one of the simulation can't be shared between threads
	std::mt19937 randomengine(static_cast<std::mt19937::result_type>(true_random_seed()));
	set_thread_random_engine(&randomengine);

	auto generation{Generation};
	while (true)
	{
		std::function<void(std::size_t)> const *job{nullptr};
		std::size_t count{0};
		{
			std::unique_lock lock(m_mutex);
			m_start.wait(lock, [&]() { return m_stop || (m_generation != generation); });
			if (m_stop)
			{
				break;
			}
			generation = m_generation;
			job = m_work;
			count = m_count;
		}
		work(*job, count);
		{
			std::scoped_lock lock(m_mutex);
			--m_busy;
		}
		m_done.notify_one();
	}
	set_thread_random_engine(nullptr);
}

worker_pool &Workers()
{
	static worker_pool pool;
	return pool;
}
