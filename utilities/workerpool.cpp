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

worker_pool::~worker_pool()
{
	m_stop = true;
	for (auto &data : m_threads)
	{
		data->start.release();
	}
	for (auto &data : m_threads)
	{
		data->thread.join();
	}
}

std::size_t worker_pool::capacity()
{
	if (false == Global.performance_multithreading)
	{
		return 1;
	}
	return std::max<std::size_t>(1, std::thread::hardware_concurrency());
}

void worker_pool::grow(std::size_t const Workers)
{
	m_threads.reserve(Workers);
	while (m_threads.size() < Workers)
	{
		auto &data{*m_threads.emplace_back(std::make_unique<thread_data>())};
		data.thread = std::thread(&worker_pool::worker, this, std::ref(data));
	}
}

void worker_pool::run(std::size_t const Count, std::size_t const Threads, std::function<void(std::size_t)> const &Work)
{
	auto const threads{std::min({Threads, capacity(), Count})};
	if (threads <= 1)
	{
		for (std::size_t index = 0; index < Count; ++index)
		{
			Work(index);
		}
		return;
	}
	grow(threads - 1);

	{
		std::scoped_lock lock(m_mutex);
		m_work = &Work;
		m_count = Count;
		m_next = 0;
		m_busy = threads - 1;
		m_exception = nullptr;
	}
	// the job is set before the threads are woken: the semaphores pass it on to them
	for (std::size_t thread = 0; thread < threads - 1; ++thread)
	{
		m_threads[thread]->start.release();
	}
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

void worker_pool::worker(thread_data &Data)
{
	// draws of Random() in this thread go to an engine of its own: the one of the simulation can't be shared between threads
	std::mt19937 randomengine(static_cast<std::mt19937::result_type>(true_random_seed()));
	set_thread_random_engine(&randomengine);

	while (true)
	{
		Data.start.acquire();
		if (m_stop)
		{
			break;
		}
		std::function<void(std::size_t)> const *job{nullptr};
		std::size_t count{0};
		{
			std::scoped_lock lock(m_mutex);
			job = m_work;
			count = m_count;
		}
		work(*job, count);
		{
			std::scoped_lock lock(m_mutex);
			--m_busy;
			// notified under the lock: the job and the pool stay as they are until the caller sees the count
			m_done.notify_one();
		}
	}
	set_thread_random_engine(nullptr);
}

worker_pool &Workers()
{
	static worker_pool pool;
	return pool;
}
