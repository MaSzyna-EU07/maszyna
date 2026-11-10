/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <semaphore>
#include <thread>
#include <vector>

// threads for work split into independent parts (Global.performance_multithreading). each job says how many threads its work is
// worth, up to all the logical cores of the machine; the thread which asks for the work takes part in it too. the threads are
// made when a job asks for more of them than there are, and wait between the jobs: a job wakes as many of them as it wants, each
// by a semaphore of its own
class worker_pool
{
  public:
	worker_pool() = default;
	worker_pool(worker_pool const &) = delete;
	worker_pool &operator=(worker_pool const &) = delete;
	~worker_pool();

	// threads a job can get, the calling one included: all the logical cores with Global.performance_multithreading, otherwise one
	static std::size_t capacity();
	// calls Work(index) once for each index below Count, on at most Threads threads (the calling one included, at most capacity()),
	// and returns when all the calls are done. the calls follow no order. not reentrant: Work can't call run(). an exception
	// thrown by Work is passed on, once the other calls are done
	void run(std::size_t const Count, std::size_t const Threads, std::function<void(std::size_t)> const &Work);
	// threads made so far, the calling one included
	std::size_t size() const
	{
		return m_threads.size() + 1;
	}

  private:
	struct thread_data
	{
		std::binary_semaphore start{0}; // released for each job the thread takes part in, and to stop it
		std::thread thread;
	};

	// makes threads up to the given count
	void grow(std::size_t const Workers);
	void worker(thread_data &Data);
	// takes the indices left of the current job
	void work(std::function<void(std::size_t)> const &Work, std::size_t const Count);

	std::vector<std::unique_ptr<thread_data>> m_threads;
	std::mutex m_mutex;
	std::condition_variable m_done;
	// the current job, set before the threads are woken for it
	std::function<void(std::size_t)> const *m_work{nullptr};
	std::size_t m_count{0};
	std::atomic<std::size_t> m_next{0};
	std::size_t m_busy{0}; // threads of the pool woken for it which aren't done
	std::exception_ptr m_exception;
	std::atomic<bool> m_stop{false};
};

// the pool of the application
worker_pool &Workers();
