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
#include <mutex>
#include <thread>
#include <vector>

// threads for work split into independent parts, as many as Global.performance_cpucores asks for (0: all logical cores of the
// machine). the thread which asks for the work takes part in it too, so the pool keeps one thread fewer. the threads wait between
// the jobs, and the pool is made again when the setting changes
class worker_pool
{
  public:
	worker_pool() = default;
	worker_pool(worker_pool const &) = delete;
	worker_pool &operator=(worker_pool const &) = delete;
	~worker_pool();

	// threads which take part in run(), the calling one included, by the current setting
	std::size_t size();
	// calls Work(index) once for each index below Count, spread over the threads, and returns when all the calls are done. the
	// calls of one index follow no order. not reentrant: Work can't call run(). an exception thrown by Work is passed on, once
	// the other calls are done
	void run(std::size_t const Count, std::function<void(std::size_t)> const &Work);

  private:
	void resize(std::size_t const Workers);
	// Generation: of the last job begun before the thread was made
	void worker(unsigned int Generation);
	// takes the indices left of the current job
	void work(std::function<void(std::size_t)> const &Work, std::size_t const Count);

	std::vector<std::thread> m_threads;
	std::mutex m_mutex;
	std::condition_variable m_start;
	std::condition_variable m_done;
	// the current job
	std::function<void(std::size_t)> const *m_work{nullptr};
	std::size_t m_count{0};
	std::atomic<std::size_t> m_next{0};
	std::size_t m_busy{0}; // threads of the pool still in it
	unsigned int m_generation{0}; // of the jobs, for the threads to tell a new one
	std::exception_ptr m_exception;
	bool m_stop{false};
};

// the pool of the application
worker_pool &Workers();
