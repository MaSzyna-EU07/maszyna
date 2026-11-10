/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the background threads of the vulkan renderer (the stage M5 of the plan): the work which would hold up a frame, done while the
// frames go on. the pipelines of the scene and the textures made when a draw first needs them, the pipelines of the last session
// made ahead. what a task makes is taken over by the thread of the renderer when it's done (the frame after, as a rule); a task
// itself touches only what it was given and what is guarded for it

#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace vk14
{

class background_tasks
{
  public:
	background_tasks() = default;
	background_tasks(background_tasks const &) = delete;
	background_tasks &operator=(background_tasks const &) = delete;
	~background_tasks();

	void start(std::size_t Threads);
	// the tasks under way are finished, the ones waiting dropped
	void stop();
	bool running() const
	{
		return false == m_threads.empty();
	}
	// Urgent: before the tasks waiting (what a draw waits for), otherwise after them (what's made ahead)
	void add(std::function<void()> Task, bool Urgent = true);
	// waits until there's nothing waiting and nothing under way
	void wait_idle();
	// the tasks waiting and under way
	std::size_t pending() const;

  private:
	void work();

	mutable std::mutex m_mutex;
	std::condition_variable m_wake;
	std::condition_variable m_idle;
	std::deque<std::function<void()>> m_queue;
	std::vector<std::thread> m_threads;
	std::size_t m_busy{0};
	bool m_stop{false};
};

} // namespace vk14
