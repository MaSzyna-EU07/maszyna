/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14tasks.h"
#include "utilities/Logs.h"

#include <exception>

namespace vk14
{

background_tasks::~background_tasks()
{
	stop();
}

void background_tasks::start(std::size_t const Threads)
{
	if (running())
	{
		return;
	}
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_stop = false;
	}
	for (std::size_t i = 0; i < Threads; ++i)
	{
		m_threads.emplace_back([this]() { work(); });
	}
}

void background_tasks::stop()
{
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_stop = true;
		m_queue.clear();
	}
	m_wake.notify_all();
	for (auto &thread : m_threads)
	{
		thread.join();
	}
	m_threads.clear();
}

void background_tasks::add(std::function<void()> Task, bool const Urgent)
{
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		if (Urgent)
		{
			m_queue.emplace_front(std::move(Task));
		}
		else
		{
			m_queue.emplace_back(std::move(Task));
		}
	}
	m_wake.notify_one();
}

void background_tasks::wait_idle()
{
	std::unique_lock<std::mutex> lock(m_mutex);
	m_idle.wait(lock, [this]() { return m_queue.empty() && (m_busy == 0); });
}

std::size_t background_tasks::pending() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_queue.size() + m_busy;
}

void background_tasks::work()
{
	std::unique_lock<std::mutex> lock(m_mutex);
	while (true)
	{
		m_wake.wait(lock, [this]() { return m_stop || (false == m_queue.empty()); });
		if (m_stop)
		{
			return;
		}
		auto task{std::move(m_queue.front())};
		m_queue.pop_front();
		++m_busy;
		lock.unlock();
		try
		{
			task();
		}
		catch (std::exception const &Exception)
		{
			ErrorLog(std::string("vulkan: a background task failed: ") + Exception.what());
		}
		lock.lock();
		--m_busy;
		if (m_queue.empty() && (m_busy == 0))
		{
			m_idle.notify_all();
		}
	}
}

} // namespace vk14
