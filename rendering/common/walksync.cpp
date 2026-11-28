/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/common/walksync.h"

#include "utilities/Logs.h"

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <shared_mutex>
#include <string>

namespace gfx
{
namespace walk_sync
{

namespace
{

// the walk lock: shared by the walks, exclusive for a change. a change waiting goes before the walks asking for the lock again
// (yield()), which the locks of the standard library don't promise
class walk_lock
{
  public:
	void lock_shared()
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		m_wake.wait(lock, [this]() { return (false == m_writer) && (m_waiting.load(std::memory_order_relaxed) == 0); });
		++m_readers;
	}
	void unlock_shared()
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		if (--m_readers == 0)
		{
			m_wake.notify_all();
		}
	}
	void lock()
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		m_waiting.fetch_add(1, std::memory_order_relaxed);
		m_wake.wait(lock, [this]() { return (false == m_writer) && (m_readers == 0); });
		m_waiting.fetch_sub(1, std::memory_order_relaxed);
		m_writer = true;
	}
	void unlock()
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_writer = false;
		m_wake.notify_all();
	}
	bool waiting() const
	{
		return m_waiting.load(std::memory_order_relaxed) > 0;
	}

  private:
	std::mutex m_mutex;
	std::condition_variable m_wake;
	int m_readers{0};
	bool m_writer{false};
	std::atomic<int> m_waiting{0};
};

// the frame of the main thread and the changes of the threads loading the scenery in the background: one at a time, a frame waiting
// goes before the changes waiting
class background_lock
{
  public:
	void lock_frame()
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		++m_framewaiting;
		m_wake.wait(lock, [this]() { return false == m_changing; });
		--m_framewaiting;
		m_frame = true;
	}
	void unlock_frame()
	{
		{
			std::scoped_lock const lock{m_mutex};
			m_frame = false;
		}
		m_wake.notify_all();
	}
	void lock_change()
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		m_wake.wait(lock, [this]() { return (false == m_frame) && (m_framewaiting == 0) && (false == m_changing); });
		m_changing = true;
	}
	void unlock_change()
	{
		{
			std::scoped_lock const lock{m_mutex};
			m_changing = false;
		}
		m_wake.notify_all();
	}

  private:
	std::mutex m_mutex;
	std::condition_variable m_wake;
	int m_framewaiting{0};
	bool m_frame{false};
	bool m_changing{false};
};

constexpr std::size_t stripe_count{64};

struct sync_state
{
	std::atomic<bool> parallel{false};
	walk_lock walk;
	// the vehicles exclusive, the scenery models shared with a stripe each
	std::shared_mutex vehicles;
	std::array<std::mutex, stripe_count> models;
	std::atomic<bool> reported{false};
	// the scenery loaded in the background (begin_background())
	std::atomic<bool> background{false};
	background_lock loading;
};

sync_state &state()
{
	static sync_state instance;
	return instance;
}

// what the thread holds
thread_local bool t_shared{false};
thread_local bool t_borrowed{false}; // the hold of another thread covers this one (borrowed_scope)
thread_local int t_exclusive{0};
thread_local int t_pattern{0}; // shared states of models held
thread_local bool t_vehicles{false};
thread_local std::uint64_t t_stripes{0}; // the stripes held, a bit each
thread_local bool t_frame{false}; // the main thread makes its frame while the scenery is loaded in the background
thread_local int t_background{0}; // changes made by the thread while the scenery is loaded in the background

std::size_t stripe_of(void const *Pointer)
{
	auto const value{static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(Pointer))};
	return static_cast<std::size_t>(((value >> 4) * 0x9E3779B97F4A7C15ull) >> 58) % stripe_count;
}

// a change made while the scenery is loaded in the background waits for the frame of the main thread; true when it was waited for
bool lock_background_change()
{
	if ((false == state().background.load(std::memory_order_acquire)) || t_frame || (t_background > 0))
	{
		return false;
	}
	state().loading.lock_change();
	++t_background;
	return true;
}

void report(char const *What)
{
	if (false == state().reported.exchange(true))
	{
		ErrorLog(std::string("parallel walks of the scene: ") + What + " (reported once)");
	}
}

} // namespace

bool parallel()
{
	return state().parallel.load(std::memory_order_acquire);
}

void open()
{
	auto &sync{state()};
	if (sync.parallel.load(std::memory_order_relaxed))
	{
		return;
	}
	sync.parallel.store(true, std::memory_order_release);
	sync.walk.lock_shared();
	t_shared = true;
}

void close()
{
	auto &sync{state()};
	if (t_shared)
	{
		sync.walk.unlock_shared();
		t_shared = false;
	}
	sync.parallel.store(false, std::memory_order_release);
}

void leave()
{
	if (t_shared)
	{
		state().walk.unlock_shared();
		t_shared = false;
	}
}

walk_scope::walk_scope()
{
	if (parallel() && (false == t_shared) && (false == t_borrowed))
	{
		state().walk.lock_shared();
		t_shared = true;
		m_held = true;
	}
}

walk_scope::~walk_scope()
{
	if (m_held)
	{
		state().walk.unlock_shared();
		t_shared = false;
	}
}

change::change(bool const Wanted)
{
	if (false == Wanted)
	{
		return;
	}
	m_background = lock_background_change();
	if ((false == parallel()) || (t_exclusive > 0))
	{
		return;
	}
	if ((t_pattern > 0) || t_borrowed)
	{
		// the walks waiting for the model held would wait for the change, which waits for them; the walk a helper works for waits for
		// it: not taken
		report(t_borrowed ? "a change made by a helper of a walk" : "a change made while the shared state of a model was held");
		return;
	}
	auto &sync{state()};
	if (t_shared)
	{
		sync.walk.unlock_shared();
		t_shared = false;
		m_shared = true;
	}
	sync.walk.lock();
	++t_exclusive;
	m_exclusive = true;
}

change::~change()
{
	auto &sync{state()};
	if (m_exclusive)
	{
		--t_exclusive;
		sync.walk.unlock();
		if (m_shared)
		{
			sync.walk.lock_shared();
			t_shared = true;
		}
	}
	if (m_background)
	{
		--t_background;
		sync.loading.unlock_change();
	}
}

void begin_background()
{
	state().background.store(true, std::memory_order_release);
}

void end_background()
{
	state().background.store(false, std::memory_order_release);
}

bool background()
{
	return state().background.load(std::memory_order_acquire);
}

background_pause::background_pause()
{
	if (t_background == 0)
	{
		return;
	}
	m_changes = t_background;
	// a change made meanwhile by the thread waits for the frame as any other
	t_background = 0;
	state().loading.unlock_change();
}

background_pause::~background_pause()
{
	if (m_changes == 0)
	{
		return;
	}
	state().loading.lock_change();
	t_background = m_changes;
}

frame_hold::frame_hold()
{
	if ((false == background()) || t_frame)
	{
		return;
	}
	state().loading.lock_frame();
	t_frame = true;
	m_held = true;
}

frame_hold::~frame_hold()
{
	if (false == m_held)
	{
		return;
	}
	t_frame = false;
	state().loading.unlock_frame();
}

borrowed_scope::borrowed_scope()
{
	t_borrowed = true;
}

borrowed_scope::~borrowed_scope()
{
	t_borrowed = false;
}

void yield()
{
	if ((false == t_shared) || (t_exclusive > 0) || (t_pattern > 0) || t_borrowed)
	{
		return;
	}
	auto &sync{state()};
	if (sync.walk.waiting())
	{
		sync.walk.unlock_shared();
		sync.walk.lock_shared();
	}
}

model_hold::model_hold(TModel3d const *Model)
{
	if ((Model == nullptr) || t_vehicles || (false == parallel()))
	{
		return;
	}
	m_stripe = stripe_of(Model);
	auto const bit{std::uint64_t{1} << m_stripe};
	if ((t_stripes & bit) != 0)
	{
		// held already by this thread (a model of the same stripe drawn inside the one held)
		return;
	}
	auto &sync{state()};
	sync.vehicles.lock_shared();
	sync.models[m_stripe].lock();
	t_stripes |= bit;
	++t_pattern;
	m_held = true;
}

model_hold::~model_hold()
{
	if (false == m_held)
	{
		return;
	}
	auto &sync{state()};
	--t_pattern;
	t_stripes &= ~(std::uint64_t{1} << m_stripe);
	sync.models[m_stripe].unlock();
	sync.vehicles.unlock_shared();
}

vehicle_hold::vehicle_hold()
{
	if (t_vehicles || (false == parallel()))
	{
		return;
	}
	if (t_pattern > 0)
	{
		// a scenery model held, which the vehicles would wait for
		report("a vehicle drawn while the shared state of a model was held");
		return;
	}
	state().vehicles.lock();
	t_vehicles = true;
	++t_pattern;
	m_held = true;
}

vehicle_hold::~vehicle_hold()
{
	if (false == m_held)
	{
		return;
	}
	--t_pattern;
	t_vehicles = false;
	state().vehicles.unlock();
}

} // namespace walk_sync
} // namespace gfx
