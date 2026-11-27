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

constexpr std::size_t stripe_count{64};

struct sync_state
{
	std::atomic<bool> parallel{false};
	walk_lock walk;
	// the vehicles exclusive, the scenery models shared with a stripe each
	std::shared_mutex vehicles;
	std::array<std::mutex, stripe_count> models;
	std::atomic<bool> reported{false};
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

std::size_t stripe_of(void const *Pointer)
{
	auto const value{static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(Pointer))};
	return static_cast<std::size_t>(((value >> 4) * 0x9E3779B97F4A7C15ull) >> 58) % stripe_count;
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
	if ((false == Wanted) || (false == parallel()) || (t_exclusive > 0))
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
	if (false == m_exclusive)
	{
		return;
	}
	auto &sync{state()};
	--t_exclusive;
	sync.walk.unlock();
	if (m_shared)
	{
		sync.walk.lock_shared();
		t_shared = true;
	}
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
