/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "utilities/memorystats.h"

#include <atomic>
#include <cstdlib>
#include <new>

#if defined(_WIN32)
#include <malloc.h>
#include <psapi.h>
#pragma comment(lib, "psapi.lib")
#elif defined(__APPLE__)
#include <mach/mach.h>
#include <malloc/malloc.h>
#else
#include <fstream>
#include <malloc.h>
#include <string>
#endif

// the global new and delete of the executable count the memory of the blocks they hand out (query_heap_memory()). the blocks come
// from malloc() as with the default operators; the aligned forms are left to the runtime
namespace
{

std::atomic<std::uint64_t> g_heapbytes{0};
std::atomic<std::uint64_t> g_heapblocks{0};

std::size_t block_size(void *Block)
{
#if defined(_WIN32)
	return ::_msize(Block);
#elif defined(__APPLE__)
	return ::malloc_size(Block);
#else
	return ::malloc_usable_size(Block);
#endif
}

void *counted_new(std::size_t Size)
{
	if (Size == 0)
	{
		Size = 1;
	}
	while (true)
	{
		if (auto *block{std::malloc(Size)})
		{
			g_heapbytes.fetch_add(block_size(block), std::memory_order_relaxed);
			g_heapblocks.fetch_add(1, std::memory_order_relaxed);
			return block;
		}
		auto const handler{std::get_new_handler()};
		if (handler == nullptr)
		{
			throw std::bad_alloc();
		}
		handler();
	}
}

void counted_delete(void *Block) noexcept
{
	if (Block == nullptr)
	{
		return;
	}
	g_heapbytes.fetch_sub(block_size(Block), std::memory_order_relaxed);
	g_heapblocks.fetch_sub(1, std::memory_order_relaxed);
	std::free(Block);
}

} // namespace

void *operator new(std::size_t Size)
{
	return counted_new(Size);
}

void *operator new[](std::size_t Size)
{
	return counted_new(Size);
}

void *operator new(std::size_t Size, std::nothrow_t const &) noexcept
{
	try
	{
		return counted_new(Size);
	}
	catch (...)
	{
		return nullptr;
	}
}

void *operator new[](std::size_t Size, std::nothrow_t const &) noexcept
{
	try
	{
		return counted_new(Size);
	}
	catch (...)
	{
		return nullptr;
	}
}

void operator delete(void *Block) noexcept
{
	counted_delete(Block);
}

void operator delete[](void *Block) noexcept
{
	counted_delete(Block);
}

void operator delete(void *Block, std::size_t) noexcept
{
	counted_delete(Block);
}

void operator delete[](void *Block, std::size_t) noexcept
{
	counted_delete(Block);
}

void operator delete(void *Block, std::nothrow_t const &) noexcept
{
	counted_delete(Block);
}

void operator delete[](void *Block, std::nothrow_t const &) noexcept
{
	counted_delete(Block);
}

heap_memory query_heap_memory()
{
	return {g_heapbytes.load(std::memory_order_relaxed), g_heapblocks.load(std::memory_order_relaxed)};
}

process_memory query_process_memory()
{
	process_memory result;
#if defined(_WIN32)
	PROCESS_MEMORY_COUNTERS_EX counters{};
	counters.cb = sizeof(counters);
	if (::GetProcessMemoryInfo(::GetCurrentProcess(), reinterpret_cast<PROCESS_MEMORY_COUNTERS *>(&counters), sizeof(counters)))
	{
		result.valid = true;
		result.resident = counters.WorkingSetSize;
		result.peak = counters.PeakWorkingSetSize;
		result.committed = counters.PrivateUsage;
	}
#elif defined(__APPLE__)
	mach_task_basic_info info{};
	mach_msg_type_number_t count{MACH_TASK_BASIC_INFO_COUNT};
	if (::task_info(::mach_task_self(), MACH_TASK_BASIC_INFO, reinterpret_cast<task_info_t>(&info), &count) == KERN_SUCCESS)
	{
		result.valid = true;
		result.resident = info.resident_size;
		result.peak = info.resident_size_max;
	}
#else
	// the lines of /proc/self/status give kB
	std::ifstream status("/proc/self/status");
	std::string line;
	auto const value = [&line](char const *Key, std::uint64_t &Target) {
		auto const length{std::char_traits<char>::length(Key)};
		if (line.compare(0, length, Key) != 0)
		{
			return false;
		}
		Target = std::stoull(line.substr(length)) * 1024;
		return true;
	};
	while (std::getline(status, line))
	{
		if (value("VmRSS:", result.resident))
		{
			result.valid = true;
		}
		value("VmHWM:", result.peak);
		// resident private memory: the closest this has to private bytes
		value("RssAnon:", result.committed);
	}
#endif
	return result;
}
