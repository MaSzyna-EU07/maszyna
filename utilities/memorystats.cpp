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
#include <bit>
#include <cstdlib>
#include <new>

#include <mutex>
#include <sstream>
#include <thread>
#include <unordered_map>

#if defined(_WIN32)
#include <malloc.h>
#include <psapi.h>
#include <dbghelp.h>
#pragma comment(lib, "psapi.lib")
#pragma comment(lib, "dbghelp.lib")
#else
#include <cxxabi.h>
#include <dlfcn.h>
#include <execinfo.h>
#if defined(__APPLE__)
#include <mach/mach.h>
#include <malloc/malloc.h>
#else
#include <fstream>
#include <malloc.h>
#endif
#endif

// the global new and delete of the executable count the memory of the blocks they hand out (query_heap_memory()), once
// start_heap_counting() turns it on. the blocks come from malloc() as with the default operators; the aligned forms are left to the
// runtime
namespace
{

std::atomic<bool> g_counting{false};
// signed: the blocks taken before the counting began are given back with it on
std::atomic<std::int64_t> g_heapbytes{0};
std::atomic<std::int64_t> g_heapblocks{0};
// by the size of the blocks, see heap_memory
std::array<std::atomic<std::int64_t>, heap_memory::CLASSES> g_classbytes{};
std::array<std::atomic<std::int64_t>, heap_memory::CLASSES> g_classblocks{};

// the large blocks live now, with the call stacks which took them. a fixed table: new can't take memory for it
constexpr std::size_t LARGE_FRAMES{16};
constexpr std::size_t LARGE_CAPACITY{16384};

struct large_record
{
	void *block{nullptr};
	std::uint64_t size{0};
	std::array<void *, LARGE_FRAMES> frames{};
	std::uint32_t framecount{0};
};

std::array<large_record, LARGE_CAPACITY> g_large; // [0, g_largeused) holds the records, with gaps of the blocks given back
std::size_t g_largeused{0};
std::uint64_t g_largeuntracked{0};
std::uint32_t g_largeskip{0}; // the frames of new itself at the top of the call stacks, see calibrate_frames()
std::atomic_flag g_largelock = ATOMIC_FLAG_INIT;

struct large_lock
{
	large_lock()
	{
		while (g_largelock.test_and_set(std::memory_order_acquire))
		{
			std::this_thread::yield();
		}
	}
	~large_lock()
	{
		g_largelock.clear(std::memory_order_release);
	}
};

std::size_t size_class(std::size_t const Size)
{
	return std::min<std::size_t>(std::bit_width(Size), heap_memory::CLASSES - 1);
}

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

std::uint32_t capture_frames(std::array<void *, LARGE_FRAMES> &Frames)
{
#if defined(_WIN32)
	return ::RtlCaptureStackBackTrace(0, static_cast<DWORD>(Frames.size()), Frames.data(), nullptr);
#else
	return static_cast<std::uint32_t>(std::max(0, ::backtrace(Frames.data(), static_cast<int>(Frames.size()))));
#endif
}

void note_large(void *Block, std::size_t const Size)
{
	large_record record;
	record.block = Block;
	record.size = Size;
	// taken outside the lock: it may take memory with malloc the first time
	record.framecount = capture_frames(record.frames);

	large_lock lock;
	std::size_t slot{0};
	while ((slot < g_largeused) && (g_large[slot].block != nullptr))
	{
		++slot;
	}
	if (slot == g_largeused)
	{
		if (g_largeused == g_large.size())
		{
			++g_largeuntracked;
			return;
		}
		++g_largeused;
	}
	g_large[slot] = record;
}

void forget_large(void *Block)
{
	large_lock lock;
	for (std::size_t slot = 0; slot < g_largeused; ++slot)
	{
		if (g_large[slot].block != Block)
		{
			continue;
		}
		g_large[slot].block = nullptr;
		while ((g_largeused > 0) && (g_large[g_largeused - 1].block == nullptr))
		{
			--g_largeused;
		}
		return;
	}
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
			if (g_counting.load(std::memory_order_relaxed))
			{
				auto const size{block_size(block)};
				g_heapbytes.fetch_add(static_cast<std::int64_t>(size), std::memory_order_relaxed);
				g_heapblocks.fetch_add(1, std::memory_order_relaxed);
				g_classbytes[size_class(size)].fetch_add(static_cast<std::int64_t>(size), std::memory_order_relaxed);
				g_classblocks[size_class(size)].fetch_add(1, std::memory_order_relaxed);
				if (size >= large_blocks::LARGE_BLOCK)
				{
					note_large(block, size);
				}
			}
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
	if (g_counting.load(std::memory_order_relaxed))
	{
		auto const size{block_size(Block)};
		g_heapbytes.fetch_sub(static_cast<std::int64_t>(size), std::memory_order_relaxed);
		g_heapblocks.fetch_sub(1, std::memory_order_relaxed);
		g_classbytes[size_class(size)].fetch_sub(static_cast<std::int64_t>(size), std::memory_order_relaxed);
		g_classblocks[size_class(size)].fetch_sub(1, std::memory_order_relaxed);
		if (size >= large_blocks::LARGE_BLOCK)
		{
			forget_large(Block);
		}
	}
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
	auto const value = [](std::atomic<std::int64_t> const &Counter) {
		return static_cast<std::uint64_t>(std::max<std::int64_t>(0, Counter.load(std::memory_order_relaxed)));
	};
	heap_memory result;
	result.bytes = value(g_heapbytes);
	result.blocks = value(g_heapblocks);
	for (std::size_t index = 0; index < heap_memory::CLASSES; ++index)
	{
		result.classbytes[index] = value(g_classbytes[index]);
		result.classblocks[index] = value(g_classblocks[index]);
	}
	return result;
}

namespace
{

// how many frames at the top of the stack of a block are of new itself: a block is taken here and its stack is compared with the one
// of this function. the names of the functions, which tell the same, aren't there in every build (linux without -rdynamic)
#if defined(_MSC_VER)
__declspec(noinline)
#else
__attribute__((noinline))
#endif
void calibrate_frames()
{
	std::array<void *, LARGE_FRAMES> own;
	auto const owncount{capture_frames(own)};
	auto *block{::operator new(large_blocks::LARGE_BLOCK)};
	large_record record;
	{
		large_lock lock;
		for (std::size_t slot = 0; slot < g_largeused; ++slot)
		{
			if (g_large[slot].block == block)
			{
				record = g_large[slot];
				break;
			}
		}
	}
	::operator delete(block);
	// the first frame of the block's stack which is in this one too (with the next one, to be sure) is the caller of this function:
	// the frame before it is this function, the ones before that are new
	for (std::uint32_t frame = 1; frame + 1 < record.framecount; ++frame)
	{
		for (std::uint32_t index = 0; index + 1 < owncount; ++index)
		{
			if ((record.frames[frame] == own[index]) && (record.frames[frame + 1] == own[index + 1]))
			{
				g_largeskip = frame - 1;
				return;
			}
		}
	}
}

} // namespace

void start_heap_counting()
{
	if (g_counting.load(std::memory_order_relaxed))
	{
		return;
	}
#if !defined(_WIN32)
	// the first backtrace() loads the library which walks the stack: done here, not in the middle of a new
	std::array<void *, LARGE_FRAMES> frames;
	capture_frames(frames);
#endif
	g_counting.store(true, std::memory_order_relaxed);
	calibrate_frames();
}

bool heap_counting()
{
	return g_counting.load(std::memory_order_relaxed);
}

namespace
{

// a frame of a call stack, as the symbols of the program name it
struct frame_name
{
	std::string function;
	std::string line; // file:line, where known and not in a header of the standard library
	bool allocator{false}; // a part of new or of the standard containers: the place which took the block is further down
};

frame_name name_frame(void *Address)
{
	frame_name result;
	// the address the call returns to: the call itself is a byte before it
	auto const address{reinterpret_cast<std::uintptr_t>(Address) - 1};
#if defined(_WIN32)
	auto const process{::GetCurrentProcess()};
	static bool const symbols{[process]() {
		::SymSetOptions(SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS | SYMOPT_LOAD_LINES);
		return ::SymInitialize(process, nullptr, TRUE) != FALSE;
	}()};
	if (symbols)
	{
		alignas(SYMBOL_INFO) char buffer[sizeof(SYMBOL_INFO) + 512];
		auto *symbol{reinterpret_cast<SYMBOL_INFO *>(buffer)};
		symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
		symbol->MaxNameLen = 511;
		DWORD64 displacement{0};
		if (::SymFromAddr(process, address, &displacement, symbol))
		{
			result.function = symbol->Name;
		}
		IMAGEHLP_LINE64 line{};
		line.SizeOfStruct = sizeof(line);
		DWORD linedisplacement{0};
		if (::SymGetLineFromAddr64(process, address, &linedisplacement, &line) && (line.FileName != nullptr))
		{
			std::string file{line.FileName};
			file = file.substr(file.find_last_of("\\/") + 1);
			// the headers of the standard library have no extension
			if (file.find('.') != std::string::npos)
			{
				result.line = file + ":" + std::to_string(line.LineNumber);
			}
		}
	}
	if (result.function.empty())
	{
		HMODULE module{nullptr};
		char path[MAX_PATH]{};
		if (::GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, reinterpret_cast<LPCSTR>(address), &module) &&
		    ::GetModuleFileNameA(module, path, MAX_PATH))
		{
			std::string file{path};
			std::ostringstream name;
			name << file.substr(file.find_last_of("\\/") + 1) << "+0x" << std::hex << (address - reinterpret_cast<std::uintptr_t>(module));
			result.function = name.str();
		}
	}
#else
	Dl_info info{};
	if (::dladdr(reinterpret_cast<void *>(address), &info) != 0)
	{
		if (info.dli_sname != nullptr)
		{
			int status{0};
			auto *demangled{abi::__cxa_demangle(info.dli_sname, nullptr, nullptr, &status)};
			result.function = (status == 0 && demangled != nullptr) ? demangled : info.dli_sname;
			std::free(demangled);
		}
		else if (info.dli_fname != nullptr)
		{
			// without the names of the functions of the executable (no -rdynamic): addr2line -f -C -e <file> <offset> tells them
			std::string file{info.dli_fname};
			std::ostringstream name;
			name << file.substr(file.find_last_of('/') + 1) << "+0x" << std::hex << (address - reinterpret_cast<std::uintptr_t>(info.dli_fbase));
			result.function = name.str();
		}
	}
#endif
	if (result.function.empty())
	{
		std::ostringstream name;
		name << "0x" << std::hex << address;
		result.function = name.str();
	}
	// the qualified name, without what the demangled names of gcc have around it: the type returned, the parameters
	auto function{result.function};
	{
		int depth{0};
		std::size_t start{0};
		for (std::size_t index = 0; index < function.size(); ++index)
		{
			auto const character{function[index]};
			if (character == '<')
			{
				++depth;
			}
			else if (character == '>')
			{
				--depth;
			}
			else if ((depth == 0) && (character == ' ') && ((index < 8) || (function.compare(index - 8, 8, "operator") != 0)))
			{
				start = index + 1;
			}
			else if ((depth == 0) && (character == '(') && (index > start))
			{
				function = function.substr(start, index - start);
				break;
			}
		}
	}
	result.allocator = (function.rfind("std::", 0) == 0) || (function.rfind("operator new", 0) == 0) || (function.rfind("__gnu_cxx::", 0) == 0) ||
	                   (function.find("counted_new") != std::string::npos) || (function.find("note_large") != std::string::npos) ||
	                   (function.find("capture_frames") != std::string::npos) || (function.find("RtlCaptureStackBackTrace") != std::string::npos) ||
	                   (function.find("backtrace") != std::string::npos) || (function.find("malloc") != std::string::npos);
	return result;
}

} // namespace

large_blocks query_large_blocks()
{
	large_blocks result;
	// a copy of the table, so the lock isn't held while the stacks get named. static: it's too large for the stack, and a block taken
	// for it would land in the table itself
	static std::array<large_record, LARGE_CAPACITY> records;
	std::size_t count{0};
	{
		large_lock lock;
		for (std::size_t slot = 0; slot < g_largeused; ++slot)
		{
			if (g_large[slot].block != nullptr)
			{
				records[count++] = g_large[slot];
			}
		}
		result.untracked = g_largeuntracked;
	}

	static std::mutex namesmutex;
	static std::unordered_map<void *, frame_name> names;
	std::scoped_lock namelock(namesmutex);
	auto const name = [](void *Address) -> frame_name const & {
		auto lookup{names.find(Address)};
		if (lookup == names.end())
		{
			lookup = names.emplace(Address, name_frame(Address)).first;
		}
		return lookup->second;
	};
	auto const place = [](frame_name const &Frame) { return Frame.line.empty() ? Frame.function : Frame.function + " (" + Frame.line + ")"; };

	std::unordered_map<std::string, std::size_t> sites;
	for (std::size_t index = 0; index < count; ++index)
	{
		auto const &record{records[index]};
		// the first frame outside new and the containers, and the one which called it
		std::string site;
		auto frame{g_largeskip};
		while ((frame < record.framecount) && name(record.frames[frame]).allocator)
		{
			++frame;
		}
		if (frame < record.framecount)
		{
			site = place(name(record.frames[frame]));
			if (frame + 1 < record.framecount)
			{
				site += " <- " + place(name(record.frames[frame + 1]));
			}
		}
		else
		{
			site = "?";
		}
		auto lookup{sites.find(site)};
		if (lookup == sites.end())
		{
			lookup = sites.emplace(site, result.sites.size()).first;
			result.sites.push_back({site});
		}
		auto &target{result.sites[lookup->second]};
		target.bytes += record.size;
		++target.blocks;
	}
	std::sort(result.sites.begin(), result.sites.end(), [](heap_site const &Left, heap_site const &Right) { return Left.bytes > Right.bytes; });
	return result;
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
