/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

// memory of the process, as the system counts it, for the debug panel
struct process_memory
{
	bool valid{false};
	std::uint64_t resident{0}; // in physical memory now (working set, resident set)
	std::uint64_t peak{0}; // the most it was in physical memory
	std::uint64_t committed{0}; // private memory the process asked for, in physical memory or not (private bytes; 0 where not known)
};

process_memory query_process_memory();

// what the program took with new and hasn't given back with delete (the global operators of the executable count it): bytes of
// the blocks, as the c runtime made them, and blocks. what libraries, the c runtime and the drivers take otherwise isn't in it
struct heap_memory
{
	std::uint64_t bytes{0};
	std::uint64_t blocks{0};
	// the same by the size of the blocks: class N holds the blocks of 2^(N-1) to 2^N - 1 bytes
	static constexpr std::size_t CLASSES{48};
	std::array<std::uint64_t, CLASSES> classbytes{};
	std::array<std::uint64_t, CLASSES> classblocks{};
};

heap_memory query_heap_memory();

// the counting of new and delete is off until this turns it on (debug.memorystats in the ini): it costs time with every block the
// program takes, which shows most in the loading of a scenery. once on, it stays on; the blocks taken before aren't counted
void start_heap_counting();
bool heap_counting();

// where the large blocks taken with new come from: the blocks of LARGE_BLOCK bytes or more, live now, summed up by the place in the
// program which took them (the first function of the call stack outside the standard library, and the one which called it)
struct heap_site
{
	std::string place;
	std::uint64_t bytes{0};
	std::uint64_t blocks{0};
};

struct large_blocks
{
	static constexpr std::uint64_t LARGE_BLOCK{256 * 1024};
	std::vector<heap_site> sites; // the most memory first
	std::uint64_t untracked{0}; // blocks taken while the table of them was full, since the counting began
};

// goes through the call stacks of the blocks, which takes a while the first time (the symbols of the program get loaded). not for
// more threads at once
large_blocks query_large_blocks();

// what a part of the program keeps in memory: things and bytes
struct memory_count
{
	std::uint64_t count{0};
	std::uint64_t bytes{0};

	memory_count &operator+=(memory_count const &Other)
	{
		count += Other.count;
		bytes += Other.bytes;
		return *this;
	}
};

// bytes a vector keeps for its elements
template <typename Type_> std::uint64_t memory_of(std::vector<Type_> const &Vector)
{
	return static_cast<std::uint64_t>(Vector.capacity()) * sizeof(Type_);
}
