/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
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
