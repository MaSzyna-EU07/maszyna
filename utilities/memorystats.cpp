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

#if defined(_WIN32)
#include <psapi.h>
#pragma comment(lib, "psapi.lib")
#elif defined(__APPLE__)
#include <mach/mach.h>
#else
#include <fstream>
#include <string>
#endif

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
