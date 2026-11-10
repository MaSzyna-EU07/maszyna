#pragma once

// the crash report of the simulator as eu07_starter receives it: the starter shares this block with the simulator it starts
// and writes the minidump of the crashed simulator from outside of it, see utilities/crashdump.cpp and starter/eu07_starter.cpp

#ifdef _WIN32

#include <cstddef>
#include <cstdint>

namespace crashdump
{

constexpr std::uint32_t channel_magic{0x37305545}; // "EU07"
constexpr std::uint32_t channel_version{1};
// the environment variable with the handles of the block and its two events the starter passes to the simulator, "mapping:request:done" in hex
constexpr wchar_t const *channel_variable{L"EU07_CRASH_CHANNEL"};
constexpr std::size_t path_capacity{1024};
constexpr std::size_t comment_capacity{64 * 1024};

enum dump_state : std::uint32_t
{
	dump_pending = 0,
	dump_written = 1,
	dump_failed = 2,
};

struct channel_data
{
	std::uint32_t magic;
	std::uint32_t version;
	std::uint32_t process_id;
	std::uint32_t thread_id;
	// the EXCEPTION_POINTERS of the crash, an address in the crashed process
	std::uint64_t exception_pointers;
	// the module whose data sections a small dump keeps, the simulator itself
	std::uint64_t main_module_base;
	std::uint32_t full_memory;
	std::uint32_t comment_length;
	std::uint32_t state;
	std::uint32_t error;
	wchar_t dump_path[path_capacity];
	// the description of the crash, stored in the dump as its comment stream
	char comment[comment_capacity];
};

} // namespace crashdump

// writes the minidump of the process described by the block to its dump_path and sets its state and error.
// ClientPointers: the exception pointers are an address in the dumped process, not in the calling one
bool crashdump_write_minidump(void *Process, crashdump::channel_data &Data, bool ClientPointers);

#endif
