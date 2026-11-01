/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "utilities/crashdump.h"

#if defined(_WIN32) && defined(WITHDUMPGEN)

#include <DbgHelp.h>
#include <psapi.h>
#include <csignal>
#include <cstdarg>
#include <cstring>
#include <cwchar>

#include "utilities/Globals.h"
#include "utilities/cpu_features.h"
#include "utilities/crashdump_channel.h"
#include "version_info.h"

#ifdef _MSC_VER
#include <intrin.h>
#pragma comment(lib, "Dbghelp.lib")
#pragma comment(lib, "psapi.lib")
#endif

namespace
{

// the crashes reported without an exception of the system
constexpr DWORD code_abort{0xE0450001};
constexpr DWORD code_purecall{0xE0450002};
constexpr DWORD code_invalid_parameter{0xE0450003};
// the reports kept in crashdumps/, the older ones are removed at the start
constexpr std::size_t reports_kept{10};
// "crash_YYYY-MM-DD_HH-MM-SS", the start of the names of the files of a report
constexpr std::size_t report_stamp_length{25};

// the named values, written by the simulator as it goes and read without a lock when it crashes
struct info_entry
{
	char name[48];
	char value[464];
};
info_entry info_entries[48];
std::atomic<std::size_t> info_used{0};
std::mutex info_mutex;

// the recent lines of the log, written under the lock of the log
char log_ring[24 * 1024];
std::size_t log_written{0};
wchar_t log_paths[2][MAX_PATH * 2]{};

// taken at the installation, when it is still safe to
wchar_t report_directory[MAX_PATH * 2]{};
wchar_t executable_stem[MAX_PATH]{};
char executable_path[MAX_PATH * 3]{};
char processor_name[49]{};
char system_version[96]{};
SYSTEMTIME start_time{};
DWORD main_thread_id{0};

// the report. with eu07_starter the block is shared with it and the starter writes the minidump
crashdump::channel_data local_channel{};
crashdump::channel_data *channel{&local_channel};
HANDLE starter_request{nullptr};
HANDLE starter_done{nullptr};

// the report is written by a thread started beforehand: the crashed one may have no stack left, and a thread started
// after a crash may wait forever for the lock of the loader the crashed one holds
HANDLE report_request{nullptr};
HANDLE report_written{nullptr};
HANDLE report_done{nullptr};
HANDLE reporter_thread{nullptr};
DWORD reporter_thread_id{0};
volatile LONG crash_started{0};
EXCEPTION_POINTERS *crash_exception{nullptr};
DWORD crash_thread_id{0};

struct text_buffer
{
	char *data;
	std::size_t capacity;
	std::size_t length;
};

void append(text_buffer &Buffer, char const *Format, ...)
{
	if (Buffer.length + 1 >= Buffer.capacity)
	{
		return;
	}
	va_list arguments;
	va_start(arguments, Format);
	auto const written{std::vsnprintf(Buffer.data + Buffer.length, Buffer.capacity - Buffer.length, Format, arguments)};
	va_end(arguments);
	if (written > 0)
	{
		Buffer.length = std::min(Buffer.length + static_cast<std::size_t>(written), Buffer.capacity - 1);
	}
}

void append_wide(text_buffer &Buffer, wchar_t const *Text)
{
	if (Buffer.length + 1 >= Buffer.capacity)
	{
		return;
	}
	auto const written{::WideCharToMultiByte(CP_UTF8, 0, Text, -1, Buffer.data + Buffer.length, static_cast<int>(Buffer.capacity - Buffer.length), nullptr, nullptr)};
	if (written > 0)
	{
		// the count includes the terminating zero
		Buffer.length += static_cast<std::size_t>(written) - 1;
	}
	Buffer.data[Buffer.length] = '\0';
}

char const *exception_name(DWORD const Code)
{
	switch (Code)
	{
	case EXCEPTION_ACCESS_VIOLATION:
		return "access violation";
	case EXCEPTION_STACK_OVERFLOW:
		return "stack overflow";
	case EXCEPTION_ILLEGAL_INSTRUCTION:
		return "illegal instruction";
	case EXCEPTION_PRIV_INSTRUCTION:
		return "privileged instruction";
	case EXCEPTION_IN_PAGE_ERROR:
		return "in page error";
	case EXCEPTION_INT_DIVIDE_BY_ZERO:
		return "integer division by zero";
	case EXCEPTION_INT_OVERFLOW:
		return "integer overflow";
	case EXCEPTION_FLT_DIVIDE_BY_ZERO:
	case EXCEPTION_FLT_INVALID_OPERATION:
	case EXCEPTION_FLT_OVERFLOW:
	case EXCEPTION_FLT_UNDERFLOW:
	case EXCEPTION_FLT_INEXACT_RESULT:
	case EXCEPTION_FLT_DENORMAL_OPERAND:
	case EXCEPTION_FLT_STACK_CHECK:
		return "floating point exception";
	case EXCEPTION_DATATYPE_MISALIGNMENT:
		return "datatype misalignment";
	case EXCEPTION_ARRAY_BOUNDS_EXCEEDED:
		return "array bounds exceeded";
	case EXCEPTION_BREAKPOINT:
		return "breakpoint";
	case 0xC0000374:
		return "heap corruption";
	case 0xC0000409:
		return "stack buffer overrun";
	case 0xE06D7363:
		return "unhandled C++ exception";
	case code_abort:
		return "abort() called";
	case code_purecall:
		return "pure virtual function call";
	case code_invalid_parameter:
		return "invalid parameter passed to a C runtime function";
	default:
		return "unknown";
	}
}

#if defined(_MSC_VER) && defined(_WIN64)
// the type and the message of an unhandled C++ exception, read from the structures msvc throws it with (image relative on x64).
// guarded, the exception may come from anywhere
void describe_cpp_exception(EXCEPTION_RECORD const &Record, text_buffer &Buffer)
{
	if ((Record.ExceptionCode != 0xE06D7363) || (Record.NumberParameters < 4) || (Record.ExceptionInformation[0] != 0x19930520))
	{
		return;
	}
	__try
	{
		auto const object{Record.ExceptionInformation[1]};
		auto const throwinfo{Record.ExceptionInformation[2]};
		auto const imagebase{Record.ExceptionInformation[3]};
		if ((throwinfo == 0) || (imagebase == 0))
		{
			return;
		}
		// ThrowInfo: attributes, unwind, forward compat, catchable type array
		auto const *types{reinterpret_cast<int const *>(imagebase + reinterpret_cast<int const *>(throwinfo)[3])};
		auto const count{types[0]};
		for (int index = 0; (index < count) && (index < 32); ++index)
		{
			// CatchableType: properties, type descriptor, displacement (mdisp, pdisp, vdisp), size, copy function
			auto const *type{reinterpret_cast<int const *>(imagebase + types[1 + index])};
			// TypeDescriptor: vftable, spare, decorated name
			auto const *name{reinterpret_cast<char const *>(imagebase + type[1]) + 2 * sizeof(void *)};
			if (index == 0)
			{
				append(Buffer, "  C++ exception type: %.200s\n", name);
			}
			if ((std::strcmp(name, ".?AVexception@std@@") == 0) && (type[3] == -1))
			{
				auto const *exception{reinterpret_cast<std::exception const *>(object + type[2])};
				append(Buffer, "  C++ exception message: %.1000s\n", exception->what());
				break;
			}
		}
	}
	__except (EXCEPTION_EXECUTE_HANDLER)
	{
		append(Buffer, "  (the C++ exception could not be read)\n");
	}
}
#endif

// the module and the offset in it. through the memory map rather than the module list of the loader, whose lock the crashed thread may hold
void describe_address(void const *Address, text_buffer &Buffer)
{
	MEMORY_BASIC_INFORMATION region{};
	wchar_t path[MAX_PATH * 2];
	if ((::VirtualQuery(Address, &region, sizeof(region)) == 0) || (region.Type != MEM_IMAGE) || (::GetMappedFileNameW(::GetCurrentProcess(), region.AllocationBase, path, MAX_PATH * 2) == 0))
	{
		append(Buffer, "  at 0x%p, outside of the modules\n", Address);
		return;
	}
	path[MAX_PATH * 2 - 1] = L'\0';
	auto const *name{std::wcsrchr(path, L'\\')};
	append(Buffer, "  at 0x%p, ", Address);
	append_wide(Buffer, name != nullptr ? name + 1 : path);
	append(Buffer, "+0x%llx\n", static_cast<unsigned long long>(reinterpret_cast<ULONG_PTR>(Address) - reinterpret_cast<ULONG_PTR>(region.AllocationBase)));
}

void describe_crash(crashdump::channel_data &Data, SYSTEMTIME const &Now)
{
	text_buffer buffer{Data.comment, crashdump::comment_capacity, 0};
	Data.comment[0] = '\0';

	append(buffer, "MaSzyna crash report\n\n");
	append(buffer, "version: %s\n", Global.asVersion.c_str());
	append(buffer, "build: %s, committed at %s\n", GIT_HASH, SRC_DATE);
	append(buffer, "executable: %s\n", executable_path);
	append(buffer, "started: %04u-%02u-%02u %02u:%02u:%02u\n", start_time.wYear, start_time.wMonth, start_time.wDay, start_time.wHour, start_time.wMinute, start_time.wSecond);
	append(buffer, "crashed: %04u-%02u-%02u %02u:%02u:%02u\n", Now.wYear, Now.wMonth, Now.wDay, Now.wHour, Now.wMinute, Now.wSecond);
	append(buffer, "processor: %s\n", processor_name);
	append(buffer, "system: %s\n\n", system_version);

	if ((crash_exception != nullptr) && (crash_exception->ExceptionRecord != nullptr))
	{
		auto const &record{*crash_exception->ExceptionRecord};
		append(buffer, "exception: 0x%08lX, %s\n", static_cast<unsigned long>(record.ExceptionCode), exception_name(record.ExceptionCode));
		describe_address(record.ExceptionAddress, buffer);
		if (((record.ExceptionCode == EXCEPTION_ACCESS_VIOLATION) || (record.ExceptionCode == EXCEPTION_IN_PAGE_ERROR)) && (record.NumberParameters >= 2))
		{
			auto const operation{record.ExceptionInformation[0]};
			append(buffer, "  %s of address 0x%p\n", operation == 0 ? "read" : (operation == 8 ? "execution" : "write"), reinterpret_cast<void *>(record.ExceptionInformation[1]));
		}
#if defined(_MSC_VER) && defined(_WIN64)
		describe_cpp_exception(record, buffer);
#endif
	}
	append(buffer, "thread: %lu%s\n\n", static_cast<unsigned long>(crash_thread_id), crash_thread_id == main_thread_id ? " (main)" : "");

	PROCESS_MEMORY_COUNTERS_EX memory{};
	memory.cb = sizeof(memory);
	if (::GetProcessMemoryInfo(::GetCurrentProcess(), reinterpret_cast<PROCESS_MEMORY_COUNTERS *>(&memory), sizeof(memory)) != FALSE)
	{
		append(buffer, "process memory: %llu MB private, %llu MB working set, %llu MB peak working set\n", static_cast<unsigned long long>(memory.PrivateUsage >> 20),
		       static_cast<unsigned long long>(memory.WorkingSetSize >> 20), static_cast<unsigned long long>(memory.PeakWorkingSetSize >> 20));
	}
	MEMORYSTATUSEX system{};
	system.dwLength = sizeof(system);
	if (::GlobalMemoryStatusEx(&system) != FALSE)
	{
		append(buffer, "system memory: %llu MB of %llu MB free, %llu MB of %llu MB of the page file free\n", static_cast<unsigned long long>(system.ullAvailPhys >> 20),
		       static_cast<unsigned long long>(system.ullTotalPhys >> 20), static_cast<unsigned long long>(system.ullAvailPageFile >> 20),
		       static_cast<unsigned long long>(system.ullTotalPageFile >> 20));
	}
	append(buffer, "dump: %s\n\n", Data.full_memory != 0 ? "full memory" : "small");

	append(buffer, "info:\n");
	auto const infocount{std::min(info_used.load(), std::size(info_entries))};
	for (std::size_t index = 0; index < infocount; ++index)
	{
		append(buffer, "  %.47s: %.463s\n", info_entries[index].name, info_entries[index].value);
	}

	append(buffer, "\nrecent log:\n");
	auto const written{log_written};
	auto const capacity{std::size(log_ring)};
	if (written <= capacity)
	{
		append(buffer, "%.*s", static_cast<int>(written), log_ring);
	}
	else
	{
		// the ring went around: from the oldest whole line on
		auto const start{written % capacity};
		auto skip{start};
		while ((skip < capacity) && (log_ring[skip] != '\n'))
		{
			++skip;
		}
		if (skip < capacity)
		{
			append(buffer, "%.*s", static_cast<int>(capacity - skip - 1), log_ring + skip + 1);
		}
		append(buffer, "%.*s", static_cast<int>(start), log_ring);
	}

	Data.comment_length = static_cast<std::uint32_t>(buffer.length);
}

void write_file(wchar_t const *Path, char const *Text, std::size_t const Length)
{
	auto const file{::CreateFileW(Path, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr)};
	if (file == INVALID_HANDLE_VALUE)
	{
		return;
	}
	DWORD written{0};
	::WriteFile(file, Text, static_cast<DWORD>(Length), &written, nullptr);
	::CloseHandle(file);
}

void write_report()
{
	auto &data{*channel};
	SYSTEMTIME now;
	::GetLocalTime(&now);

	// crashdumps\crash_<date>_<time>_<executable> with .dmp, _info.txt, _log.txt and _errors.txt
	wchar_t name[MAX_PATH];
	std::swprintf(name, MAX_PATH, L"crash_%04u-%02u-%02u_%02u-%02u-%02u_%ls", now.wYear, now.wMonth, now.wDay, now.wHour, now.wMinute, now.wSecond, executable_stem);
	wchar_t base[MAX_PATH * 3];
	std::swprintf(base, MAX_PATH * 3, L"%ls\\%ls", report_directory, name);
	::CreateDirectoryW(report_directory, nullptr);

	data.process_id = ::GetCurrentProcessId();
	data.thread_id = crash_thread_id;
	data.exception_pointers = reinterpret_cast<ULONG_PTR>(crash_exception);
	data.main_module_base = reinterpret_cast<ULONG_PTR>(::GetModuleHandleW(nullptr));
	data.full_memory = Global.crashdump_full ? 1 : 0;
	data.state = crashdump::dump_pending;
	data.error = 0;
	std::swprintf(data.dump_path, crashdump::path_capacity, L"%ls.dmp", base);
	describe_crash(data, now);

	if (starter_request != nullptr)
	{
		::SetEvent(starter_request);
		auto const timeout{static_cast<DWORD>((data.full_memory != 0 ? 15 : 3) * 60 * 1000)};
		::WaitForSingleObject(starter_done, timeout);
	}
	bool const bystarter{data.state == crashdump::dump_written};
	if (data.state != crashdump::dump_written)
	{
		// without the starter, or it failed: from inside, which works for most of the crashes
		crashdump_write_minidump(::GetCurrentProcess(), data, false);
	}

	wchar_t path[MAX_PATH * 3];
	{
		char status[160];
		auto const statuslength{data.state == crashdump::dump_written ? std::snprintf(status, sizeof(status), "\ndump written by the %s\n", bystarter ? "starter" : "simulator")
		                                                                : std::snprintf(status, sizeof(status), "\ndump failed, error %lu\n", static_cast<unsigned long>(data.error))};
		std::swprintf(path, MAX_PATH * 3, L"%ls_info.txt", base);
		auto const file{::CreateFileW(path, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr)};
		if (file != INVALID_HANDLE_VALUE)
		{
			DWORD written{0};
			::WriteFile(file, data.comment, data.comment_length, &written, nullptr);
			if (statuslength > 0)
			{
				::WriteFile(file, status, static_cast<DWORD>(std::min<int>(statuslength, sizeof(status) - 1)), &written, nullptr);
			}
			::CloseHandle(file);
		}
	}
	// the log is written again from the start by the next run, the copies keep it with the report
	if (log_paths[0][0] != L'\0')
	{
		std::swprintf(path, MAX_PATH * 3, L"%ls_log.txt", base);
		::CopyFileW(log_paths[0], path, FALSE);
	}
	if (log_paths[1][0] != L'\0')
	{
		std::swprintf(path, MAX_PATH * 3, L"%ls_errors.txt", base);
		::CopyFileW(log_paths[1], path, FALSE);
	}
	::SetEvent(report_written);

	wchar_t message[MAX_PATH * 4 + 512];
	if (data.state == crashdump::dump_written)
	{
		std::swprintf(message, std::size(message),
		              L"The simulator has crashed.\n\nA crash report was saved in:\n%ls\n\nPlease send the files %ls.dmp, %ls_info.txt and %ls_log.txt to the developers.",
		              report_directory, name, name, name);
	}
	else
	{
		std::swprintf(message, std::size(message), L"The simulator has crashed.\n\nIts memory dump could not be written (error %lu), the description of the crash is in:\n%ls_info.txt",
		              static_cast<unsigned long>(data.error), base);
	}
	::MessageBoxW(nullptr, message, L"MaSzyna", MB_ICONERROR | MB_OK | MB_TOPMOST | MB_SETFOREGROUND);
	::SetEvent(report_done);
}

DWORD WINAPI reporter_main(LPVOID)
{
	::WaitForSingleObject(report_request, INFINITE);
	write_report();
	return 0;
}

LONG WINAPI crash_filter(EXCEPTION_POINTERS *Exception)
{
	if (::GetCurrentThreadId() == reporter_thread_id)
	{
		// the report itself crashed
		return EXCEPTION_EXECUTE_HANDLER;
	}
	if (::InterlockedCompareExchange(&crash_started, 1, 0) != 0)
	{
		// another thread crashed first, its report ends the process
		::WaitForSingleObject(report_done, INFINITE);
		return EXCEPTION_EXECUTE_HANDLER;
	}
	crash_exception = Exception;
	crash_thread_id = ::GetCurrentThreadId();
	if (reporter_thread != nullptr)
	{
		::SetEvent(report_request);
		auto const timeout{static_cast<DWORD>((Global.crashdump_full ? 16 : 4) * 60 * 1000)};
		if (::WaitForSingleObject(report_written, timeout) == WAIT_OBJECT_0)
		{
			// the message the user reads
			::WaitForSingleObject(report_done, INFINITE);
		}
	}
	else
	{
		write_report();
	}
	return EXCEPTION_EXECUTE_HANDLER;
}

// the fatal errors of the C runtime, reported as crashes with the stack where they happened
void report_and_terminate(DWORD const Code, void *Address)
{
	CONTEXT context{};
	::RtlCaptureContext(&context);
	EXCEPTION_RECORD record{};
	record.ExceptionCode = Code;
	record.ExceptionFlags = EXCEPTION_NONCONTINUABLE;
	record.ExceptionAddress = Address;
	EXCEPTION_POINTERS pointers{&record, &context};
	crash_filter(&pointers);
	::TerminateProcess(::GetCurrentProcess(), Code);
}

#ifdef _MSC_VER
#define EU07_RETURN_ADDRESS() _ReturnAddress()
#else
#define EU07_RETURN_ADDRESS() __builtin_return_address(0)
#endif

void on_abort(int)
{
	report_and_terminate(code_abort, EU07_RETURN_ADDRESS());
}

void __cdecl on_purecall()
{
	report_and_terminate(code_purecall, EU07_RETURN_ADDRESS());
}

void __cdecl on_invalid_parameter(wchar_t const *, wchar_t const *, wchar_t const *, unsigned int, uintptr_t)
{
	report_and_terminate(code_invalid_parameter, EU07_RETURN_ADDRESS());
}

// the starter passes the handles of the shared block and its events in the environment
void connect_to_starter()
{
	wchar_t value[128];
	auto const length{::GetEnvironmentVariableW(crashdump::channel_variable, value, static_cast<DWORD>(std::size(value)))};
	if ((length == 0) || (length >= std::size(value)))
	{
		return;
	}
	// the processes the simulator starts do not get it
	::SetEnvironmentVariableW(crashdump::channel_variable, nullptr);

	unsigned long long mapping{0}, request{0}, done{0};
	if (std::swscanf(value, L"%llx:%llx:%llx", &mapping, &request, &done) != 3)
	{
		return;
	}
	auto *view{static_cast<crashdump::channel_data *>(::MapViewOfFile(reinterpret_cast<HANDLE>(static_cast<ULONG_PTR>(mapping)), FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, sizeof(crashdump::channel_data)))};
	if (view == nullptr)
	{
		return;
	}
	if ((view->magic != crashdump::channel_magic) || (view->version != crashdump::channel_version))
	{
		::UnmapViewOfFile(view);
		return;
	}
	channel = view;
	starter_request = reinterpret_cast<HANDLE>(static_cast<ULONG_PTR>(request));
	starter_done = reinterpret_cast<HANDLE>(static_cast<ULONG_PTR>(done));
}

void read_system_version()
{
	using rtlgetversion = LONG(WINAPI *)(PRTL_OSVERSIONINFOW);
	auto *const ntdll{::GetModuleHandleW(L"ntdll.dll")};
	auto const function{ntdll != nullptr ? reinterpret_cast<rtlgetversion>(reinterpret_cast<void *>(::GetProcAddress(ntdll, "RtlGetVersion"))) : nullptr};
	RTL_OSVERSIONINFOW version{};
	version.dwOSVersionInfoSize = sizeof(version);
	if ((function != nullptr) && (function(&version) == 0))
	{
		std::snprintf(system_version, sizeof(system_version), "Windows %lu.%lu build %lu", version.dwMajorVersion, version.dwMinorVersion, version.dwBuildNumber);
	}
}

// keeps the newest reports, each the files starting with the same "crash_<date>_<time>"
void remove_old_reports()
{
	std::error_code error;
	std::filesystem::path const directory{report_directory};
	if (false == std::filesystem::is_directory(directory, error))
	{
		return;
	}
	std::set<std::wstring, std::greater<>> stamps;
	std::vector<std::filesystem::path> files;
	for (auto const &entry : std::filesystem::directory_iterator(directory, error))
	{
		auto const filename{entry.path().filename().wstring()};
		if ((filename.size() >= report_stamp_length) && (filename.compare(0, 6, L"crash_") == 0) && entry.is_regular_file(error))
		{
			stamps.insert(filename.substr(0, report_stamp_length));
			files.push_back(entry.path());
		}
	}
	if (stamps.size() <= reports_kept)
	{
		return;
	}
	auto const oldestkept{*std::next(stamps.begin(), reports_kept - 1)};
	for (auto const &file : files)
	{
		if (file.filename().wstring().substr(0, report_stamp_length) < oldestkept)
		{
			std::filesystem::remove(file, error);
		}
	}
}

} // namespace

void crashdump_install()
{
	main_thread_id = ::GetCurrentThreadId();
	::GetLocalTime(&start_time);

	wchar_t path[MAX_PATH * 2];
	auto length{::GetCurrentDirectoryW(MAX_PATH, path)};
	if ((length > 0) && (length < MAX_PATH))
	{
		std::swprintf(report_directory, std::size(report_directory), L"%ls\\crashdumps", path);
	}
	else
	{
		std::swprintf(report_directory, std::size(report_directory), L"crashdumps");
	}
	length = ::GetModuleFileNameW(nullptr, path, MAX_PATH * 2);
	path[std::min<DWORD>(length, MAX_PATH * 2 - 1)] = L'\0';
	::WideCharToMultiByte(CP_UTF8, 0, path, -1, executable_path, sizeof(executable_path), nullptr, nullptr);
	auto const *filename{std::wcsrchr(path, L'\\')};
	std::swprintf(executable_stem, std::size(executable_stem), L"%ls", filename != nullptr ? filename + 1 : path);
	if (auto *extension{std::wcsrchr(executable_stem, L'.')})
	{
		*extension = L'\0';
	}
	cpu_features::brand(processor_name);
	read_system_version();
	connect_to_starter();

	report_request = ::CreateEventW(nullptr, FALSE, FALSE, nullptr);
	report_written = ::CreateEventW(nullptr, TRUE, FALSE, nullptr);
	report_done = ::CreateEventW(nullptr, TRUE, FALSE, nullptr);
	if ((report_request != nullptr) && (report_written != nullptr) && (report_done != nullptr))
	{
		reporter_thread = ::CreateThread(nullptr, 256 * 1024, reporter_main, nullptr, STACK_SIZE_PARAM_IS_A_RESERVATION, &reporter_thread_id);
	}

	// a debugger gets the unhandled exceptions before the filter, and the fatal errors of the C runtime keep stopping in it as they did
	::SetUnhandledExceptionFilter(crash_filter);
	if (::IsDebuggerPresent() == FALSE)
	{
#ifdef _MSC_VER
		// abort() reports to the system directly by default, past the filter
		_set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#endif
		std::signal(SIGABRT, on_abort);
		_set_purecall_handler(on_purecall);
		_set_invalid_parameter_handler(on_invalid_parameter);
	}

	remove_old_reports();
}

void crashdump_set_info(char const *Name, std::string const &Value)
{
	if (Name == nullptr)
	{
		return;
	}
	std::scoped_lock lock(info_mutex);
	auto const used{info_used.load()};
	std::size_t index{0};
	while ((index < used) && (std::strncmp(info_entries[index].name, Name, sizeof(info_entries[index].name) - 1) != 0))
	{
		++index;
	}
	if (index >= std::size(info_entries))
	{
		return;
	}
	auto &entry{info_entries[index]};
	std::snprintf(entry.value, sizeof(entry.value), "%s", Value.c_str());
	if (index == used)
	{
		std::snprintf(entry.name, sizeof(entry.name), "%s", Name);
		info_used.store(used + 1);
	}
}

void crashdump_note_log(char const *Text, std::size_t const Length)
{
	auto const capacity{std::size(log_ring)};
	for (std::size_t index = 0; index <= Length; ++index)
	{
		log_ring[log_written % capacity] = index < Length ? Text[index] : '\n';
		++log_written;
	}
}

void crashdump_set_log_file(crashdump_log_file const File, std::string const &Path)
{
	wchar_t relative[MAX_PATH * 2];
	if (::MultiByteToWideChar(CP_ACP, 0, Path.c_str(), -1, relative, MAX_PATH * 2) == 0)
	{
		return;
	}
	auto &target{log_paths[File == crashdump_log_file::log ? 0 : 1]};
	auto const length{::GetFullPathNameW(relative, MAX_PATH * 2, target, nullptr)};
	if ((length == 0) || (length >= MAX_PATH * 2))
	{
		target[0] = L'\0';
	}
}

#else

void crashdump_install() {}

void crashdump_set_info(char const *, std::string const &) {}

void crashdump_note_log(char const *, std::size_t) {}

void crashdump_set_log_file(crashdump_log_file, std::string const &) {}

#endif
