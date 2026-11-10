/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// eu07_starter: starts the simulator next to it, after it checks what the system would otherwise report with a bare error code
// or not at all: the processor (AVX2), the libraries the simulator links (the Visual C++ runtime above all) and their versions.
// it explains the failures of the start, and writes the minidump of a crashed simulator from outside of it (utilities/crashdump.cpp).
// built with the static C runtime and without AVX2, so it runs where the simulator cannot

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <shellapi.h>

#include <algorithm>
#include <cstring>
#include <cwchar>
#include <cwctype>
#include <string>
#include <vector>

#include "utilities/cpu_features.h"
#include "utilities/crashdump_channel.h"

#ifndef EU07_SIMULATOR_EXE
#error "EU07_SIMULATOR_EXE, the file name of the simulator, is set by CMakeLists.txt"
#endif

namespace
{

wchar_t const *const title{L"MaSzyna"};
wchar_t const *const redistributable_url{L"https://aka.ms/vs/17/release/vc_redist.x64.exe"};

std::wstring widen(char const *Text)
{
	auto const length{::MultiByteToWideChar(CP_UTF8, 0, Text, -1, nullptr, 0)};
	std::wstring result(length > 0 ? length - 1 : 0, L'\0');
	if (length > 1)
	{
		::MultiByteToWideChar(CP_UTF8, 0, Text, -1, result.data(), length);
	}
	return result;
}

std::wstring lowercase(std::wstring Text)
{
	std::transform(Text.begin(), Text.end(), Text.begin(), [](wchar_t const Character) { return static_cast<wchar_t>(std::towlower(Character)); });
	return Text;
}

bool starts_with(std::wstring const &Text, wchar_t const *Prefix)
{
	return Text.compare(0, std::wcslen(Prefix), Prefix) == 0;
}

bool file_exists(std::wstring const &Path)
{
	auto const attributes{::GetFileAttributesW(Path.c_str())};
	return (attributes != INVALID_FILE_ATTRIBUTES) && ((attributes & FILE_ATTRIBUTE_DIRECTORY) == 0);
}

std::wstring system_directory()
{
	wchar_t path[MAX_PATH];
	auto const length{::GetSystemDirectoryW(path, MAX_PATH)};
	return (length > 0) && (length < MAX_PATH) ? std::wstring(path, length) : std::wstring();
}

std::wstring windows_directory()
{
	wchar_t path[MAX_PATH];
	auto const length{::GetWindowsDirectoryW(path, MAX_PATH)};
	return (length > 0) && (length < MAX_PATH) ? std::wstring(path, length) : std::wstring();
}

int show(std::wstring const &Text, UINT const Type)
{
	return ::MessageBoxW(nullptr, Text.c_str(), title, Type | MB_SETFOREGROUND);
}

// asks to open the download page of the Visual C++ runtime, true when the user wants it
bool offer_redistributable(std::wstring const &Text, bool const CanContinue)
{
	auto const answer{show(Text + L"\n\nOpen the download page of the Microsoft Visual C++ Redistributable (x64) now?" + (CanContinue ? L"\n(No starts the simulator anyway.)" : L""),
	                       MB_YESNO | MB_ICONWARNING)};
	if (answer == IDYES)
	{
		::ShellExecuteW(nullptr, L"open", redistributable_url, nullptr, nullptr, SW_SHOWNORMAL);
		return true;
	}
	return false;
}

// the libraries an image links, from its import directory, read from the image mapped as the loader maps it, without running it
struct image_imports
{
	bool valid{false};
	bool same_machine{false};
	std::vector<std::wstring> libraries;
};

image_imports read_imports(std::wstring const &Path)
{
	image_imports result;
	auto const file{::CreateFileW(Path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_DELETE, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr)};
	if (file == INVALID_HANDLE_VALUE)
	{
		return result;
	}
	auto const mapping{::CreateFileMappingW(file, nullptr, PAGE_READONLY | SEC_IMAGE, 0, 0, nullptr)};
	::CloseHandle(file);
	if (mapping == nullptr)
	{
		return result;
	}
	auto const *view{static_cast<unsigned char const *>(::MapViewOfFile(mapping, FILE_MAP_READ, 0, 0, 0))};
	::CloseHandle(mapping);
	if (view == nullptr)
	{
		return result;
	}

	MEMORY_BASIC_INFORMATION region{};
	::VirtualQuery(view, &region, sizeof(region));
	auto const *dos{reinterpret_cast<IMAGE_DOS_HEADER const *>(view)};
	if (dos->e_magic == IMAGE_DOS_SIGNATURE)
	{
		auto const *nt{reinterpret_cast<IMAGE_NT_HEADERS const *>(view + dos->e_lfanew)};
		if (nt->Signature == IMAGE_NT_SIGNATURE)
		{
			result.valid = true;
#if defined(_M_ARM64)
			result.same_machine = (nt->FileHeader.Machine == IMAGE_FILE_MACHINE_ARM64);
#else
			result.same_machine = (nt->FileHeader.Machine == IMAGE_FILE_MACHINE_AMD64);
#endif
			if (result.same_machine && (nt->OptionalHeader.Magic == IMAGE_NT_OPTIONAL_HDR64_MAGIC))
			{
				auto const size{nt->OptionalHeader.SizeOfImage};
				auto const &directory{nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT]};
				if ((directory.VirtualAddress != 0) && (directory.VirtualAddress < size))
				{
					for (auto const *descriptor{reinterpret_cast<IMAGE_IMPORT_DESCRIPTOR const *>(view + directory.VirtualAddress)};
					     (reinterpret_cast<unsigned char const *>(descriptor + 1) <= view + size) && (descriptor->Name != 0); ++descriptor)
					{
						if (descriptor->Name >= size)
						{
							break;
						}
						auto const *name{reinterpret_cast<char const *>(view + descriptor->Name)};
						auto const length{strnlen(name, std::min<std::size_t>(size - descriptor->Name, MAX_PATH))};
						result.libraries.push_back(widen(std::string(name, length).c_str()));
					}
				}
			}
		}
	}
	::UnmapViewOfFile(view);
	return result;
}

struct library_check
{
	std::wstring application_directory;
	std::vector<std::wstring> visited;
	std::vector<std::wstring> missing;
	std::vector<std::wstring> foreign; // built for another processor, a 32 bit library mostly
	bool needs_universal_runtime{false};
};

bool is_runtime_library(std::wstring const &Name)
{
	return starts_with(Name, L"msvcp140") || starts_with(Name, L"vcruntime140") || starts_with(Name, L"concrt140") || (Name == L"ucrtbase.dll");
}

// where the loader finds a library of the simulator: next to it first, then in the system. KnownDLLs are system libraries anyway
std::wstring locate(library_check const &Check, std::wstring const &Name, bool &Local)
{
	Local = false;
	auto path{Check.application_directory + L"\\" + Name};
	if (file_exists(path))
	{
		Local = true;
		return path;
	}
	for (auto const &directory : {system_directory(), windows_directory()})
	{
		if (directory.empty())
		{
			continue;
		}
		path = directory + L"\\" + Name;
		if (file_exists(path))
		{
			return path;
		}
	}
	wchar_t found[MAX_PATH];
	auto const length{::SearchPathW(nullptr, Name.c_str(), nullptr, MAX_PATH, found, nullptr)};
	return (length > 0) && (length < MAX_PATH) ? std::wstring(found, length) : std::wstring();
}

// the libraries of the image and of the libraries next to the simulator, which the installation brings. the system ones are taken as they are
void check_libraries(library_check &Check, std::wstring const &Image, int const Depth)
{
	auto const imports{read_imports(Image)};
	for (auto const &library : imports.libraries)
	{
		auto const name{lowercase(library)};
		if (std::find(Check.visited.begin(), Check.visited.end(), name) != Check.visited.end())
		{
			continue;
		}
		Check.visited.push_back(name);
		// the api sets the system resolves on its own; the C runtime ones lead to the universal C runtime
		if (starts_with(name, L"api-ms-") || starts_with(name, L"ext-ms-"))
		{
			if (starts_with(name, L"api-ms-win-crt-"))
			{
				Check.needs_universal_runtime = true;
			}
			continue;
		}
		bool local{false};
		auto const path{locate(Check, name, local)};
		if (path.empty())
		{
			Check.missing.push_back(library);
			continue;
		}
		if (local)
		{
			auto const nested{read_imports(path)};
			if (nested.valid && (false == nested.same_machine))
			{
				Check.foreign.push_back(library);
				continue;
			}
			if (Depth < 8)
			{
				check_libraries(Check, path, Depth + 1);
			}
		}
	}
}

// the version of a library as major, minor and build, zeros when it has none
void file_version(std::wstring const &Path, unsigned int &Major, unsigned int &Minor, unsigned int &Build)
{
	Major = Minor = Build = 0;
	DWORD handle{0};
	auto const size{::GetFileVersionInfoSizeW(Path.c_str(), &handle)};
	if (size == 0)
	{
		return;
	}
	std::vector<unsigned char> data(size);
	if (::GetFileVersionInfoW(Path.c_str(), 0, size, data.data()) == FALSE)
	{
		return;
	}
	VS_FIXEDFILEINFO *info{nullptr};
	UINT length{0};
	if ((::VerQueryValueW(data.data(), L"\\", reinterpret_cast<void **>(&info), &length) == FALSE) || (info == nullptr))
	{
		return;
	}
	Major = HIWORD(info->dwFileVersionMS);
	Minor = LOWORD(info->dwFileVersionMS);
	Build = HIWORD(info->dwFileVersionLS);
}

std::wstring join(std::vector<std::wstring> const &Items)
{
	std::wstring result;
	for (auto const &item : Items)
	{
		result += L"\n    " + item;
	}
	return result;
}

// false when the simulator should not be started
bool check_system(std::wstring const &Simulator, std::wstring const &Directory)
{
#ifdef EU07_REQUIRES_AVX2
	auto const missingfeatures{cpu_features::missing(cpu_features::required_for_avx2)};
	if (missingfeatures != 0)
	{
		std::wstring features;
		for (unsigned int feature = 1; feature <= cpu_features::os_avx; feature <<= 1)
		{
			if ((missingfeatures & feature) != 0)
			{
				features += (features.empty() ? L"" : L", ") + widen(cpu_features::name(feature));
			}
		}
		char brand[49];
		cpu_features::brand(brand);
		show(L"This processor does not support the AVX2 instruction set the simulator is built for. The simulator won't run.\n\nProcessor: " + widen(brand) + L"\nMissing: " + features +
		         L"\n\nIt needs a processor with AVX2: Intel Core of the 4th generation (Haswell, 2013), AMD of the Excavator generation (2015) or newer.",
		     MB_OK | MB_ICONERROR);
		return false;
	}
#endif

	auto const imports{read_imports(Simulator)};
	if (imports.valid && (false == imports.same_machine))
	{
		show(L"The simulator " + Simulator + L" is built for another processor than this starter.", MB_OK | MB_ICONERROR);
		return false;
	}

	library_check check;
	check.application_directory = Directory;
	check_libraries(check, Simulator, 0);
	if (check.needs_universal_runtime)
	{
		bool local{false};
		if (locate(check, L"ucrtbase.dll", local).empty())
		{
			check.missing.push_back(L"ucrtbase.dll (Universal C Runtime, update KB2999226 on Windows 7 and 8.1)");
		}
	}

	if (false == check.foreign.empty())
	{
		show(L"These libraries next to the simulator are built for another processor (32 bit?):" + join(check.foreign) +
		         L"\n\nReplace them with the 64 bit ones of the installation: on Steam, verify the integrity of the game files.",
		     MB_OK | MB_ICONERROR);
		return false;
	}
	if (false == check.missing.empty())
	{
		bool runtime{false};
		for (auto const &library : check.missing)
		{
			runtime = runtime || is_runtime_library(lowercase(library));
		}
		std::wstring const text{L"The simulator can't start, these libraries are missing:" + join(check.missing)};
		if (runtime)
		{
			offer_redistributable(text + L"\n\nThey belong to the Microsoft Visual C++ Redistributable.", false);
		}
		else
		{
			show(text + L"\n\nThe installation is incomplete: on Steam, verify the integrity of the game files, otherwise install the simulator again.", MB_OK | MB_ICONERROR);
		}
		return false;
	}

#if defined(_MSC_VER) && (_MSC_VER >= 1940)
	// the programs of Visual Studio 2022 17.10 and newer crash in std::mutex with the runtime older than 14.40 it replaces
	bool local{false};
	auto const runtime{locate(check, L"msvcp140.dll", local)};
	unsigned int major, minor, build;
	file_version(runtime, major, minor, build);
	if ((major == 14) && (minor < 40))
	{
		if (offer_redistributable(L"The Microsoft Visual C++ runtime (" + runtime + L", version 14." + std::to_wstring(minor) + L"." + std::to_wstring(build) +
		                              L") is older than the simulator needs (14.40 or newer). The simulator will likely crash at start.",
		                          true))
		{
			return false;
		}
	}
#endif
	return true;
}

// the explanation of an exit code of a simulator which did not report a crash itself, empty for an ordinary exit
std::wstring explain_exit(DWORD const Code)
{
	switch (Code)
	{
	case 0xC0000135: // STATUS_DLL_NOT_FOUND
		return L"A library the simulator needs is missing (0xC0000135). Install the Microsoft Visual C++ Redistributable (x64); if it does not help, verify the integrity of the game files.";
	case 0xC0000139: // STATUS_ENTRYPOINT_NOT_FOUND
		return L"A library the simulator needs is too old or of another version (0xC0000139). Install the latest Microsoft Visual C++ Redistributable (x64), and verify the integrity of the game files.";
	case 0xC000007B: // STATUS_INVALID_IMAGE_FORMAT
		return L"A library of the simulator is damaged or built for another processor (0xC000007B). Verify the integrity of the game files.";
	case 0xC0000142: // STATUS_DLL_INIT_FAILED
		return L"A library of the simulator failed to start (0xC0000142).";
	case 0xC000001D: // STATUS_ILLEGAL_INSTRUCTION
		return L"The simulator used an instruction this processor does not have (0xC000001D). It needs a processor with AVX2.";
	case 0xC0000409: // STATUS_STACK_BUFFER_OVERRUN, also a fast fail of the runtime
		return L"The simulator was stopped by a fatal error check (0xC0000409), no crash report could be written.";
	case 0xC000013A: // STATUS_CONTROL_C_EXIT
		return {};
	default:
		break;
	}
	if ((Code & 0xF0000000) == 0xC0000000)
	{
		wchar_t text[160];
		std::swprintf(text, 160, L"The simulator ended with the error 0x%08lX before its crash report could be written.", static_cast<unsigned long>(Code));
		return text;
	}
	return {};
}

// the arguments of the starter, passed on to the simulator
wchar_t const *arguments()
{
	auto const *line{::GetCommandLineW()};
	if (*line == L'"')
	{
		++line;
		while ((*line != L'\0') && (*line != L'"'))
		{
			++line;
		}
		if (*line == L'"')
		{
			++line;
		}
	}
	else
	{
		while ((*line != L'\0') && (*line != L' ') && (*line != L'\t'))
		{
			++line;
		}
	}
	while ((*line == L' ') || (*line == L'\t'))
	{
		++line;
	}
	return line;
}

} // namespace

int WINAPI wWinMain(HINSTANCE, HINSTANCE, PWSTR, int)
{
	wchar_t path[MAX_PATH * 2];
	auto const length{::GetModuleFileNameW(nullptr, path, MAX_PATH * 2)};
	if ((length == 0) || (length >= MAX_PATH * 2))
	{
		return 1;
	}
	std::wstring directory(path, length);
	directory.erase(directory.find_last_of(L"\\/"));
	auto const simulator{directory + L"\\" + widen(EU07_SIMULATOR_EXE)};

	if (false == file_exists(simulator))
	{
		show(L"The simulator was not found:\n" + simulator, MB_OK | MB_ICONERROR);
		return 1;
	}
	if (false == check_system(simulator, directory))
	{
		return 1;
	}

	// the shared block of the crash report, its handles are inherited by the simulator
	SECURITY_ATTRIBUTES inheritable{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
	auto const mapping{::CreateFileMappingW(INVALID_HANDLE_VALUE, &inheritable, PAGE_READWRITE, 0, sizeof(crashdump::channel_data), nullptr)};
	auto *channel{mapping != nullptr ? static_cast<crashdump::channel_data *>(::MapViewOfFile(mapping, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, sizeof(crashdump::channel_data))) : nullptr};
	auto const request{::CreateEventW(&inheritable, FALSE, FALSE, nullptr)};
	auto const done{::CreateEventW(&inheritable, FALSE, FALSE, nullptr)};
	bool const reporting{(channel != nullptr) && (request != nullptr) && (done != nullptr)};
	if (reporting)
	{
		channel->magic = crashdump::channel_magic;
		channel->version = crashdump::channel_version;
		wchar_t value[128];
		std::swprintf(value, 128, L"%llx:%llx:%llx", static_cast<unsigned long long>(reinterpret_cast<ULONG_PTR>(mapping)), static_cast<unsigned long long>(reinterpret_cast<ULONG_PTR>(request)),
		              static_cast<unsigned long long>(reinterpret_cast<ULONG_PTR>(done)));
		::SetEnvironmentVariableW(crashdump::channel_variable, value);
	}

	std::wstring commandline{L"\"" + simulator + L"\""};
	auto const *passed{arguments()};
	if (*passed != L'\0')
	{
		commandline += L" ";
		commandline += passed;
	}
	STARTUPINFOW startup{};
	startup.cb = sizeof(startup);
	PROCESS_INFORMATION process{};
	if (::CreateProcessW(simulator.c_str(), commandline.data(), nullptr, nullptr, reporting ? TRUE : FALSE, 0, nullptr, nullptr, &startup, &process) == FALSE)
	{
		auto const error{::GetLastError()};
		wchar_t text[160];
		std::swprintf(text, 160, L"\n\nThe system refused to start it (error %lu)", static_cast<unsigned long>(error));
		show(L"The simulator could not be started:\n" + simulator + text +
		         (error == ERROR_BAD_EXE_FORMAT ? L": it is not a program for this version of Windows (it needs 64 bit Windows)." : L"."),
		     MB_OK | MB_ICONERROR);
		return 1;
	}
	::CloseHandle(process.hThread);

	bool reported{false};
	HANDLE const waited[]{process.hProcess, request};
	while (true)
	{
		auto const result{::WaitForMultipleObjects(reporting ? 2 : 1, waited, FALSE, INFINITE)};
		if (result == WAIT_OBJECT_0 + 1)
		{
			// the simulator crashed and waits: its minidump, written from here as the crashed process may be unable to
			if (channel->process_id == process.dwProcessId)
			{
				crashdump_write_minidump(process.hProcess, *channel, true);
				reported = true;
			}
			::SetEvent(done);
			continue;
		}
		break;
	}

	DWORD code{0};
	::GetExitCodeProcess(process.hProcess, &code);
	::CloseHandle(process.hProcess);
	if ((false == reported) && (code != cpu_features::exit_unsupported))
	{
		auto const explanation{explain_exit(code)};
		if (false == explanation.empty())
		{
			show(explanation, MB_OK | MB_ICONERROR);
		}
	}
	return static_cast<int>(code);
}
