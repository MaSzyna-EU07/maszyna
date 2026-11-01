/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the check of the processor of a build with ENABLE_AVX2, before any code built with AVX2 runs. the start of the C runtime runs
// the constructors of the global objects, already built with it: on a processor without AVX2 the simulator would end there
// without a word. this file is built without AVX2 and without the precompiled header (eu07_preflight in CMakeLists.txt)

#include "utilities/cpu_features.h"

#ifdef _WIN32

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

// the entry point of the C runtime, started by the one below
extern "C" int mainCRTStartup(void *);

namespace
{

#ifdef _MSC_VER
// the runtime checks need the C runtime, which is not started yet
#pragma runtime_checks("", off)
#endif

void append(wchar_t *Output, int const Capacity, int &Length, char const *Text)
{
	while ((*Text != '\0') && (Length < Capacity - 1))
	{
		Output[Length++] = static_cast<wchar_t>(*Text++);
	}
	Output[Length] = L'\0';
}

#ifdef _MSC_VER
__declspec(noinline) __declspec(safebuffers)
#endif
void report_missing(unsigned int const Missing)
{
	wchar_t text[512];
	int length{0};
	append(text, 512, length, "This processor does not support the AVX2 instruction set the simulator is built for. The simulator won't run.\n\nMissing: ");
	bool first{true};
	for (unsigned int feature = 1; feature <= cpu_features::os_avx; feature <<= 1)
	{
		if ((Missing & feature) != 0)
		{
			append(text, 512, length, first ? "" : ", ");
			append(text, 512, length, cpu_features::name(feature));
			first = false;
		}
	}
	append(text, 512, length, "\n\nIt needs a processor with AVX2: Intel Core of the 4th generation (Haswell, 2013), AMD of the Excavator generation (2015) or newer.");
	::MessageBoxW(nullptr, text, L"MaSzyna", MB_ICONERROR | MB_OK);
}

} // namespace

// the entry point of the simulator (/ENTRY), runs before the C runtime starts. without a buffer security check, the security cookie
// is set up by the runtime started from here
extern "C"
#ifdef _MSC_VER
    __declspec(safebuffers)
#endif
    int eu07_preflight_entry(void *Peb)
{
	auto const missing{cpu_features::missing(cpu_features::required_for_avx2)};
	if (missing != 0)
	{
		report_missing(missing);
		::ExitProcess(cpu_features::exit_unsupported);
	}
	return mainCRTStartup(Peb);
}

#ifdef _MSC_VER
#pragma runtime_checks("", restore)
#endif

#else

#include <cstdio>
#include <cstdlib>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char **environ;

namespace
{

bool show_dialog(char const *Program, char const *const *Arguments)
{
	pid_t process;
	if (posix_spawnp(&process, Program, nullptr, nullptr, const_cast<char *const *>(Arguments), environ) != 0)
	{
		return false;
	}
	int status{0};
	return (waitpid(process, &status, 0) == process) && WIFEXITED(status) && (WEXITSTATUS(status) != 127);
}

// before the constructors of the other global objects of the simulator
__attribute__((constructor(101))) void eu07_preflight()
{
	auto const missing{cpu_features::missing(cpu_features::required_for_avx2)};
	if (missing == 0)
	{
		return;
	}
	char text[512];
	int length{std::snprintf(text, sizeof(text), "This processor does not support the AVX2 instruction set the simulator is built for. The simulator won't run.\nMissing:")};
	for (unsigned int feature = 1; feature <= cpu_features::os_avx; feature <<= 1)
	{
		if (((missing & feature) != 0) && (length > 0) && (length < static_cast<int>(sizeof(text))))
		{
			length += std::snprintf(text + length, sizeof(text) - length, " %s", cpu_features::name(feature));
		}
	}
	std::fprintf(stderr, "%s\n", text);
	// started from the desktop or steam, the console is not seen
#ifdef __APPLE__
	char script[640];
	std::snprintf(script, sizeof(script), "display alert \"MaSzyna\" message \"%s\" as critical", text);
	char const *const osascript[]{"osascript", "-e", script, nullptr};
	show_dialog("osascript", osascript);
#else
	if ((std::getenv("DISPLAY") != nullptr) || (std::getenv("WAYLAND_DISPLAY") != nullptr))
	{
		char const *const zenity[]{"zenity", "--error", "--title=MaSzyna", "--text", text, nullptr};
		char const *const kdialog[]{"kdialog", "--title", "MaSzyna", "--error", text, nullptr};
		if (false == show_dialog("zenity", zenity))
		{
			show_dialog("kdialog", kdialog);
		}
	}
#endif
	std::_Exit(1);
}

} // namespace

#endif
