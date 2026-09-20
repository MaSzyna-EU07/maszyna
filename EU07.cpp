#include <cstdlib>
#ifdef WITHDUMPGEN
#ifdef _WIN32
#include <Windows.h>
#endif
#endif
#ifdef WITHDUMPGEN
#ifdef _WIN32
#include <DbgHelp.h>
#endif
#endif
#include <ctime>
#include <string>
#include <sstream>
#include <iomanip>
#include "utilities/Globals_macros.h"
#include <chrono>
#ifdef _WIN32
#include <windows.h>
#endif
#include <array>
#include <cstddef>
#include <deque>
#include <limits>
#include <mutex>
#include <ostream>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>
import eu07.application.application;
import eu07.utilities.logs;
import eu07.utilities.globals;
/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
/*
MaSzyna EU07 locomotive simulator
Copyright (C) 2001-2004  Marcin Wozniak, Maciej Czapkiewicz and others
*/
/*
Authors:
MarcinW, McZapkie, Shaxbee, ABu, nbmx, youBy, Ra, winger, mamut, Q424,
Stele, firleju, szociu, hunter, ZiomalCl, OLI_EU and others
*/


#ifdef WITHDUMPGEN
#ifdef _WIN32
#endif
#endif

#ifdef _MSC_VER
#pragma comment(linker, "/subsystem:windows /ENTRY:mainCRTStartup")
#endif

void export_e3d_standalone(std::string in, std::string out, int flags, bool dynamic);


#ifdef _WIN32
#endif

#ifdef _WIN32
#pragma comment(lib, "Dbghelp.lib")

LONG WINAPI CrashHandler(EXCEPTION_POINTERS *ExceptionInfo)
{
	// Get current local time
	SYSTEMTIME st;
	GetLocalTime(&st);

	// Format: crash_YYYY-MM-DD_HH-MM-SS.dmp
	std::ostringstream oss;
	oss << "crash_" << std::setw(4) << std::setfill('0') << st.wYear << "-" << std::setw(2) << std::setfill('0') << st.wMonth << "-" << std::setw(2) << std::setfill('0') << st.wDay << "_"
	    << std::setw(2) << std::setfill('0') << st.wHour << "-" << std::setw(2) << std::setfill('0') << st.wMinute << "-" << std::setw(2) << std::setfill('0') << st.wSecond << ".dmp";

	std::string filename = oss.str();

	HANDLE hFile = CreateFileA(filename.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (hFile != INVALID_HANDLE_VALUE)
	{
		MINIDUMP_EXCEPTION_INFORMATION dumpInfo;
		dumpInfo.ThreadId = GetCurrentThreadId();
		dumpInfo.ExceptionPointers = ExceptionInfo;
		dumpInfo.ClientPointers = FALSE;

		// Wybrana kombinacja flag
		MINIDUMP_TYPE dumpType = MINIDUMP_TYPE(MiniDumpWithFullMemory | MiniDumpWithHandleData | MiniDumpWithThreadInfo | MiniDumpWithUnloadedModules | MiniDumpWithIndirectlyReferencedMemory |
		                                       MiniDumpWithFullMemoryInfo | MiniDumpWithTokenInformation);

		MessageBoxA(nullptr, "Simulator crash occured :(\n", "Simulator crashed :(", MB_ICONERROR);
		MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), hFile, dumpType, &dumpInfo, nullptr, nullptr);

		CloseHandle(hFile);
	}

	return EXCEPTION_EXECUTE_HANDLER;
}

#endif

#if !defined(_WIN32)
// Ask for the discrete card before anything touches gl.
//
// A laptop with two graphics chips hands an application the integrated one unless it says otherwise,
// and that is what was happening here: a machine with a GeForce RTX 5060 was drawing on a Radeon 780M.
// The choice is made by libglvnd when it first resolves a gl entry point, which is after this runs, so
// setting the environment here is enough - no wrapper script, no launcher. Names already set by hand
// are left alone, and prefergpu in eu07.ini turns the whole thing off for anyone who would rather keep
// the battery.
void prefer_discrete_gpu()
{
	auto const ask = [](char const *Name, char const *Value) {
		if (nullptr == std::getenv(Name))
		{
			::setenv(Name, Value, 0);
		}
	};
	// nvidia's prime render offload, for glx and for egl. the vendor file is where the driver puts it
	ask("__NV_PRIME_RENDER_OFFLOAD", "1");
	ask("__GLX_VENDOR_LIBRARY_NAME", "nvidia");
	ask("__EGL_VENDOR_LIBRARY_FILENAMES", "/usr/share/glvnd/egl_vendor.d/10_nvidia.json");
	ask("__VK_LAYER_NV_optimus", "NVIDIA_only");
	// and the mesa side of the same question, for an amd or intel pair
	ask("DRI_PRIME", "1");
}
#endif

int main(int argc, char *argv[])
{
#ifdef WITHDUMPGEN
#ifdef _WIN32
	SetUnhandledExceptionFilter(CrashHandler);
#endif
#endif
	// init start timestamp
	Global.startTimestamp = std::chrono::steady_clock::now();

#if !defined(_WIN32)
	// before the configuration is read, because by the time gl is asked for a context it is too late.
	// -integratedgpu on the command line, or prefergpu integrated in eu07.ini, is read below and undoes
	// this by asking the driver for nothing in particular on the next start
	{
		auto discrete = true;
		for (int index = 1; index < argc; ++index)
		{
			if (std::string(argv[index]) == "-integratedgpu")
			{
				discrete = false;
			}
		}
		if (true == discrete)
		{
			prefer_discrete_gpu();
		}
	}
#endif

	// quick short-circuit for standalone e3d export
	if (argc == 6 && std::string(argv[1]) == "-e3d")
	{
		std::string in(argv[2]);
		std::string out(argv[3]);
		int flags = std::stoi(std::string(argv[4]));
		int dynamic = std::stoi(std::string(argv[5]));
		export_e3d_standalone(in, out, flags, dynamic);
	}
	else
	{
		try
		{
			auto result{Application.init(argc, argv)};
			if (result == 0)
			{
				result = Application.run();
				Application.exit();
			}
		}
		catch (std::bad_alloc const &Error)
		{
			ErrorLog("Critical error, memory allocation failure: " + std::string(Error.what()));
		}
#ifdef _WIN32
		catch (std::runtime_error const &Error)
		{
			std::string msg = "Simulator crash occured :(\n";
			msg += Error.what();
			MessageBoxA(nullptr, msg.c_str(), "Simulator crashed :(", MB_ICONERROR);
		}
#endif
	}
#ifndef _WIN32
	fflush(stdout);
	fflush(stderr);
#endif
	std::_Exit(0); // skip destructors, there are ordering errors which causes segfaults
}
