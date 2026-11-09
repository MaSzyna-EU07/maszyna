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

#include "stdafx.h"

#include "application/application.h"
#include "simulation/simulationtest.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include <cstdlib>
#include <exception>
#include <string_view>
#include <thread>
#include <vector>
#ifdef WITHDUMPGEN
#ifdef _WIN32
#include <Windows.h>
#include <DbgHelp.h>
#endif
#endif

#ifdef _MSC_VER
#pragma comment(linker, "/subsystem:windows /ENTRY:mainCRTStartup")
#endif

void export_e3d_standalone(std::string in, std::string out, int flags, bool dynamic);
int export_btf_standalone(std::string in, std::string out);
int generate_nodebank_previews(int Argc, char *Argv[]);

#include <ctime>
#include <string>
#include <sstream>
#include <iomanip>
#include <utilities/Globals.h>

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

		// in test mode nobody is there to close the box, and the run would wait for it until killed
		if (false == Global.test.enabled)
			MessageBoxA(nullptr, "Simulator crash occured :(\n", "Simulator crashed :(", MB_ICONERROR);
		MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), hFile, dumpType, &dumpInfo, nullptr, nullptr);

		CloseHandle(hFile);
	}

	return EXCEPTION_EXECUTE_HANDLER;
}

#endif

namespace
{

// the log is written by a thread of its own, which the closing of the application stops and waits for. on the ways
// out that skip the closing (a failed start, an exception) what was logged last would be lost with the process
void finish_log()
{
	StopLogService();
	if (auto const logservice{Global.threads.find("LogService")};
	    logservice != Global.threads.end() && logservice->second.joinable() && logservice->second.get_id() != std::this_thread::get_id())
		logservice->second.join();
}

// ends a test run with its verdict on the standard output and in the exit code. a test the simulator didn't end
// (a failed start, an exception) ends with the error and Reason
[[noreturn]] void end_test_run(std::string_view const Reason)
{
	simulation::test().conclude(simulation::test_result::error, Reason);
	finish_log();
	fflush(stdout);
	fflush(stderr);
	// skip destructors, there are ordering errors which causes segfaults
	std::_Exit(simulation::test().exit_code());
}

// a test run (--test): whatever stops it, it ends with its verdict. any other run never gets here and ends as
// it always has
[[noreturn]] void run_test(int argc, char *argv[])
{
	// for what doesn't reach the catch below: an exception leaving a noexcept function, or on Linux one leaving
	// a thread (on Windows the handler is the main thread's only)
	std::set_terminate([] { end_test_run(simulation::describe_exception(std::current_exception())); });
	try
	{
		if (Application.init(argc, argv) == 0)
		{
			Application.run();
			Application.exit();
		}
	}
	catch (...)
	{
		end_test_run(simulation::describe_exception(std::current_exception()));
	}
	end_test_run("the simulator stopped before the test ended");
}

} // namespace

int main(int argc, char *argv[])
{
#ifdef WITHDUMPGEN
#ifdef _WIN32
	SetUnhandledExceptionFilter(CrashHandler);
#endif
#endif
	// init start timestamp
	Global.startTimestamp = std::chrono::steady_clock::now();

	// quick short-circuit for standalone e3d export
	if (argc == 6 && std::string(argv[1]) == "-e3d")
	{
		std::string in(argv[2]);
		std::string out(argv[3]);
		int flags = std::stoi(std::string(argv[4]));
		int dynamic = std::stoi(std::string(argv[5]));
		export_e3d_standalone(in, out, flags, dynamic);
	}
	// standalone generation of preview images of the node bank models: -generate-nodebank-previews [options]
	else if (argc >= 2 && std::string(argv[1]) == "-generate-nodebank-previews")
	{
		auto const result{generate_nodebank_previews(argc, argv)};
		fflush(stdout);
		fflush(stderr);
		std::_Exit(result);
	}
	// standalone conversion of text terrain file to binary format: -btf -s <text file> [-o <binary file>]
	else if (argc >= 4 && std::string(argv[1]) == "-btf")
	{
		std::string in;
		std::string out;
		for (int i = 2; i + 1 < argc; i += 2)
		{
			std::string const option(argv[i]);
			if (option == "-s")
			{
				in = argv[i + 1];
			}
			else if (option == "-o")
			{
				out = argv[i + 1];
			}
		}
		auto const result{export_btf_standalone(in, out)};
		fflush(stdout);
		fflush(stderr);
		std::_Exit(result);
	}
	// unattended test run of a scenery, which says its result in the exit code (eu07/maszyna#541)
	else if (simulation::parse_test_command_line(std::vector<char const *>(argv, argv + argc)).enabled)
	{
		run_test(argc, argv);
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
