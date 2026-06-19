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
#include "utilities/Logs.h"
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <exception>
#ifdef _WIN32
#include <Windows.h>
#include <DbgHelp.h>
#endif

#ifdef _MSC_VER
#pragma comment(linker, "/subsystem:windows /ENTRY:mainCRTStartup")
#endif

void export_e3d_standalone(std::string in, std::string out, int flags, bool dynamic);

#include <ctime>
#include <string>
#include <sstream>
#include <iomanip>
#include <utilities/Globals.h>

#ifdef _WIN32
#pragma comment(lib, "Dbghelp.lib")

// Builds a "crash_YYYY-MM-DD_HH-MM-SS" timestamp shared by the .txt and .dmp.
static std::string crash_timestamp()
{
	SYSTEMTIME st;
	GetLocalTime(&st);
	std::ostringstream oss;
	oss << "crash_" << std::setw(4) << std::setfill('0') << st.wYear << "-" << std::setw(2) << std::setfill('0') << st.wMonth << "-" << std::setw(2) << std::setfill('0') << st.wDay << "_"
	    << std::setw(2) << std::setfill('0') << st.wHour << "-" << std::setw(2) << std::setfill('0') << st.wMinute << "-" << std::setw(2) << std::setfill('0') << st.wSecond;
	return oss.str();
}

// Writes a human-readable crash report with a symbolized call stack straight to
// a file. Deliberately bypasses the engine's async logger (WriteLog/ErrorLog
// push to a worker thread behind a mutex that may be dead or held at crash
// time), writing synchronously instead so the report survives the crash.
// ExceptionInfo may be null (e.g. from std::terminate), in which case the
// current thread context is captured on the spot.
static void WriteCrashReport(const std::string &basename, const char *reason, EXCEPTION_POINTERS *ExceptionInfo)
{
	std::ofstream out(basename + ".txt", std::ios::trunc);
	if (!out.is_open())
		return;

	out << "MaSzyna EU07 crash report\n";
	out << "version: " << Global.asVersion << "\n";
	out << "reason:  " << (reason ? reason : "unknown") << "\n";

	HANDLE process = GetCurrentProcess();
	CONTEXT localContext;
	CONTEXT *ctx;

	if (ExceptionInfo && ExceptionInfo->ContextRecord)
	{
		ctx = ExceptionInfo->ContextRecord;
		if (const EXCEPTION_RECORD *er = ExceptionInfo->ExceptionRecord)
		{
			const char *name = nullptr;
			switch (er->ExceptionCode)
			{
			case EXCEPTION_ACCESS_VIOLATION:      name = "ACCESS_VIOLATION"; break;
			case EXCEPTION_STACK_OVERFLOW:        name = "STACK_OVERFLOW"; break;
			case EXCEPTION_ILLEGAL_INSTRUCTION:   name = "ILLEGAL_INSTRUCTION"; break;
			case EXCEPTION_INT_DIVIDE_BY_ZERO:    name = "INT_DIVIDE_BY_ZERO"; break;
			case EXCEPTION_FLT_DIVIDE_BY_ZERO:    name = "FLT_DIVIDE_BY_ZERO"; break;
			case EXCEPTION_PRIV_INSTRUCTION:      name = "PRIV_INSTRUCTION"; break;
			case EXCEPTION_IN_PAGE_ERROR:         name = "IN_PAGE_ERROR"; break;
			case 0xE06D7363:                      name = "C++ exception (MSVC)"; break;
			default: break;
			}
			out << "code:    0x" << std::hex << er->ExceptionCode << std::dec;
			if (name) out << " (" << name << ")";
			out << "\n";
			out << "address: 0x" << std::hex << static_cast<DWORD64>(reinterpret_cast<uintptr_t>(er->ExceptionAddress)) << std::dec << "\n";
			if (er->ExceptionCode == EXCEPTION_ACCESS_VIOLATION && er->NumberParameters >= 2)
			{
				const ULONG_PTR op = er->ExceptionInformation[0];
				out << "access:  " << (op == 1 ? "write" : op == 8 ? "execute" : "read")
				    << " at 0x" << std::hex << er->ExceptionInformation[1] << std::dec << "\n";
			}
		}
	}
	else
	{
		RtlCaptureContext(&localContext);
		ctx = &localContext;
	}

	out << "\ncall stack:\n";

	SymSetOptions(SYMOPT_UNDNAME | SYMOPT_LOAD_LINES | SYMOPT_DEFERRED_LOADS);
	SymInitialize(process, nullptr, TRUE);

	STACKFRAME64 frame = {};
	DWORD machine;
#if defined(_M_X64)
	machine = IMAGE_FILE_MACHINE_AMD64;
	frame.AddrPC.Offset = ctx->Rip;
	frame.AddrFrame.Offset = ctx->Rbp;
	frame.AddrStack.Offset = ctx->Rsp;
#elif defined(_M_IX86)
	machine = IMAGE_FILE_MACHINE_I386;
	frame.AddrPC.Offset = ctx->Eip;
	frame.AddrFrame.Offset = ctx->Ebp;
	frame.AddrStack.Offset = ctx->Esp;
#endif
	frame.AddrPC.Mode = AddrModeFlat;
	frame.AddrFrame.Mode = AddrModeFlat;
	frame.AddrStack.Mode = AddrModeFlat;

	char symbolMem[sizeof(SYMBOL_INFO) + 256];
	SYMBOL_INFO *symbol = reinterpret_cast<SYMBOL_INFO *>(symbolMem);

	for (int i = 0; i < 128; ++i)
	{
		if (!StackWalk64(machine, process, GetCurrentThread(), &frame, ctx, nullptr, SymFunctionTableAccess64, SymGetModuleBase64, nullptr))
			break;
		const DWORD64 pc = frame.AddrPC.Offset;
		if (pc == 0)
			break;

		out << "  [" << std::setw(2) << std::setfill('0') << i << "] ";

		IMAGEHLP_MODULE64 mod = {};
		mod.SizeOfStruct = sizeof(mod);
		if (SymGetModuleInfo64(process, pc, &mod))
			out << mod.ModuleName << "!";

		symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
		symbol->MaxNameLen = 255;
		DWORD64 disp = 0;
		if (SymFromAddr(process, pc, &disp, symbol))
			out << symbol->Name << " + 0x" << std::hex << disp << std::dec;
		else
			out << "0x" << std::hex << pc << std::dec;

		IMAGEHLP_LINE64 lineInfo = {};
		lineInfo.SizeOfStruct = sizeof(IMAGEHLP_LINE64);
		DWORD lineDisp = 0;
		if (SymGetLineFromAddr64(process, pc, &lineDisp, &lineInfo))
			out << "  (" << lineInfo.FileName << ":" << lineInfo.LineNumber << ")";

		out << "\n";
	}

	SymCleanup(process);
	out.flush();
	out.close();
}

LONG WINAPI CrashHandler(EXCEPTION_POINTERS *ExceptionInfo)
{
	const std::string basename = crash_timestamp();

	// Readable stack trace first -- it's the most useful artifact and needs no
	// debugger to inspect.
	WriteCrashReport(basename, "unhandled SEH exception", ExceptionInfo);

	HANDLE hFile = CreateFileA((basename + ".dmp").c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (hFile != INVALID_HANDLE_VALUE)
	{
		MINIDUMP_EXCEPTION_INFORMATION dumpInfo;
		dumpInfo.ThreadId = GetCurrentThreadId();
		dumpInfo.ExceptionPointers = ExceptionInfo;
		dumpInfo.ClientPointers = FALSE;

		// Wybrana kombinacja flag
		MINIDUMP_TYPE dumpType = MINIDUMP_TYPE(MiniDumpWithFullMemory | MiniDumpWithHandleData | MiniDumpWithThreadInfo | MiniDumpWithUnloadedModules | MiniDumpWithIndirectlyReferencedMemory |
		                                       MiniDumpWithFullMemoryInfo | MiniDumpWithTokenInformation);

		MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), hFile, dumpType, &dumpInfo, nullptr, nullptr);

		CloseHandle(hFile);
	}

	MessageBoxA(nullptr,
	            ("Simulator crash occured :(\n\nA crash report was written next to the executable:\n" + basename + ".txt  (readable call stack)\n" + basename + ".dmp  (full minidump)").c_str(),
	            "Simulator crashed :(", MB_ICONERROR);

	return EXCEPTION_EXECUTE_HANDLER;
}

// Catches uncaught C++ exceptions (and other std::terminate triggers such as
// pure-virtual calls), which the SEH filter above does NOT see. Extracts the
// in-flight exception's message when possible.
static void CrashTerminateHandler()
{
	std::string reason = "std::terminate";
	if (std::exception_ptr ep = std::current_exception())
	{
		try { std::rethrow_exception(ep); }
		catch (const std::exception &e) { reason = std::string("uncaught exception: ") + e.what(); }
		catch (...)                     { reason = "uncaught non-standard exception"; }
	}

	const std::string basename = crash_timestamp();
	WriteCrashReport(basename, reason.c_str(), nullptr);
	MessageBoxA(nullptr, ("Simulator terminated :(\n\n" + reason + "\n\nSee " + basename + ".txt for the call stack.").c_str(), "Simulator crashed :(", MB_ICONERROR);
	std::_Exit(3);
}

#endif

int main(int argc, char *argv[])
{
#ifdef _WIN32
	SetUnhandledExceptionFilter(CrashHandler);
	std::set_terminate(CrashTerminateHandler);
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
#ifdef _WIN32
			WriteCrashReport(crash_timestamp(), (std::string("memory allocation failure: ") + Error.what()).c_str(), nullptr);
#endif
		}
#ifdef _WIN32
		catch (std::exception const &Error)
		{
			const std::string basename = crash_timestamp();
			WriteCrashReport(basename, (std::string("unhandled exception: ") + Error.what()).c_str(), nullptr);
			MessageBoxA(nullptr, ("Simulator crash occured :(\n\n" + std::string(Error.what()) + "\n\nSee " + basename + ".txt for details.").c_str(), "Simulator crashed :(", MB_ICONERROR);
		}
		catch (...)
		{
			const std::string basename = crash_timestamp();
			WriteCrashReport(basename, "unhandled non-standard exception", nullptr);
			MessageBoxA(nullptr, ("Simulator crash occured :(\n\nSee " + basename + ".txt for details.").c_str(), "Simulator crashed :(", MB_ICONERROR);
		}
#endif
	}
#ifndef _WIN32
	fflush(stdout);
	fflush(stderr);
#endif
	std::_Exit(0); // skip destructors, there are ordering errors which causes segfaults
}
