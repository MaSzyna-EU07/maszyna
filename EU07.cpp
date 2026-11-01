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
#include "utilities/crashdump.h"
#include <cstdlib>

#ifdef _MSC_VER
#ifdef EU07_CPU_PREFLIGHT
// checks the processor before the C runtime starts, see utilities/cpu_preflight.cpp
#pragma comment(linker, "/subsystem:windows /ENTRY:eu07_preflight_entry")
#else
#pragma comment(linker, "/subsystem:windows /ENTRY:mainCRTStartup")
#endif
#endif

void export_e3d_standalone(std::string in, std::string out, int flags, bool dynamic);
int export_btf_standalone(std::string in, std::string out);
int generate_nodebank_previews(int Argc, char *Argv[]);

#include <string>
#include <utilities/Globals.h>

// lets the log service write what is still queued and stops it; does nothing once Application.exit() has done that
static void stop_log_service()
{
	Global.applicationQuitOrder = true;
	auto const it = Global.threads.find("LogService");
	if (it != Global.threads.end() && it->second.joinable())
		it->second.join();
}

int main(int argc, char *argv[])
{
	crashdump_install();
	// init start timestamp
	Global.startTimestamp = std::chrono::steady_clock::now();

	int exitcode{0};

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
			else
			{
				exitcode = result;
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
	stop_log_service();
#ifndef _WIN32
	fflush(stdout);
	fflush(stderr);
#endif
	std::_Exit(exitcode); // skip destructors, there are ordering errors which causes segfaults
}
