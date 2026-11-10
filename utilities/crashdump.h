#pragma once

// the crash report of the simulator (WITHDUMPGEN, windows): a small minidump, a text description of the crash with the recent log lines
// and copies of the log files, written to crashdumps/. when the simulator runs under eu07_starter, the starter writes the minidump

#include <cstddef>
#include <string>

enum class crashdump_log_file
{
	log,
	errors
};

// sets up the reporting, at the start of main
void crashdump_install();
// a named value for the description of the crash: the renderer, the scenario, the graphics card...
void crashdump_set_info(char const *Name, std::string const &Value);
// a line of the log, the last ones go into the description of the crash
void crashdump_note_log(char const *Text, std::size_t Length);
// a log file, copied next to the dump when the simulator crashes
void crashdump_set_log_file(crashdump_log_file File, std::string const &Path);
