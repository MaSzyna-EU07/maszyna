/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string_view>

// the stack of a crash on Linux, where no dump is written: a signal of a fault (SIGSEGV, SIGBUS, SIGFPE, SIGILL) or
// SIGABRT writes the stack of the thread that took it to the log and to the standard error, which the starter shows,
// and then ends the simulator as the signal always did (a core dump where the system makes one). the addresses are
// read with addr2line on the eu07 file of the same version, which carries its symbols. elsewhere it does nothing
namespace crash_signals {

// sets the handlers up, once. Version goes into the report, to tell which eu07 file the addresses belong to
void install(std::string_view Version);
// the log file the report is added to, once the log has opened it
void set_log_path(std::string_view Path);

} // namespace crash_signals
