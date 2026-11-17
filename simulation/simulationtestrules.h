/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <exception>
#include <optional>
#include <span>
#include <string>
#include <string_view>

// the rules of the test mode (eu07 --test, eu07/maszyna#541) which need nothing of the running simulation: the exit
// codes, the command line, the end conditions and the real time limit. kept apart so the unit tests can check them
namespace simulation {

// exit codes of a test run; public, scripts and CI read them. none of them is a code a process commonly ends with
// when it goes wrong on its own: 1 (killed with TerminateProcess, or a plain failure of most programs), 3 (abort()
// on Windows), 126-127 (a shell that can't run the program) or 128 + a signal number (killed, on Linux)
enum class test_result : int {
	pass = 0, // __test.result set to 1, then the tested trains came to rest (or test.pass_on_standstill)
	fail = 10, // __test.result set to 0
	signal_passed_at_danger = 11, // a train went past a stop signal (SetVelocity 0 0), unless test.fail_if_stop_overran no
	timeout = 12, // test.timeout seconds of simulation time, or test.wall_timeout seconds of real time went by
	standstill = 13, // the tested trains stood still for test.standstill seconds without a result
	error = 14, // the run could not start (a bad command line, the scenery failed to load) or the simulator threw
};

std::string_view to_string(test_result Result);

// how a test ended, and why
struct test_verdict {
	test_result result{test_result::error};
	std::string reason;
};

// the test mode options of the command line
struct test_command_line {
	bool enabled{false}; // --test
	std::optional<double> timeout; // --test-timeout <seconds of simulation time>
	std::optional<double> wall_timeout; // --test-wall-timeout <seconds of real time>
	std::string error; // what's wrong with them; empty if nothing
};

// seconds given on the command line: a number, not negative. nothing for anything else
std::optional<double> parse_seconds(std::string_view Text);

// picks the test mode options out of a command line (the first argument is the program). the values of the other
// options are skipped, so a scenery named like an option isn't taken for one; options it doesn't know are left alone
test_command_line parse_test_command_line(std::span<char const *const> Arguments);

// what ended a test run that the simulator didn't end itself: the exception caught (or being handled when the
// program was terminated), or nothing
std::string describe_exception(std::exception_ptr const &Exception);

// time allowed for loading the scenery, on top of the simulation time, when the real time limit isn't given [s]
inline constexpr double wall_timeout_loading_allowance{600.0};

// the real time limit of a test run [s], 0 = none: the one given, else the simulation time limit plus time for the
// loading, so a simulation that runs at least as fast as real time is never cut short. no simulation time limit
// (test.timeout 0) means none in real time either, unless given
double wall_clock_limit(std::optional<double> Given, double Simulationtimeout);

// the limits and switches the end conditions go by
struct test_limits {
	double timeout{3600.0}; // [s] of simulation time; 0 = never
	double standstill{300.0}; // [s] of the tested trains standing without a result; 0 = never
	bool pass_on_standstill{false};
	bool no_movement_before_pass{true};
};

// what a frame of the simulation brought
struct test_frame {
	double deltatime{0.0}; // [s] of simulation time
	std::optional<double> reported; // value of the result cell, if there is one
	std::string_view reason; // text of the result cell
	bool atrest{false}; // every vehicle of the tested trains stands (test.vehicle; every vehicle without it)
};

// the state of a test between frames
struct test_progress {
	double time{0.0}; // simulation time since the start [s]
	double resttime{0.0}; // how long the tested trains have been standing [s]
	bool passrequested{false}; // __test.result was set to 1, the pass waits for the tested trains to stop
};

// how long the tested trains have to stand before a requested pass is granted [s]
inline constexpr double pass_settle_time{1.0};

// checks the end conditions after a frame and moves the progress on. returns the verdict once the test is over
std::optional<test_verdict> evaluate_frame(test_progress &Progress, test_frame const &Frame, test_limits const &Limits);

} // namespace simulation
