/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <optional>
#include <span>
#include <string>
#include <string_view>

// the rules of the test mode (eu07 --test, eu07/maszyna#541) which need nothing of the running simulation: the exit
// codes, the command line and the end conditions. kept apart so the unit tests can check them
namespace simulation {

// exit codes of a test run; public, scripts and CI read them
enum class test_result : int {
	pass = 0, // __test.result set to 1, then every vehicle came to rest (or test.pass_on_standstill)
	fail = 1, // __test.result set to 0
	signal_passed_at_danger = 2, // a train went past a stop signal (SetVelocity 0 0), unless test.fail_if_stop_overran no
	timeout = 3, // test.timeout seconds of simulation time went by without a result
	standstill = 4, // every vehicle stood still for test.standstill seconds without a result
	error = 5, // the run could not start (a bad command line, the scenery failed to load) or the simulator threw
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
	std::string error; // what's wrong with them; empty if nothing
};

// seconds given on the command line: a number, not negative. nothing for anything else
std::optional<double> parse_seconds(std::string_view Text);

// picks the test mode options out of a command line (the first argument is the program). the values of the other
// options are skipped, so a scenery named like an option isn't taken for one; options it doesn't know are left alone
test_command_line parse_test_command_line(std::span<char const *const> Arguments);

// the limits and switches the end conditions go by
struct test_limits {
	double timeout{3600.0}; // [s] of simulation time; 0 = never
	double standstill{300.0}; // [s] of every vehicle standing without a result; 0 = never
	bool pass_on_standstill{false};
	bool no_movement_before_pass{true};
};

// what a frame of the simulation brought
struct test_frame {
	double deltatime{0.0}; // [s] of simulation time
	std::optional<double> reported; // value of the result cell, if there is one
	std::string_view reason; // text of the result cell
	bool atrest{false}; // every vehicle stands
};

// the state of a test between frames
struct test_progress {
	double time{0.0}; // simulation time since the start [s]
	double resttime{0.0}; // how long every vehicle has been standing [s]
	bool passrequested{false}; // __test.result was set to 1, the pass waits for the vehicles to stop
};

// how long everything has to stand before a requested pass is granted [s]
inline constexpr double pass_settle_time{1.0};

// checks the end conditions after a frame and moves the progress on. returns the verdict once the test is over
std::optional<test_verdict> evaluate_frame(test_progress &Progress, test_frame const &Frame, test_limits const &Limits);

} // namespace simulation
