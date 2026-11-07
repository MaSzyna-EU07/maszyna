/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>

// integration test mode (eu07 --test -s <scenery>, eu07/maszyna#541): the scenery runs unattended in a tight
// fixed-step loop and the simulator quits with an exit code saying how the test went
namespace simulation {

// exit codes of a test run; public, scripts and CI read them
enum class test_result : int {
	pass = 0, // __test.result set to 1, then every vehicle came to rest (or test.pass_on_standstill)
	fail = 1, // __test.result set to 0
	signal_passed_at_danger = 2, // a train went past a stop signal (SetVelocity 0 0), unless test.fail_if_stop_overran no
	timeout = 3, // test.timeout seconds of simulation time went by without a result
	standstill = 4, // every vehicle stood still for test.standstill seconds without a result
	error = 5, // the run could not start (no -s, the scenery failed to load) or the simulator threw
};

std::string_view to_string(test_result Result);

class test_manager {

public:
	// name of the memory cell a scenery reports its result to
	static constexpr std::string_view result_cell { "__test.result" };
	// how long everything has to stand before a requested pass is granted [s]
	static constexpr double pass_settle_time { 1.0 };
	// fixed simulation time step of one frame in test mode [s]: 3 physics steps of 10 ms
	static constexpr double frame_step { 0.03 };

	// creates the result cell; called with the other scripting interface cells, before the scenery is read
	void init_result_cell();
	// the simulation is ready: start the clocks
	void start();
	// checks the end conditions after a simulation frame of Deltatime seconds
	void update(double Deltatime);
	// reported by a driver whose train went past a signal showing stop
	void signal_passed_at_danger(std::string const &Vehicle, std::string const &Signal);
	// ends the test with given result; the first call wins
	void finish(test_result Result, std::string const &Reason);

	bool finished() const { return m_finished; }
	// exit code of the process: the result, or error while the test hasn't finished
	int exit_code() const { return static_cast<int>(m_finished ? m_result : test_result::error); }

private:
	bool m_started { false };
	bool m_finished { false };
	test_result m_result { test_result::error };
	bool m_passrequested { false };
	double m_time { 0.0 }; // simulation time since the start [s]
	double m_resttime { 0.0 }; // how long every vehicle has been standing [s]
};

extern test_manager Test;

} // namespace simulation
