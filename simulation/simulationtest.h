/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "simulation/simulationtestrules.h"

#include <string_view>

// integration test mode (eu07 --test -s <scenery>, eu07/maszyna#541): the scenery runs unattended in a tight
// fixed-step loop and the simulator quits with an exit code saying how the test went
namespace simulation {

class test_manager {

public:
	// name of the memory cell a scenery reports its result to
	static constexpr std::string_view result_cell { "__test.result" };
	// fixed simulation time step of one frame in test mode [s]: 3 physics steps of 10 ms
	static constexpr double frame_step { 0.03 };

	// creates the result cell; called with the other scripting interface cells, before the scenery is read
	void init_result_cell() const;
	// the simulation is ready: start the clocks
	void start();
	// checks the end conditions after a simulation frame of Deltatime seconds
	void update(double Deltatime);
	// reported by a driver whose train went past a signal showing stop
	void signal_passed_at_danger(std::string_view Vehicle, std::string_view Signal);
	// ends the test with given result and closes the application; the first call wins
	void finish(test_result Result, std::string_view Reason);
	// ends the test with given result, without closing the application: for the ways out that close it anyway
	// (a bad command line, a failed start, an exception). returns false if the test had already ended
	bool conclude(test_result Result, std::string_view Reason);

	bool finished() const { return m_finished; }
	// exit code of the process: the result, or error while the test hasn't finished
	int exit_code() const { return static_cast<int>(m_finished ? m_result : test_result::error); }

private:
	bool m_started { false };
	bool m_finished { false };
	test_result m_result { test_result::error };
	test_progress m_progress;
};

// the test of the run
test_manager &test();

} // namespace simulation
