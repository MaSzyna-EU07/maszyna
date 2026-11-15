/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

class TMoverParameters;

// starting the engines (closing the line breakers) of the vehicles in multiple from the cab (eu07/maszyna#80). in
// vehicles like the ST43, the SP32 or the SN61 every engine of the train has its own switches in the cab: main_* work
// the vehicle the cab controls (TTrain::mvControlled: the cab's own vehicle, on an EMU its motor car), mainb_* the
// second one and mainc_* the third one under its control. Engine:JointEngineStart=No in the .fiz of the controlled
// vehicle (on an EMU the motor car's, not the one of the car with the cab) keeps main_* to that vehicle alone
// (TMoverParameters::MainSwitchRange()); by default it works all of them, as before.
namespace engine_start {

// the Number-th vehicle with an engine under the cab's control: 1 is Controlled, the cab's own; then the others
// connected through the control cable, those behind the cab of Occupied first, the nearest first, then those in front
// of it. nullptr when there are fewer of them
TMoverParameters *engine(TMoverParameters const &Occupied, TMoverParameters &Controlled, int Number);

// the switch starting the engine of another vehicle in multiple. like the cab's own switch, it has to be held closed
// for the vehicle's InitialCtrlDelay, with the vehicle ready for it, before the engine starts (the line breaker closes)
class remote_switch {
public:
	// one step of Deltatime seconds of the switch of Engine, with the switch held closed (Closeheld) or open (Openheld).
	// with Closesonrelease, the line breaker closes when the switch is let go after the delay, as the impulse switch
	// of an electric locomotive with series motors does
	void update(TMoverParameters *Engine, bool Closeheld, bool Openheld, double Deltatime, bool Closesonrelease);
	// opens the line breaker / stops the engine of Engine, the vehicle alone; returns whether it was running
	static bool open(TMoverParameters &Engine);
	// whether the line breaker of Engine is closed or its engine starting or running
	static bool is_running(TMoverParameters const &Engine);

private:
	TMoverParameters const *m_engine{nullptr}; // the vehicle the switch worked the last step
	double m_heldtime{0.0}; // how long the switch is held closed with the vehicle ready to start
	bool m_ready{false}; // held long enough, the line breaker closes now or when the switch is let go
};

} // namespace engine_start
