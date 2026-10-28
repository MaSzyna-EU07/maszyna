#pragma once

#include <functional>
#include <string>

#include "vehicle.h"

namespace test {

class trace;
class consist;

// what a driver does with a vehicle, made of the steps the AI takes (TController and the driver hints of
// application/driverhints.cpp), on the vehicle's own controls. the driver sits in a vehicle with a cab
// (Cab != 0): a lone one, or the first one of a consist, and drives the whole of it

// what the driver drives: the vehicle with the driver in it, the vehicles the driver controls the engine and
// the pantographs of (the AI's mvOccupied, mvControlling, mvPantographUnit; all the same one in a lone
// vehicle or a locomotive), and the way to run the whole of it
struct driven {
	TMoverParameters &cab;
	TMoverParameters &controlling;
	TMoverParameters &pantographs;
	std::function<void(double Seconds, track const &Track)> run;
};
driven driving(vehicle &Vehicle);
driven driving(consist &Consist);

// gets the vehicle ready to run, the way TController::PrepareEngine() does: battery, cab, pumps and heating
// of a diesel engine, pantographs, line breaker, converter, compressor and motor blowers, then waits for the
// air. stops trying after Seconds; returns whether the vehicle got ready
bool start_up(driven Driven, track const &Track, double Seconds = 300.0, trace *Trace = nullptr);

// runs the vehicle for Seconds with the driver paying attention, as TController::control_security_system()
// does: once a second the vigilance and the cab signalling get acknowledged when they call for it
void drive(driven Driven, track const &Track, double Seconds);

// releases every brake the driver has a hand on: the train brake to running, the independent brake, the
// hand brake and the spring brake; and the hand and spring brakes of the vehicles coupled to the cab's
void release_brakes(TMoverParameters &Cab);

// whether the vehicle is ready to run: line breaker closed (or the engine running), air in the main reservoir
bool is_ready(driven const &Driven);

// whether the vehicle drives itself
bool is_powered(TMoverParameters &Vehicle);

// a trace mark, when there's a trace
void mark(trace *Trace, std::string const &Event);

// a whole run: start up, release the brakes, notch up to full power, the second controller (field weakening,
// shunting) up and down, notch down, coast and brake to a stop. the controllers move on a fixed schedule,
// not the way a driver watching the ammeter would, so the relays may trip on the way
void drive_cycle(driven Driven, track const &Track, trace &Trace);

} // namespace test
