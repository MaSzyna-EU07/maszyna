/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/crewchange.h"

#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "vehicle/Train.h"
#include "vehicle/handover.h"

namespace crew {

namespace {

// TMoverParameters::CategoryFlag of the rail vehicles
constexpr int rail_category{1};
// more vehicles than a consist has, for a walk along one whose couplers form a loop
constexpr int consist_walk_limit{300};

// calls the function for each vehicle of the consist, from one end of it to the other. goes by the couplers
// alone, as the driver's own check of the consist does, so it works for a consist nobody has put in order
template <typename Function_>
void for_each_in_consist(TDynamicObject &Vehicle, Function_ const Function) {
	auto side{static_cast<int>(end::front)};
	auto *vehicle{Vehicle.FirstFind(side)};
	side = 1 - side;
	auto count{0};
	while (vehicle != nullptr && count < consist_walk_limit) {
		Function(*vehicle);
		vehicle = vehicle->Neighbour(side);
		++count;
	}
}

// whether another vehicle of the consist has the driver in charge of it
bool is_consist_driven(TDynamicObject &Vehicle) {
	auto driven{false};
	for_each_in_consist(Vehicle, [&Vehicle, &driven](TDynamicObject const &Other) {
		if (&Other != &Vehicle && Other.Mechanik != nullptr && Other.Mechanik->primary()) {
			driven = true;
		}
	});
	return driven;
}

vehicle_state state_of(TDynamicObject &Vehicle) {
	vehicle_state state;
	// a rail vehicle has a cab when it has the driver's brake valve. the others (cars, boats) have no such valve
	// to tell by, and each of them is driven from its own seat
	state.has_cab = Vehicle.MoverParameters->HasDriverCab() || Vehicle.MoverParameters->CategoryFlag != rail_category;
	state.has_driver = Vehicle.Mechanik != nullptr;
	state.driver_is_ai = state.has_driver && Vehicle.Mechanik->AIControllFlag;
	// a cab of the game is bound to the vehicle: the user's own, or another user's in a network game. the cab
	// reads the driver of its vehicle all the time, so that driver stays
	state.occupied = simulation::Trains.find(Vehicle.name()) != nullptr;
	state.consist_driven = is_consist_driven(Vehicle);
	return state;
}

// an AI driver for the vehicle, as the scenery gives one to a vehicle of its own outside a trainset: in charge of
// the consist, with no timetable, waiting for orders when the vehicle stands and driving on when it rolls
void add_driver(TDynamicObject &Vehicle, int const Cab) {
	Vehicle.MoverParameters->CabOccupied = Cab;
	Vehicle.Controller = AIdriver;
	Vehicle.create_controller(Cab > 0 ? "1" : "2", false);
}

// takes the AI driver off. nothing but the vehicle and the vehicles the driver is in charge of points at a driver
// (the panels, the events, the network and the black box read it from a vehicle each time), so these are cleared
// and the driver deleted. the vehicles in its charge are looked for among all vehicles, not along the couplers:
// a coupler broken under force or in a collision (TMoverParameters::damage_coupler()) parts the train and leaves
// the vehicles parted from it in the driver's charge, as uncoupling (TDynamicObject::Dettach()) doesn't
void remove_driver(TDynamicObject &Vehicle) {
	auto *driver{Vehicle.Mechanik};
	auto &occupied{*Vehicle.MoverParameters};
	auto &controlling{*Vehicle.FindPowered()->MoverParameters};
	if (driver->primary()) {
		// the train this driver was in charge of stops and stays braked
		secure(occupied, controlling);
	}
	else {
		// somebody else drives the train, so only what this one held is let go
		handover::release_ai_controls(occupied, controlling);
		occupied.CabOccupied = 0;
	}
	Vehicle.Mechanik = nullptr;
	auto released{0};
	for (auto *other : simulation::Vehicles.sequence()) {
		if (other->ctOwner == driver) {
			other->ctOwner = nullptr;
			++released;
		}
	}
	WriteLog("Crew: [" + Vehicle.name() + "] the driver was in charge of " + std::to_string(released) + (released == 1 ? " vehicle" : " vehicles"));
	delete driver;
}

} // namespace

void change(request const &Request, TDynamicObject const *Activator) {
	std::string const name{Request.vehicle.empty() && Activator != nullptr ? Activator->name() : Request.vehicle};
	if (name.empty()) {
		ErrorLog("Bad event: \"" + std::string{command_prefix} + "\" names no vehicle and no vehicle set the event off");
		return;
	}
	auto *vehicle{simulation::Vehicles.find(name)};
	if (vehicle == nullptr || false == vehicle->bEnabled) {
		ErrorLog("Bad event: \"" + std::string{command_prefix} + name + "\" can't find the vehicle");
		return;
	}
	if (Request.flags != 0) {
		WriteLog("Crew: [" + name + "] flags " + std::to_string(Request.flags) + " ignored, none is defined yet");
	}
	auto const decision{decide(Request, state_of(*vehicle))};
	WriteLog("Crew: [" + name + "] " + std::string{describe(decision)});
	switch (decision) {
	case action::add_front:
	case action::add_rear:
		add_driver(*vehicle, Request.cab);
		break;
	case action::remove:
		remove_driver(*vehicle);
		break;
	default:
		break;
	}
}

} // namespace crew
