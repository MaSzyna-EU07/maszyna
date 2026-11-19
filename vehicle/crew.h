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
#include <string_view>

class TMoverParameters;

// a driver put in a vehicle, or taken off it, while the scenario runs (eu07/maszyna#510): the command of a
// putvalues event
//   event <name> putvalues <delay> none 0 0 0 Crew:<vehicle> <cab> <flags> endevent
// with the name of the vehicle in the text, so the event works whatever sets it off (a key, an event
// launcher), the cab in the first value: 1 an AI driver in cab A, -1 in cab B, 0 nobody, and the second value
// reserved (0). "Crew:" with no name is for the vehicle which set the event off.
// here are the parts which need no vehicle of the game: reading the command, deciding what to do about it,
// and the state a train is left in when its driver goes. the game's side of it is in crewchange.h
namespace crew {

// what the text of the command starts with, in this very case (the commands of the events are case sensitive)
inline constexpr std::string_view command_prefix{"Crew:"};

// whether the text of a putvalues event is the crew command
bool is_command(std::string_view Text);

// the command as the event gives it
struct request {
	std::string vehicle; // as the scenery keeps the names, in lower case; empty: the vehicle which set the event off
	int cab{0}; // 1 a driver in cab A (the front one), -1 in cab B, 0 nobody
	bool cab_valid{false}; // the value was one of the three
	int flags{0}; // reserved, nothing reads it yet
};

// reads the command from the text of the event and its two values
request parse(std::string_view Text, double Cab, double Flags);

// what the command finds in the vehicle
struct vehicle_state {
	bool has_cab{false}; // the type of the vehicle has a driver's cab; a road vehicle or a boat counts as having one
	bool has_driver{false};
	bool driver_is_ai{false}; // the driver it has is the AI, not a user
	bool occupied{false}; // a user sits in the vehicle, whoever drives it
	bool consist_driven{false}; // another vehicle of the consist has the driver in charge of it
};

enum class action {
	add_front, // an AI driver goes into cab A
	add_rear, // an AI driver goes into cab B
	remove, // the AI driver is taken off
	nothing, // asked to take the driver off a vehicle which has none
	refuse_cab, // the cab asked for is none of 1, -1, 0
	refuse_user, // a user is in the vehicle, or drives it
	refuse_crewed, // asked for a driver where there is one already
	refuse_no_cab, // asked for a driver in a vehicle with no cab
	refuse_consist_driven // asked for a driver in a consist which another vehicle's driver is in charge of
};

// what to do about the command. a user in the vehicle is never touched: neither replaced nor left without
// the driver the cab of the game works with
action decide(request const &Request, vehicle_state const &State);

// whether the action changes anything
bool changes_crew(action Action);

// the action in words, for the log
std::string_view describe(action Action);

// the state a train is left in when the driver in charge of it goes: what the AI held is let go (as when it
// hands over to a user, see handover.h), the power is off, the driver's brake valve is in the emergency position,
// so a train on the move stops and stays braked, and the cab is neither active nor occupied. the engine, the
// pantographs and the lights stay as they were. Occupied is the vehicle of the driver, Controlling the one whose
// motors it drove (the AI's mvOccupied and mvControlling; the same one in a locomotive)
void secure(TMoverParameters &Occupied, TMoverParameters &Controlling);

} // namespace crew
