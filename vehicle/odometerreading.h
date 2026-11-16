/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// the odometer of a vehicle (eu07/maszyna#72), the part the vehicle physics (TMoverParameters::Odometer) keeps: plain
// data, without the store and its files, which are in vehicle/odometer.h
namespace odometer {

// where the state of a vehicle came from
enum class origin {
	saved, // the odometer store of the earlier sessions
	legacy, // the file a python screen kept it in before
	counter, // the offset of the cab's counter, which used to show it on top of the distance run in the session
	guessed // counted from a made up state, which the cab's counter can still replace, also once it was saved
};

// odometer of one vehicle
struct reading {
	double km{0.0}; // the state, km
	double start{0.0}; // the state at the start of the session, km
	origin source{origin::guessed};
	double guess{0.0}; // the made up state a guessed one counts from, km
	// a cab shows the state in this session: a cab of the vehicle was loaded for a player (its python screens can
	// show it), or a cab's counter was bound to it. such a vehicle has a cab, whatever its type tells
	bool shown{false};

	// sets the state the vehicle starts the session with
	void begin(double Km, origin Source);
	// replaces the made up state a guessed one counts from with the one the cab's counter showed at the start,
	// the distance run on top of it kept, in this session and the earlier ones. a known state stays
	void seed(double Km);
	// counts the distance run, in metres either way
	void run(double Metres);
	// the vehicle has run in this session
	bool moved() const;
};

} // namespace odometer
