/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "McZapkie/derailment.h"
#include "world/TrackUneven.h"

#include <array>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <vector>

// the check of the geometry of the rail tracks of a scenery (physics.trackcheck yes in eu07.ini): what the criteria of
// derailment (McZapkie/derailment.h) and the plain rules of the track find suspicious, to the log once the scenery
// loaded, so a run can tell a faulty scenery from a faulty simulation. one line for a track with all it found
// ("trackcheck: <name>: ..."), and a summary with the count of each kind. the damage flags are only counted: sceneries
// set them for the jolts of a worn track (damage 31 on every track of line 107). a curve without a speed limit of its
// own is no defect (the drivers go by the signals and the timetable there, the tram loops of a town have none), it is
// only told how fast it takes the reference vehicles, apart from the defects. the rail tracks of trams have no category
// of their own (a tram is a vehicle of the category of the trains, on a track of theirs), so they are checked alike
namespace track_check {

// a rail track as the check sees it
struct track {
	std::string name;
	double velocity{-1.0}; // the speed limit of the track [km/h], negative when it gives none
	double radius{0.0}; // the tightest radius of its curve [m], 0 on a straight one
	std::array<double, 2> cant{0.0, 0.0}; // at its ends [mm], positive raising the right rail
	double gauge{1.435}; // [m]
	int damage{0}; // dtrack_* flags
	bool plain{true}; // a plain track, its speed and curve checked; a switch or a crossing has the paths and the
	                  // speeds of its position, only its unevenness and its damage are
	track_uneven::attribute uneven;
};

// a vehicle the speed limit of a curve is checked for
struct reference {
	std::string name;
	derailment::vehicle_state vehicle;
};
// the default vehicles of derailment.h: an EU07 (4 axles, 80 t, 15.9 m, bogies of 3 m, the centre of gravity of its
// 4.35 m high body) and a 4-axle coach (40 t, 24.5 m, bogies of 2.5 m, 4.05 m high), alone, without a .fiz
std::vector<reference> const &default_references();

struct settings {
	// a curve without a speed limit is told of when it takes the reference vehicles slower than this [km/h]: the top
	// speed of the EU07, the trains the default vehicles make up run no faster
	double unlimited_velocity{125.0};
	double most_cant{160.0}; // [mm], 150 the most of the Polish rules (Id-1), and a margin
	double straight_cant{5.0}; // the cant both ends of a straight track have, past which it's cant without a curve [mm]
	double tight_radius{40.0}; // a curve tighter than this [m]
	double tight_velocity{40.0}; // with a speed limit over this [km/h]
	double most_uneven{30.0}; // the value, up, side and dip of the unevenness [mm]: Id-14 lets the poorest track 28 mm
	double most_cross{25.0}; // the cross level [mm]: 20 mm in class C of Id-14
};

enum class kind {
	overturning, // the speed limit is over the speed a reference vehicle overturns at
	climbing, // or over the one its flange climbs the rail at
	cant_on_straight, // both ends of a straight track have the same cant
	cant_excess, // over the most cant
	tight_curve, // a curve tighter than the tight radius with a speed limit over the tight velocity
	uneven, // the unevenness out of the plausible range
	unlimited_curve, // no defect: a curve without a speed limit, which takes the reference vehicles slower than the
	                 // unlimited velocity
	count_
};
std::string_view name_of(kind Kind);
// whether the kind is a defect of the scenery, rather than a note
bool is_defect(kind Kind);

// what the check found on a track
struct findings {
	std::vector<kind> kinds;
	std::string text; // what the log line tells after the name of the track, empty when nothing
	double unlimited_speed{0.0}; // the speed a curve without a speed limit takes the reference vehicles at [km/h], 0 when
	                             // not told of
	bool empty() const { return kinds.empty(); }
	bool defective() const; // has a kind which is a defect
};
findings check(track const &Track, std::vector<reference> const &References = default_references(), settings const &Settings = {});

struct summary {
	int tracks{0}; // checked
	int suspicious{0}; // with a defect
	int unlimited{0}; // curves without a speed limit told of, defects or not
	double slowest{0.0}; // the lowest speed one of them takes the reference vehicles at [km/h], 0 without any
	std::array<int, static_cast<std::size_t>(kind::count_)> counts{};
	std::map<int, int> damage; // the tracks of each value of the damage flags, none for 0
	int count(kind const Kind) const { return counts[static_cast<std::size_t>(Kind)]; }
};

using sink = std::function<void(std::string const &)>;
// checks the tracks, logs a line for each one found suspicious and the summary
summary scan(std::vector<track> const &Tracks, sink const &Log, std::vector<reference> const &References = default_references(), settings const &Settings = {});

} // namespace track_check
