/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "simulation/simulationsnapshot.h"

#include <string>
#include <vector>

class TDynamicObject;
class TTrack;
class basic_event;

// the simulation snapshot of the running game: taken from the scenery as it is, and put over a scenery just loaded.
// the text of the file is in simulationsnapshot.h; the parts kept by a vehicle and by its driver are with those classes
// (TDynamicObject::snapshot_save() and the like, defined in simulationsnapshotvehicle.cpp and simulationsnapshotdriver.cpp)
namespace simulation::snapshot {

// the state of the simulation as a document
document capture();

// how putting a snapshot over the scenery went
struct apply_summary {
	int entries{0}; // put in place
	int skipped{0}; // of the snapshot, with nothing in the scenery to put them on
	std::vector<std::string> notes; // what went other than the snapshot has it, for the user to know of
	std::vector<std::string> remarks; // what was done on the way, for the log
	bool refused{false}; // the snapshot wasn't used at all
};

// puts the state of the document over the scenery, which is expected to be loaded and initialized, with the simulation
// not yet running. what the snapshot doesn't hold stays as the scenery made it
apply_summary apply(document const &Document);

// writes a snapshot of the simulation to the file. returns: true on success
bool save(std::string const &File);

// the snapshots asked for on the command line (parse_command_line()): one to load when the simulation starts, and the
// ones to write when its time comes
void request(command_line const &Requests);
// the seed the snapshot to be loaded was taken with, if there's one to load and it tells: the scenery loads the same
// way again only with the seed it was loaded with then
std::optional<std::uint32_t> requested_seed();
// the vehicle the player was in when the snapshot to be loaded was taken, if it tells
std::string requested_player_vehicle();
// called when the simulation is about to start: loads the snapshot asked for
void start();
// called at the end of each frame of the simulation: writes the snapshots due
void update();

// names of the things of the scenery, as the entries of a snapshot give them (reference())
std::string track_reference(TTrack const *Track);
TTrack *find_track(std::string const &Reference);
std::string vehicle_reference(TDynamicObject const *Vehicle);
TDynamicObject *find_vehicle(std::string const &Reference);
// an event is named by its name and, among the events sharing that name, its place
int event_sibling(basic_event const *Event);
basic_event *find_event(std::string const &Name, int Sibling);

} // namespace simulation::snapshot
