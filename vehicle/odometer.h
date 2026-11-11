/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <filesystem>
#include <iosfwd>
#include <map>
#include <optional>
#include <string>

#include "vehicle/odometerreading.h"

// the odometer of a vehicle: the total distance it has run, kept between the sessions (eu07/maszyna#72).
// the analog counters in the cab (distcounter:) and the python screens (state["odometer_total"]) show
// this one state, instead of the counters starting from their .mmd offset and the screens each keeping
// their own files
namespace odometer {

// the saved odometer states of the vehicles, km by vehicle name. the name is the one the scenario gives the vehicle
// (the name of its node, TDynamicObject::name(), e.g. EU07-424), the only identity a vehicle has between the sessions:
// the vehicles of the same name in any scenarios share one state, whatever their type, and a vehicle the scenario
// names otherwise starts over, from the legacy or a made up state. the sceneries name the real vehicles by their
// numbers, so it's the same vehicle as a rule
class store {
public:
	// reads the states written by write(). lines "<km> <vehicle name>", "~<km> <vehicle name>" for a state counted
	// from a made up one, '#' starts a comment line, a malformed line is skipped. returns the number of states read
	std::size_t read(std::istream &Input);
	void write(std::ostream &Output) const;

	std::optional<double> find(std::string const &Vehicle) const;
	// the state of the vehicle is counted from a made up one, which the cab's counter can still replace
	bool guessed(std::string const &Vehicle) const;
	// records the state of a vehicle. one not in the store yet gets in only once it has run, so the vehicles
	// standing in the sceneries don't fill the file. a recorded state is replaced only by a newer one: a larger
	// one, or a known one in place of a guessed one
	void record(std::string const &Vehicle, reading const &Odometer);
	// takes the states of the other store (the file, as other simulations have saved it since) newer than these.
	// the odometers only count up, so of two states of a vehicle the larger is the newer one. they aren't added
	// up: two simulations on one machine (multiplayer) count the same runs of the same vehicles
	void merge(store const &Other);

	// there are states recorded since they were last saved
	bool modified() const { return m_modified; }
	void saved() { m_modified = false; }
	std::size_t size() const { return m_states.size(); }

private:
	struct state {
		double km{0.0};
		bool guessed{false};
	};
	// the first state is newer than the second one
	static bool newer(state const &Left, state const &Right);

	std::map<std::string, state> m_states;
	bool m_modified{false};
};

// the vehicle is one the store keeps: one with a driver's cab, which can show its odometer on the cab's counter or a
// python screen. that's
// - a vehicle whose type has a cab (Drivercab, TMoverParameters::HasDriverCab(): it has the driver's brake valve). that
//   includes the locomotives and units the AI runs before a player's cab loads them, their made up states stay
//   replaceable by the cab's counter
// - a vehicle whose cab showed the odometer in this session (reading::shown): the cab was loaded for a player, or a
//   cab's counter was bound to it. the type alone misses the vehicles driven without a driver's brake valve (the DL2
//   draisine, with a counter in its cab) and the motor cars the counter of a unit's cab counts for. such a vehicle is
//   recorded in the sessions a player's cab shows it; in the others its saved state stays as it is
// - a vehicle whose state a cab's counter seeded (origin::counter), which is the same thing, told by the state
// the wagons, which make most of the AI's trains, aren't
bool kept(bool Drivercab, reading const &Odometer);
// records the state of a vehicle in the store when it's one the store keeps (kept()). the others aren't recorded; the
// states the store already holds for any of them stay as they are
void record(store &States, std::string const &Vehicle, bool Drivercab, reading const &Odometer);

// state kept in the file of the python screens before, when it holds a number. the screens opened the file in the
// directory of their script (lookup_path), and two kinds of them kept one:
// - the renderers of the units, next to the .mmd (elf_v1: EN62, EN76, EN96, SKM; impuls_v1: EN63), in
//   <vehicle directory>/kilometry/<name without its last character>_przebieg.txt (state['name'][:-1]). the cars of
//   these units are named after the unit and the letter of the car (EN76-022a .. EN76-022d, ED78-019A .. ED78-019D),
//   so the cab cars at both ends of a unit share one file, the unit's odometer
// - the shared screens in scripts/ (koliber_hasler_elektroniczny, koliber_unicontrols_tramex[_ic]), in
//   <scripts directory>/kilometry/<name>_przebieg.txt, the whole name
std::optional<double> read_legacy(std::istream &Input);
std::optional<double> legacy_state(std::filesystem::path const &Vehicledirectory, std::string const &Vehicle,
                                   std::filesystem::path const &Scriptsdirectory = "scripts");

// made up state of a vehicle nothing is known about: a number from the name, in [10 000, 500 000) km,
// the same each session until the vehicle runs and gets its state saved
double guessed_state(std::string const &Vehicle);

// state a vehicle starts the session with: the saved one, else the legacy one, else a guessed one. a saved state
// counted from a guessed one stays guessed, so the cab's counter can still replace the made up part of it
reading starting_state(store const &States, std::string const &Vehicle, std::filesystem::path const &Vehicledirectory,
                       std::filesystem::path const &Scriptsdirectory = "scripts");

// state the cab's counter with offset Offset and scale Scale shows, once its offset no longer applies
// (the counter shows the odometer). nothing for a counter without an offset
std::optional<double> counter_seed(float Offset, float Scale);

// the store of the simulation is the player's file: false in a test run (--test, Global.test.enabled), whose vehicles
// neither start from the player's states nor change them
bool persistent();
// the odometer store of the simulation, read from the user's settings directory on the first use. in a test run an
// empty one, not read from anywhere
store &states();
std::filesystem::path file();
// writes the store to the file when anything was recorded since, merged with what other simulations saved in the
// file meanwhile. it's written to a file of this process aside first, then put in place. returns false when the
// write failed
bool save(store &States, std::filesystem::path const &File);
// saves the store of the simulation to its file. a test run doesn't save it
bool save();

} // namespace odometer
