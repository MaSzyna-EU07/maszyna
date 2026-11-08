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

// the odometer of a vehicle: the total distance it has run, kept between the sessions (eu07/maszyna#72).
// the analog counters in the cab (distcounter:) and the python screens (state["odometer_total"]) show
// this one state, instead of the counters starting from their .mmd offset and the screens each keeping
// their own files
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

// the saved odometer states of the vehicles, km by vehicle name
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

// state kept in the file of the python screens before (<vehicle directory>/kilometry/<name>_przebieg.txt,
// where the screens dropped the last character of the vehicle name), when it holds a number
std::optional<double> read_legacy(std::istream &Input);
std::optional<double> legacy_state(std::filesystem::path const &Vehicledirectory, std::string const &Vehicle);

// made up state of a vehicle nothing is known about: a number from the name, in [10 000, 500 000) km,
// the same each session until the vehicle runs and gets its state saved
double guessed_state(std::string const &Vehicle);

// state a vehicle starts the session with: the saved one, else the legacy one, else a guessed one. a saved state
// counted from a guessed one stays guessed, so the cab's counter can still replace the made up part of it
reading starting_state(store const &States, std::string const &Vehicle, std::filesystem::path const &Vehicledirectory);

// state the cab's counter with offset Offset and scale Scale shows, once its offset no longer applies
// (the counter shows the odometer). nothing for a counter without an offset
std::optional<double> counter_seed(float Offset, float Scale);

// the odometer store of the simulation, read from the user's settings directory on the first use
store &states();
std::filesystem::path file();
// writes the store to the file when anything was recorded since, merged with what other simulations saved in the
// file meanwhile. it's written to a file of this process aside first, then put in place. returns false when the
// write failed
bool save(store &States, std::filesystem::path const &File);
// saves the store of the simulation to its file
bool save();

} // namespace odometer
