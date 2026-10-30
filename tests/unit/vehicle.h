#pragma once

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "McZapkie/MOVER.h"

namespace test {

// reseeds the random number engines of the simulation, as before every test case. a test making several
// vehicles in a loop reseeds before each, so each one is the same whatever comes before it
void reseed_random();

// thrown for a test vehicle definition which can't be used: missing, malformed, or not taken by the loader
class definition_error : public std::runtime_error {
public:
	using std::runtime_error::runtime_error;
};

// definition of a test vehicle: one of the JSON files in tests/unit/data, with the sections and keys
// of the game's vehicle definitions, changed by the test where it needs to:
//   auto heavy = definition::load("wagon").set("Param", "M", 60000).set("Brake", "BrakeValve", "ESt3");
// or one of the game's own vehicles, converted with the files it includes to tests/unit/data/fiz (see the
// README there), which reads to the game's vehicle loader the same as the game's file:
//   auto eu07 = definition::fiz("4e_v1/4e").set("Brake", "MaxBP", 3.8);
// a section of a list keeps its rows under "rows", each a list of values
class definition {
public:
	// tests/unit/data/<Name>.json
	static definition load(std::string const &Name);
	// tests/unit/data/fiz/<Path>.json
	static definition fiz(std::string const &Path);

	// sets the key of the section, adding either one when it's missing
	definition &set(std::string const &Section, std::string const &Key, nlohmann::ordered_json const &Value);
	// drops the whole section, with its rows, or one key of it
	definition &remove(std::string const &Section);
	definition &remove(std::string const &Section, std::string const &Key);

	// the definition in the form the game's vehicle loader reads, the sections in their order
	std::string text() const;

	// the vehicle type of a real definition, its file name in the game; empty for the others
	std::string const &type_name() const { return m_typename; }

private:
	nlohmann::ordered_json m_sections = nlohmann::ordered_json::object();
	std::string m_typename;
};

// conditions the vehicle runs in: straight track of the given friction and gradient (rise/run), under a
// catenary of the given voltage, or without one. the track is of the vehicle's own category (rail, road,
// water), unless the test gives one, and of the standard gauge wide, unless the test gives a road's width. iterations is how many steps of the physics the game makes in a frame:
// the last one moves the vehicle fully, the ones before it (FastComputeMovement) only roughly; the game
// makes as many as it takes to keep them at 0.01 s, in fives with "full physics" set
struct track {
	double friction{0.15};
	double gradient{0.0};
	double voltage{0.0};
	int category{0};
	int iterations{1};
	double width{1.435};
};

// the catenary of the Polish railways [V]
inline constexpr double catenary_3kv{3000.0};

class trace;

// one vehicle created the way TDynamicObject::Init() creates it, from the given definition. Cab is the
// cab with a driver in it, 1 (front) or -1 (rear), as for a "headdriver" or "reardriver" in a scenario;
// 0 for a vehicle nobody drives. the vehicle steps the way TDynamicObject::Update() steps a lone one
class vehicle {
public:
	explicit vehicle(definition const &Definition, double Velocity = 0.0, std::string const &Name = "test", int Cab = 0);

	TMoverParameters &mover() { return *m_mover; }
	TMoverParameters *operator->() { return m_mover.get(); }

	// keeps the brake pipe at the given pressure [bar], as a locomotive coupled to the vehicle would;
	// a negative value leaves the pipe alone
	void hold_brake_pipe(double Pressure) { m_pipepressure = Pressure; }
	// keeps the main reservoir at the given pressure [bar], as a running compressor or a coupled vehicle
	// feeding it would; a negative value leaves it alone
	void hold_main_reservoir(double Pressure) { m_reservoirpressure = Pressure; }

	// advances the physics by one frame of Track.iterations steps of Dt, the way the simulation loop does
	void step(track const &Track = {}, double Dt = 0.01);
	// advances the physics by Seconds in frames of steps of Dt
	void run(double Seconds, track const &Track = {}, double Dt = 0.01);

	// from now on records the vehicle in the trace after every step, see golden.h
	void record(trace &Trace) { m_trace = &Trace; }

	// the parts of a step, which a consist makes for all its vehicles in turn, as vehicle_table::update()
	// does: the test's conditions and the constants, the forces, the movement (returning the distance)
	void begin_step();
	void forces(double Dt);
	double fast_move(track const &Track, double Dt, TLocation const &Location);
	double move(track const &Track, double Dt, TLocation const &Location, double Trainpipe);

	// vehicle speed in kilometres per hour, negative when moving backwards
	double speed() const { return m_mover->V * 3.6; }

private:
	std::unique_ptr<TMoverParameters> m_mover;
	int m_cab{0};
	double m_pipepressure{-1.0};
	double m_reservoirpressure{-1.0};
	trace *m_trace{nullptr};
	double m_time{0.0}; // of the simulation, since the vehicle got created
};

} // namespace test
