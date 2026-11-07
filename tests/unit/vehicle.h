#pragma once

#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "McZapkie/MOVER.h"

namespace test {

// thrown for a test vehicle definition which can't be used: missing, malformed, or not taken by the loader
class definition_error : public std::runtime_error {
public:
	using std::runtime_error::runtime_error;
};

// definition of a test vehicle: one of the JSON files in tests/unit/data, with the sections and keys
// of the game's vehicle definitions, changed by the test where it needs to:
//   auto heavy = definition::load("wagon").set("Param", "M", 60000).set("Brake", "BrakeValve", "ESt3");
class definition {
public:
	// tests/unit/data/<Name>.json
	static definition load(std::string const &Name);

	// sets the key of the section, adding either one when it's missing
	definition &set(std::string const &Section, std::string const &Key, nlohmann::ordered_json const &Value);
	// drops the whole section, or one key of it
	definition &remove(std::string const &Section);
	definition &remove(std::string const &Section, std::string const &Key);

	// the definition in the form the game's vehicle loader reads
	std::string text() const;

private:
	nlohmann::ordered_json m_sections;
};

// conditions the vehicle runs in: straight track of the given friction and gradient (rise/run)
struct track {
	double friction{0.15};
	double gradient{0.0};
};

// one vehicle created the way TDynamicObject::Init() creates it, from the given definition
class vehicle {
public:
	explicit vehicle(definition const &Definition, double Velocity = 0.0, std::string const &Name = "test");

	TMoverParameters &mover() { return *m_mover; }
	TMoverParameters *operator->() { return m_mover.get(); }

	// keeps the brake pipe at the given pressure [bar], as a locomotive coupled to the vehicle would;
	// a negative value leaves the pipe alone
	void hold_brake_pipe(double Pressure) { m_pipepressure = Pressure; }

	// advances the physics by one step of Dt, the way the simulation loop does
	void step(track const &Track = {}, double Dt = 0.01);
	// advances the physics by Seconds in steps of Dt
	void run(double Seconds, track const &Track = {}, double Dt = 0.01);

	// vehicle speed in kilometres per hour, negative when moving backwards
	double speed() const { return m_mover->V * 3.6; }

private:
	friend class pair;

	// the two halves of a step: the forces, taken by all the vehicles in the scene first, then the movement,
	// the vehicle placed at Location. returns the distance travelled [m], positive forwards
	void compute_forces(double Dt);
	double compute_movement(track const &Track, double Dt, TLocation const &Location);

	std::unique_ptr<TMoverParameters> m_mover;
	double m_pipepressure{-1.0};
};

// two vehicles on a straight track, the FirstEnd of the first one facing the SecondEnd of the second one Gap metres
// away (from coupler to coupler), with nothing else around. a step runs them the way the simulation runs the
// vehicles of a scene (vehicle_table::update()): each one finds its neighbour, then both take the forces, the
// coupler and buffer ones among them (CouplerForce()), then both move and handle the collisions (CollisionDetect()).
// they couple and part on their own, as the couplers let them; the speed of each runs along its own axis
//   test::pair pair{moving, end::front, standing, end::rear, 1.0}; // 1 m apart
//   pair.run(5.0); // for 5 s
//   CHECK(pair.coupled());
class pair {
public:
	pair(vehicle &First, int FirstEnd, vehicle &Second, int SecondEnd, double Gap);
	~pair();
	pair(pair const &) = delete;
	pair &operator=(pair const &) = delete;
	// the vehicles point at the stand-ins of the pair, which stay where they are
	pair(pair &&) = delete;
	pair &operator=(pair &&) = delete;

	// advances both vehicles by one step of Dt
	void step(track const &Track = {}, double Dt = 0.01);
	// advances both vehicles by Seconds in steps of Dt
	void run(double Seconds, track const &Track = {}, double Dt = 0.01);
	// distance between the facing couplers [m], negative while the buffers are pressed in
	double gap() const;
	// type of the coupling between the vehicles, coupling::faux when they aren't coupled
	int coupling() const;
	bool coupled() const { return coupling() != coupling::faux; }

private:
	struct placement {
		vehicle *owner{nullptr};
		int end{0}; // the end facing the other vehicle
		double position{0.0}; // of the middle of the vehicle along the track [m]
		double heading{1.0}; // +1 when the vehicle runs forwards towards the growing position, -1 otherwise
		std::vector<std::byte> standin; // what the other vehicle sees as this one, see vehicle.cpp
	};

	void find_neighbours();

	std::array<placement, 2> m_vehicles;
};

} // namespace test
