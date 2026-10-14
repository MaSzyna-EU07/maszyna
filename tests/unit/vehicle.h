#pragma once

#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include "McZapkie/MOVER.h"

namespace test {

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

	// vehicle speed [km/h], signed
	double speed() const { return m_mover->V * 3.6; }

private:
	std::unique_ptr<TMoverParameters> m_mover;
	double m_pipepressure{-1.0};
};

} // namespace test
