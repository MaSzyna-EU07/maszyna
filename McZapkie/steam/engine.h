/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <cstddef>

#include "McZapkie/steam/parameters.h"
#include "McZapkie/steam/properties.h"
#include "McZapkie/steam/valve_gear.h"

// the engine of a two-cylinder simple-expansion steam locomotive, followed round the crank: the regulator, the
// steam side of the superheater, the steam chests, the four ends of the cylinders, the exhaust passages and the
// blast pipe. every space holding steam is a control volume of a mass and an internal energy, in equilibrium; the
// valves open the ports as the valve gear moves them, so the indicator diagram, the steam used and the torque
// swinging with the cranks are results, not assumptions
//
// units: SI, pressures absolute, temperatures in K, angles in rad
namespace steam::detailed
{

// a space of steam in equilibrium, known by its mass, internal energy and volume
struct control_volume
{
	double mass{0.0};
	double energy{0.0};
	double volume{0.0};
	fluid steam{0.0, 0.0, 1.0, 0.0};
	double specific_volume{0.0};

	// fills the volume with steam of the given pressure and temperature; at or under saturation (0 for short) with
	// wet steam of the given dryness
	void fill(double Pressure, double Temperature, double Dryness = 1.0);
	// works the pressure, temperature and dryness out of the mass, energy and volume
	void update();
};

// what the engine is given for a step
struct engine_input
{
	double regulator{0.0}; // the handle, 0..1; the pilot valve alone on the first part of its way
	double reverser{0.0}; // the nut of the reverser, -1 full reverse gear .. 1 full forward gear
	bool cylinder_cocks{false};
	double boiler_pressure{0.0};
	double steam_volume{0.0}; // specific, of the steam the regulator takes
	double steam_enthalpy{0.0};
	double superheater_temperature{600.0}; // of the metal of the superheater
	double smokebox_pressure{0.0};
};

// what a step gave: the flows, heats and torques are averages over the step, the pressures those at its end
struct engine_output
{
	double torque{0.0}; // N m on the driving axle, mechanism friction taken off, + forwards
	double friction_torque{0.0};
	double rim_force{0.0}; // N at the rims of the coupled wheels
	double indicated_power{0.0}; // W
	double regulator_flow{0.0}; // kg/s out of the boiler
	double cylinder_flow{0.0}; // kg/s from the steam chests into the cylinders
	double nozzle_flow{0.0}; // kg/s out of the blast pipe
	double nozzle_velocity{0.0}; // m/s, of the steam leaving the blast pipe
	double nozzle_enthalpy_flow{0.0}; // W
	double cocks_flow{0.0}; // kg/s through the cylinder cocks and relief valves
	double superheater_heat{0.0}; // W taken from the metal of the superheater
	double wall_heat{0.0}; // W lost by the steam to the cylinder walls
	double chest_pressure{0.0};
	double chest_temperature{0.0};
	double exhaust_pressure{0.0};
	double exhaust_temperature{0.0};
	double exhaust_dryness{1.0};
	double exhaust_volume{0.0}; // specific
	std::array<double, 4> chamber_pressures{}; // right front, right back, left front, left back
	double crank{0.0}; // the angle of the right crank
	double weighshaft{0.0}; // the angle of the lifting arm
	double die_block{0.0}; // m along the link, of the right side
};

class engine
{
  public:
	explicit engine(parameters const &Parameters);

	// a warmed-up engine: the cylinder walls at the given temperature and, with a pressure given, the steam chests
	// filled with steam of that pressure and temperature
	void warm_up(double Walls, double ChestPressure = 0.0, double ChestTemperature = 0.0);
	// advances the engine by the time step at the given angular velocity of the driving wheels (+ forwards), in
	// substeps of half a degree of the crank and 0.3 ms at most
	engine_output step(double Deltatime, double Omega, engine_input const &Input);
	// the air the engine works in, from the weather
	void set_ambient(ambient_parameters const &Ambient);
	// the flow section of the regulator with the steam pipe and the superheater behind it, m²
	double regulator_area(double Regulator) const;

	// the state
	double crank{0.3};
	control_volume chest; // the superheater, the steam pipes and both steam chests
	control_volume exhaust;
	std::array<control_volume, 4> chambers; // right front, right back, left front, left back
	std::array<double, 4> wall_temperature{380.0, 380.0, 380.0, 380.0}; // of the skin of the walls the steam touches
	std::array<double, 2> block_temperature{380.0, 380.0}; // of the cylinder castings

  private:
	// what doesn't change within a step
	struct stepping
	{
		double step;
		double omega;
		double weighshaft;
		double regulator_area;
		double cocks_area; // m², of the open cylinder cocks of one end of a cylinder
		double penetration; // m the heat wave of a revolution gets into the wall
		double piston_speed; // m/s, mean
	};
	// the rates of change of the steam spaces within a substep
	struct rates
	{
		std::array<double, 4> mass{};
		std::array<double, 4> energy{};
		double chest_mass{0.0};
		double chest_energy{0.0};
		double exhaust_mass{0.0};
		double exhaust_energy{0.0};
	};
	// what a step adds up
	struct tally
	{
		double torque{0.0};
		double friction{0.0};
		double work{0.0};
		double regulator{0.0};
		double cylinders{0.0};
		double nozzle{0.0};
		double momentum{0.0};
		double nozzle_enthalpy{0.0};
		double cocks{0.0};
		double superheater_heat{0.0};
		double wall_heat{0.0};
	};
	// a cylinder at a moment
	struct cylinder_state
	{
		std::size_t cylinder;
		double crank;
		double piston;
		double piston_rate;
		port_openings ports;
	};
	struct torques
	{
		double gas{0.0};
		double friction{0.0};
	};

	void set_volumes();
	void admit(stepping const &Stepping, engine_input const &Input, rates &Rates, tally &Tally) const;
	void chamber_flows(stepping const &Stepping, cylinder_state const &Cylinder, std::size_t End, rates &Rates, tally &Tally) const;
	void wall_exchange(stepping const &Stepping, cylinder_state const &Cylinder, std::size_t End, rates &Rates, tally &Tally);
	torques cylinder(stepping const &Stepping, cylinder_state const &Cylinder, rates &Rates, tally &Tally);
	void blast(stepping const &Stepping, double SmokeboxPressure, rates &Rates, tally &Tally) const;
	void integrate(stepping const &Stepping, rates const &Rates);
	void substep(stepping const &Stepping, engine_input const &Input, tally &Tally);

	parameters m_parameters;
	std::array<valve_gear, 2> m_gears; // of the right and of the left side
	double m_mid_gear;
	double m_front_area; // of the piston less its rod, at the front end
	double m_back_area;
	double m_clearance; // m³
	double m_crank_offset; // rad
	double m_superheater_surface; // m², the steam side
	double m_die_block{0.0};
};

} // namespace steam::detailed
