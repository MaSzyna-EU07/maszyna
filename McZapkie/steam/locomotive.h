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

#include "McZapkie/steam/boiler.h"
#include "McZapkie/steam/engine.h"
#include "McZapkie/steam/parameters.h"

// a steam locomotive as the sum of its engine and its boiler, with the auxiliaries and the stores of the tender.
// it stands apart from the vehicle: the game gives it the angular velocity of the driving wheels and gets the force
// at their rims back, and keeps for itself the adhesion, the slip, the resistances and the brakes
//
// the figures given back are in the units of the simulation: bar gauge, °C, kW, N, N m, kg, kg/s, m
namespace steam::detailed
{

// what the driver and the fireman do
struct locomotive_controls
{
	double regulator{0.0}; // 0..1
	double reverser{0.0}; // -1 full reverse gear .. 1 full forward gear
	bool cylinder_cocks{false};
	double blower{0.0};
	double ashpan_dampers{1.0};
	double firehole_door{0.0};
	double firing{0.0}; // kg/s of coal shovelled onto the grate
	double live_steam_injector{0.0}; // 0..1
	double exhaust_steam_injector{0.0};
	double blowdown{0.0}; // kg/s
	double clinkering{0.0}; // kg/s
	double air_pump{0.0}; // 0..1, from the brake
	bool generator{false};
	double train_heating{0.0}; // kg/s of steam
	bool sanding{false};
};

struct locomotive_environment
{
	double relative_speed{0.0}; // m/s of the air over the locomotive, + against it
};

struct locomotive_output
{
	// the force and the power
	double rim_force{0.0}; // N at the rims of the coupled wheels
	double torque{0.0}; // N m on the driving axle
	double friction_torque{0.0}; // N m, of the mechanism
	double indicated_power{0.0}; // kW
	double rim_power{0.0}; // kW
	double crank_angle{0.0}; // rad, of the right crank
	double weighshaft_angle{0.0}; // rad
	double die_block{0.0}; // m
	// the boiler
	double boiler_pressure{0.0}; // bar gauge
	double saturation_temperature{0.0}; // °C
	double water_mass{0.0}; // kg of water and steam
	double water_volume{0.0}; // m³
	double water_level{0.0}; // m over the normal level
	double steam_dryness{0.0}; // of the steam taken from the boiler
	double regulator_flow{0.0}; // kg/s
	double auxiliary_flow{0.0}; // kg/s
	double safety_valve_flow{0.0}; // kg/s
	double feed_flow{0.0}; // kg/s
	bool safety_valves_open{false};
	bool fusible_plug_melted{false};
	double evaporation_power{0.0}; // kW
	double lagging_loss{0.0}; // kW
	// the fire and the draught
	double fuel_on_grate{0.0}; // kg
	double fuel_layer{0.0}; // m
	double clinker{0.0}; // kg
	double burn_rate{0.0}; // kg/s
	double firing_rate{0.0}; // kg/(m² h)
	double excess_air{0.0};
	double unburnt_loss{0.0};
	double fire_power{0.0}; // kW
	double adiabatic_temperature{0.0}; // °C
	double firebox_temperature{0.0}; // °C
	double smokebox_temperature{0.0}; // °C
	double air_flow{0.0}; // kg/s
	double gas_flow{0.0}; // kg/s
	double smokebox_vacuum{0.0}; // Pa
	// the superheater, the steam chests and the exhaust
	double superheater_metal_temperature{0.0}; // °C
	double chest_pressure{0.0}; // bar gauge
	double steam_temperature{0.0}; // °C, in the steam chests
	double exhaust_pressure{0.0}; // bar gauge
	double exhaust_temperature{0.0}; // °C
	double exhaust_dryness{1.0};
	double nozzle_flow{0.0}; // kg/s
	double nozzle_velocity{0.0}; // m/s
	double cocks_flow{0.0}; // kg/s
	double cylinder_wall_loss{0.0}; // kW
	std::array<double, 4> chamber_pressures{}; // bar gauge: right front, right back, left front, left back
	std::array<double, 4> wall_temperatures{}; // °C
	// the stores and the masses
	double tender_water{0.0}; // kg
	double tender_coal{0.0}; // kg
	double sand{0.0}; // kg
	double locomotive_mass{0.0}; // kg
	double tender_mass{0.0}; // kg
};

class locomotive
{
  public:
	explicit locomotive(parameters const &Parameters = {});

	// advances the locomotive by the time step with the driving wheels turning at the given angular velocity (+ forwards)
	locomotive_output step(double Deltatime, double Omega, locomotive_controls const &Controls, locomotive_environment const &Environment);
	// the air the locomotive works in, from the weather
	void set_ambient(ambient_parameters const &Ambient);
	double locomotive_mass() const;
	double tender_mass() const;

	engine steam_engine;
	boiler steam_boiler;
	double tender_water;
	double tender_coal;
	double sand;

  private:
	parameters m_parameters;
};

} // namespace steam::detailed
