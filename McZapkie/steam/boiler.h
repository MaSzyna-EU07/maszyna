/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "McZapkie/steam/parameters.h"
#include "McZapkie/steam/properties.h"

// the boiler of a steam locomotive: the fire on the grate, the heat of the gases through the firebox, the tubes
// and the flues with the superheater in them, the draught of the blast pipe and the water and steam space with
// its safety valves, its level and its fusible plug
//
// units: SI, pressures absolute, temperatures in K, the heat in W
namespace steam::detailed
{

// what the fire and the gases do for the given flow of air through the grate
struct combustion
{
	double layer{0.0}; // m, the thickness of the fuel bed
	double burnt{0.0}; // kg/s of coal leaving the bed
	double consumed{0.0}; // kg/s of it burnt, the rest being carried off unburnt
	double firing_rate{0.0}; // kg/(m² h)
	double unburnt_loss{0.0}; // of the coal burnt
	double excess_air{0.0};
	double monoxide_loss{0.0}; // of the heat, from burning short of air
	double fire_heat{0.0}; // W
	double gas_flow{0.0}; // kg/s
	double adiabatic_temperature{0.0};
	double firebox_exit_temperature{0.0};
	double firebox_heat{0.0}; // W to the water through the firebox walls
	double tube_heat{0.0}; // W through the tubes
	double flue_heat{0.0}; // W through the walls of the flues
	double superheater_heat{0.0}; // W to the metal of the superheater
	double smokebox_temperature{0.0};
	double tube_pressure_drop{0.0}; // Pa
	double tube_share{0.0}; // of the gases through the tubes, the rest in the flues
};

struct boiler_input
{
	double regulator_flow{0.0}; // kg/s of steam taken by the engine
	double auxiliary_flow{0.0}; // kg/s taken by the air pump, the generator and train heating
	double feed_flow{0.0}; // kg/s of water from the injectors
	double feed_heating{0.0}; // K, the exhaust steam injector warming it
	double nozzle_flow{0.0}; // kg/s, of steam out of the blast pipe
	double nozzle_velocity{0.0};
	double nozzle_volume{1.6}; // specific, of that steam
	double blower{0.0}; // 0..1
	double dampers{1.0}; // of the ashpan, 0..1
	double door{0.0}; // of the firehole, 0..1
	double firing{0.0}; // kg/s of coal shovelled
	double blowdown{0.0}; // kg/s of water let out of the boiler
	double clinkering{0.0}; // kg/s of clinker raked out
	double relative_speed{0.0}; // m/s of the air over the boiler
};

struct boiler_output
{
	combustion fire;
	double air_flow{0.0}; // kg/s through the grate
	double blower_flow{0.0}; // kg/s of steam
	double safety_valve_flow{0.0}; // kg/s
	double dryness_taken{1.0}; // of the steam leaving the boiler
	double enthalpy_taken{0.0}; // J/kg of it
	double heat_to_water{0.0}; // W
	double heat_lost{0.0}; // W through the lagging
	double level{0.0}; // m over the normal level
	double smokebox_vacuum{0.0}; // Pa
};

class boiler
{
  public:
	explicit boiler(parameters const &Parameters);

	// the boiler brought to the given absolute pressure with the given volume of water in it
	void set_state(double Pressure, double WaterVolume);
	boiler_output step(double Deltatime, boiler_input const &Input);
	// the air the boiler burns its fuel in and loses its heat to, from the weather
	void set_ambient(ambient_parameters const &Ambient);
	// what the fire gives for the air flow, the state of the fire and the boiler as they are
	combustion fire(double AirFlow, double Door) const;
	// the flow of air through the grate the draught of the given exhaust keeps up, kg/s
	double draught_flow(boiler_input const &Input, double BlowerFlow) const;
	// the vacuum in the smokebox needed to draw the given flow of air through the grate and the flues, Pa
	double resistance(double AirFlow, double Dampers, double TubePressureDrop) const;
	// the vacuum the exhaust steam makes in the throat of the chimney, Pa
	double ejector_vacuum(double NozzleFlow, double NozzleVelocity, double NozzleVolume, double BlowerFlow, double GasFlow, double GasTemperature) const;

	// the state
	double mass{0.0}; // of water and steam, kg
	double energy{0.0}; // J, of them and of the steel heated with them
	double pressure{0.0}; // Pa absolute
	double water_volume{0.0};
	double dryness{0.0}; // of the contents
	saturated line{};
	double fuel_mass{0.0}; // kg on the grate
	double clinker_mass{0.0};
	double superheater_temperature{0.0}; // K, of its metal
	double air_flow{0.0}; // kg/s, following the draught
	bool safety_valves_open{false};
	bool fusible_plug_melted{false};
	boiler_output last{};

	double carbon_air() const; // kg of air a kg of coal needs
	double heat_capacity_superheater() const;
	double working_pressure() const; // Pa gauge

  private:
	void equilibrium();

	parameters m_parameters;
	double m_air_need;
};

} // namespace steam::detailed
