/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "McZapkie/steam/locomotive.h"

#include <algorithm>

namespace steam::detailed
{

namespace
{

constexpr double exhaust_injector_warming{55.0}; // K the exhaust steam injector warms its water
constexpr double exhaust_injector_steam{0.085}; // of its water flow, the steam it takes from the blast pipe
constexpr double exhaust_injector_least_flow{0.5}; // kg/s of exhaust steam under which it doesn't work
constexpr double to_bar{1.0 / bar};

// what the injectors take and give
struct feed_water
{
	double flow{0.0};
	double warming{0.0};
	double steam_left{0.0}; // of the exhaust steam, to the blast pipe
};

feed_water feed(auxiliary_parameters const &Auxiliaries, locomotive_controls const &Controls, double const TenderWater, double const NozzleFlow)
{
	feed_water result;
	auto const exhaust{Auxiliaries.injector_flow[1] * Controls.exhaust_steam_injector};
	result.flow = TenderWater > 0.0 ? Auxiliaries.injector_flow[0] * Controls.live_steam_injector + exhaust : 0.0;
	result.warming = result.flow > 0.0 && NozzleFlow > exhaust_injector_least_flow ? exhaust_injector_warming * exhaust / result.flow : 0.0;
	result.steam_left = std::max(NozzleFlow - (result.warming > 0.0 ? exhaust_injector_steam * exhaust : 0.0), 0.0);
	return result;
}

// the steam taken by the air pump, the generator and the train heating
double auxiliary_steam(auxiliary_parameters const &Auxiliaries, locomotive_controls const &Controls)
{
	return Auxiliaries.air_pump_flow * Controls.air_pump + (Controls.generator ? Auxiliaries.generator_flow : 0.0) + Controls.train_heating;
}

} // namespace

locomotive::locomotive(parameters const &Parameters)
    : steam_engine{Parameters}, steam_boiler{Parameters}, tender_water{Parameters.auxiliaries.tender_water}, tender_coal{Parameters.auxiliaries.tender_coal},
      sand{Parameters.auxiliaries.sand_capacity}, m_parameters{Parameters}
{
	steam_engine.warm_up(steam_boiler.line.temperature);
}

void locomotive::set_ambient(ambient_parameters const &Ambient)
{
	m_parameters.ambient = Ambient;
	steam_engine.set_ambient(Ambient);
	steam_boiler.set_ambient(Ambient);
}

double locomotive::locomotive_mass() const
{
	auto const &auxiliaries{m_parameters.auxiliaries};
	return auxiliaries.empty_mass + steam_boiler.mass + steam_boiler.fuel_mass + steam_boiler.clinker_mass + sand + auxiliaries.crew_mass;
}

double locomotive::tender_mass() const
{
	return m_parameters.auxiliaries.tender_empty_mass + tender_water + tender_coal;
}

locomotive_output locomotive::step(double const Deltatime, double const Omega, locomotive_controls const &Controls, locomotive_environment const &Environment)
{
	auto &boiler{steam_boiler};
	auto const &auxiliaries{m_parameters.auxiliaries};
	auto const &ambient{m_parameters.ambient};
	// the engine takes the steam of the boiler as it leaves it, wet by what the steam carries over
	auto const &last{boiler.last};
	engine_input drive;
	drive.regulator = Controls.regulator;
	drive.reverser = Controls.reverser;
	drive.cylinder_cocks = Controls.cylinder_cocks;
	drive.boiler_pressure = boiler.pressure;
	drive.steam_volume = boiler.line.liquid_volume + last.dryness_taken * (boiler.line.vapour_volume - boiler.line.liquid_volume);
	drive.steam_enthalpy = last.enthalpy_taken;
	drive.superheater_temperature = boiler.superheater_temperature;
	drive.smokebox_pressure = ambient.pressure - last.smokebox_vacuum;
	auto const engine_result{steam_engine.step(Deltatime, Omega, drive)};
	// the injectors and the auxiliaries
	auto const water{feed(auxiliaries, Controls, tender_water, engine_result.nozzle_flow)};
	auto const auxiliary{auxiliary_steam(auxiliaries, Controls)};
	// the boiler
	boiler_input fire;
	fire.regulator_flow = engine_result.regulator_flow;
	fire.auxiliary_flow = auxiliary;
	fire.feed_flow = water.flow;
	fire.feed_heating = water.warming;
	fire.nozzle_flow = water.steam_left;
	fire.nozzle_velocity = engine_result.nozzle_velocity;
	fire.nozzle_volume = engine_result.exhaust_volume;
	fire.blower = Controls.blower;
	fire.dampers = Controls.ashpan_dampers;
	fire.door = Controls.firehole_door;
	fire.firing = Controls.firing;
	fire.blowdown = Controls.blowdown;
	fire.clinkering = Controls.clinkering;
	fire.relative_speed = Environment.relative_speed;
	auto const boiler_result{boiler.step(Deltatime, fire)};
	// the superheater's metal gets the heat of the gases and gives that to the steam
	boiler.superheater_temperature += (boiler_result.fire.superheater_heat - engine_result.superheater_heat) * Deltatime / boiler.heat_capacity_superheater();
	// the stores
	tender_water = std::max(tender_water - water.flow * Deltatime, 0.0);
	tender_coal = std::max(tender_coal - Controls.firing * Deltatime, 0.0);
	if (Controls.sanding)
	{
		sand = std::max(sand - auxiliaries.sanding_rate * Deltatime, 0.0);
	}
	locomotive_output output;
	output.rim_force = engine_result.rim_force;
	output.torque = engine_result.torque;
	output.friction_torque = engine_result.friction_torque;
	output.indicated_power = engine_result.indicated_power / 1.0e3;
	output.rim_power = engine_result.torque * Omega / 1.0e3;
	output.crank_angle = engine_result.crank;
	output.weighshaft_angle = engine_result.weighshaft;
	output.die_block = engine_result.die_block;
	output.boiler_pressure = (boiler.pressure - ambient.pressure) * to_bar;
	output.saturation_temperature = boiler.line.temperature - kelvin;
	output.water_mass = boiler.mass;
	output.water_volume = boiler.water_volume;
	output.water_level = boiler_result.level;
	output.steam_dryness = boiler_result.dryness_taken;
	output.regulator_flow = engine_result.regulator_flow;
	output.auxiliary_flow = auxiliary + boiler_result.blower_flow;
	output.safety_valve_flow = boiler_result.safety_valve_flow;
	output.feed_flow = water.flow;
	output.safety_valves_open = boiler.safety_valves_open;
	output.fusible_plug_melted = boiler.fusible_plug_melted;
	output.evaporation_power = boiler_result.heat_to_water / 1.0e3;
	output.lagging_loss = boiler_result.heat_lost / 1.0e3;
	output.fuel_on_grate = boiler.fuel_mass;
	output.fuel_layer = boiler_result.fire.layer;
	output.clinker = boiler.clinker_mass;
	output.burn_rate = boiler_result.fire.burnt;
	output.firing_rate = boiler_result.fire.firing_rate;
	output.excess_air = boiler_result.fire.excess_air;
	output.unburnt_loss = boiler_result.fire.unburnt_loss;
	output.fire_power = boiler_result.fire.fire_heat / 1.0e3;
	output.adiabatic_temperature = boiler_result.fire.adiabatic_temperature - kelvin;
	output.firebox_temperature = boiler_result.fire.firebox_exit_temperature - kelvin;
	output.smokebox_temperature = boiler_result.fire.smokebox_temperature - kelvin;
	output.air_flow = boiler_result.air_flow;
	output.gas_flow = boiler_result.fire.gas_flow;
	output.smokebox_vacuum = boiler_result.smokebox_vacuum;
	output.superheater_metal_temperature = boiler.superheater_temperature - kelvin;
	output.chest_pressure = (engine_result.chest_pressure - ambient.pressure) * to_bar;
	output.steam_temperature = engine_result.chest_temperature - kelvin;
	output.exhaust_pressure = (engine_result.exhaust_pressure - ambient.pressure) * to_bar;
	output.exhaust_temperature = engine_result.exhaust_temperature - kelvin;
	output.exhaust_dryness = engine_result.exhaust_dryness;
	output.nozzle_flow = engine_result.nozzle_flow;
	output.nozzle_velocity = engine_result.nozzle_velocity;
	output.cocks_flow = engine_result.cocks_flow;
	output.cylinder_wall_loss = engine_result.wall_heat / 1.0e3;
	for (std::size_t index{0}; index < output.chamber_pressures.size(); ++index)
	{
		output.chamber_pressures[index] = (engine_result.chamber_pressures[index] - ambient.pressure) * to_bar;
		output.wall_temperatures[index] = steam_engine.wall_temperature[index] - kelvin;
	}
	output.tender_water = tender_water;
	output.tender_coal = tender_coal;
	output.sand = sand;
	output.locomotive_mass = locomotive_mass();
	output.tender_mass = tender_mass();
	return output;
}

} // namespace steam::detailed
