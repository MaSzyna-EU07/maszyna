/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "McZapkie/steam/boiler.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace steam::detailed
{

namespace
{

constexpr double quarter_pi{std::numbers::pi / 4.0};
constexpr double air_gas_constant{287.0}; // J/(kg K)
constexpr double blower_steam_volume{1.6}; // m³/kg
constexpr double blower_velocity{430.0}; // m/s
constexpr double bed_oxygen_depth{0.05}; // m, of fuel bed that takes the oxygen of the air
constexpr double burning_time{600.0}; // s, a bed burns away in at the slowest
constexpr double clinker_share{0.6}; // of the ash staying on the grate
constexpr double clinker_density{1100.0};
constexpr double max_unburnt{0.45};
constexpr double wall_temperature_over_water{15.0};
constexpr double firebox_convection{25.0}; // W/(m² K)
constexpr double feed_water_heat{4186.0}; // J/(kg K)
constexpr int segments{12};
constexpr double safety_valve_discharge{0.8};
constexpr double plug_hole{2.0e-4}; // m²
constexpr double plug_exposed_temperature{900.0}; // K of the gases over the exposed crown
constexpr double exposed_crown{0.10}; // m of level under the lowest allowed

double gas_heat_capacity(double const Temperature)
{
	return 1005.0 + 0.17 * (Temperature - kelvin);
}

double gas_viscosity(double const Temperature)
{
	return 1.80e-5 * std::pow(Temperature / 300.0, 0.70);
}

double gas_conductivity(double const Temperature)
{
	return 0.026 * std::pow(Temperature / 300.0, 0.80);
}

// the heat transfer coefficient of the gases flowing in a tube of the hydraulic diameter: Dittus and Boelter with
// the correction for the entrance, and the radiation of the gas
double gas_transfer_coefficient(double const Flow, double const Section, double const Diameter, double const Temperature, double const Length)
{
	constexpr double laminar{3.66};
	constexpr double gas_radiation{0.05};
	auto const reynolds{std::max(Flow / Section * Diameter / gas_viscosity(Temperature), 100.0)};
	auto const nusselt{reynolds > 2300.0 ? 0.023 * std::pow(reynolds, 0.8) * std::pow(0.72, 0.3) * (1.0 + std::pow(Diameter / Length, 0.7)) : laminar};
	return nusselt * gas_conductivity(Temperature) / Diameter + gas_radiation * stefan_boltzmann * 4.0 * Temperature * Temperature * Temperature;
}

// the pressure loss factor of gases in a tube: friction after Blasius over its length, and the entrance and exit
double tube_loss_factor(double const Flow, double const Section, double const Diameter, double const Temperature, double const Length)
{
	auto const reynolds{std::max(Flow / Section * Diameter / gas_viscosity(Temperature), 500.0)};
	return 0.316 * std::pow(reynolds, -0.25) * Length / Diameter + 1.5;
}

// what a fire gives before the heat is taken from the gases
struct bed
{
	double layer;
	double burnt;
	double firing_rate;
	double unburnt;
	double consumed;
	double excess_air;
	double monoxide_loss;
	double fire_heat;
	double gas_flow;
};

// the gases of the firebox: the temperature of the flame, and the temperature they leave the firebox at, the walls
// seeing the flame as a well mixed chamber (Hottel, the mean temperature of the flame after Gurvich)
struct firebox_result
{
	double adiabatic;
	double exit;
	double heat;
};

} // namespace

boiler::boiler(parameters const &Parameters) : m_parameters{Parameters}
{
	auto const &fuel{Parameters.fuel};
	// the air a kg of coal needs, from its elements
	m_air_need = 11.5 * fuel.carbon + 34.5 * (fuel.hydrogen - fuel.oxygen / 8.0) + 4.3 * fuel.sulphur;
	set_state(Parameters.boiler.working_pressure + Parameters.ambient.pressure, Parameters.boiler.water_volume_normal);
	fuel_mass = 0.10 * fuel.bulk_density * Parameters.boiler.grate_area;
	superheater_temperature = saturation_temperature(pressure) + 60.0;
	last.dryness_taken = 0.996;
	last.enthalpy_taken = line.liquid_enthalpy + last.dryness_taken * (line.vapour_enthalpy - line.liquid_enthalpy);
}

void boiler::set_ambient(ambient_parameters const &Ambient)
{
	m_parameters.ambient = Ambient;
}

double boiler::carbon_air() const
{
	return m_air_need;
}

double boiler::heat_capacity_superheater() const
{
	return m_parameters.superheater.pipe_mass * m_parameters.superheater.pipe_heat;
}

double boiler::working_pressure() const
{
	return m_parameters.boiler.working_pressure;
}

void boiler::set_state(double const Pressure, double const WaterVolume)
{
	auto const &data{m_parameters.boiler};
	auto const boiling{saturation(Pressure)};
	auto const water{WaterVolume / boiling.liquid_volume};
	auto const steam{(data.volume - WaterVolume) / boiling.vapour_volume};
	mass = water + steam;
	energy = water * boiling.liquid_energy + steam * boiling.vapour_energy + data.steel_mass * data.steel_heat * (boiling.temperature - kelvin);
	equilibrium();
}

// the pressure at which the mass and energy of the boiler are in equilibrium, by bisection on its logarithm
void boiler::equilibrium()
{
	constexpr int bisections{44};
	auto const &parameters{m_parameters.boiler};
	auto const specificvolume{parameters.volume / mass};
	auto low{std::log(0.6e5)};
	auto high{std::log(24.5e5)};
	for (auto bisection{0}; bisection < bisections; ++bisection)
	{
		auto const middle{0.5 * (low + high)};
		auto const boiling{saturation(std::exp(middle))};
		auto const wetness{(specificvolume - boiling.liquid_volume) / (boiling.vapour_volume - boiling.liquid_volume)};
		auto const contained{mass * (boiling.liquid_energy + wetness * (boiling.vapour_energy - boiling.liquid_energy)) +
		                     parameters.steel_mass * parameters.steel_heat * (boiling.temperature - kelvin)};
		if (contained > energy)
		{
			high = middle;
		}
		else
		{
			low = middle;
		}
	}
	pressure = std::exp(0.5 * (low + high));
	line = saturation(pressure);
	dryness = (specificvolume - line.liquid_volume) / (line.vapour_volume - line.liquid_volume);
	water_volume = mass * (1.0 - dryness) * line.liquid_volume;
}

namespace
{

// the bed of fuel on the grate for the air through it: how much coal is burnt, the share carried off unburnt, the
// air left over and the heat of the fire
bed burn_bed(parameters const &Parameters, double const FuelMass, double const AirNeed, double const AirFlow, double const Door)
{
	auto const &fuel{Parameters.fuel};
	auto const grate{Parameters.boiler.grate_area};
	bed result{};
	result.layer = FuelMass / (fuel.bulk_density * grate);
	// the bed takes the oxygen of the air the more of it the deeper it is, and burns away in 10 minutes at the soonest
	auto const oxygen{1.0 - std::exp(-result.layer / bed_oxygen_depth)};
	result.burnt = std::min(oxygen * AirFlow / AirNeed, FuelMass / burning_time);
	result.firing_rate = result.burnt / grate * 3600.0;
	result.unburnt = std::min(fuel.unburnt_loss * result.firing_rate, max_unburnt);
	auto const secondary{AirFlow * (fuel.secondary_air + 0.6 * Door)};
	result.consumed = result.burnt * (1.0 - result.unburnt);
	result.excess_air = (AirFlow + secondary) / std::max(result.consumed * AirNeed, 1.0e-9);
	result.monoxide_loss = std::clamp((1.10 - result.excess_air) / 1.10, 0.0, 1.0) * 0.6;
	result.fire_heat = result.consumed * fuel.heating_value * (1.0 - result.monoxide_loss);
	result.gas_flow = AirFlow + secondary + result.consumed * (1.0 - fuel.ash);
	return result;
}

// the heat the walls of the firebox take from the gases, and the temperature the gases leave it at: the heat the
// gases give up cooling down from the flame's temperature to it is what the walls take, by radiation and convection
struct firebox_balance
{
	double adiabatic;
	double wall;
	double flow;
	double emissivity;
	double surface;
};

double firebox_given(firebox_balance const &Balance, double const Exit)
{
	auto const flame{std::sqrt(std::max(Balance.adiabatic * Exit, 1.0))};
	auto const radiation{stefan_boltzmann * Balance.emissivity * Balance.surface * (std::pow(flame, 4) - std::pow(Balance.wall, 4))};
	return radiation + firebox_convection * Balance.surface * (Exit - Balance.wall);
}

double firebox_excess(firebox_balance const &Balance, double const Exit)
{
	return Balance.flow * gas_heat_capacity(0.5 * (Balance.adiabatic + Exit)) * (Balance.adiabatic - Exit) - firebox_given(Balance, Exit);
}

firebox_result burn_firebox(parameters const &Parameters, bed const &Bed, double const WaterTemperature, double const Ambient)
{
	constexpr int bisections{40};
	constexpr int iterations{6};
	auto adiabatic{1800.0};
	for (auto iteration{0}; iteration < iterations; ++iteration)
	{
		adiabatic = Ambient + Bed.fire_heat / (Bed.gas_flow * gas_heat_capacity(0.5 * (adiabatic + Ambient)));
	}
	auto const wall{WaterTemperature + wall_temperature_over_water};
	firebox_balance const balance{adiabatic, wall, Bed.gas_flow, Parameters.fuel.flame_emissivity, Parameters.boiler.firebox_surface};
	auto low{wall};
	auto high{std::max(adiabatic, wall + 1.0)};
	for (auto bisection{0}; bisection < bisections; ++bisection)
	{
		auto const middle{0.5 * (low + high)};
		if (firebox_excess(balance, middle) > 0.0)
		{
			low = middle;
		}
		else
		{
			high = middle;
		}
	}
	auto const exit{0.5 * (low + high)};
	return {adiabatic, exit, std::max(firebox_given(balance, exit), 0.0)};
}

// the gases on their way through a set of tubes of one kind, in segments, giving their heat to a wall
struct gas_run
{
	double temperature; // the gases at the end
	double heat; // W given to the wall
};

// the tubes: the gases of a share of the flow through 112 of them, to the water
gas_run run_tubes(boiler_parameters const &Boiler, double const Flow, double const Entry, double const Wall)
{
	auto const section{Boiler.tubes * quarter_pi * Boiler.tube_bore * Boiler.tube_bore};
	auto const segment{Boiler.tube_length / segments};
	gas_run run{Entry, 0.0};
	for (auto step{0}; step < segments; ++step)
	{
		auto const coefficient{gas_transfer_coefficient(Flow, section, Boiler.tube_bore, run.temperature, Boiler.tube_length)};
		auto const units{coefficient * Boiler.tubes * std::numbers::pi * Boiler.tube_bore * segment / (Flow * gas_heat_capacity(run.temperature))};
		auto const next{Wall + (run.temperature - Wall) * std::exp(-units)};
		run.heat += Flow * gas_heat_capacity(0.5 * (run.temperature + next)) * (run.temperature - next);
		run.temperature = next;
	}
	return run;
}

struct flue_run
{
	double temperature;
	double heat_to_water;
	double heat_to_superheater;
};

// the flues, with the pipes of the superheater elements in them, the number of the pipes in a section falling as the
// runs of the elements end towards the smokebox
flue_run run_flues(boiler_parameters const &Boiler, superheater_parameters const &Superheater, double const Flow, double const Entry, double const Wall, double const MetalTemperature)
{
	auto const segment{Boiler.tube_length / segments};
	flue_run run{Entry, 0.0, 0.0};
	for (auto step{0}; step < segments; ++step)
	{
		auto const from_smokebox{Boiler.tube_length - (step + 0.5) * segment};
		auto const pipes{static_cast<double>(std::ranges::count_if(Superheater.passes, [from_smokebox](double const Reach) { return from_smokebox < Reach; }))};
		auto const section{quarter_pi * (Boiler.flue_bore * Boiler.flue_bore - pipes * Superheater.pipe_outer * Superheater.pipe_outer)};
		auto const diameter{4.0 * section / (std::numbers::pi * (Boiler.flue_bore + pipes * Superheater.pipe_outer))};
		auto const coefficient{gas_transfer_coefficient(Flow, Boiler.flues * section, diameter, run.temperature, Boiler.tube_length)};
		auto const wallconductance{coefficient * Boiler.flues * std::numbers::pi * Boiler.flue_bore * segment};
		auto const metalconductance{coefficient * Boiler.flues * pipes * std::numbers::pi * Superheater.pipe_outer * segment};
		auto const capacity{Flow * gas_heat_capacity(run.temperature)};
		auto const equivalent{(wallconductance * Wall + metalconductance * MetalTemperature) / (wallconductance + metalconductance)};
		auto const next{equivalent + (run.temperature - equivalent) * std::exp(-(wallconductance + metalconductance) / capacity)};
		auto const given{capacity * (run.temperature - next)};
		auto const mean{0.5 * (run.temperature + next)};
		auto const towall{wallconductance * (mean - Wall)};
		auto const tometal{metalconductance * (mean - MetalTemperature)};
		if (std::abs(towall + tometal) > 1.0e-9)
		{
			run.heat_to_water += given * towall / (towall + tometal);
			run.heat_to_superheater += given * tometal / (towall + tometal);
		}
		run.temperature = next;
	}
	return run;
}

} // namespace

combustion boiler::fire(double const AirFlow, double const Door) const
{
	constexpr int iterations{5};
	constexpr double initial_tube_share{0.42};
	auto const &data{m_parameters.boiler};
	auto const &superheater{m_parameters.superheater};
	auto const bedresult{burn_bed(m_parameters, fuel_mass, m_air_need, AirFlow, Door)};
	combustion result;
	result.layer = bedresult.layer;
	result.burnt = bedresult.burnt;
	result.consumed = bedresult.consumed;
	result.firing_rate = bedresult.firing_rate;
	result.unburnt_loss = bedresult.unburnt;
	result.excess_air = bedresult.excess_air;
	result.monoxide_loss = bedresult.monoxide_loss;
	result.fire_heat = bedresult.fire_heat;
	result.gas_flow = bedresult.gas_flow;
	auto const wall{line.temperature + wall_temperature_over_water};
	auto const ambient{m_parameters.ambient.temperature};
	if (bedresult.gas_flow < 1.0e-4)
	{
		result.adiabatic_temperature = ambient;
		result.firebox_exit_temperature = wall;
		result.smokebox_temperature = wall;
		return result;
	}
	auto const firebox{burn_firebox(m_parameters, bedresult, line.temperature, ambient)};
	result.adiabatic_temperature = firebox.adiabatic;
	result.firebox_exit_temperature = firebox.exit;
	result.firebox_heat = firebox.heat;
	// the gases divide between the tubes and the flues so that both lose the same pressure
	auto const tubesection{data.tubes * quarter_pi * data.tube_bore * data.tube_bore};
	auto const pipesection{quarter_pi * (data.flue_bore * data.flue_bore - 4.0 * superheater.pipe_outer * superheater.pipe_outer)};
	auto const flue_section{data.flues * pipesection};
	auto const flue_diameter{4.0 * pipesection / (std::numbers::pi * (data.flue_bore + 4.0 * superheater.pipe_outer))};
	auto const mean{0.5 * (firebox.exit + wall + 150.0)};
	auto const density{m_parameters.ambient.pressure / (air_gas_constant * mean)};
	auto share{initial_tube_share};
	auto tubefactor{0.0};
	for (auto iteration{0}; iteration < iterations; ++iteration)
	{
		tubefactor = tube_loss_factor(share * bedresult.gas_flow, tubesection, data.tube_bore, mean, data.tube_length);
		auto const flue_factor{tube_loss_factor((1.0 - share) * bedresult.gas_flow, flue_section, flue_diameter, mean, data.tube_length)};
		auto const ratio{tubesection / flue_section * std::sqrt(flue_factor / tubefactor)};
		share = ratio / (1.0 + ratio);
	}
	auto const tubeflow{share * bedresult.gas_flow};
	auto const flueflow{(1.0 - share) * bedresult.gas_flow};
	auto const tubes{run_tubes(data, tubeflow, firebox.exit, wall)};
	auto const flues{run_flues(data, superheater, flueflow, firebox.exit, wall, superheater_temperature)};
	result.tube_heat = tubes.heat;
	result.flue_heat = flues.heat_to_water;
	result.superheater_heat = flues.heat_to_superheater;
	result.smokebox_temperature = (tubeflow * tubes.temperature + flueflow * flues.temperature) / bedresult.gas_flow;
	auto const velocity{tubeflow / (density * tubesection)};
	result.tube_pressure_drop = tubefactor * density * velocity * velocity / 2.0;
	result.tube_share = share;
	return result;
}

double boiler::resistance(double const AirFlow, double const Dampers, double const TubePressureDrop) const
{
	auto const &draught{m_parameters.draught};
	auto const &ambient{m_parameters.ambient};
	auto const air_density{ambient.pressure / (air_gas_constant * ambient.temperature)};
	auto const layer{(fuel_mass / m_parameters.fuel.bulk_density + clinker_mass / clinker_density) / m_parameters.boiler.grate_area};
	auto const grate{(draught.grate_resistance + draught.bed_resistance * layer) * std::pow(AirFlow / m_parameters.boiler.grate_area, 2) / (2.0 * air_density)};
	auto const ashpan{draught.ashpan_resistance * std::pow(AirFlow / std::max(draught.damper_area * Dampers, 1.0e-3), 2) / (2.0 * air_density)};
	return grate + ashpan + TubePressureDrop;
}

double boiler::ejector_vacuum(double const NozzleFlow, double const NozzleVelocity, double const NozzleVolume, double const BlowerFlow, double const GasFlow, double const GasTemperature) const
{
	auto const &draught{m_parameters.draught};
	auto const throat{quarter_pi * draught.chimney_throat * draught.chimney_throat};
	auto const outlet{quarter_pi * draught.chimney_top * draught.chimney_top};
	auto const gas_volume{air_gas_constant * GasTemperature / m_parameters.ambient.pressure};
	auto const volume_flow{NozzleFlow * NozzleVolume + BlowerFlow * blower_steam_volume + GasFlow * gas_volume};
	auto const total{NozzleFlow + BlowerFlow + GasFlow};
	if (total <= 0.0 || volume_flow <= 0.0)
	{
		return 0.0;
	}
	auto const mixed_velocity{volume_flow / throat};
	auto const mixed_density{total / volume_flow};
	auto const momentum{NozzleFlow * NozzleVelocity + BlowerFlow * blower_velocity};
	return (momentum - total * mixed_velocity) / throat + draught.diffuser_efficiency * mixed_density * mixed_velocity * mixed_velocity / 2.0 * (1.0 - std::pow(throat / outlet, 2));
}

double boiler::draught_flow(boiler_input const &Input, double const BlowerFlow) const
{
	constexpr int bisections{26};
	constexpr double flow_limit{30.0};
	auto const &ambient{m_parameters.ambient};
	// the values of the input the search needs, taken by the lambda as values rather than through the reference
	auto const door{Input.door};
	auto const dampers{Input.dampers};
	auto const nozzle_flow{Input.nozzle_flow};
	auto const nozzle_velocity{Input.nozzle_velocity};
	auto const nozzle_volume{Input.nozzle_volume};
	// the vacuum the exhaust makes, plus the natural draught of the chimney, less what the gas path takes
	auto const surplus{[this, door, dampers, nozzle_flow, nozzle_velocity, nozzle_volume, BlowerFlow, &ambient](double const Flow)
	                   {
		                   auto const burning{fire(Flow, door)};
		                   auto const natural{gravity * 1.6 * ambient.pressure / air_gas_constant * (1.0 / ambient.temperature - 1.0 / std::max(burning.smokebox_temperature, ambient.temperature))};
		                   auto const ejector{ejector_vacuum(nozzle_flow, nozzle_velocity, nozzle_volume, BlowerFlow, burning.gas_flow, burning.smokebox_temperature)};
		                   return ejector + natural - resistance(Flow, dampers, burning.tube_pressure_drop);
	                   }};
	if (surplus(flow_limit) > 0.0)
	{
		return flow_limit;
	}
	auto low{0.0};
	auto high{flow_limit};
	for (auto bisection{0}; bisection < bisections; ++bisection)
	{
		auto const middle{0.5 * (low + high)};
		if (surplus(middle) > 0.0)
		{
			low = middle;
		}
		else
		{
			high = middle;
		}
	}
	return 0.5 * (low + high);
}

boiler_output boiler::step(double const Deltatime, boiler_input const &Input)
{
	constexpr double blower_overpressure_limit{1.2};
	constexpr double clinker_per_ash{clinker_share};
	constexpr double minimum_feed_pressure{3.0e5}; // gauge, under which the injectors don't feed
	auto const &data{m_parameters.boiler};
	auto const &draught{m_parameters.draught};
	auto const &ambient{m_parameters.ambient};
	boiler_output output;
	output.blower_flow = draught.blower_flow * Input.blower * std::min((pressure - ambient.pressure) / data.working_pressure, blower_overpressure_limit);
	// the air follows the draught with the inertia of the gases of the boiler
	auto const draughtflow{draught_flow(Input, output.blower_flow)};
	air_flow += (draughtflow - air_flow) * std::min(Deltatime / draught.gas_inertia, 1.0);
	output.fire = fire(air_flow, Input.door);
	output.air_flow = air_flow;
	// the fuel and the clinker
	fuel_mass = std::max(fuel_mass + (Input.firing - output.fire.burnt) * Deltatime, 0.0);
	clinker_mass = std::max(clinker_mass + (output.fire.consumed * m_parameters.fuel.ash * clinker_per_ash - Input.clinkering) * Deltatime, 0.0);
	// the safety valves, of the pop kind: they open at the working pressure and shut a little under it
	auto const gauge{pressure - ambient.pressure};
	if (gauge >= data.working_pressure)
	{
		safety_valves_open = true;
	}
	else if (gauge < data.working_pressure - data.safety_valve_blowdown)
	{
		safety_valves_open = false;
	}
	if (safety_valves_open)
	{
		auto const seat{quarter_pi * data.safety_valve_seat * data.safety_valve_seat};
		auto const lift{std::min(std::max(gauge - (data.working_pressure - data.safety_valve_blowdown), 0.0) * seat * 3.0 / data.safety_valve_spring, data.safety_valve_seat / 4.0)};
		output.safety_valve_flow = data.safety_valves * orifice_flow({std::numbers::pi * data.safety_valve_seat * lift, safety_valve_discharge}, {pressure, line.vapour_volume}, ambient.pressure);
	}
	// the water carried over with the steam, the more of it the faster the steam leaves the surface and the higher the level
	auto const taken{Input.regulator_flow + Input.auxiliary_flow + output.blower_flow + output.safety_valve_flow};
	auto const surface_velocity{taken * line.vapour_volume / data.water_surface};
	output.level = (water_volume - data.water_volume_normal) / data.water_surface;
	auto const carried{std::min(0.004 + 0.02 * std::max(surface_velocity - 0.35, 0.0) + 0.6 * std::max(output.level - 0.08, 0.0), 0.5)};
	output.dryness_taken = 1.0 - carried;
	output.enthalpy_taken = line.liquid_enthalpy + output.dryness_taken * (line.vapour_enthalpy - line.liquid_enthalpy);
	output.heat_lost = data.lagging_transmittance * data.lagging_surface * (line.temperature - ambient.temperature) * (1.0 + 0.25 * std::pow(std::abs(Input.relative_speed), 0.6));
	output.heat_to_water = output.fire.firebox_heat + output.fire.tube_heat + output.fire.flue_heat;
	auto const feed{gauge > minimum_feed_pressure ? Input.feed_flow : 0.0};
	auto const feed_enthalpy{feed_water_heat * (m_parameters.auxiliaries.feed_temperature + Input.feed_heating - kelvin)};
	auto blowdown{Input.blowdown};
	if (fusible_plug_melted)
	{
		blowdown += orifice_flow({plug_hole, safety_valve_discharge}, {pressure, line.vapour_volume}, ambient.pressure);
	}
	mass += (feed - taken - blowdown) * Deltatime;
	energy += (output.heat_to_water + feed * feed_enthalpy - taken * output.enthalpy_taken - blowdown * line.liquid_enthalpy - output.heat_lost) * Deltatime;
	equilibrium();
	// the crown of the firebox bared to the fire melts the lead of the fusible plug
	if (water_volume < data.water_volume_min - exposed_crown * data.water_surface && output.fire.firebox_exit_temperature > plug_exposed_temperature)
	{
		fusible_plug_melted = true;
	}
	output.smokebox_vacuum = resistance(air_flow, Input.dampers, output.fire.tube_pressure_drop);
	last = output;
	return output;
}

} // namespace steam::detailed
