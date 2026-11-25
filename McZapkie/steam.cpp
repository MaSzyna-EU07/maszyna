/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "McZapkie/steam.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numbers>
#include <string>

#include "utilities/utilities.h"

namespace steam {

namespace {

// a point of the saturation line
struct saturated {
	double temperature; // °C
	double pressure; // Pa
	double latent_heat; // J/kg
	double specific_heat; // J/(kg K), of the water
};

// IAPWS-IF97
constexpr std::array saturation_table{
    saturated{0.01, 611.7, 2500.9e3, 4220.0},       saturated{20.0, 2'339.0, 2453.5e3, 4182.0},
    saturated{40.0, 7'385.0, 2406.0e3, 4179.0},     saturated{60.0, 19'946.0, 2357.7e3, 4185.0},
    saturated{80.0, 47'416.0, 2308.0e3, 4197.0},    saturated{100.0, 101'420.0, 2256.4e3, 4216.0},
    saturated{120.0, 198'670.0, 2202.1e3, 4245.0},  saturated{140.0, 361'540.0, 2144.3e3, 4285.0},
    saturated{160.0, 618'230.0, 2082.0e3, 4339.0},  saturated{180.0, 1'002'800.0, 2014.2e3, 4410.0},
    saturated{200.0, 1'554'900.0, 1939.7e3, 4497.0}, saturated{220.0, 2'319'600.0, 1857.5e3, 4610.0},
    saturated{240.0, 3'346'900.0, 1765.6e3, 4760.0},
};

// the segment of the saturation line holding the value, and how far along it the value is, 0..1
struct segment {
	saturated const &low;
	saturated const &high;
	double along;
};

segment temperature_segment(double const Temperature)
{
	auto const upper{std::ranges::upper_bound(saturation_table, Temperature, {}, &saturated::temperature)};
	auto const index{std::clamp<std::ptrdiff_t>(upper - saturation_table.begin(), 1, std::ssize(saturation_table) - 1)};
	auto const &low{saturation_table[index - 1]};
	auto const &high{saturation_table[index]};
	return {low, high, std::clamp((Temperature - low.temperature) / (high.temperature - low.temperature), 0.0, 1.0)};
}

constexpr double gas_constant{461.5}; // J/(kg K) of steam
constexpr double kelvin{273.15};
constexpr double bar{1.0e5};
constexpr int chest_iterations{30};
constexpr double regulator_choke{0.55}; // the ratio of the chest to the boiler pressure under which the flow is choked
constexpr double leakage{0.002 / bar}; // kg/s per Pa of chest pressure, past the valves and pistons
constexpr double exhaust_delay{0.3}; // s
constexpr double superheat_delay{90.0}; // s
constexpr double plug_quench_time{30.0}; // s for the steam of a melted fusible plug to put the fire out
constexpr double injector_minimum_pressure{1.5 * bar}; // gauge, under which the injectors don't feed
constexpr double safety_valve_blowdown{0.3 * bar}; // under the working pressure, where the valves close again
constexpr double fireman_level{0.55}; // of the glass, where the fireman keeps the water
constexpr double fireman_reaction{60.0}; // s in which the fireman makes up the fire to what it should be

// steam taken as an ideal gas, kg/m³
double density(double const Pressure, double const Temperature)
{
	return Pressure / (gas_constant * (Temperature + kelvin));
}

// moves the value towards the target, reaching it in about the given time
double follow(double const Value, double const Target, double const Time, double const Deltatime)
{
	return Value + (Target - Value) * std::min(1.0, Deltatime / Time);
}

double full_burn_rate(parameters const &Parameters)
{
	return Parameters.grate_area * Parameters.firing_rate_max / 3600.0;
}

// the coal burned by the fire of the given mass with the given draught, kg/s. a thin fire burns as much less as it
// is thinner; a thick one, choking its own air, little more than the right one
double burn_rate(parameters const &Parameters, double const Fire, double const Draught)
{
	auto const thickness{Fire / Parameters.fire_mass};
	auto const fire{thickness <= 1.0 ? thickness : 1.0 + 0.25 * (thickness - 1.0)};
	return full_burn_rate(Parameters) * Draught * fire;
}

// the part of the heat of the coal burned at the given rate which reaches the water
double boiler_efficiency(parameters const &Parameters, double const Burnrate)
{
	auto const rate{Burnrate / full_burn_rate(Parameters)};
	return std::lerp(Parameters.efficiency_max, Parameters.efficiency_min, std::clamp(rate, 0.0, 1.0));
}

// what the fireman does with the coal, the injectors and the blower
struct fireman_work {
	double firing{0.0};
	std::array<double, 2> injectors{0.0, 0.0};
	double blower{0.0};
};

fireman_work by_hand(controls const &Controls)
{
	return {Controls.firing, Controls.injectors, Controls.blower};
}

double fireman_firing(parameters const &Parameters, state const &State)
{
	// the fire is kept up with the steam taken, thicker with the pressure falling short of a little under the
	// working one and let burn down over it, not to blow off
	auto const demand{std::clamp(State.cylinder_flow / Parameters.steam_rate, 0.0, 1.0)};
	auto const deficit{std::clamp((Parameters.working_pressure - safety_valve_blowdown - State.pressure()) / (1.5 * bar), -1.0, 1.0)};
	auto const target{Parameters.fire_mass * std::clamp(0.2 + 0.8 * demand + 0.6 * deficit, 0.02, 1.1)};
	return std::clamp(State.burn_rate + (target - State.fire_mass) / fireman_reaction, 0.0, 1.5 * full_burn_rate(Parameters));
}

std::array<double, 2> fireman_injectors(parameters const &Parameters, state const &State)
{
	// the level is kept a little over the middle of the glass, the injector set to feed what the steam takes;
	// with the pressure low the fireman holds back the cold water, unless the level is low too
	auto const level{State.water_level(Parameters)};
	auto const pressurelow{State.pressure() < Parameters.working_pressure - 1.5 * bar};
	auto const target{pressurelow ? fireman_level - 0.15 : fireman_level};
	// the exhaust steam injector works on the move, under steam, the live steam one standing
	auto const running{State.cylinder_flow > 0.2 * Parameters.steam_rate};
	auto const injector{running ? 1 : 0};
	auto const taken{State.exhaust_flow + State.safety_valve_flow};
	auto const correction{(target - level) * 4.0};
	auto const opening{correction > 0.0 ? std::clamp(taken / Parameters.injector_flow[injector] + correction, 0.0, 1.0) : 0.0};
	std::array<double, 2> injectors{0.0, 0.0};
	injectors[injector] = opening;
	// both, with the water low
	if (level < 0.25)
	{
		injectors = {1.0, 1.0};
	}
	return injectors;
}

double fireman_blower(parameters const &Parameters, state const &State)
{
	if (State.fire_mass <= 0.0)
	{
		return 0.0;
	}
	// a little blower always, not to have the flames blown back with the regulator shut; more with the exhaust weak and the pressure low
	auto const exhaustweak{State.exhaust_flow < 0.3 * Parameters.steam_rate};
	auto const pressurelow{State.pressure() < Parameters.working_pressure - 0.5 * bar};
	return exhaustweak && pressurelow ? 0.5 : 0.1;
}

fireman_work automatic_fireman(parameters const &Parameters, state const &State)
{
	return {fireman_firing(Parameters, State), fireman_injectors(Parameters, State), fireman_blower(Parameters, State)};
}

// the cylinders at a given chest pressure
struct cylinder_conditions {
	double revolutions; // per second
	double cutoff;
	double steam_temperature; // °C
	double back_pressure; // Pa
	bool cocks;
};

double admission_pressure(parameters const &Parameters, double const Chest, double const Revolutions)
{
	auto const throttling{std::max(0.0, 1.0 - Parameters.wire_drawing * std::sqrt(Revolutions * 60.0))};
	return atmosphere + (Chest - atmosphere) * throttling;
}

double condensation(parameters const &Parameters, double const Cutoff)
{
	return Parameters.condensation * std::lerp(1.0, 0.2, std::clamp((Cutoff - 0.05) / 0.5, 0.0, 1.0));
}

// the steam the cylinders take in a revolution of the wheels, kg
double steam_per_revolution(parameters const &Parameters, cylinder_conditions const &Conditions, double const Admission)
{
	auto const volume{Parameters.swept_volume()};
	auto const admitted{volume * (Conditions.cutoff + Parameters.clearance) * density(Admission, Conditions.steam_temperature)};
	auto const kept{volume * Parameters.clearance * density(Conditions.back_pressure, Conditions.steam_temperature)};
	return std::max(0.0, admitted - kept) * (1.0 + condensation(Parameters, Conditions.cutoff));
}

// the steam the cylinders take at the given chest pressure, kg/s, with what leaks and blows out of the cocks
double cylinder_demand(parameters const &Parameters, cylinder_conditions const &Conditions, double const Chest)
{
	if (Conditions.cutoff <= 0.0)
	{
		return leakage * (Chest - atmosphere);
	}
	auto const admission{admission_pressure(Parameters, Chest, Conditions.revolutions)};
	auto const cocks{Conditions.cocks ? Parameters.cocks_flow * (admission - atmosphere) / bar : 0.0};
	return Conditions.revolutions * steam_per_revolution(Parameters, Conditions, admission) + cocks + leakage * (Chest - atmosphere);
}

// the steam passing the regulator into the chest, kg/s: choked down to the critical pressure ratio, falling to none
// with the chest at the boiler pressure
double regulator_flow(parameters const &Parameters, double const Opening, double const Boiler, double const Chest)
{
	auto const ratio{Chest / Boiler};
	auto const choking{(ratio - regulator_choke) / (1.0 - regulator_choke)};
	auto const passing{ratio <= regulator_choke ? 1.0 : std::sqrt(std::max(0.0, 1.0 - choking * choking))};
	return Parameters.regulator_flow * Opening * Boiler / (Parameters.working_pressure + atmosphere) * passing;
}

// the chest pressure at which the regulator passes as much steam as the cylinders take
double chest_pressure(parameters const &Parameters, cylinder_conditions const &Conditions, double const Regulator, double const Boiler)
{
	if (Regulator <= 0.0 || Boiler <= atmosphere)
	{
		return atmosphere;
	}
	auto low{atmosphere};
	auto high{Boiler};
	for (auto iteration{0}; iteration < chest_iterations; ++iteration)
	{
		auto const chest{0.5 * (low + high)};
		if (regulator_flow(Parameters, Regulator, Boiler, chest) > cylinder_demand(Parameters, Conditions, chest))
		{
			low = chest;
		}
		else
		{
			high = chest;
		}
	}
	return 0.5 * (low + high);
}

// the mean effective pressure of the indicator diagram: admission to the cutoff, hyperbolic expansion to the end of the
// stroke, exhaust against the back pressure. the real diagram, with its release, compression and wire drawing, is the
// diagram factor of the ideal one
double mean_effective_pressure(parameters const &Parameters, double const Cutoff, double const Admission, double const Back)
{
	if (Cutoff <= 0.0)
	{
		return 0.0;
	}
	auto const clearance{Parameters.clearance};
	auto const expansion{(Cutoff + clearance) * std::log((1.0 + clearance) / (Cutoff + clearance))};
	return Parameters.diagram_factor * Admission * (Cutoff + expansion) - Back;
}

// the force at the rims of a mean effective pressure
double rim_force(parameters const &Parameters, double const Pressure)
{
	return Pressure * Parameters.swept_volume() / (std::numbers::pi * Parameters.wheel_diameter) * Parameters.mechanical_efficiency;
}

// how the force of two cylinders cranked 90° apart swings over a revolution, about its mean: from 0.79 with one
// crank on its dead centre to 1.11 with both at 45° of it. the swing is felt starting and fades away with speed,
// where the masses of the train smooth it out
double pulsation(parameters const &Parameters, double const Revolutions, double const Crankangle)
{
	if (Parameters.cylinders != 2)
	{
		return 1.0;
	}
	auto const swing{(std::abs(std::sin(Crankangle)) + std::abs(std::cos(Crankangle))) * std::numbers::pi / 4.0};
	auto const fading{std::clamp(1.0 - (Revolutions - 1.0) / 1.5, 0.0, 1.0)};
	return 1.0 + Parameters.torque_pulsation * fading * (swing - 1.0);
}

// the engine's work in the step: the chest pressure, the steam taken and the force
void run_cylinders(parameters const &Parameters, state &State, controls const &Controls, double const Revolutions, double const Crankangle, double const Deltatime)
{
	cylinder_conditions const conditions{
	    Revolutions,
	    std::clamp(Controls.cutoff, 0.0, std::max(Parameters.cutoff_max, Parameters.cutoff_max_reverse)),
	    saturation::temperature(State.boiler_pressure) + State.superheat,
	    atmosphere + Parameters.back_pressure * State.exhaust_flow * State.exhaust_flow,
	    Controls.cylinder_cocks,
	};
	State.chest_pressure = chest_pressure(Parameters, conditions, std::clamp(Controls.regulator, 0.0, 1.0), State.boiler_pressure);
	auto const admission{admission_pressure(Parameters, State.chest_pressure, Revolutions)};
	State.cylinder_flow = Revolutions * steam_per_revolution(Parameters, conditions, admission);
	if (conditions.cutoff <= 0.0)
	{
		State.cylinder_flow = 0.0;
	}
	auto pressure{mean_effective_pressure(Parameters, conditions.cutoff, admission, conditions.back_pressure)};
	if (State.chest_pressure - atmosphere < Parameters.equaliser_pressure)
	{
		// the pressure equalisers open, the pistons drift without pumping
		pressure = std::max(pressure, 0.0);
	}
	// water carried over with the steam does no work
	pressure *= 1.0 - State.priming;
	State.power = pressure * Parameters.swept_volume() * Revolutions;
	State.force = rim_force(Parameters, pressure) * pulsation(Parameters, Revolutions, Crankangle);
	// what goes out of the chimney, with what blows out of the cocks
	auto const outflow{cylinder_demand(Parameters, conditions, State.chest_pressure)};
	State.exhaust_flow = follow(State.exhaust_flow, outflow, exhaust_delay, Deltatime);
}

double safety_valves(parameters const &Parameters, state &State)
{
	auto const pressure{State.pressure()};
	if (pressure >= Parameters.working_pressure)
	{
		State.safety_valves_open = true;
	}
	else if (pressure < Parameters.working_pressure - safety_valve_blowdown)
	{
		State.safety_valves_open = false;
	}
	if (false == State.safety_valves_open)
	{
		return 0.0;
	}
	// wide open 3% over the working pressure, and passing a little down to where they close
	auto const over{(pressure - Parameters.working_pressure) / (0.03 * Parameters.working_pressure)};
	return Parameters.safety_valve_flow * std::clamp(over, 0.1, 1.0);
}

double blower_flow(parameters const &Parameters, state const &State)
{
	return State.blower * Parameters.blower_flow * std::sqrt(std::max(0.0, State.pressure()) / Parameters.working_pressure);
}

double injector_feed(parameters const &Parameters, state const &State)
{
	if (State.pressure() < injector_minimum_pressure || State.tender_water <= 0.0)
	{
		return 0.0;
	}
	return State.injectors[0] * Parameters.injector_flow[0] + State.injectors[1] * Parameters.injector_flow[1];
}

// the water and heat of the boiler: steam taken off the water at its latent heat, the feed water heated up to the
// water's temperature, the heat of the fire, and the heat lost
void run_boiler(parameters const &Parameters, state &State, controls const &Controls, double const Deltatime)
{
	State.safety_valve_flow = safety_valves(Parameters, State);
	auto const steaming{State.pressure() > 0.0};
	auto taken{0.0};
	if (steaming)
	{
		taken = State.exhaust_flow + State.safety_valve_flow + blower_flow(Parameters, State) + Controls.auxiliary_flow;
	}
	// the water carried over with the steam into the cylinders takes no latent heat
	auto const vapour{taken - State.priming * State.cylinder_flow};
	auto const feed{injector_feed(Parameters, State)};
	auto const temperature{State.water_temperature};
	auto const specificheat{saturation::specific_heat(temperature)};
	auto const heat{State.fire_heat - vapour * saturation::latent_heat(temperature) - feed * specificheat * (temperature - Parameters.feed_temperature) -
	                Parameters.heat_loss * (temperature - Parameters.feed_temperature)};
	State.water_temperature += heat / (State.water_mass * specificheat + Parameters.steel_heat) * Deltatime;
	State.water_mass = std::max(0.0, State.water_mass + (feed - taken) * Deltatime);
	State.tender_water = std::max(0.0, State.tender_water - feed * Deltatime);
	State.boiler_pressure = std::max(atmosphere, saturation::pressure(State.water_temperature));
	// the superheater heats the steam as much more as more of it flows through, damped by the water carried over
	auto const flow{std::clamp(State.cylinder_flow / Parameters.steam_rate, 0.0, 1.5)};
	auto const superheat{Parameters.superheat_max * std::sqrt(flow) * (1.0 - 2.0 * State.priming)};
	State.superheat = follow(State.superheat, std::max(0.0, superheat), superheat_delay, Deltatime);
	// the level over the top of the glass carries water over into the cylinders, half of the steam at most
	State.priming = std::clamp((State.water_level(Parameters) - 1.0) * 2.0, 0.0, 0.5);
}

// the fire: the coal shovelled on, burned with the draught of the exhaust and the blower, its heat reaching the water
void run_fire(parameters const &Parameters, state &State, double const Firing, double const Deltatime)
{
	auto const crown{Parameters.water_mass - 0.5 * Parameters.glass_mass - Parameters.crown_mass};
	if (State.water_mass < crown && State.fire_mass > 0.0)
	{
		// the firebox crown out of the water: the lead of the fusible plug melts, its steam puts the fire out
		State.fusible_plug_melted = true;
	}
	auto const exhaust{Parameters.exhaust_draught * std::pow(State.exhaust_flow / Parameters.steam_rate, 0.75)};
	auto const blower{State.blower * Parameters.blower_draught * std::sqrt(std::max(0.0, State.pressure()) / Parameters.working_pressure)};
	State.draught = std::clamp(Parameters.natural_draught + exhaust + blower, 0.0, 1.0);
	auto const coal{State.tender_coal > 0.0 && false == State.fusible_plug_melted ? Firing : 0.0};
	State.tender_coal = std::max(0.0, State.tender_coal - coal * Deltatime);
	if (State.fusible_plug_melted)
	{
		State.burn_rate = 0.0;
		State.fire_mass = follow(State.fire_mass, 0.0, plug_quench_time, Deltatime);
	}
	else
	{
		State.burn_rate = burn_rate(Parameters, State.fire_mass, State.draught);
		State.fire_mass = std::max(0.0, State.fire_mass + (coal - State.burn_rate) * Deltatime);
	}
	auto const heat{State.burn_rate * Parameters.coal_heat * boiler_efficiency(Parameters, State.burn_rate)};
	State.fire_heat = follow(State.fire_heat, heat, Parameters.fire_delay, Deltatime);
}

void read(double &Variable, std::string const &Key, std::string const &Line, double const Scale = 1.0)
{
	if (auto value{0.0}; extract_value(value, Key, Line, ""))
	{
		Variable = value * Scale;
	}
}

} // namespace

namespace saturation {

double pressure(double const Temperature)
{
	auto const [low, high, along]{temperature_segment(Temperature)};
	return std::exp(std::lerp(std::log(low.pressure), std::log(high.pressure), along));
}

double temperature(double const Pressure)
{
	auto const upper{std::ranges::upper_bound(saturation_table, Pressure, {}, &saturated::pressure)};
	auto const index{std::clamp<std::ptrdiff_t>(upper - saturation_table.begin(), 1, std::ssize(saturation_table) - 1)};
	auto const &low{saturation_table[index - 1]};
	auto const &high{saturation_table[index]};
	auto const along{(std::log(Pressure) - std::log(low.pressure)) / (std::log(high.pressure) - std::log(low.pressure))};
	return std::lerp(low.temperature, high.temperature, std::clamp(along, 0.0, 1.0));
}

double latent_heat(double const Temperature)
{
	auto const [low, high, along]{temperature_segment(Temperature)};
	return std::lerp(low.latent_heat, high.latent_heat, along);
}

double specific_heat(double const Temperature)
{
	auto const [low, high, along]{temperature_segment(Temperature)};
	return std::lerp(low.specific_heat, high.specific_heat, along);
}

} // namespace saturation

double parameters::swept_volume() const
{
	// double acting: both sides of each piston
	return cylinders * 2.0 * std::numbers::pi / 4.0 * bore * bore * stroke;
}

void load(parameters &Parameters, std::string_view const Line)
{
	std::string const line{Line};
	if (auto cylinders{0}; extract_value(cylinders, "Cylinders", line, "") && cylinders > 0)
	{
		Parameters.cylinders = cylinders;
	}
	read(Parameters.bore, "CylD", line);
	read(Parameters.stroke, "CylS", line);
	read(Parameters.clearance, "CylClearance", line);
	read(Parameters.cutoff_max, "CutoffMax", line);
	read(Parameters.cutoff_min, "CutoffMin", line);
	read(Parameters.cutoff_max_reverse, "CutoffMaxReverse", line);
	read(Parameters.diagram_factor, "DiagramFactor", line);
	read(Parameters.mechanical_efficiency, "MechEff", line);
	read(Parameters.wire_drawing, "WireDrawing", line);
	read(Parameters.back_pressure, "BackPressure", line, bar);
	read(Parameters.condensation, "Condensation", line);
	read(Parameters.regulator_flow, "RegulatorFlow", line);
	read(Parameters.cocks_flow, "CocksFlow", line);
	read(Parameters.torque_pulsation, "TorquePulsation", line);
	read(Parameters.equaliser_pressure, "EqualiserPressure", line, bar);
	read(Parameters.working_pressure, "BoilerPressure", line, bar);
	read(Parameters.safety_valve_flow, "SafetyValveFlow", line);
	read(Parameters.water_mass, "BoilerWater", line);
	read(Parameters.glass_mass, "GlassWater", line);
	read(Parameters.crown_mass, "CrownWater", line);
	read(Parameters.swell, "Swell", line);
	read(Parameters.steel_heat, "BoilerSteelHeat", line, 1.0e6);
	read(Parameters.heat_loss, "BoilerHeatLoss", line);
	read(Parameters.superheat_max, "Superheat", line);
	read(Parameters.steam_rate, "SteamRate", line);
	read(Parameters.grate_area, "Grate", line);
	read(Parameters.fire_mass, "FireMass", line);
	read(Parameters.firing_rate_max, "FiringRateMax", line);
	read(Parameters.coal_heat, "CoalHeat", line, 1.0e6);
	read(Parameters.efficiency_max, "BoilerEfficiency", line);
	read(Parameters.efficiency_min, "BoilerEfficiencyMin", line);
	read(Parameters.fire_delay, "FireDelay", line);
	read(Parameters.exhaust_draught, "ExhaustDraught", line);
	read(Parameters.blower_draught, "BlowerDraught", line);
	read(Parameters.blower_flow, "BlowerFlow", line);
	// l/min of water
	read(Parameters.injector_flow[0], "InjectorFlow1", line, 1.0 / 60.0);
	read(Parameters.injector_flow[1], "InjectorFlow2", line, 1.0 / 60.0);
	read(Parameters.feed_temperature, "FeedTemperature", line);
	// m³ of water, t of coal
	read(Parameters.tender_water, "TenderWater", line, 1000.0);
	read(Parameters.tender_coal, "TenderCoal", line, 1000.0);
	read(Parameters.air_pump_flow, "AirPumpFlow", line);
}

double state::water_level(parameters const &Parameters) const
{
	auto const bottom{Parameters.water_mass - 0.5 * Parameters.glass_mass};
	return (water_mass - bottom) / Parameters.glass_mass + Parameters.swell * (cylinder_flow + safety_valve_flow);
}

state in_steam(parameters const &Parameters)
{
	state result;
	result.water_temperature = saturation::temperature(atmosphere + Parameters.working_pressure - 0.5 * bar);
	result.boiler_pressure = saturation::pressure(result.water_temperature);
	result.water_mass = Parameters.water_mass;
	result.fire_mass = 0.7 * Parameters.fire_mass;
	result.draught = Parameters.natural_draught;
	result.burn_rate = burn_rate(Parameters, result.fire_mass, result.draught);
	result.fire_heat = result.burn_rate * Parameters.coal_heat * boiler_efficiency(Parameters, result.burn_rate);
	result.tender_water = Parameters.tender_water;
	result.tender_coal = Parameters.tender_coal;
	return result;
}

state cold(parameters const &Parameters, double const Temperature)
{
	state result;
	result.water_temperature = Temperature;
	result.boiler_pressure = std::max(atmosphere, saturation::pressure(Temperature));
	result.water_mass = Parameters.water_mass;
	result.tender_water = Parameters.tender_water;
	result.tender_coal = Parameters.tender_coal;
	return result;
}

void update(parameters const &Parameters, state &State, controls const &Controls, double const Revolutions, double const CrankAngle, double const Deltatime)
{
	if (Deltatime <= 0.0)
	{
		return;
	}
	auto const work{Controls.automatic_fireman ? automatic_fireman(Parameters, State) : by_hand(Controls)};
	State.injectors = work.injectors;
	State.blower = std::clamp(work.blower, 0.0, 1.0);
	run_cylinders(Parameters, State, Controls, std::abs(Revolutions), CrankAngle, Deltatime);
	run_boiler(Parameters, State, Controls, Deltatime);
	run_fire(Parameters, State, work.firing, Deltatime);
}

double starting_force(parameters const &Parameters, double const Cutoff, double const Pressure)
{
	return rim_force(Parameters, mean_effective_pressure(Parameters, Cutoff, Pressure, atmosphere));
}

} // namespace steam
