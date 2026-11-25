/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "McZapkie/steam/engine.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace steam::detailed
{

namespace
{

constexpr double turn{2.0 * std::numbers::pi};
constexpr double quarter_pi{std::numbers::pi / 4.0};

constexpr double regulator_discharge{0.9};
constexpr double port_equalising{1.25}; // how much of the pressure difference a flow between two spaces may level in a substep
constexpr double snifting_valve_area{3.0e-3}; // m², of the superheater's snifting valve
constexpr double snifting_valve_opening{2000.0}; // Pa under the atmosphere
constexpr double ambient_steam_volume{1.673}; // m³/kg, what the snifting valve lets in is taken for steam of the atmosphere
constexpr double ambient_steam_enthalpy{2676.0e3};
constexpr double relief_valve_area{8.0e-4}; // m², of a cylinder relief valve
constexpr double atmosphere_discharge{0.7};
constexpr double outflow_share{0.3}; // of the steam of a space that may leave it in a substep
constexpr double condensation{6000.0}; // W/(m² K), steam condensing on a wall colder than itself
constexpr double wall_diffusivity{1.2e-5}; // m²/s, cast iron
constexpr double wall_volumetric_heat{7200.0 * 540.0}; // J/(m³ K)
constexpr double wall_conductivity{50.0}; // W/(m K)
constexpr double ring_leak_area{3.0e-6}; // m², past the piston rings
constexpr double equaliser_area{6.0e-3}; // m², of the pressure equaliser joining the ends of a cylinder
constexpr double equaliser_pressure{1.5 * bar}; // of the steam chests, under which it opens
constexpr double leak_discharge{0.7};
constexpr double nozzle_discharge{0.95};
constexpr double standstill{1.0e-3}; // rad/s

double radians(double const Degrees)
{
	return Degrees * std::numbers::pi / 180.0;
}

// the flow from one space to another, + from the first to the second, and the enthalpy it carries
struct transfer
{
	double flow;
	double enthalpy;
};

// the flow through the section between two spaces, held back so that an explicit step doesn't overshoot the
// levelling of their pressures
transfer flow_between(orifice const &Orifice, control_volume const &First, control_volume const &Second, double const Step)
{
	if (Orifice.area <= 0.0)
	{
		return {0.0, First.steam.enthalpy};
	}
	auto const forwards{First.steam.pressure >= Second.steam.pressure};
	auto const &from{forwards ? First : Second};
	auto const &to{forwards ? Second : First};
	auto const free{orifice_flow(Orifice, {from.steam.pressure, from.specific_volume, from.steam.dryness}, to.steam.pressure)};
	auto const levelling{(from.steam.pressure - to.steam.pressure) / (port_equalising * from.steam.pressure / from.mass + port_equalising * to.steam.pressure / to.mass)};
	auto const flow{std::min(free, 0.5 * levelling / Step)};
	return {forwards ? flow : -flow, from.steam.enthalpy};
}

} // namespace

void control_volume::fill(double const Pressure, double const Temperature, double const Dryness)
{
	auto const line{saturation(Pressure)};
	auto specificenergy{line.liquid_energy + Dryness * (line.vapour_energy - line.liquid_energy)};
	auto specificvolume{line.liquid_volume + Dryness * (line.vapour_volume - line.liquid_volume)};
	if (Temperature > line.temperature)
	{
		auto const dry{superheated_steam(Pressure, Temperature)};
		specificenergy = dry.energy;
		specificvolume = dry.volume;
	}
	mass = volume / specificvolume;
	energy = mass * specificenergy;
	update();
}

void control_volume::update()
{
	specific_volume = volume / mass;
	steam = state_from_energy_and_volume(energy / mass, specific_volume, steam.pressure);
}

engine::engine(parameters const &Parameters)
    : m_parameters{Parameters}, m_gears{valve_gear{Parameters.valve_gear, Parameters.cylinders}, valve_gear{Parameters.valve_gear, Parameters.cylinders}}, m_mid_gear{0.0},
      m_front_area{quarter_pi * (Parameters.cylinders.bore * Parameters.cylinders.bore - Parameters.cylinders.front_rod * Parameters.cylinders.front_rod)},
      m_back_area{quarter_pi * (Parameters.cylinders.bore * Parameters.cylinders.bore - Parameters.cylinders.back_rod * Parameters.cylinders.back_rod)},
      m_clearance{Parameters.cylinders.clearance * quarter_pi * Parameters.cylinders.bore * Parameters.cylinders.bore * Parameters.cylinders.stroke},
      m_crank_offset{radians(Parameters.cylinders.crank_angle)},
      m_superheater_surface{Parameters.superheater.elements * std::numbers::pi * Parameters.superheater.pipe_bore * Parameters.superheater.steam_path}
{
	m_mid_gear = mid_gear(m_gears[0]);
	auto const &ambient{m_parameters.ambient};
	chest.volume = m_parameters.superheater.steam_volume + 2.0 * m_parameters.regulator.steam_chest_volume;
	chest.fill(ambient.pressure, 400.0);
	exhaust.volume = m_parameters.draught.exhaust_volume;
	exhaust.fill(ambient.pressure, 380.0);
	set_volumes();
	for (auto &chamber : chambers)
	{
		chamber.fill(ambient.pressure, 380.0);
	}
}

void engine::warm_up(double const Walls, double const ChestPressure, double const ChestTemperature)
{
	wall_temperature.fill(Walls);
	block_temperature.fill(Walls);
	if (ChestPressure > 0.0)
	{
		chest.fill(ChestPressure, ChestTemperature);
	}
}

void engine::set_volumes()
{
	for (std::size_t side{0}; side < 2; ++side)
	{
		auto const piston{m_gears[side].piston(crank - static_cast<double>(side) * m_crank_offset)};
		chambers[2 * side].volume = m_clearance + m_front_area * piston;
		chambers[2 * side + 1].volume = m_clearance + m_back_area * (m_parameters.cylinders.stroke - piston);
	}
}

void engine::set_ambient(ambient_parameters const &Ambient)
{
	m_parameters.ambient = Ambient;
}

double engine::regulator_area(double const Regulator) const
{
	if (Regulator <= 0.0)
	{
		return 0.0;
	}
	auto const &regulator{m_parameters.regulator};
	auto const &superheater{m_parameters.superheater};
	// the pilot valve first, taking the load off the main one
	auto area{quarter_pi * regulator.pilot_valve * regulator.pilot_valve * std::min(Regulator / regulator.pilot_travel, 1.0)};
	if (Regulator > regulator.pilot_travel)
	{
		auto const lift{(Regulator - regulator.pilot_travel) / (1.0 - regulator.pilot_travel) * regulator.lift};
		area += std::min(std::numbers::pi * regulator.main_valve * lift, quarter_pi * regulator.main_valve * regulator.main_valve);
	}
	// in a row with it the steam pipe and the pipes of the superheater, with their friction and bends
	auto const pipe{quarter_pi * regulator.steam_pipe * regulator.steam_pipe};
	auto const superheaterloss{0.02 * superheater.steam_path / superheater.pipe_bore + 4.0};
	return 1.0 / std::sqrt(1.0 / (area * area) + 1.0 / (pipe * pipe) + (1.0 + superheaterloss) / (superheater.steam_section * superheater.steam_section));
}

// the steam through the regulator, heated in the superheater on its way to the steam chests
void engine::admit(stepping const &Stepping, engine_input const &Input, rates &Rates, tally &Tally) const
{
	constexpr double still_convection{12.0}; // W/(m² K), with no steam flowing
	constexpr double steam_heat{2300.0}; // J/(kg K)
	constexpr double steam_viscosity{2.0e-5};
	constexpr double steam_conductivity{0.045};
	auto const &superheater{m_parameters.superheater};
	auto const flow{Input.boiler_pressure > chest.steam.pressure ? orifice_flow({Stepping.regulator_area, regulator_discharge}, {Input.boiler_pressure, Input.steam_volume}, chest.steam.pressure) :
	                                                               0.0};
	auto heat{still_convection * m_superheater_surface * (Input.superheater_temperature - chest.steam.temperature)};
	if (flow > 1.0e-6)
	{
		auto const elements{static_cast<double>(superheater.elements)};
		auto const reynolds{(flow / elements) * superheater.pipe_bore / (superheater.steam_section / elements * steam_viscosity)};
		auto const transfercoefficient{0.023 * std::pow(reynolds, 0.8) * steam_conductivity / superheater.pipe_bore};
		heat +=
		    flow * steam_heat * (Input.superheater_temperature - saturation_temperature(Input.boiler_pressure)) * (1.0 - std::exp(-transfercoefficient * m_superheater_surface / (flow * steam_heat)));
	}
	Rates.chest_mass += flow;
	Rates.chest_energy += flow * Input.steam_enthalpy + heat;
	// drifting, the pistons pump the steam chests out and the snifting valve lets the atmosphere in
	if (auto const ambient{m_parameters.ambient.pressure}; chest.steam.pressure < ambient - snifting_valve_opening)
	{
		auto const drawn{orifice_flow({snifting_valve_area, atmosphere_discharge}, {ambient, ambient_steam_volume}, chest.steam.pressure)};
		Rates.chest_mass += drawn;
		Rates.chest_energy += drawn * ambient_steam_enthalpy;
	}
	Tally.regulator += flow * Stepping.step;
	Tally.superheater_heat += heat * Stepping.step;
}

// the steam into and out of one end of a cylinder: through its port from the steam chest and to the exhaust, and
// to the atmosphere through the cylinder cock and the relief valve
void engine::chamber_flows(stepping const &Stepping, cylinder_state const &Cylinder, std::size_t const End, rates &Rates, tally &Tally) const
{
	auto const index{2 * Cylinder.cylinder + End};
	auto const &chamber{chambers[index]};
	auto const &gear{m_parameters.valve_gear};
	auto const admissionopening{End == 0 ? Cylinder.ports.front_admission : Cylinder.ports.back_admission};
	auto const exhaustopening{End == 0 ? Cylinder.ports.front_exhaust : Cylinder.ports.back_exhaust};
	// either way: steam compressed over the chest pressure goes back into the chest
	auto const admitted{flow_between({gear.port_discharge * gear.port_length * admissionopening}, chest, chamber, Stepping.step)};
	Rates.mass[index] += admitted.flow;
	Rates.energy[index] += admitted.flow * admitted.enthalpy;
	Rates.chest_mass -= admitted.flow;
	Rates.chest_energy -= admitted.flow * admitted.enthalpy;
	Tally.cylinders += admitted.flow * Stepping.step;
	auto const released{flow_between({gear.port_discharge * gear.port_length * exhaustopening}, chamber, exhaust, Stepping.step)};
	Rates.mass[index] -= released.flow;
	Rates.energy[index] -= released.flow * released.enthalpy;
	Rates.exhaust_mass += released.flow;
	Rates.exhaust_energy += released.flow * released.enthalpy;
	auto const ambient{m_parameters.ambient.pressure};
	auto const relief{chamber.steam.pressure > m_parameters.cylinders.relief_pressure + ambient ? relief_valve_area : 0.0};
	auto const open{Stepping.cocks_area + relief};
	if (open > 0.0 && chamber.steam.pressure > ambient)
	{
		auto const blown{
		    std::min(orifice_flow({open, atmosphere_discharge}, {chamber.steam.pressure, chamber.specific_volume, chamber.steam.dryness}, ambient), outflow_share * chamber.mass / Stepping.step)};
		Rates.mass[index] -= blown;
		Rates.energy[index] -= blown * chamber.steam.enthalpy;
		Tally.cocks += blown * Stepping.step;
	}
}

// the heat between the steam of one end of a cylinder and its wall: a skin as deep as the heat wave of a revolution
// gets, on the casting behind it
void engine::wall_exchange(stepping const &Stepping, cylinder_state const &Cylinder, std::size_t const End, rates &Rates, tally &Tally)
{
	constexpr double covers_and_passages{0.25}; // m²
	constexpr double block_loss{8.0 * 2.5 / 2.0}; // W/K, of half a cylinder casting to the air
	auto const index{2 * Cylinder.cylinder + End};
	auto const &chamber{chambers[index]};
	auto const &cylinders{m_parameters.cylinders};
	auto const barrel{End == 0 ? Cylinder.piston : cylinders.stroke - Cylinder.piston};
	auto const surface{2.0 * quarter_pi * cylinders.bore * cylinders.bore + covers_and_passages + std::numbers::pi * cylinders.bore * barrel};
	auto &wall{wall_temperature[index]};
	auto &block{block_temperature[Cylinder.cylinder]};
	// steam condenses on a wall colder than itself; otherwise the convection after Eichelberg
	auto const condensing{wall < saturation_temperature(chamber.steam.pressure) - 0.5};
	auto const transfercoefficient{condensing ? condensation :
	                                            2.44 * std::cbrt(std::max(Stepping.piston_speed, 0.3)) * std::sqrt(chamber.steam.pressure / technical_atmosphere * chamber.steam.temperature)};
	auto const heat{transfercoefficient * surface * (wall - chamber.steam.temperature)};
	Rates.energy[index] += heat;
	Tally.wall_heat -= heat * Stepping.step;
	// the skin is stiffly tied to the casting, so its step is implicit
	auto const skinheat{wall_volumetric_heat * Stepping.penetration * surface};
	auto const conductance{wall_conductivity * surface / Stepping.penetration};
	auto const blockbefore{block};
	wall = (wall + Stepping.step / skinheat * (conductance * blockbefore - heat)) / (1.0 + Stepping.step / skinheat * conductance);
	block += Stepping.step / (cylinders.wall_mass * cylinders.wall_heat) * (conductance * (wall - blockbefore) - block_loss * (blockbefore - m_parameters.ambient.temperature));
}

// one cylinder in a substep: the flows of both its ends, the leak past the piston, the forces and the work
engine::torques engine::cylinder(stepping const &Stepping, cylinder_state const &Cylinder, rates &Rates, tally &Tally)
{
	constexpr double ring_friction{600.0}; // N, of the piston rings and glands
	constexpr double gland_friction{0.03}; // of the steam force
	constexpr double slide_friction{0.06}; // of the force on the crosshead guides
	constexpr double journal_friction{0.03 * (0.09 + 0.11)}; // m: the coefficient by the radii of the crank pin and of the axle journals
	constexpr double valve_gear_drive{60.0}; // N m
	for (std::size_t end{0}; end < 2; ++end)
	{
		chamber_flows(Stepping, Cylinder, end, Rates, Tally);
		wall_exchange(Stepping, Cylinder, end, Rates, Tally);
	}
	auto const front{2 * Cylinder.cylinder};
	auto const back{front + 1};
	// past the piston rings, and through the pressure equaliser with the regulator shut
	auto const leakarea{ring_leak_area + (chest.steam.pressure < equaliser_pressure ? equaliser_area : 0.0)};
	auto const leak{flow_between({leakarea, leak_discharge}, chambers[front], chambers[back], Stepping.step)};
	Rates.mass[front] -= leak.flow;
	Rates.energy[front] -= leak.flow * leak.enthalpy;
	Rates.mass[back] += leak.flow;
	Rates.energy[back] += leak.flow * leak.enthalpy;
	// the piston's way is counted from the front dead centre, so the steam of the front end pushes it along it
	auto const &cylinders{m_parameters.cylinders};
	auto const ambient{m_parameters.ambient.pressure};
	auto const frontpressure{chambers[front].steam.pressure};
	auto const backpressure{chambers[back].steam.pressure};
	auto const steamforce{(frontpressure - ambient) * m_front_area - (backpressure - ambient) * m_back_area};
	Tally.work += steamforce * Cylinder.piston_rate * Stepping.omega * Stepping.step;
	// the inertia of the reciprocating masses, by the first two harmonics of the piston's acceleration
	auto const acceleration{cylinders.crank * std::cos(Cylinder.crank) + cylinders.crank * cylinders.crank * std::cos(2.0 * Cylinder.crank) / cylinders.connecting_rod};
	auto const rodforce{steamforce - cylinders.reciprocating_mass * acceleration * Stepping.omega * Stepping.omega};
	auto const obliquity{cylinders.crank * std::sin(Cylinder.crank) / cylinders.connecting_rod};
	auto const guideforce{std::abs(rodforce) * obliquity / std::sqrt(1.0 - obliquity * obliquity)};
	auto const sliding{ring_friction + gland_friction * std::abs(steamforce) + slide_friction * std::abs(guideforce)};
	// the work of the steam on the piston
	Rates.energy[front] -= frontpressure * m_front_area * Cylinder.piston_rate * Stepping.omega;
	Rates.energy[back] += backpressure * m_back_area * Cylinder.piston_rate * Stepping.omega;
	return {rodforce * Cylinder.piston_rate, sliding * std::abs(Cylinder.piston_rate) + journal_friction * std::abs(rodforce) + valve_gear_drive};
}

// the exhaust steam out of the blast pipe into the smokebox
void engine::blast(stepping const &Stepping, double const SmokeboxPressure, rates &Rates, tally &Tally) const
{
	auto const &steam{exhaust.steam};
	auto const free{orifice_flow({m_parameters.draught.nozzle_area, nozzle_discharge}, {steam.pressure, exhaust.specific_volume, steam.dryness}, SmokeboxPressure)};
	auto const exponent{isentropic_exponent(steam.dryness)};
	auto const critical{std::pow(2.0 / (exponent + 1.0), exponent / (exponent - 1.0))};
	auto const ratio{steam.pressure > SmokeboxPressure ? std::max(SmokeboxPressure / steam.pressure, critical) : 1.0};
	auto const velocity{std::sqrt(std::max(2.0 * exponent / (exponent - 1.0) * steam.pressure * exhaust.specific_volume * (1.0 - std::pow(ratio, (exponent - 1.0) / exponent)), 0.0))};
	auto const flow{std::min(free, outflow_share * exhaust.mass / Stepping.step)};
	Rates.exhaust_mass -= flow;
	Rates.exhaust_energy -= flow * steam.enthalpy;
	Tally.nozzle += flow * Stepping.step;
	Tally.momentum += flow * velocity * Stepping.step;
	Tally.nozzle_enthalpy += flow * steam.enthalpy * Stepping.step;
}

// explicit Euler, the substep being short
void engine::integrate(stepping const &Stepping, rates const &Rates)
{
	constexpr double least_mass{1.0e-6};
	chest.mass += Rates.chest_mass * Stepping.step;
	chest.energy += Rates.chest_energy * Stepping.step;
	exhaust.mass += Rates.exhaust_mass * Stepping.step;
	exhaust.energy += Rates.exhaust_energy * Stepping.step;
	crank += Stepping.omega * Stepping.step;
	set_volumes();
	for (std::size_t index{0}; index < chambers.size(); ++index)
	{
		auto &chamber{chambers[index]};
		chamber.mass = std::max(chamber.mass + Rates.mass[index] * Stepping.step, least_mass);
		chamber.energy += Rates.energy[index] * Stepping.step;
		chamber.update();
	}
	chest.update();
	exhaust.update();
}

void engine::substep(stepping const &Stepping, engine_input const &Input, tally &Tally)
{
	rates changes;
	admit(Stepping, Input, changes, Tally);
	torques sum;
	for (std::size_t side{0}; side < 2; ++side)
	{
		auto const sidecrank{crank - static_cast<double>(side) * m_crank_offset};
		auto const position{m_gears[side].follow(sidecrank, Stepping.weighshaft)};
		if (side == 0)
		{
			m_die_block = position.die_block;
		}
		cylinder_state const state{side, sidecrank, m_gears[side].piston(sidecrank), m_gears[side].piston_rate(sidecrank), m_gears[side].openings(position.valve)};
		auto const part{cylinder(Stepping, state, changes, Tally)};
		sum.gas += part.gas;
		sum.friction += part.friction;
	}
	blast(Stepping, Input.smokebox_pressure, changes, Tally);
	integrate(Stepping, changes);
	// the friction of the mechanism works against the turning; standing, it is left out
	auto const turning{std::abs(Stepping.omega) > standstill};
	auto const direction{Stepping.omega >= 0.0 ? 1.0 : -1.0};
	Tally.torque += (turning ? sum.gas - direction * sum.friction : sum.gas) * Stepping.step;
	Tally.friction += sum.friction * Stepping.step;
}

engine_output engine::step(double const Deltatime, double const Omega, engine_input const &Input)
{
	constexpr double crank_stride{0.5 * std::numbers::pi / 180.0};
	constexpr double time_stride{3.0e-4};
	constexpr double slowest{2.0}; // rad/s, for the depth the heat gets into the walls
	auto const substeps{std::max(1, static_cast<int>(std::ceil(std::max(std::abs(Omega) * Deltatime / crank_stride, Deltatime / time_stride))))};
	auto const &cylinders{m_parameters.cylinders};
	stepping const plan{Deltatime / substeps,
	                    Omega,
	                    weighshaft_angle(m_parameters.valve_gear, m_mid_gear, Input.reverser),
	                    regulator_area(Input.regulator),
	                    Input.cylinder_cocks ? quarter_pi * cylinders.drain_cock * cylinders.drain_cock : 0.0,
	                    std::sqrt(2.0 * wall_diffusivity / std::max(std::abs(Omega), slowest)),
	                    2.0 * cylinders.stroke * std::abs(Omega) / turn};
	tally sums;
	for (auto index{0}; index < substeps; ++index)
	{
		substep(plan, Input, sums);
	}
	crank = std::fmod(std::fmod(crank, turn) + turn, turn);
	engine_output output;
	output.torque = sums.torque / Deltatime;
	output.friction_torque = sums.friction / Deltatime;
	output.rim_force = output.torque / (m_parameters.wheel_diameter / 2.0);
	output.indicated_power = sums.work / Deltatime;
	output.regulator_flow = sums.regulator / Deltatime;
	output.cylinder_flow = sums.cylinders / Deltatime;
	output.nozzle_flow = sums.nozzle / Deltatime;
	output.nozzle_velocity = sums.nozzle > 1.0e-9 * Deltatime ? sums.momentum / sums.nozzle : 0.0;
	output.nozzle_enthalpy_flow = sums.nozzle_enthalpy / Deltatime;
	output.cocks_flow = sums.cocks / Deltatime;
	output.superheater_heat = sums.superheater_heat / Deltatime;
	output.wall_heat = sums.wall_heat / Deltatime;
	output.chest_pressure = chest.steam.pressure;
	output.chest_temperature = chest.steam.temperature;
	output.exhaust_pressure = exhaust.steam.pressure;
	output.exhaust_temperature = exhaust.steam.temperature;
	output.exhaust_dryness = exhaust.steam.dryness;
	output.exhaust_volume = exhaust.specific_volume;
	for (std::size_t index{0}; index < chambers.size(); ++index)
	{
		output.chamber_pressures[index] = chambers[index].steam.pressure;
	}
	output.crank = crank;
	output.weighshaft = plan.weighshaft;
	output.die_block = m_die_block;
	return output;
}

} // namespace steam::detailed
