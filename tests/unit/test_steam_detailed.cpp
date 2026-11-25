#include "stdafx.h"
#include "doctest.h"

#include <cmath>
#include <numbers>

#include "McZapkie/steam/boiler.h"
#include "McZapkie/steam/engine.h"
#include "McZapkie/steam/locomotive.h"
#include "McZapkie/steam/properties.h"
#include "McZapkie/steam/valve_gear.h"

// the detailed (crank angle resolved) model of a steam locomotive, McZapkie/steam/*. it stands apart from the vehicle (nothing
// of it is in Mover.cpp), so it is tested on its own. the reference figures come from the Python model of the Pt31
// (fizyka/model) that the C++ code is a port of

namespace
{

using namespace steam::detailed;

constexpr double pi{std::numbers::pi};

double relative_error(double const Value, double const Reference)
{
	return std::abs(Value - Reference) / std::abs(Reference);
}

// the mean figures of one revolution of the engine on steam of the given state
struct revolution
{
	double torque{0.0};
	double regulator_flow{0.0};
	double indicated_power{0.0};
	double chest_pressure{0.0};
};

revolution run_engine(double const Speed, double const Reverser)
{
	constexpr int steps{24};
	constexpr int revolutions{3};
	constexpr double boiler_pressure{15.72325e5};
	parameters const data;
	engine drive{data};
	drive.warm_up(505.0, 14.5e5, 620.0);
	auto const steam{saturation(boiler_pressure)};
	auto const omega{Speed / 3.6 / (data.wheel_diameter / 2.0)};
	auto const period{2.0 * pi / std::abs(omega)};
	revolution mean;
	for (auto turn{0}; turn < revolutions; ++turn)
	{
		mean = {};
		for (auto step{0}; step < steps; ++step)
		{
			engine_input input;
			input.regulator = Reverser != 0.0 ? 1.0 : 0.0;
			input.reverser = Reverser;
			input.boiler_pressure = boiler_pressure;
			input.steam_volume = steam.vapour_volume;
			input.steam_enthalpy = steam.vapour_enthalpy;
			input.superheater_temperature = 640.0;
			input.smokebox_pressure = 1.0e5;
			auto const output{drive.step(period / steps, omega, input)};
			mean.torque += output.torque / steps;
			mean.regulator_flow += output.regulator_flow / steps;
			mean.indicated_power += output.indicated_power / steps;
			mean.chest_pressure = output.chest_pressure;
		}
	}
	return mean;
}

} // namespace

TEST_CASE("steam tables: the saturation line agrees with IAPWS-IF97")
{
	// the boiling point at 1 bar and at 10 bar, K
	CHECK(saturation_temperature(1.0e5) == doctest::Approx(372.756).epsilon(1.0e-4));
	CHECK(saturation_temperature(10.0e5) == doctest::Approx(453.03).epsilon(1.0e-4));
	// and the other way round
	CHECK(saturation_pressure(saturation_temperature(15.0e5)) == doctest::Approx(15.0e5).epsilon(1.0e-5));
	auto const line{saturation(10.0e5)};
	CHECK(line.liquid_volume == doctest::Approx(0.0011273).epsilon(1.0e-3));
	CHECK(line.vapour_volume == doctest::Approx(0.19444).epsilon(1.0e-3));
	CHECK(line.vapour_enthalpy - line.liquid_enthalpy == doctest::Approx(2.0153e6).epsilon(2.0e-3));
}

TEST_CASE("steam tables: superheating lifts the volume and the enthalpy of the steam")
{
	auto const line{saturation(14.0e5)};
	auto const hot{superheated_steam(14.0e5, line.temperature + 100.0)};
	CHECK(hot.volume > 1.15 * line.vapour_volume);
	CHECK(hot.enthalpy > line.vapour_enthalpy + 1.9e5);
}

TEST_CASE("valve gear: the cutoff found for the weigh shaft is the one asked for, and the gear reverses")
{
	parameters const data;
	valve_gear gear{data.valve_gear, data.cylinders};
	for (auto const cutoff : {30.0, 50.0, 80.0})
	{
		auto const forward{valve_timing(gear, weighshaft_for_cutoff(gear, cutoff, true))};
		// the front end of the cylinder is the one the cutoff is asked for
		CHECK(forward.forward);
		CHECK(forward.cutoff[0] == doctest::Approx(cutoff).epsilon(0.005));
		CHECK(std::abs(forward.cutoff[1] - cutoff) < 4.0);
		// the events, in per cent of the stroke: the steam is let out of the cylinder before it is shut in again for the compression, and admitted just before the dead centre
		CHECK(forward.release[0] > forward.compression[0]);
		CHECK(forward.admission[0] > 95.0);
		auto const backward{valve_timing(gear, weighshaft_for_cutoff(gear, cutoff, false))};
		CHECK_FALSE(backward.forward);
		CHECK(backward.cutoff[0] == doctest::Approx(cutoff).epsilon(0.005));
	}
}

TEST_CASE("valve gear: marching along the crank angle gives the same position as following it")
{
	parameters const data;
	valve_gear gear{data.valve_gear, data.cylinders};
	auto const target{gear.follow(4.0, -0.2)};
	gear.reset();
	valve_position marched{};
	for (auto step{0}; step <= 400; ++step)
	{
		marched = gear.solve(4.0 * step / 400, -0.2);
	}
	CHECK(marched.valve == doctest::Approx(target.valve).epsilon(1.0e-6));
}

TEST_CASE("engine: the mean figures agree with the reference model at 60 km/h with 27.8% steps of the reverser")
{
	auto const mean{run_engine(60.0, 0.278)};
	CHECK(relative_error(mean.torque, 62352.7) < 0.01);
	CHECK(relative_error(mean.regulator_flow, 2.64389) < 0.01);
	CHECK(relative_error(mean.indicated_power, 1.228e6) < 0.01);
	CHECK(relative_error(mean.chest_pressure, 1.56126e6 - 0.0) < 0.05);
}

TEST_CASE("engine: full gear at a crawl gives the greatest torque, the reverse gear the opposite sign")
{
	auto const slow{run_engine(3.0, 1.0)};
	CHECK(relative_error(slow.torque, 175428.0) < 0.01);
	auto const reversed{run_engine(-30.0, -0.5)};
	CHECK(reversed.torque < 0.0);
	CHECK(relative_error(reversed.torque, -118155.0) < 0.01);
}

TEST_CASE("engine: with the regulator shut the engine takes air and brakes the wheels")
{
	auto const idle{run_engine(50.0, 0.0)};
	CHECK(idle.regulator_flow == 0.0);
	CHECK(idle.torque < 0.0);
}

TEST_CASE("boiler: the fire, the draught and the heat exchange agree with the steady state of the reference model")
{
	parameters const data;
	boiler fire{data};
	fire.set_state(14.7124311480545 * bar + data.ambient.pressure, 9.497719320178385);
	fire.fuel_mass = 432.71918718087386;
	fire.clinker_mass = 0.012287518053391552;
	fire.air_flow = 4.554070177742238;
	fire.superheater_temperature = 344.71654154353615 + kelvin;
	boiler_input input;
	input.regulator_flow = 2.2152576566626885;
	input.auxiliary_flow = 0.02;
	input.nozzle_flow = 2.209533762842864;
	input.nozzle_velocity = 247.0;
	input.nozzle_volume = 3.4;
	input.firing = 0.4477;
	// the reference point is the state after a step, so the figures agree to a fraction of a per cent
	auto const output{fire.step(1.0e-6, input)};
	CHECK(relative_error(output.fire.burnt, 0.44767) < 0.002);
	CHECK(relative_error(output.fire.adiabatic_temperature - kelvin, 1670.76) < 0.002);
	CHECK(relative_error(output.fire.firebox_exit_temperature - kelvin, 1044.15) < 0.005);
	CHECK(relative_error(output.fire.smokebox_temperature - kelvin, 317.42) < 0.005);
	CHECK(relative_error(output.fire.gas_flow, 5.44244) < 0.002);
	CHECK(relative_error(output.fire.fire_heat / 1.0e3, 10332.4) < 0.005);
	CHECK(relative_error(output.fire.excess_air, 1.44984) < 0.002);
	CHECK(relative_error(output.heat_to_water / 1.0e3, 8065.0) < 0.005);
	CHECK(relative_error(output.smokebox_vacuum, 756.3) < 0.02);
	// at the working pressure the safety valves are blowing
	CHECK(output.safety_valve_flow > 1.0);
}

TEST_CASE("boiler: the water and the steam keep the mass and the energy of the boiler")
{
	parameters const data;
	boiler tank{data};
	tank.set_state(10.0 * bar + data.ambient.pressure, data.boiler.water_volume_normal);
	auto const mass{tank.mass};
	boiler_input input;
	input.regulator_flow = 1.0;
	input.dampers = 1.0;
	tank.step(1.0, input);
	// a second of taking a kilogram of steam out of a boiler with no feed
	CHECK(tank.mass == doctest::Approx(mass - 1.0).epsilon(1.0e-6));
	CHECK(tank.pressure < 10.0 * bar + data.ambient.pressure);
}

TEST_CASE("locomotive: a few seconds under steam keep the stores and the pressure sane")
{
	locomotive engine;
	locomotive_controls controls;
	controls.regulator = 0.5;
	controls.reverser = 0.5;
	controls.live_steam_injector = 1.0;
	controls.firing = 0.4;
	locomotive_output output;
	auto const water{engine.tender_water};
	for (auto step{0}; step < 200; ++step)
	{
		output = engine.step(0.05, 3.0, controls, {});
	}
	CHECK(std::isfinite(output.rim_force));
	CHECK(output.boiler_pressure > 0.0);
	CHECK(output.boiler_pressure < 20.0);
	CHECK(output.tender_water < water);
	CHECK(output.locomotive_mass > 90000.0);
	CHECK(output.tender_mass > 0.0);
}
