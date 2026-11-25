#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
#include "consist.h"
#include "drive.h"

#include <algorithm>
#include <numbers>

#include "McZapkie/steam.h"

// the steam locomotive (McZapkie/steam.h): the steam tables, the cylinders and the boiler on their own, then the
// Ty2 of data/steam_ty2.json running a train. the figures it is held to are those of the Ty2 (DRB class 52): about
// 214 kN starting, 1192 kW indicated, 1200 t at 65 km/h on the level

using test::definition;
using test::vehicle;

namespace {

double const bar{1.0e5};

steam::parameters ty2() {
	steam::parameters parameters;
	parameters.wheel_diameter = 1.4;
	return parameters;
}

// runs the engine standing or turning at the given revolutions per second for Seconds, in steps of 0.05 s
void run(steam::parameters const &Parameters, steam::state &State, steam::controls const &Controls, double Revolutions, double Seconds) {
	auto angle{0.0};
	for (auto time{0.0}; time < Seconds; time += 0.05) {
		angle = std::fmod(angle + 2.0 * std::numbers::pi * Revolutions * 0.05, 2.0 * std::numbers::pi);
		steam::update(Parameters, State, Controls, Revolutions, angle, 0.05);
	}
}

double revolutions(double Kmh) {
	return Kmh / 3.6 / (std::numbers::pi * 1.4);
}

} // namespace

TEST_CASE("steam: the saturation line of the steam tables") {
	// 16 bar over the atmosphere boil at 204 °C, the atmosphere at 100 °C
	CHECK(steam::saturation::temperature(steam::atmosphere + 16.0 * bar) == doctest::Approx(204.3).epsilon(0.005));
	CHECK(steam::saturation::temperature(steam::atmosphere) == doctest::Approx(100.0).epsilon(0.005));
	for (auto temperature : {30.0, 100.0, 150.0, 204.0, 230.0}) {
		CAPTURE(temperature);
		CHECK(steam::saturation::temperature(steam::saturation::pressure(temperature)) == doctest::Approx(temperature).epsilon(0.001));
	}
	CHECK(steam::saturation::latent_heat(204.0) == doctest::Approx(1924.0e3).epsilon(0.01));
}

TEST_CASE("steam: the starting force of the Ty2") {
	auto const parameters{ty2()};
	// full gear and the boiler at its working pressure give the 214 kN of the DR's tables
	auto const force{steam::starting_force(parameters, parameters.cutoff_max, steam::atmosphere + parameters.working_pressure)};
	CHECK(force == doctest::Approx(214000.0).epsilon(0.03));
	// a shorter cutoff less, the steam expanding after it still doing much of the work
	auto const notched{steam::starting_force(parameters, 0.3, steam::atmosphere + parameters.working_pressure)};
	CHECK(notched > 0.6 * force);
	CHECK(notched < 0.75 * force);
	// the engine standing with the regulator open gives the same, swinging with the cranks
	auto state{steam::in_steam(parameters)};
	steam::controls controls;
	controls.regulator = 1.0;
	controls.cutoff = parameters.cutoff_max;
	auto lowest{1.0e9};
	auto highest{0.0};
	for (auto angle{0.0}; angle < 2.0 * std::numbers::pi; angle += 0.05) {
		steam::update(parameters, state, controls, 0.0, angle, 0.01);
		lowest = std::min(lowest, state.force);
		highest = std::max(highest, state.force);
	}
	auto const mean{steam::starting_force(parameters, parameters.cutoff_max, state.boiler_pressure)};
	CHECK(lowest == doctest::Approx(0.79 * mean).epsilon(0.02));
	CHECK(highest == doctest::Approx(1.11 * mean).epsilon(0.02));
}

TEST_CASE("steam: the Steam section of the definition") {
	steam::parameters parameters;
	steam::load(parameters, "Steam: CylD=0.63 CylS=0.66 BoilerPressure=14 InjectorFlow1=120 TenderWater=26 BackPressure=0.05");
	CHECK(parameters.bore == doctest::Approx(0.63));
	CHECK(parameters.working_pressure == doctest::Approx(14.0 * bar));
	CHECK(parameters.injector_flow[0] == doctest::Approx(2.0));
	CHECK(parameters.tender_water == doctest::Approx(26000.0));
	CHECK(parameters.back_pressure == doctest::Approx(0.05 * bar));
	// what isn't given keeps its default
	CHECK(parameters.cutoff_max == doctest::Approx(steam::parameters{}.cutoff_max));
}

TEST_CASE("steam: a locomotive standing in steam, looked after by the automatic fireman") {
	auto const parameters{ty2()};
	auto state{steam::in_steam(parameters)};
	steam::controls const controls;
	run(parameters, state, controls, 0.0, 1800.0);
	// the pressure kept near the working one, the water in the glass, the fire alive
	CHECK(state.pressure() > parameters.working_pressure - 1.5 * bar);
	CHECK(state.pressure() < parameters.working_pressure + 0.5 * bar);
	CHECK(state.water_level(parameters) > 0.3);
	CHECK(state.water_level(parameters) < 0.9);
	CHECK(state.fire_mass > 0.0);
	CHECK_FALSE(state.fusible_plug_melted);
}

TEST_CASE("steam: a fire left burning without feeding the boiler melts the fusible plug") {
	auto const parameters{ty2()};
	auto state{steam::in_steam(parameters)};
	steam::controls controls;
	controls.automatic_fireman = false;
	controls.blower = 1.0;
	controls.firing = 0.3;
	// the safety valves take the steam the fire makes, the water boils away
	auto melted{0.0};
	for (auto time{0.0}; time < 7200.0 && melted == 0.0; time += 10.0) {
		run(parameters, state, controls, 0.0, 10.0);
		if (state.fusible_plug_melted) {
			melted = time;
		}
	}
	REQUIRE(state.fusible_plug_melted);
	CHECK(state.water_level(parameters) < 0.0);
	// and its steam puts the fire out
	run(parameters, state, controls, 0.0, 300.0);
	CHECK(state.fire_mass < 1.0);
	CHECK(state.burn_rate == 0.0);
}

TEST_CASE("steam: a boiler filled over the glass primes, the water carried over doing no work") {
	auto const parameters{ty2()};
	steam::controls controls;
	controls.regulator = 1.0;
	controls.cutoff = 0.3;
	auto normal{steam::in_steam(parameters)};
	auto overfilled{steam::in_steam(parameters)};
	overfilled.water_mass += parameters.glass_mass;
	run(parameters, normal, controls, revolutions(30.0), 1.0);
	run(parameters, overfilled, controls, revolutions(30.0), 1.0);
	CHECK(normal.priming == 0.0);
	CHECK(overfilled.priming > 0.2);
	CHECK(overfilled.force < 0.8 * normal.force);
}

TEST_CASE("steam: the boiler limits the power the cylinders can keep up") {
	// at 65 km/h with the regulator wide open, the longest cutoff the boiler keeps over 13 bar for half an hour
	auto const parameters{ty2()};
	auto kept{0.0};
	auto power{0.0};
	for (auto cutoff{0.10}; cutoff <= 0.40; cutoff += 0.02) {
		auto state{steam::in_steam(parameters)};
		steam::controls controls;
		controls.regulator = 1.0;
		controls.cutoff = cutoff;
		run(parameters, state, controls, revolutions(65.0), 1800.0);
		if (state.pressure() < 13.0 * bar) {
			break;
		}
		kept = cutoff;
		power = state.power;
	}
	MESSAGE("65 km/h: cutoff " << kept << ", " << power / 1000.0 << " kW indicated");
	CHECK(kept >= 0.14);
	CHECK(kept <= 0.30);
	// about the 1192 kW of the Ty2
	CHECK(power > 950.0e3);
	CHECK(power < 1500.0e3);
}

namespace {

definition freight_wagon(double Mass) {
	return definition::load("wagon")
	    .set("Param", "M", Mass)
	    .set("Dimensions", "L", 10.0)
	    .set("Dimensions", "Cx", 0.3)
	    .set("Wheels", "BearingType", "Slide")
	    .set("Brake", "BrakeValve", "K");
}

// the Ty2 with its tender and Wagons of 50 t, freight wagons of its time on plain bearings, the train pipe held at
// running pressure, the brakes off
struct freight_train {
	test::consist train;
	vehicle *locomotive{nullptr};

	explicit freight_train(int Wagons) {
		locomotive = &train.add(definition::load("steam_ty2"), 1);
		train.add(freight_wagon(58700));
		for (auto wagon{0}; wagon < Wagons; ++wagon) {
			train.add(freight_wagon(50000));
		}
		auto &cab{mover()};
		cab.CabActivisation();
		locomotive->hold_brake_pipe(5.0);
		locomotive->hold_main_reservoir(8.0);
		test::release_brakes(cab);
		train.run(30.0);
	}

	TMoverParameters &mover() { return locomotive->mover(); }

	// a driver keeping the regulator wide open and notching up the cutoff as far as the boiler keeps its pressure
	void drive(double Seconds) {
		auto &cab{mover()};
		for (auto time{0.0}; time < Seconds; time += 5.0) {
			if (cab.Vel > 15.0 && cab.Steam.pressure() < 13.0 * bar) {
				cab.IncScndCtrl(1);
			}
			else if (cab.Steam.pressure() > 15.0 * bar && cab.ScndCtrlPos > 0) {
				cab.DecScndCtrl(1);
			}
			train.run(5.0);
		}
	}
};

} // namespace

TEST_CASE("steam: the Ty2 loads as a steam locomotive in steam") {
	vehicle locomotive{definition::load("steam_ty2"), 0.0, "test", 1};
	CHECK(locomotive->EngineType == TEngineType::SteamEngine);
	CHECK(locomotive->SteamParameters.wheel_diameter == doctest::Approx(1.4));
	CHECK(locomotive->Steam.pressure() > 15.0 * bar);
	CHECK(locomotive->steam_InSteam());
	// the master controller works the regulator, the secondary one notches up from full gear
	CHECK(locomotive->steam_Cutoff() == doctest::Approx(locomotive->SteamParameters.cutoff_max));
	locomotive->ScndCtrlPos = locomotive->ScndCtrlPosNo;
	CHECK(locomotive->steam_Cutoff() == doctest::Approx(locomotive->SteamParameters.cutoff_min));
}

TEST_CASE("steam: the Ty2 starts 1200 t and takes it up to speed on the level") {
	freight_train freight{24};
	auto &cab{freight.mover()};
	REQUIRE(cab.DirectionForward());
	while (cab.MainCtrlPos < cab.MainCtrlPosNo) {
		REQUIRE(cab.IncMainCtrl(1));
	}
	freight.train.run(10.0);
	// moving off, at full gear
	CHECK(cab.Vel > 1.0);
	freight.drive(1200.0);
	MESSAGE("after 20 min: " << cab.Vel << " km/h, " << cab.Steam.pressure() / bar << " bar, notch " << cab.ScndCtrlPos << ", "
	                         << cab.Steam.power / 1000.0 << " kW");
	// the 1200 t at about the 65 km/h of the Ty2's design
	CHECK(cab.Vel > 55.0);
	CHECK(cab.Vel < 75.0);
	CHECK(cab.Steam.pressure() > 11.0 * bar);
	CHECK(cab.Steam.water_level(cab.SteamParameters) > 0.0);
	// the cutoff notched up from full gear
	CHECK(cab.ScndCtrlPos > 0);
}

TEST_CASE("steam: in mid gear the Ty2 takes no steam and gives no force") {
	vehicle locomotive{definition::load("steam_ty2"), 0.0, "test", 1};
	locomotive->CabActivisation();
	locomotive.hold_brake_pipe(5.0);
	locomotive.hold_main_reservoir(8.0);
	test::release_brakes(locomotive.mover());
	locomotive->MainCtrlPos = locomotive->MainCtrlPosNo;
	locomotive.run(10.0);
	CHECK(locomotive->DirActive == 0);
	CHECK(locomotive->Vel < 0.01);
	CHECK(locomotive->Steam.cylinder_flow == 0.0);
	CHECK(locomotive->Ft == 0.0);
}

TEST_CASE("steam: the Ty23 of its drawings, with no more than its data") {
	// Stacja Muzeum, Ty23 drawings 1.03/1.05 (main data, from the 630th locomotive) and 21.03 (valve gear results):
	// 2 cylinders 650 x 720 mm, driving wheels 1450 mm, 14 at, full gear 84 % forwards and 70 % backwards, and the
	// tractive force of the drawing Z = 22 030 kg = 0.75 p d^2 s / D, which the DR's tables for the Ty2 exceed by 7 %
	steam::parameters parameters;
	steam::load(parameters, "Steam: CylD=0.65 CylS=0.72 CutoffMax=0.84 CutoffMaxReverse=0.70 BoilerPressure=13.73 Grate=4.5");
	parameters.wheel_diameter = 1.45;
	CHECK(parameters.cutoff_max_reverse == doctest::Approx(0.70));
	auto const force{steam::starting_force(parameters, parameters.cutoff_max, steam::atmosphere + parameters.working_pressure)};
	auto const drawing{22030.0 * 9.81};
	MESSAGE("Ty23 starting force " << force / 1000.0 << " kN, the drawing's Z " << drawing / 1000.0 << " kN");
	CHECK(force > drawing);
	CHECK(force < 1.10 * drawing);
	// the same against the Ty2 as the formula has them: the Ty23 has the larger cylinders, the Ty2 the higher pressure
	auto const ty2parameters{ty2()};
	auto const ty2force{steam::starting_force(ty2parameters, ty2parameters.cutoff_max, steam::atmosphere + ty2parameters.working_pressure)};
	auto const formula = [](double Pressure, double Bore, double Stroke, double Wheel) { return Pressure * Bore * Bore * Stroke / Wheel; };
	CHECK(force / ty2force == doctest::Approx(formula(13.73, 0.65, 0.72, 1.45) / formula(16.0, 0.60, 0.66, 1.40)).epsilon(0.03));
	// backwards, the shorter full gear gives less
	auto const backwards{steam::starting_force(parameters, parameters.cutoff_max_reverse, steam::atmosphere + parameters.working_pressure)};
	CHECK(backwards < force);
	CHECK(backwards > 0.85 * force);
}
