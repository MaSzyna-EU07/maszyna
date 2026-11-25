#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
#include "consist.h"
#include "drive.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <string_view>

#include "McZapkie/steam.h"

// the steam locomotive (McZapkie/steam.h): the steam tables, the cylinders and the boiler on their own, then the
// Ty2 of data/steam_ty2.json running a train. the figures it is held to are those of the Ty2 (DRB class 52): about
// 214 kN starting, 1192 kW indicated, 1200 t at 65 km/h on the level

using test::definition;
using test::vehicle;

namespace {

double const bar{1.0e5};

// the engine of the Ty2, as the defaults have it, on its 1.4 m driving wheels
SteamEngine ty2() {
	SteamEngine engine;
	engine.Parameters.wheel_diameter = 1.4;
	return engine;
}

// the same, in steam
SteamEngine ty2_in_steam() {
	auto engine{ty2()};
	engine.raise_steam();
	return engine;
}

// runs the engine standing or turning at the given revolutions per second for Seconds, in steps of 0.05 s
void run(SteamEngine &Engine, double Revolutions, double Seconds) {
	auto const steps{std::lround(Seconds / 0.05)};
	for (auto step{0L}; step < steps; ++step) {
		Engine.update(Revolutions, 0.05);
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
	auto engine{ty2_in_steam()};
	auto const &parameters{engine.Parameters};
	// full gear and the boiler at its working pressure give the 214 kN of the DR's tables
	auto const force{engine.starting_force(parameters.cutoff_max, steam::atmosphere + parameters.working_pressure)};
	CHECK(force == doctest::Approx(214000.0).epsilon(0.03));
	// a shorter cutoff less, the steam expanding after it still doing much of the work
	auto const notched{engine.starting_force(0.3, steam::atmosphere + parameters.working_pressure)};
	CHECK(notched > 0.6 * force);
	CHECK(notched < 0.75 * force);
	// the engine standing with the regulator open gives the same, swinging with the cranks
	engine.Controls.regulator = 1.0;
	engine.Controls.cutoff = parameters.cutoff_max;
	auto lowest{1.0e9};
	auto highest{0.0};
	for (auto step{0}; step * 0.05 < 2.0 * std::numbers::pi; ++step) {
		engine.CrankAngle = step * 0.05;
		engine.update(0.0, 0.01);
		lowest = std::min(lowest, engine.State.force);
		highest = std::max(highest, engine.State.force);
	}
	auto const mean{engine.starting_force(parameters.cutoff_max, engine.State.boiler_pressure)};
	CHECK(lowest == doctest::Approx(0.79 * mean).epsilon(0.02));
	CHECK(highest == doctest::Approx(1.11 * mean).epsilon(0.02));
}

TEST_CASE("steam: the Steam section of the definition") {
	SteamEngine engine;
	engine.load("Steam: CylD=0.63 CylS=0.66 BoilerPressure=14 InjectorFlow1=120 TenderWater=26 BackPressure=0.05");
	auto const &parameters{engine.Parameters};
	CHECK(parameters.bore == doctest::Approx(0.63));
	CHECK(parameters.working_pressure == doctest::Approx(14.0 * bar));
	CHECK(parameters.injector_flow[0] == doctest::Approx(2.0));
	CHECK(parameters.tender_water == doctest::Approx(26000.0));
	CHECK(parameters.back_pressure == doctest::Approx(0.05 * bar));
	// what isn't given keeps its default
	CHECK(parameters.cutoff_max == doctest::Approx(steam::parameters{}.cutoff_max));
}

TEST_CASE("steam: a locomotive standing in steam, looked after by the automatic fireman") {
	auto engine{ty2_in_steam()};
	run(engine, 0.0, 1800.0);
	// the pressure kept near the working one, the water in the glass, the fire alive
	CHECK(engine.pressure() > engine.Parameters.working_pressure - 1.5 * bar);
	CHECK(engine.pressure() < engine.Parameters.working_pressure + 0.5 * bar);
	CHECK(engine.water_level() > 0.3);
	CHECK(engine.water_level() < 0.9);
	CHECK(engine.State.fire_mass > 0.0);
	CHECK_FALSE(engine.State.fusible_plug_melted);
}

TEST_CASE("steam: a fire left burning without feeding the boiler melts the fusible plug") {
	auto engine{ty2_in_steam()};
	engine.Controls.automatic_fireman = false;
	engine.Controls.blower = 1.0;
	engine.Controls.firing = 0.3;
	// the safety valves take the steam the fire makes, the water boils away
	for (auto tens{0}; tens < 720 && false == engine.State.fusible_plug_melted; ++tens) {
		run(engine, 0.0, 10.0);
	}
	REQUIRE(engine.State.fusible_plug_melted);
	CHECK(engine.water_level() < 0.0);
	// and its steam puts the fire out
	run(engine, 0.0, 300.0);
	CHECK(engine.State.fire_mass < 1.0);
	CHECK(engine.State.burn_rate == 0.0);
}

TEST_CASE("steam: a boiler filled over the glass primes, the water carried over doing no work") {
	auto normal{ty2_in_steam()};
	normal.Controls.regulator = 1.0;
	normal.Controls.cutoff = 0.3;
	auto overfilled{normal};
	overfilled.State.water_mass += overfilled.Parameters.glass_mass;
	run(normal, revolutions(30.0), 1.0);
	run(overfilled, revolutions(30.0), 1.0);
	CHECK(normal.State.priming == 0.0);
	CHECK(overfilled.State.priming > 0.2);
	CHECK(overfilled.State.force < 0.8 * normal.State.force);
}

TEST_CASE("steam: the boiler limits the power the cylinders can keep up") {
	// at 65 km/h with the regulator wide open, the longest cutoff the boiler keeps over 13 bar for half an hour
	auto kept{0.0};
	auto power{0.0};
	for (auto notch{0}; notch <= 15; ++notch) {
		auto const cutoff{0.10 + 0.02 * notch};
		auto engine{ty2_in_steam()};
		engine.Controls.regulator = 1.0;
		engine.Controls.cutoff = cutoff;
		run(engine, revolutions(65.0), 1800.0);
		if (engine.pressure() < 13.0 * bar) {
			break;
		}
		kept = cutoff;
		power = engine.State.power;
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

	explicit freight_train(int Wagons, std::string_view Locomotive = "steam_ty2", double Tender = 58700.0, double Wagon = 50000.0) {
		locomotive = &train.add(definition::load(std::string{Locomotive}), 1);
		train.add(freight_wagon(Tender));
		for (auto wagon{0}; wagon < Wagons; ++wagon) {
			train.add(freight_wagon(Wagon));
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
		auto const steps{std::lround(Seconds / 5.0)};
		for (auto step{0L}; step < steps; ++step) {
			// the boiler kept between 3 and 1 bar under its working pressure: 13 and 15 bar for the Ty2's 16
			auto const working{cab.Steam.Parameters.working_pressure};
			if (cab.Vel > 15.0 && cab.Steam.pressure() < working - 3.0 * bar) {
				cab.IncScndCtrl(1);
			}
			else if (cab.Steam.pressure() > working - 1.0 * bar && cab.ScndCtrlPos > 0) {
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
	CHECK(locomotive->Steam.Parameters.wheel_diameter == doctest::Approx(1.4));
	CHECK(locomotive->Steam.pressure() > 15.0 * bar);
	CHECK(locomotive->steam_InSteam());
	// the master controller works the regulator, the secondary one notches up from full gear
	CHECK(locomotive->steam_Cutoff() == doctest::Approx(locomotive->Steam.Parameters.cutoff_max));
	locomotive->ScndCtrlPos = locomotive->ScndCtrlPosNo;
	CHECK(locomotive->steam_Cutoff() == doctest::Approx(locomotive->Steam.Parameters.cutoff_min));
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
	                         << cab.Steam.State.power / 1000.0 << " kW");
	// the 1200 t at about the 65 km/h of the Ty2's design
	CHECK(cab.Vel > 55.0);
	CHECK(cab.Vel < 75.0);
	CHECK(cab.Steam.pressure() > 11.0 * bar);
	CHECK(cab.Steam.water_level() > 0.0);
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
	CHECK(locomotive->Steam.State.cylinder_flow == 0.0);
	CHECK(locomotive->Ft == 0.0);
}

TEST_CASE("steam: the Ty23 of its drawings, with no more than its data") {
	// Stacja Muzeum, Ty23 drawings 1.03/1.05 (main data, from the 630th locomotive) and 21.03 (valve gear results):
	// 2 cylinders 650 x 720 mm, driving wheels 1450 mm, 14 at, full gear 84 % forwards and 70 % backwards, and the
	// tractive force of the drawing Z = 22 030 kg = 0.75 p d^2 s / D, which the DR's tables for the Ty2 exceed by 7 %
	SteamEngine ty23;
	ty23.load("Steam: CylD=0.65 CylS=0.72 CutoffMax=0.84 CutoffMaxReverse=0.70 BoilerPressure=13.73 Grate=4.5");
	ty23.Parameters.wheel_diameter = 1.45;
	CHECK(ty23.Parameters.cutoff_max_reverse == doctest::Approx(0.70));
	auto const force{ty23.starting_force(ty23.Parameters.cutoff_max, steam::atmosphere + ty23.Parameters.working_pressure)};
	auto const drawing{22030.0 * 9.81};
	MESSAGE("Ty23 starting force " << force / 1000.0 << " kN, the drawing's Z " << drawing / 1000.0 << " kN");
	CHECK(force > drawing);
	CHECK(force < 1.10 * drawing);
	// the same against the Ty2 as the formula has them: the Ty23 has the larger cylinders, the Ty2 the higher pressure
	auto const ty2engine{ty2()};
	auto const ty2force{ty2engine.starting_force(ty2engine.Parameters.cutoff_max, steam::atmosphere + ty2engine.Parameters.working_pressure)};
	auto const formula = [](double Pressure, double Bore, double Stroke, double Wheel) { return Pressure * Bore * Bore * Stroke / Wheel; };
	CHECK(force / ty2force == doctest::Approx(formula(13.73, 0.65, 0.72, 1.45) / formula(16.0, 0.60, 0.66, 1.40)).epsilon(0.03));
	// backwards, the shorter full gear gives less
	auto const backwards{ty23.starting_force(ty23.Parameters.cutoff_max_reverse, steam::atmosphere + ty23.Parameters.working_pressure)};
	CHECK(backwards < force);
	CHECK(backwards > 0.85 * force);
}

namespace {

// the tractive force of the PKP's formula, Z = 0.75 p d² s / D (kg, with p in at and d, s, D in cm), in N
double formula_force(double Pressure, double Bore, double Stroke, double Wheel) {
	return 0.75 * Pressure * Bore * Bore * Stroke / Wheel * 9.81;
}

} // namespace

TEST_CASE("steam: the Ty23 and the Pt31 of the museum's drawings load as steam locomotives in steam") {
	struct locomotive_class {
		char const *name;
		double wheel;
		double formula;
	};
	// the Ty23: 14 at, 65 x 72 cm, 145 cm wheels, the 22 030 kg of its drawing 1.03; the Pt31: 15 at, 63 x 70 cm, 185 cm
	for (auto const &tested : {locomotive_class{"steam_ty23", 1.45, formula_force(14.0, 65.0, 72.0, 145.0)},
	                           locomotive_class{"steam_pt31", 1.85, formula_force(15.0, 63.0, 70.0, 185.0)}}) {
		CAPTURE(tested.name);
		vehicle locomotive{definition::load(tested.name), 0.0, "test", 1};
		CHECK(locomotive->EngineType == TEngineType::SteamEngine);
		CHECK(locomotive->Steam.Parameters.wheel_diameter == doctest::Approx(tested.wheel));
		CHECK(locomotive->steam_InSteam());
		CHECK(locomotive->Steam.pressure() > locomotive->Steam.Parameters.working_pressure - 1.5 * bar);
		// the model's starting force at full gear a few percent over the formula, as the Ty2's is
		auto const force{locomotive->Steam.starting_force(locomotive->Steam.Parameters.cutoff_max, steam::atmosphere + locomotive->Steam.Parameters.working_pressure)};
		MESSAGE(tested.name << ": " << force / 1000.0 << " kN starting, the formula " << tested.formula / 1000.0 << " kN");
		CHECK(force > tested.formula);
		CHECK(force < 1.10 * tested.formula);
	}
}

TEST_CASE("steam: the Ty23 and the Pt31 take their trains up to speed on the level") {
	struct run_case {
		char const *name;
		double tender;
		int wagons;
		double wagon;
		double lowest;
		double highest;
	};
	// the Ty23 with 1200 t of goods wagons, the Pt31 with 500 t of coaches (as goods wagons here, of a like resistance)
	for (auto const &tested : {run_case{"steam_ty23", 52000.0, 24, 50000.0, 45.0, 70.0}, run_case{"steam_pt31", 72000.0, 10, 50000.0, 75.0, 105.0}}) {
		CAPTURE(tested.name);
		freight_train train{tested.wagons, tested.name, tested.tender, tested.wagon};
		auto &cab{train.mover()};
		REQUIRE(cab.DirectionForward());
		while (cab.MainCtrlPos < cab.MainCtrlPosNo) {
			REQUIRE(cab.IncMainCtrl(1));
		}
		train.train.run(10.0);
		CHECK(cab.Vel > 1.0);
		train.drive(1200.0);
		MESSAGE(tested.name << " after 20 min: " << cab.Vel << " km/h, " << cab.Steam.pressure() / bar << " bar, notch " << cab.ScndCtrlPos << ", "
		                    << cab.Steam.State.power / 1000.0 << " kW");
		CHECK(cab.Vel > tested.lowest);
		CHECK(cab.Vel < tested.highest);
		CHECK(cab.Steam.water_level() > 0.0);
	}
}
