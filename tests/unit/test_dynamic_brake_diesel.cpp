#include "stdafx.h"
#include "doctest.h"
#include "scenario.h"
#include "drive.h"
#include "real_vehicles.h"

#include "physics/brakes.h"

// the dynamic brake of the diesel-electric vehicles (Engine: EDBrake=, MaSzyna-EU07/maszyna#82): the traction motors
// brake into resistors while the engine runs, with the force of the definition (EDBrakeFmax, the power limit
// EDBrakePmax, the fade below EDBrakeV1 down to EDBrakeV0), set either by the negative range of the integrated
// controller (EDBrake=Controller, the BR285's TRAXX controller) or by a lever of its own (EDBrake=Lever, the
// SplitEDPneumaticBrake one, with the master controller at zero)

using test::definition;
using test::scenario;
using namespace test::channels;

namespace {

std::vector<test::channel> dynamic_brake() {
	return {
	    {"EDLevel", [](auto const &M) { return M.DEDynamicBrake.level; }},
	    {"EDForce", [](auto const &M) { return M.DEDynamicBrake.force; }},
	    {"DynamicBrakeFlag", [](auto const &M) { return M.DynamicBrakeFlag ? 1.0 : 0.0; }},
	    {"eimic_real", [](auto const &M) { return M.eimic_real; }},
	    {"DynamicBrakeCtrlPos", [](auto const &M) { return M.DynamicBrakeCtrlPos; }},
	};
}

// the BR285 with its dynamic brake: 150 kN up to the power limit of 2000 kW (48 km/h), fading out under 6 km/h
definition br285_braking() {
	return definition::fiz("br285_v1/br285")
	    .set("Engine", "EDBrake", "Controller")
	    .set("Engine", "EDBrakeFmax", 150)
	    .set("Engine", "EDBrakePmax", 2000)
	    .set("Engine", "EDBrakeV0", 2)
	    .set("Engine", "EDBrakeV1", 6);
}

// a resistor brake of a locomotive with direct current motors, on the ST44 (M62): a lever of 8 steps, the force
// growing with the speed up to 120 kN at 40 km/h, the resistors taking 1200 kW
definition st44_braking() {
	return definition::fiz("st44_v2/m62-1151")
	    .set("Cntrl", "DBPN", 8)
	    .set("Engine", "EDBrake", "Lever")
	    .set("Engine", "EDBrakeFmax", 120)
	    .set("Engine", "EDBrakePmax", 1200)
	    .set("Engine", "EDBrakeV0", 0)
	    .set("Engine", "EDBrakeV1", 40);
}

// the force of the brakes above at full effort at the speed of the vehicle
double br285_limit(TMoverParameters const &Vehicle) {
	return std::min(150000.0, 2000000.0 / std::abs(Vehicle.V)) * std::clamp((Vehicle.Vel - 2.0) / 4.0, 0.0, 1.0);
}
double st44_limit(TMoverParameters const &Vehicle) {
	return std::min(120000.0, 1200000.0 / std::abs(Vehicle.V)) * std::clamp(Vehicle.Vel / 40.0, 0.0, 1.0);
}

// the controller of the BR285 down to the given position, a step a time
void controller_to(TMoverParameters &Locomotive, int Position) {
	while (Locomotive.MainCtrlPos > Position && Locomotive.DecMainCtrl(1)) {}
	while (Locomotive.MainCtrlPos < Position && Locomotive.IncMainCtrl(1)) {}
}

} // namespace

TEST_CASE("diesel-electric dynamic brake: none without its definition") {
	// no game vehicle defines it: the diesel-electric ones load without, the BR285's braking positions brake nothing
	for (auto const &path : test::real_vehicles()) {
		CAPTURE(path);
		test::vehicle vehicle{definition::fiz(path)};
		CHECK(vehicle->DEDynamicBrake.source == TMoverParameters::diesel_electric_dynamic_brake::control::none);
	}
	scenario scenario{"dynamic brake/diesel electric/br285 without", motion() + traction() + dynamic_brake()};
	scenario.add("br285_v1/br285", 1);
	REQUIRE(scenario.start_up());
	auto &locomotive{scenario.cab()};
	scenario.get_going();
	scenario.full_power(40.0);
	auto const speed{locomotive.Vel};
	REQUIRE(speed > 30.0);
	scenario.act("braking range");
	controller_to(locomotive, 0);
	scenario.wait(15.0);
	CHECK(locomotive.eimic_real == doctest::Approx(-1.0));
	CHECK(locomotive.Ft == doctest::Approx(0.0));
	CHECK_FALSE(locomotive.DynamicBrakeFlag);
	// coasting: braking with the power of the resistors above, it would lose more than 30 km/h
	CHECK(locomotive.Vel > speed - 12.0);
	scenario.check();
}

TEST_CASE("diesel-electric dynamic brake: the BR285's controller") {
	// the braking positions of the TRAXX controller: EB+ raises the braking effort, EB holds it, EB- lowers it.
	// the force is limited by the power of the resistors at speed, holds below that, fades out at the stop; the
	// air brake stays off
	scenario scenario{"dynamic brake/diesel electric/br285", motion() + traction() + brakes() + dynamic_brake()};
	scenario.train().add(br285_braking(), 1);
	auto &locomotive{scenario.cab()};
	REQUIRE(locomotive.DEDynamicBrake.source == TMoverParameters::diesel_electric_dynamic_brake::control::controller);
	CHECK(locomotive.DEDynamicBrake.max_force == doctest::Approx(150000.0));
	CHECK(locomotive.DEDynamicBrake.max_power == doctest::Approx(2000000.0));
	REQUIRE(scenario.start_up());
	scenario.get_going();
	scenario.full_power(60.0);
	REQUIRE(locomotive.Vel > 60.0);

	scenario.act("EB+");
	controller_to(locomotive, 0);
	scenario.wait(10.0);
	CHECK(locomotive.eimic_real == doctest::Approx(-1.0));
	CHECK(locomotive.DEDynamicBrake.level == doctest::Approx(1.0));
	CHECK(locomotive.DynamicBrakeFlag);
	REQUIRE(locomotive.Vel > 6.0);
	CHECK(-locomotive.Ft == doctest::Approx(br285_limit(locomotive)).epsilon(0.01));
	CHECK(locomotive.Im > 0.0);
	CHECK(locomotive.EngineVoltage == doctest::Approx(0.0));
	CHECK(locomotive.BrakePress < 0.1);

	scenario.act("EB- to half");
	controller_to(locomotive, 2);
	for (int i = 0; i < 20 && locomotive.eimic_real < -0.5; ++i) {
		scenario.wait(1.0);
	}
	scenario.act("EB");
	controller_to(locomotive, 1);
	scenario.wait(2.0);
	auto const held{locomotive.eimic_real};
	CHECK(held == doctest::Approx(-0.5).epsilon(0.35));
	scenario.wait(5.0);
	CHECK(locomotive.eimic_real == doctest::Approx(held));
	CHECK(locomotive.DEDynamicBrake.level == doctest::Approx(-held).epsilon(0.01));

	scenario.act("EB+ to the stop");
	controller_to(locomotive, 0);
	for (int i = 0; i < 120 && locomotive.Vel > 20.0; ++i) {
		scenario.wait(1.0);
	}
	// under the power limit the force holds
	REQUIRE(locomotive.Vel < 40.0);
	REQUIRE(locomotive.Vel > 6.0);
	CHECK(-locomotive.Ft == doctest::Approx(150000.0).epsilon(0.01));
	for (int i = 0; i < 120 && locomotive.Vel > 4.0; ++i) {
		scenario.wait(1.0);
	}
	// fading out
	CHECK(-locomotive.Ft < 150000.0 * 0.6);
	for (int i = 0; i < 120 && locomotive.Vel > 1.5; ++i) {
		scenario.wait(1.0);
	}
	CHECK(locomotive.Ft == doctest::Approx(0.0));
	CHECK_FALSE(locomotive.DynamicBrakeFlag);
	// it doesn't hold the train: the air brake does
	CHECK(locomotive.Vel > 0.1);
	scenario.check();
}

TEST_CASE("diesel-electric dynamic brake: a resistor brake with its own lever") {
	// on a locomotive with direct current motors the lever sets the excitation of the motors: the force grows with
	// the speed, up to the limit of the current, then of the resistors' power. it works only with the master
	// controller at zero and the engine running
	scenario scenario{"dynamic brake/diesel electric/st44 lever", motion() + traction() + brakes() + dynamic_brake()};
	scenario.train().add(st44_braking(), 1);
	auto &locomotive{scenario.cab()};
	REQUIRE(locomotive.DEDynamicBrake.source == TMoverParameters::diesel_electric_dynamic_brake::control::lever);
	CHECK(locomotive.SplitEDPneumaticBrake);
	CHECK(locomotive.DynamicBrakeCtrlPosNo == 8);
	REQUIRE(scenario.start_up());
	scenario.get_going();
	scenario.full_power(90.0);
	REQUIRE(locomotive.Vel > 50.0);

	scenario.act("lever with power on");
	CHECK(locomotive.DynamicBrakeLevelSet(1.0));
	scenario.wait(5.0);
	// the master controller at a power position keeps it off
	CHECK(locomotive.DEDynamicBrake.level == doctest::Approx(0.0));
	CHECK(locomotive.Ft > 0.0);
	CHECK_FALSE(locomotive.DynamicBrakeFlag);

	scenario.act("power off");
	scenario.notch_back(0);
	scenario.wait(4.0);
	CHECK(locomotive.DEDynamicBrake.level == doctest::Approx(1.0));
	CHECK(locomotive.DynamicBrakeFlag);
	CHECK(-locomotive.Ft == doctest::Approx(st44_limit(locomotive)).epsilon(0.01));
	CHECK(locomotive.ShowCurrent(0) > 0.0);

	scenario.act("half the lever");
	locomotive.DecDynamicBrakeLevel(4.0f);
	CHECK(locomotive.DynamicBrakeCtrlPos == doctest::Approx(0.5));
	for (int i = 0; i < 300 && locomotive.Vel > 20.0; ++i) {
		scenario.wait(1.0);
	}
	REQUIRE(locomotive.Vel < 21.0);
	REQUIRE(locomotive.Vel > 10.0);
	// on the rising part, half the excitation
	CHECK(-locomotive.Ft == doctest::Approx(0.5 * st44_limit(locomotive)).epsilon(0.01));

	scenario.act("engine off");
	locomotive.MainSwitch(false);
	scenario.wait(1.0);
	CHECK(locomotive.DEDynamicBrake.level == doctest::Approx(0.0));
	CHECK(locomotive.Ft == doctest::Approx(0.0));
	CHECK_FALSE(locomotive.DynamicBrakeFlag);
	scenario.check();
}

TEST_CASE("diesel-electric dynamic brake: blended with the locomotive's air brake") {
	// EDBrakeBlending: the LSt valve lets the train brake off the locomotive while the dynamic brake takes over
	// (Hamulec->SetED with the share of the force it gives); without, both brake
	for (auto const blending : {false, true}) {
		CAPTURE(blending);
		scenario scenario{std::format("dynamic brake/diesel electric/st44 blending {}", blending), motion() + traction() + brakes() + dynamic_brake()};
		scenario.train().add(st44_braking().set("Engine", "EDBrakeBlending", blending ? "Yes" : "No"), 1);
		auto &locomotive{scenario.cab()};
		REQUIRE(locomotive.DEDynamicBrake.blending == blending);
		REQUIRE(scenario.start_up());
		scenario.get_going();
		scenario.full_power(150.0);
		scenario.notch_back(0);
		scenario.act("dynamic and train brake");
		locomotive.DynamicBrakeLevelSet(1.0);
		locomotive.BrakeLevelSet(locomotive.Handle->GetPos(bh_FB));
		scenario.wait(6.0);
		REQUIRE(locomotive.Vel > 15.0);
		CHECK(locomotive.DEDynamicBrake.level == doctest::Approx(1.0));
		CHECK(locomotive.PipePress < 4.6);
		CHECK((locomotive.BrakePress < 0.2) == blending);
		scenario.check();
	}
}

TEST_CASE("diesel-electric dynamic brake: without a fade it brings the vehicle to rest") {
	// EDBrakeV0 = EDBrakeV1 = 0 (the defaults): the whole force down to the stop. near it no more than what stops the
	// vehicle within the step, or the force turned round each time the speed crossed zero and the vehicle never
	// came to rest, the air brake being off
	scenario scenario{"dynamic brake/diesel electric/br285 no fade", motion() + traction() + brakes() + dynamic_brake()};
	scenario.train().add(definition::fiz("br285_v1/br285").set("Engine", "EDBrake", "Controller").set("Engine", "EDBrakeFmax", 150), 1);
	auto &locomotive{scenario.cab()};
	REQUIRE(locomotive.DEDynamicBrake.fade_low == doctest::Approx(0.0));
	REQUIRE(locomotive.DEDynamicBrake.fade_high == doctest::Approx(0.0));
	REQUIRE(scenario.start_up());
	scenario.get_going();
	scenario.full_power(30.0);
	REQUIRE(locomotive.Vel > 10.0);

	scenario.act("EB+ to the stop");
	controller_to(locomotive, 0);
	for (int i = 0; i < 120 && locomotive.V != 0.0; ++i) {
		scenario.wait(1.0);
	}
	CHECK(locomotive.BrakePress < 0.1);
	// at rest, and staying there: no speed either way, no force
	for (int i = 0; i < 5; ++i) {
		scenario.wait(1.0);
		CHECK(locomotive.V == 0.0);
		CHECK(locomotive.Ft == doctest::Approx(0.0));
	}
	CHECK(locomotive.DEDynamicBrake.level == doctest::Approx(1.0));
	scenario.check();
}
