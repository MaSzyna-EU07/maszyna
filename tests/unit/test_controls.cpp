#include "stdafx.h"
#include "doctest.h"
#include "scenario.h"
#include "drive.h"

#include "McZapkie/hamulce.h"
#include "utilities/utilities.h"

// characterisation of the controls a driver and a crew use besides driving and braking: doors, direction and
// cab change, sanding against slip, the emergency brake, releasers, the ep and the dynamic brake, heating,
// loading, collisions and the frame of several physics iterations, each in a scenario on real vehicles,
// recorded in golden traces (data/golden/controls/)

using test::scenario;
using namespace test::channels;

namespace {

void notch_up(scenario &Scenario, int Positions, double Seconds) {
	for (int i = 0; i < Positions; ++i) {
		Scenario.driver().controlling.IncMainCtrl(1);
		Scenario.wait(Seconds);
	}
}

void notch_down(scenario &Scenario) {
	while (Scenario.driver().controlling.DecMainCtrl(1)) {
		Scenario.wait(0.5);
	}
	Scenario.driver().controlling.DecScndCtrl(2);
}

void brake_to_stop(scenario &Scenario, double Seconds = 60.0) {
	Scenario.act("brake to stop");
	Scenario.cab().BrakeLevelSet(Scenario.cab().Handle->GetPos(bh_FB));
	Scenario.wait(Seconds);
	Scenario.cab().BrakeLevelSet(Scenario.cab().Handle->GetPos(bh_RP));
	Scenario.wait(20.0);
}

// the pipe charged again after an emergency stop: the releaser held with the handle at running, then let go
void recharge(scenario &Scenario) {
	auto &cab{Scenario.cab()};
	Scenario.act("releaser held, the pipe charged");
	cab.BrakeLevelSet(cab.Handle->GetPos(bh_RP));
	cab.BrakeReleaser(1);
	Scenario.wait(30.0);
	cab.BrakeReleaser(0);
	Scenario.wait(30.0);
}

bool stopped(scenario &Scenario) {
	for (std::size_t i = 0; i < Scenario.train().size(); ++i) {
		if (Scenario.vehicle(i).Vel > 0.1) {
			return false;
		}
	}
	return true;
}

// the brake cylinders of every vehicle of the train under the pressure given
bool released(scenario &Scenario, double Pressure = 0.1) {
	for (std::size_t i = 0; i < Scenario.train().size(); ++i) {
		if (Scenario.vehicle(i).BrakePress > Pressure) {
			return false;
		}
	}
	return true;
}

bool open(TMoverParameters const &Vehicle, int Side) {
	return Vehicle.Doors.instances[Side].is_open;
}

// the leaf shut, whatever the step does (closed, is_closed, wants the step folded too)
bool shut(TMoverParameters const &Vehicle, int Side) {
	return Vehicle.Doors.instances[Side].position <= 0.f;
}

bool closed(TMoverParameters const &Vehicle, int Side) {
	return Vehicle.Doors.instances[Side].is_closed;
}

} // namespace

TEST_CASE("controls: the doors of an electric unit, opened, locked, permitted and closed by driving off") {
	for (auto const &[name, cars] : std::vector<std::pair<std::string, std::vector<char const *>>>{
	         {"elf", {"elf_v1/21we-a", "elf_v1/21we-b", "elf_v1/21we-c"}},
	         {"en57", {"en57_v1/5bgaiii_led_spot", "en57_v1/6ba_840", "en57_v1/5bgb"}},
	         {"impuls", {"impuls_v1/31we-a_pr", "impuls_v1/31we-b_pr", "impuls_v1/31we-c_pr", "impuls_v1/31we-d_pr"}}}) {
		CAPTURE(name);
		scenario scenario{"controls/doors/" + name, motion() + doors()};
		for (auto const *car : cars) {
			scenario.add(car, scenario.train().size() == 0 ? 1 : 0);
		}
		// the EN57 doesn't get ready (COVERAGE.md), its doors work all the same
		auto const ready{scenario.start_up()};
		CHECK(ready == (name != "en57"));
		auto &cab{scenario.cab()};
		auto const &last{scenario.vehicle(scenario.train().size() - 1)};
		scenario.act("permit left");
		cab.PermitDoors(side::left, true);
		scenario.wait(2.0);
		scenario.act("open left");
		cab.OperateDoors(side::left, true);
		scenario.wait(10.0);
		CHECK(open(cab, side::left));
		CHECK(open(last, side::left));
		CHECK(closed(cab, side::right));
		scenario.act("close left");
		cab.OperateDoors(side::left, false);
		scenario.wait(10.0);
		CHECK(shut(cab, side::left));
		CHECK(shut(last, side::left));
		scenario.act("open right without permit");
		cab.OperateDoors(side::right, true);
		scenario.wait(5.0);
		// the EN57 needs no permit
		CHECK(shut(cab, side::right) == cab.Doors.permit_needed);
		scenario.act("permit right, open");
		cab.PermitDoors(side::right, true);
		cab.OperateDoors(side::right, true);
		scenario.wait(10.0);
		CHECK(open(cab, side::right));
		CHECK(open(last, side::right));
		scenario.act("lock");
		cab.LockDoors(true);
		scenario.wait(5.0);
		scenario.act("permit preset, steps off, remote control");
		cab.ChangeDoorPermitPreset(1);
		cab.PermitDoorStep(false);
		cab.ChangeDoorControlMode(true);
		scenario.wait(5.0);
		cab.ChangeDoorPermitPreset(-1);
		cab.PermitDoorStep(true);
		cab.ChangeDoorControlMode(false);
		scenario.act("close all, unlock");
		cab.OperateDoors(side::right, false);
		cab.OperateDoors(side::left, false);
		cab.LockDoors(false);
		scenario.wait(10.0);
		CHECK(shut(cab, side::left));
		CHECK(shut(cab, side::right));
		scenario.act("open both and drive off");
		cab.PermitDoors(side::left, true);
		cab.PermitDoors(side::right, true);
		cab.OperateDoors(side::left, true);
		cab.OperateDoors(side::right, true);
		scenario.wait(5.0);
		CHECK_FALSE(shut(cab, side::left));
		CHECK_FALSE(shut(cab, side::right));
		scenario.get_going();
		notch_up(scenario, 4, 2.0);
		scenario.wait(20.0);
		// the doors of a train ready to run close as the driver starts off. no CHECK of the speed: the Elf and
		// the Impuls keep their steps out under the permit, which keeps them from moving (COVERAGE.md)
		CHECK(shut(cab, side::left) == ready);
		CHECK(shut(cab, side::right) == ready);
		CHECK(shut(last, side::left) == ready);
		CHECK(shut(last, side::right) == ready);
		notch_down(scenario);
		brake_to_stop(scenario);
		scenario.check();
	}
}

TEST_CASE("controls: the doors of wagons, worked from the locomotive") {
	scenario scenario{"controls/doors/eu07 with wagons", motion() + doors()};
	scenario.add("4e_v1/4e", 1).add("bdhpumn_v3/bdhpumn").add("11xa_v2/93a");
	scenario.start_up();
	auto &cab{scenario.cab()};
	auto &coach{scenario.vehicle(1)};
	auto const &freight{scenario.vehicle(2)};
	// the locomotive works the coach's doors for the consist and for the unit, not with its local controls; the
	// freight wagon's doors don't move from the locomotive
	for (auto const range : {range_t::consist, range_t::unit, range_t::local}) {
		CAPTURE(static_cast<int>(range));
		scenario.act(std::format("open right {}", static_cast<int>(range)));
		cab.PermitDoors(side::right, true, range);
		cab.OperateDoors(side::right, true, range);
		scenario.wait(10.0);
		CHECK(open(coach, side::right) == (range != range_t::local));
		CHECK(shut(coach, side::left));
		CHECK(shut(freight, side::right));
		scenario.act("close right");
		cab.OperateDoors(side::right, false, range);
		scenario.wait(10.0);
		CHECK(shut(coach, side::right));
	}
	scenario.act("wagon opens its own");
	coach.OperateDoors(side::left, true, range_t::local);
	// a second at a time, the same run as one wait. the coach's doors close by themselves 5 s after they
	// opened (DoorStayOpen)
	scenario.wait(1.0);
	CHECK_FALSE(shut(coach, side::left));
	scenario.wait(9.0);
	CHECK(shut(coach, side::left));
	coach.OperateDoors(side::left, false, range_t::local);
	scenario.wait(10.0);
	CHECK(shut(coach, side::left));
	scenario.check();
}

TEST_CASE("controls: reversing and changing the cab of a locomotive") {
	scenario scenario{"controls/reverse/eu07", motion() + traction()};
	scenario.add("4e_v1/4e", 1);
	scenario.start_up();
	scenario.get_going();
	auto &cab{scenario.cab()};
	// the speed along the vehicle, its front ahead
	notch_up(scenario, 10, 1.5);
	scenario.wait(10.0);
	CHECK(cab.V > 3.0);
	notch_down(scenario);
	brake_to_stop(scenario);
	CHECK(stopped(scenario));
	scenario.act("backward");
	cab.DirectionBackward();
	cab.DirectionBackward();
	scenario.wait(2.0);
	CHECK(cab.DirActive < 0);
	notch_up(scenario, 10, 1.5);
	scenario.wait(10.0);
	CHECK(cab.V < -3.0);
	notch_down(scenario);
	brake_to_stop(scenario);
	CHECK(stopped(scenario));
	scenario.act("neutral, change cab");
	cab.DirectionForward();
	scenario.wait(1.0);
	cab.CabDeactivisation();
	// through the engine room (cab 0) to the rear cab, as the player walks there
	cab.ChangeCab(-1);
	cab.ChangeCab(-1);
	cab.CabActivisation();
	scenario.wait(5.0);
	CHECK(cab.CabActive < 0);
	scenario.act("drive from the rear cab");
	scenario.get_going();
	notch_up(scenario, 10, 1.5);
	scenario.wait(10.0);
	// forward from the rear cab is the vehicle's backward
	CHECK(cab.V < -3.0);
	notch_down(scenario);
	brake_to_stop(scenario);
	CHECK(stopped(scenario));
	scenario.check();
}

TEST_CASE("controls: wheel slip on a slippery track, sand and the anti-slip brake") {
	for (auto const *path : {"4e_v1/4e", "sm42_v1/6d1", "e186_v2/eu47"}) {
		CAPTURE(path);
		scenario scenario{std::string{"controls/slip/"} + path, motion() + traction() + brakes(), test::track{0.04, 0.0, test::catenary_3kv}};
		scenario.add(path, 1);
		scenario.start_up();
		scenario.get_going();
		auto &cab{scenario.cab()};
		// a notch a second: the controller put to full power at once never gets the line contactors closed
		scenario.act("full power");
		scenario.full_power(0.0);
		auto slipped{cab.SlippingWheels};
		for (int i = 0; i < 10; ++i) {
			scenario.wait(1.0);
			slipped = slipped || cab.SlippingWheels;
		}
		// the electric locomotives slip at full power, the SM42, of less power for its weight, doesn't
		CHECK(slipped == (std::string{path} != "sm42_v1/6d1"));
		scenario.act("sand");
		cab.Sandbox(true);
		scenario.wait(10.0);
		cab.Sandbox(false);
		scenario.act("manual sand");
		cab.SandboxManual(true);
		scenario.wait(5.0);
		cab.SandboxManual(false);
		scenario.act("anti-slip brake");
		cab.AntiSlippingButton();
		cab.AntiSlippingBrake();
		scenario.wait(10.0);
		CHECK(cab.Vel > 5.0);
		notch_down(scenario);
		brake_to_stop(scenario);
		CHECK(stopped(scenario));
		scenario.check();
	}
}

TEST_CASE("controls: emergency stops by the alarm chain and the radio-stop") {
	scenario scenario{"controls/emergency/eu07 with wagons", motion() + brakes()};
	scenario.add("4e_v1/4e", 1).add("bdhpumn_v3/bdhpumn").add("3wl_v2/3wl");
	scenario.start_up();
	scenario.get_going();
	notch_up(scenario, 20, 1.0);
	scenario.wait(20.0);
	notch_down(scenario);
	auto &cab{scenario.cab()};
	CHECK(cab.Vel > 30.0);
	scenario.act("alarm chain in the wagon");
	scenario.vehicle(1).AlarmChainSwitch(true);
	scenario.wait(40.0);
	// the pipe emptied, the whole train braked to a stop
	CHECK(stopped(scenario));
	CHECK(cab.PipePress < 0.5);
	CHECK(scenario.vehicle(2).BrakePress > 1.0);
	scenario.act("alarm chain reset");
	scenario.vehicle(1).AlarmChainSwitch(false);
	scenario.wait(10.0);
	// the FV4a handle stays cut off the emptied pipe of an LSt locomotive (lock_old in UpdatePipePressure)
	// until the releaser is held
	CHECK(cab.PipePress < 0.5);
	recharge(scenario);
	CHECK(cab.PipePress > 4.5);
	scenario.get_going();
	notch_up(scenario, 20, 1.0);
	scenario.wait(20.0);
	notch_down(scenario);
	CHECK(cab.Vel > 30.0);
	scenario.act("radio-stop");
	cab.RadiostopSwitch(true);
	scenario.wait(40.0);
	CHECK(stopped(scenario));
	CHECK(cab.PipePress < 0.5);
	scenario.act("radio-stop reset");
	cab.RadiostopSwitch(false);
	scenario.wait(10.0);
	recharge(scenario);
	CHECK(cab.PipePress > 4.5);
	scenario.check();
}

TEST_CASE("controls: releasers and the universal brake buttons") {
	scenario scenario{"controls/releaser/eu07 with wagons", brakes() + air()};
	scenario.add("4e_v1/4e", 1).add("bdhpumn_v3/bdhpumn").add("3wl_v2/3wl");
	scenario.start_up();
	scenario.get_going();
	auto &cab{scenario.cab()};
	auto &wagon{scenario.vehicle(2)};
	scenario.act("service brake");
	cab.BrakeLevelSet(cab.Handle->GetPos(bh_FB));
	scenario.wait(20.0);
	CHECK(cab.BrakePress > 1.0);
	CHECK(wagon.BrakePress > 1.0);
	cab.BrakeLevelSet(cab.Handle->GetPos(bh_NP));
	auto &coach{scenario.vehicle(1)};
	// the releaser of the cab is passed along the whole train, a wagon's works on the wagon alone
	auto const coachbraked{coach.BrakePress};
	auto wagonbraked{wagon.BrakePress};
	scenario.act("releaser of the locomotive");
	cab.BrakeReleaser(1);
	scenario.wait(10.0);
	cab.BrakeReleaser(0);
	CHECK(cab.BrakePress < 0.5);
	CHECK(coach.BrakePress < coachbraked - 1.0);
	CHECK(wagon.BrakePress < wagonbraked - 0.5);
	CHECK(cab.PipePress < 4.0); // the pipe stays where the service application left it
	wagonbraked = wagon.BrakePress;
	scenario.act("releaser of the wagon");
	wagon.BrakeReleaser(1);
	scenario.wait(10.0);
	wagon.BrakeReleaser(0);
	CHECK(wagon.BrakePress < wagonbraked - 0.5);
	for (int button = 0; button < 3; ++button) {
		scenario.act(std::format("universal button {}", button));
		cab.UniversalBrakeButton(button, 1);
		scenario.wait(5.0);
		cab.UniversalBrakeButton(button, 0);
		scenario.wait(5.0);
	}
	scenario.act("running");
	cab.BrakeLevelSet(cab.Handle->GetPos(bh_RP));
	scenario.wait(60.0);
	CHECK(cab.PipePress > 4.8);
	CHECK(released(scenario));
	scenario.check();
}

TEST_CASE("controls: the ep brake of electric units") {
	for (auto const &[name, cars] : std::vector<std::pair<std::string, std::vector<char const *>>>{
	         {"ed72", {"ed72_v1/5bs-ra", "ed72_v1/6bs-sa", "ed72_v1/6bs-sb", "ed72_v1/5bs-rb"}},
	         {"ew58", {"ew58_v1/ew58-sa", "ew58_v1/ew58-d", "ew58_v1/ew58-sb"}}}) {
		CAPTURE(name);
		scenario scenario{"controls/ep brake/" + name, motion() + brakes() + handles()};
		for (auto const *car : cars) {
			scenario.add(car, scenario.train().size() == 0 ? 1 : 0);
		}
		scenario.start_up();
		scenario.get_going();
		auto &cab{scenario.cab()};
		scenario.act("ep on");
		cab.SwitchEPBrake(1);
		scenario.wait(5.0);
		auto const &last{scenario.vehicle(scenario.train().size() - 1)};
		auto const positions{std::vector{bh_EPB, bh_EPN, bh_EPR, bh_EPB, bh_RP}};
		for (std::size_t i = 0; i < positions.size(); ++i) {
			CAPTURE(positions[i]);
			scenario.act(std::format("handle {}", positions[i]));
			cab.BrakeLevelSet(cab.Handle->GetPos(positions[i]));
			scenario.wait(15.0);
			if (i == 2) {
				CHECK(released(scenario));
			}
			if (i == 3) {
				// the ep brake works the cylinders of every car by wire, the pipe stays charged (by the first
				// application the EW58's pipe is still charging after the start-up)
				CHECK(cab.PipePress > 4.8);
				CHECK(cab.BrakePress > 1.0);
				CHECK(last.BrakePress > 1.0);
			}
		}
		// the EW58 releases at the running position, the ED72 holds the ep application there, and with the ep
		// brake switched off as well: only the ep release position lets it off
		auto const ed72{name == "ed72"};
		CHECK(released(scenario) != ed72);
		scenario.act("ep off");
		cab.SwitchEPBrake(0);
		scenario.wait(30.0);
		CHECK(released(scenario) != ed72);
		scenario.check();
	}
}

TEST_CASE("controls: the electrodynamic brake of locomotives having one") {
	for (auto const *path : {"ep09_v2/104e", "e186_v2/eu47", "e6act_v2/e6acta", "et22_v3/201e-rn"}) {
		CAPTURE(path);
		scenario scenario{std::string{"controls/dynamic brake/"} + path, motion() + traction() + brakes()};
		scenario.add(path, 1).add("bdhpumn_v3/bdhpumn").add("bdhpumn_v3/bdhpumn");
		scenario.start_up();
		scenario.get_going();
		notch_up(scenario, 60, 1.0);
		scenario.wait(30.0);
		notch_down(scenario);
		auto &cab{scenario.cab()};
		CHECK(cab.Vel > 40.0);
		auto const speed{cab.Vel};
		scenario.act("dynamic brake on");
		cab.DynamicBrakeSwitch(true);
		scenario.wait(5.0);
		scenario.act("brake");
		cab.BrakeLevelSet(cab.Handle->GetPos(bh_FB));
		scenario.wait(40.0);
		CHECK(cab.Vel < speed);
		scenario.act("dynamic brake off");
		cab.DynamicBrakeSwitch(false);
		brake_to_stop(scenario, 30.0);
		CHECK(stopped(scenario));
		scenario.check();
	}
}

TEST_CASE("controls: train heating from the locomotive") {
	scenario scenario{"controls/heating/eu07 with wagons", traction() + air()};
	scenario.add("4e_v1/4e", 1).add("bdhpumn_v3/bdhpumn").add("bdhpumn_v3/bdhpumn");
	scenario.start_up();
	auto &cab{scenario.cab()};
	scenario.act("heating on");
	cab.HeatingSwitch(true);
	scenario.wait(30.0);
	CHECK(cab.Heating);
	CHECK(scenario.vehicle(2).Heating);
	scenario.act("heating off");
	cab.HeatingSwitch(false);
	scenario.wait(10.0);
	CHECK_FALSE(cab.Heating);
	CHECK_FALSE(scenario.vehicle(2).Heating);
	scenario.check();
}

TEST_CASE("controls: a freight wagon loaded and braked") {
	auto emptymass{0.0};
	for (auto const *load : {"", "coal", "ruda", "unknown"}) {
		CAPTURE(load);
		scenario scenario{std::string{"controls/load/3wl "} + (*load ? load : "empty"), motion() + brakes()};
		scenario.add("4e_v1/4e", 1).add("3wl_v2/3wl");
		auto &wagon{scenario.vehicle(1)};
		auto const known{std::string{load} == "coal" || std::string{load} == "ruda"};
		CHECK(wagon.AssignLoad(load, 30.f) == (known || std::string{load}.empty()));
		wagon.ComputeMass();
		if (*load == '\0') {
			emptymass = wagon.TotalMass;
		}
		else {
			// 30 t of a load the wagon takes, none of one it doesn't
			CHECK(wagon.TotalMass == doctest::Approx(emptymass + (known ? 30000.0 : 0.0)).epsilon(0.01));
		}
		scenario.start_up();
		scenario.get_going();
		notch_up(scenario, 20, 1.0);
		scenario.wait(20.0);
		notch_down(scenario);
		CHECK(scenario.cab().Vel > 20.0);
		brake_to_stop(scenario);
		CHECK(stopped(scenario));
		scenario.check();
	}
}

TEST_CASE("controls: a locomotive running into a standing wagon") {
	for (auto const speed : {5.0, 15.0, 40.0}) {
		CAPTURE(speed);
		scenario scenario{std::format("controls/collision/{} kmh", static_cast<int>(speed)), motion()};
		// the wagon stands in front, the locomotive comes from behind it
		scenario.add("bdhpumn_v3/bdhpumn").add("4e_v1/4e", 0, coupling::faux, speed, 30.0);
		auto const &wagon{scenario.vehicle(0)};
		scenario.wait(30.0);
		// slow, the locomotive pushes the wagon off, the buffers only touching, and both roll to a stop; fast,
		// the wagon derails where it stands
		auto const derailed{speed > 30.0};
		CHECK(TestFlag(wagon.DamageFlag, dtrain_out) == derailed);
		CHECK((std::abs(wagon.DistCounter) > 0.0001) != derailed);
		CHECK(stopped(scenario));
		scenario.check();
	}
}

TEST_CASE("controls: a train run with five physics steps in a frame") {
	scenario scenario{"controls/full physics/eu07 with wagons", motion() + traction() + brakes(),
	            test::track{0.15, 0.0, test::catenary_3kv, 0, 5}};
	scenario.add("4e_v1/4e", 1).add("bdhpumn_v3/bdhpumn").add("3wl_v2/3wl");
	scenario.start_up();
	scenario.get_going();
	notch_up(scenario, 25, 1.0);
	scenario.wait(30.0);
	notch_down(scenario);
	// no CHECKs of the run: with five steps a frame the motors take no current and the train rolls at a few
	// km/h, which is a finding (COVERAGE.md), not the behaviour to keep
	brake_to_stop(scenario);
	scenario.check();
}

TEST_CASE("controls: the automatic electrodynamic brake of the EP09, in service braking held to a stop") {
	// held for longer, a service application of the train brake makes the EP09 brake electrodynamically: the
	// pressure its distributor asks for (GetEDBCP) brings the motors in as generators, the air of its own
	// cylinders is let out, and given back as the train slows down and the motors' brake force fades
	struct application {
		char const *name;
		double steps; // past the first braking position of the handle
	};
	for (auto const &[name, steps] : {application{"light", 0.0}, application{"medium", 1.0}, application{"high", 2.0}, application{"full service", 3.0}}) {
		CAPTURE(name);
		scenario scenario{std::string{"controls/dynamic brake held/ep09 "} + name, motion() + traction() + brakes()};
		scenario.add("ep09_v2/104e", 1, test::consist::full_coupling, 120.0);
		scenario.add("bdhpumn_v3/bdhpumn", 0, test::consist::full_coupling, 120.0);
		scenario.add("bdhpumn_v3/bdhpumn", 0, test::consist::full_coupling, 120.0);
		REQUIRE(scenario.start_up());
		auto &ep09{scenario.cab()};
		REQUIRE(ep09.Power110vIsAvailable);
		scenario.act(std::string{"brake "} + name);
		ep09.BrakeLevelSet(ep09.Handle->GetPos(bh_MB) + steps);
		scenario.wait(10.0);
		CHECK(ep09.DynamicBrakeFlag);
		CHECK(ep09.Im < -100.0);
		CHECK(ep09.BrakePress < 0.25 * ep09.Hamulec->GetEDBCP());
		scenario.wait(110.0);
		// stopped, held by the air alone. the flag of the electrodynamic brake stays on while the cylinders hold
		// less than 2.1 bar, though the motors give nothing; a stronger application drops it before the stop
		CHECK(ep09.Vel == doctest::Approx(0.0));
		CHECK(ep09.Im == doctest::Approx(0.0));
		CHECK(ep09.DynamicBrakeFlag == (ep09.BrakePress <= 2.1));
		CHECK(ep09.BrakePress == doctest::Approx(ep09.Hamulec->GetEDBCP()).epsilon(0.05));
		scenario.act("release");
		ep09.BrakeLevelSet(ep09.Handle->GetPos(bh_RP));
		scenario.wait(30.0);
		scenario.check();
	}
}

TEST_CASE("controls: the automatic electrodynamic brake under ep and service braking, from speed") {
	// ep, light and full service braking of trains running at speed, whose locomotive or unit has the automatic
	// electrodynamic brake
	struct train {
		char const *name;
		std::vector<char const *> cars; // the first one driven
		double speed; // km/h at the start
		bool epelectric; // whether the ep brake alone brings the motors in
	};
	// the EW58's motors brake by the stages of the ep brake (TUHEX), with the 110 V of the converter in its
	// middle car; the E6ACT's by its distributor, from a deeper service application
	for (auto const &[name, cars, speed, epelectric] : {
	         train{"e6act with wagons", {"e6act_v2/e6acta", "bdhpumn_v3/bdhpumn", "bdhpumn_v3/bdhpumn"}, 100.0, false},
	         train{"ew58", {"ew58_v1/ew58-sa", "ew58_v1/ew58-d", "ew58_v1/ew58-sb"}, 90.0, true},
	     }) {
		CAPTURE(name);
		scenario scenario{std::string{"controls/dynamic brake from speed/"} + name, motion() + traction() + brakes()};
		for (auto const *car : cars) {
			scenario.add(car, car == cars.front() ? 1 : 0, test::consist::full_coupling, speed);
		}
		scenario.start_up();
		auto &cab{scenario.cab()};
		auto const brake{[&scenario, &cab](char const *Event, double Position, double Seconds) {
			scenario.act(Event);
			cab.BrakeLevelSet(Position);
			scenario.wait(Seconds);
		}};
		brake("ep brake", cab.Handle->GetPos(bh_EPB), 15.0);
		CHECK(cab.DynamicBrakeFlag == epelectric);
		CHECK((cab.Im < -100.0) == epelectric);
		brake("ep hold", cab.Handle->GetPos(bh_EPN), 5.0);
		brake("first step", cab.Handle->GetPos(bh_MB), 20.0);
		brake("deeper", cab.Handle->GetPos(bh_MB) + 1.0, 15.0);
		// the pipe alone doesn't bring the EW58's motors in
		CHECK((cab.Im < -50.0) != epelectric);
		brake("release", cab.Handle->GetPos(bh_RP), 20.0);
		brake("full service", cab.Handle->GetPos(bh_FB), 40.0);
		brake("release", cab.Handle->GetPos(bh_RP), 20.0);
		scenario.check();
	}
}

TEST_CASE("controls: a draisine pumped along by hand") {
	// the pumping force, half a stroke forwards and half backwards, is below the resistance of a draisine
	// standing, which stays where it is; a rolling one is kept going
	for (auto const speed : {0.0, 10.0}) {
		CAPTURE(speed);
		scenario scenario{std::format("controls/draisine/{}", static_cast<int>(speed)), motion() + traction()};
		scenario.add("dr2_v1/dr2", 1, test::consist::full_coupling, speed);
		auto &cab{scenario.cab()};
		cab.CabActivisation();
		scenario.act("hand brake off");
		test::release_brakes(cab);
		for (int stroke = 0; stroke < 40; ++stroke) {
			scenario.act("stroke");
			cab.IncMainCtrl(1);
			scenario.wait(0.5);
			cab.DecMainCtrl(1);
			scenario.wait(0.5);
		}
		CHECK((speed > 0.0) == (cab.Vel > 1.0));
		scenario.act("coast");
		scenario.wait(20.0);
		scenario.check();
	}
}
