#include "stdafx.h"
#include "doctest.h"
#include "scenario.h"
#include "vehicle.h"

// the door closure signalling (SOD, the i-door_blocked lamp of a cab): the state of each vehicle's doors and of the
// whole train, the most severe of its vehicles, as the driver gathers it for the lamp (eu07/maszyna#288)

using test::scenario;
using test::definition;
using test::vehicle;
using namespace test::channels;

namespace {

// the state of the whole train, the way TController::determine_consist_state() gathers it
door_closure_t consist_closure(scenario &Scenario) {
	auto closure{door_closure_t::none};
	for (std::size_t i = 0; i < Scenario.train().size(); ++i) {
		closure = std::max(closure, Scenario.vehicle(i).door_closure());
	}
	return closure;
}

} // namespace

TEST_CASE("door closure: the lamp is lit for locked doors, blinks for released ones and is dark for open ones") {
	for (auto const lit : {true, false}) {
		CAPTURE(lit);
		CHECK(door_closure_lamp(door_closure_t::locked, lit));
		CHECK(door_closure_lamp(door_closure_t::released, lit) == lit);
		CHECK_FALSE(door_closure_lamp(door_closure_t::open, lit));
		CHECK_FALSE(door_closure_lamp(door_closure_t::none, lit));
	}
	// a consist takes the most severe state of its vehicles
	CHECK(std::max(door_closure_t::none, door_closure_t::locked) == door_closure_t::locked);
	CHECK(std::max(door_closure_t::locked, door_closure_t::released) == door_closure_t::released);
	CHECK(std::max(door_closure_t::released, door_closure_t::open) == door_closure_t::open);
}

TEST_CASE("door closure: a unit with permits, locked, released by the permit and open, in any of its cars") {
	scenario scenario{"door closure/elf", motion() + doors()};
	scenario.add("elf_v1/21we-a", 1).add("elf_v1/21we-b").add("elf_v1/21we-c");
	scenario.start_up();
	auto &cab{scenario.cab()};
	auto &last{scenario.vehicle(scenario.train().size() - 1)};
	// closed with no permit: no door can be opened
	CHECK(cab.door_closure() == door_closure_t::locked);
	CHECK(last.door_closure() == door_closure_t::locked);
	CHECK(consist_closure(scenario) == door_closure_t::locked);

	scenario.act("permit left");
	cab.PermitDoors(side::left, true);
	scenario.wait(2.0);
	CHECK(last.door_closure() == door_closure_t::released);
	CHECK(consist_closure(scenario) == door_closure_t::released);

	scenario.act("the last car opens its own door");
	last.OperateDoors(side::left, true, range_t::local);
	scenario.wait(5.0);
	// the cab's own doors stay closed, the train is open all the same
	CHECK(cab.door_closure() == door_closure_t::released);
	CHECK(last.door_closure() == door_closure_t::open);
	CHECK(consist_closure(scenario) == door_closure_t::open);

	scenario.act("close, the steps stay out under the permit");
	last.OperateDoors(side::left, false, range_t::local);
	cab.OperateDoors(side::left, false);
	scenario.wait(10.0);
	CHECK(last.Doors.instances[side::left].position <= 0.f);
	// a step out isn't a closed door
	CHECK(consist_closure(scenario) == door_closure_t::open);

	scenario.act("revoke the permit");
	cab.PermitDoors(side::left, false);
	scenario.wait(10.0);
	CHECK(consist_closure(scenario) == door_closure_t::locked);
}

TEST_CASE("door closure: the doors of a unit without permits are released at a standstill") {
	scenario scenario{"door closure/en57", motion() + doors()};
	scenario.add("en57_v1/5bgaiii_led_spot", 1).add("en57_v1/6ba_840").add("en57_v1/5bgb");
	scenario.start_up();
	auto &cab{scenario.cab()};
	auto &middle{scenario.vehicle(1)};
	REQUIRE_FALSE(cab.Doors.permit_needed);
	REQUIRE(middle.Doors.permit_needed);
	// the door lock engages only once the train runs (DoorLockSpeed), till then the end cars' doors can be opened
	CHECK(cab.door_closure() == door_closure_t::released);
	CHECK(middle.door_closure() == door_closure_t::locked);
	CHECK(consist_closure(scenario) == door_closure_t::released);
	scenario.act("open right");
	cab.OperateDoors(side::right, true);
	scenario.wait(10.0);
	CHECK(consist_closure(scenario) == door_closure_t::open);
	scenario.act("close right");
	cab.OperateDoors(side::right, false);
	scenario.wait(10.0);
	CHECK(consist_closure(scenario) == door_closure_t::released);
}

TEST_CASE("door closure: a locomotive has no doors to signal, its coach does") {
	scenario scenario{"door closure/eu07 with a coach", motion() + doors()};
	scenario.add("4e_v1/4e", 1).add("bdhpumn_v3/bdhpumn");
	scenario.start_up();
	CHECK(scenario.cab().door_closure() == door_closure_t::none);
	scenario.act("open right");
	scenario.cab().PermitDoors(side::right, true);
	scenario.cab().OperateDoors(side::right, true);
	scenario.wait(10.0);
	CHECK(scenario.vehicle(1).door_closure() == door_closure_t::open);
	CHECK(consist_closure(scenario) == door_closure_t::open);
}

TEST_CASE("door closure: the signalling is switched on by DoorClosureSignal in the Doors section, off without it") {
	SUBCASE("without the key") {
		vehicle wagon{definition::load("wagon")};
		CHECK_FALSE(wagon->Doors.closure_signal);
	}
	SUBCASE("with the key") {
		vehicle wagon{definition::load("wagon").set("Doors", "DoorClosureSignal", "Yes")};
		CHECK(wagon->Doors.closure_signal);
	}
}

TEST_CASE("door closure: without DoorClosureSignal the lamp shows the lock of the vehicle's own doors, as before") {
	vehicle wagon{definition::load("wagon")};
	REQUIRE_FALSE(wagon->Doors.closure_signal);
	for (auto const locked : {true, false}) {
		wagon->Doors.is_locked = locked;
		for (auto const consist : {door_closure_t::none, door_closure_t::locked, door_closure_t::released, door_closure_t::open}) {
			for (auto const lit : {true, false}) {
				CAPTURE(locked);
				CAPTURE(static_cast<int>(consist));
				CAPTURE(lit);
				// no blinking at a standstill with the doors closed, whatever the consist does
				CHECK(wagon->door_closure_lamp_lit(consist, lit) == locked);
			}
		}
	}
}

TEST_CASE("door closure: with DoorClosureSignal the lamp shows the consist in three states") {
	vehicle wagon{definition::load("wagon").set("Doors", "DoorClosureSignal", "Yes")};
	REQUIRE(wagon->Doors.closure_signal);
	for (auto const locked : {true, false}) {
		// the vehicle's own lock doesn't matter, the consist does
		wagon->Doors.is_locked = locked;
		for (auto const lit : {true, false}) {
			CAPTURE(locked);
			CAPTURE(lit);
			CHECK(wagon->door_closure_lamp_lit(door_closure_t::locked, lit));
			CHECK(wagon->door_closure_lamp_lit(door_closure_t::released, lit) == lit);
			CHECK_FALSE(wagon->door_closure_lamp_lit(door_closure_t::open, lit));
			CHECK_FALSE(wagon->door_closure_lamp_lit(door_closure_t::none, lit));
		}
	}
}
