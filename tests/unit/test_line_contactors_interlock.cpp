#include "stdafx.h"
#include "doctest.h"
#include "consist.h"
#include "drive.h"
#include "vehicle.h"

// the line contactors interlocked with the fans of the traction motors (eu07/maszyna#460): on the ET42 the fans of a
// section run off its own converter, and with them switched off the line contactors of that section don't close.

using test::definition;
using test::vehicle;

namespace {

// a vehicle whose line contactors could close, but for the interlock
vehicle ready(std::string_view const Interlock) {
	auto loaded{definition::load("wagon")};
	if (false == Interlock.empty()) {
		loaded.set("Cntrl", "LineContactorsInterlock", std::string{Interlock});
	}
	vehicle locomotive{loaded};
	locomotive->Mains = true;
	locomotive->DirActive = 1;
	locomotive->PipePress = 5.0;
	locomotive->BrakePress = 0.0;
	return locomotive;
}

test::track const line{0.15, 0.0, test::catenary_3kv};

} // namespace

TEST_CASE("line contactors interlock: read from the Cntrl. section, none by default") {
	CHECK(ready("")->LineContactorsInterlock == line_contactors_interlock::none);
	CHECK(ready("Converter")->LineContactorsInterlock == line_contactors_interlock::converter);
	CHECK(ready("MotorBlowers")->LineContactorsInterlock == line_contactors_interlock::motor_blowers);
	CHECK(ready("Something")->LineContactorsInterlock == line_contactors_interlock::none);
}

TEST_CASE("line contactors interlock: without one the converter and the blowers don't matter, as before") {
	auto locomotive{ready("")};
	locomotive->ConverterFlag = false;
	for (auto &blower : locomotive->MotorBlowers) {
		blower.is_active = false;
	}
	CHECK(locomotive->MotorConnectorsCheck());
}

TEST_CASE("line contactors interlock: on the converter, they close only with the vehicle's own converter running") {
	auto locomotive{ready("Converter")};
	locomotive->ConverterFlag = false;
	locomotive->Power110vIsAvailable = true; // the 110 V from the other section doesn't run this one's fans
	CHECK_FALSE(locomotive->MotorConnectorsCheck());
	locomotive->ConverterFlag = true;
	CHECK(locomotive->MotorConnectorsCheck());
	locomotive->Mains = false; // the other conditions still hold
	CHECK_FALSE(locomotive->MotorConnectorsCheck());
}

TEST_CASE("line contactors interlock: on the motor blowers, they close only with all of them running") {
	auto locomotive{ready("MotorBlowers")};
	locomotive->MotorBlowers[end::front].is_active = true;
	locomotive->MotorBlowers[end::rear].is_active = false;
	CHECK_FALSE(locomotive->MotorConnectorsCheck());
	locomotive->MotorBlowers[end::rear].is_active = true;
	CHECK(locomotive->MotorConnectorsCheck());
}

TEST_CASE("line contactors interlock: the ET42 section with its fans switched off doesn't start") {
	test::consist et42;
	et42.add(definition::fiz("et42_v2/112e-a").set("Cntrl", "LineContactorsInterlock", "Converter"), 1);
	et42.add(definition::fiz("et42_v2/112e-b").set("Cntrl", "LineContactorsInterlock", "Converter"));
	auto driven{test::driving(et42)};
	REQUIRE(test::start_up(driven, line));
	auto &front{et42[0].mover()};
	auto &rear{et42[1].mover()};
	REQUIRE(front.ConverterFlag);
	REQUIRE(rear.ConverterFlag);

	// the fans of the rear section switched off with its own switch, the 110 V of the front one still reaches it
	rear.ConverterAllowLocal = false;
	test::drive(driven, line, 2.0);
	REQUIRE_FALSE(rear.ConverterFlag);
	CHECK(rear.Power110vIsAvailable);

	test::release_brakes(driven.cab);
	for (int i = 0; i < 3 && driven.cab.DirActive <= 0; ++i) {
		driven.cab.DirectionForward();
	}
	test::drive(driven, line, 20.0);
	driven.controlling.IncMainCtrl(1);
	// as the AI does, it waits at the first position for the contactors (their delay, the pressure switch)
	for (int i = 0; i < 90 && false == front.StLinFlag; ++i) {
		test::drive(driven, line, 1.0);
	}
	CHECK(front.StLinFlag);
	CHECK_FALSE(rear.StLinFlag);

	// the fans back on, the section starts too
	rear.ConverterAllowLocal = true;
	test::drive(driven, line, 10.0);
	REQUIRE(rear.ConverterFlag);
	CHECK(rear.StLinFlag);

	// and the fans switched off on the way open its line contactors again
	rear.ConverterAllowLocal = false;
	test::drive(driven, line, 2.0);
	CHECK_FALSE(rear.StLinFlag);
	CHECK(front.StLinFlag);
}
