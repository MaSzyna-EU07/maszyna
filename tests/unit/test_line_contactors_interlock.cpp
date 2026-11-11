#include "stdafx.h"
#include "doctest.h"
#include "consist.h"
#include "drive.h"
#include "vehicle.h"

#include "McZapkie/hamulce.h"

// the line contactors interlocked with the fans of the traction motors (eu07/maszyna#460): on the ET42 the fans of a
// section run off its own converter, and with them switched off the line contactors of that section don't close.
// the louvres of the fans of the ET22 open with the air of the main reservoir (eu07/maszyna#451)

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

// a blower with MotorBlowersStartVelocity starts with the speed and the motor current, so it can't be running before
// the line contactors close, nor after a stop longer than it keeps running for (MotorBlowersSustainTime): the interlock
// takes it switched on and powered instead, or the vehicle would never take power

TEST_CASE("line contactors interlock: a blower starting with the speed needs to be switched on and powered, not running") {
	auto locomotive{ready("MotorBlowers")};
	for (auto &blower : locomotive->MotorBlowers) {
		blower.min_start_velocity = 5.f;
		blower.is_enabled = true; // switched on by the driver
		blower.is_active = false; // the vehicle stands
	}
	REQUIRE(locomotive->MotorBlowers[end::front].start_type == start_t::manual);
	CHECK(locomotive->MotorConnectorsCheck());

	auto &rear{locomotive->MotorBlowers[end::rear]};
	SUBCASE("switched off, it holds the contactors open") {
		rear.is_disabled = true;
		CHECK_FALSE(locomotive->MotorConnectorsCheck());
	}
	SUBCASE("not switched on, it holds the contactors open") {
		rear.is_enabled = false;
		CHECK_FALSE(locomotive->MotorConnectorsCheck());
	}
	SUBCASE("off the converter, it needs the 110 V") {
		rear.start_type = start_t::converter;
		rear.is_enabled = false; // nothing for the driver to switch on
		locomotive->Power110vIsAvailable = false;
		CHECK_FALSE(locomotive->MotorConnectorsCheck());
		locomotive->Power110vIsAvailable = true;
		CHECK(locomotive->MotorConnectorsCheck());
	}
	SUBCASE("off the battery, it needs the 24 V") {
		rear.start_type = start_t::battery;
		locomotive->Power24vIsAvailable = false;
		CHECK_FALSE(locomotive->MotorConnectorsCheck());
		locomotive->Power24vIsAvailable = true;
		CHECK(locomotive->MotorConnectorsCheck());
	}
	SUBCASE("one without a start velocity still has to run") {
		rear.min_start_velocity = -1.f;
		CHECK_FALSE(locomotive->MotorConnectorsCheck());
		rear.is_active = true;
		CHECK(locomotive->MotorConnectorsCheck());
	}
}

namespace {

bool blowers_run(TMoverParameters const &Mover) {
	return Mover.MotorBlowers[end::front].is_active && Mover.MotorBlowers[end::rear].is_active;
}

// the driver's first power position, held until the line contactors close (their delay, the pressure switch)
bool close_line_contactors(test::driven const &Driven) {
	Driven.controlling.IncMainCtrl(1);
	for (int i = 0; i < 30 && false == Driven.controlling.StLinFlag; ++i) {
		test::drive(Driven, line, 1.0);
	}
	return Driven.controlling.StLinFlag;
}

// the brakes released and the cylinders empty enough for the pressure switch to let the vehicle start
void release(test::driven const &Driven) {
	test::release_brakes(Driven.cab);
	for (int i = 0; i < 90 && (Driven.controlling.BrakePress > 0.1 || Driven.controlling.PipePress < 4.8); ++i) {
		test::drive(Driven, line, 1.0);
	}
}

} // namespace

TEST_CASE("line contactors interlock: with blowers starting with the speed the vehicle starts from a standstill") {
	// each of the circuits a blower runs off has power before the line contactors close
	std::string starttype;
	SUBCASE("blowers off the converter") { starttype = "Converter"; }
	SUBCASE("blowers off the battery") { starttype = "Battery"; }
	SUBCASE("blowers switched on by the driver, off the main circuit") { starttype = "Manual"; }
	CAPTURE(starttype);
	vehicle eu07{definition::fiz("4e_v1/4e")
	                 .set("Cntrl", "LineContactorsInterlock", "MotorBlowers")
	                 .set("Cntrl", "MotorBlowersStart", starttype)
	                 .set("Engine", "MotorBlowersStartVelocity", 5.0)
	                 .set("Engine", "MotorBlowersSustainTime", 10.0),
	             0.0, "test", 1};
	auto &mover{eu07.mover()};
	REQUIRE(mover.LineContactorsInterlock == line_contactors_interlock::motor_blowers);
	REQUIRE(mover.MotorBlowers[end::front].min_start_velocity == doctest::Approx(5.0));
	REQUIRE(mover.MotorBlowers[end::rear].sustain_time == doctest::Approx(10.0));
	auto driven{test::driving(eu07)};
	REQUIRE(test::start_up(driven, line));
	release(driven);

	// standing, the blowers don't run; the line contactors close all the same and the motors take current
	REQUIRE(std::abs(eu07.speed()) < 0.1);
	REQUIRE_FALSE(blowers_run(mover));
	CHECK(close_line_contactors(driven));
	CHECK(std::abs(mover.Im) > 0.5);

	// the blowers come on once the vehicle moves
	for (int i = 0; i < 6; ++i) {
		mover.IncMainCtrl(1);
		test::drive(driven, line, 2.0);
	}
	for (int i = 0; i < 60 && eu07.speed() < 10.0; ++i) {
		test::drive(driven, line, 1.0);
	}
	REQUIRE(eu07.speed() >= 10.0);
	CHECK(blowers_run(mover));
	CHECK(mover.StLinFlag);

	// a stop longer than the blowers keep running for
	while (mover.DecMainCtrl(1)) {
		// down to zero
	}
	driven.cab.BrakeLevelSet(driven.cab.Handle->GetPos(bh_FB));
	for (int i = 0; i < 120 && std::abs(eu07.speed()) > 0.1; ++i) {
		test::drive(driven, line, 1.0);
	}
	REQUIRE(std::abs(eu07.speed()) <= 0.1);
	test::drive(driven, line, 15.0);
	REQUIRE_FALSE(blowers_run(mover));
	REQUIRE_FALSE(mover.StLinFlag);

	// and the vehicle starts again
	release(driven);
	CHECK(close_line_contactors(driven));
	CHECK(std::abs(mover.Im) > 0.5);
	for (int i = 0; i < 6; ++i) {
		mover.IncMainCtrl(1);
		test::drive(driven, line, 2.0);
	}
	for (int i = 0; i < 60 && eu07.speed() < 10.0; ++i) {
		test::drive(driven, line, 1.0);
	}
	CHECK(eu07.speed() >= 10.0);
	CHECK(blowers_run(mover));
}

TEST_CASE("line contactors interlock: a blower starting with the speed, switched off on the way, opens the contactors") {
	vehicle eu07{definition::fiz("4e_v1/4e")
	                 .set("Cntrl", "LineContactorsInterlock", "MotorBlowers")
	                 .set("Cntrl", "MotorBlowersStart", "Manual")
	                 .set("Engine", "MotorBlowersStartVelocity", 5.0)
	                 .set("Engine", "MotorBlowersSustainTime", 10.0),
	             0.0, "test", 1};
	auto &mover{eu07.mover()};
	auto driven{test::driving(eu07)};
	REQUIRE(test::start_up(driven, line));
	release(driven);
	REQUIRE(close_line_contactors(driven));

	// the switch of the rear blower: off, the contactors open and stay open, standing as well as later
	REQUIRE(mover.MotorBlowersSwitchOff(true, end::rear));
	test::drive(driven, line, 2.0);
	CHECK_FALSE(mover.StLinFlag);
	CHECK(std::abs(mover.Im) < 0.5);
	while (mover.DecMainCtrl(1)) {
		// down to zero
	}
	test::drive(driven, line, 2.0);
	CHECK_FALSE(close_line_contactors(driven));

	// back on, the vehicle starts
	REQUIRE(mover.MotorBlowersSwitchOff(false, end::rear));
	while (mover.DecMainCtrl(1)) {
		// down to zero
	}
	test::drive(driven, line, 2.0);
	CHECK(close_line_contactors(driven));
	CHECK(std::abs(mover.Im) > 0.5);
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

TEST_CASE("louvres: closed until the main reservoir reaches the pressure that opens them") {
	vehicle et22{definition::fiz("et22_v3/201e-w")};
	CHECK(et22->LouvresOpenPressure == doctest::Approx(5.0)); // by default
	et22->Compressor = 4.9;
	CHECK(et22->LouvresClosed());
	et22->Compressor = 5.0;
	CHECK_FALSE(et22->LouvresClosed());

	vehicle given{definition::fiz("et22_v3/201e-w").set("Cntrl", "LouvresOpenPressure", 4.0)};
	given->Compressor = 4.5;
	CHECK_FALSE(given->LouvresClosed());
	given->Compressor = 3.9;
	CHECK(given->LouvresClosed());
}
