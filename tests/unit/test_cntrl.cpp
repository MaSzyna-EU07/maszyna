#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

using test::definition;
using test::vehicle;

// the Cntrl. section of the vehicle definition, as read by TMoverParameters::LoadFIZ_Cntrl()

TEST_CASE("cntrl: the brake delays of the settings come from BDelay1 to BDelay4") {
	vehicle wagon{definition::load("wagon").set("Cntrl", "BDelay1", 10.0).set("Cntrl", "BDelay2", 20.0).set("Cntrl", "BDelay3", 30.0).set("Cntrl", "BDelay4", 40.0)};
	CHECK(wagon->BrakeDelay[0] == doctest::Approx(10.0));
	// the vehicle varies the others a little, by 0.83 to 0.9 (CheckLocomotiveParameters())
	for (auto setting{1}; setting < 4; ++setting) {
		CAPTURE(setting);
		auto const given{10.0 * (setting + 1)};
		CHECK(wagon->BrakeDelay[setting] >= 0.83 * given);
		CHECK(wagon->BrakeDelay[setting] <= 0.9 * given);
	}
}

TEST_CASE("cntrl: the relays of a multiple unit start on their own, of other vehicles by hand") {
	SUBCASE("multiple unit") {
		vehicle unit{definition::load("wagon").set("Param", "Type", "ezt")};
		REQUIRE(unit->TrainType == dt_EZT);
		CHECK(unit->GroundRelayStart == start_t::automatic);
		CHECK(unit->ConverterOverloadRelayStart == start_t::converter); // activates when the converter is switched off
	}
	SUBCASE("other vehicle") {
		vehicle wagon{definition::load("wagon")};
		CHECK(wagon->GroundRelayStart == start_t::manual);
		CHECK(wagon->ConverterOverloadRelayStart == start_t::manual);
	}
	SUBCASE("given in the definition") {
		vehicle unit{definition::load("wagon").set("Param", "Type", "ezt").set("Cntrl", "GroundRelayStart", "Manual").set("Cntrl", "ConverterOverloadRelayStart", "Automatic")};
		CHECK(unit->GroundRelayStart == start_t::manual);
		CHECK(unit->ConverterOverloadRelayStart == start_t::automatic);
	}
}
