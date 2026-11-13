#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

using test::definition;
using test::vehicle;

// the compartment lights lamp of the cab (i-compartmentlights:) shows CompartmentLights.is_active of the cab's own
// vehicle, which LightsCheck() works out from the low voltage power and the compartment lights switch (eu07/maszyna#456)
// NOTE: the checks call LightsCheck() the way the vehicle update does, after setting the power by hand

namespace {

void lights_check(vehicle &Car) {
	Car->LightsCheck(0.01);
}

} // namespace

TEST_CASE("compartment lights: the lights need low voltage power and the switch") {
	vehicle car{definition::fiz("elf_v1/21we-a")};
	REQUIRE(car->CompartmentLights.start_type == start_t::manual);
	car->Power24vIsAvailable = false;
	car->Power110vIsAvailable = false;

	car->CompartmentLightsSwitch(true, range_t::local); // switch on, no power
	lights_check(car);
	CHECK_FALSE(car->CompartmentLights.is_active);

	car->Power110vIsAvailable = true; // converter on
	lights_check(car);
	CHECK(car->CompartmentLights.is_active);
	CHECK(car->CompartmentLights.intensity == doctest::Approx(1.0));

	car->Power110vIsAvailable = false; // battery only: the lights stay on, dimmed
	car->Power24vIsAvailable = true;
	lights_check(car);
	CHECK(car->CompartmentLights.is_active);
	CHECK(car->CompartmentLights.intensity == doctest::Approx(0.5));

	car->Power24vIsAvailable = false; // no power at all
	lights_check(car);
	CHECK_FALSE(car->CompartmentLights.is_active);
}

TEST_CASE("compartment lights: the off switch turns the lights off, the on switch back on") {
	vehicle car{definition::fiz("elf_v1/21we-a")};
	car->Power110vIsAvailable = true;

	lights_check(car);
	CHECK_FALSE(car->CompartmentLights.is_active); // manual start: off until switched on

	// push buttons: the on button latches the lights, they stay on once it is released
	car->CompartmentLightsSwitch(true, range_t::local);
	lights_check(car);
	car->CompartmentLightsSwitch(false, range_t::local);
	lights_check(car);
	CHECK(car->CompartmentLights.is_active);

	car->CompartmentLightsSwitchOff(true, range_t::local);
	lights_check(car);
	CHECK_FALSE(car->CompartmentLights.is_active);
	car->CompartmentLightsSwitchOff(false, range_t::local);
	lights_check(car);
	CHECK_FALSE(car->CompartmentLights.is_active); // still off once the off button is released
}

TEST_CASE("compartment lights: lights started automatically come on with the power alone") {
	vehicle car{definition::fiz("elf_v1/21we-a").set("Cntrl", "CompartmentLightsStart", "Automatic")};
	REQUIRE(car->CompartmentLights.start_type == start_t::automatic);
	car->Power24vIsAvailable = false;
	car->Power110vIsAvailable = false;

	lights_check(car);
	CHECK_FALSE(car->CompartmentLights.is_active);

	car->Power110vIsAvailable = true;
	lights_check(car);
	CHECK(car->CompartmentLights.is_active);
}
