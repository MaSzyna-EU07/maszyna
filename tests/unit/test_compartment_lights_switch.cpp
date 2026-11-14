#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

#include "vehicle/compartmentlightsswitch.h"

using test::definition;
using test::vehicle;

// the compartment lights commands of the cab (TTrain::OnCommand_compartmentlightsenable/disable/toggle): whether the
// command for one request (on: CompartmentLights.is_enabled, off: is_disabled) also withdraws the other one. In a cab
// without compartmentlights_sw: and without the push buttons (the CD 810 cab A) the keyboard toggle, ctrl+;, turned
// the lights on and off once, then never again: the off request stayed set alongside the on one.

TEST_CASE("compartment lights switch: which command withdraws the other request") {
	// a two-state compartmentlights_sw: holds one request at a time
	CHECK(compartment_lights_switch::withdraws_other_request(true, true, false));
	CHECK(compartment_lights_switch::withdraws_other_request(true, true, true));
	// a push type compartmentlights_sw: withdraws both requests itself, when it's released
	CHECK_FALSE(compartment_lights_switch::withdraws_other_request(true, false, false));
	CHECK_FALSE(compartment_lights_switch::withdraws_other_request(true, false, true));
	// no compartmentlights_sw:, the push button of the other request withdraws it when it's released
	CHECK_FALSE(compartment_lights_switch::withdraws_other_request(false, false, true));
	CHECK_FALSE(compartment_lights_switch::withdraws_other_request(false, true, true));
	// no compartmentlights_sw: and no push button: nothing else would withdraw it
	CHECK(compartment_lights_switch::withdraws_other_request(false, false, false));
	CHECK(compartment_lights_switch::withdraws_other_request(false, true, false));
}

namespace {

// the compartment lights controls a cab's .mmd may have
struct cab_controls {
	bool has_switch{false}; // compartmentlights_sw:
	bool switch_is_toggle{true}; // its type (a gauge is a toggle type unless declared otherwise)
	bool has_on_button{false}; // compartmentlightson_sw:
	bool has_off_button{false}; // compartmentlightsoff_sw:
};

// the effect of the cab's commands on the vehicle, as TTrain::OnCommand_compartmentlightsenable/disable/toggle do it
// NOTE: LightsCheck() runs between the press and the release, as the vehicle update does between two frames
void lights_check(vehicle &Car) {
	Car->LightsCheck(0.01);
}

void enable(vehicle &Car, cab_controls const &Cab) {
	// press
	Car->CompartmentLightsSwitch(true, range_t::local);
	if (compartment_lights_switch::withdraws_other_request(Cab.has_switch, Cab.switch_is_toggle, Cab.has_off_button)) {
		Car->CompartmentLightsSwitchOff(false, range_t::local);
	}
	lights_check(Car);
	// release
	if (Cab.has_switch && false == Cab.switch_is_toggle) {
		Car->CompartmentLightsSwitch(false, range_t::local);
		Car->CompartmentLightsSwitchOff(false, range_t::local);
	}
	if (Cab.has_on_button) {
		Car->CompartmentLightsSwitch(false, range_t::local);
	}
	lights_check(Car);
}

void disable(vehicle &Car, cab_controls const &Cab) {
	// press
	Car->CompartmentLightsSwitchOff(true, range_t::local);
	if (compartment_lights_switch::withdraws_other_request(Cab.has_switch, Cab.switch_is_toggle, Cab.has_on_button)) {
		Car->CompartmentLightsSwitch(false, range_t::local);
	}
	lights_check(Car);
	// release
	if (Cab.has_switch && false == Cab.switch_is_toggle) {
		Car->CompartmentLightsSwitch(false, range_t::local);
		Car->CompartmentLightsSwitchOff(false, range_t::local);
	}
	if (Cab.has_off_button) {
		Car->CompartmentLightsSwitchOff(false, range_t::local);
	}
	lights_check(Car);
}

// the keyboard toggle picks the command from the state, on the press
// NOTE: TTrain::OnCommand_compartmentlightstoggle picks it again on the release; for the cabs the toggle is checked with
// here that comes to the same, their release does nothing (no push type control) or the same thing in both commands
void toggle(vehicle &Car, cab_controls const &Cab) {
	auto const &lights{Car->CompartmentLights};
	if (false == lights.is_active && false == lights.is_enabled) {
		enable(Car, Cab);
	}
	else {
		disable(Car, Cab);
	}
}

// the Elf (21WE-a), its compartment lights started by hand, with the converter running
void power_up(vehicle &Car) {
	REQUIRE(Car->CompartmentLights.start_type == start_t::manual);
	Car->Power110vIsAvailable = true;
	lights_check(Car);
	REQUIRE_FALSE(Car->CompartmentLights.is_active);
}

} // namespace

TEST_CASE("compartment lights switch: the keyboard toggle works every time in a cab without the switch") {
	vehicle car{definition::fiz("elf_v1/21we-a")};
	power_up(car);
	cab_controls const cab{}; // no compartmentlights_sw:, no push buttons, like the CD 810 cab A

	for (auto press = 1; press <= 6; ++press) {
		CAPTURE(press);
		toggle(car, cab);
		// odd presses turn the lights on, even ones off
		CHECK(car->CompartmentLights.is_active == (press % 2 == 1));
		// one request at a time
		CHECK_FALSE((car->CompartmentLights.is_enabled && car->CompartmentLights.is_disabled));
	}
}

TEST_CASE("compartment lights switch: the keyboard toggle in a cab with the two-state switch, as before") {
	vehicle car{definition::fiz("elf_v1/21we-a")};
	power_up(car);
	cab_controls const cab{.has_switch = true, .switch_is_toggle = true};

	for (auto press = 1; press <= 4; ++press) {
		CAPTURE(press);
		toggle(car, cab);
		CHECK(car->CompartmentLights.is_active == (press % 2 == 1));
		// the switch holds the request of its position
		CHECK(car->CompartmentLights.is_enabled == (press % 2 == 1));
		CHECK(car->CompartmentLights.is_disabled == (press % 2 == 0));
	}
}

TEST_CASE("compartment lights switch: the push type switch withdraws both requests when released, as before") {
	vehicle car{definition::fiz("elf_v1/21we-a")};
	power_up(car);
	cab_controls const cab{.has_switch = true, .switch_is_toggle = false};

	for (auto press = 1; press <= 4; ++press) {
		CAPTURE(press);
		toggle(car, cab);
		CHECK(car->CompartmentLights.is_active == (press % 2 == 1));
		CHECK_FALSE(car->CompartmentLights.is_enabled);
		CHECK_FALSE(car->CompartmentLights.is_disabled);
	}
}

TEST_CASE("compartment lights switch: the on and off push buttons latch the lights, as before") {
	vehicle car{definition::fiz("elf_v1/21we-a")};
	power_up(car);
	cab_controls const cab{.has_on_button = true, .has_off_button = true};

	for (auto round = 1; round <= 2; ++round) {
		CAPTURE(round);
		// each button withdraws its own request when it's released, the lights stay as they were put
		enable(car, cab);
		CHECK(car->CompartmentLights.is_active);
		CHECK_FALSE(car->CompartmentLights.is_enabled);
		CHECK_FALSE(car->CompartmentLights.is_disabled);
		disable(car, cab);
		CHECK_FALSE(car->CompartmentLights.is_active);
		CHECK_FALSE(car->CompartmentLights.is_enabled);
		CHECK_FALSE(car->CompartmentLights.is_disabled);
	}
}
