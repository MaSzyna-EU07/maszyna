#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

#include "vehicle/compartmentlightsswitch.h"

using test::definition;
using test::vehicle;

// the compartment lights commands of the cab (TTrain::OnCommand_compartmentlightsenable/disable/toggle): whether the
// command for one request (on: CompartmentLights.is_enabled, off: is_disabled) also withdraws the other one. In a cab
// without compartmentlights_sw: and without the push buttons (the CD 810 cab A) the keyboard toggle, ctrl+;, turned
// the lights on and off once, then never again: the off request stayed set alongside the on one. In a cab with the
// compartmentlightson_sw: / compartmentlightsoff_sw: push buttons (the EN57-2000) the toggle sent the key release to the
// other command than the press, picked again from the state the press had changed: the pressed button never withdrew
// its request and the lights were off only while ctrl+; was held.

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

void enable_press(vehicle &Car, cab_controls const &Cab) {
	Car->CompartmentLightsSwitch(true, range_t::local);
	if (compartment_lights_switch::withdraws_other_request(Cab.has_switch, Cab.switch_is_toggle, Cab.has_off_button)) {
		Car->CompartmentLightsSwitchOff(false, range_t::local);
	}
}

void enable_release(vehicle &Car, cab_controls const &Cab) {
	if (Cab.has_switch && false == Cab.switch_is_toggle) {
		Car->CompartmentLightsSwitch(false, range_t::local);
		Car->CompartmentLightsSwitchOff(false, range_t::local);
	}
	if (Cab.has_on_button) {
		Car->CompartmentLightsSwitch(false, range_t::local);
	}
}

void disable_press(vehicle &Car, cab_controls const &Cab) {
	Car->CompartmentLightsSwitchOff(true, range_t::local);
	if (compartment_lights_switch::withdraws_other_request(Cab.has_switch, Cab.switch_is_toggle, Cab.has_on_button)) {
		Car->CompartmentLightsSwitch(false, range_t::local);
	}
}

void disable_release(vehicle &Car, cab_controls const &Cab) {
	if (Cab.has_switch && false == Cab.switch_is_toggle) {
		Car->CompartmentLightsSwitch(false, range_t::local);
		Car->CompartmentLightsSwitchOff(false, range_t::local);
	}
	if (Cab.has_off_button) {
		Car->CompartmentLightsSwitchOff(false, range_t::local);
	}
}

// a click on the control of the command (or its own key): the press, then the release
void enable(vehicle &Car, cab_controls const &Cab) {
	enable_press(Car, Cab);
	lights_check(Car);
	enable_release(Car, Cab);
	lights_check(Car);
}

void disable(vehicle &Car, cab_controls const &Cab) {
	disable_press(Car, Cab);
	lights_check(Car);
	disable_release(Car, Cab);
	lights_check(Car);
}

// the keyboard toggle picks the command from the state on the press, the key held down for a few frames; the release
// goes to both commands, the press changed the state it would be picked from
void toggle(vehicle &Car, cab_controls const &Cab) {
	auto const &lights{Car->CompartmentLights};
	if (false == lights.is_active && false == lights.is_enabled) {
		enable_press(Car, Cab);
	}
	else {
		disable_press(Car, Cab);
	}
	for (auto frame = 0; frame < 3; ++frame) {
		lights_check(Car);
	}
	enable_release(Car, Cab);
	disable_release(Car, Cab);
	lights_check(Car);
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

TEST_CASE("compartment lights switch: the keyboard toggle latches the lights in a cab with the on and off push buttons") {
	vehicle car{definition::fiz("elf_v1/21we-a")};
	power_up(car);
	cab_controls const cab{.has_on_button = true, .has_off_button = true}; // no compartmentlights_sw:, like the EN57-2000

	for (auto press = 1; press <= 6; ++press) {
		CAPTURE(press);
		toggle(car, cab);
		// odd presses turn the lights on, even ones off, and they stay so after the key is released
		CHECK(car->CompartmentLights.is_active == (press % 2 == 1));
		// both push buttons are back in the neutral position, neither request is left behind
		CHECK_FALSE(car->CompartmentLights.is_enabled);
		CHECK_FALSE(car->CompartmentLights.is_disabled);
	}
}

TEST_CASE("compartment lights switch: the keyboard toggle and the on and off push buttons work together") {
	vehicle car{definition::fiz("elf_v1/21we-a")};
	power_up(car);
	cab_controls const cab{.has_on_button = true, .has_off_button = true};

	toggle(car, cab);
	REQUIRE(car->CompartmentLights.is_active);
	// the off button turns off what the key turned on...
	disable(car, cab);
	CHECK_FALSE(car->CompartmentLights.is_active);
	// ...and the key turns on what the off button turned off
	toggle(car, cab);
	CHECK(car->CompartmentLights.is_active);
	toggle(car, cab);
	CHECK_FALSE(car->CompartmentLights.is_active);
	enable(car, cab);
	CHECK(car->CompartmentLights.is_active);
	toggle(car, cab);
	CHECK_FALSE(car->CompartmentLights.is_active);
	CHECK_FALSE(car->CompartmentLights.is_enabled);
	CHECK_FALSE(car->CompartmentLights.is_disabled);
}
