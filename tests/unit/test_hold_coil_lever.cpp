#include "stdafx.h"
#include "doctest.h"

#include "vehicle/holdcoillever.h"

// the lever of the line breaker's hold coil on the ET22 and the EP09 stays where it's put (eu07/maszyna#364), when the
// cab declares its main_off_bt control with "type: toggle"; otherwise main_off_bt is the push button it always was

TEST_CASE("hold coil lever: only a main_off_bt declared as a toggle switch is the bistable lever (eu07/maszyna#364)") {
	CHECK(hold_coil_lever::is_bistable(true, true));
	// declared as a push button
	CHECK_FALSE(hold_coil_lever::is_bistable(true, false));
	// no type given: a control defaults to the toggle type, which existing cabs relied on for the push button
	CHECK_FALSE(hold_coil_lever::is_bistable(false, true));
}

TEST_CASE("hold coil lever: operated, it goes over to the other position and stays there") {
	// on: the hold coil fed, the line breaker can close
	CHECK_FALSE(hold_coil_lever::cuts_off(0.f));
	auto const off{hold_coil_lever::operated(0.f)};
	CHECK(off == doctest::Approx(1.f));
	CHECK(hold_coil_lever::cuts_off(off));
	// operated again it goes back on
	auto const on{hold_coil_lever::operated(off)};
	CHECK(on == doctest::Approx(0.f));
	CHECK_FALSE(hold_coil_lever::cuts_off(on));
	// caught on the way between the positions it goes to the far one
	CHECK(hold_coil_lever::operated(0.3f) == doctest::Approx(1.f));
	CHECK(hold_coil_lever::operated(0.7f) == doctest::Approx(0.f));
}
