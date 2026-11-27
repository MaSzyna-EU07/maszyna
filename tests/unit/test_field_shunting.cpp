#include "stdafx.h"
#include "doctest.h"
#include "scenario.h"

#include "vehicle/fieldshunting.h"

// the field shunting of the diesel-electric locomotives, set by the driver or by the shunting relays, and the switch
// of the SU45 and the SP45 that lets the relays shunt the field ("załączenie bocznikowania", eu07/maszyna#167)

using test::definition;
using test::scenario;
using namespace test::channels;

TEST_CASE("field shunting: who shunts the field of a diesel-electric, by AutoRelay, FieldShuntSwitch and the switch") {
	using enum fieldshunting::control;
	struct setting {
		int type; // AutoRelay: 0 No, 1 Yes, 2 Optional
		bool fieldshuntswitch; // FieldShuntSwitch
		bool flag; // the switch, AutoRelayFlag
		fieldshunting::control expected;
	};
	for (auto const &[type, fieldshuntswitch, flag, expected] :
	     {setting{0, false, false, manual}, setting{1, false, true, automatic},
	      // AutoRelay=Optional alone, as before: off, the driver shunts by hand
	      setting{2, false, true, automatic}, setting{2, false, false, manual},
	      // with FieldShuntSwitch=Yes the switch of the field shunting: off, nobody
	      setting{2, true, true, automatic}, setting{2, true, false, off},
	      // FieldShuntSwitch=Yes without the switch (AutoRelay=Yes or No) changes nothing
	      setting{1, true, true, automatic}, setting{0, true, false, manual},
	      // AutoRelaySwitch from a coupled vehicle turns the flag of every one off, as before
	      setting{1, false, false, manual}, setting{1, true, false, manual}}) {
		CAPTURE(type);
		CAPTURE(fieldshuntswitch);
		CAPTURE(flag);
		CHECK(fieldshunting::diesel_electric(type, fieldshuntswitch, flag) == expected);
		CHECK(fieldshunting::has_switch(type, fieldshuntswitch) == (type == 2 && fieldshuntswitch));
	}
}

TEST_CASE("field shunting: the SU45 with its switch, off at full power and switched on and off on the move") {
	// the SU45 (RelayType 45) given the switch (AutoRelay=Optional, FieldShuntSwitch=Yes) its cab has, with a coach:
	// off, the field stays full and the second controller can't shunt it; on, the relays shunt it as they do with
	// AutoRelay=Yes; off again, the shunting drops
	scenario scenario{"traction/field shunting switch/su45", motion() + traction()};
	scenario.train().add(definition::fiz("su45_v2/301d").set("Cntrl", "AutoRelay", "Optional").set("Cntrl", "FieldShuntSwitch", "Yes"), 1);
	scenario.add("bdhpumn_v3/bdhpumn");
	REQUIRE(scenario.start_up());
	auto &su45{scenario.driver().controlling};
	REQUIRE(su45.AutoRelayType == 2);
	REQUIRE(su45.FieldShuntSwitch);
	// the driver switches it on as the AI does, getting the locomotive ready
	CHECK(su45.AutoRelayFlag);
	scenario.act("field shunting off");
	CHECK(su45.AutoRelaySwitch(false));
	scenario.get_going();
	scenario.full_power(120.0);
	CHECK(su45.IsMainCtrlMaxPowerPos());
	CHECK(su45.StLinFlag);
	CHECK(su45.ScndCtrlPos == 0);
	CHECK_FALSE(su45.IncScndCtrl(1));
	scenario.wait(1.0);
	CHECK(su45.ScndCtrlPos == 0);
	// over 50 km/h, where the relays of the SU45 let the third stage in
	CHECK(su45.Vel > 50.0);
	scenario.act("field shunting on");
	CHECK(su45.AutoRelaySwitch(true));
	scenario.wait(30.0);
	CHECK(su45.ScndCtrlPos == su45.ScndCtrlPosNo);
	scenario.act("field shunting off on the move");
	CHECK(su45.AutoRelaySwitch(false));
	scenario.wait(1.0);
	CHECK(su45.ScndCtrlPos == 0);
	scenario.check();
}

TEST_CASE("field shunting: a diesel-electric with AutoRelay=Optional alone shunts by hand with the switch off, as before") {
	// the SU45 with AutoRelay=Optional but no FieldShuntSwitch: the switch is the old one, off hands the shunting to the
	// second controller, and the start-up (as the AI) leaves it off
	scenario scenario{"traction/field shunting switch/su45 optional alone", motion() + traction()};
	scenario.train().add(definition::fiz("su45_v2/301d").set("Cntrl", "AutoRelay", "Optional"), 1);
	scenario.add("bdhpumn_v3/bdhpumn");
	REQUIRE(scenario.start_up());
	auto &su45{scenario.driver().controlling};
	REQUIRE(su45.AutoRelayType == 2);
	REQUIRE_FALSE(su45.FieldShuntSwitch);
	CHECK_FALSE(su45.AutoRelayFlag);
	scenario.get_going();
	scenario.full_power(120.0);
	CHECK(su45.IsMainCtrlMaxPowerPos());
	CHECK(su45.StLinFlag);
	// over 50 km/h: the relays don't shunt, the driver does
	CHECK(su45.Vel > 50.0);
	CHECK(su45.ScndCtrlPos == 0);
	scenario.act("field shunting by hand");
	CHECK(su45.IncScndCtrl(1));
	scenario.wait(1.0);
	CHECK(su45.ScndCtrlPos == 1);
	scenario.check();
}
