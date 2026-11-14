#include "stdafx.h"
#include "doctest.h"
#include "scenario.h"

#include "vehicle/fieldshunting.h"

// the field shunting of the diesel-electric locomotives, set by the driver or by the shunting relays, and the switch
// of the SU45 and the SP45 that lets the relays shunt the field ("załączenie bocznikowania", eu07/maszyna#167)

using test::definition;
using test::scenario;
using namespace test::channels;

TEST_CASE("field shunting: who shunts the field of a diesel-electric, by AutoRelay and its switch") {
	using enum fieldshunting::control;
	struct setting {
		int type; // AutoRelay: 0 No, 1 Yes, 2 Optional
		bool flag; // the switch, AutoRelayFlag
		fieldshunting::control control;
	};
	for (auto const &[type, flag, control] : {setting{0, false, manual}, setting{1, true, automatic}, setting{2, true, automatic}, setting{2, false, off},
	                                          // AutoRelaySwitch from a coupled vehicle turns the flag of every one off, as before
	                                          setting{1, false, manual}}) {
		CAPTURE(type);
		CAPTURE(flag);
		CHECK(fieldshunting::diesel_electric(type, flag) == control);
	}
}

TEST_CASE("field shunting: the SU45 with its switch, off at full power and switched on and off on the move") {
	// the SU45 (RelayType 45) given the switch (AutoRelay=Optional) its cab has, with a coach: off, the field stays
	// full and the second controller can't shunt it; on, the relays shunt it as they do with AutoRelay=Yes; off
	// again, the shunting drops
	scenario scenario{"traction/field shunting switch/su45", motion() + traction()};
	scenario.train().add(definition::fiz("su45_v2/301d").set("Cntrl", "AutoRelay", "Optional"), 1);
	scenario.add("bdhpumn_v3/bdhpumn");
	REQUIRE(scenario.start_up());
	auto &su45{scenario.driver().controlling};
	REQUIRE(su45.AutoRelayType == 2);
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
