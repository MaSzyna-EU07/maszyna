#include "stdafx.h"
#include "doctest.h"
#include "consist.h"
#include "drive.h"

#include "vehicle/linebreaker.h"

// the line breaker (WS) controls of the cab (eu07/maszyna#394)

using test::consist;
using test::definition;
using linebreaker::state;

namespace {

// the line breaker lever of the EP05 (main_sw), a single two-state lever, the way TTrain works it: the state
// the cab keeps of the line breaker follows the vehicle, the breaker closes while the lever is on and can close
struct lever {
	TMoverParameters &mover;
	state breaker{state::open}; // as TTrain::m_linebreakerstate
	bool on{false};

	void press() {
		switch (linebreaker::pressed(breaker, true, on)) {
		case linebreaker::toggle::close:
			on = true;
			break;
		case linebreaker::toggle::open:
			on = false;
			if (breaker == state::closed && mover.MainSwitch(false)) {
				breaker = state::open;
			}
			break;
		default:
			break;
		}
	}
	// TTrain::Update()
	void update() {
		if (breaker == state::open && mover.Mains) {
			breaker = state::closed;
		}
		if (breaker == state::closed && false == mover.Mains) {
			breaker = state::open;
		}
		if (on && breaker == state::open && mover.MainSwitchCheck()) {
			breaker = mover.MainSwitch(true) ? state::closed : state::open;
		}
	}
};

} // namespace

TEST_CASE("line breaker: a press of the toggle closes an open breaker and opens a closed one") {
	using enum linebreaker::toggle;
	for (auto const latching : {false, true}) {
		CHECK(linebreaker::pressed(state::open, latching, false) == close);
		CHECK(linebreaker::pressed(state::closed, latching, true) == open);
		CHECK(linebreaker::pressed(state::closed, latching, false) == open);
		// freshly closed, the cab finishes closing it first
		CHECK(linebreaker::pressed(state::closing, latching, true) == none);
	}
	// a lever left on over an open breaker goes off; push buttons have no position to keep
	CHECK(linebreaker::pressed(state::open, true, true) == open);
	CHECK(linebreaker::pressed(state::open, false, true) == close);
}

TEST_CASE("line breaker: the EP05's lever goes off with the converter overload relay tripped (eu07/maszyna#394)") {
	consist train;
	auto &ep05{train.add(definition::fiz("ep05_v2/ep05"), 1).mover()};
	test::track const line{0.15, 0.0, test::catenary_3kv};
	auto const driver{test::driving(train)};
	REQUIRE(test::start_up(driver, line));
	REQUIRE(ep05.Mains);
	REQUIRE(ep05.ConverterFlag);
	lever ws{ep05, state::closed, true};

	// the converter overload relay knocks the line breaker out, as the cab does it when the converter is
	// switched on together with the compressor; the lever stays on
	ep05.ConvOvldFlag = true;
	ep05.MainSwitch(false, range_t::local);
	ws.update();
	REQUIRE_FALSE(ep05.Mains);
	REQUIRE(ws.breaker == state::open);
	CHECK(ws.on);
	// with the relay tripped the breaker doesn't close
	test::drive(driver, line, 5.0);
	CHECK_FALSE(ep05.MainSwitchCheck());

	// nothing holds the lever: it goes off, and on again, though the breaker stays open
	ws.press();
	CHECK_FALSE(ws.on);
	ws.press();
	CHECK(ws.on);
	ws.update();
	CHECK_FALSE(ep05.Mains);
	ws.press();
	CHECK_FALSE(ws.on);

	// the relay reset, with the converter switched off, the lever put on closes the breaker again
	ep05.ConverterSwitch(false, range_t::local);
	test::drive(driver, line, 1.0);
	CHECK(ep05.RelayReset(relay_t::primaryconverteroverload, range_t::local));
	CHECK_FALSE(ep05.ConvOvldFlag);
	ws.update();
	CHECK_FALSE(ep05.Mains); // the lever is off
	ws.press();
	ws.update();
	CHECK(ep05.Mains);
	CHECK(ws.breaker == state::closed);
	// and the next press opens it
	ws.press();
	CHECK_FALSE(ws.on);
	CHECK_FALSE(ep05.Mains);
}
