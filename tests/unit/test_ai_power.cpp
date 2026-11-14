#include "stdafx.h"
#include "doctest.h"

#include "vehicle/aipower.h"

// the AI driver of an electric vehicle standing with its pantographs up and no voltage at them: what it makes of
// it, and when it tells of it in the log (eu07/maszyna#421)

namespace {

using ai_power::stall;

// standing with the pantographs up, past the end of the overhead wire
ai_power::state past_the_wire() {
	ai_power::state state;
	state.collector = true;
	state.power = false;
	state.standing = true;
	state.pantographs_up = true;
	state.wire = false;
	return state;
}

} // namespace

TEST_CASE("ai power: standing with the pantographs up and no wire above them is a stall") {
	CHECK(ai_power::check(past_the_wire()) == stall::no_wire);
}

TEST_CASE("ai power: a wire above the raised pantographs which carries no voltage is told apart") {
	auto state{past_the_wire()};
	state.wire = true;

	CHECK(ai_power::check(state) == stall::no_voltage);
}

TEST_CASE("ai power: with the voltage there, or without a pantograph to take it, there is nothing to tell") {
	auto powered{past_the_wire()};
	powered.power = true; // at the pantographs, or from the rest of the train through its high voltage cable
	auto diesel{past_the_wire()};
	diesel.collector = false;

	CHECK(ai_power::check(powered) == stall::none);
	CHECK(ai_power::check(diesel) == stall::none);
}

TEST_CASE("ai power: moving, or the pantographs not up yet, it only waits") {
	auto moving{past_the_wire()};
	moving.standing = false; // coasting through a gap of the wire, say
	auto rising{past_the_wire()};
	rising.pantographs_up = false; // the air for the pantographs still being pumped up

	CHECK(ai_power::check(moving) == stall::waiting);
	CHECK(ai_power::check(rising) == stall::waiting);
}

TEST_CASE("ai power: a stall is reported once it lasted report_delay, and only once") {
	ai_power::reporter reporter;

	CHECK_FALSE(reporter.update(stall::no_wire, 100.0).has_value());
	CHECK_FALSE(reporter.update(stall::no_wire, 100.0 + ai_power::report_delay - 0.5).has_value());
	auto const report{reporter.update(stall::no_wire, 100.0 + ai_power::report_delay)};
	REQUIRE(report.has_value());
	CHECK(*report == stall::no_wire);
	// the driver keeps trying to prepare the vehicle, without telling of it again
	CHECK_FALSE(reporter.update(stall::no_wire, 100.0 + ai_power::report_delay + 1.0).has_value());
	CHECK_FALSE(reporter.update(stall::no_wire, 1000.0).has_value());
}

TEST_CASE("ai power: the voltage coming back ends the stall, the next one is reported again") {
	ai_power::reporter reporter;
	reporter.update(stall::no_wire, 0.0);
	REQUIRE(reporter.update(stall::no_wire, ai_power::report_delay).has_value());

	// towed back under the wire: the vehicle gets prepared
	CHECK_FALSE(reporter.update(stall::none, 100.0).has_value());
	// and later stands without voltage again, under a wire switched off this time
	CHECK_FALSE(reporter.update(stall::no_voltage, 200.0).has_value());
	auto const report{reporter.update(stall::no_voltage, 200.0 + ai_power::report_delay)};
	REQUIRE(report.has_value());
	CHECK(*report == stall::no_voltage);
}

TEST_CASE("ai power: moving or lowering the pantographs restarts the count but doesn't end a reported stall") {
	ai_power::reporter reporter;
	reporter.update(stall::no_wire, 0.0);
	// the vehicle moves before the stall is reported: the count starts again once it stands
	CHECK_FALSE(reporter.update(stall::waiting, 20.0).has_value());
	CHECK_FALSE(reporter.update(stall::no_wire, 25.0).has_value());
	CHECK_FALSE(reporter.update(stall::no_wire, 25.0 + ai_power::report_delay - 1.0).has_value());
	REQUIRE(reporter.update(stall::no_wire, 25.0 + ai_power::report_delay).has_value());
	// still with no voltage, the vehicle moved (pushed a bit) and stands again: no second report
	CHECK_FALSE(reporter.update(stall::waiting, 100.0).has_value());
	CHECK_FALSE(reporter.update(stall::no_wire, 110.0).has_value());
	CHECK_FALSE(reporter.update(stall::no_wire, 110.0 + 2.0 * ai_power::report_delay).has_value());
}

TEST_CASE("ai power: the report names the vehicle, where it stands and why") {
	auto const nowire{ai_power::message(stall::no_wire, "et21-367", {1817.4, 0.38, 4785.94})};
	auto const novoltage{ai_power::message(stall::no_voltage, "et21-367", {1817.4, 0.38, 4785.94})};

	CHECK(nowire == "AI [et21-367]: can't start, no overhead wire above its raised pantographs (at 1817.40, 0.38, 4785.94); it waits for the voltage");
	CHECK(novoltage.find("no voltage in the overhead wire") != std::string::npos);
	CHECK(novoltage.find("et21-367") != std::string::npos);
}
