#include "stdafx.h"
#include "doctest.h"

#include "simulation/simulationtestrules.h"

#include <array>
#include <exception>
#include <new>
#include <stdexcept>
#include <vector>

// the test mode of the simulator (eu07 --test, eu07/maszyna#541): its exit codes, command line and end conditions

namespace {

using simulation::test_result;

simulation::test_command_line parse(std::vector<char const *> Arguments)
{
	Arguments.insert(Arguments.begin(), "eu07");
	return simulation::parse_test_command_line(Arguments);
}

// a frame of the default 0.03 s
simulation::test_frame frame(bool const Atrest, std::optional<double> const Reported = std::nullopt, std::string_view const Reason = {})
{
	return simulation::test_frame{0.03, Reported, Reason, Atrest};
}

// runs frames until the test ends or Seconds of simulation time go by
std::optional<simulation::test_verdict> run(simulation::test_progress &Progress, simulation::test_limits const &Limits, double const Seconds,
                                            simulation::test_frame const &Frame)
{
	while (Progress.time < Seconds)
	{
		if (auto verdict{simulation::evaluate_frame(Progress, Frame, Limits)})
			return verdict;
	}
	return std::nullopt;
}

} // namespace

TEST_CASE("test mode: the exit codes keep clear of the ones of a killed or aborted process")
{
	std::array const results{test_result::pass, test_result::fail, test_result::signal_passed_at_danger, test_result::timeout, test_result::standstill, test_result::error};
	for (auto const result : results)
	{
		auto const code{static_cast<int>(result)};
		CAPTURE(code);
		CHECK(code != 1); // TerminateProcess, most programs' failure
		CHECK(code != 3); // abort() on Windows
		CHECK(code < 126); // shells, signals on Linux
		CHECK(simulation::to_string(result) != "unknown");
	}
	// the codes are public: scripts and CI read them
	CHECK(static_cast<int>(test_result::pass) == 0);
	CHECK(static_cast<int>(test_result::fail) == 10);
	CHECK(static_cast<int>(test_result::signal_passed_at_danger) == 11);
	CHECK(static_cast<int>(test_result::timeout) == 12);
	CHECK(static_cast<int>(test_result::standstill) == 13);
	CHECK(static_cast<int>(test_result::error) == 14);
}

TEST_CASE("test mode: seconds on the command line are a number, not negative")
{
	CHECK(simulation::parse_seconds("240").value_or(-1.0) == doctest::Approx(240.0));
	CHECK(simulation::parse_seconds("0.5").value_or(-1.0) == doctest::Approx(0.5));
	CHECK(simulation::parse_seconds("0").value_or(-1.0) == doctest::Approx(0.0));
	CHECK_FALSE(simulation::parse_seconds("").has_value());
	CHECK_FALSE(simulation::parse_seconds("-1").has_value());
	CHECK_FALSE(simulation::parse_seconds("60s").has_value());
	CHECK_FALSE(simulation::parse_seconds("abc").has_value());
	CHECK_FALSE(simulation::parse_seconds("inf").has_value());
	CHECK_FALSE(simulation::parse_seconds("nan").has_value());
}

TEST_CASE("test mode: the command line")
{
	SUBCASE("without --test there is no test")
	{
		auto const commandline{parse({"-s", "td.scn", "-v", "ep07-424"})};
		CHECK_FALSE(commandline.enabled);
		CHECK(commandline.error.empty());
	}
	SUBCASE("--test and its limits, in any order")
	{
		auto const commandline{parse({"--test-wall-timeout", "900", "-s", "td.scn", "--test", "--test-timeout", "240"})};
		CHECK(commandline.enabled);
		CHECK(commandline.timeout.value_or(-1.0) == doctest::Approx(240.0));
		CHECK(commandline.wall_timeout.value_or(-1.0) == doctest::Approx(900.0));
		CHECK(commandline.error.empty());
	}
	SUBCASE("a limit not given stays unset")
	{
		auto const commandline{parse({"--test", "-s", "td.scn"})};
		CHECK_FALSE(commandline.timeout.has_value());
		CHECK_FALSE(commandline.wall_timeout.has_value());
	}
	SUBCASE("a limit which isn't a number is an error, not a limit turned off")
	{
		auto const commandline{parse({"--test", "-s", "td.scn", "--test-timeout", "4m"})};
		CHECK(commandline.enabled);
		CHECK_FALSE(commandline.timeout.has_value());
		CHECK(commandline.error == "--test-timeout needs a number of seconds");
	}
	SUBCASE("a limit without its value is an error")
	{
		auto const commandline{parse({"--test", "-s", "td.scn", "--test-wall-timeout"})};
		CHECK(commandline.error == "--test-wall-timeout needs a number of seconds");
	}
	SUBCASE("the first error is the one told")
	{
		auto const commandline{parse({"--test", "--test-timeout", "x", "--test-wall-timeout", "y"})};
		CHECK(commandline.error == "--test-timeout needs a number of seconds");
	}
	SUBCASE("values of other options aren't taken for options")
	{
		auto const commandline{parse({"-s", "--test", "-ini", "--test-timeout", "-v", "--test-wall-timeout", "-seed", "--test"})};
		CHECK_FALSE(commandline.enabled);
		CHECK(commandline.error.empty());
	}
	SUBCASE("-edit takes no option for its scenery, which is optional")
	{
		// a scenery after -edit or none: the option after it is read either way
		CHECK(parse({"-edit", "--test", "-s", "td.scn"}).enabled);
		CHECK(parse({"-edit", "td.scn", "--test"}).enabled);
		// -edit leaves -s alone, which takes --test for its scenery, as the simulator reads it
		CHECK_FALSE(parse({"-edit", "-s", "--test"}).enabled);
	}
}

TEST_CASE("test mode: the real time limit")
{
	SUBCASE("given, it is the limit, 0 turns it off")
	{
		CHECK(simulation::wall_clock_limit(120.0, 3600.0) == doctest::Approx(120.0));
		CHECK(simulation::wall_clock_limit(0.0, 3600.0) == doctest::Approx(0.0));
	}
	SUBCASE("not given, it is the simulation time limit and time for the loading")
	{
		CHECK(simulation::wall_clock_limit(std::nullopt, 240.0) == doctest::Approx(240.0 + simulation::wall_timeout_loading_allowance));
		CHECK(simulation::wall_clock_limit(std::nullopt, 3600.0) == doctest::Approx(3600.0 + simulation::wall_timeout_loading_allowance));
	}
	SUBCASE("no simulation time limit, no real time limit unless given")
	{
		CHECK(simulation::wall_clock_limit(std::nullopt, 0.0) == doctest::Approx(0.0));
		CHECK(simulation::wall_clock_limit(300.0, 0.0) == doctest::Approx(300.0));
	}
}

TEST_CASE("test mode: the end conditions")
{
	simulation::test_limits limits;
	simulation::test_progress progress;

	SUBCASE("nothing happens, nothing ends")
	{
		CHECK_FALSE(simulation::evaluate_frame(progress, frame(false), limits).has_value());
		CHECK(progress.time == doctest::Approx(0.03));
		CHECK(progress.resttime == doctest::Approx(0.0));
	}
	SUBCASE("a cell nobody wrote to says nothing")
	{
		CHECK_FALSE(simulation::evaluate_frame(progress, frame(false, -1.0), limits).has_value());
		CHECK_FALSE(simulation::evaluate_frame(progress, frame(false, 2.0), limits).has_value());
		CHECK_FALSE(progress.passrequested);
	}
	SUBCASE("0 fails at once, with the text of the cell")
	{
		auto const verdict{simulation::evaluate_frame(progress, frame(false, 0.0, "wrong_track"), limits)};
		REQUIRE(verdict.has_value());
		CHECK(verdict->result == test_result::fail);
		CHECK(verdict->reason == "__test.result set to 0 (wrong_track)");
	}
	SUBCASE("1 passes once the tested trains have stood for a second")
	{
		CHECK_FALSE(simulation::evaluate_frame(progress, frame(false, 1.0), limits).has_value());
		CHECK(progress.passrequested);
		// still moving: the pass waits
		CHECK_FALSE(run(progress, limits, 30.0, frame(false, 1.0)).has_value());
		auto const stoppedat{progress.time};
		auto const verdict{run(progress, limits, 60.0, frame(true, 1.0))};
		REQUIRE(verdict.has_value());
		CHECK(verdict->result == test_result::pass);
		CHECK(progress.time - stoppedat == doctest::Approx(simulation::pass_settle_time).epsilon(0.05));
	}
	SUBCASE("the pass stays requested when the cell changes back")
	{
		CHECK_FALSE(simulation::evaluate_frame(progress, frame(false, 1.0), limits).has_value());
		auto const verdict{run(progress, limits, 60.0, frame(true, -1.0))};
		REQUIRE(verdict.has_value());
		CHECK(verdict->result == test_result::pass);
	}
	SUBCASE("1 with test.no_movement_before_pass no passes at once")
	{
		limits.no_movement_before_pass = false;
		auto const verdict{simulation::evaluate_frame(progress, frame(false, 1.0, "reached_the_stop"), limits)};
		REQUIRE(verdict.has_value());
		CHECK(verdict->result == test_result::pass);
		CHECK(verdict->reason == "__test.result set to 1 (reached_the_stop)");
	}
	SUBCASE("standing still without a result ends after test.standstill")
	{
		limits.standstill = 120.0;
		auto const verdict{run(progress, limits, 1000.0, frame(true))};
		REQUIRE(verdict.has_value());
		CHECK(verdict->result == test_result::standstill);
		CHECK(progress.time == doctest::Approx(120.0).epsilon(0.001));
	}
	SUBCASE("a move starts the standstill over")
	{
		limits.standstill = 120.0;
		CHECK_FALSE(run(progress, limits, 100.0, frame(true)).has_value());
		CHECK_FALSE(simulation::evaluate_frame(progress, frame(false), limits).has_value());
		CHECK(progress.resttime == doctest::Approx(0.0));
		CHECK_FALSE(run(progress, limits, 200.0, frame(true)).has_value());
	}
	SUBCASE("test.pass_on_standstill makes the standstill a pass")
	{
		limits.standstill = 10.0;
		limits.pass_on_standstill = true;
		auto const verdict{run(progress, limits, 1000.0, frame(true))};
		REQUIRE(verdict.has_value());
		CHECK(verdict->result == test_result::pass);
	}
	SUBCASE("test.standstill 0 never ends the test")
	{
		limits.standstill = 0.0;
		limits.timeout = 0.0;
		CHECK_FALSE(run(progress, limits, 5000.0, frame(true)).has_value());
	}
	SUBCASE("no result by test.timeout")
	{
		limits.timeout = 240.0;
		auto const verdict{run(progress, limits, 1000.0, frame(false))};
		REQUIRE(verdict.has_value());
		CHECK(verdict->result == test_result::timeout);
		CHECK(verdict->reason == "no result after 240 s");
		CHECK(progress.time == doctest::Approx(240.0).epsilon(0.001));
	}
	SUBCASE("a result in the last frame wins over the timeout")
	{
		limits.timeout = 0.03;
		auto const verdict{simulation::evaluate_frame(progress, frame(false, 0.0), limits)};
		REQUIRE(verdict.has_value());
		CHECK(verdict->result == test_result::fail);
	}
}

TEST_CASE("test mode: what ended a run the simulator didn't end")
{
	SUBCASE("an exception says what it was")
	{
		CHECK(simulation::describe_exception(std::make_exception_ptr(std::runtime_error("no scenery"))) == "critical error: no scenery");
		// e.g. std::stof on a token of a scenery which isn't a number
		CHECK(simulation::describe_exception(std::make_exception_ptr(std::invalid_argument("stof"))) == "critical error: stof");
		CHECK(simulation::describe_exception(std::make_exception_ptr(std::bad_alloc())).starts_with("critical error: "));
	}
	SUBCASE("something thrown which isn't a std::exception")
	{
		CHECK(simulation::describe_exception(std::make_exception_ptr(42)) == "critical error: an exception of unknown type");
	}
	SUBCASE("terminated without an exception")
	{
		CHECK(simulation::describe_exception(nullptr) == "the simulator was terminated");
	}
}
