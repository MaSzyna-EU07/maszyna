/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "simulation/simulationtestrules.h"

#include <charconv>
#include <cmath>
#include <format>

namespace {

// the cell holds what the scenery wrote, so the values are whole numbers; the margin only keeps a stray fraction out
bool holds(double const Value, double const Expected)
{
	return std::abs(Value - Expected) < 1e-6;
}

// the text of the result cell, as said after the result
std::string reason_suffix(std::string_view const Reason)
{
	return Reason.empty() ? std::string() : std::format(" ({})", Reason);
}

// options of the simulator which take a value, see eu07_application::init_settings()
bool takes_value(std::string_view const Option)
{
	return Option == "-s" || Option == "-v" || Option == "-seed" || Option == "-ini";
}

// a number of seconds given to Option, at Index of Arguments; the index is moved past the value
std::optional<double> option_seconds(std::span<char const *const> const Arguments, std::size_t &Index)
{
	if (Index + 1 >= Arguments.size())
		return std::nullopt;
	++Index;
	return simulation::parse_seconds(Arguments[Index]);
}

} // namespace

namespace simulation {

std::string_view to_string(test_result const Result)
{
	using enum test_result;
	switch (Result)
	{
	case pass: return "pass";
	case fail: return "fail";
	case signal_passed_at_danger: return "signal passed at danger";
	case timeout: return "timeout";
	case standstill: return "standstill";
	case error: return "error";
	}
	return "unknown";
}

std::optional<double> parse_seconds(std::string_view const Text)
{
	auto const *const last{Text.data() + Text.size()};
	double value{0.0};
	if (auto const [parsed, status]{std::from_chars(Text.data(), last, value)}; status != std::errc{} || parsed != last || false == std::isfinite(value) || value < 0.0)
		return std::nullopt;
	return value;
}

test_command_line parse_test_command_line(std::span<char const *const> const Arguments)
{
	test_command_line result;
	for (std::size_t i = 1; i < Arguments.size(); ++i)
	{
		std::string_view const option{Arguments[i]};
		if (option == "--test")
		{
			result.enabled = true;
		}
		else if (option == "--test-timeout" || option == "--test-wall-timeout")
		{
			auto const seconds{option_seconds(Arguments, i)};
			if (false == seconds.has_value())
			{
				// a typo would otherwise turn the limit off
				if (result.error.empty())
					result.error = std::format("{} needs a number of seconds", option);
				continue;
			}
			if (option == "--test-timeout")
				result.timeout = seconds;
			else
				result.wall_timeout = seconds;
		}
		else if (takes_value(option))
		{
			++i;
		}
	}
	return result;
}

std::string describe_exception(std::exception_ptr const &Exception)
{
	if (nullptr == Exception)
		return "the simulator was terminated";
	try
	{
		std::rethrow_exception(Exception);
	}
	catch (std::exception const &Error)
	{
		return std::format("critical error: {}", Error.what());
	}
	catch (...)
	{
		return "critical error: an exception of unknown type";
	}
}

double wall_clock_limit(std::optional<double> const Given, double const Simulationtimeout)
{
	if (Given.has_value())
		return *Given;
	return Simulationtimeout > 0.0 ? Simulationtimeout + wall_timeout_loading_allowance : 0.0;
}

std::optional<test_verdict> evaluate_frame(test_progress &Progress, test_frame const &Frame, test_limits const &Limits)
{
	using enum test_result;

	Progress.time += Frame.deltatime;

	if (Frame.reported.has_value())
	{
		auto const value{*Frame.reported};
		if (holds(value, 0.0))
			return test_verdict{fail, std::format("__test.result set to 0{}", reason_suffix(Frame.reason))};
		if (holds(value, 1.0))
		{
			if (false == Limits.no_movement_before_pass)
				return test_verdict{pass, std::format("__test.result set to 1{}", reason_suffix(Frame.reason))};
			Progress.passrequested = true;
		}
	}

	Progress.resttime = Frame.atrest ? Progress.resttime + Frame.deltatime : 0.0;

	if (Progress.passrequested && Progress.resttime >= pass_settle_time)
		return test_verdict{pass, "__test.result set to 1 and the tested trains stopped"};
	if (Limits.standstill > 0.0 && Progress.resttime >= Limits.standstill)
		return test_verdict{Limits.pass_on_standstill ? pass : standstill, std::format("the tested trains stood still for {:.0f} s", Limits.standstill)};
	if (Limits.timeout > 0.0 && Progress.time >= Limits.timeout)
		return test_verdict{timeout, std::format("no result after {:.0f} s", Limits.timeout)};
	return std::nullopt;
}

} // namespace simulation
