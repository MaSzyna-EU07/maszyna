/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "simulation/simulationtest.h"

#include "application/application.h"
#include "scene/scene.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "vehicle/DynObj.h"
#include "world/Event.h"
#include "world/MemCell.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <format>
#include <memory>

namespace {

// the cell holds what the scenery wrote, so the values are whole numbers; the margin only keeps a stray fraction out
bool holds(double const Value, double const Expected)
{
	return std::abs(Value - Expected) < 1e-6;
}

} // namespace

namespace simulation {

test_manager &test()
{
	static test_manager instance;
	return instance;
}

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

void test_manager::init_result_cell() const
{
	auto cell { std::make_unique<TMemCell>(scene::node_data{0, -1, std::string(result_cell), "memcell"}) };
	// neither of the reported values, so that a cell nobody wrote to says nothing
	cell->UpdateValues("", -1.0, 0.0, basic_event::flags::value1 | basic_event::flags::value2);
	simulation::Region->insert(cell.get());
	// the table owns its cells
	simulation::Memory.insert(cell.release());
}

void test_manager::start()
{
	if (m_started)
		return;
	m_started = true;
	m_time = 0.0;
	m_resttime = 0.0;
	if (simulation::Vehicles.sequence().empty())
	{
		// most likely the scenery wasn't found, or it isn't a test scenery
		finish(test_result::error, "the scenery has no vehicles");
		return;
	}
	// a pause set by the scenery or the settings would never be lifted, nobody is there to press the key
	Global.iPause = 0;
	WriteLog(std::format("test: started, timeout {:.0f} s, standstill {:.0f} s{}{}{}", Global.test.timeout, Global.test.standstill,
	                     Global.test.pass_on_standstill ? ", standstill passes" : "",
	                     Global.test.fail_if_stop_overran ? "" : ", signals passed at danger allowed",
	                     Global.test.no_movement_before_pass ? "" : ", pass without waiting for a stop"));
}

void test_manager::update(double const Deltatime)
{
	if (false == m_started || m_finished)
		return;

	m_time += Deltatime;

	if (auto const *cell = simulation::Memory.find(std::string(result_cell)))
	{
		auto const value { cell->Value1() };
		auto const reason { cell->Text().empty() ? std::string() : std::format(" ({})", cell->Text()) };
		if (holds(value, 0.0))
		{
			finish(test_result::fail, std::format("__test.result set to 0{}", reason));
			return;
		}
		if (holds(value, 1.0) && false == Global.test.no_movement_before_pass)
		{
			finish(test_result::pass, std::format("__test.result set to 1{}", reason));
			return;
		}
		if (holds(value, 1.0) && false == m_passrequested)
		{
			m_passrequested = true;
			WriteLog(std::format("test: pass requested at {:.1f} s{}, waiting for every vehicle to stop", m_time, reason));
		}
	}

	auto const atrest { std::ranges::none_of(simulation::Vehicles.sequence(),
	                                         [](TDynamicObject const *Vehicle) { return std::abs(Vehicle->GetVelocity()) > 0.01; }) };
	m_resttime = atrest ? m_resttime + Deltatime : 0.0;

	if (m_passrequested && m_resttime >= pass_settle_time)
	{
		finish(test_result::pass, "__test.result set to 1 and every vehicle stopped");
		return;
	}
	if (Global.test.standstill > 0.0 && m_resttime >= Global.test.standstill)
	{
		auto const reason { std::format("every vehicle stood still for {:.0f} s", Global.test.standstill) };
		finish(Global.test.pass_on_standstill ? test_result::pass : test_result::standstill, reason);
		return;
	}
	if (Global.test.timeout > 0.0 && m_time >= Global.test.timeout)
	{
		finish(test_result::timeout, std::format("no result after {:.0f} s", Global.test.timeout));
		return;
	}
}

void test_manager::signal_passed_at_danger(std::string_view const Vehicle, std::string_view const Signal)
{
	auto const reason { std::format("{} passed signal {} at danger", Vehicle, Signal) };
	if (false == Global.test.enabled)
	{
		WriteLog(reason, logtype::generic);
		return;
	}
	if (false == Global.test.fail_if_stop_overran)
	{
		WriteLog(std::format("test: {} at {:.1f} s, allowed by test.fail_if_stop_overran", reason, m_time));
		return;
	}
	finish(test_result::signal_passed_at_danger, reason);
}

void test_manager::finish(test_result const Result, std::string_view const Reason)
{
	if (conclude(Result, Reason))
		Application.queue_quit(true);
}

bool test_manager::conclude(test_result const Result, std::string_view const Reason)
{
	if (m_finished || false == Global.test.enabled)
		return false;
	m_finished = true;
	m_result = Result;

	auto const line { std::format("test: {} (exit code {}) after {:.1f} s of simulation time: {}", to_string(Result), static_cast<int>(Result), m_time, Reason) };
	if (Result == test_result::pass)
		WriteLog(line);
	else
		ErrorLog(line);
	// the process has no console of its own; whoever started it with a redirected output gets the verdict there too
	std::fprintf(stdout, "%s\n", line.c_str());
	std::fflush(stdout);
	return true;
}

} // namespace simulation
