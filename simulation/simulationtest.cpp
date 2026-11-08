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

namespace simulation {

test_manager &test()
{
	static test_manager instance;
	return instance;
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
	m_progress = test_progress {};
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

	test_frame frame { Deltatime, std::nullopt, {},
	                   std::ranges::none_of(simulation::Vehicles.sequence(), [](TDynamicObject const *Vehicle) { return std::abs(Vehicle->GetVelocity()) > 0.01; }) };
	if (auto const *cell = simulation::Memory.find(std::string(result_cell)))
	{
		frame.reported = cell->Value1();
		frame.reason = cell->Text();
	}

	auto const requestedbefore { m_progress.passrequested };
	auto const verdict { evaluate_frame(m_progress, frame, test_limits { Global.test.timeout, Global.test.standstill, Global.test.pass_on_standstill, Global.test.no_movement_before_pass }) };
	if (verdict.has_value())
	{
		finish(verdict->result, verdict->reason);
		return;
	}
	if (m_progress.passrequested && false == requestedbefore)
	{
		WriteLog(std::format("test: pass requested at {:.1f} s{}, waiting for every vehicle to stop", m_progress.time,
		                     frame.reason.empty() ? std::string() : std::format(" ({})", frame.reason)));
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
		WriteLog(std::format("test: {} at {:.1f} s, allowed by test.fail_if_stop_overran", reason, m_progress.time));
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

	auto const line { std::format("test: {} (exit code {}) after {:.1f} s of simulation time: {}", to_string(Result), static_cast<int>(Result), m_progress.time, Reason) };
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
