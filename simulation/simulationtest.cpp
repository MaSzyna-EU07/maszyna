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
#include "utilities/utilities.h"
#include "vehicle/DynObj.h"
#include "world/Event.h"
#include "world/MemCell.h"
#ifdef _WIN32
#include "Console.h"
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <memory>

namespace {

bool is_moving(TDynamicObject const *Vehicle)
{
	return std::abs(Vehicle->GetVelocity()) > 0.01;
}

// [s] of real time since the start of the simulator
double elapsed_wall_time()
{
	return std::chrono::duration<double>(std::chrono::steady_clock::now() - Global.startTimestamp).count();
}

} // namespace

namespace simulation {

test_manager &test()
{
	static test_manager instance;
	return instance;
}

void apply_test_mode_settings()
{
	// nobody watches nor listens: no player vehicle, nothing drawn, no sound, no window in the way
	Global.editor_session = false;
	Global.local_start_vehicle = "ghostview";
	Global.GfxRenderer = "null";
	Global.NvRenderer = false;
	Global.bSoundEnabled = false;
	Global.python_enabled = false;
	Global.bInactivePause = false;
	Global.bFullScreen = false;
	Global.fullscreen_windowed = false;
	Global.VSync = false;
	Global.iMultisampling = 0;
	Global.gfx_usegles = false;
	Global.minframetime = std::chrono::duration<float>(0.0f);
	Global.extra_viewports.clear();
	Global.headtrack_conf.joy.clear();
	// a test runs on its own: no network, and no input from outside (eu07/maszyna#541)
	Global.network_servers.clear();
	Global.network_client.reset();
	Global.InputGamepad = false;
	Global.InputMouse = false;
	Global.uart_conf.enable = false;
	Global.motiontelemetry_conf.enable = false;
#ifdef WITH_ZMQ
	Global.zmq_address.clear();
#endif
	// nor anything to the outside: no keyboard leds, LPT or PoKeys desk (feedbackmode), no messages to a
	// multiplayer server (multiplayer, which also holds some events back for it), no program run at the end
	Global.iFeedbackMode = 0;
#ifdef _WIN32
	Console::ModeSet(Global.iFeedbackMode, Global.iFeedbackPort);
#endif
	Global.iMultiplayer = 0;
	Global.exec_on_exit.clear();
	// and the same way every time, unless given another seed (-seed, or randomseed in the settings)
	if (!Global.random_seed)
		Global.random_seed = seed_of("test");
	// the real time limit, which the scenery may set too
	test().watch_wall_clock(wall_clock_limit(Global.test.wall_timeout, Global.test.timeout));
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
	{
		std::scoped_lock lock { m_lock };
		m_progress = test_progress {};
	}
	if (simulation::Vehicles.sequence().empty())
	{
		// most likely the scenery wasn't found, or it isn't a test scenery
		finish(test_result::error, "the scenery has no vehicles");
		return;
	}
	for (auto const &name : Global.test.vehicles)
	{
		if (simulation::Vehicles.find(name) == nullptr)
		{
			finish(test_result::error, std::format("the scenery has no vehicle {} (test.vehicle)", name));
			return;
		}
	}
	// a pause set by the scenery or the settings would never be lifted, nobody is there to press the key
	Global.iPause = 0;
	auto tested { std::string("every vehicle") };
	if (false == Global.test.vehicles.empty())
	{
		tested = "the consists of";
		for (auto const &name : Global.test.vehicles)
			tested += " " + name;
	}
	auto const walllimit { m_walllimit.load() };
	WriteLog(std::format("test: started, testing {}, timeout {:.0f} s, real time limit {}, standstill {:.0f} s{}{}{}", tested, Global.test.timeout,
	                     walllimit > 0.0 ? std::format("{:.0f} s", walllimit) : std::string("none"), Global.test.standstill,
	                     Global.test.pass_on_standstill ? ", standstill passes" : "",
	                     Global.test.fail_if_stop_overran ? "" : ", signals passed at danger allowed",
	                     Global.test.no_movement_before_pass ? "" : ", pass without waiting for a stop"));
}

void test_manager::update(double const Deltatime)
{
	if (false == m_started || m_finished)
		return;

	test_frame frame { Deltatime, std::nullopt, {}, tested_trains_at_rest() };
	if (auto const *cell = simulation::Memory.find(std::string(result_cell)))
	{
		frame.reported = cell->Value1();
		frame.reason = cell->Text();
	}

	auto const limits { test_limits { Global.test.timeout, Global.test.standstill, Global.test.pass_on_standstill, Global.test.no_movement_before_pass } };
	std::optional<test_verdict> verdict;
	auto passrequested { false };
	{
		std::scoped_lock lock { m_lock };
		auto const requestedbefore { m_progress.passrequested };
		verdict = evaluate_frame(m_progress, frame, limits);
		passrequested = m_progress.passrequested && false == requestedbefore;
	}
	if (verdict.has_value())
	{
		finish(verdict->result, verdict->reason);
		return;
	}
	if (passrequested)
	{
		WriteLog(std::format("test: pass requested at {:.1f} s{}, waiting for the tested trains to stop", m_progress.time,
		                     frame.reason.empty() ? std::string() : std::format(" ({})", frame.reason)));
	}
}

bool test_manager::tested_trains_at_rest() const
{
	if (Global.test.vehicles.empty())
		return std::ranges::none_of(simulation::Vehicles.sequence(), is_moving);
	// the consists as they are now: a train the tested one couples to joins it, one it leaves behind doesn't
	return std::ranges::none_of(Global.test.vehicles, [](std::string const &Name) {
		auto *vehicle { simulation::Vehicles.find(Name) };
		return vehicle != nullptr && vehicle->find_vehicle(coupling::coupler, is_moving) != nullptr;
	});
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
	std::scoped_lock lock { m_lock };
	if (m_finished || false == Global.test.enabled)
		return false;
	m_result = Result;
	m_finished = true;

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

int test_manager::exit_code() const
{
	std::scoped_lock lock { m_lock };
	return static_cast<int>(m_finished ? m_result : test_result::error);
}

void test_manager::watch_wall_clock(double const Limit)
{
	m_walllimit = Limit;
	if (Limit <= 0.0 || m_watchdog.joinable())
		return;
	m_watchdog = std::jthread([this](std::stop_token const &Stop) {
		while (false == Stop.stop_requested())
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			check_wall_clock(elapsed_wall_time());
		}
	});
}

void test_manager::check_wall_clock(double const Elapsed)
{
	auto const limit { m_walllimit.load() };
	if (limit <= 0.0 || Elapsed < limit)
		return;
	// the run hangs, or is too slow: it ends here, whatever the main thread is doing. a run whose test has ended
	// but which doesn't close ends with its result
	if (false == conclude(test_result::timeout, std::format("still running after {:.0f} s of real time (test.wall_timeout)", limit)))
		ErrorLog(std::format("test: the simulator was still running {:.0f} s of real time after the start, ended", limit));
	// what was logged last gets written, unless the log is held up by a hung thread
	StopLogService();
	std::this_thread::sleep_for(std::chrono::milliseconds(250));
	std::_Exit(exit_code());
}

} // namespace simulation
