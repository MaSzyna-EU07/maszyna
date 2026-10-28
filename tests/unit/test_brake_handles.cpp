#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
#include "golden.h"
#include "real_vehicles.h"
#include "drive.h"

#include "McZapkie/hamulce.h"

using test::definition;
using test::vehicle;

// characterisation of the driver's brake handles and the local brake handles of the real locomotives and
// units: every position of the handle, with the main reservoir kept full, followed in golden traces
// (data/golden/brake handles/). the vehicle runs alone, so the handle drives only its own brake pipe

namespace {

std::vector<test::channel> handle_channels() {
	using namespace test::channels;
	return brakes() + handles() + air();
}

// the vehicles with a driver's brake handle
bool has_handle(vehicle &Vehicle) {
	return Vehicle->BrakeCtrlPosNo > 1;
}

void hold(vehicle &Vehicle, test::trace &Trace, std::string const &Event, double Position, double Seconds) {
	Trace.mark(Event);
	Vehicle->BrakeLevelSet(Position);
	Vehicle.run(Seconds);
}

// every position of the driver's handle, recorded in the trace
void every_position(vehicle &Vehicle, test::trace &Trace) {
	Vehicle.hold_main_reservoir(std::max(Vehicle->MaxCompressor, 8.0));
	auto &handle{*Vehicle->Handle};
	Vehicle.record(Trace);

	hold(Vehicle, Trace, "running", handle.GetPos(bh_RP), 90.0);
	// every position from the lowest to the highest, emergency included, then back to running
	auto const lowest{static_cast<int>(std::ceil(handle.GetPos(bh_MIN)))};
	auto const highest{static_cast<int>(std::floor(handle.GetPos(bh_MAX)))};
	for (auto position = lowest; position <= highest; ++position) {
		hold(Vehicle, Trace, "position " + std::to_string(position), position, 15.0);
		hold(Vehicle, Trace, "running", handle.GetPos(bh_RP), 30.0);
	}
	// filling stroke, neutral and the positions of the ep brake, where the handle has them
	hold(Vehicle, Trace, "filling stroke", handle.GetPos(bh_FS), 10.0);
	hold(Vehicle, Trace, "neutral", handle.GetPos(bh_NP), 20.0);
	hold(Vehicle, Trace, "first step", handle.GetPos(bh_MB), 20.0);
	hold(Vehicle, Trace, "full service", handle.GetPos(bh_FB), 20.0);
	hold(Vehicle, Trace, "ep release", handle.GetPos(bh_EPR), 10.0);
	hold(Vehicle, Trace, "ep hold", handle.GetPos(bh_EPN), 10.0);
	hold(Vehicle, Trace, "ep brake", handle.GetPos(bh_EPB), 10.0);
	hold(Vehicle, Trace, "running", handle.GetPos(bh_RP), 60.0);
}

} // namespace

TEST_CASE("brake handles: every position of the driver's handle of every real vehicle") {
	for (auto const &path : test::real_vehicles()) {
		CAPTURE(path);
		test::reseed_random();
		vehicle real{definition::fiz(path)};
		if (false == has_handle(real)) {
			continue;
		}
		test::trace trace{"brake handles/" + path, handle_channels(), 100};
		every_position(real, trace);
		trace.check();
	}
}

TEST_CASE("brake handles: the handles the game knows but none of its vehicles has, fitted to an EU07") {
	// test is a handle for trying things out, D2 and Westinghouse fall back to the plain handle of the base class
	for (auto const *name : {"test", "D2", "Westinghouse", "MHZ_EN57", "St113", "no such handle"}) {
		CAPTURE(name);
		test::reseed_random();
		vehicle real{definition::fiz("4e_v1/4e").set("Cntrl", "BrakeHandle", name)};
		test::trace trace{std::string{"brake handles/fitted/"} + name, handle_channels(), 100};
		every_position(real, trace);
		trace.check();
	}
}

TEST_CASE("brake handles: the local brake handle of every real vehicle") {
	for (auto const &path : test::real_vehicles()) {
		CAPTURE(path);
		test::reseed_random();
		vehicle real{definition::fiz(path)};
		if (false == has_handle(real)) {
			continue;
		}
		real.hold_main_reservoir(std::max(real->MaxCompressor, 8.0));
		test::trace trace{"local brake/" + path, handle_channels(), 100};
		real.record(trace);
		real->BrakeLevelSet(real->Handle->GetPos(bh_RP));
		trace.mark("charge");
		real.run(60.0);
		for (auto const position : {0.25, 0.5, 1.0, 0.5, 0.0}) {
			trace.mark("local " + std::to_string(position));
			real->LocalBrakePosA = position;
			real.run(15.0);
		}
		trace.check();
	}
}

TEST_CASE("brake handles: the overload and the high pressure buttons of the MHZ handles") {
	// the overload button charges the pipe over its running pressure (TP), which then decays; the high pressure
	// button gives the filling stroke its overpressure, where the handle has one only with the button pressed.
	// from the cab, with the battery on and the driver attending to the vigilance: the electro-pneumatic handles
	// work only so
	// the MHZ_EN57, whose unit doesn't charge its pipe alone, fitted to an EU07
	for (auto const &[name, definition] : std::vector<std::pair<std::string, test::definition>>{
	         {"eu47", definition::fiz("e186_v2/eu47")},
	         {"e6act", definition::fiz("e6act_v2/e6acta")},
	         {"eu07 with MHZ_EN57", definition::fiz("4e_v1/4e").set("Cntrl", "BrakeHandle", "MHZ_EN57")}}) {
		CAPTURE(name);
		test::reseed_random();
		vehicle real{definition, 0.0, "test", 1};
		real->BatterySwitch(true);
		real->CabActivisation();
		real.hold_main_reservoir(std::max(real->MaxCompressor, 8.0));
		test::trace trace{"brake handles/buttons/" + name, handle_channels(), 100};
		real.record(trace);
		auto &handle{*real->Handle};
		auto const at{[&](std::string const &Event, double Position, double Seconds) {
			trace.mark(Event);
			real->BrakeLevelSet(Position);
			test::drive(test::driving(real), {}, Seconds);
		}};
		at("running", handle.GetPos(bh_RP), 60.0);
		auto const running{real->PipePress};
		handle.SetUniversalFlag(TUniversalBrake::ub_Overload);
		at("overload button", handle.GetPos(bh_RP), 30.0);
		handle.SetUniversalFlag(0);
		auto const overloaded{real->PipePress};
		at("overload decaying", handle.GetPos(bh_RP), 240.0);
		auto const decayed{real->PipePress};
		handle.SetUniversalFlag(TUniversalBrake::ub_HighPressure);
		at("high pressure button, filling stroke", handle.GetPos(bh_FS), 10.0);
		auto const filled{real->PipePress};
		handle.SetUniversalFlag(0);
		at("running", handle.GetPos(bh_RP), 120.0);
		CHECK(running == doctest::Approx(5.0).epsilon(0.01));
		CHECK(overloaded > running + 0.3);
		CHECK(decayed < overloaded);
		CHECK(filled >= decayed - 0.05);
		trace.check();
	}
}
