#include "stdafx.h"
#include "doctest.h"
#include "consist.h"

#include "vehicle/enginestart.h"

// starting the engines of the vehicles in multiple from the cab (eu07/maszyna#80): the cab's own switch (main_*) works
// all of them unless Engine:JointEngineStart=No keeps it to its own vehicle, and the switches mainb_* and mainc_* start
// the second and the third vehicle alone. the cab (TTrain) passes the vehicle's MainSwitchRange() to its MainSwitch(),
// finds the vehicles with engine(), and runs remote_switch for the switches of the others

using test::consist;
using test::definition;

namespace {

// the SN61 railcar, whose cab has a set of switches for each of up to three railcars in multiple
definition sn61(bool const Joint = true) {
	auto railcar{definition::fiz("sn61_v2/sn61")};
	if (false == Joint) {
		railcar.set("Engine", "JointEngineStart", "No");
	}
	return railcar;
}

// the battery, the cab and the pumps of every railcar of the train switched on from the first one's cab, the oil
// pressure built up, and the controller at the first position giving the engines fuel, which a mechanical diesel
// starts at: everything but the engines ready to start
void prepare(consist &Train) {
	auto &cab{Train[0].mover()};
	cab.BatterySwitch(true);
	cab.CabActivisation();
	cab.OilPumpSwitch(true);
	cab.FuelPumpSwitch(true);
	while (cab.RList[cab.MainCtrlPos].R == 0.0 && cab.IncMainCtrl(1)) {
		// up to the first position with fuel
	}
	Train.run(20.0);
}

bool running(TMoverParameters const &Vehicle) {
	return Vehicle.Mains && Vehicle.enrot > 0.5 * Vehicle.dizel_nmin;
}

// the time step of the train in the loops below
auto const dt{0.01};

// the count of the steps of dt making up Seconds
int steps(double const Seconds) {
	return static_cast<int>(std::lround(Seconds / dt));
}

// the switch of Engine held closed for Seconds, then let go, in steps of the train
void hold(consist &Train, engine_start::remote_switch &Switch, TMoverParameters *Engine, double const Seconds, bool const Closesonrelease = false) {
	for (auto step = 0; step < steps(Seconds); ++step) {
		Switch.update(Engine, true, false, dt, Closesonrelease);
		Train.step({}, dt);
	}
	Switch.update(Engine, false, false, dt, Closesonrelease);
}

} // namespace

TEST_CASE("engine start: the joint start is the default, and JointEngineStart=No keeps the cab's switch to its vehicle") {
	consist train;
	auto const &joint{train.add(sn61(), 1).mover()};
	auto const &separate{train.add(sn61(false)).mover()};
	CHECK(joint.JointEngineStart);
	CHECK_FALSE(separate.JointEngineStart);
	CHECK(joint.MainSwitchRange() == range_t::consist);
	CHECK(separate.MainSwitchRange() == range_t::local);
}

TEST_CASE("engine start: the vehicles in multiple, counted from the cab's own one, those behind the cab first") {
	consist train;
	auto &first{train.add(sn61(), 1).mover()};
	auto &second{train.add(sn61()).mover()};
	auto &third{train.add(sn61()).mover()};
	CHECK(engine_start::engine(first, first, 1) == &first);
	CHECK(engine_start::engine(first, first, 2) == &second);
	CHECK(engine_start::engine(first, first, 3) == &third);
	CHECK(engine_start::engine(first, first, 4) == nullptr);
	CHECK(engine_start::engine(first, first, 0) == nullptr);
	// from the rear cab of the first railcar the others are in front of the cab, in the same order
	first.CabOccupied = -1;
	CHECK(engine_start::engine(first, first, 2) == &second);
	CHECK(engine_start::engine(first, first, 3) == &third);
	// from the middle one, the one behind its cab comes first
	second.CabOccupied = 1;
	CHECK(engine_start::engine(second, second, 2) == &third);
	CHECK(engine_start::engine(second, second, 3) == &first);
	second.CabOccupied = -1;
	CHECK(engine_start::engine(second, second, 2) == &first);
	CHECK(engine_start::engine(second, second, 3) == &third);
}

TEST_CASE("engine start: only the vehicles with an engine and under control through the cable count") {
	consist train;
	auto &first{train.add(sn61(), 1).mover()};
	// a vehicle without an engine between the railcars is passed over
	auto &trailer{train.add(definition::fiz("ed72_v1/5bs-ra")).mover()};
	auto &second{train.add(sn61()).mover()};
	REQUIRE(trailer.Power < 0.01);
	CHECK(engine_start::engine(first, first, 2) == &second);
	// the vehicle with the cab doesn't drive itself: its engine is in the vehicle it controls, which isn't counted twice
	trailer.CabOccupied = 1;
	CHECK(engine_start::engine(trailer, first, 1) == &first);
	CHECK(engine_start::engine(trailer, first, 2) == &second);
	CHECK(engine_start::engine(trailer, first, 3) == nullptr);

	// coupled without the control cable, the other railcar isn't under the cab's control
	consist apart;
	auto &alone{apart.add(sn61(), 1).mover()};
	apart.add(sn61(), 0, coupling::coupler | coupling::brakehose);
	CHECK(engine_start::engine(alone, alone, 2) == nullptr);
}

TEST_CASE("engine start: by default the cab's switch starts and stops the engines of all the vehicles in multiple") {
	consist train;
	auto &first{train.add(sn61(), 1).mover()};
	auto const &second{train.add(sn61()).mover()};
	prepare(train);
	REQUIRE_FALSE(first.Mains);
	REQUIRE_FALSE(second.Mains);
	first.MainSwitch(true, first.MainSwitchRange());
	train.run(15.0);
	CHECK(running(first));
	CHECK(running(second));
	first.MainSwitch(false, first.MainSwitchRange());
	train.run(5.0);
	CHECK_FALSE(first.Mains);
	CHECK_FALSE(second.Mains);
}

TEST_CASE("engine start: with JointEngineStart=No each engine has its own switch") {
	consist train;
	auto &first{train.add(sn61(false), 1).mover()};
	auto &second{train.add(sn61(false)).mover()};
	auto &third{train.add(sn61(false)).mover()};
	prepare(train);
	// the cab's own switch starts its own engine only
	first.MainSwitch(true, first.MainSwitchRange());
	train.run(15.0);
	REQUIRE(running(first));
	CHECK_FALSE(second.Mains);
	CHECK_FALSE(third.Mains);

	// the second one's switch, mainb_*, starts the second engine
	engine_start::remote_switch switchb;
	engine_start::remote_switch switchc;
	auto *engineb{engine_start::engine(first, first, 2)};
	auto *enginec{engine_start::engine(first, first, 3)};
	REQUIRE(engineb == &second);
	REQUIRE(enginec == &third);
	hold(train, switchb, engineb, 2.0); // past the SN61's delay of 1.5 s
	train.run(15.0);
	CHECK(running(second));
	CHECK_FALSE(third.Mains);
	// and the third one's, mainc_*, the third
	hold(train, switchc, enginec, 2.0);
	train.run(15.0);
	CHECK(running(third));

	// the switches open their own vehicles' circuits only
	CHECK(engine_start::remote_switch::open(second));
	train.run(5.0);
	CHECK_FALSE(second.Mains);
	CHECK(running(first));
	CHECK(running(third));
	first.MainSwitch(false, first.MainSwitchRange());
	train.run(5.0);
	CHECK_FALSE(first.Mains);
	CHECK(running(third));
	CHECK_FALSE(engine_start::remote_switch::open(second)); // already stopped
}

TEST_CASE("engine start: the switch of another vehicle is held for the vehicle's delay, with the vehicle ready") {
	consist train;
	train.add(sn61(false).set("Cntrl", "IniCDelay", "2.0"), 1);
	auto &second{train.add(sn61(false).set("Cntrl", "IniCDelay", "2.0")).mover()};
	REQUIRE(second.InitialCtrlDelay == doctest::Approx(2.0));
	engine_start::remote_switch switchb;

	// without the battery the switch does nothing, however long it's held
	hold(train, switchb, &second, 5.0);
	train.run(5.0);
	CHECK_FALSE(second.Mains);
	CHECK_FALSE(second.dizel_startup);

	prepare(train);
	// let go too early
	hold(train, switchb, &second, 1.0);
	CHECK_FALSE(engine_start::remote_switch::is_running(second));
	// the switch opening the circuit held along, the closing one does nothing
	for (auto step = 0; step < steps(3.0); ++step) {
		switchb.update(&second, true, true, dt, false);
		train.step({}, dt);
	}
	CHECK_FALSE(engine_start::remote_switch::is_running(second));
	// held long enough
	hold(train, switchb, &second, 2.5);
	CHECK(engine_start::remote_switch::is_running(second));
	train.run(15.0);
	CHECK(running(second));
}

TEST_CASE("engine start: a switch closing on release, as an electric locomotive's impulse switch, waits to be let go") {
	consist train;
	train.add(sn61(false), 1);
	auto &second{train.add(sn61(false).set("Cntrl", "IniCDelay", "1.0")).mover()};
	prepare(train);
	engine_start::remote_switch switchb;
	// held past the delay, the circuit stays open while the switch is held
	for (auto step = 0; step < steps(2.0); ++step) {
		switchb.update(&second, true, false, dt, true);
		train.step({}, dt);
	}
	CHECK_FALSE(engine_start::remote_switch::is_running(second));
	// and closes when let go
	switchb.update(&second, false, false, dt, true);
	CHECK(engine_start::remote_switch::is_running(second));
	train.run(15.0);
	REQUIRE(running(second));
	// let go before the delay, it doesn't
	CHECK(engine_start::remote_switch::open(second));
	train.run(5.0);
	REQUIRE_FALSE(engine_start::remote_switch::is_running(second));
	hold(train, switchb, &second, 0.5, true);
	CHECK_FALSE(engine_start::remote_switch::is_running(second));
}

TEST_CASE("engine start: an engine stopping on its own stops the others only when they started together") {
	for (auto const joint : {true, false}) {
		CAPTURE(joint);
		consist train;
		auto &first{train.add(sn61(joint), 1).mover()};
		auto const &second{train.add(sn61(joint)).mover()};
		prepare(train);
		engine_start::remote_switch switchb;
		first.MainSwitch(true, first.MainSwitchRange());
		hold(train, switchb, engine_start::engine(first, first, 2), 2.0);
		train.run(15.0);
		REQUIRE(running(first));
		REQUIRE(running(second));
		// the first one's fuel cut off, its engine stops
		first.FuelPump.is_disabled = true;
		train.run(15.0);
		CHECK_FALSE(first.Mains);
		CHECK(first.enrot == 0.0);
		// started on their own, the others keep running
		CHECK(running(second) == (false == joint));
	}
}

// a stopped engine stays stopped on its own: only an engine stopping while it ran tells the others, as a joint start's
// engines stop together; one stopped from another vehicle's switch, or not started at all, doesn't stop those running,
// whatever the flags of the vehicles in a mixed train
TEST_CASE("engine start: an engine stopped from its own switch doesn't stop the others when it has spun down") {
	consist train;
	// the middle railcar keeps the joint start, as a .fiz without the key does. what a vehicle without the cab relays
	// goes on away from the cab, so it's the third one that would stop
	auto &first{train.add(sn61(false), 1).mover()};
	auto &second{train.add(sn61()).mover()};
	auto &third{train.add(sn61(false)).mover()};
	prepare(train);
	engine_start::remote_switch switchb;
	engine_start::remote_switch switchc;
	first.MainSwitch(true, first.MainSwitchRange());
	hold(train, switchb, engine_start::engine(first, first, 2), 2.0);
	hold(train, switchc, engine_start::engine(first, first, 3), 2.0);
	train.run(15.0);
	REQUIRE(running(first));
	REQUIRE(running(second));
	REQUIRE(running(third));
	// mainb_off
	CHECK(engine_start::remote_switch::open(second));
	train.run(30.0);
	CHECK_FALSE(second.Mains);
	CHECK(second.enrot == 0.0);
	CHECK(running(first));
	CHECK(running(third));
}

TEST_CASE("engine start: the stopped engine of the cab's vehicle doesn't stop the one started from its switch") {
	consist train;
	// the cab's railcar keeps the joint start, its engine stopped
	auto &first{train.add(sn61(), 1).mover()};
	auto &second{train.add(sn61(false)).mover()};
	prepare(train);
	REQUIRE_FALSE(first.Mains);
	// mainb_on
	engine_start::remote_switch switchb;
	hold(train, switchb, engine_start::engine(first, first, 2), 2.0);
	train.run(15.0);
	CHECK(running(second));
	CHECK_FALSE(first.Mains);
	CHECK(first.enrot == 0.0);
}
