#include "stdafx.h"
#include "doctest.h"

#include "vehicle.h"
#include "drive.h"

#include "vehicle/smoke.h"

// the smoke of a vehicle's engine: how many particles the smoke source of the vehicle's model spawns in an update

namespace {

// a source of 20 particles per second, with room for 100
smoke::spawn_rate const source{0.f, 20.f, 1.f, 100.f};
double const update{0.1}; // s

// a diesel at work, its engine in the state of one of the cases below
test::vehicle diesel() {
	test::vehicle vehicle{test::definition::load("wagon")};
	vehicle->MainCtrlPosNo = 1;
	vehicle->DElist[1].RPM = 1500.0;
	vehicle->enrot = 300.0;
	vehicle->EnginePower = 400.0;
	return vehicle;
}

} // namespace

TEST_CASE("smoke: a vehicle taken off the tracks puffs no more (#90)") {
	auto vehicle{diesel()};
	SUBCASE("engine starting") { vehicle->dizel_spinup = true; }
	SUBCASE("engine idling") { vehicle->DirAbsolute = 0; }
	SUBCASE("engine under load") {
		vehicle->DirAbsolute = 1;
		vehicle->Im = 400.0;
	}
	// on the tracks the engine puffs
	REQUIRE(smoke::vehicle_spawn_count(vehicle.mover(), true, source, update) >= 1.f);

	// gone through a portal, the pending particles are dropped too
	auto pending{source};
	pending.pending = 5.f;
	CHECK(smoke::vehicle_spawn_count(vehicle.mover(), false, pending, update) == 0.f);
}

TEST_CASE("smoke: a road vehicle back on the road from the pool puffs again") {
	auto vehicle{diesel()};
	// waiting in the pool, the count stays at zero update after update
	auto rate{source};
	for (auto i{0}; i < 10; ++i) {
		rate.pending = smoke::vehicle_spawn_count(vehicle.mover(), false, rate, update);
	}
	REQUIRE(rate.pending == 0.f);

	CHECK(smoke::vehicle_spawn_count(vehicle.mover(), true, rate, update) >= 1.f);
}

TEST_CASE("smoke: the engine adds to the steady rate of the source") {
	auto vehicle{diesel()};
	vehicle->DirAbsolute = 0;
	auto const steady{source.rate * update}; // the rate of a source of the scenery

	vehicle->EnginePower = 0.0;
	CHECK(smoke::vehicle_spawn_count(vehicle.mover(), true, source, update) == doctest::Approx(steady));

	vehicle->EnginePower = 400.0;
	auto const working{smoke::vehicle_spawn_count(vehicle.mover(), true, source, update)};
	CHECK(working > steady);

	// the faster the starting engine turns, the more it puffs
	vehicle->dizel_spinup = true;
	vehicle->enrot = 100.0;
	auto const slow{smoke::vehicle_spawn_count(vehicle.mover(), true, source, update)};
	vehicle->enrot = 400.0;
	CHECK(smoke::vehicle_spawn_count(vehicle.mover(), true, source, update) > slow);
}

TEST_CASE("smoke: the pending particles never outgrow the room of the source") {
	auto vehicle{diesel()};
	vehicle->dizel_spinup = true;
	vehicle->enrot = 100000.0;
	auto rate{source};
	rate.pending = 99.f;

	CHECK(smoke::vehicle_spawn_count(vehicle.mover(), true, rate, 10.0) == rate.limit);
}

TEST_CASE("smoke: an engine the starter turns lets out no smoke until it fires (#523)") {
	auto vehicle{diesel()};
	vehicle->dizel_spinup = true;
	auto pending{source};
	pending.pending = 5.f;

	vehicle->dizel_FiringCountdown = 0.5;
	CHECK(smoke::vehicle_spawn_count(vehicle.mover(), true, pending, update) == 0.f);

	vehicle->dizel_FiringCountdown = 0.0;
	CHECK(smoke::vehicle_spawn_count(vehicle.mover(), true, pending, update) >= 1.f);
}

namespace {

// the locomotive's engine started the way the AI starts it, a second at a time, until it runs (Mains)
bool start_engine(test::driven const &Driven, test::track const &Track) {
	for (int second = 0; second < 120 && false == Driven.controlling.Mains; ++second) {
		test::start_up(Driven, Track, 1.0);
	}
	return Driven.controlling.Mains;
}

} // namespace

TEST_CASE("smoke: a real diesel smokes only once its engine has fired, SmokeDelay after the starter began (#523)") {
	// a diesel with a mechanical transmission and a diesel-electric one
	for (auto const *path : {"sm42_v1/6d1", "st44_v2/m62-1151"}) {
		CAPTURE(path);
		test::track const track{};
		{
			// SmokeDelay set in the .fiz of the locomotive
			test::vehicle locomotive{test::definition{test::definition::fiz(path)}.set("Engine", "SmokeDelay", 2)};
			REQUIRE(locomotive->dizel_SmokeDelay == doctest::Approx(2.0));
			auto const driven{test::driving(locomotive)};
			REQUIRE(start_engine(driven, track));
			// the engine started less than a second ago: the starter still turns it
			CHECK(locomotive->dizel_FiringCountdown > 0.5);
			CHECK(smoke::vehicle_spawn_count(locomotive.mover(), true, source, update) == 0.f);
			// it has fired since
			test::drive(driven, track, 3.0);
			CHECK(locomotive->Mains);
			CHECK(locomotive->dizel_FiringCountdown == 0.0);
			CHECK(smoke::vehicle_spawn_count(locomotive.mover(), true, source, update) >= 1.f);
		}
		{
			// a longer delay from the .fiz
			test::vehicle locomotive{test::definition{test::definition::fiz(path)}.set("Engine", "SmokeDelay", 5)};
			auto const driven{test::driving(locomotive)};
			REQUIRE(start_engine(driven, track));
			test::drive(driven, track, 3.0);
			CHECK(smoke::vehicle_spawn_count(locomotive.mover(), true, source, update) == 0.f);
			test::drive(driven, track, 3.0);
			CHECK(smoke::vehicle_spawn_count(locomotive.mover(), true, source, update) >= 1.f);
		}
	}
}

TEST_CASE("smoke: without SmokeDelay in its .fiz a diesel smokes as soon as the starter turns, as before (#523)") {
	for (auto const *path : {"sm42_v1/6d1", "st44_v2/m62-1151"}) {
		CAPTURE(path);
		test::track const track{};
		// a locomotive which has the delay, loaded first, doesn't hand it on to the next one: the value is the vehicle's own
		test::vehicle delayed{test::definition{test::definition::fiz(path)}.set("Engine", "SmokeDelay", 3)};
		REQUIRE(delayed->dizel_SmokeDelay == doctest::Approx(3.0));
		test::vehicle locomotive{test::definition::fiz(path)};
		REQUIRE(locomotive->dizel_SmokeDelay == 0.0);
		CHECK(delayed->dizel_SmokeDelay == doctest::Approx(3.0));
		auto const driven{test::driving(locomotive)};
		REQUIRE(start_engine(driven, track));
		CHECK(locomotive->dizel_FiringCountdown == 0.0);
		CHECK(smoke::vehicle_spawn_count(locomotive.mover(), true, source, update) >= 1.f);
	}
}
