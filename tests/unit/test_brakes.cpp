#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
#include "consist.h"

using test::definition;
using test::vehicle;

// the brake pipe of a lone wagon is held at a pressure the way a coupled locomotive would hold it.
// pressures are in bar, the brake pipe runs at 5.0, a 1.5 bar reduction is a full service application.

TEST_CASE("brakes: KE wagon applies up to its tare limit on a service application and releases again") {
	vehicle wagon{definition::load("wagon")}; // empty wagon, TareMaxBP=2.3

	wagon.hold_brake_pipe(5.0);
	wagon.run(60.0);
	CHECK(wagon->BrakePress < 0.1);
	CHECK(wagon->Hamulec->GetBRP() > 4.5); // auxiliary reservoir charged

	wagon.hold_brake_pipe(3.5);
	wagon.run(10.0);
	CHECK(wagon->BrakePress == doctest::Approx(2.3).epsilon(0.05));

	wagon.hold_brake_pipe(5.0);
	wagon.run(60.0);
	CHECK(wagon->BrakePress < 0.1);
}

TEST_CASE("brakes: a deeper brake pipe reduction brakes harder") {
	auto applied = [](double Pipe) {
		vehicle wagon{definition::load("wagon").set("Brake", "TareMaxBP", 3.8)}; // no tare limit in the way
		wagon.hold_brake_pipe(5.0);
		wagon.run(60.0);
		wagon.hold_brake_pipe(Pipe);
		wagon.run(20.0);
		return wagon->BrakePress;
	};
	auto const light{applied(4.6)};
	auto const medium{applied(4.2)};
	auto const full{applied(3.5)};
	CHECK(light > 0.1);
	CHECK(light < medium);
	CHECK(medium < full);
	CHECK(full <= 3.8 + 0.1); // MaxBP
}

// every distributor valve the engine implements goes through the same cycle; the ESt family needs the size
// of its reservoirs, as in the real .fiz files using it
TEST_CASE("brakes: every implemented distributor valve charges, applies and releases") {
	for (auto const *valve : {"W", "K", "KE", "ESt", "ESt3", "NESt3", "ESt4", "ESt3AL2", "LSt", "EStED", "EP1", "EP2", "CV1"}) {
		CAPTURE(valve);
		vehicle wagon{definition::load("wagon").set("Brake", "BrakeValve", valve).set("Brake", "Size", 16)};
		wagon.hold_brake_pipe(5.0);
		wagon.run(90.0);
		CHECK(wagon->BrakePress < 0.1);
		wagon.hold_brake_pipe(3.5);
		wagon.run(20.0);
		CHECK(wagon->BrakePress > 1.0);
		CHECK(wagon->BrakePress <= 3.8 + 0.1); // MaxBP
		wagon.hold_brake_pipe(5.0);
		wagon.run(90.0);
		CHECK(wagon->BrakePress < 0.1);
	}
}

TEST_CASE("brakes: service braking stops a wagon running at 60 km/h and keeps it stopped") {
	vehicle wagon{definition::load("wagon"), 60.0};
	wagon.hold_brake_pipe(5.0);
	wagon.run(10.0);
	REQUIRE(wagon.speed() > 50.0); // still running before the braking

	wagon.hold_brake_pipe(3.5);
	wagon.run(30.0);
	CHECK(std::abs(wagon.speed()) < 0.1);
	wagon.run(30.0);
	CHECK(std::abs(wagon.speed()) < 0.1);
}

TEST_CASE("brakes: a released wagon rolls on") {
	vehicle wagon{definition::load("wagon"), 60.0};
	wagon.hold_brake_pipe(5.0);
	wagon.run(30.0);
	CHECK(wagon.speed() > 50.0);
	CHECK(wagon.speed() < 60.0); // running resistance slows it down a little
}

// the example in README.md
TEST_CASE("brakes: service braking stops a heavier wagon too") {
	// the default wagon, with the keys the test is about changed
	test::vehicle wagon{test::definition::load("wagon").set("Param", "M", 60000), 40.0 /* km/h */};
	wagon.hold_brake_pipe(3.5); // bar, as a coupled locomotive would hold it
	wagon.run(40.0 /* s */, test::track{0.15 /* friction */, 0.0 /* gradient */});
	CHECK(std::abs(wagon.speed()) < 0.1);
}

// eu07/maszyna#87: the ESt4d of the EN57's EP2 valve charged its auxiliary reservoir from the main reservoir pipe
// alone, so a dead unit hauled by a locomotive coupled with the hook and the brake hose ("3") never charged it,
// its control reservoir stopped half way, and it didn't brake. Like every ESt, it charges from the brake pipe too
TEST_CASE("brakes: a dead EN57 hauled with the brake hose alone charges from the brake pipe and brakes") {
	test::consist train;
	auto &locomotive{train.add(definition::fiz("sm42_v1/6d1"))};
	train.add(definition::fiz("en57_v1/5bgb"), 0, coupling::coupler | coupling::brakehose);
	train.add(definition::fiz("en57_v1/6ba_840"));
	train.add(definition::fiz("en57_v1/5bgaiii_led_spot"));
	// a unit standing dead: no air anywhere, nothing in its main reservoir pipe
	for (std::size_t i = 1; i < train.size(); ++i) {
		auto &car{train[i].mover()};
		REQUIRE(car.BrakeValve == TBrakeValve::EP2);
		car.Hamulec->ForceEmptiness();
		car.Pipe->CreatePress(0.0);
		car.Pipe->Act();
		car.Pipe2->CreatePress(0.0);
		car.Pipe2->Act();
		car.CompressedVolume = 0.0;
	}
	locomotive.hold_brake_pipe(5.0);
	train.run(900.0);
	for (std::size_t i = 1; i < train.size(); ++i) {
		CAPTURE(i);
		auto &car{train[i].mover()};
		REQUIRE(car.ScndPipePress < 0.1); // no main reservoir pipe
		CHECK(car.Hamulec->GetBRP() > 4.8);
		CHECK(car.Hamulec->GetCRP() > 4.8);
		CHECK(car.BrakePress < 0.1);
	}
	// a full service application brakes the unit, and it releases again
	locomotive.hold_brake_pipe(3.5);
	train.run(20.0);
	for (std::size_t i = 1; i < train.size(); ++i) {
		CAPTURE(i);
		CHECK(train[i]->BrakePress > 2.0);
	}
	locomotive.hold_brake_pipe(5.0);
	train.run(60.0);
	for (std::size_t i = 1; i < train.size(); ++i) {
		CAPTURE(i);
		CHECK(train[i]->BrakePress < 0.1);
	}
}
