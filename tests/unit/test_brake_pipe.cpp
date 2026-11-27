#include "stdafx.h"
#include "doctest.h"

#include "physics/mover.h"
#include "physics/brakes.h"
#include "consist.h"
#include "vehicle.h"

// the air exchange through the brake hoses between the vehicles of a train (TMoverParameters::UpdatePipeCouplers()).
// taken in one explicit step of 0.01 s, the hose between two short freight wagons carried twice the air which evens
// out their pipes: the pipes overshot each other in turns, and an ESt3 distributor saw the pipe pressure up to
// 0.28 bar off what was in it, lapped at the first stage of braking and kept the AI's train from leaving
// (eu07/maszyna#368)

namespace {

using test::definition;

// a freight train standing charged: a locomotive and Wagons empty two-axle freight wagons with ESt3, set to G
void freight_train(test::consist &Train, int const Wagons) {
	Train.add(definition::fiz("st44_v2/m62-1151"));
	for (int i = 0; i < Wagons; ++i) {
		auto &wagon{Train.add(definition::fiz("3wl_v2/3wl"))};
		wagon->BrakeDelaySwitch(bdelay_G);
		REQUIRE(wagon->BrakeValve == TBrakeValve::ESt3);
		REQUIRE(wagon->Spg == doctest::Approx(0.792)); // the pipe of a freight wagon
	}
	Train[0].hold_brake_pipe(5.0); // the locomotive's driver's valve in the running position
}

// a passenger train standing charged: a locomotive and Cars passenger cars with ESt4, set to R
void passenger_train(test::consist &Train, int const Cars) {
	Train.add(definition::fiz("st44_v2/m62-1151"));
	for (int i = 0; i < Cars; ++i) {
		auto &car{Train.add(definition::fiz("11xa_v2/93a"))};
		REQUIRE(car->Spg == doctest::Approx(0.507)); // the pipe of a passenger car
	}
	Train[0].hold_brake_pipe(5.0);
}

// the time [s] until the brake pipe of the train's last vehicle is 0.5 bar down, once the locomotive holds its pipe
// at Pressure [bar], the physics stepping by Dt
double application_time(test::consist &Train, double const Pressure, double const Dt) {
	auto &last{Train[Train.size() - 1]};
	auto const start{last->PipePress};
	Train[0].hold_brake_pipe(Pressure);
	auto time{0.0};
	while (last->PipePress > start - 0.5 && time < 60.0) {
		Train.step({}, Dt);
		time += Dt;
	}
	return time;
}

// the same train standing a minute, then braked from the locomotive, all of it in steps of Dt
template <typename Builder>
double application_time(Builder Train, int const Vehicles, double const Pressure, double const Dt) {
	test::consist train;
	Train(train, Vehicles);
	train.run(60.0, {}, Dt);
	return application_time(train, Pressure, Dt);
}

} // namespace

TEST_CASE("brake pipe: the distributors of a standing freight train see the pressure in the pipe, and none of them brakes") {
	test::consist train;
	freight_train(train, 37);
	// standing a while, the locomotive keeping the pipe charged
	train.run(300.0);
	for (std::size_t i = 1; i < train.size(); ++i) {
		CAPTURE(i);
		auto &wagon{train[i].mover()};
		// the distributor's valve chamber follows the pressure the distributor works with: the pipe's own
		CHECK(std::abs(wagon.Hamulec->GetVRP() - wagon.PipePress) < 0.005);
		CHECK(wagon.PipePress > 4.9);
		// released, as the AI wants every vehicle before it starts (TController::CheckVehicles())
		CHECK((wagon.Hamulec->GetBrakeStatus() & (b_on | b_hld)) == 0);
		CHECK(wagon.BrakePress < 0.1);
	}
	// the exchange between the 14 m wagons takes 3 sub-steps of a step of 0.01 s
	CHECK(train[1]->PipeCouplersSubsteps(0.01) == 3);
}

TEST_CASE("brake pipe: an application from the locomotive reaches the end of a freight train as with a short step") {
	// a light application, the locomotive's pipe 1 bar down: the last wagon's pipe 0.5 bar down as soon as in the
	// step of 0.001 s, at which the exchange through the hoses converges
	auto const converged{application_time(freight_train, 37, 4.0, 0.001)};
	auto const usual{application_time(freight_train, 37, 4.0, 0.01)};
	CAPTURE(converged);
	CAPTURE(usual);
	CHECK(converged < 60.0);
	CHECK(usual == doctest::Approx(converged).epsilon(0.1));
	// a full service application
	auto const convergedfull{application_time(freight_train, 37, 3.5, 0.001)};
	auto const usualfull{application_time(freight_train, 37, 3.5, 0.01)};
	CAPTURE(convergedfull);
	CAPTURE(usualfull);
	CHECK(usualfull == doctest::Approx(convergedfull).epsilon(0.1));
}

TEST_CASE("brake pipe: a passenger train, whose exchange was stable in one step, brakes as with a short step") {
	test::consist train;
	passenger_train(train, 10);
	train.run(300.0);
	for (std::size_t i = 1; i < train.size(); ++i) {
		CAPTURE(i);
		auto &car{train[i].mover()};
		CHECK(std::abs(car.Hamulec->GetVRP() - car.PipePress) < 0.005);
		CHECK((car.Hamulec->GetBrakeStatus() & (b_on | b_hld)) == 0);
		CHECK(car.BrakePress < 0.1);
	}
	auto const converged{application_time(passenger_train, 10, 3.5, 0.001)};
	auto const usual{application_time(passenger_train, 10, 3.5, 0.01)};
	CAPTURE(converged);
	CAPTURE(usual);
	CHECK(usual == doctest::Approx(converged).epsilon(0.05));
}
