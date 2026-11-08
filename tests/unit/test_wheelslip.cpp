#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

using test::definition;
using test::track;
using test::vehicle;

namespace {

struct braking_result {
	bool skidded{false}; // the wheels locked while braking
	bool stopped{false}; // came (almost) to a stop on the braking rail
};

// brakes the wagon from 40 km/h on the Braking rail until it's (nearly) stopped, then has it stand on the
// Standing one for 30 seconds
braking_result brake_to_stop_then_stand(vehicle &Coach, track const &Braking, track const &Standing) {
	braking_result result;
	Coach.hold_brake_pipe(5.0);
	Coach.run(5.0, Braking);
	Coach.hold_brake_pipe(3.5); // service braking
	// a skidding wagon brakes on the sliding adhesion only, give it time
	for (auto step = 0; step < 60000; ++step) { // 600 s in 0.01 s steps
		result.skidded |= Coach->SlippingWheels;
		if (auto const speed{std::abs(Coach.speed())}; speed > 0.0 && speed < 0.1) {
			// almost stopped; a skidding wagon is still skidding at this point
			result.stopped = true;
			break;
		}
		Coach.step(Braking);
	}
	Coach.run(30.0, Standing);
	return result;
}

} // namespace

// the engine doesn't let a stopped, braked vehicle slip (the Vel > 0.1 trap in TMoverParameters::ComputeTotalForce),
// so a wagon which stopped in a skid should be no different from one which stopped without it. it used to stay
// skidding instead, with its brakes capped by the sliding adhesion, and slid or crept away (eu07/maszyna#479)
TEST_CASE("wheelslip: a wagon which stopped in a skid holds like one which stopped without it") {
	track const dry{0.15, 0.0};
	track const slippery{0.02, 0.0};
	// held only by the brakes: the sliding adhesion on this rail is too low for the slope
	track const slipperyslope{0.02, 0.08};

	vehicle normal{definition::load("wagon"), 40.0};
	auto const normalbraking{brake_to_stop_then_stand(normal, dry, slipperyslope)};
	vehicle skidding{definition::load("wagon"), 40.0};
	auto const skiddingbraking{brake_to_stop_then_stand(skidding, slippery, slipperyslope)};

	// the test means something only when the braking went as planned
	REQUIRE(normalbraking.stopped);
	REQUIRE_FALSE(normalbraking.skidded);
	REQUIRE(skiddingbraking.stopped);
	REQUIRE(skiddingbraking.skidded);

	CHECK(std::abs(normal.speed()) < 0.1);
	CHECK(std::abs(skidding.speed()) < 0.1);
	CHECK_FALSE(skidding->SlippingWheels);
}
