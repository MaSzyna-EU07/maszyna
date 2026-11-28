#include "stdafx.h"
#include "doctest.h"
#include "McZapkie/MOVER.h"

#include <memory>

// the snow lying on the track holding a vehicle back (TMoverParameters::SnowForce(), physics.snow)

namespace {

std::unique_ptr<TMoverParameters> mover(double const Snow, double const Density, double const Speed) {
	auto result{std::make_unique<TMoverParameters>(0.0, "test", "test", 1)};
	result->RunningTrack.snow = Snow;
	result->RunningTrack.snow_density = Density;
	result->V = Speed;
	return result;
}

} // namespace

TEST_CASE("snow force: none without snow on the track") {
	CHECK(mover(0.0, 150.0, 20.0)->SnowForce() == 0.0);
}

TEST_CASE("snow force: the flanges only, while the snow is below the heads of the rails") {
	auto const low{mover(0.10, 150.0, 20.0)->SnowForce()};
	// 150 kg/m3 * 0.15 m * 0.10 m * (400 + 10)
	CHECK(low == doctest::Approx(150.0 * 0.15 * 0.10 * 410.0));
	// a few hundred newtons: nothing a train notices much
	CHECK(low < 2000.0);
}

TEST_CASE("snow force: snow over the rails pushed aside, growing with the square of the speed") {
	auto const slow{mover(0.37, 150.0, 5.0)->SnowForce()};
	auto const fast{mover(0.37, 150.0, 20.0)->SnowForce()};
	CAPTURE(slow);
	CAPTURE(fast);
	// 20 cm over the heads at 72 km/h: tens of kilonewtons, a good part of what a locomotive pulls with
	CHECK(fast > 30000.0);
	CHECK(fast < 60000.0);
	CHECK(fast > slow * 10.0);
	// denser (older, wetter) snow holds back more
	CHECK(mover(0.37, 300.0, 20.0)->SnowForce() == doctest::Approx(fast * 2.0));
}
