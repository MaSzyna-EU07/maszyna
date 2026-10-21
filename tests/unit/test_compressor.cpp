#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

using test::definition;
using test::vehicle;

// compressor fed from the main circuit, started on its own: the generator of 401Da, which feeds the compressor
// only once it reaches full voltage some time after the engine start (eu07/maszyna#546)

namespace {

definition compressor_vehicle() {
	return definition::load("cab_vehicle")
	    .set("Brake", "Vv", 0.6)
	    .set("Brake", "MinCP", 6.5)
	    .set("Brake", "MaxCP", 8.0)
	    .set("Brake", "CompressorSpeed", 0.003)
	    .set("Brake", "CompressorPower", "Main")
	    .set("Cntrl", "CompressorStart", "Automatic");
}

} // namespace

TEST_CASE("compressor: without a start delay it starts together with the engine") {
	vehicle loco{compressor_vehicle()};
	REQUIRE(loco->CompressorSpeed > 0.0);
	REQUIRE(loco->CompressorStartDelay == 0.0f);

	loco->Mains = true; // engine started
	loco.run(1.0);
	CHECK(loco->CompressorFlag);
}

TEST_CASE("compressor: with a start delay it starts only once the delay since the engine start passed") {
	vehicle loco{compressor_vehicle().set("Cntrl", "CompressorStartDelay", 30)};
	REQUIRE(loco->CompressorStartDelay == doctest::Approx(30.0));

	loco.run(60.0); // engine off: the delay doesn't run down
	CHECK_FALSE(loco->CompressorFlag);

	loco->Mains = true; // engine started
	loco.run(25.0);
	CHECK_FALSE(loco->CompressorFlag);
	auto const pressure{loco->Compressor};

	loco.run(10.0);
	CHECK(loco->CompressorFlag);
	CHECK(loco->Compressor > pressure);

	// engine stopped and started again: the generator has to get up to voltage anew
	loco->Mains = false;
	loco.run(1.0);
	CHECK_FALSE(loco->CompressorFlag);
	loco->Mains = true;
	loco.run(25.0);
	CHECK_FALSE(loco->CompressorFlag);
	loco.run(10.0);
	CHECK(loco->CompressorFlag);
}
