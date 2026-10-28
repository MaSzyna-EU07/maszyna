#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
#include "drive.h"
#include "golden.h"
#include "real_vehicles.h"

using test::definition;
using test::vehicle;

// characterisation of the drive of every real powered vehicle, driven alone from its front cab under a 3 kV
// catenary through test::drive_cycle(), in golden traces (data/golden/traction/). the cars of the electric
// units which take power or battery from the rest of their unit don't start alone: their traces record
// that, test_consist.cpp drives the whole units

TEST_CASE("traction: every real powered vehicle started up, driven to full power and braked to a stop") {
	for (auto const &path : test::real_vehicles()) {
		CAPTURE(path);
		test::reseed_random();
		vehicle real{definition::fiz(path), 0.0, "test", 1};
		if (false == test::is_powered(real.mover())) {
			continue;
		}
		using namespace test::channels;
		test::trace trace{"traction/" + path, motion() + traction() + air() + brakes(), 100};
		real.record(trace);
		test::drive_cycle(test::driving(real), test::track{0.15, 0.0, test::catenary_3kv}, trace);
		trace.check();
	}
}
