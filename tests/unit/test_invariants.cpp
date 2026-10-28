#include "stdafx.h"
#include "doctest.h"
#include "golden.h"
#include "vehicle.h"

// the invariants every golden trace checks (invariants.h) catch a vehicle in a state the simulation must
// never reach, once each, naming the time and the last event

using test::definition;
using test::vehicle;

TEST_CASE("invariants: a vehicle in an impossible state, reported once with its time and the last event") {
	vehicle eu07{definition::fiz("4e_v1/4e")};
	test::trace trace{"invariants/self test", test::channels::brakes()};
	trace.sample(0.0, eu07.mover());
	CHECK(trace.violations().empty());

	trace.mark("overfilled");
	eu07->BrakePress = 50.0;
	trace.sample(0.01, eu07.mover());
	trace.sample(0.02, eu07.mover());
	REQUIRE(trace.violations().size() == 1);
	auto const &violation{trace.violations().front()};
	CHECK(violation.find("brake cylinders") != std::string::npos);
	CHECK(violation.find("at 0.010 s") != std::string::npos);
	CHECK(violation.find("after \"overfilled\"") != std::string::npos);
	CHECK(violation.find("BrakePress=50") != std::string::npos);

	eu07->BrakePress = 0.0;
	eu07->V = std::nan("");
	trace.sample(0.03, eu07.mover());
	CHECK(trace.violations().size() > 1);
	CHECK(trace.violations()[1].find("every value a number") != std::string::npos);
}
