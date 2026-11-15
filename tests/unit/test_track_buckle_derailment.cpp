#include "stdafx.h"
#include "doctest.h"
#include "McZapkie/derailment.h"
#include "world/TrackBuckle.h"

#include <cmath>

// a vehicle over a buckled track, by the criteria of derailment.h: the curvature of the buckle goes into the track
// under the vehicle as buckle_curvature (world/TrackBuckle.h: the body by the mean of its pivots, each bogie over its
// wheelbase), whole, as the curvature of the curve does

namespace {

// a four-axle coach: 42 t, 24.5 m over the buffers, the pivots 17.2 m and the axles of a bogie 2.5 m apart
derailment::vehicle_state coach()
{
	derailment::vehicle_state vehicle;
	vehicle.mass = 42000.0;
	vehicle.axles = 4;
	vehicle.length = 24.5;
	vehicle.axle_base = 2.5;
	vehicle.centre_of_gravity = 1.4;
	return vehicle;
}
double constexpr pivots{17.2};

// a two-axle wagon: 25 t, 14 m over the buffers, the axles 9 m apart, which are its pivots
derailment::vehicle_state wagon()
{
	derailment::vehicle_state vehicle;
	vehicle.mass = 25000.0;
	vehicle.axles = 2;
	vehicle.length = 14.0;
	vehicle.axle_base = 9.0;
	vehicle.centre_of_gravity = 1.6;
	return vehicle;
}

struct rolling_stock {
	derailment::vehicle_state vehicle;
	double pivots;
	double base; // of the bogies, 0 without
};
rolling_stock const four_axle{coach(), pivots, 2.5};
rolling_stock const two_axle{wagon(), 9.0, 0.0};

// Outward: the side out of the curve of Radius (-1 left, 1 right), 0 on a straight track
derailment::outcome ride(track_buckle::shape const &Shape, double const Radius, int const Outward, double const Speed, double const Step, rolling_stock const &Stock = four_axle)
{
	auto const &vehicle{Stock.vehicle};
	derailment::monitor monitor;
	auto const velocity{Speed / 3.6};
	auto const reach{track_buckle::reach(Shape) + Stock.pivots};
	for (auto at = Shape.centre - reach - 5.0; at <= Shape.centre + reach + 5.0; at += velocity * Step)
	{
		derailment::track_state track;
		track.radius = Radius;
		auto const bend{track_buckle::body_bend(Shape, at, Stock.pivots, Stock.base)};
		// positive bending with the curve, whose centre is away from the outer side
		track.buckle_curvature = (Outward != 0 ? -Outward * bend : bend);
		auto const verdict{monitor.update(vehicle, track, velocity, velocity * Step)};
		if (verdict.result != derailment::outcome::none)
			return verdict.result;
	}
	return derailment::outcome::none;
}

// the lowest speed it leaves the rails at, km/h, to 0.5 km/h
double limit(track_buckle::shape const &Shape, double const Radius, int const Outward, double const Step, rolling_stock const &Stock = four_axle)
{
	auto low{5.0};
	auto high{300.0};
	while (high - low > 0.5)
	{
		auto const middle{0.5 * (low + high)};
		if (ride(Shape, Radius, Outward, middle, Step, Stock) != derailment::outcome::none)
			high = middle;
		else
			low = middle;
	}
	return high;
}

} // namespace

TEST_CASE("a coach goes over a buckle slowly, and leaves the rails on it fast")
{
	// 0.3 m out over a main lobe of 20 m on a straight track: an equivalent radius of 110 m, felt by the body less
	track_buckle::shape const shape{0.3, 20.0, 60.0};
	CHECK(ride(shape, 0.0, 0, 30.0, 0.01) == derailment::outcome::none);
	CHECK(ride(shape, 0.0, 0, 60.0, 0.01) == derailment::outcome::none);
	CHECK(ride(shape, 0.0, 0, 140.0, 0.01) != derailment::outcome::none);
	auto const speed{limit(shape, 0.0, 0, 0.01)};
	MESSAGE("coach over a buckle 0.3 m by 20 m on a straight track: off the rails from ", speed, " km/h");
	CHECK(speed > 60.0);
	CHECK(speed < 140.0);
	// the same with any step
	CHECK(limit(shape, 0.0, 0, 0.002) == doctest::Approx(speed).epsilon(2.0 / speed));
	CHECK(limit(shape, 0.0, 0, 0.03) == doctest::Approx(speed).epsilon(2.0 / speed));
	// either way on a straight track
	track_buckle::shape const left{-0.3, 20.0, 60.0};
	CHECK(limit(left, 0.0, 0, 0.01) == doctest::Approx(speed).epsilon(1.0 / speed));
	// a larger buckle derails it slower
	auto const large{limit(track_buckle::shape{0.6, 20.0, 60.0}, 0.0, 0, 0.01)};
	MESSAGE("0.6 m by 20 m: ", large, " km/h");
	CHECK(large < speed);
	// the default one of a straight track, 0.4 m by 14 m
	auto const natural{limit(track_buckle::shape{track_buckle::straight_amplitude, track_buckle::straight_length, 60.0}, 0.0, 0, 0.01)};
	MESSAGE("the default buckle of a straight track, 0.4 m by 14 m: ", natural, " km/h");
	CHECK(natural < speed);
	// a body on two axles 9 m apart doesn't span the main lobe as one on bogies 17.2 m apart does: it follows the
	// buckle closer, and goes off the rails slower
	auto const short_wagon{limit(shape, 0.0, 0, 0.01, two_axle)};
	MESSAGE("a two-axle wagon over 0.3 m by 20 m: ", short_wagon, " km/h");
	CHECK(short_wagon < speed);
	CHECK(ride(shape, 0.0, 0, 30.0, 0.01, two_axle) == derailment::outcome::none);
}

TEST_CASE("a buckle out of a curve derails a coach slower than the same on a straight track")
{
	track_buckle::shape const shape{0.3, 20.0, 60.0};
	auto const straight{limit(shape, 0.0, 0, 0.01)};
	// a curve of 600 m without cant, the buckle to the right out of it (a curve to the left)
	auto const curve{limit(track_buckle::shape{}, 600.0, 0, 0.01)};
	auto const outwards{limit(shape, 600.0, 1, 0.01)};
	MESSAGE("R 600 m: the curve alone from ", curve, " km/h, with the buckle out of it from ", outwards, " km/h, on a straight ", straight, " km/h");
	CHECK(outwards < straight);
	CHECK(outwards < curve);
	// one into the curve (not what the heat does) still lowers it, by its flanks which bend with the curve
	CHECK(limit(track_buckle::shape{-0.3, 20.0, 60.0}, 600.0, 1, 0.01) < curve);
}
