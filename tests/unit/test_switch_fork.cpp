#include "stdafx.h"
#include "doctest.h"
#include "world/switchfork.h"

#include <cmath>
#include <numbers>
#include <optional>

// a switch thrown under a train (world/switchfork.h): the way of a switch an axle keeps, how two axles following each
// other in the train come to run on its two ways, and how far a vehicle runs forked before it derails

namespace sf = switch_fork;

namespace {

constexpr double pi{std::numbers::pi};

double degrees(double const Radians) {
	return Radians * 180.0 / pi;
}

// a point of the plane of the scenery (x, z) with the heading of the line through it, as TTrackFollower has it
struct place {
	double x{0.0};
	double z{0.0};
	double heading{0.0};
};

// the switch of td.scn (zwr01, zwrp34r300): from the points along +z, way 0 straight on, way 1 a curve of 300 m
// bending to +x. the place S metres from the points
constexpr double switch_radius{300.0};
place straight_way(double const S) {
	return {0.0, S, 0.0};
}
place curved_way(double const S, double const Radius = switch_radius) {
	auto const angle{S / Radius};
	auto const x{Radius * (1.0 - std::cos(angle))};
	auto const z{Radius * std::sin(angle)};
	// the tangent (sin a, cos a), the heading atan2(-x, z) of it
	return {x, z, sf::heading(std::sin(angle), std::cos(angle))};
}

// a vehicle with the bogie pivots Pivots apart, the front one at Front, the rear one somewhere along Way (by the
// distance from the points), Pivots from the front one: its bogies turned under the body
struct vehicle_turns {
	double front{0.0};
	double rear{0.0};
	double apart{0.0}; // how far the rear pivot is from the line of the way of the front one [m]
};
template <typename Way>
vehicle_turns turns(place const &Front, double const Pivots, Way const &Rearway, double const Rearguess) {
	// the rear pivot where it is Pivots from the front one, found by halving
	auto low{Rearguess - Pivots};
	auto high{Rearguess + Pivots};
	auto const distance = [&](double const S) {
		auto const rear{Rearway(S)};
		return std::hypot(Front.x - rear.x, Front.z - rear.z);
	};
	for (int i = 0; i < 80; ++i) {
		auto const middle{0.5 * (low + high)};
		// the farther along the way, the nearer to the front pivot
		(distance(middle) > Pivots ? low : high) = middle;
	}
	auto const rear{Rearway(0.5 * (low + high))};
	auto const body{sf::heading(Front.x - rear.x, Front.z - rear.z)};
	return {sf::turn(Front.heading, body), sf::turn(rear.heading, body), std::abs(rear.x - Front.x)};
}

} // namespace

TEST_CASE("switch fork: the way an axle leaves a switch by") {
	// set the way it ran onto: left as it is set, by the very call of before
	CHECK(sf::leaving_path(0, 0) == -1);
	CHECK(sf::leaving_path(1, 1) == -1);
	// thrown under it: its own way
	CHECK(sf::leaving_path(0, 1) == 0);
	CHECK(sf::leaving_path(1, 0) == 1);
	// the state of a switch is its lowest bit (TTrack::Switch() takes i & 1)
	CHECK(sf::leaving_path(1, 3) == -1);
	CHECK(sf::leaving_path(0, 3) == 0);
	// off a switch
	CHECK(sf::leaving_path(-1, 0) == -1);
	CHECK(sf::leaving_path(-1, 1) == -1);
}

TEST_CASE("switch fork: the ways an axle remembers") {
	int a{}, b{}, c{}, d{}, e{};
	sf::path_memory memory;
	CHECK(memory.path_of(&a) == std::nullopt);
	memory.record(&a, 0);
	memory.record(&b, 1);
	CHECK(memory.path_of(&a) == 0);
	CHECK(memory.path_of(&b) == 1);
	CHECK(memory.path_of(&c) == std::nullopt);
	SUBCASE("the last run over a switch counts") {
		memory.record(&a, 1);
		CHECK(memory.path_of(&a) == 1);
		CHECK(memory.count() == 2);
	}
	SUBCASE("the oldest is forgotten") {
		memory.record(&c, 0);
		memory.record(&d, 0);
		CHECK(memory.count() == sf::path_memory::size);
		memory.record(&e, 1);
		CHECK(memory.count() == sf::path_memory::size);
		CHECK(memory.path_of(&a) == std::nullopt);
		CHECK(memory.path_of(&b) == 1);
		CHECK(memory.path_of(&e) == 1);
	}
	SUBCASE("a switch run over again is the newest") {
		memory.record(&c, 0);
		memory.record(&d, 0);
		memory.record(&a, 1);
		memory.record(&e, 0);
		// b is the oldest now, not a
		CHECK(memory.path_of(&b) == std::nullopt);
		CHECK(memory.path_of(&a) == 1);
	}
	SUBCASE("cleared") {
		memory.clear();
		CHECK(memory.path_of(&a) == std::nullopt);
	}
}

TEST_CASE("switch fork: two axles on the two ways") {
	CHECK_FALSE(sf::forked(0, 0));
	CHECK_FALSE(sf::forked(1, 1));
	CHECK(sf::forked(1, 0));
	CHECK(sf::forked(0, 1));
	// the axle ahead didn't run over the switch, or it isn't known: no fork
	CHECK_FALSE(sf::forked(1, std::nullopt));
	CHECK_FALSE(sf::forked(1, -1));
}

TEST_CASE("switch fork: the angle a bogie takes") {
	// asin(a / 2R) in a 150 m curve, a vehicle without Rmin in its definition: a bdhpumn coach with its pivots 19.5 m
	// apart 3.73 deg, the EU07 (8.55 m) 1.63 deg; both have Rmin=80: 7.00 and 3.06 deg
	CHECK(degrees(sf::bogie_limit(19.5)) == doctest::Approx(3.727).epsilon(0.001));
	CHECK(degrees(sf::bogie_limit(8.55)) == doctest::Approx(1.633).epsilon(0.001));
	CHECK(degrees(sf::bogie_limit(19.5, 80.0)) == doctest::Approx(7.000).epsilon(0.001));
	CHECK(degrees(sf::bogie_limit(8.55, 80.0)) == doctest::Approx(3.063).epsilon(0.001));
	CHECK(sf::bogie_limit(19.0, 300.0) < sf::bogie_limit(19.0));
	CHECK(sf::bogie_limit(0.0) == 0.0);
	// pivots farther apart than the curve is wide: a half turn at most
	CHECK(sf::bogie_limit(400.0) == doctest::Approx(0.5 * pi));
	auto const state{sf::bogies(-0.05, 0.07, 19.0)};
	CHECK(state.value == doctest::Approx(0.07));
	CHECK(state.past());
	CHECK_FALSE(sf::bogies(0.03, -0.03, 19.0).past());
	// built for tighter curves, it takes more
	CHECK_FALSE(sf::bogies(-0.05, 0.07, 19.0, 80.0).past());
}

TEST_CASE("switch fork: the angle between a track and the body") {
	CHECK(sf::turn(0.1, 0.0) == doctest::Approx(0.1));
	CHECK(sf::turn(-0.1, 0.0) == doctest::Approx(-0.1));
	// a track runs either way: the line, not the direction
	CHECK(sf::turn(pi + 0.1, 0.0) == doctest::Approx(0.1));
	CHECK(sf::turn(0.1 - pi, 0.0) == doctest::Approx(0.1));
	CHECK(sf::turn(0.0, pi) == doctest::Approx(0.0).epsilon(1e-12));
	// across the seam of the angles
	CHECK(sf::turn(pi - 0.05, -pi + 0.05) == doctest::Approx(-0.1));
	CHECK(std::abs(sf::turn(0.5 * pi, 0.0)) == doctest::Approx(0.5 * pi));
}

TEST_CASE("switch fork: the ends of coupled vehicles apart sideways") {
	// heading 0 is along +z: across it is x
	CHECK(sf::end_offset(0.0, 10.0, 0.0, 0.3, 10.0, 0.0) == doctest::Approx(0.3));
	// along the vehicles it's no offset (the coupling is as long as it is)
	CHECK(sf::end_offset(0.0, 10.0, 0.0, 0.0, 11.0, 0.0) == doctest::Approx(0.0));
	// either vehicle the other way round
	CHECK(sf::end_offset(0.0, 10.0, pi, 0.3, 10.0, 0.0) == doctest::Approx(0.3));
	// the mean of the directions of the two
	CHECK(sf::end_offset(0.0, 0.0, 0.1, std::cos(0.05), std::sin(0.05), -0.0) == doctest::Approx(1.0).epsilon(1e-9));
	CHECK(sf::coupling(0.44).limit == doctest::Approx(0.45));
	CHECK_FALSE(sf::coupling(0.44).past());
	CHECK(sf::coupling(-0.46).past());
}

TEST_CASE("switch fork: a vehicle in a curve takes it, forked it doesn't for long") {
	// a coach with its pivots 19.5 m apart (bdhpumn), the EU07 8.55 m
	for (auto const pivots : {19.5, 8.55}) {
		CAPTURE(pivots);
		auto const limit{sf::bogie_limit(pivots)};
		// both bogies on the curve of 300 m: a / 2R, half of what it takes, the two the opposite ways
		auto const curve{turns(curved_way(30.0), pivots, [](double S) { return curved_way(S); }, 30.0 - pivots)};
		CHECK(curve.front == doctest::Approx(-curve.rear).epsilon(0.01));
		CHECK(std::abs(curve.front) == doctest::Approx(std::asin(pivots / (2.0 * switch_radius))).epsilon(0.01));
		CHECK_FALSE(sf::bogies(curve.front, curve.rear, pivots).past());
		// in a curve of 150 m it's at the limit; tighter, past it
		auto const tight{turns(curved_way(30.0, 150.0), pivots, [](double S) { return curved_way(S, 150.0); }, 30.0 - pivots)};
		CHECK(sf::bogies(tight.front, tight.rear, pivots).value == doctest::Approx(limit).epsilon(0.01));
		auto const tighter{turns(curved_way(30.0, 140.0), pivots, [](double S) { return curved_way(S, 140.0); }, 30.0 - pivots)};
		CHECK(sf::bogies(tighter.front, tighter.rear, pivots).past());
		// both on the straight way, past the switch
		auto const straight{turns(straight_way(40.0), pivots, [](double S) { return straight_way(S); }, 40.0 - pivots)};
		CHECK(straight.front == doctest::Approx(0.0).epsilon(1e-9));
		CHECK(straight.rear == doctest::Approx(0.0).epsilon(1e-9));
	}
}

TEST_CASE("switch fork: the coach forked on the switch of td.scn") {
	// the front bogie ran on along way 0, the rear one took way 1 after the throw: both turn the same way under the
	// body, more the farther the rear one is along the curve. the bdhpumn coach: pivots 19.5 m apart, Rmin=80
	auto constexpr pivots{19.5};
	auto constexpr tightest{80.0};
	auto previous{0.0};
	std::optional<double> derails;
	for (auto rear = 0.5; rear <= 34.0; rear += 0.5) {
		auto const front{straight_way(rear + pivots)};
		auto const forked{turns(front, pivots, [](double S) { return curved_way(S); }, rear)};
		auto const state{sf::bogies(forked.front, forked.rear, pivots, tightest)};
		// the same way round, not opposite as in a curve
		CHECK(forked.front * forked.rear > 0.0);
		CHECK(state.value > previous);
		previous = state.value;
		if (false == derails.has_value() && state.past()) {
			derails = rear;
			MESSAGE("the coach derails with its rear bogie " << rear << " m into the switch, the bogies " << forked.apart << " m apart sideways, turned "
			                                                 << degrees(state.value) << " deg");
			// 23 m into the switch (34 m long), the two ways 0.88 m apart there, the bogies turned 7 deg
			CHECK(rear == doctest::Approx(23.0).epsilon(0.03));
			CHECK(forked.apart == doctest::Approx(0.88).epsilon(0.05));
		}
	}
	REQUIRE(derails.has_value());
	// without Rmin in its definition (150 m), sooner
	std::optional<double> plain;
	for (auto rear = 0.25; rear <= 34.0 && false == plain.has_value(); rear += 0.25) {
		auto const forked{turns(straight_way(rear + pivots), pivots, [](double S) { return curved_way(S); }, rear)};
		if (sf::bogies(forked.front, forked.rear, pivots).past()) {
			plain = rear;
		}
	}
	REQUIRE(plain.has_value());
	CHECK(*plain == doctest::Approx(14.5).epsilon(0.03));
	// the EU07, its pivots closer, sooner
	auto constexpr eu07{8.55};
	std::optional<double> eu07derails;
	for (auto rear = 0.25; rear <= 34.0 && false == eu07derails.has_value(); rear += 0.25) {
		auto const forked{turns(straight_way(rear + eu07), eu07, [](double S) { return curved_way(S); }, rear)};
		if (sf::bogies(forked.front, forked.rear, eu07, tightest).past()) {
			eu07derails = rear;
		}
	}
	REQUIRE(eu07derails.has_value());
	MESSAGE("the EU07 derails with its rear bogie " << *eu07derails << " m into the switch; the coach without Rmin at " << *plain << " m");
	CHECK(*eu07derails == doctest::Approx(10.25).epsilon(0.03));
}

TEST_CASE("switch fork: coupled vehicles forked on the switch of td.scn") {
	// the end of a coach 3.65 m before its front pivot (bdhpumn: 26.8 m long, pivots 19.5 m apart), coupled to the end
	// of a vehicle running on along way 0 straight on; its front pivot took way 1
	auto constexpr overhang{3.65};
	auto constexpr pivots{19.5};
	std::optional<double> derails;
	for (auto front = 0.5; front <= 34.0 && false == derails.has_value(); front += 0.5) {
		auto const pivot{curved_way(front)};
		// the body of the coach: its rear pivot on the curve too, or before the switch on the straight line before it
		auto const rearway = [](double S) { return S >= 0.0 ? curved_way(S) : place{0.0, S, 0.0}; };
		auto low{front - pivots - 1.0}, high{front};
		for (int i = 0; i < 80; ++i) {
			auto const middle{0.5 * (low + high)};
			auto const rear{rearway(middle)};
			(std::hypot(pivot.x - rear.x, pivot.z - rear.z) > pivots ? low : high) = middle;
		}
		auto const rear{rearway(0.5 * (low + high))};
		auto const body{sf::heading(pivot.x - rear.x, pivot.z - rear.z)};
		// the end of the coach ahead of its front pivot, along the body
		auto const endx{pivot.x - overhang * std::sin(body)};
		auto const endz{pivot.z + overhang * std::cos(body)};
		// the end of the vehicle ahead on the straight way, where the coupling holds it: as far along as the coach end
		auto const offset{sf::end_offset(endx, endz, body, 0.0, endz, 0.0)};
		if (sf::coupling(offset).past()) {
			derails = front;
			MESSAGE("the coupled coach derails with its front bogie " << front << " m into the switch, its end " << offset << " m off the other end");
		}
	}
	REQUIRE(derails.has_value());
	// 15.5 m into the switch, the ends 0.48 m apart
	CHECK(*derails == doctest::Approx(15.5).epsilon(0.03));
}
