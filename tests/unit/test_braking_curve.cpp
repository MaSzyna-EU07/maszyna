#include "stdafx.h"
#include "doctest.h"

#include "vehicle/brakingcurve.h"

#include <algorithm>
#include <array>

// the deceleration the AI asks for on its way down to a stop or a lower speed ahead, outside of the final approach
// (TController::adjust_desired_speed_for_braking_curve). in shunting the brake table of the consist isn't worked out
// and stays at zero; planned from it, the curve asked for next to no braking, and shunting engines ran through the
// signals they were to stop at.

namespace {

auto constexpr tablesize{20};
using brake_values = std::array<double, tablesize + 1>;

// what the AI holds in shunting: nothing worked out
struct zeroed_table {
	brake_values a0{};
	brake_values a1{};

	braking_curve::brake_table table() const { return {a0.data(), a1.data(), tablesize, 100.0}; }
};

// a train brake that gives the same deceleration at every speed, full service 0.05 + 12 * 0.1 = 1.25 m/s2
struct flat_table {
	brake_values a0{};
	brake_values a1{};

	flat_table() {
		a0.fill(0.05);
		a1.fill(0.1);
	}
	braking_curve::brake_table table() const { return {a0.data(), a1.data(), tablesize, 120.0}; }
};

braking_curve::approach stop_at(double const Speed, double const Distance) {
	braking_curve::approach approach{};
	approach.speed = Speed;
	approach.target = 0.0;
	approach.distance = Distance;
	approach.offset = 0.0; // the braking point of a stop, outside of a train, is the stop itself (VelNext * multiplier)
	approach.speedup_horizon = 30.0 / 3.6;
	approach.fill_horizon = 30.0 / 3.6;
	return approach;
}

// the AI in shunting leaves the brake alone for a demand weaker than this (TController::fAccThreshold)
auto constexpr shunting_brake_threshold{-0.2};

struct shunting_run {
	double speed; // [km/h]
	double left; // to the stop point [m]
	bool braked_at_once;
};

// a shunting engine braking to a stop point the way the AI drives it: the brake follows the demand once it is past
// the threshold, the engine coasts otherwise. the braking curve gives way near the point to the final approach, at
// walking pace (10 km/h, 5 km/h in the last 10 m)
shunting_run brake_to_point(double const Speed, double const Distance, zeroed_table const &Table) {
	auto const table{Table.table()};
	auto const timestep{0.1};
	shunting_run run{Speed, Distance, false};
	run.braked_at_once = braking_curve::demand(stop_at(run.speed, run.left), &table) < shunting_brake_threshold;
	while ((run.speed > (run.left > 10.0 ? 10.0 : 5.0)) && (run.left > -50.0)) {
		auto const demand{braking_curve::demand(stop_at(run.speed, run.left), &table)};
		auto const acceleration{demand < shunting_brake_threshold ? demand : 0.0};
		run.speed = std::max(0.0, run.speed + 3.6 * acceleration * timestep);
		run.left -= run.speed / 3.6 * timestep;
	}
	return run;
}

} // namespace

TEST_CASE("braking curve: a brake table left at zero isn't known, one worked out is") {
	CHECK_FALSE(zeroed_table{}.table().known());
	CHECK(flat_table{}.table().known());
}

TEST_CASE("braking curve: shunting to a point asks for the constant deceleration over the distance left") {
	zeroed_table const shunting;
	auto const table{shunting.table()};
	auto approach{stop_at(33.0, 186.0)};
	SUBCASE("coasting") { approach.acceleration = 0.0; }
	SUBCASE("still speeding up") { approach.acceleration = 0.2; }

	auto const demand{braking_curve::demand(approach, &table)};

	CHECK(demand == doctest::Approx(-33.0 * 33.0 / (25.92 * 186.0 + 0.1)));
	CHECK(demand == doctest::Approx(braking_curve::demand(approach, nullptr)));
	// enough for the AI to put the brake on
	CHECK(demand < shunting_brake_threshold);
}

TEST_CASE("braking curve: a shunting engine braking to a point gets down to walking pace short of it") {
	zeroed_table const shunting;
	auto speed{0.0};
	auto distance{0.0};
	SUBCASE("33 km/h, 186 m") {
		speed = 33.0;
		distance = 186.0;
	}
	SUBCASE("40 km/h, 186 m") {
		speed = 40.0;
		distance = 186.0;
	}
	SUBCASE("33 km/h, 120 m") {
		speed = 33.0;
		distance = 120.0;
	}
	SUBCASE("25 km/h, 60 m") {
		speed = 25.0;
		distance = 60.0;
	}
	CAPTURE(speed);
	CAPTURE(distance);

	auto const run{brake_to_point(speed, distance, shunting)};

	CHECK(run.braked_at_once);
	CHECK(run.left > 0.0);
	CHECK(run.speed <= 10.0);
}

TEST_CASE("braking curve: a train with a worked out brake table plans along the shape of its brake") {
	flat_table const train;
	auto const table{train.table()};
	auto approach{stop_at(80.0, 600.0)};
	approach.offset = 20.0;
	approach.fill_horizon = 6.0;
	approach.speedup_horizon = 5.0;

	SUBCASE("before the brake is applied, the room it takes to fill is kept") {
		auto const room{580.0 - 80.0 / 3.6 * 6.0};
		auto const demand{braking_curve::demand(approach, &table)};
		CHECK(demand == doctest::Approx(braking_curve::shaped_demand(80.0, 0.0, room + 0.1 / 25.92, 0.0, table)));
		// with the same deceleration at every speed the shape is the constant deceleration over that room
		CHECK(demand == doctest::Approx(-(80.0 / 3.6) * (80.0 / 3.6) / (2.0 * room)).epsilon(0.01));
	}
	SUBCASE("once the brake is applied, the whole distance is used") {
		approach.brake_applied = true;
		auto const demand{braking_curve::demand(approach, &table)};
		CHECK(demand == doctest::Approx(braking_curve::shaped_demand(80.0, 0.0, 580.0 + 0.1 / 25.92, 0.0, table)));
	}
	SUBCASE("still speeding up, it plans from the speed reached once the brake acts") {
		approach.acceleration = 0.3;
		auto const reached{80.0 + 3.6 * 5.0 * 0.3};
		auto const distance{600.0 - (80.0 + reached) * 0.5 / 3.6 * 5.0};
		auto const demand{braking_curve::demand(approach, &table)};
		CHECK(demand == doctest::Approx(braking_curve::shaped_demand(reached, 0.0, distance - 20.0 + 0.1 / 25.92, 0.0, table)));
	}
}

TEST_CASE("braking curve: a train gets the demand it got before the curve moved out of the AI") {
	// a table that grips harder as the train slows, on a gradient; the demands were worked out with the
	// code of TController::adjust_desired_speed_for_braking_curve before the move (origin/master 80fd5aad)
	brake_values a0{};
	brake_values a1{};
	for (auto i{1}; i <= tablesize; ++i) {
		a0[i] = 0.12 - 0.003 * i;
		a1[i] = 0.06 - 0.001 * i;
	}
	braking_curve::brake_table const table{a0.data(), a1.data(), tablesize, 120.0};
	REQUIRE(table.full_braking(80.0) == doctest::Approx(0.645));
	REQUIRE(table.full_braking(10.0) == doctest::Approx(0.825));

	auto approach{stop_at(80.0, 600.0)};
	approach.offset = 20.0;
	approach.gravity = 0.1;
	approach.speedup_horizon = 5.0;
	approach.fill_horizon = 6.0;
	auto expected{0.0};
	SUBCASE("downhill, coasting, the brake still to be applied") { expected = -0.4926656801067293; }
	SUBCASE("downhill, the brake applied") {
		approach.brake_applied = true;
		expected = -0.37739905908238147;
	}
	SUBCASE("downhill, still speeding up") {
		approach.acceleration = 0.3;
		expected = -0.5330117415264249;
	}
	SUBCASE("uphill, down to 40 km/h") {
		approach = stop_at(100.0, 900.0);
		approach.target = 40.0;
		approach.gravity = -0.08;
		approach.speedup_horizon = 5.0;
		approach.fill_horizon = 6.0;
		expected = -0.40930710539221765;
	}

	CHECK(braking_curve::demand(approach, &table) == doctest::Approx(expected).epsilon(1e-7));
}
