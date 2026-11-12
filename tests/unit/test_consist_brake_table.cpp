#include "stdafx.h"
#include "doctest.h"

#include "vehicle/consistbraketable.h"

#include <array>

// the brake table the AI keeps of its consist, as its braking curve reads it: filled in for a train run,
// left zeroed in shunting, where the curve must not take it for the brakes (backlog D24: shunting AI
// passed shunting and exit signals at stop at 24-29 km/h)

namespace {

constexpr int size{20}; // BrakeAccTableSize of the AI
constexpr double vmax{120.0};

// the table as TController::CheckVehicles() leaves it in shunting: entries 0 to size, all zero
struct zeroed_table {
	std::array<double, size + 1> a0{};
	std::array<double, size + 1> a1{};
	consist_brake_table table() const {
		return {a0.data(), a1.data(), size, vmax};
	}
};

} // namespace

TEST_CASE("consist brake table: a zeroed table, as in shunting, is not known") {
	zeroed_table const zeroed;
	CHECK_FALSE(zeroed.table().known());
	// and reads as next to no brake at all, at any speed
	CHECK(zeroed.table().full_braking(30.0) == doctest::Approx(0.01));
	CHECK(zeroed.table().full_braking(100.0) == doctest::Approx(0.01));
}

TEST_CASE("consist brake table: a table filled in for a train run is known and gives its brake by speed") {
	zeroed_table filled;
	for (auto index{1}; index <= size; ++index) {
		filled.a0[index] = 0.2 + 0.01 * index;
		filled.a1[index] = 0.05;
	}
	CHECK(filled.table().known());
	// 30 km/h of 120 is entry 5: a0 + 12 a1
	CHECK(filled.table().full_braking(30.0) == doctest::Approx(0.25 + 12.0 * 0.05));
	// below the first entry and above the top speed the end entries hold
	CHECK(filled.table().full_braking(0.0) == doctest::Approx(0.21 + 12.0 * 0.05));
	CHECK(filled.table().full_braking(200.0) == doctest::Approx(0.40 + 12.0 * 0.05));
}

TEST_CASE("consist brake table: entry 0, the one for the present speed, doesn't make a zeroed table known") {
	zeroed_table zeroed;
	zeroed.a0[0] = 0.3;
	zeroed.a1[0] = 0.05;
	CHECK_FALSE(zeroed.table().known());
	// one entry of either part is enough
	zeroed_table onlya1;
	onlya1.a1[size] = 0.01;
	CHECK(onlya1.table().known());
}
