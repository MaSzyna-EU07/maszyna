#include "stdafx.h"
#include "doctest.h"

#include "vehicle/driverfigure.h"

// the driver figures of the lowpoly interior (eu07/maszyna#535): in a vehicle with one desk for both directions the
// player's figure follows the selected direction of travel, so a player who only moves the reverser is seen facing the
// way the vehicle goes, the way the AI is, which changes the cab instead

using driver_figure::cab_place;
using driver_figure::shown;
using driver_figure::single_desk;

namespace {

// the cabs of 401da_v2/401da.mmd: the same model and the same seat for both directions
cab_place const desk_401da{"interior/401da-207.t3d", glm::vec3{-0.908f, 3.324f, -1.543f}, glm::vec3{-0.908f, 3.324f, -1.543f}};

// tem2_v2/tem2a.mmd: the cab 2 has another standing position, but the same seat
cab_place const tem2_cab1{"kabina-tem2a.t3d", glm::vec3{-1.27f, 3.594f, -5.61f}, glm::vec3{-1.235f, 3.203f, -5.114f}};
cab_place const tem2_cab2{"kabina-tem2a.t3d", glm::vec3{1.27f, 3.594f, -4.72f}, glm::vec3{-1.235f, 3.203f, -5.114f}};

// sm42_v1/6d.mmd: one cab with two desks, the seat of the cab 2 is on the other side
cab_place const sm42_cab1{"6d_kabina.t3d", glm::vec3{0.99f, 3.454f, -3.311f}, glm::vec3{-1.15f, 3.58f, -2.83f}};
cab_place const sm42_cab2{"6d_kabina.t3d", glm::vec3{-0.99f, 3.454f, -3.311f}, glm::vec3{1.15f, 3.58f, -2.83f}};

// a vehicle with a cab at each end
cab_place const end_cab1{"kabina_a.t3d", glm::vec3{-0.8f, 2.5f, 7.0f}, glm::vec3{-0.8f, 2.4f, 7.2f}};
cab_place const end_cab2{"kabina_a.t3d", glm::vec3{0.8f, 2.5f, -7.0f}, glm::vec3{0.8f, 2.4f, -7.2f}};

} // namespace

TEST_CASE("driver figure: joint cabs with one model and one seat are a single desk") {
	CHECK(single_desk(true, desk_401da, desk_401da));
	CHECK(single_desk(true, tem2_cab1, tem2_cab2));
}

TEST_CASE("driver figure: without jointcabs: the cabs are never a single desk") {
	CHECK_FALSE(single_desk(false, desk_401da, desk_401da));
	CHECK_FALSE(single_desk(false, tem2_cab1, tem2_cab2));
}

TEST_CASE("driver figure: two desks in one cab, or two cabs, are not a single desk") {
	CHECK_FALSE(single_desk(true, sm42_cab1, sm42_cab2));
	CHECK_FALSE(single_desk(true, end_cab1, end_cab2));
}

TEST_CASE("driver figure: the cabs with different models are not a single desk") {
	auto cab2{desk_401da};
	cab2.model = "interior/401da-207_b.t3d";
	CHECK_FALSE(single_desk(true, desk_401da, cab2));
	cab2.model = "none";
	CHECK_FALSE(single_desk(true, desk_401da, cab2));
	auto nomodel{desk_401da};
	nomodel.model = "none";
	CHECK_FALSE(single_desk(true, nomodel, nomodel));
	nomodel.model.clear();
	CHECK_FALSE(single_desk(true, nomodel, nomodel));
}

TEST_CASE("driver figure: the seat is the sitting position, or the position without one") {
	cab_place standing{"cab.t3d", glm::vec3{-1.15f, 3.32f, -4.23f}, std::nullopt};
	REQUIRE(standing.seat().has_value());
	CHECK(standing.seat()->x == doctest::Approx(-1.15));
	CHECK(single_desk(true, standing, standing));

	// the same sitting position decides, whatever the standing ones (sm31_v1/411d.mmd)
	cab_place const sm31_cab1{"cab.t3d", glm::vec3{-1.15f, 3.32f, -4.23f}, glm::vec3{-1.13f, 3.21f, -4.77f}};
	cab_place const sm31_cab2{"cab.t3d", glm::vec3{1.15f, 3.32f, -4.23f}, glm::vec3{-1.13f, 3.21f, -4.77f}};
	CHECK(single_desk(true, sm31_cab1, sm31_cab2));

	// the cab 1 sits on the standing spot of the cab 2, which has its own seat elsewhere
	cab_place const cab2{"cab.t3d", glm::vec3{-1.15f, 3.32f, -4.23f}, glm::vec3{1.15f, 3.21f, -4.77f}};
	CHECK_FALSE(single_desk(true, standing, cab2));
}

TEST_CASE("driver figure: a cab without any driver position is not a single desk") {
	cab_place const noseat{"interior/401da-207.t3d", std::nullopt, std::nullopt};
	CHECK_FALSE(single_desk(true, desk_401da, noseat));
	CHECK_FALSE(single_desk(true, noseat, desk_401da));
	CHECK_FALSE(single_desk(true, noseat, noseat));
}

TEST_CASE("driver figure: the seats count as one within a centimetre") {
	auto cab2{desk_401da};
	cab2.sitting_position->x += 0.005f;
	CHECK(single_desk(true, desk_401da, cab2));
	cab2.sitting_position->x += 0.02f;
	CHECK_FALSE(single_desk(true, desk_401da, cab2));
}

TEST_CASE("driver figure: the figure of the occupied cab shows when it doesn't follow the direction") {
	for (auto const direction : {-1, 0, 1}) {
		CAPTURE(direction);
		CHECK(shown(1, direction, false) == 1);
		CHECK(shown(-1, direction, false) == -1);
	}
}

TEST_CASE("driver figure: following the direction, the figure faces the way the vehicle goes") {
	CHECK(shown(1, 1, true) == 1);
	CHECK(shown(1, -1, true) == -1); // the player reversing without changing the cab
	CHECK(shown(-1, -1, true) == -1); // the cab changed, driving on in the cab's own direction
	CHECK(shown(-1, 1, true) == 1);
}

TEST_CASE("driver figure: without a direction selected the figure of the occupied cab shows") {
	CHECK(shown(1, 0, true) == 1);
	CHECK(shown(-1, 0, true) == -1);
}

TEST_CASE("driver figure: no figure without an occupied cab") {
	for (auto const follows : {false, true}) {
		CAPTURE(follows);
		CHECK(shown(0, 1, follows) == 0);
		CHECK(shown(0, -1, follows) == 0);
		CHECK(shown(0, 0, follows) == 0);
	}
}
