#include "stdafx.h"
#include "doctest.h"

#include "vehicle/doorwings.h"

// the poses of the animated door wings (eu07/maszyna#512): a wing of the main model and its namesake in the low
// poly interior get the same pose, worked out once from the door position of the mover

TEST_CASE("door wings: only the door animations move wings") {
	CHECK(door_wings::is_wing(0x010)); // TAnim::TypeSet for the doors
	CHECK_FALSE(door_wings::is_wing(0x000)); // axles
	CHECK_FALSE(door_wings::is_wing(0x055)); // pantographs, five submodels
	CHECK_FALSE(door_wings::is_wing(0x070)); // doorsteps
	CHECK_FALSE(door_wings::is_wing(0x080)); // mirrors
	CHECK(door_wings::is_wing(0x210)); // the parameter count in the upper bits doesn't matter
}

TEST_CASE("door wings: odd wings are on the left, even ones on the right") {
	CHECK_FALSE(door_wings::on_right_side(1));
	CHECK(door_wings::on_right_side(2));
	CHECK_FALSE(door_wings::on_right_side(3));
	CHECK(door_wings::on_right_side(4));
}

TEST_CASE("door wings: a sliding wing shifts along the vehicle by the door position") {
	CHECK(door_wings::slide(0.f) == glm::vec3(0.f, 0.f, 0.f));
	CHECK(door_wings::slide(0.65f) == glm::vec3(0.f, 0.f, 0.65f));
}

TEST_CASE("door wings: a plug wing goes out first, then slides") {
	auto const rangeout{0.1f};
	// closed
	CHECK(door_wings::plug(0.f, rangeout) == glm::vec3(0.f, 0.f, 0.f));
	// still going out, and not sliding yet
	auto const out{door_wings::plug(0.02f, rangeout)};
	CHECK(out.x == doctest::Approx(0.04f));
	CHECK(out.z == 0.f);
	// out as far as it goes, sliding by what the position has beyond half of the way out
	auto const open{door_wings::plug(0.7f, rangeout)};
	CHECK(open.x == rangeout);
	CHECK(open.y == 0.f);
	CHECK(open.z == doctest::Approx(0.65f));
}

TEST_CASE("door wings: a folding wing turns one way and its larger wing twice as far the other way") {
	auto const angles{door_wings::fold(40.f)};
	CHECK(angles.smaller == 40.f);
	CHECK(angles.larger == -80.f);
	CHECK(angles.step == 40.f);
	auto const closed{door_wings::fold(0.f)};
	CHECK(closed.smaller == 0.f);
	CHECK(closed.larger == 0.f);
	CHECK(closed.step == 0.f);
}
