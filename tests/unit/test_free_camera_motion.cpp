#include "stdafx.h"
#include "doctest.h"
#include "rendering/common/freecameramotion.h"

#include <limits>

// the movement of the camera with no vehicle of its own, which the motion blur takes: the step since the frame before, held to
// half a metre, and none for the first frame, after a jump or after the camera sat in a vehicle
TEST_CASE("free camera motion: the first frame gives no movement, the next ones the step since the frame before")
{
	gfx::free_camera_motion motion;
	CHECK(motion.next_frame(glm::dvec3{100.0, 10.0, -50.0}) == glm::vec3{0.f});
	auto const step{motion.next_frame(glm::dvec3{100.25, 10.0, -50.0})};
	CHECK(step.x == doctest::Approx(0.25f));
	CHECK(step.y == 0.f);
	CHECK(step.z == 0.f);
	// a camera standing still
	CHECK(motion.next_frame(glm::dvec3{100.25, 10.0, -50.0}) == glm::vec3{0.f});
}

TEST_CASE("free camera motion: a fast camera is held to the longest movement, in its direction")
{
	gfx::free_camera_motion motion;
	motion.next_frame(glm::dvec3{0.0});
	auto const step{motion.next_frame(glm::dvec3{3.0, 0.0, -4.0})};
	CHECK(glm::length(step) == doctest::Approx(gfx::free_camera_motion::longest));
	CHECK(step.x == doctest::Approx(0.3f));
	CHECK(step.y == 0.f);
	CHECK(step.z == doctest::Approx(-0.4f));
}

TEST_CASE("free camera motion: a jump is no movement, and the frame after it moves from where the camera landed")
{
	gfx::free_camera_motion motion;
	motion.next_frame(glm::dvec3{0.0});
	CHECK(motion.next_frame(glm::dvec3{0.0, 0.0, gfx::free_camera_motion::jump}) == glm::vec3{0.f});
	CHECK(motion.next_frame(glm::dvec3{0.0, 0.0, gfx::free_camera_motion::jump + 0.1}).z == doctest::Approx(0.1f));
}

TEST_CASE("free camera motion: after a reset the camera starts anew")
{
	gfx::free_camera_motion motion;
	motion.next_frame(glm::dvec3{0.0});
	motion.reset();
	CHECK(motion.next_frame(glm::dvec3{0.2, 0.0, 0.0}) == glm::vec3{0.f});
	CHECK(motion.next_frame(glm::dvec3{0.4, 0.0, 0.0}).x == doctest::Approx(0.2f));
}

TEST_CASE("free camera motion: a position that isn't a number gives no movement")
{
	gfx::free_camera_motion motion;
	motion.next_frame(glm::dvec3{0.0});
	CHECK(motion.next_frame(glm::dvec3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}) == glm::vec3{0.f});
	// nor does the frame after it, which would move from that position
	CHECK(motion.next_frame(glm::dvec3{0.1, 0.0, 0.0}) == glm::vec3{0.f});
}
