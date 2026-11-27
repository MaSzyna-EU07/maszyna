#include "stdafx.h"
#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

#include "input/command.h"
#include "utilities/Timer.h"
#include "vehicle/Camera.h"
#include "vehicle/DynObj.h"

// the view matrix of the camera (TCamera::SetMatrix): the pitch (Angle.x) and the yaw (Angle.y) give the line of sight,
// the roll (Angle.z) turns the view about that line, wherever the camera looks. The roll comes from the sway of the cab
// (drivermode.cpp) or the angles of a camera entry of the scenery. Multiplied in last, it once acted about the world's z
// axis instead, and nodded the view up and down when looking out of a side window (623c4f82, put back in #414).
// vehicle/Camera.cpp isn't in the core library; the test binary compiles it itself (tests/unit/CMakeLists.txt)

// the parts of the engine TCamera::Update() reaches for, which no test here runs: the command queue and the clock
// (simulation::Commands itself is in test_script_screen_host.cpp)
bool command_queue::pop(command_data &, uint32_t const) { // NOSONAR signature from input/command.h
	return false;
}

double Timer::GetDeltaRenderTime() { // NOSONAR signature from utilities/Timer.h
	return 0.0;
}

namespace {

// the axes of the view in the world, from the view matrix
struct view_axes {
	glm::dvec3 eye;
	glm::dvec3 forward; // the line of sight
	glm::dvec3 up;
	glm::dvec3 right;
};

view_axes axes(TCamera const &Camera) {
	glm::dmat4 matrix{1.0};
	REQUIRE(Camera.SetMatrix(matrix));
	auto const toworld{glm::inverse(matrix)};
	return {
		glm::dvec3{toworld * glm::dvec4{0.0, 0.0, 0.0, 1.0}},
		glm::dvec3{toworld * glm::dvec4{0.0, 0.0, -1.0, 0.0}},
		glm::dvec3{toworld * glm::dvec4{0.0, 1.0, 0.0, 0.0}},
		glm::dvec3{toworld * glm::dvec4{1.0, 0.0, 0.0, 0.0}}};
}

void check_equal(glm::dvec3 const &Actual, glm::dvec3 const &Expected) {
	CAPTURE(Actual.x);
	CAPTURE(Actual.y);
	CAPTURE(Actual.z);
	CHECK(Actual.x == doctest::Approx(Expected.x).epsilon(1e-9));
	CHECK(Actual.y == doctest::Approx(Expected.y).epsilon(1e-9));
	CHECK(Actual.z == doctest::Approx(Expected.z).epsilon(1e-9));
}

// the free camera, at Position, turned by Angle (pitch, yaw, roll)
TCamera free_camera(glm::dvec3 const &Position, glm::vec3 const &Angle) {
	TCamera camera;
	camera.Init(Position, glm::dvec3{Angle}, nullptr);
	return camera;
}

struct turn {
	float pitch;
	float yaw;
};

// the camera looking ahead (-z), out of the left and the right side window, back, up and down at an angle
std::vector<turn> const turns{
	{0.f, 0.f},
	{0.f, static_cast<float>(std::numbers::pi / 2)},
	{0.3f, static_cast<float>(-std::numbers::pi / 2)},
	{-0.4f, static_cast<float>(std::numbers::pi)},
	{0.2f, 0.7f},
	{-0.6f, -2.1f},
};

std::vector<float> const rolls{0.1f, -0.2f, 0.5f};

} // namespace

TEST_CASE("camera: the pitch and the yaw give the line of sight, the roll leaves it as it is (#414)") {
	glm::dvec3 const position{120.0, 4.5, -300.0};
	for (auto const &turn : turns) {
		auto const pitch{static_cast<double>(turn.pitch)};
		auto const yaw{static_cast<double>(turn.yaw)};
		// the yaw from -z towards -x, the pitch up from the ground, as TCamera::RaLook() works them out
		glm::dvec3 const sight{-std::sin(yaw) * std::cos(pitch), std::sin(pitch), -std::cos(yaw) * std::cos(pitch)};
		for (auto const roll : {0.f, rolls[0], rolls[1], rolls[2]}) {
			CAPTURE(turn.pitch);
			CAPTURE(turn.yaw);
			CAPTURE(roll);
			auto const view{axes(free_camera(position, {turn.pitch, turn.yaw, roll}))};
			check_equal(view.eye, position);
			check_equal(view.forward, sight);
		}
	}
}

TEST_CASE("camera: the roll turns the view about the line of sight, wherever the camera looks (#414)") {
	for (auto const &turn : turns) {
		auto const level{axes(free_camera({}, {turn.pitch, turn.yaw, 0.f}))};
		for (auto const roll : rolls) {
			CAPTURE(turn.pitch);
			CAPTURE(turn.yaw);
			CAPTURE(roll);
			auto const rolled{axes(free_camera({}, {turn.pitch, turn.yaw, roll}))};
			auto const angle{static_cast<double>(roll)};
			// the up of the view leans towards the left by the roll, and stays square to the line of sight
			check_equal(rolled.up, std::cos(angle) * level.up - std::sin(angle) * level.right);
			check_equal(rolled.right, std::cos(angle) * level.right + std::sin(angle) * level.up);
			CHECK(glm::dot(rolled.up, rolled.forward) == doctest::Approx(0.0).epsilon(1e-9));
		}
	}
}

// the camera in the cab: the eye in the cab, looking at a point ahead of the vehicle (TCamera::LookAt), the head turned
// by the yaw and the pitch, the sway of the cab given as the roll. The camera needs no more of the vehicle than that it
// has one, so the vehicle is a zeroed block of its size, never used as an object (as the neighbours in vehicle.cpp)
static_assert(alignof(TDynamicObject) <= __STDCPP_DEFAULT_NEW_ALIGNMENT__);

TEST_CASE("camera: in the cab, the sway leaves the view out of a side window level (#414)") {
	std::vector<std::byte> vehicle(sizeof(TDynamicObject)); // zeroed

	glm::dvec3 const eye{10.0, 2.0, -5.0};
	// the vehicle heading north-east on level track
	auto const ahead{glm::normalize(glm::dvec3{1.0, 0.0, -1.0})};
	glm::dvec3 const left{ahead.z, 0.0, -ahead.x}; // the vehicle's left, flat

	for (auto const head : {-1, 1}) { // turned to the right, to the left
		for (auto const roll : {0.1f, -0.15f}) {
			CAPTURE(head);
			CAPTURE(roll);

			TCamera camera;
			camera.Init(eye, {}, reinterpret_cast<TDynamicObject *>(vehicle.data())); // NOSONAR see above
			camera.LookAt = eye + ahead;
			camera.vUp = {0.f, 1.f, 0.f};
			camera.Angle = {0.f, static_cast<float>(head * std::numbers::pi / 2), roll};

			auto const view{axes(camera)};
			check_equal(view.eye, eye);
			// out of the window to the left or the right, still level, whatever the sway (the yaw is a float, a hair
			// off a right angle)
			auto const yaw{static_cast<double>(camera.Angle.y)};
			check_equal(view.forward, std::sin(yaw) * left + std::cos(yaw) * ahead);
			CHECK(glm::dot(view.forward, static_cast<double>(head) * left) == doctest::Approx(1.0).epsilon(1e-6));
			// the sway tilts the view about that line by its angle
			CHECK(glm::dot(view.up, view.forward) == doctest::Approx(0.0).epsilon(1e-9));
			CHECK(view.up.y == doctest::Approx(std::cos(static_cast<double>(roll))).epsilon(1e-9));
		}
	}
}
