#include "stdafx.h"
#include "doctest.h"

#include <cmath>
#include <numbers>

#include "vehicle/cameraviews.h"

// the outside views of the train that go with one of its vehicles (eu07/maszyna#551): orbit, shunting from above,
// beside a coupling, trackside. The camera follows from the place of the vehicle and the keys alone.

namespace {

using cameraviews::move_keys;

// a 20 m coach standing at Position, facing Front
cameraviews::vehicle_pose coach(glm::dvec3 const &Position = {100.0, 50.0, 200.0}, glm::dvec3 const &Front = {0.0, 0.0, -1.0}) {
	cameraviews::vehicle_pose pose;
	pose.position = Position;
	pose.front = Front;
	pose.length = 20.0;
	pose.width = 3.0;
	pose.height = 4.0;
	return pose;
}

double flat_distance(glm::dvec3 const &A, glm::dvec3 const &B) {
	return std::hypot(A.x - B.x, A.z - B.z);
}

// a key held, as TCamera::Velocity has it outside the cab
double constexpr key{1.0};

} // namespace

TEST_CASE("camera views: the orbit starts outside the vehicle, ahead of it on its left, above the rails") {
	auto const pose{coach()};
	auto const orbit{cameraviews::orbit_start(pose)};
	auto const place{cameraviews::orbit_placement(orbit, pose)};
	auto const centre{cameraviews::orbit_centre(pose)};

	CHECK(place.target == centre);
	CHECK(glm::distance(place.eye, centre) == doctest::Approx(orbit.radius));
	CHECK(orbit.radius >= cameraviews::orbit_min_radius(pose));
	auto const offset{place.eye - centre};
	CHECK(glm::dot(offset, pose.front) > 0.0); // ahead
	CHECK(glm::dot(offset, glm::dvec3{-1.0, 0.0, 0.0}) > 0.0); // on the left of a vehicle facing -z
	CHECK(place.eye.y > pose.position.y + pose.height);
}

TEST_CASE("camera views: the orbit radius stays between the body of the vehicle and its limit") {
	auto const pose{coach()};
	auto orbit{cameraviews::orbit_start(pose)};
	// the up arrow held for a minute brings the camera as close as the body lets it
	for (int frame = 0; frame < 600; ++frame) {
		orbit = cameraviews::orbit_moved(orbit, move_keys{0.0, 0.0, -key}, {}, 0.1, pose);
	}
	CHECK(orbit.radius == doctest::Approx(cameraviews::orbit_min_radius(pose)));
	// the diagonal of the body, so it's outside at any angle
	CHECK(orbit.radius > 0.5 * std::hypot(pose.length, pose.width));

	for (int frame = 0; frame < 600; ++frame) {
		orbit = cameraviews::orbit_moved(orbit, move_keys{0.0, 0.0, key}, {}, 0.1, pose);
	}
	CHECK(orbit.radius == doctest::Approx(cameraviews::orbit_max_radius));

	// a radius not set yet becomes the default one of the vehicle
	cameraviews::orbit unset;
	CHECK(cameraviews::orbit_clamped(unset, pose).radius == doctest::Approx(cameraviews::orbit_start(pose).radius));
}

TEST_CASE("camera views: the orbit keeps above the rails however low it goes") {
	auto const pose{coach()};
	for (auto const radius : {12.0, 25.0, 100.0, 250.0}) {
		CAPTURE(radius);
		auto orbit{cameraviews::orbit_start(pose)};
		orbit.radius = radius;
		for (int frame = 0; frame < 300; ++frame) {
			orbit = cameraviews::orbit_moved(orbit, move_keys{0.0, -key, 0.0}, {}, 0.1, pose);
		}
		auto const place{cameraviews::orbit_placement(orbit, pose)};
		CHECK(place.eye.y >= pose.position.y + cameraviews::rail_clearance - 1e-9);
		CHECK(orbit.elevation >= cameraviews::orbit_min_elevation - 1e-9);
	}
	// up to its highest it doesn't go over the top
	auto orbit{cameraviews::orbit_start(pose)};
	for (int frame = 0; frame < 300; ++frame) {
		orbit = cameraviews::orbit_moved(orbit, move_keys{0.0, key, 0.0}, {}, 0.1, pose);
	}
	CHECK(orbit.elevation == doctest::Approx(cameraviews::orbit_max_elevation));
	// the mouse moved up a lot at once is held the same way
	orbit = cameraviews::orbit_moved(orbit, {}, {10.0, 0.0}, 0.0, pose);
	CHECK(orbit.elevation == doctest::Approx(cameraviews::orbit_lowest_elevation(pose, orbit.radius)));
}

TEST_CASE("camera views: the left key takes the orbit clockwise around the vehicle, as a step to the left") {
	auto const pose{coach()};
	cameraviews::orbit orbit;
	orbit.radius = 30.0;
	orbit = cameraviews::orbit_clamped(orbit, pose);
	auto const ahead{cameraviews::orbit_placement(orbit, pose)};
	CHECK(glm::dot(ahead.eye - ahead.target, pose.front) > 0.0);

	// a little to the left from in front of the vehicle: the camera comes round to its right side
	auto const moved{cameraviews::orbit_moved(orbit, move_keys{-key, 0.0, 0.0}, {}, 0.5, pose)};
	CHECK(moved.azimuth < orbit.azimuth);
	auto const place{cameraviews::orbit_placement(moved, pose)};
	CHECK((place.eye - place.target).x > 0.0); // the right of a vehicle facing -z is +x

	// so does the mouse moved to the right (TCamera::Angle.y goes down)
	CHECK(cameraviews::orbit_moved(orbit, {}, {0.0, -0.2}, 0.0, pose).azimuth == doctest::Approx(orbit.azimuth - 0.2));

	// all the way round, the azimuth stays within half a turn either way
	auto round{orbit};
	for (int frame = 0; frame < 200; ++frame) {
		round = cameraviews::orbit_moved(round, move_keys{key, 0.0, 0.0}, {}, 0.1, pose);
		REQUIRE(std::abs(round.azimuth) <= std::numbers::pi + 1e-9);
	}
}

TEST_CASE("camera views: the orbit turns with the vehicle and moves with it") {
	auto const north{coach({0.0, 10.0, 0.0}, {0.0, 0.0, -1.0})};
	// the same vehicle further on, turned to face east (+x) on a slope
	auto const east{coach({500.0, 20.0, -300.0}, glm::normalize(glm::dvec3{1.0, 0.05, 0.0}))};
	auto const orbit{cameraviews::orbit_start(north)};

	auto const before{cameraviews::orbit_placement(orbit, north)};
	auto const after{cameraviews::orbit_placement(orbit, east)};
	auto const a{before.eye - before.target};
	auto const b{after.eye - after.target};
	CHECK(b.y == doctest::Approx(a.y));
	// a quarter turn clockwise from above: -z becomes +x, so the offset (x, z) becomes (-z, x)
	CHECK(b.x == doctest::Approx(-a.z));
	CHECK(b.z == doctest::Approx(a.x));
	CHECK(after.target == cameraviews::orbit_centre(east));
}

TEST_CASE("camera views: a longer vehicle picked keeps the orbit outside its body") {
	auto const shortvehicle{coach()};
	auto longvehicle{coach()};
	longvehicle.length = 40.0;
	cameraviews::orbit orbit;
	orbit.radius = cameraviews::orbit_min_radius(shortvehicle);
	orbit = cameraviews::orbit_clamped(orbit, longvehicle);
	CHECK(orbit.radius == doctest::Approx(cameraviews::orbit_min_radius(longvehicle)));
	CHECK(orbit.radius > 20.0);
}

TEST_CASE("camera views: the shunting view looks straight down, the way the user drives up the screen") {
	auto const pose{coach()};
	cameraviews::topdown view;
	view = cameraviews::topdown_clamped(view, pose, 100.0);
	CHECK(view.height >= cameraviews::topdown_min_height(pose));

	auto const place{cameraviews::topdown_placement(view, pose, 1)};
	CHECK(place.eye.x == doctest::Approx(pose.position.x));
	CHECK(place.eye.z == doctest::Approx(pose.position.z));
	CHECK(place.eye.y == doctest::Approx(pose.position.y + view.height));
	CHECK(place.target.x == doctest::Approx(place.eye.x));
	CHECK(place.target.z == doctest::Approx(place.eye.z));
	CHECK(place.up.z == doctest::Approx(-1.0)); // the front of the vehicle up the screen

	// driven from the rear cab, its rear is up the screen
	CHECK(cameraviews::topdown_placement(view, pose, -1).up.z == doctest::Approx(1.0));

	// the vehicle turned to face east: the screen turns with it
	auto const east{coach({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0})};
	CHECK(cameraviews::topdown_placement(view, east, 1).up.x == doctest::Approx(1.0));
}

TEST_CASE("camera views: the shunting view moves along and across the train and up, within its limits") {
	auto const pose{coach()};
	auto const along{60.0};
	auto view{cameraviews::topdown_clamped({}, pose, along)};
	auto const height{view.height};

	// the up arrow moves it up the screen, the way the user drives
	auto const forward{cameraviews::topdown_moved(view, move_keys{0.0, 0.0, -key}, 1.0, pose, along)};
	CHECK(forward.along > 0.0);
	auto const place{cameraviews::topdown_placement(forward, pose, -1)};
	CHECK(place.eye.z > pose.position.z); // driven from the rear cab, up the screen is +z
	// the right arrow to the right of the screen
	auto const right{cameraviews::topdown_moved(view, move_keys{key, 0.0, 0.0}, 1.0, pose, along)};
	CHECK(cameraviews::topdown_placement(right, pose, 1).eye.x > pose.position.x);
	CHECK(cameraviews::topdown_placement(right, pose, -1).eye.x < pose.position.x);

	for (int frame = 0; frame < 600; ++frame) {
		view = cameraviews::topdown_moved(view, move_keys{key, -key, -key}, 0.1, pose, along);
	}
	CHECK(view.along == doctest::Approx(along));
	CHECK(view.across == doctest::Approx(cameraviews::topdown_max_across));
	CHECK(view.height == doctest::Approx(cameraviews::topdown_min_height(pose)));
	CHECK(view.height < height);
	for (int frame = 0; frame < 600; ++frame) {
		view = cameraviews::topdown_moved(view, move_keys{0.0, key, 0.0}, 0.1, pose, along);
	}
	CHECK(view.height == doctest::Approx(cameraviews::topdown_max_height));
}

TEST_CASE("camera views: the way the user drives, seen from another vehicle of the train") {
	glm::dvec3 const north{0.0, 0.0, -1.0};
	glm::dvec3 const south{0.0, 0.0, 1.0};
	CHECK(cameraviews::driving_facing(north, north, 1) == 1);
	CHECK(cameraviews::driving_facing(north, north, -1) == -1);
	CHECK(cameraviews::driving_facing(north, north, 0) == 1); // no cab occupied counts as the front one
	// a vehicle turned round in the train
	CHECK(cameraviews::driving_facing(south, north, 1) == -1);
	CHECK(cameraviews::driving_facing(south, north, -1) == 1);
}

TEST_CASE("camera views: [ and ] step through the vehicles of the train and stop at its ends") {
	CHECK(cameraviews::step_index(2, 5, 1) == 3);
	CHECK(cameraviews::step_index(2, 5, -1) == 1);
	CHECK(cameraviews::step_index(0, 5, -1) == 0);
	CHECK(cameraviews::step_index(4, 5, 1) == 4);
	CHECK(cameraviews::step_index(1, 5, 10) == 4);
	CHECK(cameraviews::step_index(3, 1, 1) == 0); // a vehicle alone
	CHECK(cameraviews::step_index(3, 0, 1) == 0); // no train
	// a picked vehicle beyond the end of a train which got shorter
	CHECK(cameraviews::step_index(7, 3, 0) == 2);
}

TEST_CASE("camera views: the head and the tail of a train are its ends the way it goes") {
	std::vector<glm::dvec3> const positions{{0.0, 0.0, 0.0}, {0.0, 0.0, 20.0}, {0.0, 0.0, 40.0}};
	CHECK(cameraviews::train_ends(positions, {0.0, 0.0, -1.0}) == std::pair{0, 2});
	CHECK(cameraviews::train_ends(positions, {0.0, 0.0, 1.0}) == std::pair{2, 0});
	CHECK(cameraviews::train_ends({}, {0.0, 0.0, 1.0}) == std::pair{-1, -1});
}

TEST_CASE("camera views: the coupling view starts where another vehicle comes closer") {
	auto const count{5};
	CHECK(cameraviews::first_coupling(-1.0, -1.0, count) == 0);
	CHECK(cameraviews::first_coupling(30.0, -1.0, count) == 0);
	CHECK(cameraviews::first_coupling(-1.0, 30.0, count) == count);
	CHECK(cameraviews::first_coupling(50.0, 10.0, count) == count);
	CHECK(cameraviews::first_coupling(10.0, 50.0, count) == 0);
	// a vehicle too far away doesn't count
	CHECK(cameraviews::first_coupling(-1.0, cameraviews::coupler_watch_range + 1.0, count) == 0);
}

TEST_CASE("camera views: the coupling view stands beside the coupler and steps back for the vehicle coming closer") {
	cameraviews::coupler coupler;
	coupler.position = {0.0, 10.0, -10.0};
	coupler.outward = {0.0, 0.0, -1.0};
	coupler.width = 3.0;
	cameraviews::coupler_view const view;

	auto const alone{cameraviews::coupler_placement(view, coupler, nullptr)};
	CHECK(alone.eye.x == doctest::Approx(-(1.5 + view.distance))); // on the left of the outward way, -x
	CHECK(alone.eye.z > coupler.position.z); // a little back towards its vehicle
	CHECK(alone.eye.y == doctest::Approx(coupler.position.y + cameraviews::buffer_height + view.height));
	CHECK(alone.target.z < coupler.position.z); // looking out past the coupler

	// a vehicle 30 m away: the camera steps back from the track and looks between the two couplers
	glm::dvec3 const other{0.0, 10.0, -40.0};
	auto const coming{cameraviews::coupler_placement(view, coupler, &other)};
	CHECK(coming.eye.x < alone.eye.x);
	CHECK(coming.target.z == doctest::Approx(-25.0));
	CHECK(coming.target.y == doctest::Approx(10.0 + cameraviews::buffer_height));
	// closer, the camera comes back to the track with it
	glm::dvec3 const closer{0.0, 10.0, -12.0};
	CHECK(cameraviews::coupler_placement(view, coupler, &closer).eye.x == doctest::Approx(alone.eye.x));
	// far away, it stops stepping back
	glm::dvec3 const distant{0.0, 10.0, -500.0};
	CHECK(cameraviews::coupler_placement(view, coupler, &distant).eye.x == doctest::Approx(-(1.5 + cameraviews::coupler_max_distance)));
}

TEST_CASE("camera views: the keys of the coupling view") {
	cameraviews::coupler_view view;
	view = cameraviews::coupler_moved(view, move_keys{key, 0.0, 0.0}, 0.1);
	CHECK(view.side == -1.0);
	view = cameraviews::coupler_moved(view, {}, 0.1);
	CHECK(view.side == -1.0); // stays on its side without the key
	view = cameraviews::coupler_moved(view, move_keys{-key, 0.0, 0.0}, 0.1);
	CHECK(view.side == 1.0);
	for (int frame = 0; frame < 600; ++frame) {
		view = cameraviews::coupler_moved(view, move_keys{0.0, key, key}, 0.1);
	}
	CHECK(view.distance == doctest::Approx(cameraviews::coupler_max_distance));
	CHECK(view.height == doctest::Approx(cameraviews::coupler_max_height));
	for (int frame = 0; frame < 600; ++frame) {
		view = cameraviews::coupler_moved(view, move_keys{0.0, -key, -key}, 0.1);
	}
	CHECK(view.distance == doctest::Approx(cameraviews::coupler_min_distance));
	CHECK(view.height == doctest::Approx(cameraviews::coupler_min_height));
	// the lowest still above the rails
	CHECK(cameraviews::buffer_height + cameraviews::coupler_min_height > 0.0);
}

TEST_CASE("camera views: along an arc of the line, and its curvature from two vehicles") {
	glm::dvec3 const start{0.0, 5.0, 0.0};
	glm::dvec3 const north{0.0, 0.0, -1.0};
	auto const [straight, straightway]{cameraviews::along_arc(start, north, 0.0, 100.0)};
	CHECK(straight.z == doctest::Approx(-100.0));
	CHECK(straight.y == doctest::Approx(5.0));
	CHECK(straightway.z == doctest::Approx(-1.0));

	// a quarter of a circle of 500 m to the left: from going north to going west
	auto const radius{500.0};
	auto const [point, way]{cameraviews::along_arc(start, north, 1.0 / radius, 0.5 * std::numbers::pi * radius)};
	CHECK(point.x == doctest::Approx(-radius));
	CHECK(point.z == doctest::Approx(-radius));
	CHECK(way.x == doctest::Approx(-1.0));
	CHECK(way.z == doctest::Approx(0.0).epsilon(1e-9));

	// two vehicles 25 m apart on that circle, the one ahead turned to the left of the one behind
	auto const angle{25.0 / radius};
	glm::dvec3 const centre{-radius, 0.0, 0.0};
	glm::dvec3 const behind{0.0, 0.0, 0.0};
	glm::dvec3 const ahead{centre + glm::dvec3{radius * std::cos(angle), 0.0, -radius * std::sin(angle)}};
	glm::dvec3 const aheadway{-std::sin(angle), 0.0, -std::cos(angle)};
	auto const curvature{cameraviews::line_curvature(ahead, aheadway, behind, north)};
	CHECK(curvature == doctest::Approx(1.0 / radius).epsilon(0.01));
	// the other way round it turns to the right
	CHECK(cameraviews::line_curvature(ahead, glm::dvec3{-aheadway.x, 0.0, aheadway.z}, behind, north) < 0.0);
	// too close to tell
	CHECK(cameraviews::line_curvature(behind, aheadway, behind, north) == 0.0);
}

TEST_CASE("camera views: the railfan waits for the train ahead of it and moves on once it's gone by") {
	cameraviews::train_motion train;
	train.direction = {0.0, 0.0, -1.0};
	train.head = {0.0, 0.0, 0.0};
	train.tail = {0.0, 0.0, 100.0};
	train.speed = 20.0; // 72 km/h
	train.width = 3.0;

	cameraviews::trackside view;
	REQUIRE(cameraviews::trackside_expired(view, train));
	view = cameraviews::trackside_next(view, train);
	REQUIRE(view.placed);
	// ahead of the head by the 15 s it needs to get there, beside the line on the right, at the eye of a man
	CHECK(view.spot.z == doctest::Approx(-20.0 * cameraviews::trackside_lead_time));
	CHECK(view.spot.x == doctest::Approx(1.5 + cameraviews::trackside_distance));
	CHECK(view.spot.y == doctest::Approx(cameraviews::trackside_eye_height));

	// the train comes and goes by: the railfan stays until its tail is well past
	auto const spot{view.spot};
	for (int step = 0; step < 50; ++step) {
		auto const run{10.0 * step};
		auto passing{train};
		passing.head.z -= run;
		passing.tail.z -= run;
		auto const tailpast{spot.z - passing.tail.z};
		CAPTURE(run);
		CHECK(cameraviews::trackside_expired(view, passing) == (tailpast > cameraviews::trackside_passed));
	}
	// and then walks to the other side of the line, ahead again
	auto gone{train};
	gone.head.z -= 500.0;
	gone.tail.z -= 500.0;
	auto const next{cameraviews::trackside_next(view, gone)};
	CHECK(next.side == -view.side);
	CHECK(next.spot.x == doctest::Approx(-(1.5 + cameraviews::trackside_distance)));
	CHECK(next.spot.z < gone.head.z);
	CHECK(false == cameraviews::trackside_expired(next, gone));
}

TEST_CASE("camera views: the railfan's spot ahead keeps to the line in a curve, and nearby when the train stands") {
	cameraviews::train_motion train;
	train.direction = {0.0, 0.0, -1.0};
	train.speed = 40.0;
	auto const radius{400.0};
	train.curvature = 1.0 / radius;
	cameraviews::trackside const view;
	auto const next{cameraviews::trackside_next(view, train)};
	// the centre of the curve is on the left of the head, the railfan on the right of the line: outside the circle
	glm::dvec3 const centre{-radius, 0.0, 0.0};
	CHECK(flat_distance(next.spot, centre) == doctest::Approx(radius + 0.5 * train.width + cameraviews::trackside_distance));
	// no further round than a quarter of the circle
	CHECK(next.spot.z >= -radius - 1e-6);

	// a train standing still gets a railfan at the shortest lead
	auto standing{train};
	standing.speed = 0.0;
	standing.curvature = 0.0;
	auto const nearby{cameraviews::trackside_next(view, standing)};
	CHECK(nearby.spot.z == doctest::Approx(-cameraviews::trackside_min_lead));
	// and as long as it stands, the railfan stays
	CHECK(false == cameraviews::trackside_expired(nearby, standing));
	// a train gone far away (moved by the user, or a long way back) gets a new railfan
	auto away{standing};
	away.head = {0.0, 0.0, 2000.0};
	away.tail = {0.0, 0.0, 2100.0};
	CHECK(cameraviews::trackside_expired(nearby, away));
}
