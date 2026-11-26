#include "stdafx.h"
#include "doctest.h"

#include "vehicle/brakereleaser.h"

// when the AI works the brake releaser of its locomotive (TController::control_releaser). in stage 3 of uncoupling
// the locomotive pushes towards the cut it leaves, until the screw coupler slackens enough to be let go
// (TController::UpdateDisconnect). the consist brakes applied in stage 2 hold the locomotive too, and only the
// releaser lets it go. the releaser was worked only while the driver wanted to speed up (AccDesired above
// -0.05), so a signal at stop the cut stood across after the direction change, which caps the speed at 0, left the
// locomotive braked and the uncoupling went on for good (SM42 at Debica on L053).

namespace {

// a locomotive standing in stage 3 of uncoupling, braked by the consist's brake application of stage 2
brake_releaser::conditions compressing_buffers(double const Acceleration) {
	brake_releaser::conditions state;
	state.acceleration_desired = Acceleration;
	state.pipe_pressure = 3.8;
	state.cylinder_pressure = 1.5;
	state.control_pressure = 5.0;
	state.velocity = 0.0;
	state.pressing = true;
	state.compressing_buffers = true;
	return state;
}

// a locomotive standing braked at the head of a train with a charged brake pipe, about to start off
brake_releaser::conditions standing_braked(double const Acceleration) {
	brake_releaser::conditions state;
	state.acceleration_desired = Acceleration;
	state.pipe_pressure = 5.0;
	state.cylinder_pressure = 1.0;
	state.control_pressure = 5.0;
	state.velocity = 0.0;
	return state;
}

} // namespace

TEST_CASE("brake releaser: stage 3 of uncoupling releases the locomotive whatever the driver's acceleration") {
	for (auto const acceleration : {0.3, 0.0, -0.05, -0.2, -0.9}) {
		CAPTURE(acceleration);
		auto const wanted{brake_releaser::decide(compressing_buffers(acceleration))};
		CHECK(wanted.actuate);
		CHECK_FALSE(wanted.fill_pipe);
	}
}

TEST_CASE("brake releaser: stage 3 of uncoupling leaves a released locomotive alone") {
	for (auto const acceleration : {0.3, -0.9}) {
		CAPTURE(acceleration);
		auto state{compressing_buffers(acceleration)};
		state.cylinder_pressure = 0.05;
		CHECK_FALSE(brake_releaser::decide(state).actuate);
	}
}

TEST_CASE("brake releaser: pressing outside stage 3 still needs the driver to want to speed up") {
	// the direction changes of stages 2 and 4, and coupling, are as before: the releaser goes on only with
	// AccDesired above -0.05
	auto state{compressing_buffers(0.3)};
	state.compressing_buffers = false;
	CHECK(brake_releaser::decide(state).actuate);
	state.acceleration_desired = -0.05;
	CHECK_FALSE(brake_releaser::decide(state).actuate);
	state.acceleration_desired = -0.9;
	CHECK_FALSE(brake_releaser::decide(state).actuate);
}

TEST_CASE("brake releaser: starting off releases a braked locomotive below 1 km/h") {
	CHECK(brake_releaser::decide(standing_braked(0.3)).actuate);
	CHECK_FALSE(brake_releaser::decide(standing_braked(-0.05)).actuate);

	auto moving{standing_braked(0.3)};
	moving.velocity = 1.0;
	CHECK_FALSE(brake_releaser::decide(moving).actuate);

	auto lightly{standing_braked(0.3)};
	lightly.cylinder_pressure = 0.4;
	CHECK_FALSE(brake_releaser::decide(lightly).actuate);

	// the control reservoir not charged: releasing would vent it
	auto uncharged{standing_braked(0.3)};
	uncharged.control_pressure = 4.9;
	CHECK_FALSE(brake_releaser::decide(uncharged).actuate);
	// unless the train brake is held on purpose to start uphill
	uncharged.uphill_start = true;
	CHECK(brake_releaser::decide(uncharged).actuate);
}

TEST_CASE("brake releaser: an empty brake pipe is filled up when the driver wants to speed up") {
	auto state{standing_braked(0.3)};
	state.pipe_pressure = 2.9;
	state.cylinder_pressure = 0.0;
	auto const wanted{brake_releaser::decide(state)};
	CHECK(wanted.actuate);
	CHECK(wanted.fill_pipe);

	state.acceleration_desired = -0.2;
	auto const braking{brake_releaser::decide(state)};
	CHECK_FALSE(braking.actuate);
	CHECK_FALSE(braking.fill_pipe);
}

TEST_CASE("brake releaser: an overcharged brake pipe turns the releaser off, in stage 3 of uncoupling too") {
	auto starting{standing_braked(0.3)};
	starting.pipe_pressure = 5.3;
	CHECK_FALSE(brake_releaser::decide(starting).actuate);

	auto uncoupling{compressing_buffers(-0.9)};
	uncoupling.pipe_pressure = 5.3;
	CHECK_FALSE(brake_releaser::decide(uncoupling).actuate);
}

TEST_CASE("brake releaser: a train charging its brakes standing fills an empty pipe without wanting to speed up") {
	// #444: a train prepared standing (bBrakeChargeStart) is unlocked below the lock of the driver's brake valve as it
	// stands, not only once the driver wants to go
	auto state{standing_braked(-0.9)};
	state.charges_standing = true;
	state.pipe_pressure = 2.0;
	auto const wanted{brake_releaser::decide(state)};
	CHECK(wanted.fill_pipe);
	CHECK(wanted.actuate);
}

TEST_CASE("brake releaser: a train charging its brakes standing with a charged pipe stays braked") {
	// only the filling of the pipe is done standing; letting the locomotive's own brake go waits for the start
	auto state{standing_braked(-0.9)};
	state.charges_standing = true;
	auto const wanted{brake_releaser::decide(state)};
	CHECK_FALSE(wanted.fill_pipe);
	CHECK_FALSE(wanted.actuate);
}
