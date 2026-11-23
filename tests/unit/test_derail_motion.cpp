#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
#include "vehicle/derailmotion.h"
#include "McZapkie/derailment.h"
#include "McZapkie/MOVER.h"
#include "utilities/Globals.h"

#include <cmath>
#include <format>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

// a vehicle off the rails moving on the ground by itself (physics.derailment new): vehicle/derailmotion.h, and what
// the vehicle of TMoverParameters makes of it

using test::definition;
using test::vehicle;
namespace dm = derail_motion;
namespace dr = derailment;

namespace {

constexpr double kmh{1.0 / 3.6};
constexpr double degree{std::numbers::pi / 180.0};

// the given model of derailment for the test, restored after it
class model_setting {
public:
	explicit model_setting(global_settings::derailment_model Model) : m_previous{Global.derailment} { Global.derailment = Model; }
	~model_setting() { Global.derailment = m_previous; }
	model_setting(model_setting const &) = delete;
	model_setting &operator=(model_setting const &) = delete;

private:
	global_settings::derailment_model m_previous;
};

// EU07 (4E of data/fiz): its dimensions and the centre of gravity of its body
dm::vehicle_shape eu07() {
	dm::vehicle_shape shape;
	shape.length = 15.915;
	shape.width = 2.903;
	shape.height = 4.348;
	shape.centre_of_gravity = dr::default_centre_of_gravity(4.348, dr::vehicle_class::powered);
	shape.contact_spacing = dr::contact_spacing(1.435);
	shape.base = 8.0;
	return shape;
}

// the lateral acceleration [m/s2] the wheels of EU07 lift at: the wheel unloading of derailment.h reaches 1
double const eu07_tipping{dr::overturning_acceleration(eu07().centre_of_gravity, dr::contact_spacing(1.435))};

// the track along +z at the height 0, its left (+x) the outside of the curve
dm::start leaving(double const Speed, dm::cause const Reason, int const Outward = 0, double const Acceleration = 0.0) {
	dm::start start;
	start.position = glm::dvec3{0.0};
	start.front = glm::dvec3{0.0, 0.0, 1.0};
	start.line = start.front;
	start.speed = Speed;
	start.reason = Reason;
	start.outward = Outward;
	start.acceleration = Acceleration;
	return start;
}

// the ground across the track, along +z: the top of the ballast 0.2 m under the rails out to 3 m each side, then the
// Slope (rise per metre outwards, negative down) to the Depth under the top (above it, rising). as a scenery has it,
// a level ground lies under it all, at the lowest of it: what is over the ceiling asked for isn't found, that is
class profile final : public dm::ground {
public:
	profile(double const Slope, double const Depth, bool const Bothsides = false) : m_slope{Slope}, m_depth{Depth}, m_bothsides{Bothsides} {}
	std::optional<double> height(double const X, double, double const Ceiling) const override {
		constexpr double top{-0.2};
		constexpr double edge{3.0};
		auto const across{m_bothsides ? std::abs(X) : X};
		auto const floor{std::min(top, top + std::clamp(m_slope * 1e6, -std::abs(m_depth), std::abs(m_depth)))};
		auto surface{top};
		if (across > edge) {
			surface += std::clamp(m_slope * (across - edge), -std::abs(m_depth), std::abs(m_depth));
		}
		return surface <= Ceiling ? surface : floor;
	}

private:
	double m_slope;
	double m_depth;
	bool m_bothsides;
};

struct outcome {
	double time{0.0};
	double travelled{0.0};
	double along{0.0};
	double across{0.0};
	double tipping{0.0};
	double roll{0.0};
	double height{0.0};
	bool lying{false};
	double lying_at{-1.0}; // the time it came to lie on its side, -1 never
};

// the vehicle slides as TMoverParameters drives it: its speed along the heading falls with the friction of the motion
// and the gravity of the grade under it, the distance of each step goes to the motion
outcome slide(dm::start const &Start, dm::ground const &Ground, double const Dt = 0.01, dm::vehicle_shape const &Shape = eu07(), double const Seconds = 60.0) {
	dm::body body;
	body.begin(Start, Shape, Ground);
	auto speed{Start.speed};
	outcome result;
	while (result.time < Seconds) {
		auto const direction{speed >= 0.0 ? 1.0 : -1.0};
		auto const deceleration{dm::gravity * (body.friction() * direction + body.grade())};
		auto next{speed - deceleration * Dt};
		if (next * speed <= 0.0 && std::abs(body.grade()) <= body.friction()) {
			next = 0.0;
		}
		auto const reversals{body.reversals()};
		body.step(Dt, 0.5 * (speed + next) * Dt, next, Ground);
		speed = (body.reversals() - reversals) % 2 != 0 ? -next : next;
		result.time += Dt;
		if (body.lying() && result.lying_at < 0.0) {
			result.lying_at = result.time;
		}
		if (body.at_rest()) {
			break;
		}
	}
	result.travelled = body.travelled();
	result.along = body.along();
	result.across = body.across();
	result.tipping = body.tipping();
	result.roll = body.roll();
	result.height = body.current().position.y;
	result.lying = body.lying();
	return result;
}

dm::flat_ground const ballast{-0.2};

} // namespace

TEST_CASE("derail motion: the distance it slides goes with the square of the speed and against the friction") {
	auto const fast{dm::stopping_distance(133.0 * kmh, dr::off_rails_friction)};
	auto const slow{dm::stopping_distance(40.0 * kmh, dr::off_rails_friction)};
	REQUIRE(fast.has_value());
	REQUIRE(slow.has_value());
	CHECK(*fast == doctest::Approx(232.0).epsilon(0.01));
	CHECK(*slow == doctest::Approx(21.0).epsilon(0.01));
	CHECK(*fast / *slow == doctest::Approx(133.0 * 133.0 / (40.0 * 40.0)).epsilon(1e-9));
	// on its side
	CHECK(*dm::stopping_distance(133.0 * kmh, dr::overturned_friction) == doctest::Approx(*fast * 0.3 / 0.5).epsilon(1e-9));
	// uphill sooner, downhill further, down a slope steeper than the friction it doesn't stop
	CHECK(*dm::stopping_distance(40.0 * kmh, 0.3, 0.05) < *slow);
	CHECK(*dm::stopping_distance(40.0 * kmh, 0.3, -0.05) > *slow);
	CHECK_FALSE(dm::stopping_distance(40.0 * kmh, 0.3, -0.4).has_value());

	SUBCASE("the vehicle off the end of the track slides as far, straight on, upright") {
		for (auto const speed : {133.0, 40.0}) {
			CAPTURE(speed);
			auto const result{slide(leaving(speed * kmh, dm::cause::end_of_track), ballast)};
			MESSAGE(std::format("off the end of the track at {:.0f} km/h: {:.1f} m in {:.1f} s", speed, result.travelled, result.time));
			CHECK(result.travelled == doctest::Approx(*dm::stopping_distance(speed * kmh, dr::off_rails_friction)).epsilon(0.02));
			CHECK(result.along == doctest::Approx(result.travelled).epsilon(1e-6));
			CHECK(std::abs(result.across) < 1e-6);
			CHECK(result.tipping == 0.0);
			// on the top of the ballast, not on the rails nor under the ground
			CHECK(result.height == doctest::Approx(-0.2));
		}
	}
	SUBCASE("the friction along it rises from the one on its wheels to the one on its side") {
		CHECK(dm::sliding_friction(0.0, false) == dr::off_rails_friction);
		CHECK(dm::sliding_friction(dm::body::half_turn, false) == dr::overturned_friction);
		CHECK(dm::sliding_friction(0.5 * dm::body::half_turn, false) == doctest::Approx(0.5 * (dr::off_rails_friction + dr::overturned_friction)));
		CHECK(dm::sliding_friction(0.0, true) == dm::guided_friction);
	}
}

TEST_CASE("derail motion: the way it leaves the track by what derailed it") {
	SUBCASE("off the end of the track and in a collision: straight on") {
		CHECK(dm::departure_angle(dm::cause::end_of_track, 30.0, 10.0) == 0.0);
		CHECK(dm::departure_angle(dm::cause::collision, 30.0, 10.0) == 0.0);
		auto const result{slide(leaving(100.0 * kmh, dm::cause::collision, 1, 5.0), ballast)};
		CHECK(std::abs(result.across) < 1e-6);
	}
	SUBCASE("the flange climbing the rail: towards the rail it climbed, out of the curve, upright") {
		for (auto const outward : {1, -1}) {
			CAPTURE(outward);
			auto const result{slide(leaving(100.0 * kmh, dm::cause::off_rails, outward, 3.0), ballast)};
			MESSAGE(std::format("climbed out of the curve to the {}: {:.1f} m across in {:.1f} m", outward > 0 ? "left" : "right", result.across, result.travelled));
			CHECK(result.across * outward > 1.0);
			CHECK(result.tipping == 0.0);
			CHECK_FALSE(result.lying);
		}
		// the faster the speed against the sideways one, the smaller the angle
		CHECK(dm::departure_angle(dm::cause::off_rails, 40.0, 3.0) < dm::departure_angle(dm::cause::off_rails, 10.0, 3.0));
		CHECK(dm::departure_angle(dm::cause::off_rails, 0.1, 1000.0) == dm::most_departure);
	}
	SUBCASE("backwards, out of the curve as well") {
		auto const result{slide(leaving(-100.0 * kmh, dm::cause::off_rails, 1, 3.0), ballast)};
		CHECK(result.along < -10.0);
		CHECK(result.across > 1.0);
	}
	SUBCASE("overturned: out of the curve, falling on its side there") {
		for (auto const outward : {1, -1}) {
			CAPTURE(outward);
			auto const result{slide(leaving(120.0 * kmh, dm::cause::overturned, outward, 2.0 * eu07_tipping), ballast)};
			MESSAGE(std::format("overturned to the {}: {:.1f} m across in {:.1f} m, on its side after {:.2f} s", outward > 0 ? "left" : "right", result.across, result.travelled, result.lying_at));
			CHECK(result.across * outward > 1.0);
			CHECK(result.lying);
			CHECK(result.tipping * outward == doctest::Approx(dm::body::half_turn));
			CHECK(result.roll * outward == doctest::Approx(dm::body::half_turn));
			// lying on its side on the top of the ballast, the origin of its model is half the width of its body up
			CHECK(result.height == doctest::Approx(-0.2 + 0.5 * eu07().width));
		}
	}
	SUBCASE("pulled off by a coupling: to the side it is pulled to") {
		auto const result{slide(leaving(50.0 * kmh, dm::cause::pulled_off, -1, 2.0), ballast)};
		CHECK(result.across < -0.1);
	}
}

TEST_CASE("derail motion: tipping over goes as far as the overturning went over its limit") {
	// the curve keeps pushing it out while its outer wheels are on the rail: the further over the limit, the sooner it lies
	auto const barely{slide(leaving(120.0 * kmh, dm::cause::overturned, 1, 1.2 * eu07_tipping), ballast)};
	auto const well{slide(leaving(120.0 * kmh, dm::cause::overturned, 1, 2.0 * eu07_tipping), ballast)};
	auto const hard{slide(leaving(120.0 * kmh, dm::cause::overturned, 1, 5.0 * eu07_tipping), ballast)};
	MESSAGE(std::format("on its side after {:.2f} / {:.2f} / {:.2f} s at a wheel unloading of 1.2 / 2 / 5", barely.lying_at, well.lying_at, hard.lying_at));
	REQUIRE(barely.lying);
	REQUIRE(well.lying);
	REQUIRE(hard.lying);
	CHECK(barely.lying_at > well.lying_at);
	CHECK(well.lying_at > hard.lying_at);
	// it takes its time to fall over, it doesn't jump onto its side
	CHECK(hard.lying_at > 0.3);

	SUBCASE("a body on the rails stays upright, on a level ground it isn't tipped by anything") {
		auto const upright{slide(leaving(120.0 * kmh, dm::cause::off_rails, 1, 2.0 * eu07_tipping), ballast)};
		CHECK(upright.tipping == 0.0);
	}
	SUBCASE("slowing down below the limit, the curve doesn't tip it any more") {
		dm::body body;
		body.begin(leaving(30.0 * kmh, dm::cause::overturned, 1, 1.05 * eu07_tipping), eu07(), ballast);
		// stopped at once: the curve pushes it out no more, it falls back on its wheels
		body.step(0.01, 0.0, 0.0, ballast);
		for (int step = 0; step < 300; ++step) {
			body.step(0.01, 0.0, 0.0, ballast);
		}
		CHECK_FALSE(body.guided());
		CHECK(body.tipping() == 0.0);
	}
}

TEST_CASE("derail motion: the ground under it") {
	SUBCASE("off the edge of an embankment it slides down the slope and falls on its side towards the bottom") {
		// 1:1.5 down 4 m to the left, the vehicle leaving the track 4 degrees to the left at 60 km/h
		profile const embankment{-1.0 / 1.5, 4.0};
		auto start{leaving(60.0 * kmh, dm::cause::off_rails)};
		start.front = glm::dvec3{std::sin(4.0 * degree), 0.0, std::cos(4.0 * degree)};
		auto const result{slide(start, embankment)};
		MESSAGE(std::format("down the embankment: {:.1f} m across, {:.1f} m down, tipped {:.0f} degrees, after {:.1f} m", result.across, -0.2 - result.height, result.tipping / degree, result.travelled));
		CHECK(result.across > 4.0);
		// lying on the slope, its body half its width up off the ground
		CHECK(result.height < -1.5);
		CHECK(result.lying);
		CHECK(result.tipping > 0.0);
	}
	SUBCASE("on a gentle slope it stays upright, leaning with the ground") {
		profile const gentle{-1.0 / 6.0, 4.0};
		auto start{leaving(80.0 * kmh, dm::cause::off_rails)};
		start.front = glm::dvec3{std::sin(6.0 * degree), 0.0, std::cos(6.0 * degree)};
		auto const result{slide(start, gentle)};
		CHECK_FALSE(result.lying);
		CHECK(result.tipping == 0.0);
		// leaning left, down the slope
		CHECK(result.roll == doctest::Approx(std::atan(1.0 / 6.0)).epsilon(0.05));
	}
	SUBCASE("a lower centre of gravity tips over on a steeper slope only") {
		// a 1:2.2 slope, 24 degrees: the steepest one EU07 stands on is 1:2.8 (20 degrees), a flat wagon tips past 1:1.7
		profile const slope{-1.0 / 2.2, 6.0};
		auto start{leaving(50.0 * kmh, dm::cause::off_rails)};
		start.front = glm::dvec3{std::sin(10.0 * degree), 0.0, std::cos(10.0 * degree)};
		auto low{eu07()};
		low.centre_of_gravity = 0.9;
		low.height = 2.0;
		CHECK(slide(start, slope).lying);
		CHECK_FALSE(slide(start, slope, 0.01, low).lying);
	}
	SUBCASE("in a cutting it climbs the bank, stops sooner and leans (here falls) back towards the track") {
		profile const cutting{1.0 / 1.5, 4.0};
		auto start{leaving(60.0 * kmh, dm::cause::off_rails)};
		start.front = glm::dvec3{std::sin(4.0 * degree), 0.0, std::cos(4.0 * degree)};
		auto const level{slide(start, ballast)};
		auto const result{slide(start, cutting)};
		MESSAGE(std::format("into a cutting: {:.1f} m against {:.1f} m on a level ground, roll {:.0f} degrees", result.travelled, level.travelled, result.roll / degree));
		CHECK(result.travelled < level.travelled);
		CHECK(result.height > -0.2);
		CHECK(result.roll < 0.0);
	}
	SUBCASE("it doesn't sink into the ground, it falls off a step") {
		// off a wall 3 m high
		profile const wall{-100.0, 3.0};
		auto start{leaving(40.0 * kmh, dm::cause::off_rails)};
		start.front = glm::dvec3{std::sin(20.0 * degree), 0.0, std::cos(20.0 * degree)};
		dm::body body;
		body.begin(start, eu07(), wall);
		auto lowest{0.0};
		auto speed{start.speed};
		for (int index = 0; index < 1000; ++index) {
			auto const next{std::max(0.0, speed - dm::gravity * body.friction() * 0.01)};
			body.step(0.01, 0.5 * (speed + next) * 0.01, next, wall);
			speed = next;
			auto const &position{body.current().position};
			auto const ground{*wall.height(position.x, position.z, 1e9)};
			CHECK(position.y >= ground - 1e-9);
			lowest = std::min(lowest, position.y);
		}
		CHECK(lowest < -3.0);
	}
}

TEST_CASE("derail motion: the same with steps of 0.01, 0.03 and 0.05 s") {
	profile const embankment{-1.0 / 1.5, 4.0};
	auto down{leaving(60.0 * kmh, dm::cause::off_rails)};
	down.front = glm::dvec3{std::sin(4.0 * degree), 0.0, std::cos(4.0 * degree)};
	auto const overturning{leaving(120.0 * kmh, dm::cause::overturned, -1, 2.0 * eu07_tipping)};
	auto const climbing{leaving(100.0 * kmh, dm::cause::off_rails, 1, 3.0)};
	auto const fine{slide(down, embankment, 0.01)};
	auto const finetip{slide(overturning, ballast, 0.01)};
	auto const fineclimb{slide(climbing, ballast, 0.01)};
	for (auto const dt : {0.03, 0.05}) {
		CAPTURE(dt);
		auto const coarse{slide(down, embankment, dt)};
		CHECK(coarse.travelled == doctest::Approx(fine.travelled).epsilon(0.02));
		CHECK(coarse.across == doctest::Approx(fine.across).epsilon(0.05));
		CHECK(coarse.lying == fine.lying);
		auto const coarsetip{slide(overturning, ballast, dt)};
		CHECK(coarsetip.travelled == doctest::Approx(finetip.travelled).epsilon(0.02));
		CHECK(coarsetip.across == doctest::Approx(finetip.across).epsilon(0.02));
		CHECK(coarsetip.lying_at == doctest::Approx(finetip.lying_at).epsilon(0.1));
		auto const coarseclimb{slide(climbing, ballast, dt)};
		CHECK(coarseclimb.across == doctest::Approx(fineclimb.across).epsilon(0.02));
	}
}

TEST_CASE("derail motion: the ground of the scenery") {
	SUBCASE("the triangles of the terrain") {
		std::vector<dm::mesh_ground::triangle> triangles;
		// a square 10 m wide at 2 m rising to 4 m along x, and a roof over half of it at 10 m
		triangles.push_back({glm::dvec3{0.0, 2.0, 0.0}, glm::dvec3{10.0, 4.0, 0.0}, glm::dvec3{10.0, 4.0, 10.0}});
		triangles.push_back({glm::dvec3{0.0, 2.0, 0.0}, glm::dvec3{10.0, 4.0, 10.0}, glm::dvec3{0.0, 2.0, 10.0}});
		triangles.push_back({glm::dvec3{0.0, 10.0, 0.0}, glm::dvec3{5.0, 10.0, 0.0}, glm::dvec3{0.0, 10.0, 10.0}});
		dm::mesh_ground const terrain{triangles, glm::dvec2{-5.0, -5.0}, glm::dvec2{15.0, 15.0}, 2.0};
		CHECK(terrain.size() == 3);
		CHECK(*terrain.height(5.0, 5.0, 8.0) == doctest::Approx(3.0));
		CHECK(*terrain.height(7.5, 2.5, 8.0) == doctest::Approx(3.5));
		// under the roof the ground, over it the roof
		CHECK(*terrain.height(1.0, 1.0, 8.0) == doctest::Approx(2.2));
		CHECK(*terrain.height(1.0, 1.0, 12.0) == doctest::Approx(10.0));
		CHECK_FALSE(terrain.height(12.0, 5.0, 8.0).has_value());
		CHECK_FALSE(terrain.height(50.0, 5.0, 8.0).has_value());
		CHECK(terrain.covers(14.0, -4.0));
		CHECK_FALSE(terrain.covers(16.0, 0.0));
	}
	SUBCASE("the bed of the track over the terrain, level ground without one") {
		dm::formation bed;
		bed.point = glm::dvec3{0.0, 10.0, 0.0};
		bed.direction = glm::dvec3{0.0, 0.0, 1.0};
		bed.half_width = 2.0;
		CHECK(bed.height(1.5, 100.0) == doctest::Approx(9.8));
		CHECK(bed.height(-2.0, -50.0) == doctest::Approx(9.8));
		CHECK(bed.height(3.5, 0.0) == doctest::Approx(9.8 - 1.0));
		dm::flat_ground const low{7.0};
		dm::site const embankment{&low, bed};
		CHECK(*embankment.height(0.0, 0.0, 12.0) == doctest::Approx(9.8));
		CHECK(*embankment.height(4.0, 0.0, 12.0) == doctest::Approx(9.8 - 2.0 / 1.5));
		CHECK(*embankment.height(20.0, 0.0, 12.0) == doctest::Approx(7.0));
		dm::site const bare{nullptr, bed};
		CHECK(*bare.height(20.0, 0.0, 12.0) == doctest::Approx(9.8));
	}
}

TEST_CASE("derail motion: a coupling pulling sideways") {
	// EU07, 80 t on 4 axles: a wheel carries 98 kN
	CHECK(dm::pull_ratio(300e3, 0.0, 80e3, 4) == 0.0);
	CHECK(dm::pull_ratio(300e3, 30.0 * degree, 80e3, 4) == doctest::Approx(0.5 * 150e3 / (80e3 * dm::gravity / 8.0)));
	CHECK(dm::pull_ratio(-300e3, -30.0 * degree, 80e3, 4) == dm::pull_ratio(300e3, 30.0 * degree, 80e3, 4));
	// an empty wagon of 20 t goes over the climbing limit with a force the locomotive takes
	CHECK(dm::pull_ratio(300e3, 30.0 * degree, 20e3, 4) > dr::climbing_limit);
	CHECK(dm::pull_ratio(300e3, 30.0 * degree, 80e3, 4) < dr::climbing_limit);
}

TEST_CASE("derail motion: the vehicle leaves the track with physics.derailment new only") {
	test::track curve;
	curve.radius = 300.0;
	auto const critical{dr::overturning_speed(300.0, 0.0, eu07().centre_of_gravity, 1.435) / kmh};

	SUBCASE("new: derailed by the criteria it leaves the track, then slides with the friction of the motion") {
		model_setting const physical{global_settings::derailment_model::physical};
		vehicle eu07vehicle{definition::fiz("4e_v1/4e"), critical + 20.0};
		for (int step = 0; step < 10; ++step) {
			eu07vehicle.step(curve);
		}
		REQUIRE(eu07vehicle->Derailed == dr::outcome::overturned);
		CHECK(eu07vehicle->DerailCause == dm::cause::overturned);
		CHECK(eu07vehicle->DerailAcceleration > eu07_tipping);
		CHECK(eu07vehicle->leaves_track());
		eu07vehicle->OffTrack.begin(leaving(eu07vehicle->V, eu07vehicle->DerailCause, 1, eu07vehicle->DerailAcceleration), eu07(), ballast);
		CHECK_FALSE(eu07vehicle->leaves_track());
		// tipping over on its outer wheels it hardly slows down
		auto const start{eu07vehicle->V};
		for (int step = 0; step < 20; ++step) {
			eu07vehicle.step(curve);
		}
		auto const deceleration{(start - eu07vehicle->V) / 0.2};
		CHECK(deceleration == doctest::Approx(dm::guided_friction * dr::gravity * 80.0 / 96.0).epsilon(0.1));
		// back on the rails, not off the track any more
		eu07vehicle->repair_damage();
		CHECK_FALSE(eu07vehicle->OffTrack.active());
		CHECK_FALSE(eu07vehicle->leaves_track());
	}
	SUBCASE("new: off the end of the track it slides on its wheels, straight on") {
		model_setting const physical{global_settings::derailment_model::physical};
		vehicle eu07vehicle{definition::fiz("4e_v1/4e"), 40.0};
		eu07vehicle.damage_track(dtrack_norail);
		eu07vehicle.step();
		REQUIRE(TestFlag(eu07vehicle->DamageFlag, dtrain_out));
		CHECK(eu07vehicle->Derailed == dr::outcome::off_rails);
		CHECK(eu07vehicle->DerailCause == dm::cause::end_of_track);
		CHECK(eu07vehicle->leaves_track());
	}
	SUBCASE("new: a vehicle pulled off by a coupling, upright, to the side it was pulled to") {
		model_setting const physical{global_settings::derailment_model::physical};
		vehicle pulled{definition::fiz("4e_v1/4e"), 40.0};
		pulled->pull_off(-1, 5.0, "test");
		CHECK(TestFlag(pulled->DamageFlag, dtrain_out));
		CHECK(pulled->Derailed == dr::outcome::off_rails);
		CHECK(pulled->DerailCause == dm::cause::pulled_off);
		CHECK(pulled->DerailSide == -1);
		CHECK(pulled->leaves_track());
	}
}

TEST_CASE("derail motion: with physics.derailment monitor and old the vehicle stays on the track, as before") {
	test::track curve;
	curve.radius = 300.0;
	for (auto const model : {global_settings::derailment_model::monitor, global_settings::derailment_model::old}) {
		auto const number{static_cast<int>(model)};
		CAPTURE(number);
		model_setting const setting{model};
		vehicle fast{definition::fiz("4e_v1/4e"), 160.0};
		for (int step = 0; step < 600; ++step) {
			fast.step(curve);
		}
		REQUIRE(TestFlag(fast->DamageFlag, dtrain_out));
		CHECK(fast->Derailed == dr::outcome::none);
		CHECK_FALSE(fast->leaves_track());
		CHECK_FALSE(fast->OffTrack.active());
		// stopped dead by the legacy model
		CHECK(fast->V == doctest::Approx(0.0).epsilon(0.01));
		vehicle ending{definition::fiz("4e_v1/4e"), 40.0};
		ending.damage_track(dtrack_norail);
		ending.step();
		REQUIRE(TestFlag(ending->DamageFlag, dtrain_out));
		CHECK(ending->Derailed == dr::outcome::none);
		CHECK_FALSE(ending->leaves_track());
		// nor is a vehicle pulled off it
		vehicle pulled{definition::fiz("4e_v1/4e"), 40.0};
		pulled->pull_off(1, 5.0, "test");
		CHECK_FALSE(TestFlag(pulled->DamageFlag, dtrain_out));
	}
}

// ---- the yaw of the body, the train folding up, the scatter

namespace {

// a coach of 40 t, 24.5 m over the buffers
dm::vehicle_shape coach() {
	dm::vehicle_shape shape;
	shape.length = 24.5;
	shape.width = 2.825;
	shape.height = 4.05;
	shape.centre_of_gravity = dr::default_centre_of_gravity(4.05, dr::vehicle_class::unpowered);
	shape.contact_spacing = dr::contact_spacing(1.435);
	shape.base = 17.2;
	shape.mass = 40e3;
	return shape;
}

// an end of the body: its front (Front) or its rear, at the height of the buffers
dm::end_point end_of(dm::body const &Body, bool const Front) {
	// over its base, as TDynamicObject takes it
	auto const half{0.5 * Body.shape().length * (Front ? 1.0 : -1.0)};
	return dm::end_point{Body.base() + glm::dvec3{0.0, dm::buffer_height, 0.0} + Body.axis() * half, Front ? Body.axis() : -Body.axis(), Body.tipping()};
}

struct car {
	dm::body body;
	double speed{0.0};
	double gap{0.0}; // of the coupling behind it
};

// the train of bodies off the track on level ground as TMoverParameters and TDynamicObject drive it: their buffers and
// couplings a spring of 2 MN/m with its damping, the longitudinal motion of each from the forces of its couplings along
// its heading and the friction of its motion; the forces across turn them. Dt the step [s]; till all stop or 30 s
std::vector<car> run_train(std::vector<std::pair<dm::start, dm::scatter>> const &Starts, dm::vehicle_shape const &Shape, double const Dt = 0.01) {
	std::vector<car> train(Starts.size());
	for (std::size_t index = 0; index < train.size(); ++index) {
		train[index].body.begin(Starts[index].first, Shape, ballast, Starts[index].second);
		train[index].speed = Starts[index].first.speed;
	}
	for (std::size_t index = 0; index + 1 < train.size(); ++index) {
		train[index].gap = dm::meet(end_of(train[index].body, false), end_of(train[index + 1].body, true)).gap;
	}
	constexpr double stiffness{2e6};
	auto const damping{0.3 * 2.0 * std::sqrt(stiffness * Shape.mass)};
	constexpr double slack{0.05};
	for (double time = 0.0; time < 30.0; time += Dt) {
		std::vector<std::array<dm::end_force, 2>> ends(train.size());
		for (std::size_t index = 0; index + 1 < train.size(); ++index) {
			auto &ahead{train[index]};
			auto &behind{train[index + 1]};
			auto const rear{dm::meet(end_of(ahead.body, false), end_of(behind.body, true))};
			auto const front{dm::meet(end_of(behind.body, true), end_of(ahead.body, false))};
			auto const closing{(ahead.gap - rear.gap) / Dt};
			ahead.gap = rear.gap;
			auto compression{0.0};
			if (rear.gap < 0.0) {
				compression = std::max(0.0, -stiffness * rear.gap + damping * closing);
			}
			else if (rear.gap > slack) {
				compression = std::min(0.0, -stiffness * (rear.gap - slack) + damping * closing);
			}
			ends[index][1] = dm::force_on(rear, compression);
			ends[index + 1][0] = dm::force_on(front, compression);
		}
		auto moving{false};
		for (std::size_t index = 0; index < train.size(); ++index) {
			auto &item{train[index]};
			auto const push{glm::dot(ends[index][0].force + ends[index][1].force, item.body.heading()) / Shape.mass};
			auto const friction{dm::gravity * item.body.friction()};
			auto next{item.speed + push * Dt};
			if (std::abs(next) <= friction * Dt) {
				next = 0.0;
			}
			else {
				next -= (next > 0.0 ? 1.0 : -1.0) * friction * Dt;
			}
			auto const reversals{item.body.reversals()};
			item.body.step(Dt, 0.5 * (item.speed + next) * Dt, next, ballast, ends[index]);
			item.speed = (item.body.reversals() - reversals) % 2 != 0 ? -next : next;
			moving |= (false == item.body.at_rest());
		}
		if (false == moving) {
			break;
		}
	}
	return train;
}

// a train of four coaches at 133 km/h overturning out of the curve to the left, the leading one ploughing into the
// ground; each scattered by the seed, or none of them
std::vector<std::pair<dm::start, dm::scatter>> overturning_train(std::uint32_t const Seed, bool const Scattered = true) {
	std::vector<std::pair<dm::start, dm::scatter>> starts;
	for (int index = 0; index < 4; ++index) {
		auto start{leaving(133.0 * kmh, dm::cause::overturned, 1, 2.0 * eu07_tipping)};
		// one after the other along the way they leave the track
		auto const angle{dm::departure_angle(start.reason, start.speed, start.acceleration)};
		start.position = -index * coach().length * glm::dvec3{std::sin(angle), 0.0, std::cos(angle)};
		auto scatter{Scattered ? dm::draw_scatter(dm::seed_of(Seed, std::format("coach{}", index))) : dm::scatter{}};
		if (index == 0 && Scattered) {
			// the leading one ploughs into the ground: held back harder all the way, the ones behind push into it
			scatter.friction_front *= 1.5;
			scatter.friction_rear *= 1.5;
		}
		starts.emplace_back(start, scatter);
	}
	return starts;
}

// the bends between the bodies of the train, from the front [rad]
std::vector<double> bends(std::vector<car> const &Train) {
	std::vector<double> result;
	for (std::size_t index = 0; index + 1 < Train.size(); ++index) {
		result.push_back(std::remainder(Train[index + 1].body.yaw() - Train[index].body.yaw(), 2.0 * std::numbers::pi));
	}
	return result;
}

std::string described(std::vector<car> const &Train) {
	std::string result;
	for (auto const &item : Train) {
		result += std::format(" [{:.1f} m along, {:+.1f} m across, yaw {:+.1f} deg]", item.body.along(), item.body.across(), item.body.yaw() / degree);
	}
	return result;
}

} // namespace

TEST_CASE("derail motion: the buffers and the coupling of two ends") {
	// two ends facing each other, the second 10 degrees to the left of the axis of the first, the buffers pressed 20 mm
	dm::end_point const first{glm::dvec3{0.0, 1.0, 0.0}, glm::dvec3{0.0, 0.0, 1.0}};
	auto const angle{10.0 * degree};
	dm::end_point const second{glm::dvec3{0.0, 1.0, -0.02}, -glm::dvec3{std::sin(angle), 0.0, std::cos(angle)}};
	auto const pressed{dm::meet(first, second)};
	CHECK(pressed.bend == doctest::Approx(angle));
	CHECK(pressed.gap == doctest::Approx(-0.02 * std::cos(0.5 * angle)));
	CHECK(pressed.overlap == doctest::Approx(1.0));
	// the push of the buffers: against the bisector of the bend, through the inner buffer (to the left); less far out
	// once the push is more than a buffer takes
	auto const light{dm::force_on(pressed, 200e3)};
	CHECK(light.force.z == doctest::Approx(-200e3 * std::cos(0.5 * angle)));
	CHECK(light.force.x == doctest::Approx(-200e3 * std::sin(0.5 * angle)));
	CHECK(light.point.x == doctest::Approx(dm::buffer_half_spacing * std::cos(0.5 * angle)).epsilon(1e-3));
	auto const heavy{dm::force_on(pressed, 4.0 * dm::buffer_limit)};
	CHECK(heavy.point.x == doctest::Approx(0.25 * dm::buffer_half_spacing * std::cos(0.5 * angle)).epsilon(1e-3));
	// a body lower by a buffer head misses the buffers of the other one: the push goes through the middle of the beams
	dm::end_point low{second};
	low.point.y -= dm::buffer_head;
	CHECK(dm::force_on(dm::meet(first, low), 200e3).point.x == doctest::Approx(0.0));
	// the coupling pulls towards the other beam, taken 1 m to the side
	dm::end_point aside{second};
	aside.point = glm::dvec3{1.0, 1.0, 0.5};
	auto const pulled{dm::meet(first, aside)};
	CHECK(pulled.gap == doctest::Approx(std::hypot(1.0, 0.5)).epsilon(0.1));
	auto const pull{dm::force_on(pulled, -100e3)};
	CHECK(pull.force.x == doctest::Approx(100e3 / std::hypot(1.0, 0.5)));
	// gone past each other side by side: the coupling stretched
	aside.point = glm::dvec3{3.0, 1.0, -5.0};
	CHECK(dm::meet(first, aside).gap > 5.0);
	// a body turned about, its end facing the same way as the first one, 5 m behind it: they don't press each other
	dm::end_point const about{glm::dvec3{0.0, 1.0, -5.0}, glm::dvec3{0.0, 0.0, 1.0}};
	CHECK(dm::meet(first, about).gap == doctest::Approx(5.0));
}

TEST_CASE("derail motion: the yaw of the body") {
	auto shape{eu07()};
	shape.mass = 80e3;
	SUBCASE("a push off the middle turns it, the friction under it holds it against a small one") {
		for (auto const side : {1.0, -1.0}) {
			CAPTURE(side);
			dm::body body;
			body.begin(leaving(0.0, dm::cause::off_rails), shape, ballast);
			// 300 kN across at the front end, to the Side: it turns its front that way
			std::array<dm::end_force, 2> ends{};
			ends[0] = dm::end_force{glm::dvec3{side * 300e3, 0.0, 0.0}, glm::dvec3{0.0, 1.0, 0.5 * shape.length}};
			for (int step = 0; step < 50; ++step) {
				body.step(0.01, 0.0, 0.0, ballast, ends);
			}
			CHECK(body.yaw_rate() * side > 0.0);
			CHECK(body.yaw() * side > 0.0);
			// through the middle along the body it doesn't turn it
			dm::body straight;
			straight.begin(leaving(0.0, dm::cause::off_rails), shape, ballast);
			ends[0] = dm::end_force{glm::dvec3{0.0, 0.0, -300e3}, glm::dvec3{0.0, 1.0, 0.5 * shape.length}};
			straight.step(0.5, 0.0, 0.0, ballast, ends);
			CHECK(straight.yaw() == 0.0);
			// 30 kN across: the friction holds it, once it stands on the ground
			dm::flat_ground const rails{0.0};
			dm::body held;
			held.begin(leaving(0.0, dm::cause::off_rails), shape, rails);
			ends[0] = dm::end_force{glm::dvec3{side * 30e3, 0.0, 0.0}, glm::dvec3{0.0, 1.0, 0.5 * shape.length}};
			held.step(0.5, 0.0, 0.0, rails, ends);
			CHECK(held.yaw() == 0.0);
			CHECK(held.at_rest());
		}
	}
	SUBCASE("the friction under it stops the turning, sliding or standing") {
		for (auto const speed : {0.0, 30.0}) {
			CAPTURE(speed);
			dm::scatter spin;
			spin.yaw_rate = 0.5;
			dm::body body;
			body.begin(leaving(speed, dm::cause::off_rails), shape, ballast, spin);
			auto current{speed};
			auto most{0.0};
			auto halfway{0.0};
			for (int step = 0; step < 3000 && false == body.at_rest(); ++step) {
				auto const next{std::max(0.0, current - dm::gravity * body.friction() * 0.01)};
				body.step(0.01, 0.5 * (current + next) * 0.01, next, ballast);
				if (current > 0.5 * speed && next <= 0.5 * speed) {
					halfway = body.yaw_rate();
				}
				current = next;
				most = std::max(most, std::abs(body.yaw_rate()));
			}
			MESSAGE(std::format("set turning at 0.5 rad/s at {:.0f} m/s: turned {:.1f} deg", speed, body.yaw() / degree));
			CHECK(body.at_rest());
			CHECK(body.yaw_rate() == 0.0);
			// it never turns faster than it was set to
			CHECK(most <= 0.5 + 1e-9);
			CHECK(body.yaw() > 0.0);
			// standing it stops within a turn of a few degrees; sliding fast, little holds it (the friction under it goes
			// almost all against the way it slides), it slows the turning as it slows down
			if (speed > 0.0) {
				CHECK(halfway < 0.4);
				CHECK(body.yaw() < 180.0 * degree);
			}
			else {
				CHECK(body.yaw() < 15.0 * degree);
			}
		}
	}
	SUBCASE("a body going askew slides harder, and the way it goes turns to the body") {
		dm::scatter askew;
		askew.yaw = 20.0 * degree;
		dm::body body;
		body.begin(leaving(30.0, dm::cause::off_rails), shape, ballast, askew);
		CHECK(body.friction() > dr::off_rails_friction);
		CHECK(body.friction() < dm::lateral_friction);
		auto const before{std::abs(std::remainder(std::atan2(body.heading().x, body.heading().z) - body.yaw(), 2.0 * std::numbers::pi))};
		auto current{30.0};
		for (int step = 0; step < 100; ++step) {
			auto const next{current - dm::gravity * body.friction() * 0.01};
			body.step(0.01, 0.5 * (current + next) * 0.01, next, ballast);
			current = next;
		}
		auto const after{std::abs(std::remainder(std::atan2(body.heading().x, body.heading().z) - body.yaw(), 2.0 * std::numbers::pi))};
		CHECK(after < before);
	}
}

TEST_CASE("derail motion: a train off the track pushed from behind folds into a zigzag") {
	// the leading one held back harder, the ones behind push into it: a chaotic matter, so over eight seeds most of the
	// trains fold, and the folds next to each other turn opposite ways
	auto foldedtrains{0};
	auto pairs{0};
	auto zigzags{0};
	for (std::uint32_t seed = 1; seed <= 8; ++seed) {
		CAPTURE(seed);
		auto const train{run_train(overturning_train(seed), coach())};
		auto const bent{bends(train)};
		MESSAGE((std::format("seed {} at rest:", seed) + described(train)));
		for (auto const &item : train) {
			CHECK(item.body.at_rest());
		}
		auto folds{0};
		for (std::size_t index = 0; index < bent.size(); ++index) {
			if (std::abs(bent[index]) > 10.0 * degree) {
				++folds;
				if (index + 1 < bent.size() && std::abs(bent[index + 1]) > 10.0 * degree) {
					++pairs;
					zigzags += (bent[index] * bent[index + 1] < 0.0 ? 1 : 0);
				}
			}
		}
		foldedtrains += (folds > 0 ? 1 : 0);
	}
	MESSAGE(std::format("{} of 8 trains folded, {} of {} neighbouring folds turn opposite ways", foldedtrains, zigzags, pairs));
	CHECK(foldedtrains >= 4);
	CHECK(pairs >= 2);
	CHECK(zigzags * 4 >= pairs * 3);
	SUBCASE("without the bite and the scatter it stays in line") {
		auto const straight{run_train(overturning_train(1, false), coach())};
		MESSAGE((std::string{"unscattered at rest:"} + described(straight)));
		for (auto const bend : bends(straight)) {
			CHECK(std::abs(bend) < 1.0 * degree);
		}
	}
}

TEST_CASE("derail motion: the scatter") {
	SUBCASE("the same seed, the same scatter; another seed, another one") {
		auto const one{dm::draw_scatter(dm::seed_of(7, "coach1"))};
		auto const again{dm::draw_scatter(dm::seed_of(7, "coach1"))};
		auto const other{dm::draw_scatter(dm::seed_of(8, "coach1"))};
		auto const next{dm::draw_scatter(dm::seed_of(7, "coach2"))};
		CHECK(one.departure == again.departure);
		CHECK(one.friction_front == again.friction_front);
		CHECK(one.dig_in == again.dig_in);
		CHECK(one.yaw != other.yaw);
		CHECK(one.yaw != next.yaw);
		auto bites{0};
		for (std::uint32_t seed = 0; seed < 300; ++seed) {
			auto const scatter{dm::draw_scatter(seed)};
			CHECK(std::abs(scatter.departure) <= 0.01);
			CHECK(std::abs(scatter.yaw) <= 0.02);
			CHECK(std::abs(scatter.yaw_rate) <= 0.02);
			CHECK(scatter.friction_front >= 0.85);
			CHECK(scatter.friction_rear <= 1.15);
			CHECK((scatter.dig_in < 0.0 || (scatter.dig_in >= 0.2 && scatter.dig_in <= 1.5)));
			bites += (scatter.dig_in >= 0.0 ? 1 : 0);
		}
		CHECK(bites > 150);
		CHECK(bites < 250);
	}
	SUBCASE("the train: the same seed, the same way; another seed, another way") {
		auto const one{bends(run_train(overturning_train(1), coach()))};
		auto const again{bends(run_train(overturning_train(1), coach()))};
		auto const othertrain{run_train(overturning_train(2), coach())};
		auto const other{bends(othertrain)};
		MESSAGE((std::string{"seed 2 at rest:"} + described(othertrain)));
		CHECK(one == again);
		auto apart{0.0};
		for (std::size_t index = 0; index < one.size(); ++index) {
			apart = std::max(apart, std::abs(one[index] - other[index]));
		}
		MESSAGE(std::format("the bends of seed 1 and seed 2 differ by {:.1f} deg at most", apart / degree));
		CHECK(apart > 2.0 * degree);
	}
	SUBCASE("the same with steps of 0.01, 0.03 and 0.05 s") {
		auto starts{overturning_train(3)};
		starts.resize(1);
		auto const fine{run_train(starts, coach(), 0.01)};
		for (auto const dt : {0.03, 0.05}) {
			CAPTURE(dt);
			auto const coarse{run_train(starts, coach(), dt)};
			CHECK(coarse[0].body.travelled() == doctest::Approx(fine[0].body.travelled()).epsilon(0.02));
			CHECK(coarse[0].body.across() == doctest::Approx(fine[0].body.across()).epsilon(0.05));
			CHECK(coarse[0].body.yaw() == doctest::Approx(fine[0].body.yaw()).epsilon(0.05));
		}
	}
}
