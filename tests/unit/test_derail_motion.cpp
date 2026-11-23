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
		body.step(Dt, 0.5 * (speed + next) * Dt, next, Ground);
		speed = next;
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
