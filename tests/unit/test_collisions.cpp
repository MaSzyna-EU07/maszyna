#include "stdafx.h"
#include "doctest.h"
#include "consist.h"
#include "vehicle.h"
#include "McZapkie/collision.h"
#include "McZapkie/MOVER.h"
#include "utilities/Globals.h"

#include <cmath>
#include <format>
#include <string>
#include <vector>

// collisions past the buffers with physics.derailment new (McZapkie/collision.h): the crush of the ends, the override,
// the push off the track, and what they do in trains of real vehicles

using test::consist;
using test::definition;
using test::vehicle;
namespace cl = collision;

namespace {

constexpr double kmh{1.0 / 3.6};

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

// the given setting of a global flag for the test, restored after it
class flag_setting {
public:
	flag_setting(bool &Flag, bool Enabled) : m_flag{Flag}, m_previous{Flag} { m_flag = Enabled; }
	~flag_setting() { m_flag = m_previous; }
	flag_setting(flag_setting const &) = delete;
	flag_setting &operator=(flag_setting const &) = delete;

private:
	bool &m_flag;
	bool m_previous;
};

// ---- the hit of two masses on the stop of their buffers, the way the vehicles take it

struct pair_outcome {
	double crush1{0.0};
	double crush2{0.0};
	double velocity1{0.0};
	double velocity2{0.0};
	double peak{0.0}; // force
};

// the first mass runs at the velocity into the second one standing, their buffers pressed all the way already; the
// springs carry the given force while pressed, stepped by Dt as the vehicles are: the springs over the step first, the
// hit after them
pair_outcome hit_pair(cl::end_state First, cl::end_state Second, double const Mass1, double const Mass2, double const Velocity, double const Springs, double const Dt) {
	pair_outcome result;
	auto velocity1{Velocity};
	auto velocity2{0.0};
	for (int step = 0; step < 1000000; ++step) {
		velocity1 -= Springs * Dt / Mass1;
		velocity2 += Springs * Dt / Mass2;
		auto const closing{velocity1 - velocity2};
		if (closing <= 0.0) {
			break;
		}
		cl::hit_input input;
		input.velocity = closing;
		input.mass1 = Mass1;
		input.mass2 = Mass2;
		input.restitution = 0.2;
		input.stiffness = 1.4e8;
		input.springs = Springs;
		input.dt = Dt;
		auto const hit{cl::hit(First, Second, input)};
		if (false == hit.crushing) {
			// elastic: bounced apart with the restitution
			auto const reduced{Mass1 * Mass2 / (Mass1 + Mass2)};
			auto const impulse{reduced * closing * (1.0 + input.restitution)};
			velocity1 -= impulse / Mass1;
			velocity2 += impulse / Mass2;
			break;
		}
		velocity1 -= hit.impulse / Mass1;
		velocity2 += hit.impulse / Mass2;
		First.crush += hit.crush1;
		Second.crush += hit.crush2;
		result.peak = std::max(result.peak, hit.force);
		if (hit.velocity <= 0.0) {
			break;
		}
	}
	result.crush1 = First.crush;
	result.crush2 = Second.crush;
	result.velocity1 = velocity1;
	result.velocity2 = velocity2;
	return result;
}

cl::crush_curve const wagon_curve{cl::default_crush(cl::vehicle_kind::wagon)};
cl::crush_curve const locomotive_curve{cl::default_crush(cl::vehicle_kind::locomotive)};

// ---- trains of real vehicles

bool derailed(TMoverParameters const &Vehicle) {
	return TestFlag(Vehicle.DamageFlag, dtrain_out);
}

double crush_of(TMoverParameters const &Vehicle) {
	return std::max(Vehicle.Couplers[end::front].crush, Vehicle.Couplers[end::rear].crush);
}

// a mixed freight train as the PoC has it: every third wagon from the given one an empty two-axle 3WL (10.5 t), the
// rest 430W coal wagons loaded full (80 t)
definition mixed_wagon(int const Index) {
	return definition::fiz(Index % 3 == 1 ? "3wl_v2/3wl" : "430w_v2/430w");
}

vehicle &add_mixed(consist &Train, int const Index, int const Coupling, double const Velocity, double const Gap) {
	auto &wagon{Train.add(mixed_wagon(Index), 0, Coupling, Velocity, Gap)};
	if (Index % 3 != 1) {
		wagon->AssignLoad("coal", 59.5f);
		wagon->ComputeMass();
	}
	return wagon;
}

struct crash_outcome {
	int derailed{0};
	int overridden{0};
	int pushed{0};
	int crushed{0}; // ends crushed past the mark
	int broken{0}; // couplings torn off
	double crush{0.0}; // the deepest crush of an end
	bool locomotive{false}; // it derailed
	std::string list; // what happened to which vehicle, for the messages
};

// the PoC of G14: a locomotive (EU07, 80 t) with 12 mixed wagons running into the rear of 8 mixed wagons standing
// braked, in emergency, on a straight track or in a curve
crash_outcome freight_crash(double const Speed, double const Radius, double const Dt = 0.01) {
	test::track track;
	track.radius = Radius;
	consist train;
	// the standing train, the wagon it's hit at last; its brakes charged, then in emergency
	constexpr int standing{8};
	for (int i = standing - 1; i >= 0; --i) {
		add_mixed(train, i, consist::full_coupling, 0.0, 0.0).hold_brake_pipe(5.0);
	}
	train.run(90.0, track);
	for (int i = 0; i < standing; ++i) {
		train[i].hold_brake_pipe(0.0);
	}
	train.run(30.0, track);
	// the train running into it, its locomotive leading, a metre short of the hit
	train.add(definition::fiz("4e_v1/4e"), 0, coupling::faux, Speed, 1.0);
	for (int i = 0; i < 12; ++i) {
		add_mixed(train, i, consist::full_coupling, Speed, 0.0);
	}
	train.run(12.0, track, Dt);

	crash_outcome result;
	for (std::size_t i = 0; i < train.size(); ++i) {
		auto const &vehicle{train[i].mover()};
		auto const name{i < standing ? std::format("standing {}", standing - 1 - i) : i == standing ? std::string{"locomotive"} : std::format("running {}", i - standing - 1)};
		for (auto const end : {end::front, end::rear}) {
			if (vehicle.Couplers[end].crush >= cl::crush_mark) {
				++result.crushed;
			}
		}
		result.crush = std::max(result.crush, crush_of(vehicle));
		if (derailed(vehicle)) {
			++result.derailed;
			result.overridden += vehicle.CollisionDerail == COLLISION_OVERRIDE;
			result.pushed += vehicle.CollisionDerail == COLLISION_PUSHED_OFF;
			result.locomotive = result.locomotive || i == standing;
			result.list += std::format("{} ({:.1f} t) {}; ", name, vehicle.TotalMass / 1000.0, vehicle.CollisionDerail == COLLISION_OVERRIDE ? "overrode" : "pushed off");
		}
		else if (crush_of(vehicle) >= cl::crush_mark) {
			result.list += std::format("{} crushed {:.2f} m; ", name, crush_of(vehicle));
		}
	}
	for (std::size_t i = 0; i + 1 < train.size(); ++i) {
		result.broken += train[i]->Couplers[end::rear].broken;
	}
	return result;
}

// a locomotive running into a loaded coal wagon standing free, both on the rails: the crush of the pair after the hit
double locomotive_into_wagon(double const Speed, double const Dt) {
	consist train;
	auto &wagon{train.add(definition::fiz("430w_v2/430w"))};
	wagon->AssignLoad("coal", 59.5f);
	wagon->ComputeMass();
	// behind the wagon, running forwards into its rear end
	auto &locomotive{train.add(definition::fiz("4e_v1/4e"), 0, coupling::faux, Speed, 0.5)};
	train.run(3.0, {}, Dt);
	return wagon->Couplers[end::rear].crush + locomotive->Couplers[end::front].crush;
}

} // namespace

// ---- the crush curve

TEST_CASE("collisions: the crush curve, a peak, a plateau, packing down, and the limit") {
	auto const &curve{wagon_curve};
	CHECK(curve.force(0.0) == doctest::Approx(3.0e6));
	CHECK(curve.force(0.049) == doctest::Approx(3.0e6));
	CHECK(curve.force(0.05) == doctest::Approx(2.0e6));
	CHECK(curve.force(0.79) == doctest::Approx(2.0e6));
	// 2 / m past 0.8 m: a metre further it's three times the plateau
	CHECK(curve.force(1.3) == doctest::Approx(2.0e6 * (1.0 + 2.0 * 0.5)));
	CHECK_FALSE(std::isfinite(curve.force(1.5)));
	// the energy is the area under it
	CHECK(curve.energy(0.0, 0.05) == doctest::Approx(3.0e6 * 0.05));
	CHECK(curve.energy(0.0, 0.8) == doctest::Approx(3.0e6 * 0.05 + 2.0e6 * 0.75));
	CHECK(curve.energy(0.8, 1.3) == doctest::Approx(0.5 * (2.0e6 + 4.0e6) * 0.5));
	CHECK(curve.energy(0.0, 0.3) + curve.energy(0.3, 1.0) == doctest::Approx(curve.energy(0.0, 1.0)));
	// past the limit there's nothing more to crush
	CHECK(curve.energy(1.5, 3.0) == 0.0);

	// the kinds of vehicles; a crash zone gives a steady force with no peak
	CHECK(locomotive_curve.peak > wagon_curve.peak);
	auto const zone{cl::default_crush(cl::vehicle_kind::crash_zone)};
	CHECK(zone.force(0.0) == doctest::Approx(zone.force(0.5)));
	// the keys of the .fiz override the defaults of the kind
	cl::end_definition given;
	given.plateau = 1.0e6;
	given.crash_zone = true;
	auto const own{cl::crush_of(given, cl::vehicle_kind::coach)};
	CHECK(own.plateau == doctest::Approx(1.0e6));
	CHECK(own.densify == doctest::Approx(zone.densify));
	CHECK(cl::anticlimber_of(given, cl::vehicle_kind::coach));
	CHECK_FALSE(cl::anticlimber_of({}, cl::vehicle_kind::locomotive));
}

TEST_CASE("collisions: the buffers of EN 15551, their stroke and energy") {
	for (auto const category : {cl::buffer_category::A, cl::buffer_category::B, cl::buffer_category::C, cl::buffer_category::L, cl::buffer_category::passenger}) {
		auto const rated{cl::rating(category)};
		for (auto const progression : {0.0, 0.5, 0.9}) {
			auto const curve{cl::buffer_curve(category, progression)};
			CHECK(curve.travel == doctest::Approx(rated.stroke));
			// two buffers side by side
			CHECK(curve.energy(curve.travel) == doctest::Approx(2.0 * rated.energy));
		}
	}
	CHECK(cl::rating(cl::buffer_category::A).energy < cl::rating(cl::buffer_category::C).energy);
	CHECK(cl::buffer_category_of('C') == cl::buffer_category::C);
	CHECK_FALSE(cl::buffer_category_of('X').has_value());
	// the triangle of category A: 2 x 30 kJ over 105 mm is about 1.1 MN at the end of the stroke, 570 kN a buffer
	CHECK(cl::buffer_curve(cl::buffer_category::A, 0.0).end_force() == doctest::Approx(4.0 * 30.0e3 / 0.105));
}

// ---- the hit

TEST_CASE("collisions: a hit the steel takes stays elastic, a harder one crushes the weaker end") {
	cl::hit_input input;
	input.mass1 = 80.0e3;
	input.mass2 = 80.0e3;
	input.stiffness = 1.4e8;
	input.dt = 0.01;
	// 40 t reduced: the steel takes 3 MN at 3 / sqrt(1.4e8 * 4e4) = 1.27 m/s
	input.velocity = 1.0;
	CHECK_FALSE(cl::hit({wagon_curve, 0.0}, {wagon_curve, 0.0}, input).crushing);
	input.velocity = 5.0;
	auto const hit{cl::hit({locomotive_curve, 0.0}, {wagon_curve, 0.0}, input)};
	REQUIRE(hit.crushing);
	// the wagon gives, the locomotive doesn't; at its peak force for the whole step
	CHECK(hit.crush1 == 0.0);
	CHECK(hit.crush2 > 0.0);
	CHECK(hit.force == doctest::Approx(3.0e6));
	CHECK(hit.impulse == doctest::Approx(3.0e6 * 0.01));
	CHECK(hit.duration == doctest::Approx(0.01));
	CHECK(hit.velocity == doctest::Approx(5.0 - 3.0e6 * 0.01 / 40.0e3));
	// with the springs carrying a part of the force, the hit adds the rest
	input.springs = 1.0e6;
	auto const sprung{cl::hit({locomotive_curve, 0.0}, {wagon_curve, 0.0}, input)};
	CHECK(sprung.impulse == doctest::Approx(2.0e6 * 0.01));
	// two alike crush alike
	input.springs = 0.0;
	auto const alike{cl::hit({wagon_curve, 0.0}, {wagon_curve, 0.0}, input)};
	CHECK(alike.crush1 == doctest::Approx(alike.crush2));
	// a wall: the end crushed all the way doesn't give, the other one does
	auto const wall{cl::hit({wagon_curve, 2.0}, {wagon_curve, 0.0}, input)};
	CHECK(wall.crush1 == 0.0);
	CHECK(wall.crush2 > 0.0);
	// both crushed all the way: rigid, the legacy exchange
	CHECK_FALSE(cl::hit({wagon_curve, 2.0}, {wagon_curve, 2.0}, input).crushing);
}

TEST_CASE("collisions: the crush doesn't depend on the step, and it takes the energy of the hit") {
	struct pair {
		char const *name;
		double mass1;
		double mass2;
		double velocity;
		double springs;
	};
	for (auto const &[name, mass1, mass2, velocity, springs] : {
	         pair{"80 t into 80 t at 25 km/h", 80.0e3, 80.0e3, 25.0 * kmh, 0.0},
	         pair{"80 t into 80 t at 40 km/h, the springs at 1 MN", 80.0e3, 80.0e3, 40.0 * kmh, 1.0e6},
	         pair{"80 t into 12 t at 40 km/h", 80.0e3, 12.0e3, 40.0 * kmh, 0.5e6},
	         pair{"300 t into 80 t at 35 km/h, past the densification", 300.0e3, 80.0e3, 35.0 * kmh, 0.0},
	     }) {
		CAPTURE(name);
		auto const fine{hit_pair({locomotive_curve, 0.0}, {wagon_curve, 0.0}, mass1, mass2, velocity, springs, 0.001)};
		REQUIRE(fine.crush2 > 0.05);
		CHECK(fine.crush1 == 0.0);
		for (auto const dt : {0.005, 0.01, 0.05}) {
			CAPTURE(dt);
			auto const coarse{hit_pair({locomotive_curve, 0.0}, {wagon_curve, 0.0}, mass1, mass2, velocity, springs, dt)};
			CHECK(coarse.crush2 == doctest::Approx(fine.crush2).epsilon(0.02));
			// the momentum stays with the pair
			CHECK(mass1 * coarse.velocity1 + mass2 * coarse.velocity2 == doctest::Approx(mass1 * velocity));
			// never past the force the wagon gives at: its peak, or what it packed down to
			CHECK(coarse.peak <= std::max(wagon_curve.peak, wagon_curve.force(coarse.crush2)) * (1.0 + 1e-9));
		}
		if (springs == 0.0) {
			// the energy of the relative motion goes into the crush, but for what the steel takes back
			auto const reduced{mass1 * mass2 / (mass1 + mass2)};
			auto const kinetic{0.5 * reduced * velocity * velocity};
			auto const crushforce{wagon_curve.force(fine.crush2)};
			auto const elastic{crushforce * crushforce / (2.0 * 1.4e8)};
			CHECK(wagon_curve.energy(0.0, fine.crush2) == doctest::Approx(kinetic - elastic).epsilon(0.01));
		}
	}
}

TEST_CASE("collisions: the crush of two pairs of masses at the speeds of the table") {
	// 80 t on 80 t: the steel takes the hit up to 1.3 m/s past the buffers; at 25 km/h the two ends alike crush by
	// 0.4 m between them, the 0.97 MJ at 3 MN to 0.05 m each and at 2 MN past it (the buffers aren't here)
	auto const slow{hit_pair({wagon_curve, 0.0}, {wagon_curve, 0.0}, 80.0e3, 80.0e3, 1.2, 0.0, 0.01)};
	CHECK(slow.crush1 + slow.crush2 == 0.0);
	auto const hard{hit_pair({wagon_curve, 0.0}, {wagon_curve, 0.0}, 80.0e3, 80.0e3, 25.0 * kmh, 0.0, 0.01)};
	CHECK(hard.crush1 + hard.crush2 > 0.2);
	CHECK(hard.crush1 + hard.crush2 < 0.5);
}

// ---- override

TEST_CASE("collisions: the lighter or the higher vehicle climbs onto the other one, both past a deeper crush") {
	cl::override_input input;
	input.mass1 = 80.0e3;
	input.mass2 = 80.0e3;
	input.height1 = 1.040;
	input.height2 = 1.040;
	SUBCASE("a shallow crush holds") {
		input.crush1 = 0.3;
		auto const result{cl::override_of(input)};
		CHECK_FALSE(result.first);
		CHECK_FALSE(result.second);
	}
	SUBCASE("alike: past 0.35 m they hold, past 0.6 m both climb") {
		input.crush1 = 0.4;
		CHECK_FALSE(cl::override_of(input).first);
		input.crush2 = 0.25;
		auto const result{cl::override_of(input)};
		CHECK(result.first);
		CHECK(result.second);
	}
	SUBCASE("the lighter one climbs past 0.35 m") {
		input.mass1 = 12.0e3;
		input.crush1 = 0.4;
		auto const result{cl::override_of(input)};
		CHECK(result.first);
		CHECK_FALSE(result.second);
	}
	SUBCASE("40 mm higher, the higher one climbs") {
		input.height2 = 1.000;
		input.crush1 = 0.2;
		input.crush2 = 0.2;
		auto const result{cl::override_of(input)};
		CHECK(result.first);
		CHECK_FALSE(result.second);
	}
	SUBCASE("interlocked anticlimbers hold them until the crush zone of an end is used up") {
		input.anticlimbers = true;
		input.zone1 = 0.8;
		input.zone2 = 0.8;
		input.height2 = 1.000;
		input.mass1 = 40.0e3;
		input.crush1 = 0.5;
		input.crush2 = 0.5;
		auto const held{cl::override_of(input)};
		CHECK_FALSE(held.first);
		CHECK_FALSE(held.second);
		input.crush1 = 0.85;
		auto const spent{cl::override_of(input)};
		CHECK(spent.first);
		CHECK(spent.second);
	}
}

// ---- pushed off the track

TEST_CASE("collisions: an empty two-axle wagon between loaded ones in a curve of 150 m is pushed off past the safe push of EN 15566") {
	// the vehicles of data/fiz: the 3WL empty (10.5 t, two axles 6 m apart), between 430W coal wagons
	vehicle empty{definition::fiz("3wl_v2/3wl")};
	vehicle loaded{definition::fiz("430w_v2/430w")};
	cl::pushed_vehicle wagon;
	wagon.mass = empty->TotalMass - empty->Mred;
	wagon.axles = empty->NAxles;
	wagon.length = empty->Dim.L;
	wagon.pivots = empty->BDist;
	wagon.axle_base = empty->ADist > 0.0 ? empty->ADist : empty->BDist;
	REQUIRE(wagon.axles == 2);
	cl::pushed_end end;
	end.other_length = loaded->Dim.L;
	auto const ratio = [&](double const Force, double const Radius) {
		end.force = Force;
		return cl::pushed_ratio(wagon, end, Radius);
	};
	// 250 kN on the pair of buffers in a curve of 150 m is safe (EN 15566, 4.2); 600 kN pushes it off
	CHECK(ratio(250.0e3, 150.0) < 1.2);
	CHECK(ratio(250.0e3, 150.0) > 0.5);
	CHECK(ratio(600.0e3, 150.0) > 1.2);
	// the wider the curve the more it takes; on a straight track the empty wagon lifts off past 1.5 MN: the force
	// pitching it takes the load off its leading wheels
	CHECK(ratio(600.0e3, 500.0) < ratio(600.0e3, 150.0));
	CHECK(ratio(600.0e3, 0.0) < 1.2);
	CHECK(ratio(1.5e6, 0.0) < 1.2);
	CHECK(ratio(2.5e6, 0.0) > 1.2);
	// a crushed end no longer holds the ends in line
	end.crush = 0.5;
	CHECK(ratio(1.5e6, 0.0) > 1.2);
	end.crush = 0.0;

	// the loaded wagon, four axles, takes it
	cl::pushed_vehicle heavy;
	heavy.mass = loaded->TotalMass - loaded->Mred + 59.5e3;
	heavy.axles = loaded->NAxles;
	heavy.length = loaded->Dim.L;
	heavy.pivots = loaded->BDist;
	heavy.axle_base = loaded->ADist;
	end.force = 600.0e3;
	CHECK(cl::pushed_ratio(heavy, end, 150.0) < 0.5);

	// it has to hold for 0.05 s, however long the steps
	cl::pushed_monitor monitor;
	CHECK_FALSE(monitor.update(true, 0.03));
	CHECK_FALSE(monitor.update(false, 0.01));
	CHECK_FALSE(monitor.update(true, 0.03));
	CHECK(monitor.update(true, 0.03));
}

// ---- the model of eu07.ini

TEST_CASE("collisions: the legacy rule with physics.derailment old, the crush with new") {
	flag_setting const crashdamage{Global.crash_damage, true};
	struct collision {
		char const *name;
		char const *front;
		char const *back;
		double speed; // of the back one, running into the front one standing
		bool frontout_old;
		bool backout_old;
		bool frontout_new;
		bool backout_new;
	};
	// the legacy rule: over 15 km/h weighed by the ratio of the masses. the crush: the light wagon is crushed deep and
	// climbs onto the locomotive at 60 km/h; two locomotives at 25 km/h crush their ends 0.1-0.3 m and stay on the rails
	for (auto const &[name, front, back, speed, frontout_old, backout_old, frontout_new, backout_new] : {
	         collision{"locomotive into a wagon at 60", "3wl_v2/3wl", "4e_v1/4e", 60.0, true, false, true, false},
	         collision{"locomotive into a locomotive at 25", "4e_v1/4e", "4e_v1/4e", 25.0, true, true, false, false},
	         collision{"locomotive into a locomotive at 50", "4e_v1/4e", "4e_v1/4e", 50.0, true, true, true, true},
	     }) {
		CAPTURE(name);
		for (auto const model : {global_settings::derailment_model::old, global_settings::derailment_model::physical}) {
			auto const physical{model == global_settings::derailment_model::physical};
			CAPTURE(physical);
			model_setting const setting{model};
			consist train;
			train.add(definition::fiz(front));
			train.add(definition::fiz(back), 0, coupling::faux, speed, 20.0);
			train.run(10.0);
			CHECK(derailed(train[0].mover()) == (physical ? frontout_new : frontout_old));
			CHECK(derailed(train[1].mover()) == (physical ? backout_new : backout_old));
			auto const crush{train[0]->Couplers[end::rear].crush + train[1]->Couplers[end::front].crush};
			if (physical) {
				MESSAGE(name, ": crush ", crush, " m");
				CHECK(crush > 0.05);
			}
			else {
				CHECK(crush == 0.0);
			}
		}
	}
}

TEST_CASE("collisions: the crush of a locomotive hitting a wagon doesn't depend on the step, nor on the model of the coupling") {
	model_setting const physical{global_settings::derailment_model::physical};
	flag_setting const crashdamage{Global.crash_damage, true};
	for (auto const physics : {false, true}) {
		CAPTURE(physics);
		flag_setting const couplerphysics{Global.coupler_physics, physics};
		flag_setting const everywhere{Global.coupler_physics_all, physics};
		auto const fine{locomotive_into_wagon(30.0, 0.001)};
		auto const game{locomotive_into_wagon(30.0, 0.01)};
		auto const coarse{locomotive_into_wagon(30.0, 0.05)};
		MESSAGE(std::format("30 km/h, couplerphysics {}: crush {:.3f} / {:.3f} / {:.3f} m with dt 0.001 / 0.01 / 0.05 s", physics, fine, game, coarse));
		REQUIRE(fine > 0.05);
		// the step of the game, and a step five times as long, which the buffers past the crush aren't made for
		CHECK(game == doctest::Approx(fine).epsilon(0.1));
		CHECK(coarse == doctest::Approx(fine).epsilon(0.5));
	}
	// the models of the coupling differ in their buffers, not past them
	{
		auto const legacy{locomotive_into_wagon(30.0, 0.01)};
		flag_setting const couplerphysics{Global.coupler_physics, true};
		flag_setting const everywhere{Global.coupler_physics_all, true};
		CHECK(locomotive_into_wagon(30.0, 0.01) == doctest::Approx(legacy).epsilon(0.3));
	}
	// slow enough, the buffers take it all
	CHECK(locomotive_into_wagon(5.0, 0.01) == 0.0);
}

TEST_CASE("collisions: automatic couplers hit too fast don't catch, the ends crush; torn off, they don't couple again") {
	model_setting const physical{global_settings::derailment_model::physical};
	flag_setting const crashdamage{Global.crash_damage, true};
	auto const automatic{definition::load("wagon").set("BuffCoupl", "CType", "Automatic").set("BuffCoupl", "AutomaticFlag", coupling::coupler | coupling::brakehose)};
	for (auto const speed : {20.0, 30.0, 45.0}) {
		CAPTURE(speed);
		vehicle moving{automatic, speed, "moving"};
		vehicle standing{automatic, 0.0, "standing"};
		test::pair pair{moving, end::front, standing, end::rear, 1.0};
		auto coupled{false};
		auto fast{false}; // coupled while closing in faster than the couplers catch
		for (int i = 0; i < 1000; ++i) {
			auto const closing{moving->CollisionVelocity(end::front, standing.mover(), end::rear)};
			pair.step();
			fast = fast || (pair.coupled() && false == coupled && closing > TMoverParameters::SafeCollisionVelocity);
			coupled = coupled || pair.coupled();
		}
		auto const crush{moving->Couplers[end::front].crush + standing->Couplers[end::rear].crush};
		MESSAGE(speed, " km/h: crush ", crush, " m, derailed ", derailed(moving.mover()), " ", derailed(standing.mover()), ", coupled later ", coupled);
		CHECK_FALSE(fast);
		CHECK(crush > 0.0);
		// crushed past the draw gear, the coupler is gone; short of it, the wagons may roll together again slowly
		// after the hit, and couple then
		if (std::max(moving->Couplers[end::front].crush, standing->Couplers[end::rear].crush) >= cl::draw_gear_crush) {
			CHECK_FALSE(coupled);
			CHECK((moving->Couplers[end::front].broken || standing->Couplers[end::rear].broken));
		}
		// the two alike: neither climbs onto the other until the crush is deep enough for both to
		CHECK(derailed(moving.mover()) == derailed(standing.mover()));
		CHECK(derailed(moving.mover()) == (crush > cl::override_both));
	}
}

TEST_CASE("collisions: road vehicles keep the legacy rule") {
	// the traffic of a scenery: cars and buses have no underframe to crush; over 15 km/h weighed by the masses they
	// derail, as with physics.derailment old (found running line 107: two cars of its traffic, 119 km/h head-on)
	flag_setting const crashdamage{Global.crash_damage, true};
	for (auto const model : {global_settings::derailment_model::old, global_settings::derailment_model::physical}) {
		model_setting const setting{model};
		consist road;
		road.add(definition::fiz("road/man/nl293"));
		road.add(definition::fiz("road/man/nl293"), 0, coupling::faux, 40.0, 5.0);
		road.run(5.0);
		CHECK(derailed(road[0].mover()));
		CHECK(derailed(road[1].mover()));
		CHECK(road[0]->Couplers[end::rear].crush == 0.0);
		CHECK(road[1]->CollisionDerail == NONE);
	}
}

TEST_CASE("collisions: a vehicle off the rails by the new model stays in the chain of forces") {
	model_setting const physical{global_settings::derailment_model::physical};
	flag_setting const crashdamage{Global.crash_damage, true};
	consist train;
	auto &wagon{train.add(definition::fiz("430w_v2/430w"))};
	auto &locomotive{train.add(definition::fiz("4e_v1/4e"), 0, coupling::faux, 10.0, 0.5)};
	wagon->derail_in_collision(COLLISION_PUSHED_OFF, "for the test");
	REQUIRE(derailed(wagon.mover()));
	auto const before{wagon->TotalMass * wagon->V + locomotive->TotalMass * locomotive->V};
	// the hit is over within a second, the friction of the derailed wagon slows the pair down over it
	train.run(1.0);
	auto const after{wagon->TotalMass * wagon->V + locomotive->TotalMass * locomotive->V};
	// pushed along, not a wall the locomotive bounces off
	CHECK(wagon.speed() > 1.0);
	CHECK(locomotive.speed() > 0.0);
	CHECK(after < before);
	CHECK(after > 0.5 * before);
	// and it slides on with the friction of the ballast, stopping before long
	train.run(10.0);
	CHECK(std::abs(wagon.speed()) < 0.1);
}

TEST_CASE("collisions: a freight train running into a standing one, the cases of the PoC of G14") {
	model_setting const physical{global_settings::derailment_model::physical};
	flag_setting const crashdamage{Global.crash_damage, true};
	struct crash {
		double speed;
		double radius;
	};
	for (auto const &[speed, radius] : {crash{10.0, 0.0}, crash{15.0, 0.0}, crash{15.0, 190.0}, crash{25.0, 0.0}, crash{25.0, 190.0}, crash{40.0, 0.0}, crash{40.0, 190.0}}) {
		CAPTURE(speed);
		CAPTURE(radius);
		auto const result{freight_crash(speed, radius)};
		MESSAGE(std::format("{:.0f} km/h, R {:.0f}: {} derailed ({} overrode, {} pushed off), {} ends crushed (deepest {:.2f} m), {} couplings broken: {}", speed, radius, result.derailed,
		                    result.overridden, result.pushed, result.crushed, result.crush, result.broken, result.list));
		if (speed <= 10.0) {
			// the buffers take it
			CHECK(result.derailed == 0);
			CHECK(result.crushed == 0);
		}
	}
}

TEST_CASE("collisions: identical units built to EN 15227 at 36 km/h") {
	model_setting const physical{global_settings::derailment_model::physical};
	flag_setting const crashdamage{Global.crash_damage, true};
	// two coaches of data/wagon.json (42.2 t) a unit, their ends crash zones with anticlimbers, unbraked
	auto const unit = [](bool const Anticlimbers, double const Height) {
		return definition::load("wagon").set("BuffCoupl", "CrashZone", "Yes").set("BuffCoupl", "AntiClimber", Anticlimbers ? "Yes" : "No").set("BuffCoupl", "BufferHeight", Height);
	};
	auto const crash = [&](bool const Anticlimbers, double const Speed, double const Offset) {
		consist train;
		train.add(unit(Anticlimbers, 1.040));
		train.add(unit(Anticlimbers, 1.040));
		train.add(unit(Anticlimbers, 1.040 + Offset), 0, coupling::faux, Speed, 1.0);
		train.add(unit(Anticlimbers, 1.040 + Offset), 0, consist::full_coupling, Speed);
		train.run(6.0);
		struct {
			double crush;
			double inner;
			int derailed;
			bool higher;
		} result{train[1]->Couplers[end::rear].crush + train[2]->Couplers[end::front].crush, std::max(crush_of(train[0].mover()), crush_of(train[3].mover())), 0,
		         derailed(train[2].mover())};
		for (std::size_t i = 0; i < train.size(); ++i) {
			result.derailed += derailed(train[i].mover());
		}
		return result;
	};
	SUBCASE("scenario 1: the crush stays in the zones, nothing climbs") {
		auto const result{crash(true, 36.0, 0.04)};
		MESSAGE("36 km/h, 40 mm offset: crush ", result.crush, " m, the far ends ", result.inner, " m");
		CHECK(result.crush > 0.2);
		CHECK(result.crush < 2.0 * 0.8);
		CHECK(result.derailed == 0);
	}
	SUBCASE("the same units without the anticlimbers override") {
		auto const result{crash(false, 36.0, 0.0)};
		CHECK(result.derailed >= 2);
	}
	SUBCASE("slower, without the anticlimbers, the higher one climbs") {
		auto const result{crash(false, 32.0, 0.04)};
		MESSAGE("32 km/h, 40 mm offset: crush ", result.crush, " m");
		CHECK(result.crush > cl::override_crush);
		CHECK(result.crush < cl::override_both);
		CHECK(result.higher);
		CHECK(result.derailed == 1);
		auto const held{crash(true, 32.0, 0.04)};
		CHECK(held.derailed == 0);
	}
}
