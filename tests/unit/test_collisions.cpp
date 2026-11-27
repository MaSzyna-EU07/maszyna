#include "stdafx.h"
#include "doctest.h"
#include "consist.h"
#include "drive.h"
#include "vehicle.h"
#include "physics/collision.h"
#include "physics/mover.h"
#include "utilities/Globals.h"

#include <cmath>
#include <format>
#include <string>
#include <vector>

// collisions past the buffers with physics.collisions new (physics/collision.h): the crush of the ends, the override,
// the push off the track, and what they do in trains of real vehicles

using test::consist;
using test::definition;
using test::vehicle;
namespace cl = collision;

namespace {

constexpr double kmh{1.0 / 3.6};

// the given model of collisions for the test, restored after it
class model_setting {
public:
	explicit model_setting(global_settings::collision_model Model) : m_previous{Global.collisions} { Global.collisions = Model; }
	~model_setting() { Global.collisions = m_previous; }
	model_setting(model_setting const &) = delete;
	model_setting &operator=(model_setting const &) = delete;

private:
	global_settings::collision_model m_previous;
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

// the energy the crush of both ends of the pair took, by their curves [J]
double crush_energy(TMoverParameters const &First, int const FirstEnd, TMoverParameters const &Second, int const SecondEnd) {
	return First.crush_curve_at(FirstEnd).energy(0.0, First.Couplers[FirstEnd].crush) + Second.crush_curve_at(SecondEnd).energy(0.0, Second.Couplers[SecondEnd].crush);
}

// what the buffers of EN 15551 of the two ends take in [J]
double capacity_of(TMoverParameters const &First, int const FirstEnd, TMoverParameters const &Second, int const SecondEnd) {
	return cl::buffer_capacity(First.Couplers[FirstEnd].collision, First.collision_kind()) + cl::buffer_capacity(Second.Couplers[SecondEnd].collision, Second.collision_kind());
}

// the kinetic energy of the relative motion of two vehicles closing in at the velocity [J]
double relative_energy(TMoverParameters const &First, TMoverParameters const &Second, double const Velocity) {
	auto const mass{First.TotalMass * Second.TotalMass / (First.TotalMass + Second.TotalMass)};
	return 0.5 * mass * Velocity * Velocity;
}

struct contact_outcome {
	double crush{0.0}; // of both ends [m]
	double energy{0.0}; // the crush took [J]
	double available{0.0}; // the kinetic energy of the hit past the buffers of EN 15551 [J]
	bool derailed{false}; // either vehicle
};

// the second vehicle runs into the rear end of the first one standing, Gap metres short of the hit (negative: the
// buffers pressed that far already), unbraked and alone on the track
contact_outcome contact(definition const &Front, definition const &Back, double const Speed, double const Gap, double const Seconds = 5.0, double const Dt = 0.01) {
	consist train;
	auto &front{train.add(Front)};
	auto &back{train.add(Back, 0, coupling::faux, Speed, Gap)};
	train.run(Seconds, {}, Dt);
	contact_outcome result;
	result.crush = front->Couplers[end::rear].crush + back->Couplers[end::front].crush;
	result.energy = crush_energy(front.mover(), end::rear, back.mover(), end::front);
	result.available = std::max(0.0, relative_energy(front.mover(), back.mover(), Speed * kmh) - capacity_of(front.mover(), end::rear, back.mover(), end::front));
	result.derailed = derailed(front.mover()) || derailed(back.mover());
	return result;
}

// a coach of data/wagon.json of the given mass without its rotating masses
definition coach(double const Mass) {
	return definition::load("wagon").set("Param", "M", Mass);
}

// Y / Q of the leading wheel of the vehicle at the end pressed in by its coupling, as check_pushed_off() takes it
double pushed_ratio_of(TMoverParameters const &Vehicle, int const End, double const OtherLength, double const Radius) {
	cl::pushed_vehicle pushed;
	pushed.mass = Vehicle.TotalMass - Vehicle.Mred;
	pushed.axles = Vehicle.NAxles > 0 ? Vehicle.NAxles : 4;
	pushed.length = Vehicle.Dim.L;
	pushed.pivots = Vehicle.BDist;
	pushed.axle_base = Vehicle.ADist > 0.0 ? Vehicle.ADist : Vehicle.BDist;
	cl::pushed_end end;
	end.force = std::max(0.0, (End == end::front ? -1.0 : 1.0) * Vehicle.Couplers[End].CForce);
	end.other_length = OtherLength;
	end.crush = Vehicle.Couplers[End].crush;
	return cl::pushed_ratio(pushed, end, Radius);
}

// a loaded 430W coal wagon
vehicle &add_loaded(consist &Train, int const Coupling, double const Velocity, double const Gap) {
	auto &wagon{Train.add(definition::fiz("430w_v2/430w"), 0, Coupling, Velocity, Gap)};
	wagon->AssignLoad("coal", 59.5f);
	wagon->ComputeMass();
	return wagon;
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

	// past 3 MN on a straight track the push takes the whole load off its leading wheels: they lift off the rail
	CHECK_FALSE(cl::pushed_forces(wagon, end, 0.0).lifted());
	end.force = 3.5e6;
	CHECK(cl::pushed_forces(wagon, end, 0.0).lifted());
	CHECK(cl::pushed_ratio(wagon, end, 0.0) > 1e8);
}

TEST_CASE("collisions: the flange climbs while the wheels roll on, a lifted wheel goes in a short time") {
	SUBCASE("Y / Q over the limit over 2 m rolled, however long the steps") {
		cl::pushed_monitor monitor;
		CHECK_FALSE(monitor.update(true, 0.7, false, 0.03));
		CHECK_FALSE(monitor.update(true, 0.7, false, 0.03));
		CHECK(monitor.distance() == doctest::Approx(1.4));
		CHECK(monitor.update(true, 0.7, false, 0.03));
	}
	SUBCASE("a break starts it over") {
		cl::pushed_monitor monitor;
		CHECK_FALSE(monitor.update(true, 1.5, false, 0.03));
		CHECK_FALSE(monitor.update(false, 0.1, false, 0.03));
		CHECK_FALSE(monitor.update(true, 1.5, false, 0.03));
		CHECK(monitor.update(true, 0.5, false, 0.03));
	}
	SUBCASE("standing, it never climbs, however long the push holds") {
		cl::pushed_monitor monitor;
		auto climbed{false};
		for (int step = 0; step < 100000; ++step) {
			climbed = climbed || monitor.update(true, 0.0, false, 0.01);
		}
		CHECK_FALSE(climbed);
		CHECK(monitor.distance() == 0.0);
	}
	SUBCASE("the wheels lifted off the rail for 0.05 s, standing or not") {
		cl::pushed_monitor monitor;
		CHECK_FALSE(monitor.update(true, 0.0, true, 0.03));
		CHECK_FALSE(monitor.update(false, 0.0, false, 0.01));
		CHECK_FALSE(monitor.update(true, 0.0, true, 0.03));
		CHECK(monitor.update(true, 0.0, true, 0.03));
	}
	SUBCASE("a reset clears both") {
		cl::pushed_monitor monitor;
		monitor.update(true, 1.9, true, 0.04);
		monitor.reset();
		CHECK(monitor.distance() == 0.0);
		CHECK(monitor.time() == 0.0);
		CHECK_FALSE(monitor.update(true, 0.05, true, 0.005));
	}
}

TEST_CASE("collisions: a locomotive at its starting effort against a braked train doesn't push the empty wagon off at a stand") {
	// the reviewer's case: an ST44 pressing a rake with an empty two-axle wagon right in front of it, in a yard curve.
	// the push is over the climbing limit of the empty wagon, but nothing rolls: a wheel which doesn't roll doesn't
	// climb. the time criterion pushed it off in five steps of 0.01 s. the ST44 gives some 390 kN, at the climbing
	// limit of the empty wagon in a curve of 150 m: the yard curve is one of 120 m, to be past it
	model_setting const physical{global_settings::collision_model::physical};
	flag_setting const crashdamage{Global.crash_damage, true};
	test::track line;
	line.radius = 120.0;
	consist train;
	constexpr int wagons{4};
	for (int i = 0; i < wagons; ++i) {
		add_loaded(train, consist::full_coupling, 0.0, 0.0).hold_brake_pipe(5.0);
	}
	auto &empty{train.add(definition::fiz("3wl_v2/3wl"))};
	empty.hold_brake_pipe(5.0);
	train.run(90.0, line);
	for (std::size_t i = 0; i < train.size(); ++i) {
		train[i].hold_brake_pipe(0.0);
	}
	train.run(30.0, line);
	auto &locomotive{train.add(definition::fiz("st44_v2/m62-1151"), 1, coupling::faux, 0.0, 0.0)};
	test::driven const driver{locomotive.mover(), locomotive.mover(), locomotive.mover(), [&train](double const Seconds, test::track const &Track) { train.run(Seconds, Track); }};
	REQUIRE(test::start_up(driver, line));
	test::release_brakes(locomotive.mover());
	for (int i = 0; i < 3 && locomotive->DirActive <= 0; ++i) {
		locomotive->DirectionForward();
	}
	test::drive(driver, line, 10.0);
	auto const start{empty->DistCounter};
	auto pushed{0.0};
	auto worst{0.0};
	auto const watch = [&]() {
		pushed = std::max(pushed, -locomotive->Couplers[end::front].CForce);
		worst = std::max(worst, pushed_ratio_of(empty.mover(), end::rear, locomotive->Dim.L, line.radius));
	};
	// the line contactors close at the first position, once their delay and the pressure switch let them
	locomotive->IncMainCtrl(1);
	for (int i = 0; i < 90 && false == locomotive->StLinFlag; ++i) {
		test::drive(driver, line, 1.0);
	}
	for (int i = 0; i < 60 && false == locomotive->IsMainCtrlMaxPowerPos(); ++i) {
		locomotive->IncMainCtrl(1);
		test::drive(driver, line, 1.0);
		watch();
	}
	for (int i = 0; i < 60; ++i) {
		test::drive(driver, line, 1.0);
		watch();
	}
	auto const rolled{(empty->DistCounter - start) * 1000.0};
	MESSAGE(std::format("ST44 pushing at up to {:.0f} kN, the empty 3WL at Y/Q {:.2f}, rolled {:.3f} m", pushed / 1000.0, worst, rolled));
	REQUIRE(worst >= derailment::climbing_limit);
	CHECK(rolled < 0.5);
	CHECK_FALSE(derailed(empty.mover()));
	CHECK(empty->CollisionDerail == NONE);
}

TEST_CASE("collisions: a locomotive running onto an empty wagon at 9 km/h doesn't push it off with the peak of the springs") {
	// the reviewer's case: an 80 t locomotive onto a free empty 3WL in a curve of 150 m. the springs of the buffers
	// press past the climbing limit for some 0.06 s, the wagon rolls a few centimetres meanwhile
	model_setting const physical{global_settings::collision_model::physical};
	flag_setting const crashdamage{Global.crash_damage, true};
	test::track curve;
	curve.radius = 150.0;
	consist train;
	auto &empty{train.add(definition::fiz("3wl_v2/3wl"))};
	auto &locomotive{train.add(definition::fiz("4e_v1/4e"), 0, coupling::faux, 9.0, 0.5)};
	auto pushed{0.0};
	auto worst{0.0};
	auto over{0}; // steps past the climbing limit
	for (int step = 0; step < 500; ++step) {
		train.step(curve);
		pushed = std::max(pushed, empty->Couplers[end::rear].CForce);
		auto const ratio{pushed_ratio_of(empty.mover(), end::rear, locomotive->Dim.L, curve.radius)};
		worst = std::max(worst, ratio);
		over += ratio >= derailment::climbing_limit;
	}
	MESSAGE(std::format("EU07 at 9 km/h onto the empty 3WL: up to {:.0f} kN, Y/Q {:.2f}, {} steps of 0.01 s past the limit, the wagon at {:.1f} km/h", pushed / 1000.0, worst, over,
	                    empty.speed()));
	REQUIRE(worst >= derailment::climbing_limit);
	REQUIRE(over >= 5);
	CHECK_FALSE(derailed(empty.mover()));
	CHECK_FALSE(derailed(locomotive.mover()));
}

TEST_CASE("collisions: a train propelling an empty wagon into a braked one in a curve pushes it off as it rolls on") {
	// the genuine push off: the empty 3WL leading a heavy train at 15 km/h, the train pressing it on into a braked
	// rake through the curve while it all slides along
	model_setting const physical{global_settings::collision_model::physical};
	flag_setting const crashdamage{Global.crash_damage, true};
	test::track curve;
	curve.radius = 150.0;
	consist train;
	constexpr int standing{4};
	for (int i = 0; i < standing; ++i) {
		add_loaded(train, consist::full_coupling, 0.0, 0.0).hold_brake_pipe(5.0);
	}
	train.run(90.0, curve);
	for (int i = 0; i < standing; ++i) {
		train[i].hold_brake_pipe(0.0);
	}
	train.run(30.0, curve);
	auto &empty{train.add(definition::fiz("3wl_v2/3wl"), 0, coupling::faux, 15.0, 1.0)};
	train.add(definition::fiz("4e_v1/4e"), 0, consist::full_coupling, 15.0, 0.0);
	for (int i = 0; i < 6; ++i) {
		add_loaded(train, consist::full_coupling, 15.0, 0.0);
	}
	auto const start{empty->DistCounter};
	train.run(10.0, curve);
	MESSAGE(std::format("the empty 3WL: derailed {}, {}, rolled {:.2f} m", derailed(empty.mover()), empty->CollisionDerail == COLLISION_PUSHED_OFF ? "pushed off" : "not pushed off",
	                    (empty->DistCounter - start) * 1000.0));
	CHECK(empty->CollisionDerail == COLLISION_PUSHED_OFF);
}

// ---- recovery

TEST_CASE("collisions: a repaired vehicle has its ends rebuilt and is back on the rails") {
	// resetconsist: the crush of the ends shifted the stop of the buffers, the next hit of the repaired consist would
	// have started from it; and a vehicle derailed in a collision kept its reason and the state of the push
	model_setting const physical{global_settings::collision_model::physical};
	flag_setting const crashdamage{Global.crash_damage, true};
	consist train;
	auto &wagon{add_loaded(train, coupling::faux, 0.0, 0.0)};
	auto &locomotive{train.add(definition::fiz("4e_v1/4e"), 0, coupling::faux, 30.0, 0.5)};
	train.run(3.0);
	REQUIRE(wagon->Couplers[end::rear].crush + locomotive->Couplers[end::front].crush > cl::override_crush);
	REQUIRE((derailed(wagon.mover()) || derailed(locomotive.mover())));
	for (auto *vehicle : {&wagon, &locomotive}) {
		auto &mover{vehicle->mover()};
		mover.repair_damage();
		CHECK(mover.DamageFlag == 0);
		CHECK(mover.Derailed == derailment::outcome::none);
		CHECK(mover.CollisionDerail == NONE);
		CHECK(mover.pushed_off_monitor.distance() == 0.0);
		CHECK(mover.pushed_off_monitor.time() == 0.0);
		for (auto const &coupler : mover.Couplers) {
			CHECK(coupler.crush == 0.0);
			CHECK(coupler.contact.absorbed() == 0.0);
			CHECK(coupler.contact.crush_work() == 0.0);
			CHECK_FALSE(coupler.hit_handled);
			CHECK(coupler.hit_force == 0.0);
			CHECK_FALSE(coupler.crush_monitored);
			CHECK_FALSE(coupler.inside_reported);
		}
	}
}

TEST_CASE("collisions: the repair of a coupler torn off by the crush rebuilds its end") {
	vehicle wagon{definition::fiz("430w_v2/430w")};
	auto &coupler{wagon->Couplers[end::rear]};
	coupler.crush = 0.3;
	coupler.broken = true;
	coupler.hit_force = 2.0e6;
	SetFlag(wagon->DamageFlag, dtrain_coupling);
	REQUIRE(wagon->repair_coupler(end::rear));
	CHECK(coupler.crush == 0.0);
	CHECK(coupler.hit_force == 0.0);
	CHECK_FALSE(coupler.broken);
	CHECK_FALSE(TestFlag(wagon->DamageFlag, dtrain_coupling));
}

TEST_CASE("collisions: SetDamage taking the derailment off clears the collision's") {
	vehicle wagon{definition::fiz("3wl_v2/3wl")};
	wagon->derail_in_collision(COLLISION_PUSHED_OFF, "test");
	REQUIRE(wagon->CollisionDerail == COLLISION_PUSHED_OFF);
	REQUIRE(wagon->Derailed == derailment::outcome::off_rails);
	wagon->RunCommand(VehicleCommand::setDamage, dtrain_out, -1);
	CHECK(wagon->CollisionDerail == NONE);
	CHECK(wagon->Derailed == derailment::outcome::none);
	CHECK_FALSE(TestFlag(wagon->DamageFlag, dtrain_out));
}

// ---- the model of eu07.ini

TEST_CASE("collisions: the legacy rule with physics.collisions old, the crush with new") {
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
		int monitored; // the hits the monitor tells of
	};
	// the legacy rule: over 15 km/h weighed by the ratio of the masses. the crush: the light wagon is crushed deep and
	// climbs onto the locomotive at 60 km/h; two locomotives at 25 km/h crush their ends 0.1-0.3 m and stay on the rails
	// the monitor tells of each hit once: the wagon derailed by the legacy rule is a wall the locomotive presses on into
	// until it springs back, the same hit (the wall slowing down took the pair from parting at the start of a step to
	// closing in at 0.3 km/h, which was told of as a second hit); the locomotives derailed both stay where they are
	for (auto const &[name, front, back, speed, frontout_old, backout_old, frontout_new, backout_new, monitored] : {
	         collision{"locomotive into a wagon at 60", "3wl_v2/3wl", "4e_v1/4e", 60.0, true, false, true, false, 1},
	         collision{"locomotive into a locomotive at 25", "4e_v1/4e", "4e_v1/4e", 25.0, true, true, false, false, 1},
	         collision{"locomotive into a locomotive at 50", "4e_v1/4e", "4e_v1/4e", 50.0, true, true, true, true, 1},
	     }) {
		CAPTURE(name);
		// monitor: the legacy rule, and the log tells of the crush once
		for (auto const model : {global_settings::collision_model::old, global_settings::collision_model::physical, global_settings::collision_model::monitor}) {
			auto const physical{model == global_settings::collision_model::physical};
			CAPTURE(name_of(model));
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
			CHECK(train[0]->crush_reports + train[1]->crush_reports == (model == global_settings::collision_model::monitor ? monitored : 0));
		}
	}
}

TEST_CASE("collisions: the names of the models in eu07.ini") {
	using model = global_settings::collision_model;
	CHECK(collision_model_of("new") == model::physical);
	CHECK(collision_model_of("old") == model::old);
	CHECK(collision_model_of("monitor") == model::monitor);
	// a name it doesn't know is the default
	CHECK(collision_model_of("physical") == model::physical);
	CHECK(global_settings{}.collisions == model::physical);
	for (auto const value : {model::old, model::physical, model::monitor}) {
		CHECK(collision_model_of(name_of(value)) == value);
	}
	CHECK(name_of(model::physical) == "new");
}

TEST_CASE("collisions: the model of the collisions goes apart from the one of the derailment") {
	// the crush with the legacy draw of the derailment in a curve, the legacy collisions with the criteria
	flag_setting const crashdamage{Global.crash_damage, true};
	for (auto const derailment : {global_settings::derailment_model::old, global_settings::derailment_model::physical}) {
		for (auto const collisions : {global_settings::collision_model::old, global_settings::collision_model::physical}) {
			auto const previous{Global.derailment};
			Global.derailment = derailment;
			model_setting const setting{collisions};
			consist train;
			train.add(definition::fiz("4e_v1/4e"));
			train.add(definition::fiz("4e_v1/4e"), 0, coupling::faux, 25.0, 20.0);
			train.run(10.0);
			Global.derailment = previous;
			auto const crush{train[0]->Couplers[end::rear].crush + train[1]->Couplers[end::front].crush};
			CHECK((crush > 0.05) == (collisions == global_settings::collision_model::physical));
		}
	}
}

TEST_CASE("collisions: the crush of a locomotive hitting a wagon doesn't depend on the step, nor on the model of the coupling") {
	model_setting const physical{global_settings::collision_model::physical};
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
	model_setting const physical{global_settings::collision_model::physical};
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
	// derail, as with physics.collisions old (found running line 107: two cars of its traffic, 119 km/h head-on)
	flag_setting const crashdamage{Global.crash_damage, true};
	for (auto const model : {global_settings::collision_model::old, global_settings::collision_model::physical, global_settings::collision_model::monitor}) {
		model_setting const setting{model};
		consist road;
		road.add(definition::fiz("road/man/nl293"));
		road.add(definition::fiz("road/man/nl293"), 0, coupling::faux, 40.0, 5.0);
		road.run(5.0);
		CHECK(derailed(road[0].mover()));
		CHECK(derailed(road[1].mover()));
		CHECK(road[0]->Couplers[end::rear].crush == 0.0);
		CHECK(road[1]->CollisionDerail == NONE);
		CHECK(road[0]->crush_reports + road[1]->crush_reports == 0);
	}
}

TEST_CASE("collisions: a vehicle off the rails by the new model stays in the chain of forces") {
	model_setting const physical{global_settings::collision_model::physical};
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
	model_setting const physical{global_settings::collision_model::physical};
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
	model_setting const physical{global_settings::collision_model::physical};
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

// ---- the energy of the hit: what the buffers of EN 15551 take in never crushes (found running every scenario of the
// install: calkowo_v2_kontenera, baltyk_sdm)

TEST_CASE("collisions: the buffers of EN 15551 of an end take in their energy, of the category given or by the kind") {
	CHECK(cl::buffer_capacity({}, cl::vehicle_kind::wagon) == doctest::Approx(60.0e3));
	CHECK(cl::buffer_capacity({}, cl::vehicle_kind::locomotive) == doctest::Approx(60.0e3));
	CHECK(cl::buffer_capacity({}, cl::vehicle_kind::coach) == doctest::Approx(60.0e3));
	CHECK(cl::default_buffers(cl::vehicle_kind::coach) == cl::buffer_category::passenger);
	cl::end_definition category_c;
	category_c.buffers = cl::buffer_category::C;
	CHECK(cl::buffer_capacity(category_c, cl::vehicle_kind::wagon) == doctest::Approx(140.0e3));
	// the key of the .fiz
	vehicle wagon{definition::load("wagon").set("BuffCoupl", "BufferCategory", "B")};
	REQUIRE(wagon->Couplers[end::front].collision.buffers.has_value());
	CHECK(*wagon->Couplers[end::front].collision.buffers == cl::buffer_category::B);
	CHECK(cl::buffer_capacity(wagon->Couplers[end::rear].collision, wagon->collision_kind()) == doctest::Approx(100.0e3));
}

TEST_CASE("collisions: the budget of a contact, by what the hit brings to the buffers") {
	cl::contact_energy contact;
	// not touching: nothing
	contact.update(false, false, 1.0, 0.0, 0.0, 0.01);
	CHECK(contact.absorbed() == 0.0);
	// pressed in, closing: the springs take in their work
	contact.update(true, false, 1.0, 1.0e6, 0.05, 0.01);
	CHECK(contact.absorbed() == doctest::Approx(10.0e3));
	// pressed all the way, they take in nothing more
	contact.update(true, true, 1.0, 1.4e6, 0.3, 0.01);
	CHECK(contact.absorbed() == doctest::Approx(10.0e3));
	// the kinetic energy and what the buffers took, past their capacity
	CHECK(contact.budget(40.0e3, 2.0, 60.0e3) == doctest::Approx(0.5 * 40.0e3 * 4.0 + 10.0e3 - 60.0e3));
	// the springs of the .fiz taking in more than the buffers of EN 15551: no more than the kinetic energy
	CHECK(contact.budget(40.0e3, 2.0, 5.0e3) == doctest::Approx(0.5 * 40.0e3 * 4.0));
	// springing back they hold no more than their springs pressed in
	contact.update(true, false, -0.5, 0.2e6, 0.02, 0.01);
	CHECK(contact.absorbed() == doctest::Approx(0.5 * 0.2e6 * 0.02));
	// a train pushing at a stand: the oscillation of the pair never builds the energy up
	for (int i = 0; i < 6000; ++i) {
		auto const closing{0.05 * std::sin(i * 0.3)};
		contact.update(true, false, closing, 0.3e6 + 1.0e6 * closing, 0.03, 0.01);
	}
	CHECK(contact.absorbed() < 0.5 * 0.35e6 * 0.03 + 0.35e6 * 0.05 * 0.01 * 12.0);
	// parted: a new contact
	contact.crushed(5.0e3);
	CHECK(contact.crush_work() == doctest::Approx(5.0e3));
	contact.update(false, false, 0.0, 0.0, 0.0, 0.01);
	CHECK(contact.absorbed() == 0.0);
	CHECK(contact.crush_work() == 0.0);
}

TEST_CASE("collisions: the crush of a hit takes no more than its energy past the buffers, with any step") {
	// two masses at the stop of their buffers from the start (nothing taken in by the springs), the budget of the
	// crush taken anew in every step of the hit, as the vehicles take it
	struct masses {
		double mass1;
		double mass2;
	};
	constexpr double capacity{120.0e3};
	constexpr double springs{1.2e6};
	for (auto const &[mass1, mass2] : {masses{80.0e3, 80.0e3}, masses{117.0e3, 84.0e3}, masses{300.0e3, 20.0e3}, masses{36.0e3, 41.6e3}}) {
		auto const reduced{mass1 * mass2 / (mass1 + mass2)};
		for (auto const speed : {2.0, 5.0, 8.0, 12.0, 20.0, 30.0, 45.0, 60.0}) {
			for (auto const dt : {0.001, 0.003, 0.01, 0.02, 0.03}) {
				CAPTURE(mass1);
				CAPTURE(mass2);
				CAPTURE(speed);
				CAPTURE(dt);
				cl::end_state first{locomotive_curve, 0.0};
				cl::end_state second{wagon_curve, 0.0};
				auto velocity1{speed * kmh};
				auto velocity2{0.0};
				auto work{0.0};
				for (int step = 0; step < 100000; ++step) {
					auto const start{velocity1 - velocity2};
					velocity1 -= springs * dt / mass1;
					velocity2 += springs * dt / mass2;
					auto const closing{velocity1 - velocity2};
					if (closing <= 0.0) {
						break;
					}
					cl::hit_input input;
					input.velocity = closing;
					input.start_velocity = start;
					input.mass1 = mass1;
					input.mass2 = mass2;
					input.stiffness = 1.4e8;
					input.springs = springs;
					input.dt = dt;
					input.budget = 0.5 * reduced * start * start - capacity;
					auto const hit{cl::hit(first, second, input)};
					if (false == hit.crushing) {
						break;
					}
					CHECK(hit.work <= input.budget * (1.0 + 1e-9) + 1e-6);
					work += hit.work;
					first.crush += hit.crush1;
					second.crush += hit.crush2;
					velocity1 -= hit.impulse / mass1;
					velocity2 += hit.impulse / mass2;
					if (hit.velocity <= 0.0) {
						break;
					}
				}
				auto const available{std::max(0.0, 0.5 * reduced * (speed * kmh) * (speed * kmh) - capacity)};
				auto const crushed{locomotive_curve.energy(0.0, first.crush) + wagon_curve.energy(0.0, second.crush)};
				CHECK(work <= available + 1.0);
				CHECK(crushed <= available * 1.001 + 1.0);
				if (available <= 0.0) {
					CHECK(first.crush + second.crush == 0.0);
				}
			}
		}
	}
}

TEST_CASE("collisions: the crush of two vehicles takes no more than the energy of the hit past the buffers, with any step") {
	model_setting const physical{global_settings::collision_model::physical};
	flag_setting const crashdamage{Global.crash_damage, true};
	struct pair_of {
		char const *name;
		definition front;
		definition back;
	};
	for (auto const &[name, front, back] : {pair_of{"EU07 into EU07", definition::fiz("4e_v1/4e"), definition::fiz("4e_v1/4e")},
	                                        pair_of{"SU45 into SM42", definition::fiz("sm42_v1/6d1"), definition::fiz("su45_v2/301d")},
	                                        pair_of{"EU07 into a wagon", definition::fiz("430w_v2/430w"), definition::fiz("4e_v1/4e")}}) {
		for (auto const speed : {2.0, 5.0, 10.0, 15.0, 20.0, 30.0, 45.0, 60.0}) {
			for (auto const dt : {0.001, 0.01, 0.03}) {
				CAPTURE(name);
				CAPTURE(speed);
				CAPTURE(dt);
				auto const result{contact(front, back, speed, 0.5, 4.0, dt)};
				// the explicit steps of the springs before the stop rate what they took in a little off. at 0.03 s, 0.5 m a
				// step at 60 km/h, the legacy movement of the vehicles past the stop gives the hit energy of its own (the
				// crush goes past what it is at 0.01 s and 0.001 s): the budget doesn't bind there, every step of the hit
				// is within it (the test of the hit of two masses above), the same crush as before the budget
				CHECK(result.energy <= result.available * (dt > 0.02 ? 1.15 : 1.05) + 5.0e3);
				if (result.available <= 0.0) {
					CHECK(result.crush == 0.0);
				}
			}
		}
	}
}

TEST_CASE("collisions: two locomotives (117 t and 84 t) touching at walking speed don't crush (calkowo_v2_kontenera)") {
	model_setting const physical{global_settings::collision_model::physical};
	flag_setting const crashdamage{Global.crash_damage, true};
	auto const su45{definition::fiz("su45_v2/301d")};
	auto const sm42{definition::fiz("sm42_v1/6d1")};
	{
		vehicle check{su45};
		CHECK(check->TotalMass == doctest::Approx(117.0e3).epsilon(0.02));
	}
	for (auto const physics : {false, true}) {
		CAPTURE(physics);
		flag_setting const couplerphysics{Global.coupler_physics, physics};
		flag_setting const everywhere{Global.coupler_physics_all, physics};
		for (auto const speed : {5.0, 5.8, 6.0}) {
			CAPTURE(speed);
			// a hit from a metre away
			{
				auto const result{contact(sm42, su45, speed, 1.0)};
				CHECK(result.crush == 0.0);
				CHECK_FALSE(result.derailed);
			}
			// put down by the scenery with the buffers pressed in, up to past their stop, closing in: the log of the run
			// had 5.8 km/h at the stop, both ends crushed, deeper hit after hit until both overrode
			for (auto const overlap : {0.1, 0.4, 1.0}) {
				CAPTURE(overlap);
				auto const result{contact(sm42, su45, speed, -overlap, 10.0)};
				CHECK(result.crush == 0.0);
				CHECK_FALSE(result.derailed);
			}
		}
	}
}

TEST_CASE("collisions: locomotives put down inside each other don't crush, whatever their springs make of them") {
	// calkowo_v2_kontenera puts an SM42 on an SU45 with their middles 2.7 m apart: the springs at the stops of both
	// ends and the neighbours found at either end shook them up to 9-13 km/h, past the energy of the buffers
	model_setting const physical{global_settings::collision_model::physical};
	flag_setting const crashdamage{Global.crash_damage, true};
	for (auto const speed : {0.0, 10.0, 15.0}) {
		CAPTURE(speed);
		auto const result{contact(definition::fiz("sm42_v1/6d1"), definition::fiz("su45_v2/301d"), speed, -3.0, 10.0)};
		CHECK(result.crush == 0.0);
		CHECK_FALSE(result.derailed);
	}
	// a hit from a distance at the same speeds crushes past the energy of the buffers
	CHECK(contact(definition::fiz("sm42_v1/6d1"), definition::fiz("su45_v2/301d"), 20.0, 0.5).crush > 0.0);
}

TEST_CASE("collisions: two coaches (36 t and 41.6 t) at 8 km/h don't crush, nor run into a wall (baltyk_sdm)") {
	model_setting const physical{global_settings::collision_model::physical};
	flag_setting const crashdamage{Global.crash_damage, true};
	auto const light{coach(34.3e3)};
	auto const heavy{coach(39.9e3)};
	{
		vehicle check{light};
		CHECK(check->TotalMass == doctest::Approx(36.0e3).epsilon(0.01));
		CHECK(vehicle{heavy}->TotalMass == doctest::Approx(41.6e3).epsilon(0.01));
	}
	for (auto const speed : {5.0, 8.0}) {
		CAPTURE(speed);
		// a hit from a metre away, and one with the buffers pressed all the way already, as the coach knocked onto the
		// other one by the hit of the train on the vehicle derailed at the end of the track
		for (auto const gap : {1.0, -0.25}) {
			CAPTURE(gap);
			auto const result{contact(light, heavy, speed, gap)};
			CHECK(result.crush == 0.0);
			CHECK_FALSE(result.derailed);
		}
	}
	// the vehicle derailed the legacy way is a wall that the coach runs into
	for (auto const gap : {1.0, -0.25}) {
		CAPTURE(gap);
		consist train;
		auto &wall{train.add(light)};
		wall->Derail(END_OF_TRACK);
		REQUIRE(derailed(wall.mover()));
		auto &running{train.add(heavy, 0, coupling::faux, 5.0, gap)};
		train.run(5.0);
		CHECK(wall->Couplers[end::rear].crush + running->Couplers[end::front].crush == 0.0);
		CHECK(running->CollisionDerail == NONE);
	}
}

TEST_CASE("collisions: a locomotive pushing a standing braked train at full power doesn't crush") {
	model_setting const physical{global_settings::collision_model::physical};
	flag_setting const crashdamage{Global.crash_damage, true};
	test::track const line{0.15, 0.0, test::catenary_3kv};
	// the diesel-electric keeps its full power at a stand (an electric one trips its overload relays there)
	for (auto const *path : {"su45_v2/301d"}) {
		CAPTURE(path);
		consist train;
		// four loaded coal wagons, braked in emergency once their brakes are charged
		constexpr int wagons{4};
		for (int i = 0; i < wagons; ++i) {
			add_mixed(train, 0, consist::full_coupling, 0.0, 0.0).hold_brake_pipe(5.0);
		}
		train.run(90.0, line);
		for (int i = 0; i < wagons; ++i) {
			train[i].hold_brake_pipe(0.0);
		}
		train.run(30.0, line);
		// the locomotive behind them, touching, driven forwards into them at full power
		auto &locomotive{train.add(definition::fiz(path), 1, coupling::faux, 0.0, 0.0)};
		test::driven const driver{locomotive.mover(), locomotive.mover(), locomotive.mover(), [&train](double const Seconds, test::track const &Track) { train.run(Seconds, Track); }};
		REQUIRE(test::start_up(driver, line));
		test::release_brakes(locomotive.mover());
		for (int i = 0; i < 3 && locomotive->DirActive <= 0; ++i) {
			locomotive->DirectionForward();
		}
		test::drive(driver, line, 10.0);
		auto pushed{0.0};
		auto seconds{0}; // pushing with 100 kN or more
		auto const push = [&]() {
			auto const force{std::max(0.0, -locomotive->Couplers[end::front].CForce)};
			pushed = std::max(pushed, force);
			seconds += force >= 100.0e3;
		};
		// the line contactors close at the first position, once their delay and the pressure switch let them
		locomotive->IncMainCtrl(1);
		for (int i = 0; i < 90 && false == locomotive->StLinFlag; ++i) {
			test::drive(driver, line, 1.0);
		}
		for (int i = 0; i < 60 && false == locomotive->IsMainCtrlMaxPowerPos(); ++i) {
			locomotive->IncMainCtrl(1);
			test::drive(driver, line, 1.0);
			push();
		}
		for (int i = 0; i < 60; ++i) {
			test::drive(driver, line, 1.0);
			push();
		}
		MESSAGE(std::format("{}: pushing at up to {:.0f} kN, {} s at 100 kN or more, at {:.1f} km/h at the end", path, pushed / 1000.0, seconds, locomotive.speed()));
		CHECK(pushed > 200.0e3);
		CHECK(seconds >= 55);
		for (std::size_t i = 0; i < train.size(); ++i) {
			CAPTURE(i);
			CHECK(crush_of(train[i].mover()) == 0.0);
			CHECK_FALSE(derailed(train[i].mover()));
		}
	}
}

TEST_CASE("collisions: vehicles put down with their buffers pressed in don't crush") {
	model_setting const physical{global_settings::collision_model::physical};
	flag_setting const crashdamage{Global.crash_damage, true};
	for (auto const physics : {false, true}) {
		CAPTURE(physics);
		flag_setting const couplerphysics{Global.coupler_physics, physics};
		flag_setting const everywhere{Global.coupler_physics_all, physics};
		for (auto const overlap : {0.1, 0.3, 0.6}) {
			CAPTURE(overlap);
			for (auto const &[front, back] : {std::pair{definition::fiz("sm42_v1/6d1"), definition::fiz("su45_v2/301d")}, std::pair{coach(34.3e3), coach(39.9e3)},
			                                  std::pair{definition::fiz("3wl_v2/3wl"), definition::fiz("4e_v1/4e")}}) {
				auto const result{contact(front, back, 0.0, -overlap, 10.0)};
				CHECK(result.crush == 0.0);
				CHECK_FALSE(result.derailed);
			}
		}
	}
}

TEST_CASE("collisions: the monitor goes by the energy of the hit as well") {
	flag_setting const crashdamage{Global.crash_damage, true};
	model_setting const monitor{global_settings::collision_model::monitor};
	// a hit the buffers take in: the monitor tells of nothing
	for (auto const gap : {1.0, -0.4}) {
		CAPTURE(gap);
		consist train;
		train.add(definition::fiz("sm42_v1/6d1"));
		train.add(definition::fiz("su45_v2/301d"), 0, coupling::faux, 5.8, gap);
		train.run(5.0);
		CHECK(train[0]->crush_reports + train[1]->crush_reports == 0);
	}
	// one past them it does
	consist train;
	train.add(definition::fiz("4e_v1/4e"));
	train.add(definition::fiz("4e_v1/4e"), 0, coupling::faux, 25.0, 20.0);
	train.run(10.0);
	CHECK(train[0]->crush_reports + train[1]->crush_reports == 1);
}

// ---- the legacy derailment (physics.derailment old or monitor) in a train, with physics.collisions new

namespace {

// what the vehicles of the train did to each other
struct train_damage {
	double crush{0.0}; // the deepest crush of an end [m]
	double energy{0.0}; // the crush of all the ends took [J]
	int derailed{0}; // in a collision
	int broken{0}; // couplings torn off
	std::string list; // which vehicle, for the messages
};

train_damage damage_of(consist &Train) {
	train_damage result;
	for (std::size_t i = 0; i < Train.size(); ++i) {
		auto const &vehicle{Train[i].mover()};
		result.crush = std::max(result.crush, crush_of(vehicle));
		for (auto const end : {end::front, end::rear}) {
			result.energy += vehicle.crush_curve_at(end).energy(0.0, vehicle.Couplers[end].crush);
		}
		result.derailed += vehicle.CollisionDerail != NONE;
		if (i + 1 < Train.size()) {
			result.broken += vehicle.Couplers[end::rear].broken;
		}
		if (crush_of(vehicle) > 0.0 || vehicle.CollisionDerail != NONE) {
			result.list += std::format("{} crushed {:.2f} m{}; ", i, crush_of(vehicle), vehicle.CollisionDerail != NONE ? ", derailed" : "");
		}
	}
	return result;
}

// the kinetic energy of the vehicles of the train [J]
double kinetic_of(consist &Train) {
	auto energy{0.0};
	for (std::size_t i = 0; i < Train.size(); ++i) {
		energy += 0.5 * Train[i]->TotalMass * Train[i]->V * Train[i]->V;
	}
	return energy;
}

} // namespace

TEST_CASE("collisions: the cut of the speed of the legacy derailment is no hit inside the train (moczniki_eu06_eu07, tomaszewo_raj49843)") {
	// the legacy derailment takes 35 % of the speed of the vehicle off in a step: the coaches coupled behind it, at the
	// speed they had, ran into it at a third of the speed of the train, crushed their ends and rode up onto it
	model_setting const physical{global_settings::collision_model::physical};
	flag_setting const crashdamage{Global.crash_damage, true};
	for (auto const physics : {false, true}) {
		CAPTURE(physics);
		flag_setting const couplerphysics{Global.coupler_physics, physics};
		flag_setting const everywhere{Global.coupler_physics_all, physics};
		// an SU46 with its coaches at 95 km/h, as in moczniki_eu06_eu07; an electric locomotive at 48 km/h, as in
		// tomaszewo_raj49843
		for (auto const &[path, speed] : {std::pair{"su46_v2/303d", 95.0}, std::pair{"4e_v1/4e", 48.0}}) {
			CAPTURE(path);
			// too fast in a curve, the draw in the movement of the vehicle; off the end of the track, in its next step
			for (auto const reason : {TOO_HIGH_SPEED, END_OF_TRACK}) {
				CAPTURE(reason);
				consist train;
				auto &locomotive{train.add(definition::fiz(path), 0, coupling::faux, speed)};
				for (int i = 0; i < 3; ++i) {
					train.add(coach(41.0e3), 0, consist::full_coupling, speed);
				}
				train.run(2.0);
				if (reason == END_OF_TRACK) {
					locomotive.damage_track(dtrack_norail);
				}
				else {
					locomotive->Derail(reason);
				}
				train.step();
				REQUIRE(derailed(locomotive.mover()));
				// the train loses the speed with it
				for (std::size_t i = 1; i < train.size(); ++i) {
					CAPTURE(i);
					CHECK(train[i].speed() == doctest::Approx(locomotive.speed()).epsilon(0.03));
				}
				train.run(15.0);
				auto const damage{damage_of(train)};
				INFO(damage.list);
				CHECK(damage.crush < cl::crush_mark);
				CHECK(damage.derailed == 0);
				CHECK(damage.broken == 0);
			}
		}
	}
}

TEST_CASE("collisions: the train behind a vehicle that ran off the end of the track doesn't crush into it (l053-sluzba-2-night, l053_calkowo-sluzba-tlk-1, linia61_osobowy3)") {
	// the vehicle derailed the legacy way at the end of the track stopped as a wall the train behind it ran into: the
	// vehicles behind it pressed into each other past their stops, and crushed by more than the energy of the train
	model_setting const physical{global_settings::collision_model::physical};
	flag_setting const crashdamage{Global.crash_damage, true};
	for (auto const physics : {false, true}) {
		CAPTURE(physics);
		flag_setting const couplerphysics{Global.coupler_physics, physics};
		flag_setting const everywhere{Global.coupler_physics_all, physics};
		// pulled: an M62 leading wagons of 22 t at 30 km/h (l053_calkowo-sluzba-tlk-1, linia61_osobowy3); pushed: an SM42
		// at the back of wagons of 35 t at 33 km/h, the leading wagon goes off (l053-sluzba-2-night)
		for (auto const pushed : {false, true}) {
			CAPTURE(pushed);
			auto const speed{pushed ? 33.0 : 30.0};
			consist train;
			if (false == pushed) {
				train.add(definition::fiz("st44_v2/m62-1151"), 0, coupling::faux, speed);
			}
			constexpr int wagons{6};
			for (int i = 0; i < wagons; ++i) {
				auto &wagon{train.add(definition::fiz("430w_v2/430w"), 0, consist::full_coupling, speed)};
				if (pushed) {
					wagon->AssignLoad("coal", 12.0f);
					wagon->ComputeMass();
				}
			}
			if (pushed) {
				train.add(definition::fiz("sm42_v1/6d1"), 0, consist::full_coupling, speed);
			}
			train.run(2.0);
			auto const kinetic{kinetic_of(train)};
			train[0].damage_track(dtrack_norail);
			train.run(20.0);
			REQUIRE(derailed(train[0].mover()));
			auto const damage{damage_of(train)};
			INFO(damage.list);
			MESSAGE(std::format("{}: crush {:.3f} m, {:.0f} kJ of the {:.0f} kJ of the train", pushed ? "pushed" : "pulled", damage.crush, damage.energy / 1000.0, kinetic / 1000.0));
			CHECK(damage.crush < cl::crush_mark);
			CHECK(damage.energy <= kinetic);
			CHECK(damage.derailed == 0);
			CHECK(damage.broken == 0);
			for (std::size_t i = 1; i < train.size(); ++i) {
				CAPTURE(i);
				CHECK_FALSE(derailed(train[i].mover()));
			}
		}
	}
}

TEST_CASE("collisions: the buffers of vehicles found inside each other take in nothing for the budget of the crush") {
	cl::contact_energy contact;
	// pressed in at 2 m/s under 1 MN for 0.1 s: 200 kJ, past the 120 kJ the buffers of both ends take
	contact.update(true, false, 2.0, 1.0e6, 0.1, 0.1);
	REQUIRE(contact.absorbed() == doctest::Approx(200.0e3));
	CHECK(contact.budget(11.0e3, 1.7, 120.0e3) > 0.0);
	// found inside each other: what their springs made of it is no hit (l053_calkowo-sluzba-tlk-1: 163 kJ "in the
	// buffers" let a pair crush with 15 kJ of motion)
	contact.inside();
	CHECK(contact.absorbed() == 0.0);
	contact.update(true, false, 2.0, 1.0e6, 0.1, 0.1);
	CHECK(contact.absorbed() == 0.0);
	CHECK(contact.budget(11.0e3, 1.7, 120.0e3) < 0.0);
	// the buffers apart, the next contact is a hit again
	contact.update(false, false, 0.0, 0.0, 0.0, 0.01);
	contact.update(true, false, 2.0, 1.0e6, 0.1, 0.1);
	CHECK(contact.absorbed() == doctest::Approx(200.0e3));
}

TEST_CASE("collisions: a pair parting at the start of the step isn't crushed (l053_calkowo-sluzba-tlk-1, linia61_osobowy3)") {
	// the log had "closing at -7.1 km/h at the start of the step" on a crush of a pair knocked back onto each other in
	// the step: the legacy collision takes such a knock, a pair still closing in at the start of a step is a hit
	model_setting const physical{global_settings::collision_model::physical};
	flag_setting const crashdamage{Global.crash_damage, true};
	for (auto const parting : {true, false}) {
		CAPTURE(parting);
		consist train;
		auto &front{train.add(definition::fiz("430w_v2/430w"))};
		// behind it with the buffers pressed to their stop, moving back away from it, or standing
		auto &back{train.add(definition::fiz("430w_v2/430w"), 0, coupling::faux, parting ? -3.0 : 0.0, -0.3)};
		// one step, the way the consist steps it, with the vehicle behind knocked forwards onto the front one between
		// the forces and the movement, as a hit of another vehicle in the step does
		constexpr double dt{0.01};
		for (auto *vehicle : {&front, &back}) {
			vehicle->begin_step();
		}
		for (auto *vehicle : {&front, &back}) {
			vehicle->forces(dt);
		}
		REQUIRE((front->Couplers[end::rear].closing < 0.0) == parting);
		back->V = 25.0 * kmh;
		for (auto *vehicle : {&front, &back}) {
			vehicle->move({}, dt, (*vehicle)->Loc, 5.0);
		}
		auto const crush{front->Couplers[end::rear].crush + back->Couplers[end::front].crush};
		if (parting) {
			CHECK(crush == 0.0);
			// the legacy collision pushed them apart
			CHECK(back->V < 25.0 * kmh);
		}
		else {
			// closing in from a stand, the knock is a hit past the energy of the buffers
			CHECK(crush > 0.0);
		}
	}
}
