#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
#include "McZapkie/couplerphysics.h"
#include "utilities/Globals.h"

#include <algorithm>
#include <cmath>

// the physical model of the couplings (McZapkie/couplerphysics.h, couplerphysics in eu07.ini): progressive buffers and
// draw gear with friction hysteresis, pressed in series, the play of the coupling, its yield and its breaking

using test::definition;
using test::vehicle;
namespace cp = coupler_physics;

namespace {

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

// one end of the wagon of data/wagon.json as the model takes it, with the play of a loose screw coupling
cp::end_parameters wagon_end() {
	cp::end_parameters wagon;
	wagon.buffers = {9.2e6, 0.11, 0.5};
	wagon.draw_gear = {4.5e6, 0.1, 0.3};
	wagon.slack = 0.05;
	wagon.buffers_unloading = 0.58;
	wagon.draw_gear_unloading = 0.58;
	wagon.link_stiffness = 850e3 / 0.003;
	wagon.crush_force = 2125e3;
	wagon.link = {680e3, 850e3, 170e3 / 0.02, true};
	return wagon;
}

// the force between two such ends at the gap, the state of the pair fresh
cp::pair_result fresh_pair(double Gap, bool Coupled = true, cp::end_parameters const &End = wagon_end()) {
	cp::pair_state state;
	return cp::update_pair(End, End, state, {Gap, 0.0, Coupled, false, true});
}

} // namespace

TEST_CASE("coupler physics: a progressive spring is soft at first and reaches its end force at the end of its travel") {
	cp::spring_curve const buffers{9.2e6, 0.11, 0.5};
	CHECK(buffers.force(0.0) == 0.0);
	CHECK(buffers.force(0.11) == doctest::Approx(buffers.end_force()));
	CHECK(buffers.end_force() == doctest::Approx(1012e3));
	// held at the end of the travel
	CHECK(buffers.force(0.2) == doctest::Approx(buffers.end_force()));
	CHECK(buffers.force(0.055) < 0.5 * buffers.end_force());
	CHECK(buffers.energy(0.11) == doctest::Approx(1012e3 * 0.11 * (0.5 - 0.5 / 4.0)));
	for (auto const deflection : {0.001, 0.01, 0.05, 0.1}) {
		CAPTURE(deflection);
		CHECK(buffers.deflection(buffers.force(deflection)) == doctest::Approx(deflection));
	}
	CHECK(buffers.deflection(2.0 * buffers.end_force()) == doctest::Approx(0.11));

	cp::spring_curve const linear{9.2e6, 0.11, 0.0};
	CHECK(linear.force(0.05) == doctest::Approx(9.2e6 * 0.05));
	CHECK(linear.deflection(9.2e6 * 0.05) == doctest::Approx(0.05));
}

TEST_CASE("coupler physics: the devices of both ends carry the same force, their deflections add up") {
	SUBCASE("alike, they share the deflection") {
		cp::spring_curve const draw{4.5e6, 0.1, 0.3};
		auto const split{cp::series(draw, draw, 0.1)};
		CHECK(split.first == doctest::Approx(0.05));
		CHECK(split.second == doctest::Approx(0.05));
		CHECK(split.force == doctest::Approx(draw.force(0.05)));
	}
	SUBCASE("the softer one gives more") {
		cp::spring_curve const soft{4.5e6, 0.1, 0.3};
		cp::spring_curve const stiff{9e6, 0.1, 0.3};
		auto const split{cp::series(soft, stiff, 0.12)};
		CHECK(split.first + split.second == doctest::Approx(0.12));
		CHECK(split.first > split.second);
		CHECK(soft.force(split.first) == doctest::Approx(split.force));
		CHECK(stiff.force(split.second) == doctest::Approx(split.force));
		// the same force whichever end asks, as both vehicles of the pair do
		CHECK(cp::series(stiff, soft, 0.12).force == split.force);
	}
	SUBCASE("past the travel of one, the other takes the rest") {
		cp::spring_curve const short_travel{1e6, 0.02, 0.0};
		cp::spring_curve const long_travel{1e6, 0.2, 0.0};
		auto const split{cp::series(short_travel, long_travel, 0.1)};
		CHECK(split.first == doctest::Approx(0.02));
		CHECK(split.second == doctest::Approx(0.08));
		CHECK(split.force == doctest::Approx(80e3));
		// both solid
		CHECK(cp::series(short_travel, long_travel, 0.3).force == doctest::Approx(200e3));
	}
}

TEST_CASE("coupler physics: the friction of a spring device takes energy on each stroke") {
	cp::spring_curve const draw{4.5e6, 0.1, 0.3};
	auto const unloading{0.5};
	auto const transition{3.0 * draw.end_force() / draw.travel};
	cp::hysteresis device;
	auto work{0.0};
	auto deflection{0.0};
	auto const move_to = [&device, &work, &deflection, &draw, unloading, transition](double Target) {
		while (std::abs(Target - deflection) > 1e-9) {
			auto const next{deflection + std::clamp(Target - deflection, -1e-4, 1e-4)};
			auto const before{device.force};
			auto const after{device.update(next, draw.force(next), unloading * draw.force(next), transition)};
			work += 0.5 * (before + after) * (next - deflection);
			deflection = next;
		}
	};

	move_to(0.08);
	// loading, along the loading curve
	CHECK(device.force == doctest::Approx(draw.force(0.08)));
	move_to(0.06);
	// unloading, on the unloading one
	CHECK(device.force == doctest::Approx(unloading * draw.force(0.06)));
	// loaded again, the friction holds the device stiff until it slips onto the loading curve
	auto const held{device.force};
	device.update(0.0601, draw.force(0.0601), unloading * draw.force(0.0601), transition);
	CHECK(device.force == doctest::Approx(held + transition * 0.0001));
	device.update(0.06, draw.force(0.06), unloading * draw.force(0.06), transition);
	move_to(0.0);
	CHECK(device.force == doctest::Approx(0.0));
	// the stroke took part of the energy of the loading curve, less than the unloading curve alone would have: the
	// device springs back stiffly at first, along the transition
	CHECK(work > 0.3 * draw.energy(0.08));
	CHECK(work < (1.0 - unloading) * draw.energy(0.08));
}

TEST_CASE("coupler physics: the link stretches for good past its yield force and breaks at its ultimate one") {
	cp::link_strength const link{600e3, 850e3, 250e3 / 0.02, true};
	CHECK(cp::plastic_elongation(link, 500e3, 0.0) == 0.0);
	CHECK(cp::plastic_elongation(link, 700e3, 0.0) == doctest::Approx(0.008));
	// it doesn't get shorter again
	CHECK(cp::plastic_elongation(link, 650e3, 0.008) == doctest::Approx(0.008));
	// an articulated joint holds whatever
	auto joint{link};
	joint.breakable = false;
	CHECK(cp::plastic_elongation(joint, 700e3, 0.0) == 0.0);

	// all the kinetic energy of the relative motion into the stop
	CHECK(cp::impact_force(1.0, 1e8, 2e4, 2e4) == doctest::Approx(1e6));
	CHECK(cp::impact_force(-1.0, 1e8, 2e4, 2e4) == 0.0);

	for (int index = 0; index < 2; ++index) {
		auto const scatter{cp::strength_scatter("wagon", index)};
		CHECK(scatter >= 1.0);
		CHECK(scatter <= 1.2);
		CHECK(cp::strength_scatter("wagon", index) == scatter);
	}
	CHECK(cp::strength_scatter("wagon", 0) != cp::strength_scatter("wagon", 1));
}

TEST_CASE("coupler physics: the coupling pulls only past its play, the buffers push") {
	auto const wagon{wagon_end()};
	// the play of both ends: 0.1 m
	CHECK(fresh_pair(0.05).force == 0.0);
	CHECK(fresh_pair(0.099).force == 0.0);

	auto const pulled{fresh_pair(0.15)};
	CHECK(pulled.force > 0.0);
	CHECK(pulled.tension == doctest::Approx(cp::series(wagon.draw_gear, wagon.draw_gear, 0.05).force));
	// uncoupled vehicles don't pull each other
	CHECK(fresh_pair(0.15, false).force == 0.0);

	auto const pushed{fresh_pair(-0.05)};
	CHECK(pushed.force == doctest::Approx(-cp::series(wagon.buffers, wagon.buffers, 0.05).force));
	CHECK(pushed.buffers_deflection == doctest::Approx(0.025));
	CHECK(fresh_pair(-0.05, false).force == pushed.force);

	SUBCASE("a tightened screw coupling presses the buffers in, both carry force at rest") {
		auto tight{wagon_end()};
		tight.slack = -0.02;
		auto const atrest{fresh_pair(-0.02, true, tight)};
		CHECK(atrest.tension > 0.0);
		CHECK(atrest.compression > 0.0);
	}
}

TEST_CASE("coupler physics: the springs of a pair go solid at the end of their travel") {
	cp::pair_state state;
	auto const wagon{wagon_end()};
	cp::update_pair(wagon, wagon, state, {0.1 + 0.2, 0.0, true, false, true});
	CHECK(state.stop == 1);
	CHECK(state.stop_force == doctest::Approx(450e3));
	cp::update_pair(wagon, wagon, state, {-0.25, 0.0, true, false, true});
	CHECK(state.stop == -1);
	CHECK(state.stop_force == doctest::Approx(1012e3));
	cp::update_pair(wagon, wagon, state, {0.0, 0.0, true, false, true});
	CHECK(state.stop == 0);
}

TEST_CASE("coupler physics: a pull past the ultimate force breaks the coupling, past the yield force stretches it") {
	auto weak{wagon_end()};
	weak.link = {300e3, 400e3, 100e3 / 0.02, true};

	SUBCASE("broken, with crash damage") {
		cp::pair_state state;
		auto const result{cp::update_pair(weak, weak, state, {0.1 + 0.19, 0.0, true, false, true})};
		CHECK(result.broken);
		CHECK(result.tension == 0.0);
		CHECK(result.force == 0.0);
	}
	SUBCASE("held, without it") {
		cp::pair_state state;
		auto const result{cp::update_pair(weak, weak, state, {0.1 + 0.19, 0.0, true, false, false})};
		CHECK_FALSE(result.broken);
		CHECK(result.tension > 400e3);
		CHECK(state.elongation == 0.0);
	}
	SUBCASE("stretched for good, which eases the pull") {
		auto stronger{weak};
		stronger.link.ultimate = 1000e3;
		cp::pair_state state;
		auto const first{cp::update_pair(stronger, stronger, state, {0.1 + 0.19, 0.0, true, false, true})};
		CHECK(first.overloaded);
		CHECK(state.elongation == doctest::Approx((first.tension - 300e3) / stronger.link.hardening));
		auto const second{cp::update_pair(stronger, stronger, state, {0.1 + 0.19, 0.0, true, false, true})};
		CHECK(second.tension < first.tension);
	}
}

TEST_CASE("coupler physics: a jerk on the solid draw gear breaks the coupling by the force of the hit") {
	auto const wagon{wagon_end()};
	constexpr double mass{42200.0};
	auto const stopped = [&wagon](cp::pair_state &State) { cp::update_pair(wagon, wagon, State, {0.1 + 0.21, 0.0, true, false, true}); };

	cp::pair_state state;
	stopped(state);
	auto const hard{cp::impact(wagon, wagon, state, 1.0, mass, mass)};
	// 450 kN of the springs and about 1.7 MN of the hit
	CHECK(hard.force == doctest::Approx(450e3 + std::sqrt(850e3 / 0.003 / 2.0 * mass / 2.0)));
	CHECK(hard.broken);

	cp::pair_state gentle;
	stopped(gentle);
	CHECK_FALSE(cp::impact(wagon, wagon, gentle, 0.1, mass, mass).broken);
	CHECK(gentle.elongation == 0.0);

	cp::pair_state harder;
	stopped(harder);
	auto const yielding{cp::impact(wagon, wagon, harder, 0.15, mass, mass)};
	CHECK_FALSE(yielding.broken);
	CHECK(yielding.force > 680e3);
	CHECK(harder.elongation > 0.0);

	// the buffers crushed by a hit past their travel
	cp::pair_state pressed;
	cp::update_pair(wagon, wagon, pressed, {-0.25, 0.0, false, false, true});
	CHECK_FALSE(cp::impact(wagon, wagon, pressed, 0.1, mass, mass).crushed);
	CHECK(cp::impact(wagon, wagon, pressed, 1.0, mass, mass).crushed);
}

// the model in the simulation, two vehicles on a track

namespace {

// couples the facing ends of the two vehicles, as the scenery does
void join(vehicle &First, int FirstEnd, vehicle &Second, int SecondEnd, int Coupling = coupling::coupler) {
	REQUIRE(First->Attach(FirstEnd, SecondEnd, &Second.mover(), Coupling, true, false));
	REQUIRE(Second->Attach(SecondEnd, FirstEnd, &First.mover(), Coupling, true, false));
}

// the wagon of data/wagon.json with the given play of its screw couplings
vehicle loose_wagon(double Slack, double Velocity, std::string const &Name) {
	return vehicle{definition::load("wagon").set("BuffCoupl", "Slack", Slack), Velocity, Name};
}

bool coupler_damaged(vehicle &Vehicle) {
	return TestFlag(Vehicle->DamageFlag, dtrain_coupling);
}

} // namespace

TEST_CASE("coupler physics: the play of the coupling and the parameters of the model come from the vehicle definition") {
	auto loose{loose_wagon(0.05, 0.0, "loose")};
	auto const &physics{loose->Couplers[end::front].physics};
	CHECK(physics.slack == doctest::Approx(0.05));
	CHECK(loose->Couplers[end::rear].physics.slack == doctest::Approx(0.05));
	CHECK(physics.buffers.stiffness == doctest::Approx(9200e3));
	CHECK(physics.buffers.travel == doctest::Approx(0.11));
	CHECK(physics.draw_gear.end_force() == doctest::Approx(450e3));
	CHECK(physics.buffers_unloading == doctest::Approx(0.58));
	// the rated breaking force, and a little more for the coupling as made
	CHECK(physics.link.ultimate >= 850e3);
	CHECK(physics.link.ultimate <= 1020e3);
	CHECK(physics.link.yield == doctest::Approx(0.8 * physics.link.ultimate));
	CHECK(physics.link.breakable);

	auto tight{vehicle{definition::load("wagon"), 0.0, "tight"}};
	CHECK(tight->Couplers[end::front].physics.slack == 0.0);
	auto automatic{vehicle{definition::load("wagon").set("BuffCoupl", "CType", "Automatic"), 0.0, "automatic"}};
	CHECK(automatic->Couplers[end::front].physics.slack == doctest::Approx(0.005));
	auto articulated{vehicle{definition::load("wagon").set("BuffCoupl", "CType", "Articulated"), 0.0, "articulated"}};
	CHECK_FALSE(articulated->Couplers[end::front].physics.link.breakable);
}

TEST_CASE("coupler physics: a coupling with play pulls only once the play is taken up") {
	flag_setting const physics{Global.coupler_physics, true};
	flag_setting const crashdamage{Global.crash_damage, true};
	auto front{loose_wagon(0.05, 3.0, "front")};
	auto rear{loose_wagon(0.05, 0.1, "rear")};
	join(rear, end::front, front, end::rear);
	test::pair pair{rear, end::front, front, end::rear, 0.0};
	front.hold_brake_pipe(5.0);
	rear.hold_brake_pipe(5.0);

	auto pulledwithinplay{false};
	for (int i = 0; i < 300 && pair.gap() < 0.09; ++i) {
		pair.step();
		pulledwithinplay = pulledwithinplay || rear->Couplers[end::front].CForce != 0.0;
	}
	REQUIRE(pair.gap() >= 0.09);
	CHECK_FALSE(pulledwithinplay);
	CHECK(rear.speed() == doctest::Approx(0.1).epsilon(0.5));

	pair.run(3.0);
	CHECK(pair.coupled());
	CHECK_FALSE(coupler_damaged(rear));
	CHECK_FALSE(coupler_damaged(front));
	// the front wagon took the rear one along
	CHECK(rear.speed() > 1.0);
	CHECK(front.speed() < 2.5);
}

TEST_CASE("coupler physics: buffers hit at a walking pace give back less than they took") {
	flag_setting const physics{Global.coupler_physics, true};
	flag_setting const crashdamage{Global.crash_damage, true};
	auto moving{vehicle{definition::load("wagon"), 5.0, "moving"}};
	// barely moving, so that its brakes are released
	auto standing{vehicle{definition::load("wagon"), 0.1, "standing"}};
	test::pair pair{moving, end::front, standing, end::rear, 0.3};
	moving.hold_brake_pipe(5.0);
	standing.hold_brake_pipe(5.0);

	auto touched{false};
	auto solid{false};
	for (int i = 0; i < 500; ++i) {
		pair.step();
		touched = touched || pair.gap() < 0.0;
		solid = solid || moving->Couplers[end::front].physics_state.stop != 0;
		if (touched && pair.gap() > 0.3) {
			break; // parted
		}
	}
	CAPTURE(moving.speed());
	CAPTURE(standing.speed());
	REQUIRE(touched);
	REQUIRE(pair.gap() > 0.3);
	// the buffers took the hit within their travel, nothing collided
	CHECK_FALSE(solid);
	CHECK_FALSE(pair.coupled());
	// the momentum stays, rolling resistance aside
	CHECK(moving.speed() + standing.speed() == doctest::Approx(5.1).epsilon(0.05));
	// the vehicles part slower than they met: the friction of the buffers took part of the energy
	auto const restitution{(standing.speed() - moving.speed()) / (5.0 - 0.1)};
	CHECK(restitution > 0.3);
	CHECK(restitution < 0.95);
}

TEST_CASE("coupler physics: coupled vehicles at rest stay at rest") {
	flag_setting const physics{Global.coupler_physics, true};
	auto first{vehicle{definition::load("wagon"), 0.0, "first"}};
	auto second{vehicle{definition::load("wagon"), 0.0, "second"}};
	join(first, end::rear, second, end::front);
	test::pair pair{first, end::rear, second, end::front, 0.0};

	pair.run(5.0);
	CHECK(std::abs(first.speed()) < 0.01);
	CHECK(std::abs(second.speed()) < 0.01);
	CHECK(std::abs(pair.gap()) < 0.001);
}

TEST_CASE("coupler physics: a jerk on a coupling with play breaks it, the brake hoses torn off") {
	flag_setting const physics{Global.coupler_physics, true};
	auto front{loose_wagon(0.05, 20.0, "front")};
	auto rear{loose_wagon(0.05, 0.1, "rear")};
	join(rear, end::front, front, end::rear, coupling::coupler | coupling::brakehose);
	test::pair pair{rear, end::front, front, end::rear, 0.0};
	front.hold_brake_pipe(5.0);
	rear.hold_brake_pipe(5.0);

	SUBCASE("with crash damage, it breaks") {
		flag_setting const crashdamage{Global.crash_damage, true};
		pair.run(1.0);
		CHECK_FALSE(pair.coupled());
		CHECK((coupler_damaged(rear) || coupler_damaged(front)));
		// both parts of the train brake, their pipe vented through the torn hoses
		CHECK(rear->AlarmChainFlag);
		CHECK(front->AlarmChainFlag);
		// the front wagon runs away from the rear one
		CHECK(front.speed() > rear.speed() + 5.0);
		CHECK(pair.gap() > 0.3);
	}
	SUBCASE("without it, the coupling holds and the wagons run on together") {
		flag_setting const crashdamage{Global.crash_damage, false};
		pair.run(3.0);
		CHECK(pair.coupled());
		CHECK_FALSE(coupler_damaged(rear));
		CHECK_FALSE(coupler_damaged(front));
		CHECK(front.speed() == doctest::Approx(rear.speed()).epsilon(0.2));
	}
}

TEST_CASE("coupler physics: a gentle pull on a coupling with play doesn't hurt it") {
	flag_setting const physics{Global.coupler_physics, true};
	flag_setting const crashdamage{Global.crash_damage, true};
	auto front{loose_wagon(0.05, 3.0, "front")};
	auto rear{loose_wagon(0.05, 0.1, "rear")};
	join(rear, end::front, front, end::rear);
	test::pair pair{rear, end::front, front, end::rear, 0.0};
	front.hold_brake_pipe(5.0);
	rear.hold_brake_pipe(5.0);

	pair.run(3.0);
	CHECK(pair.coupled());
	CHECK_FALSE(coupler_damaged(rear));
	CHECK(rear->Couplers[end::front].physics_state.elongation == 0.0);
}

TEST_CASE("coupler physics: automatic couplers still catch a wagon rolling into a standing one") {
	flag_setting const physics{Global.coupler_physics, true};
	flag_setting const crashdamage{Global.crash_damage, true};
	auto automatic{definition::load("wagon").set("BuffCoupl", "CType", "Automatic").set("BuffCoupl", "AutomaticFlag", coupling::coupler | coupling::brakehose)};
	auto moving{vehicle{automatic, 5.0, "moving"}};
	auto standing{vehicle{automatic, 0.0, "standing"}};
	test::pair pair{moving, end::front, standing, end::rear, 1.0};

	pair.run(3.0);
	REQUIRE(pair.coupled());
	pair.run(2.0);
	CHECK(moving.speed() == doctest::Approx(standing.speed()).epsilon(0.1));
	CHECK_FALSE(TestFlag(moving->DamageFlag, dtrain_out));
}

TEST_CASE("couplers: a broken coupling of vehicles facing each other uncouples the ends coupled") {
	// it used to uncouple the far end of the other vehicle, presuming the vehicles faced the same way
	auto first{vehicle{definition::load("wagon"), 0.0, "first"}};
	auto second{vehicle{definition::load("wagon"), 0.0, "second"}};
	auto third{vehicle{definition::load("wagon"), 0.0, "third"}};
	join(first, end::front, second, end::front);
	join(second, end::rear, third, end::front);

	first->damage_coupler(end::front);
	CHECK(first->Couplers[end::front].CouplingFlag == coupling::faux);
	CHECK(second->Couplers[end::front].CouplingFlag == coupling::faux);
	// the other coupling of the second vehicle stays
	CHECK(second->Couplers[end::rear].CouplingFlag == coupling::coupler);
}
