#include "stdafx.h"
#include "doctest.h"
#include "consist.h"
#include "scenario.h"

#include "utilities/utilities.h"

// characterisation of the couplers between vehicles: coupling and uncoupling by hand (Attach, DettachStatus,
// Dettach), the automatic couplers coupling on contact, a broken coupler, and collisions which derail, on real
// vehicles, the moving ones recorded in golden traces (data/golden/couplers/)

using test::consist;
using test::definition;
using test::scenario;
using namespace test::channels;

TEST_CASE("couplers: coupled by hand, touching or apart, of the same kind or not") {
	consist train;
	auto &locomotive{train.add(definition::fiz("4e_v1/4e")).mover()};
	// the wagon's buffers touch the locomotive's, the unit's car stands a metre away
	auto &wagon{train.add(definition::fiz("3wl_v2/3wl"), 0, coupling::faux).mover()};
	train.step();

	// nothing to couple to, or nothing to couple
	CHECK_FALSE(locomotive.Attach(end::rear, end::front, nullptr, coupling::coupler));
	CHECK_FALSE(locomotive.Attach(end::rear, end::front, &wagon, coupling::faux));
	// the hook first: the coupler alone, which sounds
	locomotive.Couplers[end::rear].sounds = 0;
	REQUIRE(locomotive.Attach(end::rear, end::front, &wagon, coupling::coupler));
	CHECK(locomotive.Couplers[end::rear].Connected == &wagon);
	CHECK(wagon.Couplers[end::front].Connected == &locomotive);
	CHECK(wagon.Couplers[end::front].CouplingFlag == coupling::coupler);
	CHECK(locomotive.Couplers[end::rear].sounds == sound::attachcoupler);
	// then the hoses: only what changes sounds
	locomotive.Couplers[end::rear].sounds = 0;
	REQUIRE(locomotive.Attach(end::rear, end::front, &wagon, coupling::coupler | coupling::brakehose | coupling::mainhose));
	CHECK(locomotive.Couplers[end::rear].sounds == (sound::attachbrakehose | sound::attachmainhose));
	// and again, quietly, the other side naming the coupler by the one it's connected to (2)
	locomotive.Couplers[end::rear].sounds = 0;
	REQUIRE(wagon.Attach(end::front, 2, &locomotive, coupling::coupler | coupling::brakehose, false, false));
	CHECK(locomotive.Couplers[end::rear].CouplingFlag == (coupling::coupler | coupling::brakehose));
	CHECK(locomotive.Couplers[end::rear].sounds == 0);

	// a screw coupler with the buffers pressed together comes apart whole
	train.step();
	CHECK(locomotive.DettachStatus(end::rear) < 0);
	locomotive.Couplers[end::rear].sounds = 0;
	CHECK(locomotive.Dettach(end::rear));
	CHECK(locomotive.Couplers[end::rear].Connected == nullptr);
	CHECK(wagon.Couplers[end::front].Connected == nullptr);
	CHECK(locomotive.Couplers[end::rear].sounds == (sound::detach | sound::attachcoupler | sound::attachbrakehose));
	CHECK(locomotive.DettachStatus(end::rear) == 0);
	CHECK(locomotive.Dettach(end::rear));
}

TEST_CASE("couplers: a screw coupler uncoupled with the buffers apart, or with the hoses alone, or broken") {
	struct state {
		char const *name;
		double gap; // between the buffers, the coupler stretched
		int coupling;
		bool damaged;
		bool apart; // whether it comes apart whole
	};
	for (auto const &[name, gap, couplingtype, damaged, apart] : {
	         state{"stretched a little", 0.1, coupling::coupler | coupling::brakehose, false, false},
	         state{"stretched far", 0.3, coupling::coupler | coupling::brakehose, false, true},
	         state{"hoses alone", 0.1, coupling::brakehose, false, true},
	         state{"broken", 0.1, coupling::coupler | coupling::brakehose, true, true},
	     }) {
		CAPTURE(name);
		consist train;
		auto &locomotive{train.add(definition::fiz("4e_v1/4e")).mover()};
		auto &wagon{train.add(definition::fiz("3wl_v2/3wl"), 0, coupling::faux, 0.0, gap).mover()};
		REQUIRE(locomotive.Attach(end::rear, end::front, &wagon, couplingtype, true));
		if (damaged) {
			SetFlag(locomotive.DamageFlag, dtrain_coupling);
		}
		train.step();
		CHECK((locomotive.DettachStatus(end::rear) < 0) == apart);
		CHECK(locomotive.Dettach(end::rear) == apart);
		// what's left when it doesn't come apart is the coupler, the hoses taken off
		CHECK(locomotive.Couplers[end::rear].CouplingFlag == (apart ? coupling::faux : coupling::coupler));
		CHECK(wagon.Couplers[end::front].CouplingFlag == (apart ? coupling::faux : coupling::coupler));
	}
	// couplers of different kinds don't couple, unless forced
	consist train;
	auto &locomotive{train.add(definition::fiz("4e_v1/4e")).mover()};
	auto &unit{train.add(definition::fiz("en57_v1/5bgb"), 0, coupling::faux).mover()};
	train.step();
	CHECK_FALSE(locomotive.Attach(end::rear, end::front, &unit, coupling::coupler));
	CHECK(locomotive.Attach(end::rear, end::front, &unit, coupling::coupler, true));
}

TEST_CASE("couplers: automatic couplers coupling on contact") {
	// a car running slowly into another from close by (its brakes are on), the automatic couplers of both ends
	// set to couple what their AutomaticFlag says; without one, the hook alone
	struct meeting {
		char const *name;
		char const *standing; // its rear coupler met
		char const *running; // its front coupler meets
		int coupling; // what they couple: what both flags allow, the control cable only of one kind
	};
	for (auto const &[name, standing, running, couplingtype] : {
	         meeting{"en57 to en57", "en57_v1/5bgb", "en57_v1/5bgaiii_led_spot", 39},
	         meeting{"impuls to en57", "en57_v1/5bgb", "impuls_v1/31we-a_pr", 39 & ~coupling::control},
	         meeting{"en57 without the flag", "en57_v1/5bgaiii_led_spot", "en57_v1/6ba_840", coupling::coupler},
	     }) {
		CAPTURE(name);
		scenario scenario{std::string{"couplers/automatic/"} + name, motion(), test::track{}};
		scenario.add(standing, 0, consist::full_coupling);
		scenario.add(running, 0, coupling::faux, 3.0, 0.2);
		scenario.wait(10.0);
		auto &first{scenario.vehicle(0)};
		auto &second{scenario.vehicle(1)};
		CHECK(first.Couplers[end::rear].CouplingFlag == couplingtype);
		CHECK(second.Couplers[end::front].CouplingFlag == couplingtype);
		// once coupled, not again until uncoupled; the vehicle which coupled marks it for the AI
		CHECK_FALSE(first.Couplers[end::rear].AutomaticCouplingAllowed);
		CHECK_FALSE(second.Couplers[end::front].AutomaticCouplingAllowed);
		CHECK((TestFlag(first.AIFlag, sound::attachcoupler) || TestFlag(second.AIFlag, sound::attachcoupler)));
		scenario.check();
	}
}

TEST_CASE("couplers: a coupler broken, with its brake hose torn off") {
	consist train;
	auto &locomotive{train.add(definition::fiz("4e_v1/4e")).mover()};
	auto &wagon{train.add(definition::fiz("3wl_v2/3wl")).mover()};
	REQUIRE(locomotive.Couplers[end::rear].Connected == &wagon);
	locomotive.damage_coupler(end::rear);
	CHECK(TestFlag(locomotive.DamageFlag, dtrain_coupling));
	CHECK(locomotive.AlarmChainFlag);
	CHECK(locomotive.Couplers[end::rear].CouplingFlag == coupling::faux);
	CHECK(wagon.Couplers[end::front].CouplingFlag == coupling::faux);
	// the front coupler of the wagon, broken on its own: no hose to tear
	wagon.damage_coupler(end::front);
	CHECK(TestFlag(wagon.DamageFlag, dtrain_coupling));
	CHECK_FALSE(wagon.AlarmChainFlag);
	// articulated couplers don't break
	consist articulated;
	auto &front{articulated.add(definition::fiz("impuls_v1/31we-a_pr")).mover()};
	articulated.add(definition::fiz("impuls_v1/31we-b_pr"));
	front.damage_coupler(end::rear);
	CHECK_FALSE(TestFlag(front.DamageFlag, dtrain_coupling));
	CHECK(front.Couplers[end::rear].CouplingFlag != coupling::faux);
}

TEST_CASE("couplers: collisions fast enough to derail") {
	// a locomotive running into a standing wagon from behind, and a wagon running backwards into a standing
	// locomotive; over 15 km/h, weighed by the ratio of the masses, the vehicles derail
	struct collision {
		char const *name;
		char const *front;
		double frontspeed;
		char const *back;
		double backspeed;
		bool frontout;
		bool backout;
	};
	for (auto const &[name, front, frontspeed, back, backspeed, frontout, backout] : {
	         collision{"locomotive into a wagon at 60", "3wl_v2/3wl", 0.0, "4e_v1/4e", 60.0, true, false},
	         collision{"wagon backwards into a locomotive at 60", "3wl_v2/3wl", -60.0, "4e_v1/4e", 0.0, true, false},
	         collision{"locomotive into a locomotive at 25", "4e_v1/4e", 0.0, "4e_v1/4e", 25.0, true, true},
	     }) {
		CAPTURE(name);
		scenario scenario{std::string{"couplers/collision/"} + name, motion(), test::track{}};
		scenario.add(front, 0, consist::full_coupling, frontspeed);
		scenario.add(back, 0, coupling::faux, backspeed, 20.0);
		scenario.wait(10.0);
		CHECK(TestFlag(scenario.vehicle(0).DamageFlag, dtrain_out) == frontout);
		CHECK(TestFlag(scenario.vehicle(1).DamageFlag, dtrain_out) == backout);
		scenario.check();
	}
}
