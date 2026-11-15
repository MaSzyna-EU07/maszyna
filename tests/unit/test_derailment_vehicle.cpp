#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
#include "McZapkie/derailment.h"
#include "McZapkie/MOVER.h"
#include "utilities/Globals.h"

// a vehicle too fast in a curve with physics.derailment of eu07.ini: the criteria of McZapkie/derailment.h checked by
// TMoverParameters::check_derailment(), and what the vehicle comes to off the rails

using test::definition;
using test::vehicle;
namespace dr = derailment;

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

// the centres of gravity of EU07 (4E) and of the 430W coal wagon of data/fiz, without a CoGH of their own
double const eu07{dr::default_centre_of_gravity(4.348, dr::vehicle_class::powered)};
double const coal_wagon{dr::default_centre_of_gravity(3.235, dr::vehicle_class::unpowered)};

} // namespace

TEST_CASE("derailment: a vehicle too fast in a curve") {
	model_setting const physical{global_settings::derailment_model::physical};
	test::track curve;
	curve.radius = 300.0;
	auto const critical{dr::overturning_speed(300.0, 0.0, eu07, 1.435) / kmh};

	SUBCASE("below the overturning speed it runs on") {
		vehicle eu07vehicle{definition::fiz("4e_v1/4e"), critical - 5.0};
		for (int step = 0; step < 300; ++step) {
			eu07vehicle.step(curve);
		}
		CHECK_FALSE(TestFlag(eu07vehicle->DamageFlag, dtrain_out));
		CHECK(eu07vehicle->Derailed == dr::outcome::none);
	}
	SUBCASE("over it, it falls over and slides to a stop on its side") {
		vehicle eu07vehicle{definition::fiz("4e_v1/4e"), critical + 5.0};
		CHECK(eu07vehicle->centre_of_gravity() == doctest::Approx(eu07));
		// 2 m at 113 km/h is 0.07 s
		for (int step = 0; step < 10; ++step) {
			eu07vehicle.step(curve);
		}
		REQUIRE(TestFlag(eu07vehicle->DamageFlag, dtrain_out));
		CHECK(eu07vehicle->Derailed == dr::outcome::overturned);
		CHECK_FALSE(eu07vehicle->Mains);
		// not stopped dead as the legacy model has it: it slows down at half of g on the 80 of its 96 t
		auto const start{eu07vehicle->V};
		CHECK(start > 0.95 * (critical + 5.0) * kmh);
		for (int step = 0; step < 100; ++step) {
			eu07vehicle.step(curve);
		}
		auto const deceleration{(start - eu07vehicle->V) / 1.0};
		CHECK(deceleration == doctest::Approx(0.5 * dr::gravity * 80.0 / 96.0).epsilon(0.05));
		for (int step = 0; step < 2000; ++step) {
			eu07vehicle.step(curve);
		}
		CHECK(eu07vehicle->V == doctest::Approx(0.0));
	}
	SUBCASE("a higher centre of gravity of its definition overturns it slower") {
		vehicle high{definition::fiz("4e_v1/4e").set("Dimensions", "CoGH", 2.2), critical - 5.0};
		CHECK(high->centre_of_gravity() == doctest::Approx(2.2));
		for (int step = 0; step < 50; ++step) {
			high.step(curve);
		}
		CHECK(high->Derailed == dr::outcome::overturned);
	}
	SUBCASE("the same with steps of 0.01 and 0.05 s") {
		for (auto const dt : {0.01, 0.05}) {
			CAPTURE(dt);
			vehicle eu07vehicle{definition::fiz("4e_v1/4e"), critical + 2.0};
			for (auto time = 0.0; time < 1.0; time += dt) {
				eu07vehicle.step(curve, dt);
			}
			CHECK(eu07vehicle->Derailed == dr::outcome::overturned);
			vehicle slower{definition::fiz("4e_v1/4e"), critical - 2.0};
			for (auto time = 0.0; time < 1.0; time += dt) {
				slower.step(curve, dt);
			}
			CHECK(slower->Derailed == dr::outcome::none);
		}
	}
	SUBCASE("a load raises the centre of gravity") {
		vehicle wagon{definition::fiz("430w_v2/430w")};
		auto const empty{wagon->centre_of_gravity()};
		CHECK(empty == doctest::Approx(coal_wagon));
		wagon->AssignLoad("coal", 59.5f);
		wagon->ComputeMass();
		CHECK(wagon->centre_of_gravity() > empty + 0.5);
	}
}

TEST_CASE("derailment: the model of eu07.ini") {
	test::track curve;
	curve.radius = 300.0;
	SUBCASE("off: no derailment at any speed") {
		model_setting const off{global_settings::derailment_model::off};
		vehicle eu07vehicle{definition::fiz("4e_v1/4e"), 160.0};
		for (int step = 0; step < 300; ++step) {
			eu07vehicle.step(curve);
		}
		CHECK_FALSE(TestFlag(eu07vehicle->DamageFlag, dtrain_out));
	}
	SUBCASE("old: the legacy draw stops the vehicle dead") {
		model_setting const old{global_settings::derailment_model::old};
		vehicle eu07vehicle{definition::fiz("4e_v1/4e"), 160.0};
		for (int step = 0; step < 600; ++step) {
			eu07vehicle.step(curve);
		}
		// far over its threshold the draw comes out at once
		CHECK(TestFlag(eu07vehicle->DamageFlag, dtrain_out));
		CHECK(eu07vehicle->Derailed == dr::outcome::none);
		CHECK(eu07vehicle->V == doctest::Approx(0.0).epsilon(0.01));
	}
}

TEST_CASE("derailment: the brake hoses tear") {
	model_setting const physical{global_settings::derailment_model::physical};
	vehicle eu07vehicle{definition::fiz("4e_v1/4e"), 150.0};
	eu07vehicle->Couplers[end::rear].CouplingFlag = coupling::coupler | coupling::brakehose;
	test::track curve;
	curve.radius = 300.0;
	for (int step = 0; step < 20; ++step) {
		eu07vehicle.step(curve);
	}
	REQUIRE(eu07vehicle->Derailed == dr::outcome::overturned);
	CHECK(eu07vehicle->AlarmChainFlag);
}
