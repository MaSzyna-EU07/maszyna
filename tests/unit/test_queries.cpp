#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

// characterisation of what the game's panels read of a vehicle besides the gauges recorded in the traces: the
// description of its damage on the driver's panel (driveruipanels.cpp)

using test::definition;
using test::vehicle;

TEST_CASE("queries: the damage of a vehicle as the driver's panel describes it") {
	vehicle locomotive{definition::fiz("4e_v1/4e")};
	vehicle wagon{definition::fiz("3wl_v2/3wl")};
	// a powered locomotive and a wagon read some of the flags differently
	locomotive->Power = 2000.0;
	REQUIRE(wagon->Power < 0.1);

	struct damage {
		DamageSet flag;
		char const *locomotive;
		char const *wagon;
	};
	for (auto const &[flag, locomotivetext, wagontext] : {
	         damage{{}, "OK", "OK"},
	         damage{VehicleDamage::thinWheel, "Thin wheel", "Load shifted"},
	         damage{VehicleDamage::wheelWear, "Wheel wear", "Wheel wear"},
	         damage{VehicleDamage::bearing, "Bearing damaged", "Bearing damaged"},
	         damage{VehicleDamage::coupling, "Coupler broken", "Coupler broken"},
	         damage{VehicleDamage::loadDamage, "Ventilator damaged", "Load damaged"},
	         damage{VehicleDamage::loadDestroyed, "Engine damaged", "LOAD DESTROYED"},
	         damage{VehicleDamage::axle, "Axle broken", "Axle broken"},
	         damage{VehicleDamage::derailed, "DERAILED", "DERAILED"},
	         // the last flag of the list wins
	         damage{VehicleDamage::bearing | VehicleDamage::axle, "Axle broken", "Axle broken"},
	         damage{DamageSet::FromBits(255), "WRECKED", "WRECKED"},
	     }) {
		CAPTURE(flag);
		locomotive->DamageFlag = flag;
		wagon->DamageFlag = flag;
		CHECK(locomotive->EngineDescription(0) == locomotivetext);
		CHECK(wagon->EngineDescription(0) == wagontext);
	}
	// flat wheels count as worn ones
	wagon->DamageFlag = {};
	wagon->WheelFlat = 6.0;
	CHECK(wagon->EngineDescription(0) == "Wheel wear");
	// nothing else is described
	CHECK(wagon->EngineDescription(1) == "Invalid qualifier");
}

TEST_CASE("queries: the ammeters of a vehicle without power show the current of the ones it controls") {
	vehicle unpowered{definition::fiz("en57_v1/6ba_840")};
	vehicle powered{definition::fiz("en57_v1/5bgb")};
	unpowered->Attach(end::rear, end::front, &powered.mover(), Coupling::control, true, false);
	powered->Power = 600.0;
	powered->Itot = 321.7;
	unpowered->Power = 0.0;
	CHECK(unpowered->ShowCurrent(0) == doctest::Approx(321.0)); // whole amperes
	powered->Power = 0.0;
	CHECK(unpowered->ShowCurrent(0) == doctest::Approx(0.0));
}
