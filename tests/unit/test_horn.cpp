#include "stdafx.h"
#include "doctest.h"

#include "vehicle.h"

#include "vehicle/horn.h"

// the horn keys of the cab: low tone (A) and high tone (shift+A), held to sound

TEST_CASE("horn: a single lever for both tones sounds one tone at a time (eu07/maszyna#392)") {
	auto signal{0};
	signal = horn::pressed(signal, horn::low, true);
	REQUIRE(signal == horn::low);

	// the lever moved over to the high tone while the low tone key is still held
	signal = horn::pressed(signal, horn::high, true);
	CHECK(signal == horn::high);

	// letting go of the low tone key leaves the high tone sounding
	signal = horn::released(signal, horn::low);
	CHECK(signal == horn::high);

	signal = horn::released(signal, horn::high);
	CHECK(signal == 0);
}

TEST_CASE("horn: separate buttons for the tones can sound both at once") {
	auto signal{0};
	signal = horn::pressed(signal, horn::low, false);
	signal = horn::pressed(signal, horn::high, false);
	CHECK(signal == (horn::low | horn::high));

	signal = horn::released(signal, horn::low);
	CHECK(signal == horn::high);
}

TEST_CASE("horn: releasing a tone leaves the other bits of the signal alone") {
	auto const third{4}; // the third horn, operated separately
	CHECK(horn::released(horn::low | third, horn::low) == third);
	CHECK(horn::pressed(third, horn::high, true) == (horn::high | third));
}

// what the horns need to sound (eu07/maszyna#290): a pneumatic horn the air of the main reservoir, an electric one
// the low voltage. the horn lever stays where the driver put it, only the horn is silent
namespace {

test::vehicle horn_vehicle(test::definition Definition) {
	// a main reservoir of its own
	return test::vehicle{Definition.set("Brake", "Vv", 1.0).set("Brake", "MinCP", 6.5).set("Brake", "MaxCP", 8.0)};
}

int const allhorns{horn::low | horn::high | 4};

} // namespace

TEST_CASE("horn: a pneumatic horn sounds on the air of the main reservoir only") {
	auto vehicle{horn_vehicle(test::definition::load("wagon"))};
	REQUIRE(vehicle->HornSupply == horn_supply::air); // the default

	vehicle->Compressor = 6.5;
	CHECK(vehicle->SoundingHorns(allhorns) == allhorns);
	vehicle->Compressor = 0.0; // the reservoir is empty
	CHECK(vehicle->SoundingHorns(allhorns) == 0);
}

TEST_CASE("horn: a vehicle without a main reservoir takes the horn air from the main pipe") {
	test::vehicle vehicle{test::definition::load("wagon")};
	REQUIRE(vehicle->VeselVolume == 0.0);

	vehicle->ScndPipePress = 5.0;
	CHECK(vehicle->SoundingHorns(horn::low) == horn::low);
	vehicle->ScndPipePress = 0.0;
	CHECK(vehicle->SoundingHorns(horn::low) == 0);
}

TEST_CASE("horn: an electric horn sounds on the low voltage, whatever the air") {
	auto vehicle{horn_vehicle(test::definition::load("wagon").set("Cntrl", "HornType", "Electric"))};
	REQUIRE(vehicle->HornSupply == horn_supply::electric);
	REQUIRE(vehicle->Horn3Supply == horn_supply::electric); // the third horn follows the others unless told otherwise

	vehicle->Compressor = 0.0;
	vehicle->Power24vIsAvailable = true;
	CHECK(vehicle->SoundingHorns(allhorns) == allhorns);
	vehicle->Power24vIsAvailable = false;
	CHECK(vehicle->SoundingHorns(allhorns) == 0);
}

TEST_CASE("horn: the third horn can have a supply of its own, an electric horn of a diesel say") {
	auto vehicle{horn_vehicle(test::definition::load("wagon").set("Cntrl", "Horn3Type", "Electric"))};
	REQUIRE(vehicle->HornSupply == horn_supply::air);
	REQUIRE(vehicle->Horn3Supply == horn_supply::electric);

	vehicle->Compressor = 0.0;
	vehicle->Power24vIsAvailable = true;
	CHECK(vehicle->SoundingHorns(allhorns) == 4);
	vehicle->Compressor = 6.5;
	vehicle->Power24vIsAvailable = false;
	CHECK(vehicle->SoundingHorns(allhorns) == (horn::low | horn::high));
}

TEST_CASE("horn: a vehicle without any air, a road vehicle say, sounds its pneumatic horn as before") {
	// no main reservoir and no main pipe connection at either end
	test::vehicle vehicle{test::definition::load("wagon").set("BuffCoupl", "AllowedFlag", 3)};
	REQUIRE(vehicle->VeselVolume == 0.0);
	vehicle->ScndPipePress = 0.0;
	CHECK(vehicle->SoundingHorns(allhorns) == allhorns);
}

TEST_CASE("horn: a pneumatic horn sounds from the threshold pressure up") {
	auto vehicle{horn_vehicle(test::definition::load("wagon"))};
	vehicle->Compressor = TMoverParameters::HornMinPressure;
	CHECK(vehicle->SoundingHorns(horn::low) == horn::low);
	vehicle->Compressor = TMoverParameters::HornMinPressure - 0.01;
	CHECK(vehicle->SoundingHorns(horn::low) == 0);
}

TEST_CASE("horn: an electric horn sounds on 110 V too") {
	auto vehicle{horn_vehicle(test::definition::load("wagon").set("Cntrl", "HornType", "Electric"))};
	vehicle->Power110vIsAvailable = true;
	CHECK(vehicle->SoundingHorns(horn::high) == horn::high);
}
