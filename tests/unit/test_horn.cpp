#include "stdafx.h"
#include "doctest.h"

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
