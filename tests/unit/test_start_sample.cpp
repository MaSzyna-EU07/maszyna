#include "stdafx.h"
#include "doctest.h"

#include "audio/startsample.h"

#include <limits>

// the sample a sound with a start offset (startoffset:, or the random one of motors, motor blowers and
// the running noise) begins to play from, in the 16 bit mono buffers the sound files are loaded into

TEST_CASE("start_sample: a fraction of the samples of the buffer, two bytes each") {
	CHECK(audio::start_sample(0.5f, 1000) == 250);
	CHECK(audio::start_sample(0.25f, 88200) == 11025);
	CHECK(audio::start_sample(0.999f, 1000) == 499);
}

TEST_CASE("start_sample: no offset starts at the first sample") {
	CHECK(audio::start_sample(0.f, 1000) == 0);
	CHECK(audio::start_sample(-0.5f, 1000) == 0);
	CHECK(audio::start_sample(std::numeric_limits<float>::quiet_NaN(), 1000) == 0);
}

TEST_CASE("start_sample: the end of the buffer is its last sample, not past it") {
	// OpenAL takes an offset at or past the end of the buffer for AL_INVALID_VALUE
	CHECK(audio::start_sample(1.f, 1000) == 499);
	CHECK(audio::start_sample(1.5f, 1000) == 499);
	CHECK(audio::start_sample(1.f, 2) == 0);
}

TEST_CASE("start_sample: a buffer of unknown or no size starts at 0") {
	CHECK(audio::start_sample(0.5f, 0) == 0);
	CHECK(audio::start_sample(0.5f, 1) == 0);
	CHECK(audio::start_sample(0.5f, -4) == 0);
}
