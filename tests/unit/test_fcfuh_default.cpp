#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
#include "utilities/utilities.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

using test::definition;
using test::vehicle;

// the figure of the voltage by the frequency of an induction motor drive in the dynamic brake (fcfuH) takes the
// one of the traction (fcfu) when the definition has none. the loader took it by writing fcfu out with
// std::to_string() and reading the text back as the default of fcfuH (eu07/maszyna#99); it assigns it now

namespace {

// a figure read from a line the way TMoverParameters::LoadFIZ_EngineInductionMotor() reads fcfu
double read_fcfu(std::string const &Text) {
	double fcfu{0.0};
	extract_value(fcfu, "fcfu", "fcfu=" + Text, "");
	return fcfu;
}

// fcfuH missing, the way the loader took it before: the figure written out with six decimals in the C locale
// (the game never sets another) and read back
double fcfuh_before(double const Fcfu) {
	double fcfuh{-1.0};
	extract_value(fcfuh, "fcfuH", "", std::to_string(Fcfu));
	return fcfuh;
}

// the bits of the value, which tell -0.0 from 0.0 too
std::uint64_t bits(double const Value) {
	std::uint64_t result;
	std::memcpy(&result, &Value, sizeof(result));
	return result;
}

// the value with all its digits, for the message
std::string exact(double const Value) {
	char text[64];
	std::snprintf(text, sizeof(text), "%.17g", Value);
	return text;
}

// a generator of its own (splitmix64), for the same numbers on every platform
struct numbers {
	std::uint64_t state{0x2545f4914f6cdd1dull};
	std::uint64_t next() {
		auto value{state += 0x9e3779b97f4a7c15ull};
		value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ull;
		value = (value ^ (value >> 27)) * 0x94d049bb133111ebull;
		return value ^ (value >> 31);
	}
};

// the decimal text of Integer / 10^Decimals, as a definition could hold it
std::string decimal(std::uint64_t const Integer, int const Decimals, bool const Negative) {
	auto digits{std::to_string(Integer)};
	if (Decimals > 0) {
		if (digits.size() <= static_cast<std::size_t>(Decimals)) {
			digits.insert(0, Decimals + 1 - digits.size(), '0');
		}
		digits.insert(digits.size() - Decimals, ".");
	}
	return (Negative ? "-" : "") + digits;
}

} // namespace

TEST_CASE("fcfuH: the figure of the traction is the same, assigned or written out and read back, for every figure of up to six decimals") {
	// any decimal figure of up to six decimals: the value read from it lies within half a millionth of it, so
	// std::to_string() writes the figure itself and the text reads back to the very same value. checked over the
	// figures the game's definitions use (33.5 to 81.5) and the ones around them, and over many others, of up
	// to six decimals and up to 18 digits, of either sign
	auto compared{0};
	auto const check{[&](std::string const &Text) {
		auto const fcfu{read_fcfu(Text)};
		auto const before{fcfuh_before(fcfu)};
		if (bits(before) != bits(fcfu)) {
			FAIL_CHECK("fcfu=" << Text << " was " << exact(before) << " as fcfuH, now " << exact(fcfu));
		}
		++compared;
	}};
	// every figure of up to two decimals, from -10 to 200
	for (auto hundredths{-1000}; hundredths <= 20000; ++hundredths) {
		check(decimal(static_cast<std::uint64_t>(std::abs(hundredths)), 2, hundredths < 0));
	}
	// every figure of up to six decimals around the ones of the game's definitions
	for (auto const figure : {33.5, 40.1, 43.7, 45.18, 46.8, 47.6, 48.9, 49.0, 81.5}) {
		auto const millionths{static_cast<std::int64_t>(figure * 1e6 + 0.5)};
		for (auto offset{-2000}; offset <= 2000; ++offset) {
			check(decimal(static_cast<std::uint64_t>(millionths + offset), 6, false));
		}
	}
	// figures of 0 to 6 decimals and 1 to 18 digits
	numbers random;
	for (auto decimals{0}; decimals <= 6; ++decimals) {
		for (auto digits{1}; digits <= 18; ++digits) {
			std::uint64_t limit{1};
			for (auto digit{0}; digit < digits; ++digit) {
				limit *= 10;
			}
			for (auto sample{0}; sample < 400; ++sample) {
				check(decimal(random.next() % limit, decimals, (sample & 1) != 0));
			}
		}
	}
	// the way of writing it doesn't matter, only the value
	for (auto const text : {"4.76e1", "47.60", "+47.6", "0047.6", "476e-1", "0", "-0", "1e15", "-1e-6", "1e300"}) {
		check(text);
	}
	// nor does a figure the stream can't read, which reads as 0
	for (auto const text : {"abc", "-", "x47.6"}) {
		check(text);
	}
	CHECK(compared > 70000);
}

TEST_CASE("fcfuH: where the two ways part, figures no definition has") {
	// no definition of the game has such a figure (they have one or two decimals, from 33.5 to 81.5). more than
	// six decimals: std::to_string() rounded the value to six, the figure is taken as it's written now
	CHECK(fcfuh_before(read_fcfu("0.1234567")) == read_fcfu("0.123457"));
	CHECK(fcfuh_before(read_fcfu("1e-7")) == 0.0);
	CHECK(read_fcfu("1e-7") == 1e-7);
	// a figure past the largest value reads as infinite (with MSVC; the largest value with GCC): written out it
	// was "inf", which the stream doesn't read, so the dynamic brake got 0 where it gets the traction's figure now
	auto const huge{read_fcfu("1e999")};
	CHECK(huge > 1e300);
	if (std::isinf(huge)) {
		CHECK(fcfuh_before(huge) == 0.0);
	}
}

TEST_CASE("fcfuH: the loader gives the dynamic brake the traction's figure when the definition has none") {
	for (auto const fcfu : {"43.7", "45.18", "49", "0.000001", "1e2"}) {
		CAPTURE(fcfu);
		// the EU47's definition has fcfu only
		vehicle eu47{definition::fiz("e186_v2/eu47").set("Engine", "fcfu", fcfu)};
		CHECK(eu47->eimc[eimc_f_cfu] == read_fcfu(fcfu));
		CHECK(bits(eu47->eimc[eimc_f_cfuH]) == bits(eu47->eimc[eimc_f_cfu]));
		CHECK(bits(eu47->eimc[eimc_f_cfuH]) == bits(fcfuh_before(read_fcfu(fcfu))));
		// its own figure is kept
		vehicle own{definition::fiz("e186_v2/eu47").set("Engine", "fcfu", fcfu).set("Engine", "fcfuH", "40.1")};
		CHECK(own->eimc[eimc_f_cfu] == read_fcfu(fcfu));
		CHECK(own->eimc[eimc_f_cfuH] == read_fcfu("40.1"));
	}
}
