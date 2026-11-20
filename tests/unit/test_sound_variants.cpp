#include "stdafx.h"
#include "doctest.h"

#include "audio/soundvariants.h"
#include "utilities/parser.h"

#include <cmath>
#include <string>

// the inside variant of a sound definition (MaSzyna-EU07/maszyna#61): where the listener is, which definition plays
// for it, how the two cross over when the camera moves, and how the variant's block is read from the definition

namespace {

using namespace sound_variants;

// the tokens of a text, joined with '|', as the sound definition reads its keys
std::string key_tokens(std::string const &Text) {
	cParser parser(Text, cParser::buffer_TEXT);
	std::string joined;
	for (auto token{parser.getToken<std::string>(true, "\n\r\t  ,;[]")}; false == token.empty(); token = parser.getToken<std::string>(true, "\n\r\t  ,;[]")) {
		if (false == joined.empty()) {
			joined += '|';
		}
		joined += token;
	}
	return joined;
}

// the block captured from the start of a text, and the token which follows it
struct captured {
	std::string block;
	std::string next;
};

captured capture(std::string const &Text) {
	cParser parser(Text, cParser::buffer_TEXT);
	captured result;
	result.block = capture_block(parser);
	result.next = parser.getToken<std::string>(true, "\n\r\t ,;");
	return result;
}

} // namespace

// where the listener is ----------------------------------------------------------------------------------------------

TEST_CASE("sound variants: the listener in a cab is in the cab it occupies") {
	CHECK(listener_location(false, false, true, -1) == rear_cab);
	CHECK(listener_location(false, false, true, 0) == engine_compartment);
	CHECK(listener_location(false, false, true, 1) == front_cab);
}

TEST_CASE("sound variants: looking out of the cab window puts the listener at the window, also in the external view") {
	CHECK(listener_location(true, false, true, 1) == cab_window);
	CHECK(listener_location(true, true, true, 1) == cab_window);
	CHECK(listener_location(true, true, false, 0) == cab_window);
}

TEST_CASE("sound variants: in the external view the listener is a camera, attached to a vehicle or free") {
	CHECK(listener_location(false, true, true, 1) == attached_camera);
	CHECK(listener_location(false, true, false, 1) == free_camera);
}

TEST_CASE("sound variants: a listener outside of the external view but without a vehicle is a free camera") {
	CHECK(listener_location(false, false, false, 1) == free_camera);
}

TEST_CASE("sound variants: the locations number the columns of the soundproofing tables") {
	// the tables list: rear cab, engine, front cab, window, attached camera, free camera; the column is the location + 1
	CHECK(rear_cab + 1 == 0);
	CHECK(engine_compartment + 1 == 1);
	CHECK(front_cab + 1 == 2);
	CHECK(cab_window + 1 == 3);
	CHECK(attached_camera + 1 == 4);
	CHECK(free_camera + 1 == 5);
}

TEST_CASE("sound variants: the cabs and the engine compartment enclose the listener, the window and the cameras don't") {
	CHECK(listener_enclosed(rear_cab));
	CHECK(listener_enclosed(engine_compartment));
	CHECK(listener_enclosed(front_cab));
	CHECK_FALSE(listener_enclosed(cab_window));
	CHECK_FALSE(listener_enclosed(attached_camera));
	CHECK_FALSE(listener_enclosed(free_camera));
}

// which definition plays ---------------------------------------------------------------------------------------------

TEST_CASE("sound variants: a sound without the inside variant plays its definition everywhere") {
	for (int location{rear_cab}; location <= free_camera; ++location) {
		CAPTURE(location);
		CHECK(select(false, location) == choice::main);
	}
}

TEST_CASE("sound variants: a sound with the inside variant plays it to an enclosed listener only") {
	CHECK(select(true, rear_cab) == choice::inside);
	CHECK(select(true, engine_compartment) == choice::inside);
	CHECK(select(true, front_cab) == choice::inside);
	CHECK(select(true, cab_window) == choice::main);
	CHECK(select(true, attached_camera) == choice::main);
	CHECK(select(true, free_camera) == choice::main);
}

// the crossfade ------------------------------------------------------------------------------------------------------

TEST_CASE("sound variants: the crossfade starts with the previous definition alone") {
	CHECK(crossfade_gain(true, 0.0, 0.3) == doctest::Approx(0.0));
	CHECK(crossfade_gain(false, 0.0, 0.3) == doctest::Approx(1.0));
}

TEST_CASE("sound variants: the crossfade ends exactly with the chosen definition alone") {
	CHECK(crossfade_gain(true, 0.3, 0.3) == 1.f);
	CHECK(crossfade_gain(false, 0.3, 0.3) == 0.f);
	CHECK(crossfade_gain(true, 10.0, 0.3) == 1.f);
	CHECK(crossfade_gain(false, 10.0, 0.3) == 0.f);
}

TEST_CASE("sound variants: before the crossfade starts the previous definition is alone") {
	CHECK(crossfade_gain(true, -1.0, 0.3) == doctest::Approx(0.0));
	CHECK(crossfade_gain(false, -1.0, 0.3) == doctest::Approx(1.0));
}

TEST_CASE("sound variants: without a crossfade time the chosen definition takes over at once") {
	CHECK(crossfade_gain(true, 0.0, 0.0) == 1.f);
	CHECK(crossfade_gain(false, 0.0, 0.0) == 0.f);
}

TEST_CASE("sound variants: the crossfade keeps the power, the chosen definition rising and the other falling") {
	auto previouschosen{-1.f};
	auto previousother{2.f};
	for (auto step{0}; step <= 30; ++step) {
		auto const elapsed{0.01 * step};
		CAPTURE(elapsed);
		auto const chosen{crossfade_gain(true, elapsed, 0.3)};
		auto const other{crossfade_gain(false, elapsed, 0.3)};
		CHECK(chosen * chosen + other * other == doctest::Approx(1.0).epsilon(1e-6));
		CHECK(chosen >= previouschosen);
		CHECK(other <= previousother);
		previouschosen = chosen;
		previousother = other;
	}
	// halfway both play at the same level, -3 dB
	CHECK(crossfade_gain(true, 0.15, 0.3) == doctest::Approx(crossfade_gain(false, 0.15, 0.3)));
	CHECK(crossfade_gain(true, 0.15, 0.3) == doctest::Approx(std::sqrt(0.5)));
}

// the block of the variant -------------------------------------------------------------------------------------------

TEST_CASE("sound variants: the key of the variant") {
	CHECK(inside_key == "inside:");
}

TEST_CASE("sound variants: the block of the variant is taken up to its end, and the definition goes on after it") {
	auto const result{capture("{ soundmain: engine_cab.wav } range: 50")};
	CHECK(result.block == "{ soundmain: engine_cab.wav }");
	CHECK(result.next == "range:");
}

TEST_CASE("sound variants: a block inside of the variant's block doesn't end it") {
	auto const result{capture("{ a: { b: c } d: e } f:")};
	CHECK(result.block == "{ a: { b: c } d: e }");
	CHECK(result.next == "f:");
}

TEST_CASE("sound variants: the variant's block gives the same tokens as its text in the definition") {
	std::string const text{
		"{\n"
		"    soundmain: [ engine_cab_a.wav, engine_cab_b.wav ]\n"
		"    soundproofing: [1.0,0.8;0.5 0.1 0.1 0.1]\n"
		"    // a comment\n"
		"    sound0: idle.wav sound50: half.wav sound100: full.wav\n"
		"    pitch50: 1.1 crossfade: 25\n"
		"}"};
	auto const result{capture(text + " next:")};
	CHECK(key_tokens(result.block) == key_tokens(text));
	CHECK(result.next == "next:");
}

TEST_CASE("sound variants: the variant's block keeps quoted names as they are") {
	auto const result{capture(R"({ soundmain: "Cab Engine.wav" soundend: Stop.WAV })")};
	// unquoted names come lower case, as the definition reads them; quoted ones keep their case and spaces
	CHECK(result.block == R"({ soundmain: "Cab Engine.wav" soundend: stop.wav })");
	cParser parser(result.block, cParser::buffer_TEXT);
	CHECK(parser.getToken<std::string>(true, "\n\r\t ,;") == "{");
	CHECK(parser.getToken<std::string>(true, "\n\r\t ,;") == "soundmain:");
	CHECK(parser.getToken<std::string>(true, "\n\r\t ,;") == "Cab Engine.wav");
}

TEST_CASE("sound variants: anything else than a block in place of the variant gives nothing") {
	auto const result{capture("engine_cab.wav range: 50")};
	CHECK(result.block.empty());
	CHECK(result.next == "range:");
}

TEST_CASE("sound variants: a block cut off by the end of the input is closed") {
	auto const result{capture("{ soundmain: engine_cab.wav")};
	CHECK(result.block == "{ soundmain: engine_cab.wav }");
	CHECK(result.next.empty());
}
